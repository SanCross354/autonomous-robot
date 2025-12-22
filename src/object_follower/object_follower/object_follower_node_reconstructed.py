#!/usr/bin/env python3
"""
object_follower_auto_waypoints_improved.py
Improved follower: minimal changes from your earlier node, with:
 - wait-for-critical-data (map, amcl_pose, nav2)
 - QoS transient_local for /map
 - stable detection frames
 - choose largest bbox when multiple detections
 - dynamic single-waypoint sampling + retry/skip
 - local fallback reached check (NEAR_GOAL_EPS)
 - MarkerArray publishing for RViz
 - optional return_home_after_success
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped, Point
from std_msgs.msg import String, Header, ColorRGBA
from yolov8_msgs.msg import Yolov8Inference  # your custom inference msg list
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import tf_transformations
import tf2_ros
import math
import time
import csv
import os
import random

# ----------------- Tunables (param defaults, also loadable as ros2 params) -----------------
FOLLOW_DISTANCE = 1.0            # meters
OBJECT_LOST_TIMEOUT = 2.5        # s before declaring object lost
ROTATE_TIMEOUT = 6.0             # s rotate before forward step (SEARCHING)
SEARCH_FORWARD_TIME = 1.0        # s forward in SEARCHING
SEARCH_FORWARD_SPEED = 0.18      # m/s forward in SEARCHING
ROTATION_SPEED = 0.35            # rad/s rotate speed in SEARCHING / scanning
DETECTION_STOP_THRESHOLD = 0.6   # bbox height ratio => vision stop (same as before)
MIN_GOAL_UPDATE = 0.15           # m change needed to refresh goal
MIN_GOAL_PERIOD = 0.8            # s min period between goal sends
NAV_RESULT_COOLDOWN = 0.8        # s after a NAV2 result before we can send again
NEAR_GOAL_EPS = 0.25             # m fallback distance to treat as reached
SECTOR_HALF_WIDTH = 5            # laser beams around center to check for obstacle
SCAN_ROTATIONS = 1.0
SCAN_DURATION = (2.0 * math.pi * SCAN_ROTATIONS) / max(1e-6, abs(ROTATION_SPEED))
SEARCH_MAX_DURATION = 18.0
WAYPOINT_MARGIN = 0.6
SNAP_SEARCH_RADIUS_M = 0.8
METRICS_CSV = "/tmp/object_follower_metrics.csv"
USE_EXPLORATION = True

# Centering servo parameters (pixels)
CAM_WIDTH_DEFAULT = 640
CAM_HEIGHT_DEFAULT = 480
CENTER_TOL_PX = 40  # if bbox center within +/- this px of image center -> considered centered
CENTER_ANGULAR_SPEED = 0.45
CENTER_SPIN_MAX = 3.0

# Stable detection
STABLE_DETECTION_FRAMES = 2

# Waypoint sampling / retry
WAYPOINT_RETRY_MAX = 2

# ----------------- States -----------------
IDLE = "IDLE"
EXPLORING = "EXPLORING"
SEARCHING = "SEARCHING"
TRACKING = "TRACKING"
STOPPED = "STOPPED"

class ObjectFollower(Node):
    def __init__(self):
        super().__init__('object_follower_auto_waypoints_improved')

        # --- State ---
        self.state = IDLE
        self.selected_object = None
        self.last_detection_time = None
        self.current_detection = None
        self.detection_image_w = CAM_WIDTH_DEFAULT
        self.detection_image_h = CAM_HEIGHT_DEFAULT
        self.latest_laser = None

        # Nav2 action
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav2_ready = False
        self.goal_handle = None
        self.goal_sent = False
        self.last_nav_goal = None   # (x, y, yaw)
        self.last_goal_time = 0.0
        self.last_nav_result_time = 0.0
        self.nav_goals_sent_count = 0
        self.nav_goals_canceled_count = 0

        # Searching bookkeeping
        self.last_rotate_time = None
        self.search_forward_start = None
        self.search_forward_active = False
        self.search_start_time = None

        # Map / waypoints
        self.map_received = False
        self.map = None  # nav_msgs/OccupancyGrid
        self.explore_waypoints = []   # list of (x,y,yaw)
        self.current_wp = None  # current target waypoint (x,y,yaw)
        self.current_wp_retries = 0

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # AMCL pose
        self.amcl_pose = None

        # Metrics
        self.selection_counter = 0
        self.selection_start_time = None
        self.first_detection_time = None
        self.reached_time = None

        # Stable detection counters
        self.stable_count = 0
        self.last_detected_bbox = None

        # Return home
        self.home_pose = None
        self.return_home_after_success = False

        # Marker publisher
        self.waypoint_marker_pub = self.create_publisher(MarkerArray, '/object_follower/waypoints', 1)
        self.status_pub = self.create_publisher(String, '/object_follower_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # QoS for map (transient_local)
        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # ROS I/O
        self.create_subscription(Yolov8Inference, '/Yolov8_Inference', self.detection_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.create_subscription(String, '/selected_object_class', self.selector_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_map)
        # AMCL pose subscription (to get current robot global pose)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)

        # Timer
        self.timer = self.create_timer(0.2, self.timer_callback)

        # ensure metrics CSV exists with header
        if not os.path.exists(METRICS_CSV):
            with open(METRICS_CSV, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "selection_id", "selected_class",
                    "selection_time", "first_detection_time", "detection_latency_s",
                    "reach_time", "time_to_reach_s",
                    "nav_goals_sent", "nav_goals_canceled",
                    "result"
                ])

        self.get_logger().info("Object follower node (reconstructed) started")

        # load ros parameters (if provided via ros2 param)
        self.declare_parameter('return_home_after_success', False)
        self.return_home_after_success = self.get_parameter('return_home_after_success').get_parameter_value().bool_value

    # ---------------- map -> generate waypoints ----------------
    def map_callback(self, msg: OccupancyGrid):
        if self.map_received:
            # ignore subsequent updates in this simplified logic
            return
        self.get_logger().info("Map received; computing auto waypoints.")
        self.map = msg
        self.map_received = True
        self.explore_waypoints = self.generate_waypoints_from_map(msg)
        self.get_logger().info(f"Auto waypoints set from map ({len(self.explore_waypoints)} pts).")
        for i,w in enumerate(self.explore_waypoints):
            self.get_logger().info(f"  {i+1}: {w}")
    
    # ---------------- detection (stable + largest bbox) ----------------
    def detection_callback(self, msg: Yolov8Inference):
        # Only consider when a selection exists
        if not self.selected_object:
            return

        # store image dims if provided
        self.detection_image_w = getattr(msg, 'image_width', self.detection_image_w) or self.detection_image_w
        self.detection_image_h = getattr(msg, 'image_height', self.detection_image_h) or self.detection_image_h

        found_any = False
        # Find all matched-class dets and pick largest bbox (area)
        best_det = None
        best_area = 0.0
        for det in msg.yolov8_inference:
            if det.class_name == self.selected_object:
                found_any = True
                bbox_w = max(0.0, float(det.right - det.left))
                bbox_h = max(0.0, float(det.bottom - det.top))
                area = bbox_w * bbox_h
                if area > best_area:
                    best_area = area
                    best_det = det

        if found_any and best_det:
            # stable detection logic
            # if same bbox center roughly as before -> increment, else reset
            cx = 0.5 * (float(best_det.left) + float(best_det.right))
            cy = 0.5 * (float(best_det.top) + float(best_det.bottom))
            if self.last_detected_bbox is None:
                self.stable_count = 1
                self.last_detected_bbox = (cx, cy, best_area)
            else:
                last_cx, last_cy, last_area = self.last_detected_bbox
                # modest tolerance to treat as same detection
                if abs(cx - last_cx) < 30 and abs(cy - last_cy) < 30:
                    self.stable_count += 1
                else:
                    self.stable_count = 1
                    self.last_detected_bbox = (cx, cy, best_area)

            # update current_detection only when stable_count >= threshold
            if self.stable_count >= STABLE_DETECTION_FRAMES:
                self.current_detection = best_det
                self.last_detection_time = time.time()
                # compute ratio
                image_height = self.detection_image_h or 640
                bbox_height = max(0.0, float(best_det.bottom - best_det.top))
                ratio = (bbox_height / float(image_height)) if image_height > 0 else 0.0
                if self.first_detection_time is None:
                    self.first_detection_time = time.time()
                # if too close by vision -> STOPPED
                if ratio >= DETECTION_STOP_THRESHOLD:
                    self.state = STOPPED
                    self.publish_status("Object close enough (vision). Stopping.")
                    self.cancel_current_goal()
                    self.stop_motion()
                    self.record_metrics(success=True)
                    return

                # If we were exploring / searching / stopped, start TRACKING
                if self.state in (EXPLORING, SEARCHING, STOPPED):
                    self.scanning_start = None
                    self.cancel_current_goal()
                    self.publish_twist(0.0, 0.0)
                    self.state = TRACKING
                    self.search_start_time = None
                    self.publish_status("Object detected (stable)! Switching to TRACKING.")
                self.get_logger().info(f"Detected {best_det.class_name} bbox (({best_det.left},{best_det.top}) -> ({best_det.right},{best_det.bottom})) stable_count={self.stable_count}")
                return

        else:    
            # not found
            self.current_detection = None
            self.stable_count = 0
            self.last_detected_bbox = None
    

    def generate_waypoints_from_map(self, grid: OccupancyGrid):
        # same logic as previous: center + 4 corners (snapped)
        info = grid.info
        w = int(info.width)
        h = int(info.height)
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        xmin = ox
        ymin = oy
        xmax = ox + w * res
        ymax = oy + h * res

        candidates = []
        cx = (xmin + xmax) / 2.0
        cy = (ymin + ymax) / 2.0
        candidates.append((cx, cy, 0.0))
        candidates.append((xmin + WAYPOINT_MARGIN, ymin + WAYPOINT_MARGIN, 0.0))
        candidates.append((xmax - WAYPOINT_MARGIN, ymin + WAYPOINT_MARGIN, 0.0))
        candidates.append((xmax - WAYPOINT_MARGIN, ymax - WAYPOINT_MARGIN, 0.0))
        candidates.append((xmin + WAYPOINT_MARGIN, ymax - WAYPOINT_MARGIN, 0.0))

        snapped = []
        for (x, y, yaw) in candidates:
            s = self.snap_to_free_cell(grid, x, y, SNAP_SEARCH_RADIUS_M)
            if s is not None:
                snapped.append((s[0], s[1], yaw))
            else:
                self.get_logger().warn(f"Could not snap waypoint near ({x:.2f},{y:.2f}), skipping")
        return snapped

    def snap_to_free_cell(self, grid: OccupancyGrid, x_target, y_target, radius_m):
        info = grid.info
        w = int(info.width)
        h = int(info.height)
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        i_center = int((x_target - ox) / res)
        j_center = int((y_target - oy) / res)
        search_radius_cells = max(1, int(radius_m / res))

        for r in range(search_radius_cells + 1):
            for di in range(-r, r + 1):
                dj = r - abs(di)
                for sign in (1, -1) if dj != 0 else (1,):
                    jj = j_center + sign * dj
                    ii = i_center + di
                    if ii < 0 or ii >= w or jj < 0 or jj >= h:
                        continue
                    idx = jj * w + ii
                    val = grid.data[idx]
                    if val == 0:
                        x_world = ox + (ii + 0.5) * res
                        y_world = oy + (jj + 0.5) * res
                        return (x_world, y_world)
        return None

    def sample_random_waypoint_from_map(self):
        # Sample random world coords within map bounds and snap to free cell
        if not self.map:
            return None
        info = self.map.info
        w = int(info.width)
        h = int(info.height)
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        xmin = ox
        ymin = oy
        xmax = ox + w * res
        ymax = oy + h * res

        for _ in range(40):
            xr = random.uniform(xmin + WAYPOINT_MARGIN, xmax - WAYPOINT_MARGIN)
            yr = random.uniform(ymin + WAYPOINT_MARGIN, ymax - WAYPOINT_MARGIN)
            s = self.snap_to_free_cell(self.map, xr, yr, SNAP_SEARCH_RADIUS_M)
            if s:
                return (s[0], s[1], 0.0)
        return None

    # ---------------- selector / GUI ----------------
    def selector_callback(self, msg: String):
        sel = msg.data.strip() if msg.data else None
        if sel:
            self.selection_counter += 1
            self.selection_start_time = time.time()
            self.first_detection_time = None
            self.reached_time = None
            self.nav_goals_sent_count = 0
            self.nav_goals_canceled_count = 0

            self.selected_object = sel
            # store home pose once (first selection), if not set
            if self.home_pose is None and self.amcl_pose:
                self.home_pose = (self.amcl_pose.pose.pose.position.x,
                                  self.amcl_pose.pose.pose.position.y,
                                  self._yaw_from_quat(self.amcl_pose.pose.pose.orientation))
            # Decide initial state: prefer EXPLORING only when map+amcl available
            if USE_EXPLORATION and self.map_received and self.amcl_pose:
                self.state = EXPLORING
                self.current_wp = None
                self.current_wp_retries = 0
                self.publish_status(f"[{self.state}] Selected '{self.selected_object}'. Start exploring waypoints...")
                # immediately plan first waypoint
                self.goto_next_waypoint()
            else:
                # fallback to SEARCHING (rotate+forward)
                self.state = SEARCHING
                self.last_rotate_time = time.time()
                self.search_start_time = self.last_rotate_time
                self.publish_status(f"[{self.state}] Selected '{self.selected_object}'. Start searching...")
        else:
            self.publish_status("Stop requested by GUI")
            self.selected_object = None
            self.state = IDLE
            self.cancel_current_goal()
            self.stop_motion()

    def is_path_clear(self):
        if not self.latest_laser or not self.latest_laser.ranges:
            return True
        ranges = self.latest_laser.ranges
        n = len(ranges)
        c = n // 2
        left = max(0, c - SECTOR_HALF_WIDTH)
        right = min(n, c + SECTOR_HALF_WIDTH + 1)
        for r in ranges[left:right]:
            if r and not math.isinf(r) and not math.isnan(r):
                if r <= FOLLOW_DISTANCE:
                    return False
        return True

    # ---------------- Nav2 helpers ----------------
    def should_send_goal(self, x, y, yaw):
        now = time.time()
        if (now - self.last_goal_time) < MIN_GOAL_PERIOD:
            return False
        if (now - self.last_nav_result_time) < NAV_RESULT_COOLDOWN:
            return False
        if not self.goal_sent or not self.last_nav_goal:
            return True
        dx = x - self.last_nav_goal[0]
        dy = y - self.last_nav_goal[1]
        return math.hypot(dx, dy) > MIN_GOAL_UPDATE

    def send_nav_goal(self, x, y, yaw=0.0):
        if not self.nav2_ready:
            self.get_logger().warn("Nav2 not ready; skipping send_nav_goal")
            return False
        if not self.should_send_goal(x, y, yaw):
            return False
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation.x = float(q[0])
        goal.pose.orientation.y = float(q[1])
        goal.pose.orientation.z = float(q[2])
        goal.pose.orientation.w = float(q[3])

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal

        try:
            if not self.nav_action_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warning("Nav2 action server not available")
                return False
            send_future = self.nav_action_client.send_goal_async(nav_goal)
            send_future.add_done_callback(self.goal_response_callback)
            self.last_nav_goal = (x, y, yaw)
            self.last_goal_time = time.time()
            self.nav_goals_sent_count += 1
            self.get_logger().info(f"Nav goal send attempt (count={self.nav_goals_sent_count}) -> ({x:.2f},{y:.2f})")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to send nav goal: {e}")
            return False

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            self.goal_handle = goal_handle
            if not goal_handle.accepted:
                self.get_logger().info("Goal rejected by server")
                self.publish_status("Goal rejected")
                self.goal_sent = False
                self.last_nav_goal = None
                return
            self.goal_sent = True
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.goal_result_callback)
            self.publish_status("Nav goal accepted")
        except Exception as e:
            self.get_logger().error(f"goal_response_callback exception: {e}")
            self.goal_sent = False

    def goal_result_callback(self, future):
        try:
            res = future.result()
            status = None
            try:
                status = res.status
            except Exception:
                try:
                    status = res.result.status
                except Exception:
                    status = None

            self.last_nav_result_time = time.time()
            self.get_logger().info(f"Nav2 goal finished (callback), status={status}")

            # If succeeded and current waypoint was exploration waypoint: trigger scanning
            if status == GoalStatus.STATUS_SUCCEEDED:
                if self.state == EXPLORING and self.current_wp is not None:
                    # arrived at waypoint -> start scanning
                    self.stop_motion()
                    self.start_scanning()
                elif self.state == TRACKING:
                    # could be reached target (but tracking uses lidar stop too)
                    self.publish_status("Navigation succeeded (tracking)")
            else:
                # not succeeded -> record & handle retry logic if exploring
                self.get_logger().warning(f"Nav2 goal ended with status={status}")
                self.publish_status("Navigation ended (non-success result)")
        except Exception as e:
            self.get_logger().warning(f"Nav2 goal result callback error: {e}")
        finally:
            self.goal_sent = False
            self.goal_handle = None
            self.last_nav_goal = None

    def cancel_current_goal(self):
        if self.goal_handle:
            try:
                cancel_future = self.goal_handle.cancel_goal_async()
                def _done(fut):
                    try:
                        _res = fut.result()
                        self.get_logger().info("Cancel request processed")
                    except Exception as e:
                        self.get_logger().warning(f"Cancel request error: {e}")
                cancel_future.add_done_callback(_done)
                self.nav_goals_canceled_count += 1
            except Exception as e:
                self.get_logger().warning(f"Failed to cancel goal: {e}")
        self.goal_sent = False
        self.goal_handle = None
        self.last_nav_goal = None

    # ---------------- Scanning / Waypoints ----------------
    def goto_next_waypoint(self):
        # dynamic single-step: choose next waypoint either from precomputed or sample random
        if not self.map_received:
            self.publish_status("No map yet; cannot goto waypoint")
            return

        # pick next: if explore_waypoints available (precomputed), use sequentially
        if self.explore_waypoints:
            # if current_wp is None, pop first; else move to next index
            if self.current_wp is None:
                self.current_wp = self.explore_waypoints[0]
                self.current_wp_retries = 0
            else:
                # find index and get next
                try:
                    idx = self.explore_waypoints.index(self.current_wp)
                    next_idx = (idx + 1) % len(self.explore_waypoints)
                    self.current_wp = self.explore_waypoints[next_idx]
                    self.current_wp_retries = 0
                except Exception:
                    self.current_wp = self.explore_waypoints[0]
                    self.current_wp_retries = 0
        else:
            # sample random
            wp = self.sample_random_waypoint_from_map()
            if wp:
                self.current_wp = wp
                self.current_wp_retries = 0
            else:
                self.publish_status("No valid sampled waypoint")
                self.current_wp = None
                return

        x, y, yaw = self.current_wp
        self.publish_status(f"Exploring waypoint -> ({x:.2f},{y:.2f})")
        sent = self.send_nav_goal(x, y, yaw)
        if not sent:
            self.publish_status("Failed to send waypoint to Nav2 (not ready)")

        # publish current waypoints to RViz
        self.publish_waypoint_markers()

    def start_scanning(self):
        self.scanning_start = time.time()
        self.publish_status("Scanning at waypoint (rotating 360°)")

    # ---------------- local motion ----------------
    def publish_twist(self, linear=0.0, angular=0.0):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self.cmd_vel_pub.publish(t)

    def rotate_in_place(self):
        self.publish_twist(linear=0.0, angular=ROTATION_SPEED)

    def stop_motion(self):
        self.cancel_current_goal()
        for _ in range(3):
            self.publish_twist(0.0, 0.0)

    # ---------------- metrics ----------------
    def record_metrics(self, success: bool):
        try:
            with open(METRICS_CSV, 'a', newline='') as f:
                writer = csv.writer(f)
                detect_t = self.first_detection_time
                reach_t = self.reached_time if self.reached_time else (time.time() if success else None)
                detection_latency = (detect_t - self.selection_start_time) if (detect_t and self.selection_start_time) else None
                time_to_reach = (reach_t - detect_t) if (reach_t and detect_t) else None
                writer.writerow([
                    self.selection_counter,
                    self.selected_object,
                    f"{self.selection_start_time:.3f}" if self.selection_start_time else "",
                    f"{detect_t:.3f}" if detect_t else "",
                    f"{detection_latency:.3f}" if detection_latency else "",
                    f"{reach_t:.3f}" if reach_t else "",
                    f"{time_to_reach:.3f}" if time_to_reach else "",
                    self.nav_goals_sent_count,
                    self.nav_goals_canceled_count,
                    "success" if success else "failure"
                ])
        except Exception as e:
            self.get_logger().warning(f"Failed to write metrics: {e}")

    # ---------------- Visualization markers ----------------
    def publish_waypoint_markers(self):
        ma = MarkerArray()
        idx = 0
        if self.explore_waypoints:
            for (x,y,yaw) in self.explore_waypoints:
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'object_follower_waypoints'
                m.id = idx
                m.type = Marker.CUBE
                m.action = Marker.ADD
                m.pose.position.x = x
                m.pose.position.y = y
                m.pose.position.z = 0.05
                q = tf_transformations.quaternion_from_euler(0,0,yaw)
                m.pose.orientation.x = q[0]; m.pose.orientation.y = q[1]; m.pose.orientation.z = q[2]; m.pose.orientation.w = q[3]
                m.scale.x = 0.2; m.scale.y = 0.2; m.scale.z = 0.02
                m.color = ColorRGBA(r=0.0, g=0.8, b=0.1, a=0.9)
                ma.markers.append(m)
                idx += 1
        # also add current target bigger marker
        if self.current_wp:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'object_follower_current_wp'
            m.id = 999
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = self.current_wp[0]
            m.pose.position.y = self.current_wp[1]
            m.pose.position.z = 0.1
            m.scale.x = 0.25; m.scale.y = 0.25; m.scale.z = 0.25
            m.color = ColorRGBA(r=0.9, g=0.2, b=0.2, a=0.9)
            ma.markers.append(m)
        self.waypoint_marker_pub.publish(ma)

    # ---------------- main behavior ----------------
    def timer_callback(self):
        now = time.time()

        # check nav2 action server availability
        if not self.nav2_ready:
            if self.nav_action_client.wait_for_server(timeout_sec=0.5):
                self.nav2_ready = True
                self.get_logger().info("Nav2 action server is available.")
            else:
                # not ready yet, but continue other logic (search fallback etc.)
                # we still "wait"
                pass

        # IDLE
        if self.state == IDLE:
            return

        # Ensure we have map & amcl_pose for exploration transitions
        if self.state == EXPLORING and (not self.map_received or not self.amcl_pose):
            # Wait until both available before doing exploration
            self.publish_status("[EXPLORING] Waiting for map/amcl_pose before sending waypoints")
            return

        # EXPLORING: handle scanning after reaching waypoint OR if detection present => TRACKING
        if self.state == EXPLORING:
            if self.current_detection:
                self.publish_twist(0.0, 0.0)
                return

            # handle scanning
            if getattr(self, 'scanning_start', None) is not None:
                elapsed = now - self.scanning_start
                if elapsed < SCAN_DURATION:
                    self.rotate_in_place()
                    return
                else:
                    # finished scanning -> goto next waypoint
                    self.scanning_start = None
                    self.stop_motion()
                    # mark next waypoint
                    self.goto_next_waypoint()
                    return
            # if no current waypoint or waypoint was skipped/reached, ensure one is selected
            if self.current_wp is None and self.explore_waypoints:
                self.goto_next_waypoint()
                return
            return

        # SEARCHING: rotate -> forward step; if exceeds SEARCH_MAX_DURATION -> resume EXPLORING
        if self.state == SEARCHING:
            if self.last_detection_time and (now - self.last_detection_time) < OBJECT_LOST_TIMEOUT:
                self.state = TRACKING
                self.publish_status("Switch to TRACKING (detected during search)")
                return

            if not self.search_start_time:
                self.search_start_time = now

            if (now - self.search_start_time) >= SEARCH_MAX_DURATION:
                if USE_EXPLORATION and self.map_received:
                    self.state = EXPLORING
                    self.publish_status("Search timed out -> resuming EXPLORING")
                    if not self.goal_sent and getattr(self, 'scanning_start', None) is None:
                        self.goto_next_waypoint()
                    return
                else:
                    self.search_start_time = now

            if not self.last_rotate_time:
                self.last_rotate_time = now
            if (now - self.last_rotate_time) < ROTATE_TIMEOUT:
                self.publish_status("Searching: rotating")
                self.rotate_in_place()
                return

            if not self.search_forward_active:
                self.search_forward_active = True
                self.search_forward_start = now
                self.publish_status("Searching: forward step")
                self.publish_twist(linear=SEARCH_FORWARD_SPEED, angular=0.0)
                return
            else:
                if (now - self.search_forward_start) < SEARCH_FORWARD_TIME:
                    self.publish_twist(linear=SEARCH_FORWARD_SPEED, angular=0.0)
                    return
                else:
                    self.search_forward_active = False
                    self.last_rotate_time = now
                    self.publish_twist(0.0, 0.0)
                    self.publish_status("Search step done, resuming rotation")
                    return

        # TRACKING: servo + compute short horizon Nav2 goal
        if self.state == TRACKING:
            # lost detection? go to SEARCHING
            if not self.last_detection_time or (now - self.last_detection_time) >= OBJECT_LOST_TIMEOUT:
                self.state = SEARCHING
                self.last_rotate_time = now
                self.search_start_time = now
                self.search_forward_active = False
                self.publish_status("Lost object. Switching to SEARCHING.")
                return

            # center in image (pixel center)
            if self.current_detection:
                img_w = self.detection_image_w or CAM_WIDTH_DEFAULT
                cx = 0.5 * (float(self.current_detection.left) + float(self.current_detection.right))
                error_px = (cx - (img_w / 2.0))
                if abs(error_px) > CENTER_TOL_PX:
                    # rotate proportional
                    ang = -max(-CENTER_ANGULAR_SPEED, min(CENTER_ANGULAR_SPEED, (error_px / (img_w/2.0)) * CENTER_ANGULAR_SPEED))
                    self.publish_twist(0.0, ang)
                    self.publish_status(f"Centering object (err_px={error_px:.1f}) angular={ang:.3f}")
                    return
                # else centered -> compute goal

            # lidar safety check
            if not self.is_path_clear():
                self.publish_status("Path blocked (lidar). Canceling goal.")
                self.cancel_current_goal()
                return

            # lidar -> STOP (close)
            if self.latest_laser:
                center_idx = len(self.latest_laser.ranges) // 2
                r = self.latest_laser.ranges[center_idx]
                if r and not math.isinf(r) and not math.isnan(r) and r <= FOLLOW_DISTANCE:
                    self.state = STOPPED
                    self.reached_time = time.time()
                    self.publish_status("At safe distance (lidar). Stopping.")
                    self.stop_motion()
                    self.record_metrics(success=True)
                    return

            # compute camera_link pose in map and send short goal (only after centered)
            try:
                tf = self.tf_buffer.lookup_transform('map', 'camera_link', Time())
                yaw = tf_transformations.euler_from_quaternion([
                    tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z,
                    tf.transform.rotation.w
                ])[2]

                goal_x = tf.transform.translation.x + FOLLOW_DISTANCE * math.cos(yaw)
                goal_y = tf.transform.translation.y + FOLLOW_DISTANCE * math.sin(yaw)

                # send nav goal
                self.send_nav_goal(goal_x, goal_y, yaw)
                self.publish_status("Sent/updated Nav2 goal (tracking)")
            except Exception as e:
                self.get_logger().error(f"TF lookup failed in TRACKING: {e}")
                self.state = SEARCHING
                self.search_start_time = now
                self.last_rotate_time = now
                return

        # STOPPED -> when object lost: SEARCHING
        if self.state == STOPPED:
            if not self.last_detection_time or (now - self.last_detection_time) >= OBJECT_LOST_TIMEOUT:
                self.state = SEARCHING
                self.last_rotate_time = now
                self.search_forward_active = False
                self.search_start_time = now
                self.publish_status("Object lost from STOPPED. Switching to SEARCHING before resuming exploration.")
            else:
                self.stop_motion()

        # Additional: handle Nav2 non-success for exploration: check retry/skip
        # We inspect last_nav_result_time and current_wp_retries (managed via callbacks)
        # Here we add fallback: if goal aborted but robot near target -> treat reached.
        if self.current_wp and self.amcl_pose:
            # compute distance to current wp
            dx = self.current_wp[0] - self.amcl_pose.pose.pose.position.x
            dy = self.current_wp[1] - self.amcl_pose.pose.pose.position.y
            d = math.hypot(dx, dy)
            # If Nav2 not active or aborted often but we are physically close -> treat as reached
            if d < NEAR_GOAL_EPS:
                self.get_logger().info(f"Local fallback: near waypoint d={d:.3f} treat as reached")
                # simulate reached waypoint: start scanning
                if self.state == EXPLORING:
                    self.stop_motion()
                    self.start_scanning()
                    self.current_wp = None
                elif self.state == TRACKING:
                    self.stop_motion()
                # metrics handled by TRACKING/STOPPED logic

    # ---------------- status helper ----------------
    def publish_status(self, text):
        try:
            msg = String()
            msg.data = f"[{self.state}] {text}"
            self.status_pub.publish(msg)
        except Exception:
            pass
        self.get_logger().info(text)

    # ---------------- utilities ----------------
    def _yaw_from_quat(self, q):
        return tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

def main(args=None):
    rclpy.init(args=args)
    node = ObjectFollower()
    try:
        rclpy.spin(node)
    finally:
        node.stop_motion()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()