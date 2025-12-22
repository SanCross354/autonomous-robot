#!/usr/bin/env python3
"""
object_follower_node_auto_waypoints.py
Revised follower with:
 - Robust handling for Nav2 readiness (pending waypoint queue)
 - Safer STOPPED -> SEARCHING -> EXPLORING transition
 - Advance to next waypoint if NAV2 returns non-success while exploring
 - Start exploring automatically when map later becomes available
 - Minor extra logging for debugging
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from yolov8_msgs.msg import Yolov8Inference
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import tf_transformations
import tf2_ros
import math
import time
import csv
import os

# ----------------- Tunables (unchanged) -----------------
FOLLOW_DISTANCE = 1.0            # meters
OBJECT_LOST_TIMEOUT = 2.5        # s before declaring object lost
ROTATE_TIMEOUT = 6.0             # s rotate before forward step (SEARCHING)
SEARCH_FORWARD_TIME = 1.0        # s forward in SEARCHING
SEARCH_FORWARD_SPEED = 0.18      # m/s forward in SEARCHING
ROTATION_SPEED = 0.35            # rad/s rotate speed in SEARCHING / scanning
DETECTION_STOP_THRESHOLD = 0.6   # bbox height ratio => vision stop
MIN_GOAL_UPDATE = 0.15           # m change needed to refresh goal
MIN_GOAL_PERIOD = 0.8            # s min period between goal sends
NAV_RESULT_COOLDOWN = 0.8        # s after a NAV2 result before we can send again
NEAR_GOAL_EPS = 0.25             # m: if already this close to last goal, don't resend
SECTOR_HALF_WIDTH = 5            # laser beams around center to check for obstacle
SCAN_ROTATIONS = 1.0             # how many full turns at a waypoint
SCAN_DURATION = (2.0 * math.pi * SCAN_ROTATIONS) / max(1e-6, abs(ROTATION_SPEED))
SEARCH_MAX_DURATION = 18.0       # s to remain in SEARCHING before giving up to EXPLORING
WAYPOINT_MARGIN = 0.6            # meters margin from map edges for auto waypoints
SNAP_SEARCH_RADIUS_M = 0.8       # when snapping waypoint to free cell, search radius
METRICS_CSV = "/tmp/object_follower_metrics.csv"
USE_EXPLORATION = True           # toggle exploring behavior on/off

# Centering servo parameters (before sending tracking goal)
CENTER_TOLERANCE = 0.15         # fraction of half-image; <= this considered centered
CENTER_ANGULAR_SPEED = 0.45     # rad/s max angular while centering
CENTER_SPIN_MAX = 3.0           # max seconds to try centering continuously

# ----------------- States -----------------
IDLE = "IDLE"
EXPLORING = "EXPLORING"
SEARCHING = "SEARCHING"
TRACKING = "TRACKING"
STOPPED = "STOPPED"


class ObjectFollower(Node):
    def __init__(self):
        super().__init__('object_follower_auto_waypoints')

        # State
        self.state = IDLE
        self.selected_object = None
        self.last_detection_time = None
        self.current_detection = None
        self.detection_image_w = None
        self.detection_image_h = None
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
        self.current_wp_index = 0
        self.scanning_start = None

        # Pending waypoint (for when nav2 isn't ready when we request navigation)
        self.pending_waypoint = None  # (x,y,yaw)
        self.wp_retry_limit = 2       # how many times to retry same wp before skipping
        self.wp_retry_count = 0

        # If GUI selects while no map: wait to start exploring after map arrives
        self.waiting_for_map_for_exploration = False

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Metrics
        self.selection_counter = 0
        self.selection_start_time = None
        self.first_detection_time = None
        self.reached_time = None

        # ROS I/O
        self.create_subscription(Yolov8Inference, '/Yolov8_Inference', self.detection_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.create_subscription(String, '/selected_object_class', self.selector_callback, 10)
        # self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)
        q = QoSProfile(depth=1)
        q.durability = DurabilityPolicy.TRANSIENT_LOCAL
        q.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile=q)


        self.status_pub = self.create_publisher(String, '/object_follower_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

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

        self.get_logger().info("Object follower node (auto waypoints) started")

    # ---------------- map -> generate waypoints ----------------
    def map_callback(self, msg: OccupancyGrid):
        # Called repeatedly by map_server; only generate once or if map changes
        if self.map_received:
            return
        self.get_logger().info("Map received; computing auto waypoints.")
        self.map = msg
        self.map_received = True
        self.explore_waypoints = self.generate_waypoints_from_map(msg)
        self.get_logger().info(f"Auto waypoints set from map ({len(self.explore_waypoints)} pts).")
        if len(self.explore_waypoints) > 0:
            self.get_logger().info("Waypoints (x,y,yaw):")
            for i, w in enumerate(self.explore_waypoints):
                self.get_logger().info(f"  {i+1}: {w}")

        # If GUI asked to explore earlier but map was not present, start exploring now
        if self.waiting_for_map_for_exploration and self.selected_object:
            self.waiting_for_map_for_exploration = False
            self.state = EXPLORING
            self.current_wp_index = 0
            self.scanning_start = None
            self.get_logger().info("Map arrived and selection pending -> start EXPLORING.")
            self.goto_next_waypoint()

    def generate_waypoints_from_map(self, grid: OccupancyGrid):
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

        self.get_logger().info(f"Map bounds (xmin,xmax,ymin,ymax) = ({xmin:.2f},{xmax:.2f},{ymin:.2f},{ymax:.2f}), res={res:.3f}, size={w}x{h}")

        # Candidate points: center + 4 corners with margin
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
                    # free = 0 ; occupied > 0 ; unknown = -1. Accept free only
                    if val == 0:
                        x_world = ox + (ii + 0.5) * res
                        y_world = oy + (jj + 0.5) * res
                        return (x_world, y_world)
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
            # If we want exploration and the map is present -> start exploring immediately
            if USE_EXPLORATION and self.map_received and len(self.explore_waypoints) > 0:
                self.state = EXPLORING
                self.current_wp_index = 0
                self.scanning_start = None
                self.pending_waypoint = None
                self.wp_retry_count = 0
                self.goto_next_waypoint()
                self.publish_status(f"[{self.state}] Selected '{self.selected_object}'. Start exploring waypoints...")
            elif USE_EXPLORATION and not self.map_received:
                # Wait for map; meanwhile search in-place
                self.waiting_for_map_for_exploration = True
                self.state = SEARCHING
                self.last_rotate_time = time.time()
                self.search_start_time = self.last_rotate_time
                self.publish_status(f"[{self.state}] Selected '{self.selected_object}'. Waiting for map then EXPLORING.")
            else:
                # fallback: search in place
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

    # ---------------- detection ----------------
    def detection_callback(self, msg: Yolov8Inference):
        if not self.selected_object:
            return

        found = False
        self.detection_image_w = getattr(msg, 'image_width', None)
        self.detection_image_h = getattr(msg, 'image_height', None)

        for det in msg.yolov8_inference:
            if det.class_name == self.selected_object:
                found = True
                self.current_detection = det
                self.last_detection_time = time.time()
                image_height = self.detection_image_h or getattr(msg, 'image_height', 0) or 640
                bbox_height = max(0.0, float(det.bottom - det.top))
                ratio = (bbox_height / float(image_height)) if image_height > 0 else 0.0

                if self.first_detection_time is None:
                    self.first_detection_time = time.time()

                if ratio >= DETECTION_STOP_THRESHOLD:
                    self.state = STOPPED
                    self.publish_status("Object close enough (vision). Stopping.")
                    self.cancel_current_goal()
                    self.stop_motion()
                    self.record_metrics(success=True)
                    return

                # transition to TRACKING
                if self.state in (EXPLORING, SEARCHING, STOPPED):
                    self.scanning_start = None
                    self.cancel_current_goal()
                    self.publish_twist(0.0, 0.0)
                    self.state = TRACKING
                    self.search_start_time = None
                    self.publish_status("Object detected! Switching to TRACKING.")
                self.get_logger().info(f"Detected {det.class_name} bbox (({det.left},{det.top}) -> ({det.right},{det.bottom}))")
                return

        if not found:
            self.current_detection = None

    # ---------------- laser ----------------
    def laser_callback(self, msg: LaserScan):
        self.latest_laser = msg

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
        # If action server not ready, stash pending waypoint and return.
        if not self.nav2_ready:
            self.pending_waypoint = (x, y, yaw)
            self.get_logger().warn("Nav2 not ready; storing pending waypoint")
            return

        if not self.should_send_goal(x, y, yaw):
            return

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
            if not self.nav_action_client.wait_for_server(timeout_sec=0.5):
                # store pending waypoint and leave
                self.pending_waypoint = (x, y, yaw)
                self.get_logger().warning("Nav2 action server not available; saved pending waypoint")
                return
            send_future = self.nav_action_client.send_goal_async(nav_goal)
            send_future.add_done_callback(self.goal_response_callback)
            self.last_nav_goal = (x, y, yaw)
            self.last_goal_time = time.time()
            self.nav_goals_sent_count += 1
            self.get_logger().info(f"Nav goal send attempt (count={self.nav_goals_sent_count}) -> ({x:.2f},{y:.2f})")
        except Exception as e:
            self.get_logger().error(f"Failed to send nav goal: {e}")
            self.pending_waypoint = (x, y, yaw)

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
        # Called when a nav goal finishes (succeeded/cancelled/aborted)
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
            if status == GoalStatus.STATUS_SUCCEEDED:
                # success -> behave according to mode
                if self.state == EXPLORING:
                    self.stop_motion()
                    self.start_scanning()
                    # reset retry count for this waypoint
                    self.wp_retry_count = 0
                else:
                    self.publish_status("Navigation succeeded (result callback)")
            else:
                # not succeeded => if exploring, skip or retry limited times
                self.get_logger().warning(f"Nav2 goal ended with status={status}; not treating waypoint as reached")
                self.publish_status("Navigation ended (non-success result)")
                if self.state == EXPLORING:
                    # attempt retry limited times, otherwise skip to next waypoint
                    if self.wp_retry_count < self.wp_retry_limit:
                        self.wp_retry_count += 1
                        self.get_logger().info(f"Retrying waypoint #{self.current_wp_index+1} (retry {self.wp_retry_count})")
                        # resend same waypoint if nav ready
                        wp = self.explore_waypoints[self.current_wp_index] if self.current_wp_index < len(self.explore_waypoints) else None
                        if wp:
                            self.pending_waypoint = wp
                    else:
                        self.get_logger().info(f"Skipping waypoint #{self.current_wp_index+1} after {self.wp_retry_count} retries")
                        self.wp_retry_count = 0
                        self.current_wp_index += 1
                        # go to next
                        self.goto_next_waypoint()
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
        self.pending_waypoint = None

    # ---------------- Scanning / Waypoints ----------------
    def goto_next_waypoint(self):
        if not self.explore_waypoints or self.current_wp_index >= len(self.explore_waypoints):
            self.publish_status("Exploration finished or no waypoints available.")
            self.state = IDLE
            return
        x, y, yaw = self.explore_waypoints[self.current_wp_index]
        self.publish_status(f"Exploring waypoint {self.current_wp_index + 1}/{len(self.explore_waypoints)} -> ({x:.2f},{y:.2f})")
        # store as pending and attempt to send (send_nav_goal will handle nav2_ready)
        self.pending_waypoint = (x, y, yaw)
        self.wp_retry_count = 0
        self.send_pending_waypoint_if_ready()

    def send_pending_waypoint_if_ready(self):
        if not self.pending_waypoint:
            return
        x, y, yaw = self.pending_waypoint
        self.get_logger().debug("Attempting to send pending waypoint")
        self.send_nav_goal(x, y, yaw)

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

    # ---------------- main behavior ----------------
    def timer_callback(self):
        now = time.time()

        # non-blocking check for nav2 action server availability
        if not self.nav2_ready:
            if self.nav_action_client.wait_for_server(timeout_sec=0.5):
                self.nav2_ready = True
                self.get_logger().info("Nav2 action server is available.")
                # if we had a pending waypoint, send it now
                if self.pending_waypoint:
                    self.get_logger().info("Sending pending waypoint now that Nav2 is ready")
                    self.send_pending_waypoint_if_ready()
            # else: still not ready; continue

        # IDLE
        if self.state == IDLE:
            return

        # EXPLORING: waypoint -> scan -> next
        if self.state == EXPLORING:
            # If detection present, TRACKING takes over (detection_callback already switched)
            if self.current_detection:
                self.publish_twist(0.0, 0.0)
                return

            # handle scanning after reaching waypoint
            if self.scanning_start is not None:
                elapsed = now - self.scanning_start
                if elapsed < SCAN_DURATION:
                    self.rotate_in_place()
                    return
                else:
                    # finished scanning -> goto next waypoint
                    self.scanning_start = None
                    self.stop_motion()
                    self.current_wp_index += 1
                    self.goto_next_waypoint()
                    return

            # If we haven't sent the pending waypoint for some reason and Nav2 is ready, try sending
            if self.pending_waypoint and not self.goal_sent:
                self.get_logger().debug("Exploring: attempting to send pending waypoint")
                self.send_pending_waypoint_if_ready()
                return

            # otherwise wait for nav2 to bring robot to waypoint (goal_result callbacks handle next)
            return

        # SEARCHING: rotate -> forward step; if exceeds SEARCH_MAX_DURATION -> resume EXPLORING
        if self.state == SEARCHING:
            # recently detected? -> TRACKING
            if self.last_detection_time and (now - self.last_detection_time) < OBJECT_LOST_TIMEOUT:
                self.state = TRACKING
                self.publish_status("Switch to TRACKING (detected during search)")
                return

            if not self.search_start_time:
                self.search_start_time = now

            if (now - self.search_start_time) >= SEARCH_MAX_DURATION:
                if USE_EXPLORATION and self.explore_waypoints:
                    self.state = EXPLORING
                    self.publish_status("Search timed out -> resuming EXPLORING")
                    # try to pick up exploration
                    if not self.goal_sent and self.scanning_start is None:
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

        # TRACKING: compute short horizon goal from camera_link in map frame
        if self.state == TRACKING:
            # lost detection? go to SEARCHING
            if not self.last_detection_time or (now - self.last_detection_time) >= OBJECT_LOST_TIMEOUT:
                self.state = SEARCHING
                self.last_rotate_time = now
                self.search_start_time = now
                self.search_forward_active = False
                self.publish_status("Lost object. Switching to SEARCHING.")
                return

            # If we have a detection, try to center the object in the camera before sending nav goal
            if self.current_detection:
                img_w = self.detection_image_w or 640
                cx = 0.5 * (float(self.current_detection.left) + float(self.current_detection.right))
                error = (cx - (img_w / 2.0)) / (img_w / 2.0)  # normalized [-1,1]
                if abs(error) > CENTER_TOLERANCE:
                    ang = -max(-CENTER_ANGULAR_SPEED, min(CENTER_ANGULAR_SPEED, error * CENTER_ANGULAR_SPEED))
                    self.publish_twist(0.0, ang)
                    self.publish_status(f"Centering object (err={error:.3f}) angular={ang:.3f}")
                    return
                # else: centered — proceed to compute goal

            # lidar safety check
            if not self.is_path_clear():
                self.publish_status("Path blocked (lidar). Canceling goal.")
                self.cancel_current_goal()
                return

            # lidar -> STOP
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

                # reuse send_nav_goal (which will check nav2_ready)
                self.send_nav_goal(goal_x, goal_y, yaw)
                self.publish_status("Sent/updated Nav2 goal (tracking)")
            except Exception as e:
                self.get_logger().error(f"TF lookup failed in TRACKING: {e}")
                self.state = SEARCHING
                self.search_start_time = now
                self.last_rotate_time = now
                return

        # STOPPED: wait until object moves away; then go to SEARCHING (not straight to EXPLORING)
        if self.state == STOPPED:
            if not self.last_detection_time or (now - self.last_detection_time) >= OBJECT_LOST_TIMEOUT:
                self.state = SEARCHING
                self.last_rotate_time = now
                self.search_forward_active = False
                self.search_start_time = now
                self.publish_status("Object lost from STOPPED. Switching to SEARCHING before resuming exploration.")
            else:
                self.stop_motion()

    # ---------------- status helper ----------------
    def publish_status(self, text):
        msg = String()
        msg.data = f"[{self.state}] {text}"
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data)


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