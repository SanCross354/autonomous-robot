#!/usr/bin/env python3
"""
object_follower_node_full.py

Complete improved object follower node for ROS2 Foxy:
 - Nav2 exploration for waypoints (map-based)
 - YOLOv8 detection subscription
 - CSRT tracking (OpenCV) for local visual servoing via /cmd_vel
 - Annotated image publisher (/camera/yolo_annotated)
 - Robust Nav2 goal validation (snap to free cell)
 - Safe tracker init (clamp bbox to image)
 - Status publishing to /object_follower_status

[Thesis Note] This file keeps the high-level state-machine:
  IDLE -> EXPLORING -> SEARCHING -> TRACKING -> STOPPED
Switch to TRACKING cancels Nav2 goals and uses cmd_vel based servoing for fine approach.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf_transformations import euler_from_quaternion

import math
import time
import random
import csv
import os
import threading

# ROS messages / actions
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from std_msgs.msg import String, ColorRGBA
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from yolov8_msgs.msg import Yolov8Inference  # Your custom inference message (adjust if different)

from cv_bridge import CvBridge
import cv2
import tf_transformations
import tf2_ros

# ----------------- Tunables (configurable) -----------------
FOLLOW_DISTANCE = 1.0            # desired distance in meters from camera to object
OBJECT_LOST_TIMEOUT = 2.0        # seconds before declaring object lost
ROTATE_TIMEOUT = 6.0             # secs to rotate in SEARCHING phase before forward step
SEARCH_FORWARD_TIME = 1.0        # secs to drive forward in SEARCHING
SEARCH_FORWARD_SPEED = 0.18      # m/s during SEARCHING forward step
ROTATION_SPEED = 0.35            # rad/s rotation speed
DETECTION_STOP_THRESHOLD = 0.55  # bbox height ratio => stop (visual close)
MIN_GOAL_UPDATE = 0.15           # m change to consider as new nav goal
MIN_GOAL_PERIOD = 0.8            # s min between nav goal sends
NAV_RESULT_COOLDOWN = 0.8        # s after nav2 result before sending again
NEAR_GOAL_EPS = 0.25             # m treat as reached fallback
SECTOR_HALF_WIDTH = 5            # laser beams half width around front to check obstacles
SCAN_ROTATIONS = 1.0
SEARCH_MAX_DURATION = 18.0
WAYPOINT_MARGIN = 0.6
SNAP_SEARCH_RADIUS_M = 0.8
METRICS_CSV = "/tmp/object_follower_metrics.csv"
USE_EXPLORATION = True

# Camera defaults
CAM_WIDTH_DEFAULT = 640
CAM_HEIGHT_DEFAULT = 480

# Stable detection (not strict; we use immediate switching with CSRT)
STABLE_DETECTION_FRAMES = 2

# Tracking params (CSRT)
TRACKING_SWITCH_RATIO = 0.17    # when bbox height covers > this fraction of image -> start local tracking
TRACKER_TYPE = 'CSRT'           # 'CSRT' (recommended), can fallback to 'KCF' or 'MOSSE'
TRACKING_LOST_TIMEOUT = 1.5     # sec until tracker considered lost

# Control gains for visual servoing (tweak as needed)
Kp_ang = 0.002
Kp_lin = 0.6
MAX_LIN = 0.18
MAX_ANG = 0.6

# [Thesis Note]: how often tracking loop runs (Hz)
TRACKING_HZ = 10.0

# ----------------- States -----------------
IDLE = "IDLE"
EXPLORING = "EXPLORING"
SEARCHING = "SEARCHING"
TRACKING = "TRACKING"
STOPPED = "STOPPED"

# -----------------------------------------------------------------------------------------
# Utility helpers
# -----------------------------------------------------------------------------------------
def clamp_bbox(x, y, w, h, img_w, img_h):
    """Clamp bbox to be inside image. Return (x,y,w,h) ints or None if invalid."""
    x = int(max(0, round(x)))
    y = int(max(0, round(y)))
    w = int(round(w))
    h = int(round(h))
    # adjust w/h to not exceed image bounds
    if x >= img_w or y >= img_h:
        return None
    if w <= 0 or h <= 0:
        return None
    if x + w > img_w:
        w = img_w - x
    if y + h > img_h:
        h = img_h - y
    if w <= 0 or h <= 0:
        return None
    return (x, y, w, h)

# -----------------------------------------------------------------------------------------
# Main Node
# -----------------------------------------------------------------------------------------
class ObjectFollower(Node):
    def __init__(self):
        super().__init__('object_follower_full')

        # --- Node state ---
        self.state = IDLE
        self.selected_object = None
        self.current_detection = None        # Yolov8Inference det object (best)
        self.last_detection_time = None
        self.detection_image_w = CAM_WIDTH_DEFAULT
        self.detection_image_h = CAM_HEIGHT_DEFAULT

        # Tracking / CSRT
        self.tracking_mode = False
        self.tracker = None
        self.tracking_lost_time = None
        self.tracking_switch_ratio = TRACKING_SWITCH_RATIO
        self.tracker_type = TRACKER_TYPE
        self.camera_image = None
        self.cv_bridge = CvBridge()
        self.cx_ema = None
        self.ratio_ema = None

        # Nav2 action client
        self.nav_action_client = None
        try:
            self.nav_action_client = rclpy.action.ActionClient(self, NavigateToPose, 'navigate_to_pose')
        except Exception:
            # if rclpy version or import differences, still allow
            self.get_logger().warn("Nav2 action client init error; Nav2-based features may not work.")
            self.nav_action_client = None
        self.nav2_ready = False
        self.goal_handle = None
        self.goal_sent = False
        self.last_nav_goal = None
        self.last_goal_time = 0.0
        self.last_nav_result_time = 0.0
        self.last_nav_goal_time = 0.0
        self.nav_goals_sent_count = 0
        self.nav_goals_canceled_count = 0
        self.nav_cancel_pending_until = 0.0  # small wait time after cancel to avoid immediate re-send

        # Map and waypoints
        self.map_received = False
        self.map = None
        self.explore_waypoints = []
        self.current_wp = None
        self.current_wp_retries = 0

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # AMCL pose
        self.amcl_pose = None

        # Laser
        self.latest_laser = None

        # Searching bookkeeping
        self.last_rotate_time = None
        self.search_forward_start = None
        self.search_forward_active = False
        self.search_start_time = None

        # Markers / publishers
        self.waypoint_marker_pub = self.create_publisher(MarkerArray, '/object_follower/waypoints', 1)
        self.status_pub = self.create_publisher(String, '/object_follower_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_tracking_pub = self.create_publisher(Twist, '/cmd_vel_tracking', 10)
        self.annotated_pub = self.create_publisher(Image, '/camera/yolo_annotated', 1)

        # Camera publisher
        self.camera_frame = None
        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        # QoS for map transient local
        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Tracking
        self._tracking_topic_sub_count = 0

        # internal helper flags & locks
        self._warned_no_twist_mux = False
        self.yolo_reinit_counter = 0
        self.yolo_reinit_interval = 8      # reinit tracker from YOLO every N detections while TRACKING
        self._nav_lock = threading.Lock()  # serialize nav send/cancel

        # Subscriptions
        self.create_subscription(Yolov8Inference, '/Yolov8_Inference', self.detection_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.create_subscription(String, '/selected_object_class', self.selector_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_map)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)
        self.create_subscription(Image, '/camera/image_raw', self._image_cb, 2)

        # Timers
        self.timer = self.create_timer(0.2, self.timer_callback)          # main state machine / housekeeping
        self.tracking_timer = self.create_timer(1.0 / TRACKING_HZ, self.tracking_loop)

        # Metrics CSV
        if not os.path.exists(METRICS_CSV):
            try:
                with open(METRICS_CSV, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        "selection_id", "selected_class",
                        "selection_time", "first_detection_time", "detection_latency_s",
                        "reach_time", "time_to_reach_s",
                        "nav_goals_sent", "nav_goals_canceled",
                        "result"
                    ])
            except Exception as e:
                self.get_logger().warn(f"Could not create metrics CSV: {e}")

        # Misc
        self.selection_counter = 0
        self.selection_start_time = None
        self.first_detection_time = None
        self.reached_time = None

        self.get_logger().info("Object follower node (full, CSRT) started")

        # default param
        self.declare_parameter('return_home_after_success', False)
        self.return_home_after_success = self.get_parameter('return_home_after_success').get_parameter_value().bool_value


    def camera_info_callback(self, msg: CameraInfo):
        # store the camera frame name as published by the camera plugin
        if msg and msg.header and msg.header.frame_id:
            self.camera_frame = msg.header.frame_id

    # ---------------- Map / Waypoint generation ----------------
    def map_callback(self, msg: OccupancyGrid):
        if self.map_received:
            # optionally handle updates from SLAM-Toolbox; in our simple approach ignore
            return
        self.get_logger().info("Map received; computing auto waypoints.")
        self.map = msg
        self.map_received = True
        self.explore_waypoints = self.generate_waypoints_from_map(msg)
        self.get_logger().info(f"Auto waypoints set from map ({len(self.explore_waypoints)} pts).")
        for i,w in enumerate(self.explore_waypoints):
            self.get_logger().info(f"  {i+1}: {w}")
        # publish markers
        self.publish_waypoint_markers()

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

        # center + four corners offset by margin
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

    # ---------------- Selector / GUI Callback ----------------
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
            self.get_logger().info(f"Selected object: {sel}")
            # store home pose once (first selection), if not set
            if self.amcl_pose and getattr(self, 'home_pose', None) is None:
                self.home_pose = (self.amcl_pose.pose.pose.position.x,
                                  self.amcl_pose.pose.pose.position.y,
                                  self._yaw_from_quat(self.amcl_pose.pose.pose.orientation))
            # decide initial state
            if USE_EXPLORATION and self.map_received and self.amcl_pose:
                self.state = EXPLORING
                self.current_wp = None
                self.current_wp_retries = 0
                self.publish_status(f"[{self.state}] Selected '{self.selected_object}'. Start exploring waypoints...")
                self.goto_next_waypoint()
            else:
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

    # ---------------- Detection callback (YOLOv8) ----------------
    def detection_callback(self, msg: Yolov8Inference):
        """Immediate detection trigger; choose largest bbox of selected class, scale/clamp
        and then:
            - if ratio >= DETECTION_STOP_THRESHOLD -> stop
            - if ratio >= tracking_switch_ratio -> cancel Nav2 and init CSRT tracking (local cmd_vel)
            - else: send a short Nav2 goal in front of camera (rate-limited & validated)
        """
        if not self.selected_object:
            return

        # preserve image dims if provided by YOLO publisher
        self.detection_image_w = getattr(msg, 'image_width', self.detection_image_w) or self.detection_image_w
        self.detection_image_h = getattr(msg, 'image_height', self.detection_image_h) or self.detection_image_h

        # select largest bbox for the selected class
        best_det = None
        best_area = 0.0
        for det in getattr(msg, 'yolov8_inference', []):
            if det.class_name and det.class_name.lower().strip() == self.selected_object.lower().strip():
                bw = max(0.0, float(det.right - det.left))
                bh = max(0.0, float(det.bottom - det.top))
                area = bw * bh
                if area > best_area:
                    best_area = area
                    best_det = det

        if not best_det:
            # no detection, but do not immediately clear tracker; allow tracking loop to handle lost timeout
            self.current_detection = None
            return

        # compute pixel bbox mapped into camera_image space
        left, top, right, bottom = self._scale_detection_to_camera(best_det)
        bbox_h = max(1, bottom - top)
        img_h = self.camera_image.shape[0] if self.camera_image is not None else self.detection_image_h or CAM_HEIGHT_DEFAULT
        ratio = float(bbox_h) / float(img_h) if img_h > 0 else 0.0

        self.current_detection = best_det
        self.last_detection_time = time.time()
        if self.first_detection_time is None:
            self.first_detection_time = self.last_detection_time

        # Near (visual) stop
        if ratio >= DETECTION_STOP_THRESHOLD:
            self.state = STOPPED
            self.publish_status("Object close enough (vision). Stopping.")
            # cancel any current nav goal then zero tracking topic to allow safe stop
            with self._nav_lock:
                try:
                    self.cancel_current_goal()
                except Exception:
                    pass
            self.stop_motion()
            self.record_metrics(success=True)
            return

        # If object large enough -> use local tracking via CSRT
        if ratio >= self.tracking_switch_ratio:
            # Cancel Nav2 and start local tracking
            with self._nav_lock:
                try:
                    self.cancel_current_goal()
                except Exception:
                    pass
                # prevent immediate re-sending after cancel
                self.nav_cancel_pending_until = time.time() + 0.5

            # set tracking state
            self.stop_motion()
            prev_state = self.state
            self.state = TRACKING
            self.search_start_time = None
            self.publish_status(f"Object '{best_det.class_name}' detected! Switching to TRACKING (CSRT).")

            # Try to init tracker on latest camera image
            if self.camera_image is not None:
                try:
                    img_h, img_w = self.camera_image.shape[:2]
                    x = left; y = top; w = right - left; h = bottom - top

                    # safe clamp + fallback
                    clamped = clamp_bbox(x, y, w, h, img_w, img_h)
                    if not clamped or min(clamped[2], clamped[3]) < 16:
                        # try fallback centered ROI
                        cx = int((left + right) / 2)
                        cy = int((top + bottom) / 2)
                        size = max(40, int(min(img_w, img_h) * 0.12))
                        x2 = max(0, cx - size // 2)
                        y2 = max(0, cy - size // 2)
                        w2 = min(size, img_w - x2)
                        h2 = min(size, img_h - y2)
                        clamped = clamp_bbox(x2, y2, w2, h2, img_w, img_h)

                    if clamped:
                        x2, y2, w2, h2 = clamped
                        if self.tracker_type.upper() == 'CSRT':
                            self.tracker = cv2.TrackerCSRT_create()
                        elif self.tracker_type.upper() == 'KCF':
                            self.tracker = cv2.TrackerKCF_create()
                        else:
                            self.tracker = cv2.TrackerMOSSE_create()
                        ok = self.tracker.init(self.camera_image, (x2, y2, w2, h2))
                        if not ok:
                            raise RuntimeError("tracker.init returned False")
                        self.tracking_mode = True
                        self.tracking_lost_time = None
                        self.cx_ema = None
                        self.ratio_ema = None
                        self.yolo_reinit_counter = 0
                        self.get_logger().info(f"Tracker initialized at {(x2,y2,w2,h2)}")
                    else:
                        self.get_logger().warn("Detected bbox invalid after clamp; CSRT not initialized")
                        self.tracking_mode = False
                        self.tracker = None
                except Exception as e:
                    self.get_logger().warn(f"Tracker init failed: {e}")
                    self.tracker = None
                    self.tracking_mode = False
            else:
                # no camera image available now: still in TRACKING, rely on detection updates
                self.tracking_mode = False
            return

        # else: far -> send Nav2 approach goal (rate-limited and validated)
        now = time.time()
        if now - self.last_nav_goal_time > 2.0 and (time.time() > self.nav_cancel_pending_until):
            try:
                # serialize nav sends
                with self._nav_lock:
                    self.send_nav_goal_to_map_point(best_det)
                    self.last_nav_goal_time = now
            except Exception as e:
                self.get_logger().warn(f"send_nav_goal_to_map_point failed: {e}")
        return


    # ------------- convert detection coords to camera image coords -------------
    def _scale_detection_to_camera(self, det):
        """
        Returns left,top,right,bottom in camera_image pixel coordinates (ints).
        If YOLO provides image_width/image_height fields, scale accordingly, else assume detection coords match camera resolution.
        """
        # some YOLO messages include image_width / image_height
        det_w = getattr(det, 'image_width', None)
        det_h = getattr(det, 'image_height', None)
        if self.camera_image is not None:
            cam_h, cam_w = self.camera_image.shape[:2]
        else:
            cam_w = self.detection_image_w or CAM_WIDTH_DEFAULT
            cam_h = self.detection_image_h or CAM_HEIGHT_DEFAULT

        if det_w and det_h and (det_w > 0 and det_h > 0):
            sx = float(cam_w) / float(det_w)
            sy = float(cam_h) / float(det_h)
        else:
            sx = sy = 1.0

        left = int(max(0, round(det.left * sx)))
        top = int(max(0, round(det.top * sy)))
        right = int(min(cam_w, round(det.right * sx)))
        bottom = int(min(cam_h, round(det.bottom * sy)))

        # ensure valid
        if right <= left:
            right = min(cam_w, left + 1)
        if bottom <= top:
            bottom = min(cam_h, top + 1)
        return left, top, right, bottom

    # ---------------- Laser callback ----------------
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
            if r and (not math.isinf(r)) and (not math.isnan(r)):
                if r <= FOLLOW_DISTANCE:
                    return False
        return True

    # ---------------- Image cb: capture camera image and publish annotated image -----------
    def _image_cb(self, msg: Image):
        try:
            cv_img = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.camera_image = cv_img.copy()
        except Exception as e:
            self.get_logger().warn(f"cv bridge convert failed: {e}")
            self.camera_image = None
            return

        # annotate with YOLO bbox and tracker bbox if available
        disp = cv_img.copy()
        # draw current_detection (YOLO)
        try:
            if self.current_detection is not None:
                left, top, right, bottom = self._scale_detection_to_camera(self.current_detection)
                cv2.rectangle(disp, (left, top), (right, bottom), (0, 255, 0), 2)
                label = f"{self.current_detection.class_name}"
                cv2.putText(disp, label, (left, max(10, top - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

            # draw tracker bbox
            if self.tracking_mode and self.tracker is not None:
                try:
                    ok, bbox = self.tracker.update(self.camera_image)
                    if ok:
                        xb, yb, wb, hb = [int(v) for v in bbox]
                        cv2.rectangle(disp, (xb, yb), (xb + wb, yb + hb), (0, 0, 255), 2)
                        cv2.putText(disp, "CSRT", (xb, max(10, yb - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
                except Exception:
                    pass
        except Exception:
            pass

        # publish annotated image
        try:
            out_msg = self.cv_bridge.cv2_to_imgmsg(disp, 'bgr8')
            out_msg.header.stamp = msg.header.stamp
            out_msg.header.frame_id = msg.header.frame_id
            self.annotated_pub.publish(out_msg)
        except Exception:
            pass

    # ---------------- Tracking loop - run at TRACKING_HZ ----------------
    def tracking_loop(self):
        """If tracking_mode true: update tracker or fallback to detections, compute cmd_vel and publish.
        If tracker lost for > TRACKING_LOST_TIMEOUT -> go to SEARCHING.
        """
        if not self.selected_object:
            return
        if self.state != TRACKING:
            return

        # prefer tracker predictions
        bbox = None
        if self.tracker is not None and self.camera_image is not None:
            try:
                ok, b = self.tracker.update(self.camera_image)
                if ok:
                    bbox = (int(b[0]), int(b[1]), int(b[2]), int(b[3]))
                    self.tracking_lost_time = None
                else:
                    if self.tracking_lost_time is None:
                        self.tracking_lost_time = time.time()
                    elif time.time() - self.tracking_lost_time > TRACKING_LOST_TIMEOUT:
                        self.get_logger().info("Tracker lost -> switching to SEARCHING")
                        self.tracking_mode = False
                        self.tracker = None
                        self.start_searching()
                        return
            except Exception as e:
                self.get_logger().warn(f"Tracker update exception: {e}")
                self.tracker = None
                self.tracking_mode = False
                self.start_searching()
                return
        else:
            # fallback: use latest detection bbox if present
            if self.current_detection is not None:
                left, top, right, bottom = self._scale_detection_to_camera(self.current_detection)
                bbox = (left, top, right - left, bottom - top)
            else:
                self.get_logger().info("No tracker and no detection -> SEARCHING")
                self.tracking_mode = False
                self.start_searching()
                return

        # compute control from bbox
        x, y, w, h = [float(v) for v in bbox]
        cx = x + w / 2.0
        img_w = self.detection_image_w or (self.camera_image.shape[1] if self.camera_image is not None else CAM_WIDTH_DEFAULT)
        img_h = self.detection_image_h or (self.camera_image.shape[0] if self.camera_image is not None else CAM_HEIGHT_DEFAULT)

        # pixel error from center (positive = to right)
        cx_err = (cx - (img_w / 2.0))

        # ratio of bbox height
        ratio = h / (img_h if img_h > 0 else 1.0)

        # smoothing (EMA)
        alpha = 0.35
        if self.cx_ema is None:
            self.cx_ema = cx_err
            self.ratio_ema = ratio
        else:
            self.cx_ema = alpha * cx_err + (1.0 - alpha) * self.cx_ema
            self.ratio_ema = alpha * ratio + (1.0 - alpha) * self.ratio_ema

        # angular control (centering)
        ang = -Kp_ang * self.cx_ema
        ang = max(-MAX_ANG, min(MAX_ANG, ang))

        # linear control: exponential approach if we are not yet close
        target_ratio = 0.32
        lin = 0.0
        if self.ratio_ema < target_ratio and self.is_path_clear():
            # exponential approach mapping -> smooth deceleration as ratio approaches target
            beta = 6.0
            v_max = MAX_LIN
            delta = max(0.0, target_ratio - self.ratio_ema)
            lin = v_max * (1.0 - math.exp(-beta * delta))
            lin = max(0.0, min(MAX_LIN, lin))

        # publish cmd_vel (through twist_mux if present)
        twist = Twist()
        twist.linear.x = float(lin)
        twist.angular.z = float(ang)
        self._publish_tracking_twist(twist)

        # if close by vision -> stop and record success
        if self.ratio_ema >= DETECTION_STOP_THRESHOLD:
            self.get_logger().info("Close enough (visual). Stopping and recording success.")
            # stop and zero tracking topic so twist_mux will allow nav to resume
            self.stop_motion()
            self.tracking_mode = False
            self.tracker = None
            self.record_metrics(success=True)
            self.state = STOPPED
            return

        # Periodic YOLO re-init while TRACKING (if YOLO continues to confirm)
        # This helps correct tracker drift between detections.
        if self.current_detection is not None:
            self.yolo_reinit_counter += 1
            if self.yolo_reinit_counter >= self.yolo_reinit_interval:
                # attempt safe reinit from latest detection (non-blocking, logs)
                try:
                    self._reinit_tracker_from_detection(self.current_detection)
                except Exception as e:
                    self.get_logger().warn(f"YOLO reinit attempt failed: {e}")
                finally:
                    self.yolo_reinit_counter = 0


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
        if self.nav_action_client is None:
            self.get_logger().warn("Nav2 action client not present; skip send_nav_goal")
            return False
        if not self.nav_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warning("Nav2 action server not available")
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
            status = getattr(res, 'status', None)
            if status is None:
                try:
                    status = res.result.status
                except Exception:
                    status = None

            self.last_nav_result_time = time.time()
            self.get_logger().info(f"Nav2 goal finished (callback), status={status}")

            if status == GoalStatus.STATUS_SUCCEEDED:
                if self.state == EXPLORING and self.current_wp is not None:
                    self.stop_motion()
                    self.start_scanning()
                elif self.state == TRACKING:
                    self.publish_status("Navigation succeeded (tracking)")
            else:
                # handle aborts: if robot physically near current_wp, treat as reached
                if self.current_wp and self.amcl_pose:
                    dx = self.current_wp[0] - self.amcl_pose.pose.pose.position.x
                    dy = self.current_wp[1] - self.amcl_pose.pose.pose.position.y
                    d = math.hypot(dx, dy)
                    if d < NEAR_GOAL_EPS:
                        self.get_logger().info(f"Nav aborted but near waypoint (d={d:.3f}) -> treat as reached")
                        if self.state == EXPLORING:
                            self.stop_motion()
                            self.start_scanning()
                            self.current_wp = None
                        return
                self.get_logger().warning(f"Nav2 goal ended with status={status}")
                self.publish_status("Navigation ended (non-success result)")
        except Exception as e:
            self.get_logger().warning(f"Nav2 goal result callback error: {e}")
        finally:
            self.goal_sent = False
            self.goal_handle = None
            self.last_nav_goal = None

    def cancel_current_goal(self):
        # Request goal cancellation and set a short lock to avoid immediate re-send
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
        # small lock time to prevent immediate sending after cancel
        self.nav_cancel_pending_until = time.time() + 0.15

    # ---------------- Scanning and waypoint movement --------------
    def goto_next_waypoint(self):
        # dynamic single-step: choose next waypoint either from precomputed or sample random
        if not self.map_received:
            self.publish_status("No map yet; cannot goto waypoint")
            return

        if self.explore_waypoints:
            if self.current_wp is None:
                self.current_wp = self.explore_waypoints[0]
                self.current_wp_retries = 0
            else:
                try:
                    idx = self.explore_waypoints.index(self.current_wp)
                    next_idx = (idx + 1) % len(self.explore_waypoints)
                    self.current_wp = self.explore_waypoints[next_idx]
                    self.current_wp_retries = 0
                except Exception:
                    self.current_wp = self.explore_waypoints[0]
                    self.current_wp_retries = 0
        else:
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

        self.publish_waypoint_markers()

    def start_scanning(self):
        self.scanning_start = time.time()
        self.publish_status("Scanning at waypoint (rotating 360°)")

    # ---------------- Local motion helpers ----------------
    def publish_twist(self, linear=0.0, angular=0.0):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self.cmd_vel_pub.publish(t)

    def rotate_in_place(self):
        self.publish_twist(linear=0.0, angular=ROTATION_SPEED)

    def stop_motion(self):
        # Cancel any nav goals and ensure both cmd_vel and cmd_vel_tracking are zeroed
        try:
            self.cancel_current_goal()
        except Exception:
            pass
        # Zero tracking topic (ensures twist_mux receives zero from tracking)
        try:
            self._publish_tracking_stop()
        except Exception:
            pass
        # Also publish directly to cmd_vel a few times (fallback)
        for _ in range(3):
            self.publish_twist(0.0, 0.0)

    # ---------------- Utility: send small nav goal in front of camera ----------------

    def send_nav_goal_to_map_point(self, det):
        """
        Compute a point FOLLOW_DISTANCE in front of camera (in map frame)
        and send that as Nav2 goal. Use available camera frame (camera_info header),
        fallback to camera_link_optical/camera_link, wait briefly for TF.
        """

        # pick prioritized frames to try
        prioritized_frames = []
        if getattr(self, 'camera_frame', None):
            prioritized_frames.append(self.camera_frame)
        # common fallbacks
        prioritized_frames += ['camera_link_optical', 'camera_link']

        tf_msg = None
        tried = []
        timeout_per_try = 0.6  # seconds to wait per frame attempt

        for frame in prioritized_frames:
            if frame in tried:
                continue
            tried.append(frame)
            # Wait briefly for the transform to become available
            start = self.get_clock().now().seconds_nanoseconds()[0]
            while True:
                try:
                    # NOTE: using Time() (zero) requests latest transform; can_transform is helpful
                    if self.tf_buffer.can_transform('map', frame, Time()):
                        tf_msg = self.tf_buffer.lookup_transform('map', frame, Time())
                        break
                except Exception:
                    # ignore small exceptions and retry until timeout
                    pass
                now = self.get_clock().now().seconds_nanoseconds()[0]
                if (now - start) > timeout_per_try:
                    break
                # sleep small amount (non-blocking)
                rclpy.spin_once(self, timeout_sec=0.05)

            if tf_msg:
                self.get_logger().info(f"Using camera frame '{frame}' for map<-camera transform.")
                break
            else:
                self.get_logger().debug(f"Frame '{frame}' not available yet (tried {timeout_per_try}s).")

        if not tf_msg:
            # ultimate failure
            self.get_logger().warn("TF lookup failed for map <- camera (no usable camera frame). Cannot compute approach goal.")
            # optionally log available frames for debug
            try:
                frames = self.tf_buffer.all_frames_as_string()
                self.get_logger().info(f"TF frames:\n{frames}")
            except Exception:
                pass
            return False

        # Extract translation + yaw from transform
        tx = tf_msg.transform.translation.x
        ty = tf_msg.transform.translation.y
        tz = tf_msg.transform.translation.z
        q = tf_msg.transform.rotation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        # compute desired goal point in front of camera (map frame)
        goal_x = float(tx + FOLLOW_DISTANCE * math.cos(yaw))
        goal_y = float(ty + FOLLOW_DISTANCE * math.sin(yaw))
        goal_yaw = float(yaw)  # orientation aligning with camera yaw

        self.get_logger().info(f"send_nav_goal_to_map_point: camera at ({tx:.2f},{ty:.2f}),"
                            f" goal candidate -> ({goal_x:.2f},{goal_y:.2f}) yaw={goal_yaw:.2f}")

        # Optionally snap to a free cell on the map to ensure it's valid
        snapped = None
        try:
            s = self.snap_to_free_cell(self.map, goal_x, goal_y, SNAP_SEARCH_RADIUS_M) if getattr(self, 'map', None) else None
            if s:
                snapped = (s[0], s[1], goal_yaw)
                goal_x, goal_y = snapped[0], snapped[1]
                self.get_logger().info(f"send_nav_goal_to_map_point: snapped goal to free cell ({goal_x:.2f},{goal_y:.2f})")
        except Exception as e:
            self.get_logger().warn(f"send_nav_goal_to_map_point: snap_to_free_cell failed: {e}")

        # Finally, send nav goal (use your existing send_nav_goal wrapper)
        sent = self.send_nav_goal(goal_x, goal_y, goal_yaw)
        if not sent:
            self.get_logger().warn("send_nav_goal_to_map_point: send_nav_goal returned False (Nav2 not ready or rate-limited).")
            return False

        self.get_logger().info("send_nav_goal_to_map_point: sent nav goal in front of camera")
        return True


    # ---------------- Metrics ----------------
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
        # current waypoint marker
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
    
    # ---------------- Tracking publish helpers ----------------
    def _publish_tracking_twist(self, twist: Twist):
        """
        Publish twist to /cmd_vel_tracking when twist_mux present.
        Fallback to /cmd_vel if no subscriber on tracking topic.
        """
        try:
            if self.cmd_vel_tracking_pub.get_subscription_count() > 0:
                self.cmd_vel_tracking_pub.publish(twist)
            else:
                if not self._warned_no_twist_mux:
                    self.get_logger().warn("No subscriber on /cmd_vel_tracking (twist_mux likely not running). Falling back to /cmd_vel.")
                    self._warned_no_twist_mux = True
                self.cmd_vel_pub.publish(twist)
        except Exception as e:
            self.get_logger().warn(f"_publish_tracking_twist exception: {e} — falling back to /cmd_vel")
            try:
                self.cmd_vel_pub.publish(twist)
            except Exception:
                pass

    def _publish_tracking_stop(self):
        """
        Publish zero twist several times to ensure twist_mux latches zero and Nav2 regains control cleanly.
        Call this when leaving TRACKING (stop_motion calls it).
        """
        zero = Twist()
        for _ in range(3):
            try:
                if self.cmd_vel_tracking_pub.get_subscription_count() > 0:
                    self.cmd_vel_tracking_pub.publish(zero)
                else:
                    self.cmd_vel_pub.publish(zero)
            except Exception:
                try:
                    self.cmd_vel_pub.publish(zero)
                except Exception:
                    pass
            try:
                rclpy.spin_once(self, timeout_sec=0.02)
            except Exception:
                pass

    def _reinit_tracker_from_detection(self, det):
        """
        Reinitialize tracker from a YOLO detection (safe clamp + fallback ROI).
        det is the Yolov8Inference single detection object with left/top/right/bottom or normalized coords.
        """
        try:
            # compute pixel bbox
            left, top, right, bottom = self._scale_detection_to_camera(det)
            if self.camera_image is None:
                self.get_logger().warn("_reinit_tracker_from_detection: no camera image to init tracker")
                return False
            img_h, img_w = self.camera_image.shape[:2]
            x = int(left); y = int(top); w = int(right - left); h = int(bottom - top)

            clamped = clamp_bbox(x, y, w, h, img_w, img_h)
            if not clamped or min(clamped[2], clamped[3]) < 16:
                # fallback centered ROI around detection center
                cx = int((left + right) / 2)
                cy = int((top + bottom) / 2)
                size = max(40, int(min(img_w, img_h) * 0.12))
                x2 = max(0, cx - size // 2)
                y2 = max(0, cy - size // 2)
                w2 = min(size, img_w - x2)
                h2 = min(size, img_h - y2)
                clamped = clamp_bbox(x2, y2, w2, h2, img_w, img_h)

            if not clamped:
                self.get_logger().warn("_reinit_tracker_from_detection: clamped bbox invalid, cannot init tracker")
                return False

            x2, y2, w2, h2 = clamped
            # create tracker safely
            if self.tracker_type.upper() == 'CSRT':
                tracker_new = cv2.TrackerCSRT_create()
            elif self.tracker_type.upper() == 'KCF':
                tracker_new = cv2.TrackerKCF_create()
            else:
                tracker_new = cv2.TrackerMOSSE_create()

            ok = tracker_new.init(self.camera_image, (x2, y2, w2, h2))
            if not ok:
                self.get_logger().warn("_reinit_tracker_from_detection: tracker.init returned False")
                return False
            # swap in
            self.tracker = tracker_new
            self.tracking_mode = True
            self.tracking_lost_time = None
            self.cx_ema = None
            self.ratio_ema = None
            self.get_logger().info(f"Tracker re-initialized from YOLO at {(x2,y2,w2,h2)}")
            return True
        except Exception as e:
            self.get_logger().warn(f"_reinit_tracker_from_detection exception: {e}")
            return False


    # ---------------- Main timer (state machine) ----------------
    def timer_callback(self):
        now = time.time()

        # check nav2 action server availability
        if not self.nav2_ready and self.nav_action_client is not None:
            if self.nav_action_client.wait_for_server(timeout_sec=0.5):
                self.nav2_ready = True
                self.get_logger().info("Nav2 action server is available.")

        if self.state == IDLE:
            return

        if self.state == EXPLORING:
            # if detection present, pause exploration to TRACKING will be triggered by detection_callback
            if self.current_detection:
                self.publish_twist(0.0, 0.0)
                return

            # scanning after reaching waypoint
            if getattr(self, 'scanning_start', None) is not None:
                elapsed = now - self.scanning_start
                # rotate for a full spin then proceed
                if elapsed < (2.0 * math.pi / abs(ROTATION_SPEED)) * SCAN_ROTATIONS:
                    self.rotate_in_place()
                    return
                else:
                    self.scanning_start = None
                    self.stop_motion()
                    self.goto_next_waypoint()
                    return

            if self.current_wp is None and self.explore_waypoints:
                self.goto_next_waypoint()
                return
            return

        if self.state == SEARCHING:
            # detection may have set last_detection_time; if recent, switch to TRACKING
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

            # rotating
            if not self.last_rotate_time:
                self.last_rotate_time = now
            if (now - self.last_rotate_time) < ROTATE_TIMEOUT:
                self.publish_status("Searching: rotating")
                self.rotate_in_place()
                return

            # forward step
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

        if self.state == TRACKING:
            # If detection lost for OBJECT_LOST_TIMEOUT and tracker not active, go to SEARCHING
            if (not self.last_detection_time or (now - self.last_detection_time) >= OBJECT_LOST_TIMEOUT) and (not self.tracking_mode):
                self.state = SEARCHING
                self.last_rotate_time = now
                self.search_start_time = now
                self.search_forward_active = False
                self.publish_status("Lost object. Switching to SEARCHING.")
                return

            # centering via camera pixel method if detection present (quick small rotations)
            if self.current_detection:
                try:
                    left, top, right, bottom = self._scale_detection_to_camera(self.current_detection)
                    img_w = self.detection_image_w or CAM_WIDTH_DEFAULT
                    cx = 0.5 * (float(left) + float(right))
                    error_px = (cx - (img_w / 2.0))
                    center_tol = 40
                    if abs(error_px) > center_tol:
                        ang = - max(-0.45, min(0.45, (error_px / (img_w/2.0)) * 0.45))
                        self.publish_twist(0.0, ang)
                        self.publish_status(f"Centering object (err_px={error_px:.1f}) angular={ang:.3f}")
                        return
                    # else fall through to tracking behavior which will send cmd_vel from tracking_loop
                except Exception:
                    pass

            # lidar safety check (stop if obstacle)
            if not self.is_path_clear():
                self.publish_status("Path blocked (lidar). Canceling goal and stopping.")
                self.cancel_current_goal()
                self.stop_motion()
                return

            # lidar stopping near object (reliable)
            if self.latest_laser:
                center_idx = len(self.latest_laser.ranges) // 2
                try:
                    r = self.latest_laser.ranges[center_idx]
                    if r and (not math.isinf(r)) and (not math.isnan(r)) and r <= FOLLOW_DISTANCE:
                        self.state = STOPPED
                        self.reached_time = time.time()
                        self.publish_status("At safe distance (lidar). Stopping.")
                        self.stop_motion()
                        self.record_metrics(success=True)
                        return
                except Exception:
                    pass

            # compute short Nav2 goal only if neither tracker nor close-centered detection are used
            # but our detection_callback already attempted to send a short Nav2 goal if far.
            return

        if self.state == STOPPED:
            if not self.last_detection_time or (now - self.last_detection_time) >= OBJECT_LOST_TIMEOUT:
                self.state = SEARCHING
                self.last_rotate_time = now
                self.search_forward_active = False
                self.search_start_time = now
                self.publish_status("Object lost from STOPPED. Switching to SEARCHING before resuming exploration.")
            else:
                self.stop_motion()

        # Exploration fallback near current wp (distance fallback)
        if self.current_wp and self.amcl_pose:
            dx = self.current_wp[0] - self.amcl_pose.pose.pose.position.x
            dy = self.current_wp[1] - self.amcl_pose.pose.pose.position.y
            d = math.hypot(dx, dy)
            if d < NEAR_GOAL_EPS:
                self.get_logger().info(f"Local fallback: near waypoint d={d:.3f} treat as reached")
                if self.state == EXPLORING:
                    self.stop_motion()
                    self.start_scanning()
                    self.current_wp = None
                elif self.state == TRACKING:
                    self.stop_motion()

    # ---------------- Searching helpers ----------------
    def start_searching(self):
        """
        Enter SEARCHING state reliably:
        - cancel any outstanding Nav2 goal(s)
        - wait briefly for cancel to be processed (so controller does not keep sending commands)
        - zero tracking topic so twist_mux can switch cleanly
        - then set SEARCHING and start rotation
        """
        # 1) Cancel any currently tracked goal (our own) and try to cancel all via action client as fallback
        try:
            # cancel our stored goal handle (if present)
            if getattr(self, 'goal_handle', None):
                try:
                    cancel_future = self.goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=1.0)
                except Exception:
                    pass

            # Attempt to cancel all goals on Nav2 action server (best-effort)
            if self.nav_action_client is not None:
                try:
                    # Some ROS2 ActionClient implementations support cancel_all_goals_async
                    cancel_all = getattr(self.nav_action_client, 'cancel_all_goals_async', None)
                    if callable(cancel_all):
                        fut = cancel_all()
                        # wait briefly
                        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
                except Exception:
                    pass
        except Exception as e:
            self.get_logger().warn(f"start_searching: exception during cancel: {e}")

        # 2) Give a small lockout time so we don't immediately resend a nav goal
        self.nav_cancel_pending_until = time.time() + 0.6

        # 3) Zero tracking topic so twist_mux latches a safe zero if needed
        try:
            self._publish_tracking_stop()
        except Exception:
            pass

        # 4) Finally, set SEARCHING state and start timers
        self.state = SEARCHING
        self.search_start_time = time.time()
        self.last_rotate_time = time.time()
        self.search_forward_active = False
        self.publish_status("Entering SEARCHING state (tracker lost).")


    # ---------------- status helper ----------------
    def publish_status(self, text):
        try:
            msg = String()
            msg.data = f"[{self.state}] {text}"
            self.status_pub.publish(msg)
        except Exception:
            pass
        self.get_logger().info(text)

    # ---------------- AMCL pose cb ----------------
    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.amcl_pose = msg
        if getattr(self, 'home_pose', None) is None:
            self.home_pose = (msg.pose.pose.position.x,
                              msg.pose.pose.position.y,
                              self._yaw_from_quat(msg.pose.pose.orientation))

    # ---------------- Utilities ----------------
    def _yaw_from_quat(self, q):
        return tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

# -----------------------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ObjectFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motion()
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
