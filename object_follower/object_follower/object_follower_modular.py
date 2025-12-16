#!/usr/bin/env python3
"""
object_follower_modular.py - Modular Object Follower Node

Refactored version of object_follower_experiment.py with:
- Modular architecture for maintainability
- Fixed scan timing bug (uses ROS time consistently)
- Fixed detection during scan (interrupts scan when object detected)
- Fixed waypoint distance validation (prevents "0 poses" error)
- Hybrid tracking with YOLO drift correction (from KCF-YOLO paper)

State Machine:
    IDLE -> EXPLORING -> APPROACHING -> TRACKING -> STOPPED
             ^                |            |
             |                v            v
             +----------SEARCHING<---------+

Key Fixes Applied:
1. Scan timing: Uses ROS time consistently (fixes 0.16s instead of 6s)
2. Detection during scan: Interrupts scan when object detected
3. Waypoint validation: Skips waypoints too close to robot
4. Tracker drift: YOLO periodic reinit corrects CSRT drift
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import math
import time
import csv
import os
import threading
from typing import Optional

# ROS messages
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from std_msgs.msg import String, ColorRGBA
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from yolov8_msgs.msg import Yolov8Inference

from cv_bridge import CvBridge
import cv2
import tf_transformations
import tf2_ros
from tf_transformations import euler_from_quaternion
from rclpy.time import Time

# Import modular components
from .config import (
    FOLLOW_DISTANCE,
    OBJECT_LOST_TIMEOUT,
    ROTATE_TIMEOUT,
    SEARCH_FORWARD_TIME,
    SEARCH_FORWARD_SPEED,
    ROTATION_SPEED,
    SEARCH_MAX_DURATION,
    SECTOR_HALF_WIDTH,
    USE_EXPLORATION,
    METRICS_CSV,
    TRACKING_HZ,
    CAM_WIDTH_DEFAULT,
    CAM_HEIGHT_DEFAULT,
    TRACKING_SWITCH_RATIO,
    APPROACH_SWITCH_RATIO,
    DETECTION_STOP_THRESHOLD,
    NEAR_GOAL_EPS,
    NAV_RESULT_COOLDOWN,
)
from .states import FollowerState
from .time_utils import TimeManager, get_ros_time_sec
from .trackers import HybridTracker, clamp_bbox
from .scan_handler import ScanHandler
from .nav2_handler import Nav2Handler
from .waypoint_manager import WaypointManager
from .visual_servo import VisualServoController
from .detection_processor import DetectionProcessor


class ObjectFollowerModular(Node):
    """
    Modular object follower node.
    
    Uses separate modules for:
    - Time management (fixes scan timing)
    - Scanning (at waypoints)
    - Nav2 navigation
    - Waypoint management
    - Visual servoing
    - Detection processing
    - Hybrid tracking
    """
    
    def __init__(self):
        super().__init__('object_follower_modular')
        
        # ==================== Initialize Modules ====================
        
        # Time manager - CRITICAL for consistent timing
        self._time = TimeManager(self)
        
        # Detection processor
        self._detector = DetectionProcessor(logger=self.get_logger())
        
        # Waypoint manager
        self.declare_parameter('use_random_search', False)
        use_random = self.get_parameter('use_random_search').get_parameter_value().bool_value
        self._waypoints = WaypointManager(logger=self.get_logger(), use_random=use_random)
        self.get_logger().info(
            f"Search Mode: {'RANDOM (Algo A)' if use_random else 'SYSTEMATIC (Algo B)'}"
        )
        
        # Nav2 handler
        self._nav2 = Nav2Handler(self, self._time, logger=self.get_logger())
        self._nav2.set_callbacks(
            on_reached=self._on_waypoint_reached,
            on_aborted=self._on_nav_aborted,
            on_canceled=self._on_nav_canceled,
        )
        
        # Scan handler - with ROS time (fixes timing bug!)
        self._scanner = ScanHandler(self._time, logger=self.get_logger())
        self._scanner.set_callbacks(
            on_complete=self._on_scan_complete,
        )
        
        # Visual servo controller
        self._servo = VisualServoController(logger=self.get_logger())
        self._servo.set_path_clear_callback(self._is_path_clear)
        
        # Hybrid tracker (with YOLO drift correction)
        self._tracker = HybridTracker(logger=self.get_logger())
        
        # ==================== Node State ====================
        
        self._state = FollowerState.IDLE
        self._lock = threading.Lock()
        
        # Tracking state
        self._tracking_mode = False
        self._tracking_lost_time: Optional[float] = None
        
        # Detection timing
        self._last_detection_time: Optional[float] = None
        
        # Searching state
        self._last_rotate_time: Optional[float] = None
        self._search_forward_start: Optional[float] = None
        self._search_forward_active = False
        self._search_start_time: Optional[float] = None
        
        # Camera image
        self._camera_image = None
        self._cv_bridge = CvBridge()
        
        # Laser scan
        self._latest_laser: Optional[LaserScan] = None
        
        # AMCL pose
        self._amcl_pose: Optional[PoseWithCovarianceStamped] = None
        self._home_pose = None
        
        # Exploration lock (prevents detection from interfering with waypoint nav)
        self._exploration_locked = False
        
        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._camera_frame = None
        
        # Metrics
        self._selection_counter = 0
        self._selection_start_time: Optional[float] = None
        self._first_detection_time: Optional[float] = None
        self._reached_time: Optional[float] = None
        
        # ==================== Publishers ====================
        
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._cmd_vel_tracking_pub = self.create_publisher(Twist, '/cmd_vel_tracking', 10)
        self._status_pub = self.create_publisher(String, '/object_follower_status', 10)
        self._annotated_pub = self.create_publisher(Image, '/camera/yolo_annotated', 1)
        self._waypoint_marker_pub = self.create_publisher(
            MarkerArray, '/object_follower/waypoints', 1
        )
        
        # ==================== QoS Profiles ====================
        
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # ==================== Subscriptions ====================
        
        self.create_subscription(
            Yolov8Inference, '/Yolov8_Inference', 
            self._detection_callback, 10
        )
        self.create_subscription(
            LaserScan, '/scan', 
            self._laser_callback, qos_sensor
        )
        self.create_subscription(
            String, '/selected_object_class', 
            self._selector_callback, 10
        )
        self.create_subscription(
            OccupancyGrid, '/map', 
            self._map_callback, qos_map
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', 
            self._amcl_pose_callback, 10
        )
        self.create_subscription(
            Image, '/camera/image_raw', 
            self._image_callback, qos_sensor
        )
        self.create_subscription(
            CameraInfo, '/camera/camera_info', 
            self._camera_info_callback, qos_sensor
        )
        
        # ==================== Timers ====================
        
        self._timer = self.create_timer(0.2, self._timer_callback)
        self._tracking_timer = self.create_timer(1.0 / TRACKING_HZ, self._tracking_loop)
        
        # ==================== Metrics CSV ====================
        
        self._init_metrics_csv()
        
        # ==================== Parameters ====================
        
        self.declare_parameter('return_home_after_success', False)
        self._return_home = self.get_parameter(
            'return_home_after_success'
        ).get_parameter_value().bool_value
        
        self.get_logger().info("Object Follower (Modular) started - with all fixes applied")
        
    # ==================== Callbacks (ROS) ====================
    
    def _map_callback(self, msg: OccupancyGrid):
        """Handle map updates."""
        if self._waypoints.has_map:
            return  # Already have map
            
        self.get_logger().info("Map received; generating waypoints.")
        self._waypoints.set_map(msg)
        self._publish_waypoint_markers()
        
    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Handle AMCL pose updates."""
        self._amcl_pose = msg
        if self._home_pose is None:
            self._home_pose = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                self._yaw_from_quat(msg.pose.pose.orientation)
            )
            
    def _camera_info_callback(self, msg: CameraInfo):
        """Handle camera info updates."""
        if msg and msg.header and msg.header.frame_id:
            self._camera_frame = msg.header.frame_id
            
    def _laser_callback(self, msg: LaserScan):
        """Handle laser scan updates."""
        self._latest_laser = msg
        
    def _selector_callback(self, msg: String):
        """Handle object selection from GUI."""
        sel = msg.data.strip() if msg.data else None
        
        if sel:
            # New selection
            self._selection_counter += 1
            self._selection_start_time = time.time()
            self._first_detection_time = None
            self._reached_time = None
            
            self._detector.set_selected_class(sel)
            self.get_logger().info(f"Selected object: {sel}")
            
            # Store home pose
            if self._amcl_pose and self._home_pose is None:
                self._home_pose = (
                    self._amcl_pose.pose.pose.position.x,
                    self._amcl_pose.pose.pose.position.y,
                    self._yaw_from_quat(self._amcl_pose.pose.pose.orientation)
                )
                
            # Start exploration or searching
            if USE_EXPLORATION and self._waypoints.has_map and self._amcl_pose:
                self._state = FollowerState.EXPLORING
                self._publish_status(f"Selected '{sel}'. Starting exploration...")
                self._goto_next_waypoint()
            else:
                self._state = FollowerState.SEARCHING
                self._last_rotate_time = self._time.now()
                self._search_start_time = self._time.now()
                self._publish_status(f"Selected '{sel}'. Searching...")
        else:
            # Stop requested
            self._publish_status("Stop requested by GUI")
            self._detector.set_selected_class(None)
            self._state = FollowerState.IDLE
            self._nav2.cancel_current_goal()
            self._stop_motion()
            
    def _detection_callback(self, msg: Yolov8Inference):
        """
        Handle YOLO detection.
        
        CRITICAL FIX: This now interrupts scanning when object detected!
        """
        if not self._detector.selected_class:
            return
            
        # Process detection
        detection = self._detector.process_inference(msg)
        
        if not detection:
            return
            
        # Scale to camera coordinates
        if self._camera_image is not None:
            cam_h, cam_w = self._camera_image.shape[:2]
        else:
            cam_w, cam_h = CAM_WIDTH_DEFAULT, CAM_HEIGHT_DEFAULT
            
        left, top, right, bottom = self._detector.scale_to_camera(
            detection, cam_w, cam_h
        )
        bbox_h = max(1, bottom - top)
        ratio = float(bbox_h) / float(cam_h)
        
        # Update timing
        self._last_detection_time = self._time.now()
        if self._first_detection_time is None:
            self._first_detection_time = self._last_detection_time
            
        # ==================== FIX: Interrupt scan on detection ====================
        if self._state == FollowerState.EXPLORING and self._scanner.is_active:
            if self._detector.should_interrupt_scan(detection):
                self.get_logger().info(
                    f"Object detected during scan (ratio={ratio:.3f}), interrupting!"
                )
                self._scanner.interrupt(reason="object detected")
                self._exploration_locked = False
                
                # Switch to TRACKING
                self._nav2.cancel_current_goal()
                self._state = FollowerState.TRACKING
                self._tracking_mode = True
                self._init_tracker_from_detection(left, top, right, bottom)
                return
                
        # ==================== State transitions based on ratio ====================
        
        # Update tracker in CENTROID mode with every detection
        if (self._state == FollowerState.TRACKING and 
            self._tracker.is_initialized):
            x, y, w, h = left, top, right - left, bottom - top
            self._tracker.reinit_from_detection(self._camera_image, (x, y, w, h))
            
        # STOPPED condition
        if ratio >= DETECTION_STOP_THRESHOLD:
            if self._state in [FollowerState.TRACKING, FollowerState.STOPPED]:
                self._state = FollowerState.STOPPED
                self._publish_status("Object close enough. Stopping.")
                self._nav2.cancel_current_goal()
                self._stop_motion()
                self._record_metrics(success=True)
                return
                
        # TRACKING condition (close)
        if ratio >= TRACKING_SWITCH_RATIO:
            if self._state in [FollowerState.SEARCHING, FollowerState.EXPLORING, 
                              FollowerState.APPROACHING]:
                self.get_logger().info(f"Object close (ratio={ratio:.3f}), switching to TRACKING")
                self._nav2.cancel_current_goal()
                self._exploration_locked = False
                self._state = FollowerState.TRACKING
                self._tracking_mode = True
                self._init_tracker_from_detection(left, top, right, bottom)
                return
                
        # APPROACHING condition (far but visible)
        if ratio >= APPROACH_SWITCH_RATIO:
            if self._state in [FollowerState.EXPLORING, FollowerState.SEARCHING]:
                self.get_logger().info(
                    f"Object detected (ratio={ratio:.3f}), switching to APPROACHING"
                )
                self._nav2.cancel_current_goal()
                self._exploration_locked = False
                self._state = FollowerState.APPROACHING
                self._send_nav_goal_to_object()
                return
                
            # Update approach goal periodically
            if self._state == FollowerState.APPROACHING:
                # Rate limited by Nav2Handler
                self._send_nav_goal_to_object()
                return
                
    def _image_callback(self, msg: Image):
        """Handle camera image and publish annotated version."""
        try:
            cv_img = self._cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            self._camera_image = cv_img.copy()
        except Exception as e:
            self.get_logger().warn(f"CV Bridge error: {e}")
            return
            
        # Update detection processor with actual dimensions
        self._detector.set_image_dimensions(cv_img.shape[1], cv_img.shape[0])
        
        # Annotate and publish
        self._publish_annotated_image(cv_img, msg.header)
        
    # ==================== Nav2 Callbacks ====================
    
    def _on_waypoint_reached(self):
        """Called when Nav2 goal reached."""
        if self._state == FollowerState.EXPLORING:
            self.get_logger().info("Waypoint reached, starting scan")
            self._exploration_locked = False
            self._stop_motion()
            self._scanner.start()
            
    def _on_nav_aborted(self, distance: float):
        """Called when Nav2 goal aborted."""
        # Check if we're close enough to treat as reached
        if self._amcl_pose and self._waypoints.current_waypoint:
            wp = self._waypoints.current_waypoint
            dx = wp.x - self._amcl_pose.pose.pose.position.x
            dy = wp.y - self._amcl_pose.pose.pose.position.y
            d = math.hypot(dx, dy)
            
            if d < NEAR_GOAL_EPS:
                self.get_logger().info(f"Near waypoint (d={d:.3f}m), treating as reached")
                self._on_waypoint_reached()
                return
            else:
                self.get_logger().warn(f"Far from goal (d={d:.2f}m), skipping waypoint")
                self._waypoints.mark_waypoint_failed()
                self._exploration_locked = False
                
    def _on_nav_canceled(self):
        """Called when Nav2 goal canceled."""
        self._exploration_locked = False
        
    def _on_scan_complete(self):
        """Called when scan completes."""
        self.get_logger().info("Scan complete, moving to next waypoint")
        self._waypoints.advance_to_next()
        self._nav2.reset_rate_limiting()
        self._goto_next_waypoint()
        
    # ==================== Main Timer Callback ====================
    
    def _timer_callback(self):
        """Main state machine timer."""
        now = self._time.now()
        
        # Check Nav2 server availability
        if not self._nav2._ready:
            self._nav2.wait_for_server(timeout_sec=0.5)
            
        if self._state == FollowerState.IDLE:
            return
            
        # ==================== EXPLORING State ====================
        if self._state == FollowerState.EXPLORING:
            # Handle scanning
            if self._scanner.is_active:
                twist = self._scanner.update()
                if twist:
                    self._publish_twist(twist)
                # Scan complete handled by callback
                return
                
            # Auto-unlock after timeout
            if self._exploration_locked:
                # Use the time since we last sent a waypoint
                if not self._nav2.is_goal_active:
                    self.get_logger().debug("No active goal, unlocking exploration")
                    self._exploration_locked = False
                    
            # Get next waypoint if needed
            if not self._nav2.is_goal_active and not self._scanner.is_active:
                if self._waypoints.current_waypoint is None:
                    self._goto_next_waypoint()
            return
            
        # ==================== SEARCHING State ====================
        if self._state == FollowerState.SEARCHING:
            # Check if detected during search
            if (self._last_detection_time and 
                (now - self._last_detection_time) < OBJECT_LOST_TIMEOUT):
                self._state = FollowerState.TRACKING
                self._publish_status("Switch to TRACKING (detected during search)")
                return
                
            if not self._search_start_time:
                self._search_start_time = now
                
            # Timeout -> return to exploring
            if (now - self._search_start_time) >= SEARCH_MAX_DURATION:
                if USE_EXPLORATION and self._waypoints.has_map:
                    self._state = FollowerState.EXPLORING
                    self._publish_status("Search timed out -> resuming exploration")
                    self._goto_next_waypoint()
                    return
                else:
                    self._search_start_time = now
                    
            # Rotating
            if not self._last_rotate_time:
                self._last_rotate_time = now
                
            if (now - self._last_rotate_time) < ROTATE_TIMEOUT:
                self._publish_status("Searching: rotating")
                self._rotate_in_place()
                return
                
            # Forward step
            if not self._search_forward_active:
                self._search_forward_active = True
                self._search_forward_start = now
                self._publish_status("Searching: forward step")
                self._publish_linear(SEARCH_FORWARD_SPEED)
                return
            else:
                if (now - self._search_forward_start) < SEARCH_FORWARD_TIME:
                    self._publish_linear(SEARCH_FORWARD_SPEED)
                    return
                else:
                    self._search_forward_active = False
                    self._last_rotate_time = now
                    self._stop_motion()
                    return
                    
        # ==================== APPROACHING State ====================
        if self._state == FollowerState.APPROACHING:
            # Check for lost object
            if (self._last_detection_time and 
                (now - self._last_detection_time) > 3.0):
                self.get_logger().info("Object lost during APPROACHING -> SEARCHING")
                self._state = FollowerState.SEARCHING
                self._nav2.cancel_current_goal()
                self._start_searching()
            return
            
        # ==================== TRACKING State ====================
        if self._state == FollowerState.TRACKING:
            # Check if detection lost
            if (not self._last_detection_time or 
                (now - self._last_detection_time) >= OBJECT_LOST_TIMEOUT):
                if not self._tracking_mode:
                    self._start_searching()
                    return
                    
            # Centering control (quick adjustments)
            if self._detector.current_detection:
                det = self._detector.current_detection
                if self._camera_image is not None:
                    cam_h, cam_w = self._camera_image.shape[:2]
                else:
                    cam_w, cam_h = CAM_WIDTH_DEFAULT, CAM_HEIGHT_DEFAULT
                    
                left, top, right, bottom = self._detector.scale_to_camera(
                    det, cam_w, cam_h
                )
                bbox = (left, top, right - left, bottom - top)
                
                centering_twist = self._servo.compute_centering_only(bbox, cam_w)
                if centering_twist:
                    self._publish_twist(centering_twist)
                    return
                    
            # Lidar safety check
            if not self._is_path_clear():
                self._publish_status("Path blocked. Stopping.")
                self._nav2.cancel_current_goal()
                self._stop_motion()
                return
                
            # Lidar stop condition
            if self._check_lidar_close():
                self._state = FollowerState.STOPPED
                self._reached_time = time.time()
                self._publish_status("At safe distance (lidar). Stopping.")
                self._stop_motion()
                self._record_metrics(success=True)
            return
            
        # ==================== STOPPED State ====================
        if self._state == FollowerState.STOPPED:
            # Check if object lost
            if (not self._last_detection_time or 
                (now - self._last_detection_time) >= OBJECT_LOST_TIMEOUT):
                self._start_searching()
            else:
                self._stop_motion()
                
    # ==================== Tracking Loop ====================
    
    def _tracking_loop(self):
        """High-frequency tracking loop."""
        if not self._detector.selected_class:
            return
        if self._state != FollowerState.TRACKING:
            return
            
        bbox = None
        
        # Get bbox from tracker
        if self._tracker.is_initialized and self._camera_image is not None:
            # Get YOLO bbox for drift correction
            yolo_bbox = None
            if self._detector.current_detection:
                det = self._detector.current_detection
                if self._camera_image is not None:
                    cam_h, cam_w = self._camera_image.shape[:2]
                else:
                    cam_w, cam_h = CAM_WIDTH_DEFAULT, CAM_HEIGHT_DEFAULT
                left, top, right, bottom = self._detector.scale_to_camera(
                    det, cam_w, cam_h
                )
                yolo_bbox = (left, top, right - left, bottom - top)
                
            # Update tracker with YOLO drift correction!
            ok, b = self._tracker.update(self._camera_image, yolo_bbox)
            
            if ok:
                bbox = b
                self._tracking_lost_time = None
            else:
                if self._tracking_lost_time is None:
                    self._tracking_lost_time = self._time.now()
                elif self._time.elapsed_since(self._tracking_lost_time) > 1.5:
                    self.get_logger().info("Tracker lost -> SEARCHING")
                    self._tracking_mode = False
                    self._tracker.reset()
                    self._start_searching()
                    return
        else:
            # Fallback to detection
            if self._detector.current_detection:
                det = self._detector.current_detection
                if self._camera_image is not None:
                    cam_h, cam_w = self._camera_image.shape[:2]
                else:
                    cam_w, cam_h = CAM_WIDTH_DEFAULT, CAM_HEIGHT_DEFAULT
                left, top, right, bottom = self._detector.scale_to_camera(
                    det, cam_w, cam_h
                )
                bbox = (left, top, right - left, bottom - top)
            else:
                self.get_logger().info("No tracker and no detection -> SEARCHING")
                self._tracking_mode = False
                self._start_searching()
                return
                
        # Compute control
        if bbox:
            if self._camera_image is not None:
                cam_h, cam_w = self._camera_image.shape[:2]
            else:
                cam_w, cam_h = CAM_WIDTH_DEFAULT, CAM_HEIGHT_DEFAULT
                
            twist, is_close = self._servo.compute_control(bbox, cam_w, cam_h)
            self._publish_twist(twist)
            
            if is_close:
                self.get_logger().info("Close enough (visual). Stopping.")
                self._stop_motion()
                self._tracking_mode = False
                self._tracker.reset()
                self._record_metrics(success=True)
                self._state = FollowerState.STOPPED
                
    # ==================== Helper Methods ====================
    
    def _goto_next_waypoint(self):
        """Navigate to next waypoint."""
        if not self._waypoints.has_map:
            self._publish_status("No map yet")
            return
            
        # Get robot position
        robot_x, robot_y = 0.0, 0.0
        if self._amcl_pose:
            robot_x = self._amcl_pose.pose.pose.position.x
            robot_y = self._amcl_pose.pose.pose.position.y
            
        wp = self._waypoints.get_next_waypoint(robot_x, robot_y)
        
        if not wp:
            self._publish_status("No valid waypoint available")
            return
            
        self._publish_status(f"Navigating to waypoint ({wp.x:.2f},{wp.y:.2f})")
        
        # Lock exploration
        self._exploration_locked = True
        
        sent = self._nav2.send_goal(wp.x, wp.y, wp.yaw)
        if not sent:
            self._publish_status("Failed to send waypoint to Nav2")
            self._exploration_locked = False
            
        self._publish_waypoint_markers()
        
    def _send_nav_goal_to_object(self):
        """Send Nav2 goal towards detected object."""
        # Get camera transform
        tf_msg = self._get_camera_transform()
        if not tf_msg:
            return
            
        tx = tf_msg.transform.translation.x
        ty = tf_msg.transform.translation.y
        q = tf_msg.transform.rotation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        
        # Goal in front of camera
        goal_x = float(tx + FOLLOW_DISTANCE * math.cos(yaw))
        goal_y = float(ty + FOLLOW_DISTANCE * math.sin(yaw))
        
        self._nav2.send_goal(goal_x, goal_y, yaw)
        
    def _get_camera_transform(self):
        """Get camera to map transform."""
        frames = []
        if self._camera_frame:
            frames.append(self._camera_frame)
        frames.extend(['camera_link_optical', 'camera_link'])
        
        for frame in frames:
            try:
                if self._tf_buffer.can_transform('map', frame, Time()):
                    return self._tf_buffer.lookup_transform('map', frame, Time())
            except Exception:
                pass
        return None
        
    def _init_tracker_from_detection(self, left, top, right, bottom):
        """Initialize tracker from detection coordinates."""
        if self._camera_image is None:
            return
        x, y, w, h = left, top, right - left, bottom - top
        self._tracker.init(self._camera_image, (x, y, w, h))
        
    def _start_searching(self):
        """Enter SEARCHING state."""
        self._nav2.cancel_current_goal()
        self._exploration_locked = False
        
        self._state = FollowerState.SEARCHING
        self._search_start_time = self._time.now()
        self._last_rotate_time = self._time.now()
        self._search_forward_active = False
        
        self._publish_status("Entering SEARCHING state")
        
    def _is_path_clear(self) -> bool:
        """Check if path ahead is clear using lidar."""
        if not self._latest_laser or not self._latest_laser.ranges:
            return True
            
        ranges = self._latest_laser.ranges
        n = len(ranges)
        c = n // 2
        left = max(0, c - SECTOR_HALF_WIDTH)
        right = min(n, c + SECTOR_HALF_WIDTH + 1)
        
        for r in ranges[left:right]:
            if r and not math.isinf(r) and not math.isnan(r):
                if r <= FOLLOW_DISTANCE:
                    return False
        return True
        
    def _check_lidar_close(self) -> bool:
        """Check if something is close in front using lidar."""
        if not self._latest_laser:
            return False
            
        center_idx = len(self._latest_laser.ranges) // 2
        try:
            r = self._latest_laser.ranges[center_idx]
            if r and not math.isinf(r) and not math.isnan(r):
                return r <= FOLLOW_DISTANCE
        except Exception:
            pass
        return False
        
    # ==================== Motion Control ====================
    
    def _publish_twist(self, twist: Twist):
        """Publish twist to appropriate topic."""
        try:
            if self._cmd_vel_tracking_pub.get_subscription_count() > 0:
                self._cmd_vel_tracking_pub.publish(twist)
            else:
                self._cmd_vel_pub.publish(twist)
        except Exception:
            self._cmd_vel_pub.publish(twist)
            
    def _stop_motion(self):
        """Stop all motion."""
        twist = Twist()
        for _ in range(3):
            self._publish_twist(twist)
            
    def _rotate_in_place(self):
        """Rotate in place."""
        twist = Twist()
        twist.angular.z = ROTATION_SPEED
        self._publish_twist(twist)
        
    def _publish_linear(self, speed: float):
        """Publish linear velocity."""
        twist = Twist()
        twist.linear.x = speed
        self._publish_twist(twist)
        
    # ==================== Publishing ====================
    
    def _publish_status(self, text: str):
        """Publish status message."""
        msg = String()
        msg.data = f"[{self._state.value}] {text}"
        self._status_pub.publish(msg)
        self.get_logger().info(text)
        
    def _publish_annotated_image(self, img, header):
        """Publish annotated image with detection/tracking boxes."""
        disp = img.copy()
        
        # Draw YOLO detection (green)
        det = self._detector.current_detection
        if det:
            cam_h, cam_w = img.shape[:2]
            left, top, right, bottom = self._detector.scale_to_camera(
                det, cam_w, cam_h
            )
            cv2.rectangle(disp, (left, top), (right, bottom), (0, 255, 0), 2)
            cv2.putText(disp, det.class_name, (left, max(10, top - 6)),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                       
        # Draw tracker bbox (red)
        if self._tracking_mode and self._tracker.is_initialized:
            try:
                ok, bbox = self._tracker.update(img)
                if ok:
                    x, y, w, h = [int(v) for v in bbox]
                    cv2.rectangle(disp, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    cv2.putText(disp, "TRACKER", (x, max(10, y - 6)),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            except Exception:
                pass
                
        # Publish
        try:
            out_msg = self._cv_bridge.cv2_to_imgmsg(disp, 'bgr8')
            out_msg.header = header
            self._annotated_pub.publish(out_msg)
        except Exception:
            pass
            
    def _publish_waypoint_markers(self):
        """Publish waypoint markers for visualization."""
        ma = MarkerArray()
        
        # All waypoints (green cubes)
        for i, wp in enumerate(self._waypoints.waypoints):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'object_follower_waypoints'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = wp.x
            m.pose.position.y = wp.y
            m.pose.position.z = 0.05
            q = tf_transformations.quaternion_from_euler(0, 0, wp.yaw)
            m.pose.orientation.x = q[0]
            m.pose.orientation.y = q[1]
            m.pose.orientation.z = q[2]
            m.pose.orientation.w = q[3]
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.02
            m.color = ColorRGBA(r=0.0, g=0.8, b=0.1, a=0.9)
            ma.markers.append(m)
            
        # Current waypoint (red sphere)
        if self._waypoints.current_waypoint:
            wp = self._waypoints.current_waypoint
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'object_follower_current_wp'
            m.id = 999
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wp.x
            m.pose.position.y = wp.y
            m.pose.position.z = 0.1
            m.scale.x = 0.25
            m.scale.y = 0.25
            m.scale.z = 0.25
            m.color = ColorRGBA(r=0.9, g=0.2, b=0.2, a=0.9)
            ma.markers.append(m)
            
        self._waypoint_marker_pub.publish(ma)
        
    # ==================== Metrics ====================
    
    def _init_metrics_csv(self):
        """Initialize metrics CSV file."""
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
                
    def _record_metrics(self, success: bool):
        """Record metrics to CSV."""
        try:
            with open(METRICS_CSV, 'a', newline='') as f:
                writer = csv.writer(f)
                detect_t = self._first_detection_time
                reach_t = self._reached_time or (time.time() if success else None)
                detection_latency = (
                    (detect_t - self._selection_start_time) 
                    if (detect_t and self._selection_start_time) else None
                )
                time_to_reach = (reach_t - detect_t) if (reach_t and detect_t) else None
                
                sent, canceled = self._nav2.metrics
                
                writer.writerow([
                    self._selection_counter,
                    self._detector.selected_class,
                    f"{self._selection_start_time:.3f}" if self._selection_start_time else "",
                    f"{detect_t:.3f}" if detect_t else "",
                    f"{detection_latency:.3f}" if detection_latency else "",
                    f"{reach_t:.3f}" if reach_t else "",
                    f"{time_to_reach:.3f}" if time_to_reach else "",
                    sent,
                    canceled,
                    "success" if success else "failure"
                ])
        except Exception as e:
            self.get_logger().warning(f"Failed to write metrics: {e}")
            
    # ==================== Utilities ====================
    
    def _yaw_from_quat(self, q):
        """Extract yaw from quaternion."""
        return tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def main(args=None):
    rclpy.init(args=args)
    node = ObjectFollowerModular()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_motion()
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
