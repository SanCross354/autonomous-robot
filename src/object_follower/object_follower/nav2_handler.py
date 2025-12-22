#!/usr/bin/env python3
"""
nav2_handler.py - Nav2 action client handler for Object Follower

Handles:
- Goal sending with rate limiting
- Goal cancellation
- Result processing with abort handling
- Distance validation to prevent "0 poses" errors
"""

import math
import threading
from typing import Optional, Tuple, Callable
from dataclasses import dataclass, field
import logging

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
import tf_transformations

from .config import (
    MIN_GOAL_UPDATE,
    MIN_GOAL_PERIOD,
    NAV_RESULT_COOLDOWN,
    NEAR_GOAL_EPS,
    MIN_WAYPOINT_DISTANCE,
)
from .time_utils import TimeManager


@dataclass
class Nav2State:
    """Holds Nav2 navigation state."""
    goal_sent: bool = False
    goal_handle: Optional[object] = None
    last_nav_goal: Optional[Tuple[float, float, float]] = None  # (x, y, yaw)
    last_goal_time: float = 0.0
    last_nav_result_time: float = 0.0
    nav_cancel_pending_until: float = 0.0
    
    # Metrics
    goals_sent_count: int = 0
    goals_canceled_count: int = 0


class Nav2Handler:
    """
    Handles Nav2 action client operations.
    
    Key features:
    - Rate limiting for goal sending
    - Proper abort handling (treats near-goal aborts as success)
    - Thread-safe cancellation
    - Distance validation to prevent "0 poses" errors
    
    Usage:
        handler = Nav2Handler(node, time_manager)
        handler.set_callbacks(on_reached=..., on_aborted=...)
        handler.send_goal(x, y, yaw)
    """
    
    def __init__(self, node: Node, time_manager: TimeManager,
                 logger: Optional[logging.Logger] = None):
        """
        Initialize Nav2 handler.
        
        Args:
            node: ROS2 node
            time_manager: TimeManager for consistent timing
            logger: Optional logger
        """
        self._node = node
        self._time = time_manager
        self._logger = logger or node.get_logger()
        self._state = Nav2State()
        self._lock = threading.Lock()
        
        # Callbacks
        self._on_reached: Optional[Callable[[], None]] = None
        self._on_aborted: Optional[Callable[[float], None]] = None  # (distance)
        self._on_canceled: Optional[Callable[[], None]] = None
        
        # Create action client
        self._action_client: Optional[ActionClient] = None
        self._ready = False
        
        try:
            self._action_client = ActionClient(
                node, NavigateToPose, 'navigate_to_pose'
            )
        except Exception as e:
            self._logger.warn(f"Nav2 action client init error: {e}")
            
    def set_callbacks(self,
                      on_reached: Optional[Callable[[], None]] = None,
                      on_aborted: Optional[Callable[[float], None]] = None,
                      on_canceled: Optional[Callable[[], None]] = None):
        """
        Set callbacks for navigation events.
        
        Args:
            on_reached: Called when goal reached successfully
            on_aborted: Called when goal aborted (with distance to goal)
            on_canceled: Called when goal canceled
        """
        self._on_reached = on_reached
        self._on_aborted = on_aborted
        self._on_canceled = on_canceled
        
    def wait_for_server(self, timeout_sec: float = 2.0) -> bool:
        """
        Wait for Nav2 action server.
        
        Args:
            timeout_sec: Timeout in seconds
            
        Returns:
            True if server is available
        """
        if self._action_client is None:
            return False
            
        self._ready = self._action_client.wait_for_server(timeout_sec=timeout_sec)
        if self._ready:
            self._logger.info("Nav2 action server is available")
        return self._ready
        
    def should_send_goal(self, x: float, y: float) -> Tuple[bool, str]:
        """
        Check if we should send a new goal (rate limiting).
        
        Args:
            x, y: Target position
            
        Returns:
            (should_send, reason) tuple
        """
        now = self._time.now()
        
        # Check time since last goal
        if (now - self._state.last_goal_time) < MIN_GOAL_PERIOD:
            return False, f"MIN_GOAL_PERIOD ({now - self._state.last_goal_time:.2f}s < {MIN_GOAL_PERIOD}s)"
            
        # Check cooldown after result
        if (now - self._state.last_nav_result_time) < NAV_RESULT_COOLDOWN:
            return False, f"NAV_RESULT_COOLDOWN ({now - self._state.last_nav_result_time:.2f}s < {NAV_RESULT_COOLDOWN}s)"
            
        # Check cancel pending
        if now < self._state.nav_cancel_pending_until:
            return False, "cancel pending"
            
        # Check distance to last goal
        if self._state.goal_sent and self._state.last_nav_goal:
            dx = x - self._state.last_nav_goal[0]
            dy = y - self._state.last_nav_goal[1]
            dist = math.hypot(dx, dy)
            if dist <= MIN_GOAL_UPDATE:
                return False, f"MIN_GOAL_UPDATE (dist={dist:.3f}m <= {MIN_GOAL_UPDATE}m)"
                
        return True, "ok"
        
    def validate_waypoint_distance(self, x: float, y: float,
                                    robot_x: float, robot_y: float) -> Tuple[bool, float]:
        """
        Validate waypoint is not too close to robot.
        
        This prevents the "0 poses" Nav2 error.
        
        Args:
            x, y: Waypoint position
            robot_x, robot_y: Robot position
            
        Returns:
            (is_valid, distance) tuple
        """
        dist = math.hypot(x - robot_x, y - robot_y)
        is_valid = dist >= MIN_WAYPOINT_DISTANCE
        return is_valid, dist
        
    def send_goal(self, x: float, y: float, yaw: float = 0.0) -> bool:
        """
        Send navigation goal to Nav2.
        
        Args:
            x, y: Target position in map frame
            yaw: Target orientation
            
        Returns:
            True if goal was sent successfully
        """
        if self._action_client is None:
            self._logger.warn("Nav2 action client not present")
            return False
            
        if not self._ready:
            if not self.wait_for_server(timeout_sec=2.0):
                self._logger.warning("Nav2 action server not available")
                return False
                
        # Check rate limiting
        should_send, reason = self.should_send_goal(x, y)
        if not should_send:
            self._logger.debug(f"Rate-limited: not sending goal to ({x:.2f},{y:.2f}). Reason: {reason}")
            return False
            
        # Create goal
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation.x = float(q[0])
        goal.pose.orientation.y = float(q[1])
        goal.pose.orientation.z = float(q[2])
        goal.pose.orientation.w = float(q[3])
        
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal
        
        # Send goal
        try:
            send_future = self._action_client.send_goal_async(nav_goal)
            send_future.add_done_callback(self._goal_response_callback)
            
            self._state.last_nav_goal = (x, y, yaw)
            self._state.last_goal_time = self._time.now()
            self._state.goals_sent_count += 1
            
            self._logger.info(
                f"[NAV2 SEND] Goal #{self._state.goals_sent_count} to ({x:.2f},{y:.2f})"
            )
            return True
        except Exception as e:
            self._logger.error(f"Failed to send nav goal: {e}")
            return False
            
    def cancel_current_goal(self):
        """Cancel the current navigation goal."""
        with self._lock:
            if self._state.goal_handle:
                try:
                    self._logger.info("[NAV2 CANCEL] Requesting cancellation")
                    cancel_future = self._state.goal_handle.cancel_goal_async()
                    
                    def _done(fut):
                        try:
                            fut.result()
                            self._logger.info("[NAV2 CANCEL] Cancellation processed")
                        except Exception as e:
                            self._logger.warning(f"[NAV2 CANCEL] Error: {e}")
                            
                    cancel_future.add_done_callback(_done)
                    self._state.goals_canceled_count += 1
                except Exception as e:
                    self._logger.warning(f"Failed to cancel goal: {e}")
                    
            self._state.goal_sent = False
            self._state.goal_handle = None
            self._state.last_nav_goal = None
            self._state.nav_cancel_pending_until = self._time.now() + 0.15
            
    def _goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        try:
            goal_handle = future.result()
            self._state.goal_handle = goal_handle
            
            if not goal_handle.accepted:
                self._logger.info("[NAV2] Goal REJECTED by server")
                self._state.goal_sent = False
                self._state.last_nav_goal = None
                return
                
            self._state.goal_sent = True
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._goal_result_callback)
            self._logger.info("[NAV2] Goal ACCEPTED by server")
            
        except Exception as e:
            self._logger.error(f"goal_response_callback exception: {e}")
            self._state.goal_sent = False
            
    def _goal_result_callback(self, future):
        """Handle goal result."""
        try:
            res = future.result()
            status = getattr(res, 'status', None)
            if status is None:
                try:
                    status = res.result.status
                except Exception:
                    status = None
                    
            self._state.last_nav_result_time = self._time.now()
            
            status_str = {
                GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                GoalStatus.STATUS_ABORTED: "ABORTED",
                GoalStatus.STATUS_CANCELED: "CANCELED",
            }.get(status, f"UNKNOWN({status})")
            
            self._logger.info(f"[NAV2 RESULT] Status: {status_str}")
            
            if status == GoalStatus.STATUS_SUCCEEDED:
                # Clear state
                self._state.goal_sent = False
                self._state.goal_handle = None
                self._state.last_nav_goal = None
                
                if self._on_reached:
                    self._on_reached()
                    
            elif status == GoalStatus.STATUS_ABORTED:
                # Clear state
                self._state.goal_sent = False
                self._state.goal_handle = None
                self._state.last_nav_goal = None
                
                if self._on_aborted:
                    self._on_aborted(0.0)  # Distance would need robot pose
                    
            elif status == GoalStatus.STATUS_CANCELED:
                self._state.goal_sent = False
                self._state.goal_handle = None
                self._state.last_nav_goal = None
                
                if self._on_canceled:
                    self._on_canceled()
                    
        except Exception as e:
            self._logger.warning(f"Nav2 goal result callback error: {e}")
        finally:
            if self._state.goal_sent:
                self._state.goal_sent = False
                self._state.goal_handle = None
                self._state.last_nav_goal = None
                
    @property
    def is_goal_active(self) -> bool:
        """Check if a goal is currently active."""
        return self._state.goal_sent
        
    @property
    def last_goal(self) -> Optional[Tuple[float, float, float]]:
        """Get last sent goal (x, y, yaw)."""
        return self._state.last_nav_goal
        
    @property
    def metrics(self) -> Tuple[int, int]:
        """Get (goals_sent, goals_canceled) counts."""
        return (self._state.goals_sent_count, self._state.goals_canceled_count)
        
    def reset_rate_limiting(self):
        """Reset rate limiting to allow immediate goal send."""
        now = self._time.now()
        self._state.last_goal_time = 0.0
        self._state.last_nav_result_time = now - NAV_RESULT_COOLDOWN - 1.0
        self._state.nav_cancel_pending_until = 0.0
