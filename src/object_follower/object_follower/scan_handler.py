#!/usr/bin/env python3
"""
scan_handler.py - Waypoint scanning handler for Object Follower

CRITICAL FIX: Uses ROS time consistently to fix the scan timing bug.

The original bug: Scan completed in 0.16 seconds instead of 6 seconds
because of mixing time.time() (wall clock) with ROS simulation time.

This module handles:
- 360° rotation scanning at waypoints
- Detection interruption during scan (from KCF-YOLO paper insights)
"""

from dataclasses import dataclass
from typing import Optional, Callable
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import logging
import math
import numpy as np

from .config import SCAN_DURATION, SCAN_SPEED
from .time_utils import TimeManager


@dataclass
class ScanState:
    """Holds the current state of a scan operation."""
    active: bool = False
    start_time: Optional[float] = None
    duration: float = SCAN_DURATION
    speed: float = SCAN_SPEED
    last_log_second: int = -1


class ScanHandler:
    """
    Handles 360° scanning at waypoints.
    
    Key features:
    - Uses ROS time consistently (fixes timing bug)
    - Can be interrupted by detection
    - Reports progress via callbacks
    
    Usage:
        scan_handler = ScanHandler(time_manager, logger)
        scan_handler.start()
        
        # In timer callback:
        if scan_handler.is_active:
            twist = scan_handler.update()
            if twist is None:
                # Scan complete
                pass
            else:
                # Publish twist
                cmd_vel_pub.publish(twist)
    """
    
    def __init__(self, time_manager: TimeManager, 
                 logger: Optional[logging.Logger] = None,
                 duration: float = SCAN_DURATION,
                 speed: float = SCAN_SPEED):
        """
        Initialize scan handler.
        
        Args:
            time_manager: TimeManager for consistent ROS time
            logger: Optional logger
            duration: Scan duration in seconds
            speed: Angular velocity in rad/s
        """
        self._time = time_manager
        self._logger = logger or logging.getLogger(__name__)
        self._state = ScanState(duration=duration, speed=speed)
        self._on_complete: Optional[Callable[[], None]] = None
        self._on_progress: Optional[Callable[[float, float], None]] = None
        
    def set_callbacks(self, 
                      on_complete: Optional[Callable[[], None]] = None,
                      on_progress: Optional[Callable[[float, float], None]] = None):
        """
        Set optional callbacks.
        
        Args:
            on_complete: Called when scan completes (no args)
            on_progress: Called with (elapsed, duration) during scan
        """
        self._on_complete = on_complete
        self._on_progress = on_progress
        
    def start(self) -> bool:
        """
        Start a new scan.
        
        Returns:
            True if scan started successfully
        """
        if self._state.active:
            self._logger.warning("[SCAN] Already scanning, ignoring start request")
            return False
            
        # Use ROS time consistently (CRITICAL FIX!)
        self._state.start_time = self._time.now()
        self._state.active = True
        self._state.last_log_second = -1
        
        self._logger.info(
            f"[SCAN] Starting 360° rotation (will take {self._state.duration}s)"
        )
        
        return True
        
    def stop(self) -> Twist:
        """
        Stop scanning and return zero twist.
        
        Returns:
            Zero velocity Twist message
        """
        self._state.active = False
        self._state.start_time = None
        self._state.last_log_second = -1
        
        return self._create_stop_twist()
        
    def interrupt(self, reason: str = "detection") -> Twist:
        """
        Interrupt scan (e.g., when object detected during scan).
        
        Args:
            reason: Reason for interruption (for logging)
            
        Returns:
            Zero velocity Twist message
        """
        if self._state.active:
            self._logger.info(f"[SCAN] Interrupted due to {reason}")
            
        return self.stop()
        
    def update(self) -> Optional[Twist]:
        """
        Update scan state and get command.
        
        Call this in your timer callback while scanning.
        
        Returns:
            Twist message for rotation, or None if scan complete
        """
        if not self._state.active:
            return None
            
        if self._state.start_time is None:
            self._logger.warning("[SCAN] Active but no start time, resetting")
            self._state.active = False
            return None
            
        # Calculate elapsed time using ROS time (CRITICAL FIX!)
        elapsed = self._time.elapsed_since(self._state.start_time)
        
        if elapsed < self._state.duration:
            # Still scanning - send rotation command
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = self._state.speed
            
            # Log progress every second
            current_second = int(elapsed)
            if current_second != self._state.last_log_second:
                self._state.last_log_second = current_second
                self._logger.info(
                    f"[SCAN] Progress: {elapsed:.1f}/{self._state.duration:.1f}s"
                )
                
                # Progress callback
                if self._on_progress:
                    self._on_progress(elapsed, self._state.duration)
                    
            return twist
        else:
            # Scan complete
            self._logger.info(
                f"[SCAN] Completed 360° rotation (actual: {elapsed:.1f}s)"
            )
            self._state.active = False
            self._state.start_time = None
            self._state.last_log_second = -1
            
            # Complete callback
            if self._on_complete:
                self._on_complete()
                
            return None
            
    @property
    def is_active(self) -> bool:
        """Check if scan is currently active."""
        return self._state.active
        
    @property
    def elapsed(self) -> float:
        """Get elapsed scan time in seconds."""
        if not self._state.active or self._state.start_time is None:
            return 0.0
        return self._time.elapsed_since(self._state.start_time)
        
    @property
    def progress(self) -> float:
        """Get scan progress as fraction (0.0 to 1.0)."""
        if not self._state.active:
            return 0.0
        return min(1.0, self.elapsed / self._state.duration)
        
    def _create_stop_twist(self) -> Twist:
        """Create a zero velocity twist message."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        return twist

    def check_safety(self, scan_msg: LaserScan, 
                     min_dist: float = 0.6, 
                     angle_range_deg: float = 60.0) -> bool:
        """
        Check if path ahead is clear.
        
        Args:
            scan_msg: LaserScan message
            min_dist: Minimum safe distance in meters
            angle_range_deg: Check sector width centered at 0 (e.g. 60 = +/- 30 deg)
            
        Returns:
            True if UNSAFE (obstacle detected), False if clear
        """
        if scan_msg is None:
            return False # Assume safe if no data
            
        # Convert range to radians
        half_angle_rad = math.radians(angle_range_deg / 2.0)
        
        # Ranges are usually CCW from X-axis. 
        # 0 is Forward. But scan might start at -Pi.
        # Simplest way: iterate and check angles.
        
        angle_min = scan_msg.angle_min
        angle_inc = scan_msg.angle_increment
        
        # Optimization: Only check indices in range
        # Target: [0 - half_angle, 0 + half_angle]
        # But indices wrap around if 360 scan.
        # Assuming standard planar lidar where 0 is forward index or middle.
        
        # Robust method: Convert all meaningful points
        ranges = np.array(scan_msg.ranges)
        
        # Filter invalid points (inf, nan)
        valid_mask = np.isfinite(ranges) & (ranges > scan_msg.range_min)
        
        # We need angles for each point
        num_points = len(ranges)
        angles = angle_min + np.arange(num_points) * angle_inc
        
        # Normalize angles to [-pi, pi]
        angles = (angles + np.pi) % (2 * np.pi) - np.pi
        
        # Check sector: abs(angle) < half_angle
        sector_mask = np.abs(angles) < half_angle_rad
        
        # Check safety check
        # Points in sector AND smaller than min_dist
        unsafe_mask = valid_mask & sector_mask & (ranges < min_dist)
        
        return np.any(unsafe_mask)
