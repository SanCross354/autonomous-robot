#!/usr/bin/env python3
"""
visual_servo.py - Visual servoing controller for Object Follower

Implements the visual servoing control law for tracking objects.
Uses proportional control with EMA smoothing for stable tracking.
"""

import math
from typing import Optional, Tuple
from dataclasses import dataclass
import logging

from geometry_msgs.msg import Twist

from .config import (
    Kp_ang,
    MAX_LIN,
    MAX_ANG,
    DETECTION_STOP_THRESHOLD,
    CAM_WIDTH_DEFAULT,
    CAM_HEIGHT_DEFAULT,
)


@dataclass
class VisualServoState:
    """Holds visual servoing state for smoothing."""
    cx_ema: Optional[float] = None  # EMA of center x error
    ratio_ema: Optional[float] = None  # EMA of bbox height ratio
    
    
class VisualServoController:
    """
    Visual servoing controller for object tracking.
    
    Uses pixel error from image center to control angular velocity,
    and bbox height ratio to control linear velocity.
    
    Features:
    - EMA smoothing for stable control
    - Exponential approach for smooth deceleration
    - Obstacle checking integration
    """
    
    def __init__(self, logger: Optional[logging.Logger] = None,
                 target_ratio: float = 0.32,
                 ema_alpha: float = 0.35):
        """
        Initialize visual servo controller.
        
        Args:
            logger: Optional logger
            target_ratio: Target bbox height ratio (when to stop approaching)
            ema_alpha: EMA smoothing factor (0-1, higher = less smoothing)
        """
        self._logger = logger or logging.getLogger(__name__)
        self._target_ratio = target_ratio
        self._ema_alpha = ema_alpha
        self._state = VisualServoState()
        
        # Path clear callback (optional)
        self._is_path_clear_fn: Optional[callable] = None
        
    def set_path_clear_callback(self, callback: callable):
        """
        Set callback to check if path is clear.
        
        Args:
            callback: Function that returns True if path is clear
        """
        self._is_path_clear_fn = callback
        
    def compute_control(self, bbox: Tuple[int, int, int, int],
                        image_width: int = CAM_WIDTH_DEFAULT,
                        image_height: int = CAM_HEIGHT_DEFAULT) -> Tuple[Twist, bool]:
        """
        Compute control command from bounding box.
        
        Args:
            bbox: (x, y, w, h) bounding box
            image_width: Image width in pixels
            image_height: Image height in pixels
            
        Returns:
            (twist, is_close_enough) tuple
        """
        x, y, w, h = [float(v) for v in bbox]
        
        # Calculate center x and height ratio
        cx = x + w / 2.0
        ratio = h / image_height if image_height > 0 else 0.0
        
        # Pixel error from image center (positive = object to the right)
        cx_err = cx - (image_width / 2.0)
        
        # Apply EMA smoothing
        if self._state.cx_ema is None:
            self._state.cx_ema = cx_err
            self._state.ratio_ema = ratio
        else:
            self._state.cx_ema = (
                self._ema_alpha * cx_err + 
                (1.0 - self._ema_alpha) * self._state.cx_ema
            )
            self._state.ratio_ema = (
                self._ema_alpha * ratio + 
                (1.0 - self._ema_alpha) * self._state.ratio_ema
            )
            
        # Check if close enough (stop condition)
        is_close_enough = self._state.ratio_ema >= DETECTION_STOP_THRESHOLD
        
        # Compute angular control (centering)
        ang = -Kp_ang * self._state.cx_ema
        ang = max(-MAX_ANG, min(MAX_ANG, ang))
        
        # Compute linear control (approach)
        lin = 0.0
        path_clear = True
        
        if self._is_path_clear_fn is not None:
            path_clear = self._is_path_clear_fn()
            
        if self._state.ratio_ema < self._target_ratio and path_clear:
            # Exponential approach for smooth deceleration
            beta = 6.0
            delta = max(0.0, self._target_ratio - self._state.ratio_ema)
            lin = MAX_LIN * (1.0 - math.exp(-beta * delta))
            lin = max(0.0, min(MAX_LIN, lin))
            
        # Create twist message
        twist = Twist()
        twist.linear.x = float(lin)
        twist.angular.z = float(ang)
        
        return twist, is_close_enough
        
    def compute_centering_only(self, bbox: Tuple[int, int, int, int],
                               image_width: int = CAM_WIDTH_DEFAULT) -> Optional[Twist]:
        """
        Compute centering control only (no forward motion).
        
        Useful for quick centering adjustments.
        
        Args:
            bbox: (x, y, w, h) bounding box
            image_width: Image width in pixels
            
        Returns:
            Twist message or None if already centered
        """
        x, y, w, h = [float(v) for v in bbox]
        
        cx = x + w / 2.0
        error_px = cx - (image_width / 2.0)
        
        center_tol = 40  # pixels
        
        if abs(error_px) <= center_tol:
            return None  # Already centered
            
        # Simple proportional centering
        ang = -max(-0.45, min(0.45, (error_px / (image_width / 2.0)) * 0.45))
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = float(ang)
        
        return twist
        
    def reset(self):
        """Reset EMA state."""
        self._state.cx_ema = None
        self._state.ratio_ema = None
        
    @property
    def current_ratio(self) -> Optional[float]:
        """Get current (smoothed) height ratio."""
        return self._state.ratio_ema
        
    @property
    def current_error(self) -> Optional[float]:
        """Get current (smoothed) center error in pixels."""
        return self._state.cx_ema
