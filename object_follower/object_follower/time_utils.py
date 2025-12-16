#!/usr/bin/env python3
"""
time_utils.py - Consistent time handling for Object Follower

CRITICAL FIX: Use ROS time consistently throughout the system.
This fixes the scan timing bug where scan completed in 0.16s instead of 6s.

The issue was mixing time.time() (wall clock) with ROS simulation time.
In Gazebo simulation, ROS time runs differently from wall time.
"""

import time
from typing import Optional
from rclpy.node import Node


class TimeManager:
    """
    Manages time consistently using ROS time when available.
    
    This class ensures we always use ROS time (which works correctly
    in simulation) instead of mixing wall time and ROS time.
    """
    
    def __init__(self, node: Optional[Node] = None):
        """
        Initialize TimeManager.
        
        Args:
            node: ROS2 node for clock access. If None, falls back to wall time.
        """
        self._node = node
        self._use_ros_time = node is not None
        
    def set_node(self, node: Node):
        """Set the ROS node for clock access."""
        self._node = node
        self._use_ros_time = True
        
    def now(self) -> float:
        """
        Get current time in seconds.
        
        Uses ROS time if node is available, otherwise falls back to wall time.
        
        Returns:
            Current time in seconds (float)
        """
        if self._use_ros_time and self._node is not None:
            try:
                return float(self._node.get_clock().now().nanoseconds) / 1e9
            except Exception:
                pass
        return time.time()
    
    def elapsed_since(self, start_time: float) -> float:
        """
        Calculate elapsed time since start_time.
        
        Args:
            start_time: Start time in seconds (from now())
            
        Returns:
            Elapsed time in seconds
        """
        return self.now() - start_time
    
    def is_timeout(self, start_time: float, timeout: float) -> bool:
        """
        Check if timeout has elapsed since start_time.
        
        Args:
            start_time: Start time in seconds
            timeout: Timeout duration in seconds
            
        Returns:
            True if timeout has elapsed
        """
        return self.elapsed_since(start_time) >= timeout


def get_ros_time_sec(node: Node) -> float:
    """
    Get current ROS time in seconds.
    
    Helper function for when you don't want to use TimeManager.
    
    Args:
        node: ROS2 node
        
    Returns:
        Current ROS time in seconds
    """
    try:
        return float(node.get_clock().now().nanoseconds) / 1e9
    except Exception:
        return time.time()
