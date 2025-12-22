#!/usr/bin/env python3
"""
waypoint_manager.py - Waypoint generation and management for Object Follower

Handles:
- Waypoint generation from occupancy grid map
- Random waypoint sampling
- Waypoint validation (distance checks, free cell snapping)
- Waypoint cycling (systematic vs random exploration)
"""

import math
import random
from typing import Optional, List, Tuple
from dataclasses import dataclass, field
import logging

from nav_msgs.msg import OccupancyGrid

from .config import (
    WAYPOINT_MARGIN,
    SNAP_SEARCH_RADIUS_M,
    MIN_WAYPOINT_DISTANCE,
)


@dataclass
class Waypoint:
    """Represents a waypoint with position and orientation."""
    x: float
    y: float
    yaw: float = 0.0
    
    def distance_to(self, other_x: float, other_y: float) -> float:
        """Calculate distance to another point."""
        return math.hypot(self.x - other_x, self.y - other_y)
        
    def as_tuple(self) -> Tuple[float, float, float]:
        """Return as (x, y, yaw) tuple."""
        return (self.x, self.y, self.yaw)


class WaypointManager:
    """
    Manages waypoints for exploration.
    
    Supports two exploration strategies:
    - SYSTEMATIC (Algo B): Pre-calculated waypoints from map
    - RANDOM (Algo A): Randomly sampled waypoints
    
    Key features:
    - Waypoint distance validation (prevents "0 poses" error)
    - Free cell snapping (ensures navigable waypoints)
    - Configurable exploration strategy
    """
    
    def __init__(self, logger: Optional[logging.Logger] = None,
                 use_random: bool = False):
        """
        Initialize waypoint manager.
        
        Args:
            logger: Optional logger
            use_random: If True, use random waypoints (Algo A)
        """
        self._logger = logger or logging.getLogger(__name__)
        self._use_random = use_random
        
        self._map: Optional[OccupancyGrid] = None
        self._waypoints: List[Waypoint] = []
        self._current_index: int = 0
        self._current_waypoint: Optional[Waypoint] = None
        self._retries: int = 0
        
    def set_map(self, occupancy_grid: OccupancyGrid):
        """
        Set the occupancy grid map and generate waypoints.
        
        Args:
            occupancy_grid: Nav2 occupancy grid message
        """
        self._map = occupancy_grid
        self._waypoints = self._generate_waypoints_from_map(occupancy_grid)
        self._current_index = 0
        
        self._logger.info(
            f"Waypoints generated from map: {len(self._waypoints)} points"
        )
        for i, wp in enumerate(self._waypoints):
            self._logger.info(f"  {i+1}: ({wp.x:.2f}, {wp.y:.2f})")
            
    def _generate_waypoints_from_map(self, grid: OccupancyGrid) -> List[Waypoint]:
        """
        Generate waypoints from occupancy grid.
        
        Creates center + four corners with margin.
        """
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
        
        # Center + four corners
        candidates = [
            ((xmin + xmax) / 2.0, (ymin + ymax) / 2.0, 0.0),  # Center
            (xmin + WAYPOINT_MARGIN, ymin + WAYPOINT_MARGIN, 0.0),  # Bottom-left
            (xmax - WAYPOINT_MARGIN, ymin + WAYPOINT_MARGIN, 0.0),  # Bottom-right
            (xmax - WAYPOINT_MARGIN, ymax - WAYPOINT_MARGIN, 0.0),  # Top-right
            (xmin + WAYPOINT_MARGIN, ymax - WAYPOINT_MARGIN, 0.0),  # Top-left
        ]
        
        waypoints = []
        for (x, y, yaw) in candidates:
            snapped = self._snap_to_free_cell(grid, x, y, SNAP_SEARCH_RADIUS_M)
            if snapped is not None:
                waypoints.append(Waypoint(x=snapped[0], y=snapped[1], yaw=yaw))
            else:
                self._logger.warn(
                    f"Could not snap waypoint near ({x:.2f},{y:.2f}), skipping"
                )
                
        return waypoints
        
    def _snap_to_free_cell(self, grid: OccupancyGrid, 
                           x_target: float, y_target: float,
                           radius_m: float) -> Optional[Tuple[float, float]]:
        """
        Snap position to nearest free cell on map.
        
        Searches in expanding rings from target position.
        """
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
                    
                    if val == 0:  # Free cell
                        x_world = ox + (ii + 0.5) * res
                        y_world = oy + (jj + 0.5) * res
                        return (x_world, y_world)
                        
        return None
        
    def sample_random_waypoint(self) -> Optional[Waypoint]:
        """
        Sample a random waypoint from the map.
        
        Returns:
            Random waypoint or None if sampling fails
        """
        if not self._map:
            return None
            
        info = self._map.info
        w = int(info.width)
        h = int(info.height)
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        
        xmin = ox
        ymin = oy
        xmax = ox + w * res
        ymax = oy + h * res
        
        for _ in range(40):  # Max attempts
            xr = random.uniform(xmin + WAYPOINT_MARGIN, xmax - WAYPOINT_MARGIN)
            yr = random.uniform(ymin + WAYPOINT_MARGIN, ymax - WAYPOINT_MARGIN)
            snapped = self._snap_to_free_cell(self._map, xr, yr, SNAP_SEARCH_RADIUS_M)
            
            if snapped:
                return Waypoint(x=snapped[0], y=snapped[1], yaw=0.0)
                
        return None
        
    def get_next_waypoint(self, robot_x: float = 0.0, 
                          robot_y: float = 0.0) -> Optional[Waypoint]:
        """
        Get the next waypoint to navigate to.
        
        Args:
            robot_x, robot_y: Current robot position (for distance validation)
            
        Returns:
            Next waypoint or None if no valid waypoint available
        """
        if self._use_random:
            # ALGO A: Random waypoints
            wp = self.sample_random_waypoint()
            if wp and wp.distance_to(robot_x, robot_y) >= MIN_WAYPOINT_DISTANCE:
                self._current_waypoint = wp
                self._retries = 0
                return wp
            return None
            
        # ALGO B: Systematic waypoints
        if not self._waypoints:
            return None
            
        # Get next waypoint from list
        wp = self._waypoints[self._current_index]
        
        # Validate distance
        if wp.distance_to(robot_x, robot_y) < MIN_WAYPOINT_DISTANCE:
            self._logger.warn(
                f"Waypoint too close ({wp.distance_to(robot_x, robot_y):.2f}m), skipping to next"
            )
            self._current_index = (self._current_index + 1) % len(self._waypoints)
            wp = self._waypoints[self._current_index]
            
        self._current_waypoint = wp
        self._retries = 0
        return wp
        
    def advance_to_next(self):
        """Advance to the next waypoint in the list."""
        if self._waypoints and not self._use_random:
            self._current_index = (self._current_index + 1) % len(self._waypoints)
        self._current_waypoint = None
        self._retries = 0
        
    def mark_waypoint_failed(self):
        """Mark current waypoint as failed (for retry logic)."""
        self._retries += 1
        if self._retries >= 3:
            self._logger.warn(f"Waypoint failed {self._retries} times, skipping")
            self.advance_to_next()
            
    @property
    def current_waypoint(self) -> Optional[Waypoint]:
        """Get current waypoint."""
        return self._current_waypoint
        
    @property
    def waypoints(self) -> List[Waypoint]:
        """Get all waypoints."""
        return self._waypoints
        
    @property
    def has_map(self) -> bool:
        """Check if map is available."""
        return self._map is not None
        
    @property
    def use_random(self) -> bool:
        """Check if using random waypoints."""
        return self._use_random
        
    @use_random.setter
    def use_random(self, value: bool):
        """Set random waypoint mode."""
        self._use_random = value
