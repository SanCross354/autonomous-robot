#!/usr/bin/env python3
"""
trackers.py - Tracking implementations for Object Follower

Implements the hybrid tracking approach from KCF-YOLO paper (Section 3.3):
- Primary: YOLO detection (always running)
- Auxiliary: KCF/CSRT tracker (between YOLO detections, corrected periodically)

Key insight from paper:
"When the target re-enters the field of view, YOLO v5s can identify and 
continue tracking the previously lost target by combining it with the KCF."

This module implements:
1. SimpleCentroidTracker - Fallback when OpenCV contrib not available
2. HybridTracker - YOLO + CSRT/KCF with periodic drift correction
"""

import cv2
import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass
import logging

from .config import (
    YOLO_REINIT_INTERVAL,
    TRACKER_TYPE,
    CAM_WIDTH_DEFAULT,
    CAM_HEIGHT_DEFAULT,
)


@dataclass
class BoundingBox:
    """Represents a bounding box with x, y, width, height."""
    x: int
    y: int
    width: int
    height: int
    
    @property
    def center(self) -> Tuple[float, float]:
        """Get center point of bbox."""
        return (self.x + self.width / 2.0, self.y + self.height / 2.0)
    
    @property
    def area(self) -> int:
        """Get area of bbox."""
        return self.width * self.height
    
    def as_tuple(self) -> Tuple[int, int, int, int]:
        """Return as (x, y, w, h) tuple."""
        return (self.x, self.y, self.width, self.height)
    
    def as_ltrb(self) -> Tuple[int, int, int, int]:
        """Return as (left, top, right, bottom) tuple."""
        return (self.x, self.y, self.x + self.width, self.y + self.height)
    
    @classmethod
    def from_ltrb(cls, left: int, top: int, right: int, bottom: int) -> 'BoundingBox':
        """Create from left, top, right, bottom coordinates."""
        return cls(x=left, y=top, width=right - left, height=bottom - top)
    
    @classmethod
    def from_tuple(cls, bbox: Tuple[int, int, int, int]) -> 'BoundingBox':
        """Create from (x, y, w, h) tuple."""
        return cls(x=bbox[0], y=bbox[1], width=bbox[2], height=bbox[3])


def clamp_bbox(x: int, y: int, w: int, h: int, 
               img_w: int, img_h: int) -> Optional[Tuple[int, int, int, int]]:
    """
    Clamp bbox to be inside image boundaries.
    
    Args:
        x, y, w, h: Bounding box coordinates
        img_w, img_h: Image dimensions
        
    Returns:
        (x, y, w, h) tuple or None if invalid
    """
    x = int(max(0, round(x)))
    y = int(max(0, round(y)))
    w = int(round(w))
    h = int(round(h))
    
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


class SimpleCentroidTracker:
    """
    Simple tracker that relies on YOLO updates.
    
    Use this when OpenCV contrib is not available.
    It holds the last known position until YOLO updates it.
    This is essentially "dead reckoning" between YOLO frames.
    """
    
    def __init__(self):
        self.center: Optional[Tuple[float, float]] = None
        self.bbox: Optional[Tuple[int, int, int, int]] = None
        
    def init(self, image: np.ndarray, bbox: Tuple[int, int, int, int]) -> bool:
        """
        Initialize tracker with bounding box.
        
        Args:
            image: Current frame (unused, for API compatibility)
            bbox: (x, y, w, h) bounding box
            
        Returns:
            True if initialization successful
        """
        self.bbox = bbox
        x, y, w, h = bbox
        self.center = (x + w / 2.0, y + h / 2.0)
        return True
        
    def update(self, image: np.ndarray) -> Tuple[bool, Tuple[int, int, int, int]]:
        """
        Update tracker (returns last known position).
        
        This tracker doesn't do actual tracking - it relies on
        periodic YOLO updates via init() calls.
        
        Args:
            image: Current frame (unused)
            
        Returns:
            (success, bbox) tuple
        """
        if self.bbox is None:
            return False, (0, 0, 0, 0)
        return True, self.bbox


class HybridTracker:
    """
    Hybrid YOLO + OpenCV Tracker with periodic drift correction.
    
    Implements the KCF-YOLO paper's approach (Section 3.3):
    - Use OpenCV tracker (CSRT/KCF) for inter-frame prediction
    - Periodically re-initialize from YOLO to correct drift
    
    "The KCF-YOLO algorithm utilizes the target detection frame obtained 
    by the YOLO v5s algorithm as the region of interest to initialize 
    the KCF algorithm for auxiliary tracking."
    
    Benefits:
    - Drift eliminated: YOLO corrects tracker every N frames
    - Smooth tracking: OpenCV tracker fills gaps between YOLO detections
    - Handles occlusions: YOLO can re-acquire lost targets
    """
    
    def __init__(self, tracker_type: str = TRACKER_TYPE, 
                 reinit_interval: int = YOLO_REINIT_INTERVAL,
                 logger: Optional[logging.Logger] = None):
        """
        Initialize hybrid tracker.
        
        Args:
            tracker_type: Type of OpenCV tracker ('CSRT', 'KCF', 'MOSSE', 'CENTROID')
            reinit_interval: Re-init from YOLO every N frames
            logger: Optional logger for debug output
        """
        self.tracker_type = tracker_type
        self.reinit_interval = reinit_interval
        self.logger = logger or logging.getLogger(__name__)
        
        self._tracker = None
        self._reinit_counter = 0
        self._last_bbox: Optional[Tuple[int, int, int, int]] = None
        self._initialized = False
        
    def _create_opencv_tracker(self) -> Optional[object]:
        """
        Create OpenCV tracker with compatibility for different versions.
        
        Returns:
            OpenCV tracker object or None if creation fails
        """
        if self.tracker_type == 'CENTROID':
            return SimpleCentroidTracker()
            
        tracker = None
        
        # Try standard path first (OpenCV 4.x)
        try:
            if self.tracker_type == 'CSRT':
                tracker = cv2.TrackerCSRT_create()
            elif self.tracker_type == 'KCF':
                tracker = cv2.TrackerKCF_create()
            elif self.tracker_type == 'MOSSE':
                tracker = cv2.TrackerMOSSE_create()
        except AttributeError:
            pass
            
        if tracker is not None:
            return tracker
            
        # Try legacy path (OpenCV 4.5+)
        try:
            if self.tracker_type == 'CSRT':
                tracker = cv2.legacy.TrackerCSRT_create()
            elif self.tracker_type == 'KCF':
                tracker = cv2.legacy.TrackerKCF_create()
            elif self.tracker_type == 'MOSSE':
                tracker = cv2.legacy.TrackerMOSSE_create()
        except AttributeError:
            pass
            
        if tracker is None:
            self.logger.warning(
                f"Could not create tracker {self.tracker_type}, "
                "falling back to SimpleCentroidTracker"
            )
            return SimpleCentroidTracker()
            
        return tracker
        
    def init(self, image: np.ndarray, bbox: Tuple[int, int, int, int]) -> bool:
        """
        Initialize tracker with bounding box.
        
        Args:
            image: Current frame
            bbox: (x, y, w, h) bounding box
            
        Returns:
            True if initialization successful
        """
        if image is None:
            return False
            
        # Clamp bbox to image bounds
        img_h, img_w = image.shape[:2]
        clamped = clamp_bbox(bbox[0], bbox[1], bbox[2], bbox[3], img_w, img_h)
        
        if clamped is None or min(clamped[2], clamped[3]) < 16:
            self.logger.warning(f"Invalid bbox for tracker init: {bbox}")
            return False
            
        # Create new tracker
        self._tracker = self._create_opencv_tracker()
        if self._tracker is None:
            return False
            
        # Initialize tracker
        try:
            ok = self._tracker.init(image, clamped)
            if ok:
                self._last_bbox = clamped
                self._initialized = True
                self._reinit_counter = 0
                return True
        except Exception as e:
            self.logger.warning(f"Tracker init failed: {e}")
            
        return False
        
    def update(self, image: np.ndarray, 
               yolo_bbox: Optional[Tuple[int, int, int, int]] = None) -> Tuple[bool, Tuple[int, int, int, int]]:
        """
        Update tracker with current frame.
        
        This is the KEY method implementing the KCF-YOLO hybrid approach:
        1. Get prediction from OpenCV tracker
        2. Every N frames, re-init from YOLO detection to correct drift
        
        Args:
            image: Current frame
            yolo_bbox: Optional YOLO detection (x, y, w, h) for drift correction
            
        Returns:
            (success, bbox) tuple
        """
        if not self._initialized or self._tracker is None:
            if yolo_bbox is not None:
                return True, yolo_bbox
            return False, (0, 0, 0, 0)
            
        if image is None:
            return False, self._last_bbox or (0, 0, 0, 0)
            
        # Step 1: Get OpenCV tracker prediction
        try:
            ok, bbox = self._tracker.update(image)
            if ok:
                self._last_bbox = (int(bbox[0]), int(bbox[1]), 
                                   int(bbox[2]), int(bbox[3]))
                self._reinit_counter += 1
            else:
                # Tracker lost - try to recover from YOLO
                if yolo_bbox is not None:
                    self.logger.info("Tracker lost, recovering from YOLO detection")
                    self.init(image, yolo_bbox)
                    return True, yolo_bbox
                return False, self._last_bbox or (0, 0, 0, 0)
        except Exception as e:
            self.logger.warning(f"Tracker update failed: {e}")
            if yolo_bbox is not None:
                return True, yolo_bbox
            return False, self._last_bbox or (0, 0, 0, 0)
            
        # Step 2: Periodic YOLO re-initialization (CRITICAL for drift correction!)
        # From KCF-YOLO paper: "re-initialize the KCF algorithm for auxiliary tracking"
        if yolo_bbox is not None and self._reinit_counter >= self.reinit_interval:
            try:
                # Re-init from YOLO to correct drift
                new_tracker = self._create_opencv_tracker()
                if new_tracker is not None:
                    img_h, img_w = image.shape[:2]
                    clamped = clamp_bbox(yolo_bbox[0], yolo_bbox[1], 
                                        yolo_bbox[2], yolo_bbox[3], 
                                        img_w, img_h)
                    if clamped is not None:
                        if new_tracker.init(image, clamped):
                            self._tracker = new_tracker
                            self._last_bbox = clamped
                            self._reinit_counter = 0
                            self.logger.debug(
                                f"✅ Tracker drift corrected by YOLO at {clamped}"
                            )
            except Exception as e:
                self.logger.warning(f"YOLO reinit failed: {e}")
                
        return True, self._last_bbox
        
    def reinit_from_detection(self, image: np.ndarray, 
                               bbox: Tuple[int, int, int, int]) -> bool:
        """
        Force re-initialization from detection (e.g., when switching states).
        
        Args:
            image: Current frame
            bbox: Detection bbox (x, y, w, h)
            
        Returns:
            True if successful
        """
        return self.init(image, bbox)
        
    def reset(self):
        """Reset tracker state."""
        self._tracker = None
        self._last_bbox = None
        self._initialized = False
        self._reinit_counter = 0
        
    @property
    def is_initialized(self) -> bool:
        """Check if tracker is initialized."""
        return self._initialized
        
    @property
    def last_bbox(self) -> Optional[Tuple[int, int, int, int]]:
        """Get last known bounding box."""
        return self._last_bbox


def create_tracker(tracker_type: str = TRACKER_TYPE) -> HybridTracker:
    """
    Factory function to create a tracker.
    
    Args:
        tracker_type: Type of tracker to create
        
    Returns:
        HybridTracker instance
    """
    return HybridTracker(tracker_type=tracker_type)
