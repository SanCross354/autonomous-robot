#!/usr/bin/env python3
"""
detection_processor.py - YOLO detection processing for Object Follower

Handles:
- Detection filtering by class
- Best detection selection (largest bbox)
- Detection ratio calculation (for state transitions)
- Scan interruption on detection
"""

from typing import Optional, Tuple, List, Any
from dataclasses import dataclass
import logging

from .config import (
    APPROACH_SWITCH_RATIO,
    TRACKING_SWITCH_RATIO,
    DETECTION_STOP_THRESHOLD,
    CAM_WIDTH_DEFAULT,
    CAM_HEIGHT_DEFAULT,
    EDGE_TRIGGER_ZONE_PX,
)


@dataclass
class ProcessedDetection:
    """Represents a processed YOLO detection."""
    class_name: str
    left: int
    top: int
    right: int
    bottom: int
    confidence: float = 0.0
    
    @property
    def width(self) -> int:
        return self.right - self.left
        
    @property
    def height(self) -> int:
        return self.bottom - self.top
        
    @property
    def area(self) -> int:
        return self.width * self.height
        
    @property
    def center_x(self) -> float:
        return (self.left + self.right) / 2.0
        
    @property
    def center_y(self) -> float:
        return (self.top + self.bottom) / 2.0
        
    def as_xywh(self) -> Tuple[int, int, int, int]:
        """Return as (x, y, w, h) tuple."""
        return (self.left, self.top, self.width, self.height)
        
    def ratio(self, image_height: int) -> float:
        """Calculate height ratio relative to image."""
        return self.height / image_height if image_height > 0 else 0.0


class DetectionProcessor:
    """
    Processes YOLO detections for object following.
    
    Key features:
    - Filters detections by selected class
    - Selects best detection (largest area)
    - Calculates ratio for state transitions
    - Checks trigger zones (from KCF-YOLO paper)
    """
    
    def __init__(self, logger: Optional[logging.Logger] = None):
        """
        Initialize detection processor.
        
        Args:
            logger: Optional logger
        """
        self._logger = logger or logging.getLogger(__name__)
        self._selected_class: Optional[str] = None
        self._current_detection: Optional[ProcessedDetection] = None
        self._image_width = CAM_WIDTH_DEFAULT
        self._image_height = CAM_HEIGHT_DEFAULT
        
    def set_selected_class(self, class_name: Optional[str]):
        """
        Set the class to track.
        
        Args:
            class_name: Class name to track (e.g., "person")
        """
        self._selected_class = class_name.lower().strip() if class_name else None
        self._current_detection = None
        
    def set_image_dimensions(self, width: int, height: int):
        """
        Set image dimensions for ratio calculations.
        
        Args:
            width: Image width in pixels
            height: Image height in pixels
        """
        self._image_width = width or CAM_WIDTH_DEFAULT
        self._image_height = height or CAM_HEIGHT_DEFAULT
        
    def process_inference(self, inference_msg: Any) -> Optional[ProcessedDetection]:
        """
        Process YOLO inference message.
        
        Args:
            inference_msg: Yolov8Inference message
            
        Returns:
            Best detection for selected class, or None
        """
        if not self._selected_class:
            return None
            
        # Update image dimensions if provided
        msg_width = getattr(inference_msg, 'image_width', None)
        msg_height = getattr(inference_msg, 'image_height', None)
        if msg_width and msg_height:
            self._image_width = msg_width
            self._image_height = msg_height
            
        # Find best detection for selected class
        best_det = None
        best_area = 0.0
        
        detections = getattr(inference_msg, 'yolov8_inference', [])
        for det in detections:
            det_class = getattr(det, 'class_name', '')
            if det_class and det_class.lower().strip() == self._selected_class:
                bw = max(0.0, float(det.right - det.left))
                bh = max(0.0, float(det.bottom - det.top))
                area = bw * bh
                
                if area > best_area:
                    best_area = area
                    best_det = ProcessedDetection(
                        class_name=det_class,
                        left=int(det.left),
                        top=int(det.top),
                        right=int(det.right),
                        bottom=int(det.bottom),
                        confidence=getattr(det, 'confidence', 0.0)
                    )
                    
        self._current_detection = best_det
        return best_det
        
    def scale_to_camera(self, detection: ProcessedDetection,
                        camera_width: int, camera_height: int,
                        det_width: Optional[int] = None,
                        det_height: Optional[int] = None) -> Tuple[int, int, int, int]:
        """
        Scale detection coordinates to camera image dimensions.
        
        Args:
            detection: Detection to scale
            camera_width, camera_height: Camera image dimensions
            det_width, det_height: Original detection dimensions (optional)
            
        Returns:
            (left, top, right, bottom) in camera coordinates
        """
        det_w = det_width or self._image_width
        det_h = det_height or self._image_height
        
        if det_w > 0 and det_h > 0:
            sx = float(camera_width) / float(det_w)
            sy = float(camera_height) / float(det_h)
        else:
            sx = sy = 1.0
            
        left = int(max(0, round(detection.left * sx)))
        top = int(max(0, round(detection.top * sy)))
        right = int(min(camera_width, round(detection.right * sx)))
        bottom = int(min(camera_height, round(detection.bottom * sy)))
        
        # Ensure valid
        if right <= left:
            right = min(camera_width, left + 1)
        if bottom <= top:
            bottom = min(camera_height, top + 1)
            
        return (left, top, right, bottom)
        
    def get_state_from_ratio(self, ratio: float) -> str:
        """
        Determine recommended state based on detection ratio.
        
        Based on KCF-YOLO paper trigger zones:
        - ratio >= DETECTION_STOP_THRESHOLD: STOPPED
        - ratio >= TRACKING_SWITCH_RATIO: TRACKING
        - ratio >= APPROACH_SWITCH_RATIO: APPROACHING
        - ratio < APPROACH_SWITCH_RATIO: (no detection / too small)
        
        Args:
            ratio: Bbox height ratio (0.0 to 1.0)
            
        Returns:
            Recommended state string
        """
        if ratio >= DETECTION_STOP_THRESHOLD:
            return "STOPPED"
        elif ratio >= TRACKING_SWITCH_RATIO:
            return "TRACKING"
        elif ratio >= APPROACH_SWITCH_RATIO:
            return "APPROACHING"
        else:
            return "NONE"
            
    def is_in_trigger_zone(self, detection: ProcessedDetection) -> bool:
        """
        Check if detection is in edge trigger zone (from KCF-YOLO paper).
        
        "The parameter `dis` represents a certain distance from the left or right 
        boundary of the image to the image center."
        
        Args:
            detection: Detection to check
            
        Returns:
            True if detection is near image edge
        """
        cx = detection.center_x
        
        left_trigger = EDGE_TRIGGER_ZONE_PX
        right_trigger = self._image_width - EDGE_TRIGGER_ZONE_PX
        
        return cx < left_trigger or cx > right_trigger
        
    def should_interrupt_scan(self, detection: ProcessedDetection) -> bool:
        """
        Check if detection should interrupt an ongoing scan.
        
        Object must be reasonably sized (>= TRACKING_SWITCH_RATIO) to
        interrupt scanning.
        
        Args:
            detection: Detection to check
            
        Returns:
            True if scan should be interrupted
        """
        ratio = detection.ratio(self._image_height)
        return ratio >= TRACKING_SWITCH_RATIO
        
    @property
    def current_detection(self) -> Optional[ProcessedDetection]:
        """Get current detection."""
        return self._current_detection
        
    @property
    def selected_class(self) -> Optional[str]:
        """Get selected class name."""
        return self._selected_class
        
    @property
    def image_width(self) -> int:
        """Get image width."""
        return self._image_width
        
    @property
    def image_height(self) -> int:
        """Get image height."""
        return self._image_height
