"""
Object Follower Package

A modular ROS2 package for autonomous object following with:
- YOLOv8 detection
- Hybrid CSRT/KCF tracking with YOLO drift correction
- Nav2 waypoint exploration
- Visual servoing

Modules:
    config          - Configuration constants and tunables
    states          - State machine definitions
    time_utils      - Consistent ROS time handling
    trackers        - SimpleCentroidTracker and HybridTracker
    scan_handler    - Waypoint scanning logic
    nav2_handler    - Nav2 action client handling
    waypoint_manager - Waypoint generation and cycling
    visual_servo    - Visual servoing controller
    detection_processor - YOLO detection processing

Main Nodes:
    object_follower_modular  - Refactored modular version (recommended)
    object_follower_experiment - Original monolithic version

Usage:
    ros2 run object_follower object_follower_modular
    
Import example:
    from object_follower.config import FOLLOW_DISTANCE
    from object_follower.states import FollowerState
    from object_follower.trackers import HybridTracker
"""

__version__ = '0.2.0'
