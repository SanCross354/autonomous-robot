#!/usr/bin/env python3
"""
config.py - Configuration constants for Object Follower

All tunables in one place for easy adjustment and thesis experimentation.
Based on KCF-YOLO paper recommendations where applicable.
"""

# =============================================================================
# NAVIGATION & FOLLOWING DISTANCES
# =============================================================================
FOLLOW_DISTANCE = 1.0            # Desired distance (meters) from camera to object
NEAR_GOAL_EPS = 0.25             # Treat as reached if within this distance (meters)
MIN_GOAL_UPDATE = 0.15           # Min change (meters) to consider new nav goal
MIN_GOAL_PERIOD = 0.8            # Min seconds between nav goal sends
NAV_RESULT_COOLDOWN = 0.8        # Seconds after nav2 result before sending again

# =============================================================================
# TIMEOUT SETTINGS
# =============================================================================
OBJECT_LOST_TIMEOUT = 2.0        # Seconds before declaring object lost
ROTATE_TIMEOUT = 6.0             # Seconds to rotate in SEARCHING phase
SEARCH_FORWARD_TIME = 1.0        # Seconds to drive forward in SEARCHING
SEARCH_MAX_DURATION = 18.0       # Max seconds in SEARCHING before returning to EXPLORING
APPROACH_LOST_TIMEOUT = 8.0      # Seconds to continue approach after losing object (Motion Memory)
TRACKING_LOST_TIMEOUT = 1.5      # Seconds until tracker considered lost

# =============================================================================
# MOTION SPEEDS
# =============================================================================
SEARCH_FORWARD_SPEED = 0.18      # m/s during SEARCHING forward step
ROTATION_SPEED = 0.35            # rad/s rotation speed

# =============================================================================
# VISUAL SERVOING CONTROL GAINS
# =============================================================================
# OPTIMIZED for faster demo response (was: Kp_ang=0.002, MAX_LIN=0.18, MAX_ANG=0.6)
Kp_ang = 0.003                   # Angular proportional gain (50% more responsive)
Kp_lin = 0.6                     # Linear proportional gain (unused currently)
MAX_LIN = 0.30                   # Max linear velocity for visual servoing (67% faster)
MAX_ANG = 0.85                   # Max angular velocity for visual servoing (42% faster)

# =============================================================================
# DETECTION & TRACKING THRESHOLDS (Based on KCF-YOLO paper)
# =============================================================================
# These define the "trigger zones" from the KCF-YOLO paper (Section 3.3)
# FIX: Skip APPROACHING (Nav2). Go straight to TRACKING (Visual Servoing) when object visible.
# Nav2 path planning causes the robot to turn away from the object.
APPROACH_SWITCH_RATIO = 0.05     # (Unused now, kept for reference)
TRACKING_SWITCH_RATIO = 0.05    # Switch directly to TRACKING when object detected
DETECTION_STOP_THRESHOLD = 0.55  # bbox height ratio => stop (visual close)

# Stable detection frames (for debouncing)
STABLE_DETECTION_FRAMES = 2

# =============================================================================
# HYBRID TRACKING (From KCF-YOLO Paper Section 3.3)
# =============================================================================
# "When the target re-enters the field of view, YOLO can identify and continue
# tracking the previously lost target by combining it with the KCF."
YOLO_REINIT_INTERVAL = 5         # Re-init tracker every N frames to prevent drift
TRACKER_TYPE = 'CENTROID'        # CSRT not available, using CENTROID + freshness check
TRACKING_HZ = 10.0               # Tracking loop frequency (Hz)

# =============================================================================
# SCANNING SETTINGS (At Waypoints)
# =============================================================================
SCAN_DURATION = 6.0              # Seconds for full 360° rotation
SCAN_SPEED = 0.5                 # rad/s (about 30°/s -> 6s for 360°)
SCAN_ROTATIONS = 1.0             # Number of full rotations

# =============================================================================
# CAMERA DEFAULTS
# =============================================================================
CAM_WIDTH_DEFAULT = 640
CAM_HEIGHT_DEFAULT = 480

# =============================================================================
# MAP & WAYPOINT SETTINGS
# =============================================================================
WAYPOINT_MARGIN = 2.5            # Margin from map edges for waypoint generation
SNAP_SEARCH_RADIUS_M = 0.8       # Search radius for snapping to free cell
MIN_WAYPOINT_DISTANCE = 0.5      # Skip waypoints closer than this (meters)

# =============================================================================
# LIDAR SETTINGS
# =============================================================================
SECTOR_HALF_WIDTH = 5            # Laser beams half width around front

# =============================================================================
# FEATURE FLAGS
# =============================================================================
USE_EXPLORATION = True           # Enable autonomous waypoint exploration

# =============================================================================
# METRICS & LOGGING
# =============================================================================
METRICS_CSV = "/home/sancross354/articubot_TA/metrics_results.csv"

# =============================================================================
# TRIGGER ZONE (From KCF-YOLO Paper)
# =============================================================================
# "The parameter `dis` represents a certain distance from the left or right 
# boundary of the image to the image center." - Used for edge detection
EDGE_TRIGGER_ZONE_PX = 200       # Pixels from edge to trigger auxiliary tracking
