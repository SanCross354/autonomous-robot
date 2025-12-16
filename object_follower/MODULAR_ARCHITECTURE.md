# Object Follower - Modular Architecture

## 🎯 Overview

This document describes the modular refactoring of the `object_follower_experiment.py` node.
The refactoring addresses critical bugs identified in your research and implements insights
from the KCF-YOLO paper.

## 📁 File Structure

```
object_follower/
├── __init__.py                  # Package metadata
├── config.py                    # ✅ All tunables in one place
├── states.py                    # ✅ State machine definitions
├── time_utils.py                # ✅ FIX: Consistent ROS time handling
├── trackers.py                  # ✅ FIX: HybridTracker with YOLO drift correction
├── scan_handler.py              # ✅ FIX: Scan timing using ROS time
├── nav2_handler.py              # ✅ FIX: Nav2 abort handling
├── waypoint_manager.py          # ✅ FIX: Waypoint distance validation
├── visual_servo.py              # Visual servoing controller
├── detection_processor.py       # ✅ FIX: Detection during scan
├── object_follower_modular.py   # Main node (NEW - uses all modules)
└── object_follower_experiment.py # Original (kept for reference)
```

## 🐛 Bugs Fixed

### 1. Scan Timing Bug (CRITICAL)
**Problem:** Scan completed in 0.16s instead of 6s
**Root Cause:** Mixing `time.time()` (wall clock) with ROS simulation time
**Fix Location:** `time_utils.py`, `scan_handler.py`

```python
# OLD (broken):
self.scanning_start = time.time()  # Wall time
elapsed = now - self.scanning_start  # ROS time - Wall time = WRONG!

# NEW (fixed):
self._time = TimeManager(self)  # Uses ROS time consistently
elapsed = self._time.elapsed_since(self.scanning_start)  # Correct!
```

### 2. Detection Ignored During Scan
**Problem:** Robot ignored YOLO detections while scanning at waypoints
**Fix Location:** `detection_processor.py`, `object_follower_modular.py`

```python
# In _detection_callback():
if self._state == FollowerState.EXPLORING and self._scanner.is_active:
    if self._detector.should_interrupt_scan(detection):
        self._scanner.interrupt(reason="object detected")
        self._state = FollowerState.TRACKING
```

### 3. "0 Poses" Nav2 Error
**Problem:** Nav2 fails when waypoint is too close to robot
**Fix Location:** `waypoint_manager.py`, `nav2_handler.py`

```python
# Validate waypoint distance before sending
if wp.distance_to(robot_x, robot_y) < MIN_WAYPOINT_DISTANCE:
    self._logger.warn("Waypoint too close, skipping to next")
    self._waypoints.advance_to_next()
```

### 4. Tracker Drift (From KCF-YOLO Paper)
**Problem:** CSRT tracker drifts after 2-3 seconds
**Fix Location:** `trackers.py` (HybridTracker class)

```python
# Every N frames, re-init tracker from YOLO detection
if yolo_bbox is not None and self._reinit_counter >= self.reinit_interval:
    self._tracker = self._create_opencv_tracker()
    self._tracker.init(image, yolo_bbox)
    self._reinit_counter = 0
    self.logger.debug("✅ Tracker drift corrected by YOLO")
```

## 🔬 KCF-YOLO Paper Implementation

### Hybrid Tracking Architecture (Section 3.3)
```
Primary:    YOLO detection (always running)
               ↓
Auxiliary:  CSRT tracker (between YOLO detections)
               ↓
Correction: YOLO re-init every 5 frames (prevents drift)
```

### Trigger Zones (Section 3.3)
```python
# From config.py
EDGE_TRIGGER_ZONE_PX = 200  # Pixels from edge

# When object is at edge, switch to tracking mode
if detection.center_x < EDGE_TRIGGER_ZONE_PX:
    activate_auxiliary_tracker()
```

### State Transitions Based on Ratio
```python
# From config.py (based on paper recommendations)
APPROACH_SWITCH_RATIO = 0.05   # Far: Use Nav2
TRACKING_SWITCH_RATIO = 0.12   # Close: Use visual servo
DETECTION_STOP_THRESHOLD = 0.55  # Very close: Stop
```

## 🚀 Usage

### Run the Modular Node
```bash
ros2 run object_follower object_follower_modular
```

### Parameters
```bash
# Use random waypoints (Algo A) instead of systematic (Algo B)
ros2 run object_follower object_follower_modular --ros-args -p use_random_search:=true

# Return home after success
ros2 run object_follower object_follower_modular --ros-args -p return_home_after_success:=true
```

### Configuration
Edit `config.py` to tune parameters:
```python
# Navigation
FOLLOW_DISTANCE = 1.0         # Desired distance from object
NEAR_GOAL_EPS = 0.25          # "Close enough" tolerance

# Tracking
YOLO_REINIT_INTERVAL = 5      # Re-init tracker every N frames
TRACKING_SWITCH_RATIO = 0.12  # Switch to tracking at this ratio

# Scanning
SCAN_DURATION = 6.0           # Seconds for 360° rotation
SCAN_SPEED = 0.5              # rad/s
```

## 📊 Module Responsibilities

| Module | Responsibility | Key Classes |
|--------|---------------|-------------|
| `config.py` | All tunables | (constants only) |
| `states.py` | State definitions | `FollowerState` |
| `time_utils.py` | ROS time handling | `TimeManager` |
| `trackers.py` | Hybrid tracking | `HybridTracker` |
| `scan_handler.py` | Waypoint scanning | `ScanHandler` |
| `nav2_handler.py` | Nav2 navigation | `Nav2Handler` |
| `waypoint_manager.py` | Waypoint cycling | `WaypointManager` |
| `visual_servo.py` | Visual control | `VisualServoController` |
| `detection_processor.py` | YOLO processing | `DetectionProcessor` |

## 🔄 State Machine

```
       ┌───────────────────────────────────────────────────┐
       │                    IDLE                           │
       │  (waiting for object selection)                   │
       └───────────────────────┬───────────────────────────┘
                               │ select object
                               ▼
       ┌───────────────────────────────────────────────────┐
       │                 EXPLORING                          │
       │  • Navigate to waypoints (Nav2)                   │
       │  • Scan 360° at each waypoint                     │
       │  • Detection can interrupt scan!                  │
       └───────────────────────┬───────────────────────────┘
                               │ object detected (ratio >= 0.05)
                               ▼
       ┌───────────────────────────────────────────────────┐
       │                APPROACHING                         │
       │  • Nav2 goal towards object                       │
       │  • Updates goal periodically                      │
       └───────────────────────┬───────────────────────────┘
                               │ object close (ratio >= 0.12)
                               ▼
       ┌───────────────────────────────────────────────────┐
       │                 TRACKING                           │
       │  • Pure visual servoing                           │
       │  • Hybrid tracker with YOLO correction            │
       │  • EMA smoothing for stable control               │
       └───────────────────────┬───────────────────────────┘
                               │ object very close (ratio >= 0.55)
                               ▼
       ┌───────────────────────────────────────────────────┐
       │                  STOPPED                           │
       │  • Maintain position                              │
       │  • Record metrics                                 │
       └───────────────────────────────────────────────────┘

       SEARCHING state entered when object lost from TRACKING/APPROACHING
```

## 📈 Metrics

Metrics are recorded to `/tmp/object_follower_metrics.csv`:
- `selection_id`: Counter for each object selection
- `selected_class`: Class name (e.g., "person")
- `detection_latency_s`: Time from selection to first detection
- `time_to_reach_s`: Time from detection to reaching object
- `nav_goals_sent`: Number of Nav2 goals sent
- `nav_goals_canceled`: Number of goals canceled
- `result`: "success" or "failure"

## 🎓 Thesis Contribution

Your system is **novel** compared to KCF-YOLO and other papers:

| Feature | KCF-YOLO (2024) | Your System |
|---------|-----------------|-------------|
| Environment | Orchard (structured) | **Indoor (complex)** |
| Exploration | ❌ None | **✅ Autonomous waypoints** |
| Navigation | Pure visual | **Hybrid Nav2 + visual** |
| Detection | YOLOv5s | **YOLOv8 (newer)** |
| Tracking | KCF | **CSRT (more accurate)** |

**Your Unique Selling Point:**
> "Unlike KCF-YOLO which requires the target to be initially visible, our system
> autonomously explores an unknown environment to discover and then follow targets,
> combining global path planning (Nav2) with local visual servoing."

## 📝 Build and Test

```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select object_follower

# Source the workspace
source install/setup.bash

# Run the modular node
ros2 run object_follower object_follower_modular

# Run with RViz visualization
ros2 launch object_follower object_follower_launch.py
```
