# Technical Report: Object Follower Debugging & Optimization

# Part I: Research Identity & Architecture

## 0. Research Identity & Project Architecture

### 🎓 Thesis Direction
**Research Topic:**
> "Design and Analysis of an Autonomous Object-Searching Robot Using YOLOv11 and Nav2 Path Planning in a Predefined Map Environment."

**Research Description:**
Developing an autonomous mobile robot in **ROS 2 + Gazebo** that uses **YOLOv11** for object detection and **Nav2** (with SLAM map) for navigation, to autonomously *search* for and *approach* a desired object chosen via GUI.

### 🎯 Research Goals
1.  **Integration:** Integrate YOLOv11 object detection with ROS 2 navigation stack.
2.  **Algorithm Development:** Develop a searching algorithm (random vs systematic/exploration).
3.  **Performance Analysis:** Compare performance using metrics (see detailed breakdown below).

### 📊 Performance Metrics (Detailed Breakdown)

| Metric | Unit | Purpose | How It's Measured |
|--------|------|---------|-------------------|
| **Total Path Length** | meters | Measures navigation efficiency | Integrated from `/odom` topic |
| **Time to Find Object** | seconds | Measures search algorithm speed | From target selection to first detection |
| **Time to Reach Object** | seconds | Measures approach efficiency | From first detection to mission success |
| **Collision/Safety Stops** | count | Measures safety & reliability | Count of Safety Stop triggers |
| **Control Loop Latency** | ms/frame | Measures tracking loop speed | Average processing time per tracking loop iteration |

#### 📏 1. Total Path Length (meters)
**What it measures:** The total distance the robot travels from the moment a target is selected until it successfully reaches the target.

**Why it matters for your thesis:**
- **Efficiency comparison:** A *systematic* search algorithm should have a **shorter** path than a *random* search.
- **Energy proxy:** Longer paths = more energy consumed = less efficient.

**Formula:**
```
path_length = Σ √[(x₂-x₁)² + (y₂-y₁)²]  (integrated over all odometry updates)
```

---

#### ⏱️ 2. Time to Find Object (seconds)
**What it measures:** The time elapsed from when the user clicks a target in the GUI until the robot's camera first detects that object.

**Why it matters for your thesis:**
- **Search algorithm effectiveness:** The primary comparison metric between Random vs Systematic search.
- **Probability of success:** Longer times may indicate the robot is searching inefficiently.

**Formula:**
```
time_to_find = first_detection_timestamp - selection_timestamp
```

---

#### 🏁 3. Time to Reach Object (seconds)
**What it measures:** The time elapsed from first detection until the robot is close enough to declare "MISSION SUCCESS."

**Why it matters for your thesis:**
- **Visual servoing performance:** How quickly the robot can approach once it sees the target.
- **Tracking stability:** Frequent losses during approach would increase this time.

**Formula:**
```
time_to_reach = success_timestamp - first_detection_timestamp
```

---

#### 🛡️ 4. Collision Rate / Safety Stops (count)
**What it measures:** The number of times the Safety Stop feature triggers (obstacle detected < 0.6m while tracking).

**Why it matters for your thesis:**
- **Safety:** A robot that never triggers Safety Stop in a clear environment = good path planning.
- **Obstacle avoidance:** In cluttered environments, this shows how often the robot encounters obstacles.

**Interpretation:**
- **0 stops:** Perfect run (no obstacles encountered).
- **1-3 stops:** Minor obstacles handled safely.
- **4+ stops:** May indicate navigation issues or cluttered environment.

---

#### ⚡ 5. Control Loop Latency (ms/frame)
**What it measures:** How long each iteration of the tracking loop takes to execute.

**Why it matters for your thesis:**
- **Real-time capability:** If computation > 100ms, the robot reacts too slowly.
- **Hardware requirements:** Helps determine minimum CPU/GPU specs.

**Target:** < 50ms average (20 Hz real-time tracking).

### 🏗️ System Architecture (Intersection of Layers)
| Layer | Function | Technology |
| :--- | :--- | :--- |
| **Perception** | Understanding environment | YOLOv11 (vision) + LIDAR (SLAM map) |
| **Decision-Making** | Choosing what to do next | Object-following node + State Machine Logic |
| **Navigation** | Moving efficiently and safely | Nav2 Stack (A*, Dijkstra) |
| **HRI (Human-Robot Interaction)** | User tells robot what to find | Tkinter GUI |

### ⚙️ Core Technical Implementation
-   **Middleware:** ROS 2 Galactic
-   **Simulation:** Gazebo Classic
-   **Robot Platform:** Articubot (Differential Drive)
-   **Key Features:** Hybrid Tracking (Nav2 + Visual Servo), Motion Memory/Freshness Guard, Robust Scanning.

---

---

# Part II: Problem Diagnosis & Debugging Log

## 1. The "Ghost Object" Problem (YOLO Coordinates)
**Analysis:**
The robot was detecting the person, but the bounding box (red box) appeared at the bottom of the screen or in incorrect positions. This caused the interaction to look "broken" even though detections were happening.

**Root Cause:**
In `yolov8_ros2_pt.py`, the YOLO output `xyxy` format (x1, y1, x2, y2) corresponds to (Left, Top, Right, Bottom).
The code was assigning:
- `top = left`
- `left = top`
...and so on. This swapped the X and Y axes of the detection.

**Solution:**
Corrected the variable assignment mapping. This immediately placed the bounding box correctly on the target person.

---

## 2. The "Turning Away" Problem (Nav2 vs. Visual Servoing)
**Analysis:**
When the robot entered the `APPROACHING` state, it would often turn *away* from the target and lose visual contact immediately.

**Why this happened:**
We were using **Nav2** to approach the target. Nav2 is a "path planner" - it calculates a trajectory to get from Point A to Point B. It does *not* care about what the camera sees. If the optimal path required a turn, Nav2 turned the robot, causing the object to leave the camera frame.

**Considered Solutions:**
1.  **Tune Nav2:** Force it to strafe or limit turning? (Too complex, unreliable).
2.  **Visual Servoing:** Directly control wheel velocities based on the object's position in the image.

**Chosen Solution:**
**Visual Servoing (Hybrid Approach).**
- **Long Range (Exploration):** Use Nav2.
- **Visual Contact:** Bypass Nav2 path planning entirely. Use a Proportional (P) controller to turn towards the object and drive forward while keeping it centered.
- **Justification:** Visual servoing guarantees the camera stays pointed at the target, which is critical for object following.

---

## 3. The "Spiraling" Problem (Stale Tracking)
**Analysis:**
When the robot moved, the YOLO detector sometimes missed a frame or two (motion blur, etc.). The robot would start spinning or spiraling uncontrollably.

**Root Cause:**
We were using a `CENTROID` tracker as a fallback.
- **YOLO:** Detects object.
- **Tracker:** "Where is it now?"
- **Problem:** A Centroid tracker (unlike KCF/CSRT) doesn't *predict* motion. It just remembers the *last seen position*. When YOLO missed a detection, the tracker said "It's still right there!", even though the robot had moved. The robot tried to correct for a "ghost" position, creating a feedback loop spiraling.

**Solution:**
We initially attempted to switch to `CSRT` (Discriminative Correlation Filter), but it was not available on the system.
**The Effective Fix (Motion Memory & Freshness Guard):**
We implemented a form of **"Motion Memory"** using the `CENTROID` tracker + a **Freshness Guard**:

1.  **Short-Term Memory (< 0.5s):** Robot trusts the tracker completely (Full speed).
2.  **Stale Memory (0.5s - 1.5s):** Robot "remembers" the object is there but reduces speed (Cautious approach).
3.  **Memory Expired (> 1.5s):** **STOP.** The memory is too old to be safe.
4.  **Long-Term Timeout (> 3.0s):** If memory is fully lost, switch to `SEARCHING`.

**Why this works:** The spiral happens when the robot chases *ghosts* (stale data). By defining these strict time-based conditions, we prevent the robot from acting on obsolete information.

---

## 4. The "Frozen Robot" Problem (State Deadlock)
**Analysis:**
The robot would often get stuck in the `TRACKING` state, simply logging "Detection too old... stopping" forever.

**Root Cause:**
The state machine logic had a gap. It handled "Object Visible" (Move) and "Object Lost Briefly" (Wait), but it didn't have a transition for "Object Lost for a Long Time" while in Tracking mode.

**Solution:**
Added a timeout transition:
- If detection is lost for **> 3.0 seconds**, force a transition to `SEARCHING`.
- This triggers the robot to rotate in place, scanning the environment to re-acquire the target.

---

## 5. The "Negative Box" Problem (Crash/Warnings)
**Analysis:**
Logs were spamming `Invalid bbox for tracker init`.

**Root Cause:**
When an object touched the edge of the image (e.g., coordinate 639 in a 640-wide image), the math to calculate width/height caused rounding errors or negative values after expansion logic.

**Solution:**
Implemented robust clamping in `detection_processor.py`.
- **Clamp:** Force coordinates to be within `[0, width]` and `[0, height]`.
- **Min Size:** Ensure bounding box is at least 20px wide/high.
- **Directional Expansion:** If at the bottom edge, expand *up* instead of down.

---

## Final System Architecture

We arrived at a robust **Hybrid State Automaton**:

1.  **EXPLORING:** Nav2 wanders between waypoints.
2.  **DETECTED:** YOLO spots a target.
    - Architecture Optimization: Skip `APPROACHING` (Nav2).
    - Go straight to `TRACKING` (Visual Servoing).
3.  **TRACKING:**
    - **Fresh Data:** Drive to keep object centered (`cmd_vel`).
    - **Stale Data:** Slow down / Stop.
    - **Lost:** Switch to `SEARCHING`.
## 6. Navigation & System Stability (Nav2 & ROS 2)

### A. The "Out of Bounds" & CPU Overload (Nav2 Tuning)
**Problem:**
1.  **Bounds:** Robot frequently aborted with `Sensor origin is out of map bounds`.
2.  **Overload:** Control loop missed rate (20Hz -> 2.5Hz), causing jerky motion and aborts.

**Diagnosis:**
-   **Margins:** `WAYPOINT_MARGIN=1.0m` was too tight, forcing the robot near map edges.
-   **Costmap:** A 6x6m local costmap was too heavy for the CPU.

**Solution (Hybrid Configuration):**
-   **Costmap:** Optimized to **5x5m** (Sweet spot: safe but lighter than 6x6).
-   **Controller:** Reduced frequency to **10 Hz** (saves CPU, sufficient for this speed).
-   **Margins:** Increased safety margin to **2.5m** to keep robot in valid map areas.
-   **Planner:** Enabled **A\*** (more robust than Dijkstra).

### B. The "Invisible Sensors" (QoS Mismatch)
**Problem:**
The Object Follower node wasn't receiving Lidar or Camera data despite topics being active.
**Log:** `New publisher discovered... offering incompatible QoS`

**Root Cause:**
-   **Simulation:** Publishes `BEST_EFFORT` (standard for high-frequency sensors).
-   **Node:** Requested `RELIABLE` (ROS 2 default).
-   **Result:** ROS 2 protocol mismatch prevented data flow.

**Solution:**
Updated node subscriptions in `scan_handler.py` and `object_follower_modular.py` to use `ReliabilityPolicy.BEST_EFFORT`.

### C. The "Fighting Robot" (Velocity Conflicts)
**Relevant File:** `/home/sancross354/articubot_TA/src/object_follower/launch/object_follower_launch.py`
**Problem:**
Robot moved jerkily or stuttered during 360° scans.

**Root Cause:**
Two nodes were publishing to `/cmd_vel` at the same priority (10):
1.  Nav2 (Navigation)
2.  Object Follower (Scanning)
This caused a "fight" for control.

**Solution:**
-   Retained Nav2 on `/cmd_vel`.
-   Moved Object Follower manual maneuvers (Scanning/Tracking) to a higher-priority topic (e.g., `/cmd_vel_tracking` -> Priority 50 in Twist Mux).
-   **Result:** Smooth, uninterrupted motion.

### D. The "Blind Tracking" (Safety Stop Implementation)
**Relevant File:** `/home/sancross354/articubot_TA/src/object_follower/object_follower/scan_handler.py`
**Problem:**
Visual Servoing bypassed costmaps, making the robot "blind" to obstacles while tracking.
**Solution:**
Integrated a Lidar Safety Check in the tracking loop.
-   **Logic:** If `min(front_scan) < 0.6m`, override tracking and **STOP**.
-   **Result:** Robot successfully stops before hitting obstacles even if the target is further away.

### E. Visual Servo Debugging (Critical Fixes)
**Relevant Files:** 
- `/home/sancross354/articubot_TA/src/object_follower/object_follower/object_follower_modular.py`
- `/home/sancross354/articubot_TA/src/object_follower/object_follower/visual_servo.py`

**Problem 1: Robot Idle Despite Detecting Target**
YOLO occasionally misclassified the person as "bird" or "kite". The code required fresh YOLO detection to servo, even though the **tracker** had a valid lock.

**Solution:**
Changed servo condition from:
```python
if detection_age < 0.5:  # Only YOLO freshness
```
to:
```python
if tracker_active or fresh_detection:  # Trust tracker OR YOLO
```

**Problem 2: Robot Stopped Too Early (target_ratio Mismatch)**
VisualServoController used default `target_ratio=0.32`, but `DETECTION_STOP_THRESHOLD=0.55` in config. Robot stopped approaching at 32% bbox size.

**Solution:**
```python
self._servo = VisualServoController(
    logger=self.get_logger(),
    target_ratio=DETECTION_STOP_THRESHOLD  # 0.55
)
```

**Problem 3: Velocity Too Low to Overcome Friction**
Exponential approach formula produced velocities < 0.05 m/s, insufficient for Gazebo simulation.

**Solution:**
Added minimum velocity floor in `visual_servo.py`:
```python
MIN_LIN_VEL = 0.10  # Minimum velocity floor
if lin > 0.01:
    lin = max(MIN_LIN_VEL, min(MAX_LIN, lin))
```

**Problem 4: Negative/Incorrect Time Metrics**
Code mixed `time.time()` (real clock) with `self._time.now()` (ROS simulation time), causing metrics like "Time to Find: -1766131072s".

**Solution:**
All timestamps changed to use ROS time consistently:
```python
self._selection_start_time = self._time.now()  # Was time.time()
self._reached_time = self._time.now()  # Was time.time()
```

---

## 7. Metrics Recording System
**Relevant File:** `/home/sancross354/articubot_TA/src/object_follower/object_follower/object_follower_modular.py`

### Implementation
Comprehensive thesis-grade metrics recording:

| Metric | How Measured |
|--------|--------------|
| **Path Length** | Integrated from `/odom` topic |
| **Time to Find** | Selection timestamp → First detection |
| **Time to Reach** | First detection → Mission success |
| **Safety Stops** | Counter incremented on each trigger |
| **Control Loop Latency** | `time.perf_counter()` per tracking loop iteration |

### Output
-   **CSV File:** `/home/sancross354/articubot_TA/metrics_results.csv`
-   **Terminal Summary:** Printed on mission success with all metrics
-   **Automatic:** Starts recording when user clicks target in GUI

### Sample Results (RANDOM Search Mode)
```
Trial #1 | Target: person | Mode: RANDOM
--------------------------------------------------
⏱️  Time to Find:    12.45s
🏁 Time to Reach:   30.21s
📍 Total Time:      78.76s
📏 Path Length:     9.68m
🛡️  Safety Stops:    0
⚡ Avg Frame Time:  1.4ms
⚡ Max Frame Time:  9.3ms
```

**Analysis:**
- **Time to Find (12.45s):** Random exploration located target in ~12 seconds
- **Time to Reach (30.21s):** Visual servoing approach phase
- **Path Length (9.68m):** Total distance traveled during search and approach
- **Computation (1.4ms avg):** Excellent real-time capability (well under 50ms target)

---

---

# Part III: Future Recommendations

## 8. Proposed Future Improvements

### A. Smooth Motion Decay
**Analysis:**
When detection becomes stale (0.5s - 1.5s), the speed drops instantly to 30%. This effectively prevents spiraling but causes "jerky" motion.
**Proposed Logic:**
Use a linear decay function: `speed = base_speed * (1.0 - (age / max_age))`.
**Value:** **Medium**. Improves aesthetic quality of the demo.

### B. Smart Search
**Analysis:**
When the object is lost, the robot defaults to a generic rotation. If the object moved slightly Right, and the robot rotates Left, it may lose the object entirely.
**Proposed Logic:**
Store the `last_known_bearing` (left/right of image center). Rotate in that direction first.
**Value:** **High** for robust tracking, but complex to implement/test.
