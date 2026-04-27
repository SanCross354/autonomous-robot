# Autonomous Mobile Robot — Undergraduate Thesis (TA)

> **ROS 2 Humble** | Gazebo | Nav2 | YOLO11 | FSM-based Object Search & Following

A fully integrated autonomous mobile robot system implementing a **Search → Detect → Track → Approach** pipeline using Finite State Machine (FSM) coordination. Includes comparative evaluation of **random search** vs. **waypoint-based (Boustrophedon) search** strategies.

---

## 📦 Packages

| Package | Description |
|---|---|
| `articubot_TA` | Robot URDF, Gazebo worlds, Nav2 config, simulation launch |
| `object_follower` | Main FSM node, visual servo, object selector GUI (PyQt5) |
| `yolobot_recognition` | YOLO11n real-time object detection via camera topic |
| `yolov8_msgs` | Custom ROS 2 message definitions for YOLO inference results |
| `explore_lite` | Frontier-based autonomous exploration (used in random search mode) |

---

## ⚙️ Prerequisites

- **OS**: Ubuntu 22.04
- **ROS 2**: [Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html)
- **Gazebo**: Classic (version 11, ships with ROS 2 Humble desktop)
- **Nav2**: `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`
- **Python dependencies**:

```bash
pip install ultralytics PyQt5 opencv-python
```

- **ROS 2 dependencies** (run from workspace root):

```bash
sudo rosdep init  # skip if already done
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## 🚀 Setup & Build

### 1. Clone the repo

```bash
git clone https://github.com/SanCross354/autonomous-robot.git articubot_TA
cd articubot_TA
```

### 2. Download YOLO model weights

The model file is **not included in the repo** (large binary). Download it manually:

```bash
# Option A: Download yolo11n.pt directly
wget -O src/yolobot_recognition/scripts/yolo11n.pt \
  https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt
```

> ⚠️ The model **must** be placed at: `src/yolobot_recognition/scripts/yolo11n.pt`

### 3. Source ROS 2 and build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> **Tip**: Add `source /opt/ros/humble/setup.bash` to your `~/.bashrc` to avoid running it each session.

---

## 🎮 Running the Simulation

All commands below assume you've sourced the workspace:

```bash
source install/setup.bash
```

### Terminal 1 — Launch Gazebo + Robot + Nav2

```bash
ros2 launch articubot_TA launch_sim.launch.py
```

> Use `world:=/path/to/file.world` to override the default Gazebo world.
> Maps (`map_fix1.yaml`, `map_fix2.yaml`) are in the **workspace root**.

### Terminal 2 — Launch YOLO Detection

```bash
ros2 launch yolobot_recognition launch_yolov8.launch.py
```

### Terminal 3 — Launch Object Follower (FSM + GUI)

```bash
ros2 launch object_follower object_follower_launch.py map_name:=map_fix1
```

Replace `map_name` with `map_fix1` or `map_fix2` depending on the map you loaded.

---

## 🗺️ Maps

Pre-built maps are stored in the **workspace root**:

| File | Description |
|---|---|
| `map_fix1.yaml` / `map_fix1.pgm` | Primary experiment map |
| `map_fix2.yaml` / `map_fix2.pgm` | Secondary experiment map |

Pass the map to Nav2 via the launch file's `map` argument if needed.

---

## 📊 Experiment Data

- `metrics_results.csv` — Raw results from all experimental trials
- `generate_charts.py` — Script to regenerate thesis chart figures

```bash
python3 generate_charts.py
```

---

## 📁 Repository Structure

```
articubot_TA/            ← ROS 2 workspace root
├── src/
│   ├── articubot_TA/        ← Robot config, URDF, worlds, Nav2 params
│   ├── object_follower/     ← FSM, visual servo, GUI nodes
│   ├── yolobot_recognition/ ← YOLO detection node
│   ├── yolov8_msgs/         ← Custom message types
│   └── explore_lite/        ← Frontier exploration
├── map_fix1.yaml / .pgm     ← Experiment maps
├── map_fix2.yaml / .pgm
└── metrics_results.csv      ← Experiment results
```

---

## 📝 License

This project is part of an undergraduate thesis. All rights reserved.
