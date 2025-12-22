#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

# === EDIT PATHS ===
BASE="${HOME}/articubot_TA"
MAP_YAML="${BASE}/map_obstacle1_save.yaml"
NAV2_PARAMS="${BASE}/src/articubot_TA/config/nav2_params.yaml"
WORLD="${BASE}/src/articubot_TA/worlds/map_obstacle1.world"
# ===================

echo "Orchestrator (fixed) starting..."
echo "BASE=${BASE}"
echo "MAP_YAML=${MAP_YAML}"
echo "NAV2_PARAMS=${NAV2_PARAMS}"
echo "WORLD=${WORLD}"

# helper wait for topic
wait_for_topic() {
  TOPIC="$1"; TIMEOUT="${2:-30}"
  echo -n "Waiting for topic ${TOPIC} (timeout ${TIMEOUT}s) ... "
  for i in $(seq 1 $((TIMEOUT*2))); do
    if ros2 topic list | grep -xq "${TOPIC}"; then
      echo "found."
      return 0
    fi
    sleep 0.5
  done
  echo "TIMEOUT"
  return 1
}

# helper lifecycle check
get_lifecycle_state() {
  node="$1"
  out="$(ros2 lifecycle get "$node" 2>/dev/null || true)"
  if [ -z "$out" ]; then
    echo "unknown"
  else
    echo "$out" | awk '{print $1}'
  fi
}

attempt_activate() {
  node="$1"
  state="$(get_lifecycle_state "$node")"
  echo "Lifecycle $node state: $state"
  if [ "$state" = "unknown" ]; then
    echo "  $node not present yet."
    return 1
  fi
  if [ "$state" = "unconfigured" ]; then
    ros2 lifecycle set "$node" configure || true
    sleep 0.5
  fi
  state="$(get_lifecycle_state "$node")"
  if [ "$state" = "inactive" -o "$state" = "configuring" ]; then
    ros2 lifecycle set "$node" activate || true
    sleep 0.5
  fi
  [ "$(get_lifecycle_state "$node")" = "active" ]
}

# 1) Launch Gazebo + robot (launch must publish /clock, tf, /odom, /scan)
ros2 launch articubot_TA launch_sim.launch.py world:="${WORLD}" use_sim_time:=true &
GAZ_PID=$!
echo "Launched Gazebo (pid ${GAZ_PID})"

# 2) Wait for sim clock + essential topics
wait_for_topic "/clock" 20 || echo "Warning: /clock not found"
wait_for_topic "/odom" 30 || echo "Warning: /odom not found"
wait_for_topic "/scan" 30 || echo "Warning: /scan not found"
wait_for_topic "/tf" 20 || echo "Warning: /tf not found"

# small stabilization pause
sleep 1.5

# 3) Start map_server explicitly (safe form - passes the map metadata file as a parameter)
echo "Starting map_server (explicit) ..."
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="${MAP_YAML}" -p use_sim_time:=true &
MAP_PID=$!
sleep 0.6

# 4) Start amcl (explicit)
echo "Starting amcl ..."
ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true &
AMCL_PID=$!
sleep 0.6

# 5) Start lifecycle manager to drive map_server & amcl to active
echo "Starting lifecycle_manager to autostart map_server & amcl ..."
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p use_sim_time:=true -p autostart:=true -p node_names:="['map_server','amcl']" &
LIFEC_PID=$!

# 6) Wait for /map
if wait_for_topic "/map" 20 ; then
  echo "/map is published"
else
  echo "Warning: /map not seen - check 'ros2 topic echo /rosout' and node logs"
fi

# 7) Wait for nodes to become active (map_server and amcl)
for node in "/map_server" "amcl"; do
  # try friendly names
  n="${node#/}"  # remove leading slash for lifecycle get
  for i in $(seq 1 40); do
    s="$(get_lifecycle_state "$n")"
    if [ "$s" = "active" ]; then
      echo "$n is active."
      break
    fi
    attempt_activate "$n" >/dev/null 2>&1 || true
    sleep 0.5
  done
  echo "final state $n: $(get_lifecycle_state "$n")"
done

# 8) Publish a safe /initialpose once (use the map frame)
echo "Publishing /initialpose (one-shot) ..."
ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped \
"{ header: { frame_id: 'map' }, pose: { pose: { position: { x: 0.62, y: -0.78, z: 0.0 }, orientation: { z: 0.0, w: 1.0 } }, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0] } }"

sleep 1.0

# 9) Wait for map -> base_link TF (AMCL should publish it once localized)
echo "Waiting for map->base_link transform..."
for i in $(seq 1 40); do
  if ros2 run tf2_ros tf2_echo map base_link >/dev/null 2>&1 ; then
    echo "map->base_link TF OK"
    break
  fi
  sleep 0.5
done

# 10) Launch navigation stack (planner/controller)
echo "Launching navigation stack (navigation_launch.py) ..."
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:="${NAV2_PARAMS}" &
NAV_PID=$!

echo "Orchestration finished. Useful checks:"
echo "  ros2 topic info --verbose /map"
echo "  ros2 lifecycle get /map_server"
echo "  ros2 topic echo /amcl_pose"
echo "  ros2 run tf2_ros tf2_echo map base_link"
