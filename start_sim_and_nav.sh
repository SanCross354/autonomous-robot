#!/usr/bin/env bash
set -euo pipefail

# === EDIT THESE ===
MAP_YAML="/home/sancross354/articubot_TA/map_obstacle1_save.yaml"
NAV2_PARAMS="/home/sancross354/articubot_TA/src/articubot_TA/config/nav2_params.yaml"
WORLD="$(pwd)/src/articubot_TA/worlds/map_obstacle1.world"
AMCL_NODE="/amcl"
MAP_SERVER_NODE="/map_server"
# ==================

# helper
echo_stderr() { echo "$@" >&2; }

wait_for_topic() {
  TOPIC="$1"; TIMEOUT="${2:-30}"
  echo "Waiting for topic ${TOPIC} (timeout ${TIMEOUT}s)..."
  for i in $(seq 1 $((TIMEOUT*2))); do
    if ros2 topic list | grep -x "${TOPIC}" >/dev/null 2>&1; then
      echo "  ${TOPIC} found."
      return 0
    fi
    sleep 0.5
  done
  echo "  TIMEOUT waiting for ${TOPIC}"
  return 1
}

get_lifecycle_state() {
  node="$1"
  out=$(ros2 lifecycle get "$node" 2>/dev/null || true)
  # returns first token or "unknown"
  if [ -z "$out" ]; then
    echo "unknown"
  else
    echo "$out" | awk '{print $1}'
  fi
}

attempt_activate() {
  node="$1"
  # read and attempt to drive to active
  state=$(get_lifecycle_state "$node")
  echo "Lifecycle $node state: $state"
  if [ "$state" = "unknown" ]; then
    echo "  $node not present yet."
    return 1
  fi
  if [ "$state" = "unconfigured" ]; then
    echo "  configuring $node ..."
    ros2 lifecycle set "$node" configure || true
    sleep 0.5
  fi
  state=$(get_lifecycle_state "$node")
  if [ "$state" = "inactive" -o "$state" = "configuring" ]; then
    echo "  activating $node ..."
    ros2 lifecycle set "$node" activate || true
    sleep 0.5
  fi
  state=$(get_lifecycle_state "$node")
  echo "  after attempts, state=$state"
  if [ "$state" = "active" ]; then
    return 0
  else
    return 1
  fi
}

wait_for_active() {
  node="$1"; TIMEOUT="${2:-20}"
  echo "Waiting for lifecycle node $node to become active..."
  for i in $(seq 1 $((TIMEOUT*2))); do
    state=$(get_lifecycle_state "$node")
    if [ "$state" = "active" ]; then
      echo "  $node is active."
      return 0
    fi
    # attempt to progress transitions (fallback)
    attempt_activate "$node" >/dev/null 2>&1 || true
    sleep 0.5
  done
  echo "  timeout waiting for $node to become active (last state: $(get_lifecycle_state "$node"))"
  return 1
}

wait_for_tf() {
  SRC="$1"; DST="$2"; TIMEOUT="${3:-20}"
  echo "Waiting for TF ${SRC} -> ${DST} ..."
  for i in $(seq 1 $((TIMEOUT*2))); do
    if timeout 1 ros2 run tf2_ros tf2_echo "$SRC" "$DST" >/dev/null 2>&1 ; then
      echo "  TF ${SRC}->${DST} available."
      return 0
    fi
    sleep 0.5
  done
  echo "  TF ${SRC}->${DST} not found (timeout)."
  return 1
}

# 1) start Gazebo (sim)
echo "Starting Gazebo (sim) ..."
ros2 launch articubot_TA launch_sim.launch.py world:="${WORLD}" use_sim_time:=true &
GAZ_PID=$!
echo " Gazebo PID=${GAZ_PID}"

# 2) wait for sim clock + base topics
wait_for_topic "/clock" 20 || echo "Warning: /clock not found."
wait_for_topic "/odom" 30 || echo "Warning: /odom not found."
wait_for_topic "/scan" 30 || echo "Warning: /scan not found."
# tf topic
wait_for_topic "/tf" 20 || echo "Warning: /tf not found."

sleep 1.0

# 3) Launch nav2 localization (map_server + amcl)
echo "Launching nav2 localization (map_server + amcl) ..."
ros2 launch nav2_bringup localization_launch.py use_sim_time:=true map:="${MAP_YAML}" params_file:="${NAV2_PARAMS}" &
LOC_PID=$!
echo " nav2 localization pid ${LOC_PID}"

# 4) Wait for /map
if wait_for_topic "/map" 20 ; then
  echo "/map is being published."
else
  echo_stderr "Warning: /map not appearing. Check 'ros2 topic echo /rosout' or 'ros2 lifecycle get $MAP_SERVER_NODE'."
fi

# 5) Ensure map_server active & amcl active
wait_for_active "$MAP_SERVER_NODE" 20 || echo_stderr "map_server not active after attempts."
wait_for_active "$AMCL_NODE" 20 || echo_stderr "amcl not active after attempts."

# 6) Publish a safe one-shot initialpose (no header.stamp) — adjust x,y to be inside the map
echo "Publishing /initialpose one-shot (no header.stamp) ..."
INIT_X=0.62; INIT_Y=-0.78
ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped \
"{
  header: { frame_id: 'map' },
  pose: {
    pose: {
      position: { x: ${INIT_X}, y: ${INIT_Y}, z: 0.0 },
      orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
    },
    covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0]
  }
}"

sleep 1.0

# 7) Wait for map->base_link
if wait_for_tf "map" "base_link" 20 ; then
  echo "map->base_link TF present."
else
  echo_stderr "map->base_link TF still missing. Dump recent AMCL logs: run 'ros2 topic echo /rosout | grep -i amcl -n' and 'ros2 topic echo /amcl_pose --once'."
fi

# 8) Launch navigation (planner/controller)
echo "Launching nav2 navigation (planner/controller) ..."
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:="${NAV2_PARAMS}" &
NAV_PID=$!
echo " nav pid ${NAV_PID}"

echo "Orchestration done. Useful checks:"
echo "ros2 topic info --verbose /map"
echo "ros2 topic echo /amcl_pose --once"
echo "ros2 run tf2_ros tf2_echo map base_link"
echo "In RViz -> Map display -> QoS choose 'System Default' or 'Transient Local' (map is transient_local)."
