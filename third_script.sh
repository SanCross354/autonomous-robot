#!/usr/bin/env bash
# start_sim_and_nav_full.sh
# Orchestrates Gazebo -> Nav2 (map_server + amcl lifecycle) -> nav2 navigation.
# Robust checks: explicit lifecycle_manager fallback, explicit map_server fallback,
# wait for TF, publish initial pose, then launch navigation.

set -euo pipefail
IFS=$'\n\t'

# ===== USER EDITS (absolute paths recommended) =====
BASE="${HOME}/articubot_TA"                      # root of your project
MAP_YAML="${BASE}/map_obstacle1_save.yaml"      # absolute path to your map yaml
NAV2_PARAMS="${BASE}/src/articubot_TA/config/nav2_params.yaml"
WORLD="${BASE}/src/articubot_TA/worlds/map_obstacle1.world"
INIT_X=0.62
INIT_Y=-0.78
# ===================================================

echo "Orchestrator starting..."
echo "BASE: ${BASE}"
echo "MAP_YAML: ${MAP_YAML}"
echo "NAV2_PARAMS: ${NAV2_PARAMS}"
echo "WORLD: ${WORLD}"

# sanity checks
for f in "${MAP_YAML}" "${NAV2_PARAMS}" "${WORLD}"; do
  if [ ! -e "$f" ]; then
    echo "ERROR: required file not found: $f" >&2
    exit 1
  fi
done

# helper functions
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

get_lifecycle_state() {
  node="$1"
  out="$(ros2 lifecycle get "$node" 2>/dev/null || true)"
  if [ -z "$out" ]; then
    echo "unknown"
  else
    # returns the state token, e.g. "active", "unconfigured", "inactive"
    echo "$out" | awk '{print $1}'
  fi
}

attempt_configure_activate() {
  node="$1"
  state="$(get_lifecycle_state "$node")"
  echo "Lifecycle $node state: $state"
  if [ "$state" = "unknown" ]; then
    echo "  $node not present."
    return 1
  fi
  if [ "$state" = "unconfigured" ]; then
    echo "  configuring $node ..."
    ros2 lifecycle set "$node" configure || true
    sleep 0.5
  fi
  state="$(get_lifecycle_state "$node")"
  if [ "$state" = "inactive" -o "$state" = "configuring" ]; then
    echo "  activating $node ..."
    ros2 lifecycle set "$node" activate || true
    sleep 0.5
  fi
  state="$(get_lifecycle_state "$node")"
  echo "  now $node state: $state"
  [ "$state" = "active" ]
}

wait_for_lifecycle_active() {
  node="$1"; TIMEOUT="${2:-20}"
  echo "Waiting for lifecycle node $node to become active (timeout ${TIMEOUT}s)..."
  for i in $(seq 1 $((TIMEOUT*2))); do
    if [ "$(get_lifecycle_state "$node")" = "active" ]; then
      echo "  $node is active."
      return 0
    fi
    attempt_configure_activate "$node" >/dev/null 2>&1 || true
    sleep 0.5
  done
  echo "  timeout waiting for $node to become active. last state: $(get_lifecycle_state "$node")"
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

# --------------------------
# 1) Launch Gazebo (your launch should publish robot_description, odom, scan, tf, /clock, etc.)
# --------------------------
echo "Launching Gazebo + robot_state_publisher..."
ros2 launch articubot_TA launch_sim.launch.py world:="${WORLD}" use_sim_time:=true &
GAZ_PID=$!
echo " Gazebo pid=${GAZ_PID}"

# 2) Wait for sim clock and essential topics
wait_for_topic "/clock" 20 || echo "Warning: /clock not found after 20s"
wait_for_topic "/odom" 30 || echo "Warning: /odom not found after 30s"
wait_for_topic "/scan" 30 || echo "Warning: /scan not found after 30s"
# tf is present in ros2 topic list as /tf
wait_for_topic "/tf" 20 || echo "Warning: /tf not found after 20s"
sleep 1.0

# --------------------------
# 3) Launch localization (prefer standard localization_launch which starts map_server+amcl)
# --------------------------
echo "Launching nav2 localization (localization_launch.py) with map and params..."
ros2 launch nav2_bringup localization_launch.py \
    use_sim_time:=true \
    map:="${MAP_YAML}" \
    params_file:="${NAV2_PARAMS}" &
LOC_PID=$!
echo " nav2 localization pid=${LOC_PID}"

# short sleep for processes to start
sleep 1.0

# --------------------------
# 4) Ensure map_server + amcl are present and active; if not, provide fallback (explicit map_server + lifecycle manager)
# --------------------------
# preferred node names (no leading slash) - adjust if your system shows namespaced nodes
MAP_NODE_CANDIDATES=( map_server /map_server )
AMCL_NODE_CANDIDATES=( amcl /amcl )

find_node_name() {
  candidates=("$@")
  for n in "${candidates[@]}"; do
    if ros2 node list | grep -xq "$n"; then
      echo "$n"
      return 0
    fi
  done
  return 1
}

MAP_NODE="$(find_node_name "${MAP_NODE_CANDIDATES[@]}" || true)"
AMCL_NODE="$(find_node_name "${AMCL_NODE_CANDIDATES[@]}" || true)"

if [ -z "${MAP_NODE}" ]; then
  echo "map_server not found in node list after localization_launch. Will start explicit map_server as fallback."
  ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="${MAP_YAML}" -p use_sim_time:=true &
  sleep 0.8
  MAP_NODE="$(find_node_name "${MAP_NODE_CANDIDATES[@]}" || true)"
fi

# Start a lifecycle manager if map_server or amcl aren't driven to active automatically
echo "Launching lifecycle_manager fallback (nav2_lifecycle_manager) to autostart map_server & amcl..."
ros2 run nav2_lifecycle_manager lifecycle_manager \
  --ros-args -p use_sim_time:=true -p autostart:=true -p node_names:="['map_server','amcl']" &
LIFEC_PID=$!
sleep 0.7

# 5) Wait for /map topic
if wait_for_topic "/map" 20 ; then
  echo "/map topic is published."
else
  echo "Warning: /map not seen in 20s. Inspect 'ros2 topic echo /rosout' and 'ros2 node list'."
fi

# 6) Wait for map_server and amcl to become active
if [ -n "${MAP_NODE}" ]; then
  wait_for_lifecycle_active "${MAP_NODE}" 20 || echo "Warning: map_server did not become active."
else
  echo "map_server node still not detected by name; continuing but things may fail."
fi

# Try to discover AMCL name again
if [ -z "${AMCL_NODE}" ]; then
  AMCL_NODE="$(find_node_name "${AMCL_NODE_CANDIDATES[@]}" || true)"
fi
if [ -n "${AMCL_NODE}" ]; then
  wait_for_lifecycle_active "${AMCL_NODE}" 20 || echo "Warning: amcl did not become active."
else
  echo "amcl node not detected; you may need to run amcl manually (ros2 run nav2_amcl amcl ...)."
fi

# 7) Publish a safe initialpose (one-shot). No header.stamp -> AMCL uses latest available TF/time.
echo "Publishing /initialpose one-shot (map frame) at x=${INIT_X}, y=${INIT_Y} ..."
ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped \
"{ header: { frame_id: 'map' }, pose: { pose: { position: { x: ${INIT_X}, y: ${INIT_Y}, z: 0.0 }, orientation: { z: 0.0, w: 1.0 } }, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0] } }" || true

sleep 1.0

# 8) Wait for map -> base_link transform (AMCL should be publishing appropriate TF once it has a pose)
if wait_for_tf "map" "base_link" 20 ; then
  echo "map->base_link TF present."
else
  echo "Warning: map->base_link TF missing after attempts. Check 'ros2 topic echo /rosout | grep -i amcl' and '/tf' topic."
fi

# 9) Finally start navigation (planner/controller stack)
echo "Launching nav2 navigation (navigation_launch.py) with params..."
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=true \
    params_file:="${NAV2_PARAMS}" &
NAV_PID=$!
echo " nav pid=${NAV_PID}"

echo ""
echo "Orchestration finished. Useful checks:"
echo "  ros2 topic info --verbose /map"
echo "  ros2 topic echo /amcl_pose"
echo "  ros2 run tf2_ros tf2_echo map base_link"
echo ""
echo "If RViz does not show the map: in Map display -> QoS set 'Transient Local' or 'System Default'."
echo "If map_server or amcl are not active: run 'ros2 node list' and 'ros2 lifecycle get <node>' to inspect."

exit 0
