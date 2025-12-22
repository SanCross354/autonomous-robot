#!/usr/bin/env bash
# Robust orchestrator: start gazebo, ensure /clock, start nav2 (params), ensure /map is published and /map_server activated,
# then publish an initialpose. Adjust the BASE variable if your workspace is elsewhere.

set -euo pipefail
BASE=/home/sancross354/articubot_TA
WORLD="$BASE/src/articubot_TA/worlds/map_obstacle1.world"
NAV2_PARAMS="$BASE/src/articubot_TA/config/nav2_params.yaml"
MAP_SERVER_PARAMS="$BASE/src/articubot_TA/config/map_server_params.yaml"
MAP_YAML="$BASE/map_obstacle1_save.yaml"
SLEEP_AFTER_CLOCK=2.5
TF_FILL_SECONDS=4
WAIT_TOPIC_TIMEOUT=30

echo "Orchestrator starting. BASE=$BASE"

# helper: wait for topic
wait_for_topic() {
  local topic=$1; local to=${2:-$WAIT_TOPIC_TIMEOUT}
  echo -n "Waiting for topic $topic (timeout ${to}s) ..."
  for i in $(seq 1 $((to*2))); do
    if ros2 topic list | grep -x "$topic" >/dev/null 2>&1; then
      echo " found."
      return 0
    fi
    sleep 0.5
  done
  echo " TIMEOUT"
  return 1
}

# helper: wait for node
wait_for_node() {
  local node_substr=$1; local to=${2:-$WAIT_TOPIC_TIMEOUT}
  echo -n "Waiting for node matching '$node_substr' (timeout ${to}s) ..."
  for i in $(seq 1 $((to*2))); do
    if ros2 node list 2>/dev/null | grep -F "$node_substr" >/dev/null 2>&1; then
      echo " found."
      return 0
    fi
    sleep 0.5
  done
  echo " TIMEOUT"
  return 1
}

# sanity checks
if [[ ! -f "$WORLD" ]]; then echo "ERROR: world not found: $WORLD"; exit 1; fi
if [[ ! -f "$NAV2_PARAMS" ]]; then echo "ERROR: nav2 params not found: $NAV2_PARAMS"; exit 1; fi
if [[ ! -f "$MAP_SERVER_PARAMS" ]]; then echo "ERROR: map_server params not found: $MAP_SERVER_PARAMS"; exit 1; fi
if [[ ! -f "$MAP_YAML" ]]; then echo "ERROR: map YAML not found: $MAP_YAML"; exit 1; fi

# 1) start simulation
echo "Launching Gazebo (background)..."
ros2 launch articubot_TA launch_sim.launch.py world:="$WORLD" &
GAZ_PID=$!
echo "Gazebo launched (pid ${GAZ_PID})."

# 2) wait for /clock
if ! wait_for_topic "/clock" 20 ; then
  echo "Warning: /clock not detected after 20s. Continuing but expect sim-time issues."
else
  echo "Sleeping ${SLEEP_AFTER_CLOCK}s to allow nodes to pick up sim time..."
  sleep "$SLEEP_AFTER_CLOCK"
fi

# 3) wait for core topics (odom/scan/tf)
echo "Waiting for odom/scan/tf..."
wait_for_topic "/odom" 30 || echo "Warning: /odom not visible"
wait_for_topic "/scan" 30 || echo "Warning: /scan not visible"
# tf appears as topic /tf, but some systems may not list it quickly
wait_for_topic "/tf" 30 || echo "Warning: /tf not visible"

echo "Sleeping ${TF_FILL_SECONDS}s to let TF buffer populate..."
sleep "$TF_FILL_SECONDS"

# 4) Start nav2 bringup (it will start map_server/amcl/etc using nav2 params)
echo "Starting nav2_bringup (navigation_launch.py) with params_file=$NAV2_PARAMS"
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true autostart:=False slam:=False map:="$MAP_YAML" params_file:="$NAV2_PARAMS" &
NAV_PID=$!
echo "Nav2 bringup started (pid ${NAV_PID})."

# 5) wait for /map topic to appear (map_server should publish after activation)
if wait_for_topic "/map" 20 ; then
  echo "/map topic is present."
else
  echo "Warning: /map topic did NOT appear. We'll try to locate/activate map_server manually."
fi

# 6) try to find map_server node and ensure it's active/publishing
MAP_NODE=$(ros2 node list 2>/dev/null | grep -F map_server | head -n1 || true)
if [[ -z "$MAP_NODE" ]]; then
  echo "map_server node not in node list (yet). Waiting a bit then checking again..."
  sleep 4
  MAP_NODE=$(ros2 node list 2>/dev/null | grep -F map_server | head -n1 || true)
fi

if [[ -z "$MAP_NODE" ]]; then
  echo "map_server not found. As a fallback, launching a standalone map_server with the params file..."
  ros2 run nav2_map_server map_server --ros-args --params-file "$MAP_SERVER_PARAMS" &
  sleep 1
  MAP_NODE=$(ros2 node list 2>/dev/null | grep -F map_server | head -n1 || true)
fi

if [[ -n "$MAP_NODE" ]]; then
  # ensure name starts with slash for lifecycle commands
  if [[ "${MAP_NODE:0:1}" != "/" ]]; then MAP_NODE="/${MAP_NODE}"; fi
  echo "Found map_server node name: $MAP_NODE"
  echo "Lifecycle state (may error if not lifecycle-enabled):"
  ros2 lifecycle get "$MAP_NODE" || true
  echo "Attempting to configure and activate map_server (ignore errors if lifecycle manager is already handling this)..."
  ros2 lifecycle set "$MAP_NODE" configure || true
  ros2 lifecycle set "$MAP_NODE" activate || true

  echo "Checking /map publisher status..."
  sleep 1
  ros2 topic info --verbose /map || true
else
  echo "map_server still not found. Check 'ros2 node list' and nav2 logs in $HOME/.ros/log for errors."
fi

# 7) If /map still not being published, try the load_map service if available
MAP_PUB_COUNT=$(ros2 topic info --verbose /map 2>/dev/null | grep -E 'Publisher count' || true)
if echo "$MAP_PUB_COUNT" | grep -q 'Publisher count: 0'; then
  echo "/map has publisher count 0. Trying load_map service (if available) with $MAP_YAML ..."
  if ros2 service list | grep -q '/map_server/load_map'; then
    echo "Calling /map_server/load_map ..."
    ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: '$MAP_YAML'}" || echo "load_map call failed"
  else
    echo "load_map service not present yet."
  fi
fi

# 8) Publish an initialpose (36 element covariance). No header.stamp -> AMCL will use latest.
echo "Publishing a one-shot initial pose on /initialpose"
ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped \
"{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.62, y: -0.78, z: 0.0}, orientation: {z: 0.0, w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0]}}"

echo "Done. Check AMCL logs for 'initialPoseReceived' and 'Setting pose'."
echo "Helpful checks: ros2 topic info --verbose /map, ros2 node list | grep map_server, ros2 lifecycle get /map_server"
