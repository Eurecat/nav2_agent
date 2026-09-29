#!/usr/bin/env bash
# Run nav2_agent with a simulated TurtleBot 4 and Nav2 in the development container.
#
# Usage:
#   scripts/sim_demo.sh start [--gazebo]   Start the simulation, Nav2 and nav2_agent
#   scripts/sim_demo.sh send "<command>"   Send a command and print its progress
#   scripts/sim_demo.sh logs               Follow the logs
#   scripts/sim_demo.sh stop               Stop the container
#
# The default simulation is Nav2's loopback simulator: kinematic, no physics, shown in RViz.
# --gazebo runs the full Gazebo simulation instead.
#
# Environment:
#   LLM_BASE_URL  OpenAI-compatible endpoint (default http://localhost:8080/v1)
#   LLM_MODEL     Model name on the server (default gemma-4-e4b)
#   LLM_API_KEY   API key (default EMPTY)
set -euo pipefail

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
COMPOSE_FILE="$REPO_DIR/docker/docker-compose.yml"
CONTAINER=nav2_agent_jazzy
WORKSPACE=/home/user/workspace
LOG_DIR=log/sim_demo
ROS_SETUP="source /opt/ros/jazzy/setup.bash && cd $WORKSPACE && if [ -f install/setup.bash ]; then source install/setup.bash; fi"

LLM_BASE_URL="${LLM_BASE_URL:-http://localhost:8080/v1}"
LLM_MODEL="${LLM_MODEL:-gemma-4-e4b}"
LLM_API_KEY="${LLM_API_KEY:-EMPTY}"

run() {
  docker exec "$CONTAINER" bash -c "$ROS_SETUP && $1"
}

wait_for() {
  local description="$1" check="$2" timeout="$3"
  for _ in $(seq 1 "$timeout"); do
    if run "$check" >/dev/null 2>&1; then
      return 0
    fi
    if run "grep -qs '\[ERROR\] \[launch\]' $LOG_DIR/*.log" >/dev/null 2>&1; then
      echo "Launch failed while waiting for $description. See $LOG_DIR/ for logs." >&2
      exit 1
    fi
    sleep 1
  done
  echo "Timed out waiting for $description. See $LOG_DIR/ for logs." >&2
  exit 1
}

start() {
  local simulation=loopback
  if [[ "${1:-}" == "--gazebo" ]]; then
    simulation=gazebo
  fi

  if ! docker image inspect nav2_agent:jazzy >/dev/null 2>&1; then
    echo "Building the nav2_agent:jazzy image..."
    (cd "$REPO_DIR/docker" && ./build.sh)
  fi
  if ! curl -s -m 3 -o /dev/null "$LLM_BASE_URL/models"; then
    echo "Warning: no model server reachable at $LLM_BASE_URL. Commands will fail until it is available." >&2
  fi

  xhost +local:docker >/dev/null 2>&1 || true
  docker compose -f "$COMPOSE_FILE" up -d >/dev/null
  run "mkdir -p $LOG_DIR"

  echo "Building workspace..."
  run "colcon build --symlink-install > $LOG_DIR/build.log 2>&1" || {
    echo "Build failed. See $LOG_DIR/build.log." >&2
    exit 1
  }

  echo "Starting $simulation simulation and Nav2..."
  if [[ "$simulation" == "gazebo" ]]; then
    run "nohup ros2 launch nav2_bringup tb4_simulation_launch.py headless:=False > $LOG_DIR/simulation.log 2>&1 &"
  else
    run "nohup ros2 launch nav2_bringup tb4_loopback_simulation.launch.py > $LOG_DIR/simulation.log 2>&1 &"
  fi
  wait_for "the Nav2 lifecycle manager" "grep -q 'Managed nodes are active' $LOG_DIR/simulation.log" 180

  if [[ "$simulation" == "loopback" ]]; then
    run "ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
      '{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}}}'" >/dev/null
  fi
  wait_for "the map -> base_link transform" "timeout 3 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -q Translation" 60

  echo "Starting nav2_agent..."
  run "nohup ros2 run nav2_agent nav2_agent_node --ros-args \
    --params-file install/nav2_agent/share/nav2_agent/config/agent_params.yaml \
    -p use_sim_time:=true -p llm_base_url:=$LLM_BASE_URL -p llm_model:=$LLM_MODEL -p llm_api_key:=$LLM_API_KEY \
    > $LOG_DIR/nav2_agent.log 2>&1 &"
  wait_for "nav2_agent" "ros2 action list | grep -q /nav2_agent/execute_command" 30

  echo
  echo "Ready. Send a command:"
  echo "  scripts/sim_demo.sh send \"Move 2 meters forward\""
}

send() {
  if [[ $# -eq 0 ]]; then
    echo "Usage: scripts/sim_demo.sh send \"<command>\"" >&2
    exit 1
  fi
  local tty_flags=-i
  if [[ -t 0 ]]; then
    tty_flags=-it
  fi
  docker exec $tty_flags "$CONTAINER" bash -c "$ROS_SETUP && ros2 run nav2_agent send \"\$@\"" _ "$@"
}

logs() {
  docker exec "$CONTAINER" bash -c "cd $WORKSPACE && tail -n 20 -f $LOG_DIR/simulation.log $LOG_DIR/nav2_agent.log"
}

stop() {
  docker compose -f "$COMPOSE_FILE" down >/dev/null
  echo "Stopped."
}

case "${1:-}" in
  start) shift; start "$@" ;;
  send) shift; send "$@" ;;
  logs) logs ;;
  stop) stop ;;
  *)
    sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
    ;;
esac
