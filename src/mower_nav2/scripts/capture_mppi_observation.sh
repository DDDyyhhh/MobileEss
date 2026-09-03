#!/usr/bin/env bash
# Capture one observation-only MPPI dynamic-obstacle reproduction.
# Run only after launching mower_navigation.launch.py with enable_motion:=false.
set -euo pipefail

output_root="${1:-$HOME/mppi_observations}"
duration_sec="${2:-20}"
mode="${3:-performance}"
goal_timeout_sec="${4:-90}"

# With enable_motion:=false the SimpleProgressChecker intentionally aborts a
# stationary FollowPath after 15 seconds. Candidate capture must complete
# inside that observation-only window.
if [[ "$mode" == visualize && "$duration_sec" -ge 15 ]]; then
  echo "visualize duration must be below 15 s while enable_motion:=false" >&2
  exit 2
fi

if ! [[ "$duration_sec" =~ ^[1-9][0-9]*$ ]]; then
  echo "duration must be a positive integer (seconds)" >&2
  exit 2
fi
if ! [[ "$goal_timeout_sec" =~ ^[1-9][0-9]*$ ]]; then
  echo "goal timeout must be a positive integer (seconds)" >&2
  exit 2
fi
if [[ "$mode" != performance && "$mode" != visualize ]]; then
  echo "mode must be 'performance' or 'visualize'" >&2
  exit 2
fi

# Do not pipe `ros2 node list` into `grep -q`: grep closes early after a match,
# which makes Humble ros2cli raise BrokenPipeError under `set -o pipefail`.
# Capture the graph once, then search the completed output locally.
node_list="$(ros2 node list)"
node_exists() {
  grep -Fxq "$1" <<< "$node_list"
}

if node_exists '/cmd_vel_can_bridge'; then
  cat >&2 <<'EOF'
Refusing capture: /cmd_vel_can_bridge is running.
Stop the stack and relaunch with enable_motion:=false before collecting MPPI
performance or candidate-trajectory evidence. This script is observation-only.
EOF
  exit 1
fi

for node in /controller_server /local_costmap/local_costmap /collision_gate; do
  if ! node_exists "$node"; then
    echo "Required node is absent: $node" >&2
    exit 1
  fi
done

# Linux task comm names are limited to 15 bytes, so controller_server is
# commonly shown as "controller_ser". Match its executable arguments instead
# of using pgrep -x on the truncated comm name.
controller_pid="$(pgrep -fo '(/|^)controller_server([[:space:]]|$)' || true)"
if [[ -z "$controller_pid" ]]; then
  echo "Unable to find controller_server process for CPU capture" >&2
  echo "Candidates were:" >&2
  pgrep -af 'controller_ser|controller_server' >&2 || true
  exit 1
fi

run_id="$(date -u +%Y%m%dT%H%M%SZ)_${mode}"
run_dir="$output_root/$run_id"
mkdir -p "$run_dir"

# The parameter is dynamically handled by Humble MPPI. Always switch it back
# off so a debug capture cannot consume normal runtime CPU/bandwidth. Also stop
# child recorders if validation or an operator interrupt ends the script early.
bag_pid=""
top_pid=""
cleanup() {
  [[ -z "$top_pid" ]] || kill "$top_pid" 2>/dev/null || true
  [[ -z "$bag_pid" ]] || kill -INT "$bag_pid" 2>/dev/null || true
  [[ -z "$bag_pid" ]] || wait "$bag_pid" 2>/dev/null || true
  ros2 param set /controller_server FollowPath.visualize false >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

ros2 param dump /controller_server > "$run_dir/controller_server_params.yaml"
ros2 param dump /local_costmap/local_costmap > "$run_dir/local_costmap_params.yaml"
ros2 node info /controller_server > "$run_dir/controller_server_node_info.txt"
ros2 topic info --verbose /cmd_vel_nav > "$run_dir/cmd_vel_nav_topic_info.txt"
ros2 topic info --verbose /cmd_vel > "$run_dir/cmd_vel_topic_info.txt"
ros2 topic info --verbose /cmd_vel_gated > "$run_dir/cmd_vel_gated_topic_info.txt"
ros2 param get /controller_server controller_frequency > "$run_dir/controller_frequency.txt"
ros2 param get /controller_server FollowPath.model_dt > "$run_dir/model_dt.txt"
ros2 param get /controller_server FollowPath.time_steps > "$run_dir/time_steps.txt"
ros2 param get /controller_server FollowPath.batch_size > "$run_dir/batch_size.txt"
ros2 param get /controller_server FollowPath.visualize > "$run_dir/visualize_initial.txt"

# The Nav2 action status topic is hidden. Retain it as evidence, but do not
# use it as the arming signal: rosbag2 and ROS CLI discovery can suppress hidden
# action topics. A newly received /cmd_vel_nav is non-hidden and is produced
# only when controller_server is actively computing FollowPath / MPPI output.
ros2 bag record --include-hidden-topics -o "$run_dir/bag" \
  /trajectories \
  /transformed_global_plan \
  /local_plan \
  /plan \
  /local_costmap/costmap_raw \
  /nav2/mid360_obstacles \
  /odometry/filtered \
  /cmd_vel_nav \
  /cmd_vel \
  /cmd_vel_gated \
  /collision_gate/diagnostics \
  /follow_path/_action/status &
bag_pid=$!
sleep 2

if [[ "$mode" == visualize ]]; then
  ros2 param set /controller_server FollowPath.visualize true | tee "$run_dir/visualize_enable.txt"
  ros2 param get /controller_server FollowPath.visualize | tee "$run_dir/visualize_verified.txt"
  grep -Fqx 'Boolean value is: True' "$run_dir/visualize_verified.txt" || {
    echo 'MPPI visualisation did not enable; refusing an invalid candidate capture.' >&2
    exit 1
  }
else
  ros2 param set /controller_server FollowPath.visualize false >/dev/null
fi

# Do not start the duration clock while Nav2 is idle or in recovery. A newly
# published /cmd_vel_nav proves controller_server is actually calling MPPI for
# FollowPath. The operator may send the goal after this script starts; rosbag
# is already subscribed before this wait begins.
echo "Waiting up to ${goal_timeout_sec}s for an active MPPI /cmd_vel_nav command..."
active_command="$run_dir/first_cmd_vel_nav.yaml"
if ! timeout "${goal_timeout_sec}s" ros2 topic echo \
    /cmd_vel_nav \
    --qos-profile system_default \
    --once > "$active_command"; then
  echo "Timed out waiting for an active /cmd_vel_nav command." >&2
  echo "Send a NavigateToPose goal while this script is waiting, then retry." >&2
  exit 1
fi

# Begin the CPU sample only after MPPI has produced a command, so the report is
# tied to real controller computation rather than Nav2 recovery behavior.
sample_count=$((duration_sec * 2 + 2))
top -b -H -d 0.5 -n "$sample_count" -p "$controller_pid" > "$run_dir/controller_top.txt" &
top_pid=$!

echo "Capture $run_id is armed in $mode mode after MPPI command output."
echo "Place the obstacle now; recording active control for ${duration_sec}s."
sleep "$duration_sec"

# SIGINT lets rosbag2 flush metadata/indexes before the artifact is inspected.
kill -INT "$bag_pid" 2>/dev/null || true
wait "$bag_pid" || true
bag_pid=""
wait "$top_pid" || true
top_pid=""
ros2 param set /controller_server FollowPath.visualize false | tee "$run_dir/visualize_disable.txt"
trap - EXIT INT TERM

echo "Saved observation: $run_dir"
echo "Check cmd_vel_nav timing and controller_top.txt before candidate tuning."
echo "In visualize mode, review /trajectories (Candidate and Optimal Trajectory"
echo "namespaces), transformed_global_plan, and raw local costmap. Humble MPPI"
echo "does not publish /local_plan. Keep this capture below the 15 s stationary"
echo "progress-checker allowance used by enable_motion:=false."
