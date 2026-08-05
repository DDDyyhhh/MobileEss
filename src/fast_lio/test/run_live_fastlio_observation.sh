#!/usr/bin/env bash
# Run FAST-LIO only against native live Mid-360 input for a bounded read-only window.
set -eo pipefail

output_dir=${1:?usage: run_live_fastlio_observation.sh OUTPUT_DIRECTORY}
config=${FAST_LIO_REAL_CONFIG:-/etc/mower/fast_lio/mid360_real.yaml}
mkdir -p "$output_dir"

set +u
source /opt/ros/humble/setup.bash
source /opt/mower_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
set -u

observer_pid=
lio_pid=

stop_process() {
  local pid=${1:-}
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
}

cleanup() {
  stop_process "$observer_pid"
  stop_process "$lio_pid"
}
trap cleanup EXIT

for file in \
  "$config" \
  /etc/mower/local_ekf.yaml \
  /etc/mower/global_ekf.yaml \
  /etc/mower/navsat_transform.yaml \
  /etc/mower/t13-coordinate-contract.json; do
  test -s "$file" || { printf 'missing required runtime file: %s\n' "$file" >&2; exit 66; }
done

if ps -eo args= | grep -E 'fastlio_mapping|ros2 bag|ekf_node|navsat_transform|localization_tf_owner|static_transform_publisher|planner|executor|nav2|can_bridge|blade|unlock' | grep -v grep; then
  printf '%s\n' 'prohibited process is already active' >&2
  exit 67
fi

for topic in /cmd_vel /blade/cmd /blade/command /control/unlock /motion/unlock /tf /tf_static; do
  if ros2 topic info "$topic" -v 2>/dev/null | grep -q 'Publisher count: [1-9]'; then
    printf 'prohibited publisher is active on %s\n' "$topic" >&2
    exit 68
  fi
done

rm -f "$output_dir"/{odometry.json,observer.log,fastlio.log,topic-info.txt,topic-hz.txt,alive-before-stop.txt}

python3 /opt/mower_ws/src/fast_lio/test/replay_odometry_observer.py \
  --duration 35 --output "$output_dir/odometry.json" \
  >"$output_dir/observer.log" 2>&1 &
observer_pid=$!

sleep 2
ros2 launch fast_lio fast_lio_real.launch.py \
  config_file:="$config" \
  use_sim_time:=false \
  publish_tf:=false \
  runtime_mapping_diagnostics:=true \
  >"$output_dir/fastlio.log" 2>&1 &
lio_pid=$!

sleep 2
ros2 topic info /fast_lio/odometry -v >"$output_dir/topic-info.txt" 2>&1 || true
timeout 30 ros2 topic hz /fast_lio/odometry >"$output_dir/topic-hz.txt" 2>&1 || true

if wait "$observer_pid"; then
  observer_pid=
else
  observer_status=$?
  observer_pid=
  exit "$observer_status"
fi

if kill -0 "$lio_pid" 2>/dev/null; then
  printf 'true\n' >"$output_dir/alive-before-stop.txt"
else
  printf 'false\n' >"$output_dir/alive-before-stop.txt"
fi
stop_process "$lio_pid"
lio_pid=
