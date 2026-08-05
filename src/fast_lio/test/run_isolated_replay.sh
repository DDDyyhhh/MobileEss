#!/usr/bin/env bash
# Replay a recorded Mid-360/IMU bag into an isolated FAST-LIO namespace.
# This is diagnostic-only: it creates no command, TF, EKF, navsat, CAN, or blade nodes.
set -eo pipefail

if [[ $# -ne 2 ]]; then
  echo "usage: $0 BAG_DIRECTORY OUTPUT_DIRECTORY" >&2
  exit 64
fi

bag_dir=$1
output_dir=$2
config=${FAST_LIO_REPLAY_CONFIG:-/etc/mower/fast_lio/mid360_real.yaml}
if [[ ! -f "$bag_dir/metadata.yaml" ]]; then
  echo "rosbag metadata missing: $bag_dir/metadata.yaml" >&2
  exit 66
fi
mkdir -p "$output_dir"

source /opt/ros/humble/setup.bash
source /opt/mower_ws/install/setup.bash
set -u

stop_process() {
  local pid=${1:-}
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
}

cleanup() {
  stop_process "${observer_pid:-}"
  stop_process "${lio_pid:-}"
}
trap cleanup EXIT

python3 /opt/mower_ws/src/fast_lio/test/replay_odometry_observer.py \
  --duration 35 --output "$output_dir/odometry.json" \
  >"$output_dir/observer.log" 2>&1 &
observer_pid=$!

/opt/mower_ws/install/lib/fast_lio/fastlio_mapping \
  --ros-args --params-file "$config" \
  -p common.lid_topic:=/replay/livox/lidar \
  -p common.imu_topic:=/replay/livox/imu \
  -p runtime_mapping_diagnostics:=true \
  >"$output_dir/fastlio.log" 2>&1 &
lio_pid=$!

# Let the observer and mapping node discover each other before input begins.
sleep 2
ros2 bag play "$bag_dir" \
  --remap /livox/lidar:=/replay/livox/lidar /livox/imu:=/replay/livox/imu \
  >"$output_dir/bag-play.log" 2>&1

sleep 2
if kill -0 "$lio_pid" 2>/dev/null; then
  lio_alive_before_observer_summary=true
else
  lio_alive_before_observer_summary=false
fi

# The observer samples endpoint state while FAST-LIO is still alive, then writes
# its report at the end of its configured observation period.
if wait "$observer_pid"; then
  observer_pid=
else
  observer_status=$?
  observer_pid=
  echo "odometry observer exited with status $observer_status" >&2
  exit "$observer_status"
fi

if kill -0 "$lio_pid" 2>/dev/null; then
  lio_alive_before_stop=true
else
  lio_alive_before_stop=false
fi
stop_process "$lio_pid"
lio_pid=

python3 - "$output_dir" "$lio_alive_before_stop" "$lio_alive_before_observer_summary" <<'PY'
import json
import sys
from pathlib import Path

output = Path(sys.argv[1])
alive = sys.argv[2] == "true"
alive_during_observation = sys.argv[3] == "true"
log = (output / "fastlio.log").read_text(errors="replace")
observer = json.loads((output / "odometry.json").read_text())
summary = {
    "fastlio_alive_before_observer_summary": alive_during_observation,
    "fastlio_alive_before_stop": alive,
    "mapping_initializations": log.count("Initialize the map kdtree"),
    "no_effective_points": log.count("No Effective Points!"),
    "no_match_skips": log.count("No effective correspondences; skipping this scan"),
    "publisher_side_odometry_count": log.count("[T13-DIAG] odometry_published="),
    "observer": observer,
}
(output / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
print(json.dumps(summary, sort_keys=True))
PY
