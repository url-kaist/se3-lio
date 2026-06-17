#!/usr/bin/env bash
# End-to-end verification: compare the live ROS2 node trajectory against the
# Python binding running the same rosbag through the same data path.
#
# Run inside the se3_lio:ros2 container.
set -eo pipefail

BAG="${1:?usage: run_verification.sh <bag_dir> [max_frames] [play_duration_s]}"
MAX_FRAMES="${2:-300}"
PLAY_DURATION="${3:-40}"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARAMS="${HERE}/../../pipelines/ros2/config/params.yaml"
OUT="${OUT:-/verify}"
mkdir -p "$OUT"

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

# Clean up any stale processes from previous (interrupted) runs
pkill -f "ros2 bag play" 2>/dev/null || true
pkill -f "lio_node" 2>/dev/null || true
pkill -f "record_node_odom" 2>/dev/null || true
sleep 1

echo "=== [1/3] reference: live ROS2 node on bag ==="
ros2 launch se3_lio run.launch.py >"$OUT/node.log" 2>&1 &
NODE_PID=$!
sleep 4

python3 "$HERE/record_node_odom.py" --topic /local/odometry --out "$OUT/node.tum" \
    --idle-timeout 6 >"$OUT/recorder.log" 2>&1 &
REC_PID=$!
sleep 1

# `timeout` bounds playback (version-agnostic; --playback-duration is not in all distros)
timeout "$PLAY_DURATION" ros2 bag play "$BAG" >"$OUT/play.log" 2>&1 || true

# let the node finish processing its queues, then let the recorder time out
sleep 8
wait "$REC_PID" 2>/dev/null || true
kill "$NODE_PID" 2>/dev/null || true
wait "$NODE_PID" 2>/dev/null || true

echo "=== [2/3] estimate: Python binding offline ==="
python3 "$HERE/run_offline.py" --bag "$BAG" --params "$PARAMS" \
    --max-frames "$MAX_FRAMES" --out "$OUT/python.tum"

echo "=== [3/3] compare ==="
python3 "$HERE/ate.py" "$OUT/node.tum" "$OUT/python.tum"
