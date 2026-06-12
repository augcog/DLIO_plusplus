#!/usr/bin/env bash
# Orchestrated GICP-localization replay against a prebuilt map.
#
# Starts (1) the localization launch, (2) the UTM->map GT bridge,
# (3) a recorder for evaluation topics, waits for the map to load, then
# (4) replays the prepped bag with /clock. Tears everything down when the
# replay ends and prints the evaluation.
#
# Usage (local ROS 2 Jazzy shell; setup.bash files are sourced when found):
#   scripts/run_localization_replay.sh <prepped_bag_dir> <map.pcd> <T_world_utm.txt> <out_dir> [rviz]
set -u

if [ $# -lt 4 ]; then
  echo "usage: $0 <prepped_bag_dir> <map.pcd> <T_world_utm.txt> <out_dir> [rviz]" >&2
  exit 1
fi
BAG="$1"; MAP="$2"; UTM="$3"; OUT="$4"; RVIZ="${5:-false}"
for f in "$BAG" "$MAP" "$UTM"; do
  [ -e "$f" ] || { echo "missing: $f" >&2; exit 1; }
done
mkdir -p "$OUT"
[ -e "$OUT/loc_eval" ] && { echo "$OUT/loc_eval already exists — choose a fresh out_dir" >&2; exit 1; }

REPO="$(cd "$(dirname "$0")/.." && pwd)"

source_if_exists() {
  local file="$1"
  if [ -f "$file" ]; then
    # shellcheck disable=SC1090
    source "$file"
  fi
}

source_if_exists "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source_if_exists "$REPO/install/setup.bash"

command -v ros2 >/dev/null 2>&1 || { echo "ros2 not found; source your ROS environment first" >&2; exit 1; }
command -v python3 >/dev/null 2>&1 || { echo "python3 not found" >&2; exit 1; }

cleanup() {
  echo "[replay] stopping nodes..."
  kill "${PLAY_PID:-0}" "${REC_PID:-0}" "${BRIDGE_PID:-0}" "${LOC_PID:-0}" 2>/dev/null
  sleep 3
  pkill -9 -f gicp_localization_node 2>/dev/null
  pkill -9 -f utm_to_map_odom 2>/dev/null
  pkill -9 -f "ros2 bag record" 2>/dev/null
  pkill -9 -f robot_state_publisher 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "[replay] starting localization (log: $OUT/localization.log)"
# stdbuf: when stdout is a pipe/file, the C++ node block-buffers its INFO
# lines and the "Map loaded successfully" marker below would sit unflushed
# in the buffer for the entire run. libstdbuf is inherited by launch's
# children, forcing line-buffering.
stdbuf -oL -eL ros2 launch gicp_localization localization_with_tf.launch.py \
    rviz:="$RVIZ" \
    pointcloud_topic:=/luminar_front/points \
    imu_topic:=/gps_p1/imu \
    gt_odom_topic:=/gps_p1/filtered_odom_map \
    map_path:="$MAP" \
    utm_transform_path:="$UTM" > "$OUT/localization.log" 2>&1 &
LOC_PID=$!

echo "[replay] starting UTM->map GT bridge"
python3 "$REPO/gicp_localization/scripts/utm_to_map_odom.py" --ros-args \
    -p utm_transform_path:="$UTM" > "$OUT/utm_bridge.log" 2>&1 &
BRIDGE_PID=$!

echo "[replay] recording eval topics to $OUT/loc_eval"
ros2 bag record -o "$OUT/loc_eval" -s mcap \
    /gicp/localization/odom /gicp/localization/pose /gps_p1/filtered_odom_map \
    /gicp/localization/debug/gicp_elapsed_ms /gicp/localization/debug/fitness \
    > "$OUT/record.log" 2>&1 &
REC_PID=$!

# Loading + voxelizing + building GICP covariances for a ~25M-point map takes
# several minutes; allow up to 30. Readiness is detected via ROS graph
# introspection (the odom publisher only exists once the node's constructor —
# which does the whole map load — has finished); log-grepping is unreliable
# because the node's stdout INFO lines stay block-buffered when piped.
ready() { ros2 topic info /gicp/localization/odom 2>/dev/null | grep -q "Publisher count: [1-9]"; }
echo -n "[replay] waiting for map load (can take minutes on big maps)"
for i in $(seq 1 360); do
  if ready; then break; fi
  if ! kill -0 "$LOC_PID" 2>/dev/null; then
    echo; echo "[replay] localization process died — see $OUT/localization.log" >&2; exit 1
  fi
  echo -n "."; sleep 5
done
echo
ready || { echo "[replay] map never loaded" >&2; exit 1; }
echo "[replay] localization node is up"
sleep 3

echo "[replay] playing bag (realtime, with /clock)"
ros2 bag play "$BAG" --clock 100 > "$OUT/play.log" 2>&1 &
PLAY_PID=$!
wait "$PLAY_PID"
echo "[replay] bag finished; letting the pipeline drain"
sleep 5
cleanup
trap - EXIT

echo "[replay] evaluation:"
python3 "$REPO/scripts/eval_odom_vs_gt.py" --bag "$OUT/loc_eval" \
    --est-topic /gicp/localization/odom --gt-topic /gps_p1/filtered_odom_map \
    --csv "$OUT/loc_err.csv"
