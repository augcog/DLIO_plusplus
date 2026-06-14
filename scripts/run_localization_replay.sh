#!/usr/bin/env bash
# Orchestrated GICP-localization replay against a prebuilt map.
#
# Starts (1) the localization launch, (2) the UTM->map GT bridge,
# (3) a live GNSS/GICP error monitor, (4) a recorder for evaluation topics,
# waits for the map to load, then (5) replays the prepped bag with /clock.
# Tears everything down when the replay ends and prints the evaluation.
#
# Usage (local ROS 2 Jazzy shell; setup.bash files are sourced when found):
#   scripts/run_localization_replay.sh <prepped_bag_dir> <map.pcd> <T_world_utm.txt> <out_dir> [rviz]
set -u

if [ $# -lt 4 ]; then
  echo "usage: $0 <prepped_bag_dir> <map.pcd> <T_world_utm.txt> <out_dir> [rviz]" >&2
  exit 1
fi
BAG="$1"; MAP="$2"; UTM="$3"; OUT="$4"; RVIZ="${5:-false}"

missing_input() {
  local path="$1"
  echo "missing: $path" >&2
  echo "Hint: PIPELINE.md examples use DATA/RUN placeholders. Set them first, e.g.:" >&2
  echo '  DATA="${DATA:-./dlio_data}"' >&2
  echo '  RUN="${RUN:-run_5}"' >&2
  echo '  MAP_RUN="${MAP_RUN:-$RUN}"' >&2
  echo "or pass concrete paths to this script." >&2
  exit 1
}

for f in "$BAG" "$MAP" "$UTM"; do
  [ -e "$f" ] || missing_input "$f"
done
mkdir -p "$OUT"
[ -e "$OUT/loc_eval" ] && { echo "$OUT/loc_eval already exists — choose a fresh out_dir" >&2; exit 1; }

REPO="$(cd "$(dirname "$0")/.." && pwd)"

source_if_exists() {
  local file="$1"
  if [ -f "$file" ]; then
    local restore_nounset=false
    case "$-" in
      *u*) restore_nounset=true; set +u ;;
    esac
    # shellcheck disable=SC1090
    source "$file"
    if [ "$restore_nounset" = true ]; then
      set -u
    fi
  fi
}

source_if_exists "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source_if_exists "$REPO/install/setup.bash"

command -v ros2 >/dev/null 2>&1 || { echo "ros2 not found; source your ROS environment first" >&2; exit 1; }
command -v python3 >/dev/null 2>&1 || { echo "python3 not found" >&2; exit 1; }

kill_if_running() {
  local pid="${1:-}"
  if [ -n "$pid" ] && [ "$pid" -gt 0 ] 2>/dev/null; then
    kill "$pid" 2>/dev/null || true
  fi
}

cleanup() {
  echo "[replay] stopping nodes..."
  kill_if_running "${PLAY_PID:-}"
  kill_if_running "${REC_PID:-}"
  kill_if_running "${ERR_PID:-}"
  kill_if_running "${BRIDGE_PID:-}"
  kill_if_running "${LOC_PID:-}"
  sleep 3
  pkill -9 -f gicp_localization_node 2>/dev/null
  pkill -9 -f utm_to_map_odom 2>/dev/null
  pkill -9 -f live_gnss_error_monitor.py 2>/dev/null
  pkill -9 -f "ros2 bag record" 2>/dev/null
  pkill -9 -f robot_state_publisher 2>/dev/null
  pkill -9 -f rviz2 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "[replay] starting localization (log: $OUT/localization.log)"
# stdbuf: when stdout is a pipe/file, the C++ node block-buffers its INFO
# lines and the "Map loaded successfully" marker below would sit unflushed
# in the buffer for the entire run. libstdbuf is inherited by launch's
# children, forcing line-buffering.
PUBLISH_TF=false
case "${RVIZ,,}" in
  true|1|yes|on) PUBLISH_TF=true ;;
esac
stdbuf -oL -eL ros2 launch gicp_localization localization_with_tf.launch.py \
    rviz:="$RVIZ" \
    publish_tf:="$PUBLISH_TF" \
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

echo "[replay] starting live GNSS/GICP error monitor"
python3 "$REPO/scripts/live_gnss_error_monitor.py" --ros-args \
    -p est_topic:=/gicp/localization/odom \
    -p gnss_topic:=/gps_p1/filtered_odom_map \
    -p csv_path:="$OUT/live_error.csv" \
    -p plot_path:="$OUT/live_error.png" \
    > "$OUT/error_monitor.log" 2>&1 &
ERR_PID=$!

echo "[replay] recording eval topics to $OUT/loc_eval"
ros2 bag record -o "$OUT/loc_eval" -s mcap \
    /gicp/localization/odom /gicp/localization/pose /gps_p1/filtered_odom_map \
    /gicp/localization/debug/gicp_elapsed_ms /gicp/localization/debug/fitness \
    /gps_p1/filtered_odom_map/path /gicp/localization/odom_path_live \
    /gicp/localization/debug/gnss_error_m /gicp/localization/debug/gnss_error_markers \
    /gicp/localization/debug/gnss_error_rollercoaster \
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
echo "[replay] live error csv: $OUT/live_error.csv"
echo "[replay] live error plot: $OUT/live_error.png"
