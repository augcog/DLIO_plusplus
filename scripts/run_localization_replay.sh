#!/usr/bin/env bash
# Orchestrated GICP-localization replay against a prebuilt map.
#
# Starts (1) the localization launch, (2) the UTM->map GT bridge,
# (3) a recorder for evaluation topics, waits for the map to load, then
# (4) replays the prepped bag with /clock. Tears everything down when the
# replay ends and prints the evaluation.
#
# Usage (inside the ros2 distrobox, repo root, both setup.bash sourced):
#   scripts/run_localization_replay.sh <prepped_bag_dir> <map.pcd> <T_world_utm.txt> <out_dir> [rviz]
set -u

if [ $# -lt 4 ]; then
  echo "usage: $0 <prepped_bag_dir> <map.pcd> <T_world_utm.txt> <out_dir> [rviz]" >&2
  exit 1
fi
BAG="$1"; MAP="$2"; UTM="$3"; OUT="$4"; RVIZ="${5:-false}"

REPO="$(cd "$(dirname "$0")/.." && pwd)"

source_if_exists() {
  local file="$1"
  if [ -f "$file" ]; then
    local had_nounset=0
    case $- in
      *u*) had_nounset=1 ;;
    esac
    set +u
    # shellcheck disable=SC1090
    source "$file"
    if [ "$had_nounset" -eq 1 ]; then
      set -u
    fi
  fi
}

source_if_exists "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source_if_exists "$REPO/install/setup.bash"

prepend_ld_library_path_if_dir() {
  local dir="$1"
  [ -d "$dir" ] || return 0
  case ":${LD_LIBRARY_PATH:-}:" in
    *":$dir:"*) ;;
    *) export LD_LIBRARY_PATH="$dir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;;
  esac
}

prepend_ld_library_path_if_dir "/usr/local/lib"
prepend_ld_library_path_if_dir "/usr/local/cuda/targets/x86_64-linux/lib"

command -v ros2 >/dev/null 2>&1 || { echo "ros2 not found; source your ROS environment first" >&2; exit 1; }

# Large-message transport fix (VERIFIED 2026-06-12). The Luminar clouds are
# ~3 MB/msg @ 20 Hz; ROS 2's default datareader socket buffer (208 KB) holds
# only ~3 of a scan's ~47 UDP fragments, so a microburst overflows it and the
# WHOLE scan is lost — an independent `ros2 topic hz` probe measured
# /luminar_front/points at 0.6-1.3 Hz of 20 Hz without this fix. This Fast DDS
# profile keeps UDPv4 (multicast discovery stays intact — unlike
# FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA, which switches to SHM+TCP and broke
# discovery on this box) and only enlarges the socket buffers. With it, the
# same probe delivers 19.99 Hz. Exported here so the player, localizer, bridge
# and recorder all inherit it. (Override by pre-setting FASTRTPS_DEFAULT_PROFILES_FILE.)
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$REPO/gicp_localization/config/fastdds_largedata.xml}"
echo "[replay] FastDDS large-data profile: $FASTRTPS_DEFAULT_PROFILES_FILE"
# The profile requests 64 MB buffers but the kernel clamps each socket to
# net.core.rmem_max (4 MB stock here — already enough to hold one full scan).
# For burst headroom under heavy load, raise it once on the HOST as root:
#   sudo sysctl -w net.core.rmem_max=134217728 net.core.wmem_max=134217728
RMEM_MAX="$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)"
if [ "$RMEM_MAX" -lt 8388608 ]; then
  echo "[replay] NOTE: net.core.rmem_max=$RMEM_MAX (<8MB). Delivery is fixed at this level;"
  echo "[replay]       'sudo sysctl -w net.core.rmem_max=134217728' on the host adds burst headroom."
fi

for f in "$BAG" "$MAP" "$UTM"; do
  [ -e "$f" ] || { echo "missing: $f" >&2; exit 1; }
done
mkdir -p "$OUT"
[ -e "$OUT/loc_eval" ] && { echo "$OUT/loc_eval already exists — choose a fresh out_dir" >&2; exit 1; }

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

# Optional synthetic RTK denial (env): DENY_WINDOWS="95:135,445:495" makes the
# bridge inflate GT covariance inside those windows (seconds from bag start) so
# the localizer's rtk_gate drops the samples. DENY_COV_XY/_Z override variances.
echo "[replay] starting UTM->map GT bridge${DENY_WINDOWS:+ (RTK denial: $DENY_WINDOWS)}"
python3 "$REPO/gicp_localization/scripts/utm_to_map_odom.py" --ros-args \
    -p utm_transform_path:="$UTM" \
    ${DENY_WINDOWS:+-p deny_windows:="$DENY_WINDOWS"} \
    ${DENY_COV_XY:+-p deny_cov_xy:="$DENY_COV_XY"} \
    ${DENY_COV_Z:+-p deny_cov_z:="$DENY_COV_Z"} > "$OUT/utm_bridge.log" 2>&1 &
BRIDGE_PID=$!

# EXTRA_RECORD_TOPICS (env) appends more topics to the eval recording.
echo "[replay] recording eval topics to $OUT/loc_eval"
ros2 bag record -o "$OUT/loc_eval" -s mcap \
    /gicp/localization/odom /gicp/localization/pose /gps_p1/filtered_odom_map \
    /gicp/localization/debug/gicp_elapsed_ms /gicp/localization/debug/fitness \
    ${EXTRA_RECORD_TOPICS:-} \
    > "$OUT/record.log" 2>&1 &
REC_PID=$!

# Loading + voxelizing + building GICP covariances for a ~25M-point map takes
# several minutes; allow up to 30. On this machine, ROS graph introspection can
# fail to discover the localization publishers under the custom FastDDS
# transport profile even after the node is alive, which makes `ros2 topic info`
# an unreliable readiness check. Instead, treat the node as "up" once its log
# shows either:
#   - "Map loaded successfully ..." (constructor finished map load), or
#   - the periodic "No IMU received ..." warning (the node has entered spin and
#     is waiting for bag playback).
ready() {
  grep -Eq \
    "Map loaded successfully|No IMU received on '/gps_p1/imu'|No IMU received on \"/gps_p1/imu\"" \
    "$OUT/localization.log" 2>/dev/null
}
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
