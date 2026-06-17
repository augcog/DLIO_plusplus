#!/usr/bin/env bash
# Orchestrated GICP-localization replay against a prebuilt map.
#
# Starts (1) the localization launch, (2) the UTM->map GT bridge,
# (3) a live GNSS/GICP error monitor, (4) a recorder for evaluation topics,
# waits for the map to load, then (5) replays the prepped bag with /clock.
# Tears everything down when the replay ends and prints the evaluation.
#
# Usage (local ROS 2 Jazzy shell; setup.bash files are sourced when found):
#   scripts/run_localization_replay.sh <prepped_bag_dir> <map.pcd> <T_world_utm.txt> <out_dir> [rviz=true|false]
#   scripts/run_localization_replay.sh --raw-live --utm-origin E,N <raw_bag_dir> <map.pcd> <T_world_utm.txt> <out_dir> [rviz=true|false]
set -u

RAW_LIVE="${DLIO_USE_ADAPTER:-false}"
ADAPTER_UTM_ORIGIN="${DLIO_ADAPTER_UTM_ORIGIN:-}"
ADAPTER_IMU_STAMP_MODE="${DLIO_ADAPTER_IMU_STAMP_MODE:-auto}"
ADAPTER_LOOKAHEAD="${DLIO_ADAPTER_IMU_LOOKAHEAD:-16}"
ADAPTER_POSE_RELIABILITY="${DLIO_ADAPTER_POSE_RELIABILITY:-reliable}"
ADAPTER_POSE_QOS_DEPTH="${DLIO_ADAPTER_POSE_QOS_DEPTH:-100}"
ADAPTER_IMU_RELIABILITY="${DLIO_ADAPTER_IMU_RELIABILITY:-best_effort}"
ADAPTER_IMU_QOS_DEPTH="${DLIO_ADAPTER_IMU_QOS_DEPTH:-100}"
ADAPTER_PLAY_RATE="${DLIO_ADAPTER_PLAY_RATE:-}"
ADAPTER_PLAY_DELAY="${DLIO_ADAPTER_PLAY_DELAY:-}"
ADAPTER_LIDAR_RELIABILITY="${DLIO_ADAPTER_LIDAR_RELIABILITY:-best_effort}"
ADAPTER_LIDAR_QOS_DEPTH="${DLIO_ADAPTER_LIDAR_QOS_DEPTH:-5}"
BAG_QOS_OVERRIDES="${DLIO_BAG_QOS_OVERRIDES:-${BAG_QOS_OVERRIDES:-}}"
while [ $# -gt 0 ]; do
  case "$1" in
    --raw-live|--live-adapter)
      RAW_LIVE="true"
      shift
      ;;
    --utm-origin)
      ADAPTER_UTM_ORIGIN="${2:-}"
      shift 2
      ;;
    --adapter-imu-stamp-mode)
      ADAPTER_IMU_STAMP_MODE="${2:-}"
      shift 2
      ;;
    --adapter-lookahead)
      ADAPTER_LOOKAHEAD="${2:-}"
      shift 2
      ;;
    --adapter-pose-reliability)
      ADAPTER_POSE_RELIABILITY="${2:-}"
      shift 2
      ;;
    --adapter-pose-qos-depth)
      ADAPTER_POSE_QOS_DEPTH="${2:-}"
      shift 2
      ;;
    --adapter-imu-reliability)
      ADAPTER_IMU_RELIABILITY="${2:-}"
      shift 2
      ;;
    --adapter-imu-qos-depth)
      ADAPTER_IMU_QOS_DEPTH="${2:-}"
      shift 2
      ;;
    --adapter-play-rate)
      ADAPTER_PLAY_RATE="${2:-}"
      shift 2
      ;;
    --adapter-play-delay)
      ADAPTER_PLAY_DELAY="${2:-}"
      shift 2
      ;;
    --adapter-lidar-reliability)
      ADAPTER_LIDAR_RELIABILITY="${2:-}"
      shift 2
      ;;
    --adapter-lidar-qos-depth)
      ADAPTER_LIDAR_QOS_DEPTH="${2:-}"
      shift 2
      ;;
    --bag-qos-overrides)
      BAG_QOS_OVERRIDES="${2:-}"
      shift 2
      ;;
    --)
      shift
      break
      ;;
    -*)
      echo "unknown option: $1" >&2
      exit 1
      ;;
    *)
      break
      ;;
  esac
done

if [ $# -lt 4 ]; then
  echo "usage: $0 [--raw-live --utm-origin E,N] <bag_dir> <map.pcd> <T_world_utm.txt> <out_dir> [rviz=true|false]" >&2
  exit 1
fi
BAG="$1"; MAP="$2"; UTM="$3"; OUT="$4"; RVIZ="${5:-false}"
DESKEW="${DESKEW:-false}"
CROP_SIZE="${CROP_SIZE:-80.0}"
SENSOR_TYPE="${SENSOR_TYPE:-luminar}"
LIDAR_CONCAT_ENABLED="${LIDAR_CONCAT_ENABLED:-true}"
GT_RECOVERY_ENABLED="${GT_RECOVERY_ENABLED:-true}"
GT_REJECTION_ENABLED="${GT_REJECTION_ENABLED:-true}"
GT_VETO_ENABLED="${GT_VETO_ENABLED:-true}"
VERBOSE="${VERBOSE:-false}"
VERBOSE_SCAN_LOG="${VERBOSE_SCAN_LOG:-false}"
BAG_PLAY_ARGS="${BAG_PLAY_ARGS:-}"

case "${RAW_LIVE,,}" in
  true|1|yes|on) RAW_LIVE="true" ;;
  *) RAW_LIVE="false" ;;
esac
if [ "$RAW_LIVE" = "true" ] && [ -z "$ADAPTER_UTM_ORIGIN" ]; then
  echo "--raw-live requires --utm-origin E,N or DLIO_ADAPTER_UTM_ORIGIN" >&2
  exit 1
fi

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
if [ -n "${DLIO_RACE_COMMON_SETUP:-}" ]; then
  if [ ! -f "$DLIO_RACE_COMMON_SETUP" ]; then
    echo "DLIO_RACE_COMMON_SETUP not found: $DLIO_RACE_COMMON_SETUP" >&2
    exit 1
  fi
  source_if_exists "$DLIO_RACE_COMMON_SETUP"
fi
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
command -v python3 >/dev/null 2>&1 || { echo "python3 not found" >&2; exit 1; }

kill_if_running() {
  local pid="${1:-}"
  if [ -n "$pid" ] && [ "$pid" -gt 0 ] 2>/dev/null; then
    kill "$pid" 2>/dev/null || true
  fi
}

for f in "$BAG" "$MAP" "$UTM"; do
  [ -e "$f" ] || missing_input "$f"
done
mkdir -p "$OUT"
[ -e "$OUT/loc_eval" ] && { echo "$OUT/loc_eval already exists — choose a fresh out_dir" >&2; exit 1; }

cleanup() {
  echo "[replay] stopping nodes..."
  kill_if_running "${PLAY_PID:-}"
  kill_if_running "${REC_PID:-}"
  kill_if_running "${ERR_PID:-}"
  kill_if_running "${BRIDGE_PID:-}"
  kill_if_running "${ADAPTER_PID:-}"
  kill_if_running "${LOC_PID:-}"
  sleep 3
  pkill -9 -f gicp_localization_node 2>/dev/null
  pkill -9 -f dlio_input_adapter_node 2>/dev/null
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
    utm_transform_path:="$UTM" \
    deskew:="$DESKEW" \
    crop_size:="$CROP_SIZE" \
    sensor_type:="$SENSOR_TYPE" \
    lidar_concat_enabled:="$LIDAR_CONCAT_ENABLED" \
    gt_recovery_enabled:="$GT_RECOVERY_ENABLED" \
    gt_rejection_enabled:="$GT_REJECTION_ENABLED" \
    gt_veto_enabled:="$GT_VETO_ENABLED" \
    verbose:="$VERBOSE" \
    verbose_scan_log:="$VERBOSE_SCAN_LOG" > "$OUT/localization.log" 2>&1 &
LOC_PID=$!

if [ "$RAW_LIVE" = "true" ]; then
  echo "[replay] starting DLIO input adapter (raw-live)"
  ADAPTER_BIN="$(ros2 pkg prefix dlio_input_adapter)/lib/dlio_input_adapter/dlio_input_adapter_node"
  "$ADAPTER_BIN" --ros-args \
      -p use_sim_time:=true \
      -p utm_origin:="$ADAPTER_UTM_ORIGIN" \
      -p T_world_utm_path:="$UTM" \
      -p imu_stamp_mode:="$ADAPTER_IMU_STAMP_MODE" \
      -p imu_arrival_retime_lookahead:="$ADAPTER_LOOKAHEAD" \
      -p pose_input_reliability:="$ADAPTER_POSE_RELIABILITY" \
      -p pose_input_qos_depth:="$ADAPTER_POSE_QOS_DEPTH" \
      -p imu_input_reliability:="$ADAPTER_IMU_RELIABILITY" \
      -p imu_input_qos_depth:="$ADAPTER_IMU_QOS_DEPTH" \
      -p lidar_input_reliability:="$ADAPTER_LIDAR_RELIABILITY" \
      -p lidar_input_qos_depth:="$ADAPTER_LIDAR_QOS_DEPTH" \
      > "$OUT/input_adapter.log" 2>&1 &
  ADAPTER_PID=$!
else
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
fi

echo "[replay] starting live GNSS/GICP error monitor"
python3 "$REPO/scripts/live_gnss_error_monitor.py" --ros-args \
    -p est_topic:=/gicp/localization/odom \
    -p gnss_topic:=/gps_p1/filtered_odom_map \
    -p csv_path:="$OUT/live_error.csv" \
    -p plot_path:="$OUT/live_error.png" \
    > "$OUT/error_monitor.log" 2>&1 &
ERR_PID=$!

# EXTRA_RECORD_TOPICS (env) appends more topics to the eval recording.
echo "[replay] recording eval topics to $OUT/loc_eval"
ros2 bag record -o "$OUT/loc_eval" -s mcap \
    /gicp/localization/odom /gicp/localization/pose /gps_p1/filtered_odom_map \
    /gicp/localization/debug/gicp_elapsed_ms /gicp/localization/debug/fitness \
    /gps_p1/filtered_odom_map/path /gicp/localization/odom_path_live \
    /gicp/localization/debug/gnss_error_m /gicp/localization/debug/gnss_error_markers \
    /gicp/localization/debug/gnss_error_rollercoaster \
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
# shellcheck disable=SC2206
EXTRA_BAG_PLAY_ARGS=($BAG_PLAY_ARGS)
PLAY_CMD=(ros2 bag play "$BAG" --clock 100)
if [ -n "$ADAPTER_PLAY_RATE" ]; then
  PLAY_CMD+=(-r "$ADAPTER_PLAY_RATE")
fi
if [ -n "$ADAPTER_PLAY_DELAY" ]; then
  PLAY_CMD+=(--delay "$ADAPTER_PLAY_DELAY")
fi
PLAY_CMD+=("${EXTRA_BAG_PLAY_ARGS[@]}")
if [ "$RAW_LIVE" = "true" ]; then
  PLAY_CMD+=(
    --topics
    /atlas/imu_calibrated
    /atlas/pose_filtered
    /luminar_front/points
    /luminar_left/points
    /luminar_right/points
    --remap
    /luminar_front/points:=/dlio_raw/luminar_front/points
    /luminar_left/points:=/dlio_raw/luminar_left/points
    /luminar_right/points:=/dlio_raw/luminar_right/points
  )
  if [ -n "$BAG_QOS_OVERRIDES" ]; then
    PLAY_CMD+=(--qos-profile-overrides-path "$BAG_QOS_OVERRIDES")
  fi
fi
"${PLAY_CMD[@]}" > "$OUT/play.log" 2>&1 &
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
