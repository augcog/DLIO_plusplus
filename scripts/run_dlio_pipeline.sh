#!/usr/bin/env bash
# One-command wrapper around the documented DLIO++ bag prep, map build,
# map export, and localization replay flow.
#
# Run this from a local ROS 2 Jazzy environment. The script will source
# /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash and install/setup.bash when
# available.
#
# Common cases:
#   0. Preferred public-clone setup: copy the root config template, edit local
#      paths, then run without flags.
#      cp dlio.env.example dlio.env
#      ${EDITOR:-nano} dlio.env
#      scripts/run_dlio_pipeline.sh
#
#   1. Full pipeline for a run (prep -> map -> export pcd -> localize replay)
#      scripts/run_dlio_pipeline.sh \
#        --raw "${DLIO_ROSBAG_ROOT:-../rosbags}/putnam/may_26/run_5/filtered/all" \
#        --data-root "./dlio_data" \
#        --run "run_5"
#
#   2. Prep + localize a different run against an existing map/origin
#      scripts/run_dlio_pipeline.sh \
#        --raw "${DLIO_ROSBAG_ROOT:-../rosbags}/putnam/may_26/run_3/filtered/all" \
#        --data-root "./dlio_data" \
#        --run "run_3" \
#        --origin-run "run_5" \
#        --map-run "run_5" \
#        --rviz true
#
#   3. Reuse an already-prepped bag when the raw bag is not mounted locally
#      scripts/run_dlio_pipeline.sh \
#        --prepped "./dlio_data/run_3_prepped" \
#        --data-root "./dlio_data" \
#        --run "run_3" \
#        --map-run "run_5" \
#        --rviz true
#
#   4. Validate local config and paths without starting ROS replay
#      scripts/run_dlio_pipeline.sh --dry-run
#
#   5. Live-adapter path: raw bag -> dlio_input_adapter -> GLIM/GICP
#      scripts/run_dlio_pipeline.sh --raw-live --raw "$RAW" --data-root "$DATA" --run run_5

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_FILE="$REPO/dlio.env"
LOAD_CONFIG="true"
CONFIG_LOADED="false"

args=("$@")
idx=0
while [ "$idx" -lt "${#args[@]}" ]; do
  case "${args[$idx]}" in
    --config)
      CONFIG_FILE="${args[$((idx + 1))]:-}"
      idx=$((idx + 2))
      ;;
    --no-config)
      LOAD_CONFIG="false"
      idx=$((idx + 1))
      ;;
    *)
      idx=$((idx + 1))
      ;;
  esac
done

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

if [ "$LOAD_CONFIG" = "true" ]; then
  if [ -f "$CONFIG_FILE" ]; then
    source_if_exists "$CONFIG_FILE"
    CONFIG_LOADED="true"
  elif [ "$CONFIG_FILE" != "$REPO/dlio.env" ]; then
    echo "config file not found: $CONFIG_FILE" >&2
    exit 1
  fi
fi

usage() {
  cat <<'EOF'
usage:
  scripts/run_dlio_pipeline.sh [--config dlio.env] [options]
  scripts/run_dlio_pipeline.sh (--raw <filtered/all> | --prepped <bag>) --run <name> [options]

required:
  --raw <path>            Raw input bag directory or .mcap file for prep_bag.py.
                          Can also be set as DLIO_RAW in dlio.env.
  --prepped <path>        Existing prepared bag directory or .mcap file; skips prep.
                          Can also be set as DLIO_PREPPED in dlio.env.
  --run <name>            Logical run name, e.g. run_5.
                          Can also be set as DLIO_RUN in dlio.env.
  --raw-live, --live-adapter
                          Do not run prep_bag.py. Replay --raw through
                          dlio_input_adapter and feed GLIM/GICP online.
                          Can also be set as DLIO_USE_ADAPTER=true.

optional:
  --config <file>         Read pipeline defaults from this env file.
                          Default: ./dlio.env when present.
  --no-config             Ignore ./dlio.env.
  --data-root <dir>       Output root directory for generated artifacts.
                          Default: ./dlio_data, or DLIO_DATA_ROOT.
  --origin-run <name>     Reuse UTM origin from <data-root>/<name>_prepped/utm_origin.txt
  --utm-origin-file <f>   Reuse UTM origin from an explicit utm_origin.txt file
  --map-run <name>        Localize against <data-root>/<name>_map.pcd and
                          <data-root>/<name>_dump/T_world_utm.txt.
                          Default: current --run. If different, map building is skipped.
  --rviz <true|false>     Pass-through to localization replay helper. Default: false
  --adapter-imu-stamp-mode <auto|p1|arrival_retime>
                          Passed to dlio_input_adapter. Default: auto.
  --adapter-lookahead <n> Bounded IMU burst-retime lookahead for legacy raw bags.
                          Default: 16 for wrapper runs.
  --adapter-pose-reliability <best_effort|reliable>
                          Adapter input QoS for raw-live pose replay.
  --adapter-pose-qos-depth <n>
                          Adapter input QoS depth for raw-live pose replay; 0 means keep_all.
  --adapter-imu-reliability <best_effort|reliable>
                          Adapter input QoS for raw-live IMU replay.
  --adapter-imu-qos-depth <n>
                          Adapter input QoS depth for raw-live IMU replay; 0 means keep_all.
  --adapter-imu-p1-pcap <f>
                          Decode Point One IMU_OUTPUT online from this PCAP in
                          raw-live mode instead of replaying /atlas/imu_calibrated
                          from the raw bag.
  --adapter-play-rate <r> Optional ros2 bag play rate for raw-live mode.
  --adapter-play-delay <s>
                          Optional ros2 bag play startup delay in seconds.
  --adapter-lidar-reliability <best_effort|reliable>
                          Adapter input QoS for raw-live Luminar replay.
  --adapter-lidar-qos-depth <n>
                          Adapter input QoS depth for raw-live Luminar replay; 0 means keep_all.
  --bag-qos-overrides <f> ros2 bag play QoS override YAML for raw-live mode.
  --adapter-utm-origin <E,N>
                          Explicit UTM origin for raw-live localization.
  --adapter-t-world-utm <f>
                          Override T_world_utm.txt for raw-live localization.
  --gt-recovery-min-consecutive-failures <n>
                          Override GICP GT recovery trigger count. Default: 4.
  --gt-veto-dist <m>      Override GICP GT veto distance in meters. Default: 3.0.
  --dry-run               Validate config, paths, and output choices, then exit
                          before prep/mapping/localization starts.
  -h, --help              Show this help

generated artifacts under <data-root>:
  <run>_prepped/          Prepared replay bag
  <run>_dump/             GLIM dump (full-pipeline mode only)
  <run>_map.pcd           Exported map PCD (full-pipeline mode only)
  <run>_loc/              Localization replay logs + evaluation

examples:
  cp dlio.env.example dlio.env
  ${EDITOR:-nano} dlio.env
  scripts/run_dlio_pipeline.sh

  scripts/run_dlio_pipeline.sh \
    --raw "${DLIO_ROSBAG_ROOT:-../rosbags}/putnam/may_26/run_5/filtered/all" \
    --data-root "./dlio_data" \
    --run "run_5"

  scripts/run_dlio_pipeline.sh \
    --raw "${DLIO_ROSBAG_ROOT:-../rosbags}/putnam/may_26/run_3/filtered/all" \
    --data-root "./dlio_data" \
    --run "run_3" \
    --origin-run "run_5" \
    --map-run "run_5" \
    --rviz true

  scripts/run_dlio_pipeline.sh \
    --prepped "./dlio_data/run_3_prepped" \
    --data-root "./dlio_data" \
    --run "run_3" \
    --map-run "run_5" \
    --rviz true

  scripts/run_dlio_pipeline.sh --dry-run

  scripts/run_dlio_pipeline.sh --raw-live \
    --raw "${DLIO_ROSBAG_ROOT:-../rosbags}/putnam/may_26/run_5/filtered/all" \
    --data-root "./dlio_data_adapter" \
    --run "run_5"
EOF
}

RAW="${DLIO_RAW:-${RAW:-}}"
PREPPED_INPUT="${DLIO_PREPPED:-${PREPPED_INPUT:-}}"
DATA_ROOT="${DLIO_DATA_ROOT:-${DATA_ROOT:-./dlio_data}}"
RUN_NAME="${DLIO_RUN:-${RUN_NAME:-}}"
ORIGIN_RUN="${DLIO_ORIGIN_RUN:-${ORIGIN_RUN:-}}"
UTM_ORIGIN_FILE="${DLIO_UTM_ORIGIN_FILE:-${UTM_ORIGIN_FILE:-}}"
MAP_RUN="${DLIO_MAP_RUN:-${MAP_RUN:-}}"
RVIZ="${DLIO_RVIZ:-${RVIZ:-false}}"
DRY_RUN="${DLIO_DRY_RUN:-${DRY_RUN:-false}}"
USE_ADAPTER="${DLIO_USE_ADAPTER:-${USE_ADAPTER:-false}}"
ADAPTER_IMU_STAMP_MODE="${DLIO_ADAPTER_IMU_STAMP_MODE:-${ADAPTER_IMU_STAMP_MODE:-auto}}"
ADAPTER_LOOKAHEAD="${DLIO_ADAPTER_IMU_LOOKAHEAD:-${ADAPTER_LOOKAHEAD:-16}}"
ADAPTER_POSE_RELIABILITY="${DLIO_ADAPTER_POSE_RELIABILITY:-${ADAPTER_POSE_RELIABILITY:-reliable}}"
ADAPTER_POSE_QOS_DEPTH="${DLIO_ADAPTER_POSE_QOS_DEPTH:-${ADAPTER_POSE_QOS_DEPTH:-100}}"
ADAPTER_IMU_RELIABILITY="${DLIO_ADAPTER_IMU_RELIABILITY:-${ADAPTER_IMU_RELIABILITY:-best_effort}}"
ADAPTER_IMU_QOS_DEPTH="${DLIO_ADAPTER_IMU_QOS_DEPTH:-${ADAPTER_IMU_QOS_DEPTH:-100}}"
ADAPTER_IMU_P1_PCAP="${DLIO_ADAPTER_IMU_P1_PCAP:-${ADAPTER_IMU_P1_PCAP:-}}"
ADAPTER_PLAY_RATE="${DLIO_ADAPTER_PLAY_RATE:-${ADAPTER_PLAY_RATE:-}}"
ADAPTER_PLAY_DELAY="${DLIO_ADAPTER_PLAY_DELAY:-${ADAPTER_PLAY_DELAY:-}}"
ADAPTER_LIDAR_RELIABILITY="${DLIO_ADAPTER_LIDAR_RELIABILITY:-${ADAPTER_LIDAR_RELIABILITY:-best_effort}}"
ADAPTER_LIDAR_QOS_DEPTH="${DLIO_ADAPTER_LIDAR_QOS_DEPTH:-${ADAPTER_LIDAR_QOS_DEPTH:-5}}"
BAG_QOS_OVERRIDES="${DLIO_BAG_QOS_OVERRIDES:-${BAG_QOS_OVERRIDES:-}}"
ADAPTER_UTM_ORIGIN="${DLIO_ADAPTER_UTM_ORIGIN:-${ADAPTER_UTM_ORIGIN:-}}"
ADAPTER_T_WORLD_UTM="${DLIO_ADAPTER_T_WORLD_UTM:-${ADAPTER_T_WORLD_UTM:-}}"
GT_RECOVERY_MIN_CONSECUTIVE_FAILURES="${DLIO_GT_RECOVERY_MIN_CONSECUTIVE_FAILURES:-${GT_RECOVERY_MIN_CONSECUTIVE_FAILURES:-4}}"
GT_VETO_DIST="${DLIO_GT_VETO_DIST:-${GT_VETO_DIST:-3.0}}"

while [ $# -gt 0 ]; do
  case "$1" in
    --raw)
      RAW="${2:-}"
      PREPPED_INPUT=""
      shift 2
      ;;
    --prepped)
      PREPPED_INPUT="${2:-}"
      RAW=""
      shift 2
      ;;
    --data-root)
      DATA_ROOT="${2:-}"
      shift 2
      ;;
    --run)
      RUN_NAME="${2:-}"
      shift 2
      ;;
    --origin-run)
      ORIGIN_RUN="${2:-}"
      shift 2
      ;;
    --utm-origin-file)
      UTM_ORIGIN_FILE="${2:-}"
      shift 2
      ;;
    --map-run)
      MAP_RUN="${2:-}"
      shift 2
      ;;
    --rviz)
      RVIZ="${2:-}"
      shift 2
      ;;
    --raw-live|--live-adapter)
      USE_ADAPTER="true"
      shift
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
    --adapter-imu-p1-pcap)
      ADAPTER_IMU_P1_PCAP="${2:-}"
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
    --adapter-utm-origin)
      ADAPTER_UTM_ORIGIN="${2:-}"
      shift 2
      ;;
    --adapter-t-world-utm)
      ADAPTER_T_WORLD_UTM="${2:-}"
      shift 2
      ;;
    --gt-recovery-min-consecutive-failures)
      GT_RECOVERY_MIN_CONSECUTIVE_FAILURES="${2:-}"
      shift 2
      ;;
    --gt-veto-dist)
      GT_VETO_DIST="${2:-}"
      shift 2
      ;;
    --dry-run)
      DRY_RUN="true"
      shift
      ;;
    --config)
      shift 2
      ;;
    --no-config)
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [ -z "$RAW" ] && [ -z "$PREPPED_INPUT" ] && [ -n "${DLIO_ROSBAG_ROOT:-}" ] && [ -n "$RUN_NAME" ]; then
  RAW="${DLIO_ROSBAG_ROOT%/}/putnam/may_26/${RUN_NAME}/filtered/all"
fi

if [ -z "$RAW" ] && [ -z "$PREPPED_INPUT" ]; then
  echo "one of --raw or --prepped is required" >&2
  usage >&2
  exit 1
fi
if [ -n "$RAW" ] && [ -n "$PREPPED_INPUT" ]; then
  echo "use either --raw or --prepped, not both" >&2
  usage >&2
  exit 1
fi
[ -n "$DATA_ROOT" ] || { echo "--data-root must not be empty" >&2; usage >&2; exit 1; }
[ -n "$RUN_NAME" ] || { echo "--run is required" >&2; usage >&2; exit 1; }

if [ -n "$ORIGIN_RUN" ] && [ -n "$UTM_ORIGIN_FILE" ]; then
  echo "use either --origin-run or --utm-origin-file, not both" >&2
  exit 1
fi
if [ -n "$PREPPED_INPUT" ] && { [ -n "$ORIGIN_RUN" ] || [ -n "$UTM_ORIGIN_FILE" ]; }; then
  echo "--origin-run/--utm-origin-file only apply when preparing from --raw" >&2
  exit 1
fi

if [ -z "$MAP_RUN" ]; then
  MAP_RUN="$RUN_NAME"
fi

case "${USE_ADAPTER,,}" in
  true|1|yes|on) USE_ADAPTER="true" ;;
  *) USE_ADAPTER="false" ;;
esac
if [ "$USE_ADAPTER" = "true" ] && [ -n "$PREPPED_INPUT" ]; then
  echo "--raw-live/--live-adapter requires --raw; --prepped is already normalized" >&2
  exit 1
fi
if [ -n "$ADAPTER_IMU_P1_PCAP" ] && [ "$ADAPTER_IMU_STAMP_MODE" = "arrival_retime" ]; then
  echo "--adapter-imu-p1-pcap requires --adapter-imu-stamp-mode auto or p1" >&2
  exit 1
fi

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

# Local gtsam_points/CUDA installs on this machine live under /usr/local.
# When the linker cache is stale, ros2-run'd GLIM binaries can fail to find
# libgtsam_points_cuda.so.1 unless these paths are visible explicitly.
prepend_ld_library_path_if_dir "/usr/local/lib"
prepend_ld_library_path_if_dir "/usr/local/cuda/targets/x86_64-linux/lib"

command -v ros2 >/dev/null 2>&1 || { echo "ros2 not found; source your ROS environment first" >&2; exit 1; }
command -v python3 >/dev/null 2>&1 || { echo "python3 not found" >&2; exit 1; }

if [ "$CONFIG_LOADED" = "true" ]; then
  echo "[pipeline] config: $CONFIG_FILE"
fi

if [ -n "$RAW" ] && [ ! -e "$RAW" ]; then
  case "$RAW" in
    /*) RAW_RESOLVED="$RAW" ;;
    *) RAW_RESOLVED="$(pwd)/$RAW" ;;
  esac
  echo "raw input does not exist: $RAW" >&2
  echo "resolved from current directory as: $RAW_RESOLVED" >&2
  echo "If the raw bags live on another disk, set DLIO_ROSBAG_ROOT or create a symlink, e.g.:" >&2
  echo "  export DLIO_ROSBAG_ROOT=/path/to/rosbags" >&2
  echo "  ln -s /path/to/rosbags \"$REPO/../rosbags\"" >&2
  echo "If this run is already prepared, use --prepped <prepared_bag> instead." >&2
  exit 1
fi
if [ -n "$PREPPED_INPUT" ] && [ ! -e "$PREPPED_INPUT" ]; then
  echo "prepped input does not exist: $PREPPED_INPUT" >&2
  exit 1
fi
if [ -n "$ADAPTER_IMU_P1_PCAP" ] && [ ! -e "$ADAPTER_IMU_P1_PCAP" ]; then
  echo "adapter IMU PCAP does not exist: $ADAPTER_IMU_P1_PCAP" >&2
  exit 1
fi
mkdir -p "$DATA_ROOT"

PREPPED_OUT="$DATA_ROOT/${RUN_NAME}_prepped"
PREPPED="$PREPPED_OUT"
if [ -n "$PREPPED_INPUT" ]; then
  PREPPED="$PREPPED_INPUT"
fi
ADAPTER_OUT="$DATA_ROOT/${RUN_NAME}_adapter"
ADAPTER_GNSS_BAG="$DATA_ROOT/${RUN_NAME}_adapter_gnss"
DUMP="$DATA_ROOT/${RUN_NAME}_dump"
MAP="$DATA_ROOT/${RUN_NAME}_map.pcd"
LOC="$DATA_ROOT/${RUN_NAME}_loc"

REF_MAP="$DATA_ROOT/${MAP_RUN}_map.pcd"
REF_UTM="$DATA_ROOT/${MAP_RUN}_dump/T_world_utm.txt"
REF_ADAPTER_ORIGIN="$DATA_ROOT/${MAP_RUN}_adapter/utm_origin.txt"
if [ -n "$ADAPTER_T_WORLD_UTM" ]; then
  REF_UTM="$ADAPTER_T_WORLD_UTM"
fi

if [ -n "$ORIGIN_RUN" ]; then
  UTM_ORIGIN_FILE="$DATA_ROOT/${ORIGIN_RUN}_prepped/utm_origin.txt"
fi

UTM_ORIGIN_ARGS=()
if [ -n "$UTM_ORIGIN_FILE" ]; then
  [ -f "$UTM_ORIGIN_FILE" ] || { echo "UTM origin file not found: $UTM_ORIGIN_FILE" >&2; exit 1; }
  UTM_ORIGIN_VALUE="$(tail -n 1 "$UTM_ORIGIN_FILE")"
  [ -n "$UTM_ORIGIN_VALUE" ] || { echo "UTM origin file is empty: $UTM_ORIGIN_FILE" >&2; exit 1; }
  UTM_ORIGIN_ARGS=(--utm-origin "$UTM_ORIGIN_VALUE")
  if [ -z "$ADAPTER_UTM_ORIGIN" ]; then
    ADAPTER_UTM_ORIGIN="$UTM_ORIGIN_VALUE"
  fi
fi

if [ "$USE_ADAPTER" = "false" ] && [ -z "$PREPPED_INPUT" ] && [ -e "$PREPPED_OUT" ]; then
  echo "refusing to overwrite existing prepped output: $PREPPED_OUT" >&2
  echo "Use --prepped \"$PREPPED_OUT\" to reuse it, or choose a fresh --run name." >&2
  exit 1
fi
if [ "$USE_ADAPTER" = "true" ]; then
  [ ! -e "$ADAPTER_OUT" ] || { echo "refusing to overwrite existing adapter output dir: $ADAPTER_OUT" >&2; exit 1; }
  [ ! -e "$ADAPTER_GNSS_BAG" ] || { echo "refusing to overwrite existing adapter GNSS bag: $ADAPTER_GNSS_BAG" >&2; exit 1; }
fi
if [ -e "$LOC" ]; then
  echo "refusing to overwrite existing localization output: $LOC" >&2
  exit 1
fi

FULL_PIPELINE="false"
if [ "$MAP_RUN" = "$RUN_NAME" ]; then
  FULL_PIPELINE="true"
  [ ! -e "$DUMP" ] || { echo "refusing to overwrite existing dump dir: $DUMP" >&2; exit 1; }
  [ ! -e "$MAP" ] || { echo "refusing to overwrite existing map file: $MAP" >&2; exit 1; }
else
  [ -e "$REF_MAP" ] || { echo "reference map not found: $REF_MAP" >&2; exit 1; }
  [ -e "$REF_UTM" ] || { echo "reference T_world_utm.txt not found: $REF_UTM" >&2; exit 1; }
  if [ "$USE_ADAPTER" = "true" ] && [ -z "$ADAPTER_UTM_ORIGIN" ]; then
    if [ -f "$REF_ADAPTER_ORIGIN" ]; then
      ADAPTER_UTM_ORIGIN="$(tail -n 1 "$REF_ADAPTER_ORIGIN")"
    elif [ -f "$DATA_ROOT/${MAP_RUN}_prepped/utm_origin.txt" ]; then
      ADAPTER_UTM_ORIGIN="$(tail -n 1 "$DATA_ROOT/${MAP_RUN}_prepped/utm_origin.txt")"
    fi
  fi
fi

echo "[pipeline] repo root: $REPO"
if [ -n "$RAW" ]; then
  echo "[pipeline] raw input: $RAW"
else
  echo "[pipeline] prepped input: $PREPPED"
fi
echo "[pipeline] data root: $DATA_ROOT"
echo "[pipeline] run name: $RUN_NAME"
echo "[pipeline] map run: $MAP_RUN"
echo "[pipeline] full pipeline: $FULL_PIPELINE"
echo "[pipeline] live adapter: $USE_ADAPTER"
echo "[pipeline] dry run: $DRY_RUN"
if [ ${#UTM_ORIGIN_ARGS[@]} -gt 0 ]; then
  echo "[pipeline] reusing UTM origin from: $UTM_ORIGIN_FILE"
fi
if [ "$USE_ADAPTER" = "true" ] && [ -n "$ADAPTER_UTM_ORIGIN" ]; then
  echo "[pipeline] adapter UTM origin: $ADAPTER_UTM_ORIGIN"
fi
if [ "$USE_ADAPTER" = "true" ] && [ -n "$ADAPTER_IMU_P1_PCAP" ]; then
  echo "[pipeline] adapter IMU PCAP: $ADAPTER_IMU_P1_PCAP"
fi
echo "[pipeline] GT recovery min consecutive failures: $GT_RECOVERY_MIN_CONSECUTIVE_FAILURES"
echo "[pipeline] GT veto distance: $GT_VETO_DIST"

if [ "$DRY_RUN" = "true" ]; then
  echo "[pipeline] dry run complete; exiting before prep/mapping/localization"
  exit 0
fi

stop_process() {
  local pid="${1:-}"
  local timeout="${2:-30}"
  [ -n "$pid" ] || return 0
  kill -0 "$pid" 2>/dev/null || return 0
  kill -INT "$pid" 2>/dev/null || true
  local waited=0
  while kill -0 "$pid" 2>/dev/null && [ "$waited" -lt "$timeout" ]; do
    sleep 1
    waited=$((waited + 1))
  done
  if kill -0 "$pid" 2>/dev/null; then
    kill -TERM "$pid" 2>/dev/null || true
    sleep 3
  fi
  if kill -0 "$pid" 2>/dev/null; then
    kill -KILL "$pid" 2>/dev/null || true
  fi
}

run_raw_live_mapping() {
  mkdir -p "$ADAPTER_OUT"
  echo "[pipeline] step 1/4: raw-live adapter mapping"
  local adapter_bin
  adapter_bin="$(ros2 pkg prefix dlio_input_adapter)/lib/dlio_input_adapter/dlio_input_adapter_node"
  local adapter_args=(
    "$adapter_bin" --ros-args
    -p use_sim_time:=true
    -p utm_origin_output_path:="$ADAPTER_OUT/utm_origin.txt"
    -p imu_stamp_mode:="$ADAPTER_IMU_STAMP_MODE"
    -p imu_arrival_retime_lookahead:="$ADAPTER_LOOKAHEAD"
    -p pose_input_reliability:="$ADAPTER_POSE_RELIABILITY"
    -p pose_input_qos_depth:="$ADAPTER_POSE_QOS_DEPTH"
    -p imu_input_reliability:="$ADAPTER_IMU_RELIABILITY"
    -p imu_input_qos_depth:="$ADAPTER_IMU_QOS_DEPTH"
    -p lidar_input_reliability:="$ADAPTER_LIDAR_RELIABILITY"
    -p lidar_input_qos_depth:="$ADAPTER_LIDAR_QOS_DEPTH"
  )
  if [ -n "$ADAPTER_UTM_ORIGIN" ]; then
    adapter_args+=(-p utm_origin:="$ADAPTER_UTM_ORIGIN")
  fi

  local play_cmd=(ros2 bag play "$RAW" --clock 100)
  if [ -n "$ADAPTER_PLAY_RATE" ]; then
    play_cmd+=(-r "$ADAPTER_PLAY_RATE")
  fi
  if [ -n "$ADAPTER_PLAY_DELAY" ]; then
    play_cmd+=(--delay "$ADAPTER_PLAY_DELAY")
  fi
  local raw_topics=(/atlas/pose_filtered /luminar_front/points /luminar_left/points /luminar_right/points)
  if [ -z "$ADAPTER_IMU_P1_PCAP" ]; then
    raw_topics=(/atlas/imu_calibrated "${raw_topics[@]}")
  fi
  play_cmd+=(
    --topics
    "${raw_topics[@]}"
    --remap
    /luminar_front/points:=/dlio_raw/luminar_front/points
    /luminar_left/points:=/dlio_raw/luminar_left/points
    /luminar_right/points:=/dlio_raw/luminar_right/points
  )
  if [ -n "$BAG_QOS_OVERRIDES" ]; then
    play_cmd+=(--qos-profile-overrides-path "$BAG_QOS_OVERRIDES")
  fi

  local ADAPTER_PID=""
  local PCAP_IMU_PID=""
  local GLIM_PID=""
  local REC_PID=""
  local PLAY_PID=""
  cleanup_raw_live_mapping() {
    stop_process "$PLAY_PID" 20
    stop_process "$REC_PID" 30
    stop_process "$GLIM_PID" 180
    stop_process "$PCAP_IMU_PID" 20
    stop_process "$ADAPTER_PID" 20
  }
  trap cleanup_raw_live_mapping EXIT INT TERM

  "${adapter_args[@]}" > "$ADAPTER_OUT/adapter.log" 2>&1 &
  ADAPTER_PID=$!
  if [ -n "$ADAPTER_IMU_P1_PCAP" ]; then
    local pcap_imu_bin
    pcap_imu_bin="$(ros2 pkg prefix dlio_input_adapter)/lib/dlio_input_adapter/p1_imu_pcap_replay_node.py"
    "$pcap_imu_bin" --ros-args \
      -p use_sim_time:=true \
      -p pcap_path:="$ADAPTER_IMU_P1_PCAP" \
      -p output_topic:=/atlas/imu_calibrated \
      -p pace_mode:=clock \
      > "$ADAPTER_OUT/p1_imu_pcap_replay.log" 2>&1 &
    PCAP_IMU_PID=$!
  fi
  sleep 3

  ros2 run glim_ros glim_rosnode --ros-args -p dump_path:="$DUMP" \
    > "$ADAPTER_OUT/glim_rosnode.log" 2>&1 &
  GLIM_PID=$!
  sleep 5

  ros2 bag record -o "$ADAPTER_GNSS_BAG" -s mcap \
    /gps_p1/filtered_odom /gps_p1/filtered_odom_rtk_fixed \
    > "$ADAPTER_OUT/record_gnss.log" 2>&1 &
  REC_PID=$!
  sleep 2

  "${play_cmd[@]}" > "$ADAPTER_OUT/play.log" 2>&1 &
  PLAY_PID=$!
  wait "$PLAY_PID"
  PLAY_PID=""
  echo "[pipeline] raw bag finished; letting GLIM drain"
  sleep 20

  cleanup_raw_live_mapping
  trap - EXIT INT TERM

  [ -d "$DUMP" ] || { echo "[pipeline] ERROR: GLIM dump missing: $DUMP" >&2; exit 1; }
  [ -s "$DUMP/T_world_utm.txt" ] || { echo "[pipeline] ERROR: T_world_utm.txt missing: $DUMP/T_world_utm.txt" >&2; exit 1; }
  [ -s "$ADAPTER_GNSS_BAG/metadata.yaml" ] || { echo "[pipeline] ERROR: adapter GNSS reference bag missing: $ADAPTER_GNSS_BAG" >&2; exit 1; }
}

if [ -n "$RAW" ]; then
  if [ "$USE_ADAPTER" = "true" ]; then
    echo "[pipeline] step 1/4: using raw-live adapter path"
  else
    echo "[pipeline] step 1/4: prepping bag"
    python3 -u "$REPO/scripts/prep_bag.py" \
      --input "$RAW" \
      --output "$PREPPED" \
      "${UTM_ORIGIN_ARGS[@]}"
  fi
else
  echo "[pipeline] step 1/4: using existing prepped bag"
fi

METADATA_YAML="$PREPPED/metadata.yaml"
if [ "$USE_ADAPTER" = "false" ] && [ ! -s "$METADATA_YAML" ]; then
  echo "[pipeline] ERROR: prep output metadata is missing or empty: $METADATA_YAML" >&2
  echo "[pipeline]        The prepared bag is incomplete. Common cause: output disk ran out of space." >&2
  exit 1
fi

if [ "$FULL_PIPELINE" = "true" ]; then
  if [ "$USE_ADAPTER" = "true" ]; then
    run_raw_live_mapping
  else
    echo "[pipeline] step 2/4: building map with GLIM"
    ros2 run glim_ros glim_rosbag "$PREPPED" \
      --ros-args -p dump_path:="$DUMP" -p auto_quit:=true
  fi

  echo "[pipeline] step 2b/4: evaluating map trajectory against GNSS"
  GNSS_EVAL_BAG="$PREPPED"
  if [ "$USE_ADAPTER" = "true" ]; then
    GNSS_EVAL_BAG="$ADAPTER_GNSS_BAG"
  fi
  python3 "$REPO/scripts/eval_traj_vs_gnss.py" \
    --dump "$DUMP" \
    --bag "$GNSS_EVAL_BAG"

  echo "[pipeline] step 3/4: exporting PCD map"
  ros2 run glim_ros glim_dump_to_pcd "$DUMP" "$MAP"
else
  echo "[pipeline] skipping map build/export; using existing map from run '$MAP_RUN'"
fi

echo "[pipeline] step 4/4: localization replay"
if [ "$USE_ADAPTER" = "true" ]; then
  if [ -z "$ADAPTER_UTM_ORIGIN" ]; then
    if [ -f "$ADAPTER_OUT/utm_origin.txt" ]; then
      ADAPTER_UTM_ORIGIN="$(tail -n 1 "$ADAPTER_OUT/utm_origin.txt")"
    elif [ -f "$REF_ADAPTER_ORIGIN" ]; then
      ADAPTER_UTM_ORIGIN="$(tail -n 1 "$REF_ADAPTER_ORIGIN")"
    fi
  fi
  [ -n "$ADAPTER_UTM_ORIGIN" ] || { echo "[pipeline] ERROR: raw-live localization needs adapter UTM origin" >&2; exit 1; }
  LOC_REPLAY_CMD=(
    "$REPO/scripts/run_localization_replay.sh"
    --raw-live
    --utm-origin "$ADAPTER_UTM_ORIGIN"
    --adapter-imu-stamp-mode "$ADAPTER_IMU_STAMP_MODE"
    --adapter-lookahead "$ADAPTER_LOOKAHEAD"
    --adapter-pose-reliability "$ADAPTER_POSE_RELIABILITY"
    --adapter-pose-qos-depth "$ADAPTER_POSE_QOS_DEPTH"
    --adapter-imu-reliability "$ADAPTER_IMU_RELIABILITY"
    --adapter-imu-qos-depth "$ADAPTER_IMU_QOS_DEPTH"
    --adapter-lidar-reliability "$ADAPTER_LIDAR_RELIABILITY"
    --adapter-lidar-qos-depth "$ADAPTER_LIDAR_QOS_DEPTH"
    --gt-recovery-min-consecutive-failures "$GT_RECOVERY_MIN_CONSECUTIVE_FAILURES"
    --gt-veto-dist "$GT_VETO_DIST"
  )
  if [ -n "$ADAPTER_PLAY_RATE" ]; then
    LOC_REPLAY_CMD+=(--adapter-play-rate "$ADAPTER_PLAY_RATE")
  fi
  if [ -n "$ADAPTER_PLAY_DELAY" ]; then
    LOC_REPLAY_CMD+=(--adapter-play-delay "$ADAPTER_PLAY_DELAY")
  fi
  if [ -n "$ADAPTER_IMU_P1_PCAP" ]; then
    LOC_REPLAY_CMD+=(--adapter-imu-p1-pcap "$ADAPTER_IMU_P1_PCAP")
  fi
  if [ -n "$BAG_QOS_OVERRIDES" ]; then
    LOC_REPLAY_CMD+=(--bag-qos-overrides "$BAG_QOS_OVERRIDES")
  fi
  LOC_REPLAY_CMD+=(
    "$RAW"
    "$REF_MAP"
    "$REF_UTM"
    "$LOC"
    "$RVIZ"
  )
  "${LOC_REPLAY_CMD[@]}"
else
  "$REPO/scripts/run_localization_replay.sh" \
    --gt-recovery-min-consecutive-failures "$GT_RECOVERY_MIN_CONSECUTIVE_FAILURES" \
    --gt-veto-dist "$GT_VETO_DIST" \
    "$PREPPED" \
    "$REF_MAP" \
    "$REF_UTM" \
    "$LOC" \
    "$RVIZ"
fi

echo
echo "[pipeline] done"
echo "[pipeline] prepared bag: $PREPPED"
if [ "$USE_ADAPTER" = "true" ]; then
  echo "[pipeline] adapter dir:  $ADAPTER_OUT"
  echo "[pipeline] adapter GNSS: $ADAPTER_GNSS_BAG"
fi
if [ "$FULL_PIPELINE" = "true" ]; then
  echo "[pipeline] dump dir:     $DUMP"
  echo "[pipeline] map pcd:      $MAP"
fi
echo "[pipeline] loc output:   $LOC"
