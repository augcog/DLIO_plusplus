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
mkdir -p "$DATA_ROOT"

PREPPED_OUT="$DATA_ROOT/${RUN_NAME}_prepped"
PREPPED="$PREPPED_OUT"
if [ -n "$PREPPED_INPUT" ]; then
  PREPPED="$PREPPED_INPUT"
fi
DUMP="$DATA_ROOT/${RUN_NAME}_dump"
MAP="$DATA_ROOT/${RUN_NAME}_map.pcd"
LOC="$DATA_ROOT/${RUN_NAME}_loc"

REF_MAP="$DATA_ROOT/${MAP_RUN}_map.pcd"
REF_UTM="$DATA_ROOT/${MAP_RUN}_dump/T_world_utm.txt"

if [ -n "$ORIGIN_RUN" ]; then
  UTM_ORIGIN_FILE="$DATA_ROOT/${ORIGIN_RUN}_prepped/utm_origin.txt"
fi

UTM_ORIGIN_ARGS=()
if [ -n "$UTM_ORIGIN_FILE" ]; then
  [ -f "$UTM_ORIGIN_FILE" ] || { echo "UTM origin file not found: $UTM_ORIGIN_FILE" >&2; exit 1; }
  UTM_ORIGIN_VALUE="$(tail -n 1 "$UTM_ORIGIN_FILE")"
  [ -n "$UTM_ORIGIN_VALUE" ] || { echo "UTM origin file is empty: $UTM_ORIGIN_FILE" >&2; exit 1; }
  UTM_ORIGIN_ARGS=(--utm-origin "$UTM_ORIGIN_VALUE")
fi

if [ -z "$PREPPED_INPUT" ] && [ -e "$PREPPED_OUT" ]; then
  echo "refusing to overwrite existing prepped output: $PREPPED_OUT" >&2
  echo "Use --prepped \"$PREPPED_OUT\" to reuse it, or choose a fresh --run name." >&2
  exit 1
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
echo "[pipeline] dry run: $DRY_RUN"
if [ ${#UTM_ORIGIN_ARGS[@]} -gt 0 ]; then
  echo "[pipeline] reusing UTM origin from: $UTM_ORIGIN_FILE"
fi

if [ "$DRY_RUN" = "true" ]; then
  echo "[pipeline] dry run complete; exiting before prep/mapping/localization"
  exit 0
fi

if [ -n "$RAW" ]; then
  echo "[pipeline] step 1/4: prepping bag"
  python3 -u "$REPO/scripts/prep_bag.py" \
    --input "$RAW" \
    --output "$PREPPED" \
    "${UTM_ORIGIN_ARGS[@]}"
else
  echo "[pipeline] step 1/4: using existing prepped bag"
fi

METADATA_YAML="$PREPPED/metadata.yaml"
if [ ! -s "$METADATA_YAML" ]; then
  echo "[pipeline] ERROR: prep output metadata is missing or empty: $METADATA_YAML" >&2
  echo "[pipeline]        The prepared bag is incomplete. Common cause: output disk ran out of space." >&2
  exit 1
fi

if [ "$FULL_PIPELINE" = "true" ]; then
  echo "[pipeline] step 2/4: building map with GLIM"
  ros2 run glim_ros glim_rosbag "$PREPPED" \
    --ros-args -p dump_path:="$DUMP" -p auto_quit:=true

  echo "[pipeline] step 2b/4: evaluating map trajectory against GNSS"
  python3 "$REPO/scripts/eval_traj_vs_gnss.py" \
    --dump "$DUMP" \
    --bag "$PREPPED"

  echo "[pipeline] step 3/4: exporting PCD map"
  ros2 run glim_ros glim_dump_to_pcd "$DUMP" "$MAP"
else
  echo "[pipeline] skipping map build/export; using existing map from run '$MAP_RUN'"
fi

echo "[pipeline] step 4/4: localization replay"
"$REPO/scripts/run_localization_replay.sh" \
  "$PREPPED" \
  "$REF_MAP" \
  "$REF_UTM" \
  "$LOC" \
  "$RVIZ"

echo
echo "[pipeline] done"
echo "[pipeline] prepared bag: $PREPPED"
if [ "$FULL_PIPELINE" = "true" ]; then
  echo "[pipeline] dump dir:     $DUMP"
  echo "[pipeline] map pcd:      $MAP"
fi
echo "[pipeline] loc output:   $LOC"
