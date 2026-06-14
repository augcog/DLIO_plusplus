#!/usr/bin/env bash
# Show cached GICP-vs-GNSS replay results in RViz without rerunning GICP.
#
# Usage:
#   scripts/show_cached_error_viz.sh <result_dir> [map.pcd] [duration_seconds]
#
# Example:
#   scripts/show_cached_error_viz.sh \
#     dlio_data/run_3_loc_gnss_live \
#     dlio_data/run_5_map.pcd \
#     90
set -u

if [ $# -lt 1 ]; then
  echo "usage: $0 <result_dir> [map.pcd] [duration_seconds]" >&2
  exit 1
fi

RESULT_DIR="$1"
MAP="${2:-}"
DURATION="${3:-90}"
CSV="$RESULT_DIR/live_error.csv"

[ -f "$CSV" ] || { echo "missing cached live_error.csv: $CSV" >&2; exit 1; }
if [ -n "$MAP" ] && [ ! -f "$MAP" ]; then
  echo "missing map: $MAP" >&2
  exit 1
fi

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

command -v ros2 >/dev/null 2>&1 || { echo "ros2 not found; source ROS first" >&2; exit 1; }
command -v rviz2 >/dev/null 2>&1 || { echo "rviz2 not found" >&2; exit 1; }

RVIZ_CFG="$(mktemp --suffix=.rviz)"
sed \
  -e 's/__MAP_FRAME__/map/g' \
  -e 's/__BASE_FRAME__/gps_antenna_top/g' \
  -e 's/Distance: 30$/Distance: 720/' \
  -e 's/Hide Left Dock: false/Hide Left Dock: true/' \
  -e 's/Hide Right Dock: false/Hide Right Dock: true/' \
  "$REPO/gicp_localization/launch/localization.rviz" > "$RVIZ_CFG"

cleanup() {
  kill "${VIZ_PID:-}" 2>/dev/null || true
  rm -f "$RVIZ_CFG"
}
trap cleanup EXIT INT TERM

echo "[cached-viz] result: $RESULT_DIR"
echo "[cached-viz] csv:    $CSV"
if [ -n "$MAP" ]; then
  echo "[cached-viz] map:    $MAP"
fi
echo "[cached-viz] no GICP, no rosbag playback; publishing cached RViz topics"

ARGS=(--csv "$CSV" --duration "$DURATION")
if [ -n "$MAP" ]; then
  ARGS+=(--map "$MAP")
fi

python3 "$REPO/scripts/replay_cached_error_viz.py" "${ARGS[@]}" \
  > "$RESULT_DIR/cached_error_viz.log" 2>&1 &
VIZ_PID=$!

rviz2 -d "$RVIZ_CFG"
