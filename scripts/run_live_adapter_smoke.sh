#!/usr/bin/env bash
# Live hardware smoke gate for the DLIO input adapter.
#
# Expected live graph:
#   raw driver topics:
#     /atlas/imu_calibrated
#     /atlas/pose_filtered
#     /dlio_raw/luminar_front|left|right/points
#   normalized adapter topics:
#     /gps_p1/imu
#     /gps_p1/filtered_odom
#     /gps_p1/filtered_odom_rtk_fixed
#     /luminar_front|left|right/points
#
# The script can start dlio_input_adapter, but it does not start race_common
# hardware drivers. Bring those up first with the FusionEngine driver configured
# for imu_output_stamp_source:=p1_time when validating the live P1 path.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

DURATION="${DLIO_LIVE_SMOKE_DURATION:-30}"
OUT="${DLIO_LIVE_SMOKE_OUT:-}"
START_ADAPTER="false"
EXPECT_RAW_IMU_P1="false"
PTP_LOCK_CONFIRMED="false"
REQUIRE_RTK_FIXED="false"
REQUIRE_MAP_ODOM="false"
SKIP_LUMINAR_VALIDATOR="false"
ADAPTER_ARGS=()

usage() {
  cat <<'EOF'
usage: scripts/run_live_adapter_smoke.sh [options] [-- adapter_ros_args...]

Options:
  --duration <sec>          Live collection window. Default: 30.
  --out <dir>               Output directory. Default: dlio_data/live_adapter_smoke_<timestamp>.
  --start-adapter           Start dlio_input_adapter in this script.
  --expect-raw-imu-p1       Require /atlas/imu_calibrated header.stamp to be P1-like.
  --ptp-lock-confirmed      Pass through to live Luminar validator.
  --require-rtk-fixed       Require /gps_p1/filtered_odom_rtk_fixed samples.
  --require-map-odom        Require /gps_p1/filtered_odom_map samples.
  --skip-luminar-validator  Skip scripts/validate_luminar_timestamps_live.py.
  -h, --help                Show this help.

Adapter pass-through example:
  scripts/run_live_adapter_smoke.sh --start-adapter --expect-raw-imu-p1 -- \
    -p imu_stamp_mode:=auto \
    -p T_world_utm_path:=/path/to/T_world_utm.txt \
    -p utm_origin:=520000.000,4380000.000
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --duration)
      DURATION="${2:-}"
      shift 2
      ;;
    --out)
      OUT="${2:-}"
      shift 2
      ;;
    --start-adapter)
      START_ADAPTER="true"
      shift
      ;;
    --expect-raw-imu-p1)
      EXPECT_RAW_IMU_P1="true"
      shift
      ;;
    --ptp-lock-confirmed)
      PTP_LOCK_CONFIRMED="true"
      shift
      ;;
    --require-rtk-fixed)
      REQUIRE_RTK_FIXED="true"
      shift
      ;;
    --require-map-odom)
      REQUIRE_MAP_ODOM="true"
      shift
      ;;
    --skip-luminar-validator)
      SKIP_LUMINAR_VALIDATOR="true"
      shift
      ;;
    --)
      shift
      ADAPTER_ARGS=("$@")
      break
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [ -z "$OUT" ]; then
  OUT="$REPO/dlio_data/live_adapter_smoke_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$OUT"

set +u
source /opt/ros/jazzy/setup.bash
if [ -n "${DLIO_RACE_COMMON_SETUP:-}" ]; then
  source "$DLIO_RACE_COMMON_SETUP"
elif [ -f /home/roar/Documents/race_common/install/setup.bash ]; then
  source /home/roar/Documents/race_common/install/setup.bash
fi
source "$REPO/install/setup.bash"
set -u

echo "[live-smoke] output: $OUT"
echo "[live-smoke] duration: $DURATION s"
echo "[live-smoke] start adapter: $START_ADAPTER"

ADAPTER_PID=""
cleanup() {
  if [ -n "$ADAPTER_PID" ] && kill -0 "$ADAPTER_PID" 2>/dev/null; then
    kill -INT "$ADAPTER_PID" 2>/dev/null || true
    sleep 2
    kill -TERM "$ADAPTER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [ "$START_ADAPTER" = "true" ]; then
  ADAPTER_BIN="$(ros2 pkg prefix dlio_input_adapter)/lib/dlio_input_adapter/dlio_input_adapter_node"
  echo "[live-smoke] starting adapter: $ADAPTER_BIN"
  "$ADAPTER_BIN" --ros-args \
    -p use_sim_time:=false \
    -p summary_output_path:="$OUT/adapter_summary.txt" \
    "${ADAPTER_ARGS[@]}" \
    > "$OUT/input_adapter.log" 2>&1 &
  ADAPTER_PID=$!
  sleep 5
fi

CHECK_ARGS=(
  --duration "$DURATION"
  --json-out "$OUT/live_adapter_smoke.json"
)
if [ "$EXPECT_RAW_IMU_P1" = "true" ]; then
  CHECK_ARGS+=(--expect-raw-imu-p1)
fi
if [ "$REQUIRE_RTK_FIXED" = "true" ]; then
  CHECK_ARGS+=(--require-rtk-fixed)
fi
if [ "$REQUIRE_MAP_ODOM" = "true" ]; then
  CHECK_ARGS+=(--require-map-odom)
fi

echo "[live-smoke] checking raw and normalized topics"
CHECK_STATUS=0
python3 scripts/live_adapter_smoke_check.py "${CHECK_ARGS[@]}" \
  > "$OUT/live_adapter_smoke_check.log" 2>&1 || CHECK_STATUS=$?
cat "$OUT/live_adapter_smoke_check.log"
if [ "$CHECK_STATUS" -ne 0 ]; then
  echo "[live-smoke] FAIL: raw/normalized topic check failed (status=$CHECK_STATUS)" >&2
  exit "$CHECK_STATUS"
fi

if [ "$SKIP_LUMINAR_VALIDATOR" != "true" ]; then
  LUM_ARGS=(
    --duration "$DURATION"
    --json-out "$OUT/live_luminar_timestamp_check.json"
  )
  if [ "$PTP_LOCK_CONFIRMED" = "true" ]; then
    LUM_ARGS+=(--ptp-lock-confirmed)
  fi
  echo "[live-smoke] validating live Luminar per-point timestamps"
  LUM_STATUS=0
  python3 scripts/validate_luminar_timestamps_live.py "${LUM_ARGS[@]}" \
    > "$OUT/live_luminar_timestamp_check.log" 2>&1 || LUM_STATUS=$?
  cat "$OUT/live_luminar_timestamp_check.log"
  if [ "$LUM_STATUS" -ne 0 ]; then
    echo "[live-smoke] FAIL: live Luminar timestamp check failed (status=$LUM_STATUS)" >&2
    exit "$LUM_STATUS"
  fi
fi

echo "[live-smoke] PASS"
echo "[live-smoke] artifacts: $OUT"
