#!/usr/bin/env bash
# Run a lossless, auditable offline GICP replay.
#
# The runner is dataset-independent. It derives DATASET_ROOT from --map-dir or
# --map when possible and writes to DATASET_ROOT/gicp_result unless --out-root
# is supplied. Unaudited runs default to DATASET_ROOT/gicp_result/intermediate.
# Multiple --bag arguments are passed to ros2 bag play as inputs.
set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  printf '%s\n' \
    'Usage: run_gicp_replay_audit.sh [options]' \
    '' \
    'Required:' \
    '  --map FILE | --map-dir DIR   ENU PCD or directory containing map.pcd' \
    '  --bag PATH                   rosbag2 input; repeat for multiple inputs' \
    '  --run-name NAME              new result directory name' \
    '  --overlay SETUP.BASH         built GICP++ overlay' \
    '  --duration SECONDS           playback duration' \
    '' \
    'Common options:' \
    '  --out-root DIR               defaults to DATASET_ROOT/gicp_result/intermediate' \
    '  --start-offset SECONDS       default 0' \
    '  --rate RATE                  default 1.0' \
    '  --domain-id ID               default 177' \
    '  --pointcloud-topic TOPIC     default /luminar_front/points' \
    '  --imu-topic TOPIC            default /gps_p1/imu' \
    '  --gt-topic TOPIC             default /gps_p1/filtered_odom' \
    '  --reference-topic TOPIC      defaults to --gt-topic' \
    '  --mode MODE                  evidence label: gnss_aided | independent' \
    '  --reference-is-gt-ack        acknowledge aided scoring is not independent truth' \
    '  --min-accept-rate FRACTION   scorecard gate; default 0 (disabled)' \
    '  --max-rejection-streak N     scorecard gate; default 0 (disabled)' \
    '  --min-debug-coverage FRACTION fail if debug frames/input scans is lower; default 0.80' \
    '  --require-zero-drops         fail on front drops or timestamp resets' \
    '  --primary-queue-size N       default 8' \
    '  --read-ahead-queue-size N    rosbag playback prefetch; default 50000' \
    '  --config-path YAML           run-local overrides loaded after package defaults' \
    '  --qos-overrides YAML         optional publisher QoS override' \
    '  --play-topic TOPIC           repeat to replace the default topic set' \
    '  --bridge-script FILE         optional preprocessing/offset ROS node' \
    '  --bridge-arg VALUE           repeat; passed literally to the bridge'
}

MAP=
MAP_DIR=
OUT_ROOT=
RUN_NAME=
OVERLAY=
START_OFFSET=0
DURATION=
RATE=1.0
DOMAIN_ID=177
STORAGE_ID=mcap
POINTCLOUD_TOPIC=/luminar_front/points
IMU_TOPIC=/gps_p1/imu
GT_TOPIC=/gps_p1/filtered_odom
REFERENCE_TOPIC=
MODE=
REFERENCE_IS_GT_ACK=false
MIN_ACCEPT_RATE=0
MAX_REJECTION_STREAK=0
MIN_DEBUG_COVERAGE=0.80
REQUIRE_ZERO_DROPS=false
PRIMARY_QUEUE_SIZE=8
READ_AHEAD_QUEUE_SIZE=50000
CONFIG_PATH=
FUTURE_AUX_WAIT_TIMEOUT_S=0.150
LIDAR_CONCAT_ENABLED=false
REQUIRE_ALL_AUX=false
LIDAR_RELIABLE_QOS=true
QOS_OVERRIDES=
BRIDGE_SCRIPT=
declare -a BAGS=()
declare -a BRIDGE_ARGS=()
declare -a PLAY_TOPICS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map) MAP="${2:?missing value for --map}"; shift 2 ;;
    --map-dir) MAP_DIR="${2:?missing value for --map-dir}"; shift 2 ;;
    --bag) BAGS+=("${2:?missing value for --bag}"); shift 2 ;;
    --out-root) OUT_ROOT="${2:?missing value for --out-root}"; shift 2 ;;
    --run-name) RUN_NAME="${2:?missing value for --run-name}"; shift 2 ;;
    --overlay) OVERLAY="${2:?missing value for --overlay}"; shift 2 ;;
    --start-offset) START_OFFSET="${2:?missing value for --start-offset}"; shift 2 ;;
    --duration) DURATION="${2:?missing value for --duration}"; shift 2 ;;
    --rate) RATE="${2:?missing value for --rate}"; shift 2 ;;
    --domain-id) DOMAIN_ID="${2:?missing value for --domain-id}"; shift 2 ;;
    --storage-id) STORAGE_ID="${2:?missing value for --storage-id}"; shift 2 ;;
    --pointcloud-topic) POINTCLOUD_TOPIC="${2:?missing value}"; shift 2 ;;
    --imu-topic) IMU_TOPIC="${2:?missing value}"; shift 2 ;;
    --gt-topic) GT_TOPIC="${2:?missing value}"; shift 2 ;;
    --reference-topic) REFERENCE_TOPIC="${2:?missing value}"; shift 2 ;;
    --mode) MODE="${2:?missing value}"; shift 2 ;;
    --reference-is-gt-ack) REFERENCE_IS_GT_ACK=true; shift ;;
    --min-accept-rate) MIN_ACCEPT_RATE="${2:?missing value}"; shift 2 ;;
    --max-rejection-streak) MAX_REJECTION_STREAK="${2:?missing value}"; shift 2 ;;
    --min-debug-coverage) MIN_DEBUG_COVERAGE="${2:?missing value}"; shift 2 ;;
    --require-zero-drops) REQUIRE_ZERO_DROPS=true; shift ;;
    --primary-queue-size) PRIMARY_QUEUE_SIZE="${2:?missing value}"; shift 2 ;;
    --read-ahead-queue-size) READ_AHEAD_QUEUE_SIZE="${2:?missing value}"; shift 2 ;;
    --config-path) CONFIG_PATH="${2:?missing value}"; shift 2 ;;
    --future-aux-wait-timeout) FUTURE_AUX_WAIT_TIMEOUT_S="${2:?missing value}"; shift 2 ;;
    --lidar-concat-enabled) LIDAR_CONCAT_ENABLED="${2:?missing value}"; shift 2 ;;
    --require-all-aux) REQUIRE_ALL_AUX="${2:?missing value}"; shift 2 ;;
    --lidar-reliable-qos) LIDAR_RELIABLE_QOS="${2:?missing value}"; shift 2 ;;
    --qos-overrides) QOS_OVERRIDES="${2:?missing value}"; shift 2 ;;
    --play-topic) PLAY_TOPICS+=("${2:?missing value}"); shift 2 ;;
    --bridge-script) BRIDGE_SCRIPT="${2:?missing value}"; shift 2 ;;
    --bridge-arg) BRIDGE_ARGS+=("${2:?missing value}"); shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -n "$MAP" && -n "$MAP_DIR" ]]; then
  printf 'Pass only one of --map or --map-dir\n' >&2
  exit 2
fi
if [[ -n "$MAP_DIR" ]]; then
  MAP="${MAP_DIR%/}/map.pcd"
fi
if [[ -z "$MAP" || -z "$RUN_NAME" || -z "$OVERLAY" || -z "$DURATION" ]]; then
  printf '%s\n' '--map/--map-dir, --run-name, --overlay and --duration are required' >&2
  usage >&2
  exit 2
fi
if [[ ${#BAGS[@]} -eq 0 ]]; then
  printf 'At least one --bag is required\n' >&2
  exit 2
fi
if [[ "$MODE" != "gnss_aided" && "$MODE" != "independent" ]]; then
  printf '%s\n' '--mode must be explicitly set to gnss_aided or independent' >&2
  exit 2
fi
if [[ ! "$PRIMARY_QUEUE_SIZE" =~ ^[1-9][0-9]*$ ||
      ! "$READ_AHEAD_QUEUE_SIZE" =~ ^[1-9][0-9]*$ ||
      ! "$MAX_REJECTION_STREAK" =~ ^[0-9]+$ ]]; then
  printf 'Queue sizes must be positive integers\n' >&2
  exit 2
fi
if ! awk -v a="$MIN_ACCEPT_RATE" -v c="$MIN_DEBUG_COVERAGE" \
    'BEGIN { exit !(a >= 0 && a <= 1 && c >= 0 && c <= 1) }'; then
  printf 'Acceptance and coverage thresholds must be fractions in [0,1]\n' >&2
  exit 2
fi

resolve_existing() {
  local label="$1"
  local path="$2"
  local resolved
  if ! resolved="$(realpath -e -- "$path" 2>/dev/null)"; then
    printf '%s does not exist: %s\n' "$label" "$path" >&2
    return 1
  fi
  printf '%s\n' "$resolved"
}

MAP="$(resolve_existing Map "$MAP")" || exit 3
OVERLAY="$(resolve_existing Overlay "$OVERLAY")" || exit 3
for index in "${!BAGS[@]}"; do
  BAGS[$index]="$(resolve_existing Bag "${BAGS[$index]}")" || exit 3
done
if [[ -n "$QOS_OVERRIDES" ]]; then
  QOS_OVERRIDES="$(resolve_existing 'QoS overrides' "$QOS_OVERRIDES")" || exit 3
fi
if [[ -n "$BRIDGE_SCRIPT" ]]; then
  BRIDGE_SCRIPT="$(resolve_existing 'Bridge script' "$BRIDGE_SCRIPT")" || exit 3
fi
if [[ -n "$CONFIG_PATH" ]]; then
  CONFIG_PATH="$(resolve_existing 'Config path' "$CONFIG_PATH")" || exit 3
fi

if [[ "$MAP" == */maps/* ]]; then
  DATASET_ROOT="${MAP%%/maps/*}"
elif [[ "$(basename "$(dirname "$MAP")")" == "maps" ]]; then
  DATASET_ROOT="$(dirname "$(dirname "$MAP")")"
else
  DATASET_ROOT=
fi
if [[ -z "$OUT_ROOT" ]]; then
  if [[ -z "$DATASET_ROOT" ]]; then
    printf 'Could not derive DATASET_ROOT from map path; pass --out-root explicitly\n' >&2
    exit 2
  fi
  OUT_ROOT="$DATASET_ROOT/gicp_result/intermediate"
fi
OUT_ROOT="$(realpath -m "$OUT_ROOT")"
RUN_DIR="$OUT_ROOT/$RUN_NAME"

if [[ -e "$RUN_DIR" ]]; then
  printf 'Refusing to overwrite run directory: %s\n' "$RUN_DIR" >&2
  exit 3
fi
if [[ ! -s "$MAP" ]]; then
  printf 'Map is missing or empty: %s\n' "$MAP" >&2
  exit 3
fi
if [[ "$LIDAR_RELIABLE_QOS" == "true" && -z "$QOS_OVERRIDES" ]]; then
  QOS_OVERRIDES="$SCRIPT_DIR/../GICP_plusplus/cfg/lidar_reliable_replay.yaml"
fi
if [[ "$LIDAR_RELIABLE_QOS" == "true" && ! -s "$QOS_OVERRIDES" ]]; then
  printf 'Reliable replay QoS file is missing or empty: %s\n' "$QOS_OVERRIDES" >&2
  exit 3
fi
if [[ -z "$REFERENCE_TOPIC" ]]; then
  REFERENCE_TOPIC="$GT_TOPIC"
fi
if [[ "$MODE" == "gnss_aided" && "$REFERENCE_TOPIC" == "$GT_TOPIC" &&
      "$REFERENCE_IS_GT_ACK" != "true" ]]; then
  printf '%s\n' \
    'The GNSS-aided run uses the same topic for seeding/gating and scoring.' \
    'Pass --reference-is-gt-ack to label and acknowledge this non-independent evidence,' \
    'or pass a genuinely independent --reference-topic.' >&2
  exit 2
fi
if [[ "$MODE" == "independent" && "$REFERENCE_TOPIC" == "$GT_TOPIC" ]]; then
  printf '%s\n' \
    'Independent evidence requires --reference-topic to differ from the runtime --gt-topic.' \
    'A parameter profile alone cannot turn the same aided stream into independent truth.' >&2
  exit 2
fi
if [[ ${#PLAY_TOPICS[@]} -eq 0 ]]; then
  PLAY_TOPICS=(
    "$POINTCLOUD_TOPIC"
    /luminar_left/points
    /luminar_right/points
    "$IMU_TOPIC"
    "$GT_TOPIC"
  )
  if [[ "$REFERENCE_TOPIC" != "$GT_TOPIC" ]]; then
    PLAY_TOPICS+=("$REFERENCE_TOPIC")
  fi
fi

source /opt/ros/jazzy/setup.bash
source "$OVERLAY"
set -u
export ROS_DOMAIN_ID="$DOMAIN_ID"
export ROS_LOG_DIR="$RUN_DIR/ros_logs"
mkdir -p "$RUN_DIR" "$ROS_LOG_DIR"

bridge_pid=
launch_pid=
record_pid=
reference_record_pid=
resource_pid=
playback_pid=

stop_pid() {
  local pid="${1:-}"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 "$pid" 2>/dev/null || return 0
      sleep 0.25
    done
    kill -TERM "$pid" 2>/dev/null || true
  fi
}

stop_launch() {
  if [[ -z "$launch_pid" ]] || ! kill -0 "$launch_pid" 2>/dev/null; then
    return 0
  fi
  local child
  while read -r child; do
    [[ -n "$child" ]] && kill -INT "$child" 2>/dev/null || true
  done < <(pgrep -P "$launch_pid" || true)
  for _ in {1..80}; do
    pgrep -P "$launch_pid" >/dev/null 2>&1 || break
    sleep 0.25
  done
  stop_pid "$launch_pid"
}

cleanup() {
  stop_pid "$playback_pid"
  stop_pid "$record_pid"
  stop_pid "$reference_record_pid"
  stop_launch
  stop_pid "$bridge_pid"
  stop_pid "$resource_pid"
}
trap cleanup EXIT INT TERM

if [[ -n "$BRIDGE_SCRIPT" ]]; then
  python3 "$BRIDGE_SCRIPT" "${BRIDGE_ARGS[@]}" >"$RUN_DIR/bridge.log" 2>&1 &
  bridge_pid=$!
fi

ros2 launch gicp_plusplus localization_with_tf.launch.py \
  rviz:=false \
  map_path:="$MAP" \
  pointcloud_topic:="$POINTCLOUD_TOPIC" \
  imu_topic:="$IMU_TOPIC" \
  gt_odom_topic:="$GT_TOPIC" \
  lidar_concat_enabled:="$LIDAR_CONCAT_ENABLED" \
  require_all_aux:="$REQUIRE_ALL_AUX" \
  lidar_reliable_qos:="$LIDAR_RELIABLE_QOS" \
  future_aux_wait_timeout_s:="$FUTURE_AUX_WAIT_TIMEOUT_S" \
  primary_queue_size:="$PRIMARY_QUEUE_SIZE" \
  config_path:="$CONFIG_PATH" \
  >"$RUN_DIR/localization.log" 2>&1 &
launch_pid=$!

initialized=0
for _ in {1..180}; do
  if grep -q "DLIO Localization Node Initialized" "$RUN_DIR/localization.log"; then
    initialized=1
    break
  fi
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    printf 'Localization launch exited before initialization\n' >&2
    exit 4
  fi
  sleep 1
done
if [[ "$initialized" -ne 1 ]]; then
  printf 'Timed out waiting for localization initialization\n' >&2
  exit 5
fi

(
  while kill -0 "$launch_pid" 2>/dev/null; do
    date --iso-8601=seconds
    ps -o pid,etime,%cpu,%mem,rss,stat,cmd -C gicp_plusplus_node || true
    sleep 5
  done
) >"$RUN_DIR/resource.log" 2>&1 &
resource_pid=$!

ros2 bag record --storage mcap --output "$RUN_DIR/debug_topics_bag" \
  --regex '(^/gicp/localization/debug(/.*)?$)' \
  >"$RUN_DIR/record.log" 2>&1 &
record_pid=$!

ros2 bag record --storage mcap --output "$RUN_DIR/reference_topics_bag" \
  "$REFERENCE_TOPIC" >"$RUN_DIR/reference_record.log" 2>&1 &
reference_record_pid=$!

wait_for_subscription() {
  local topic="$1"
  local label="$2"
  local count
  for _ in {1..60}; do
    count="$(ros2 topic info "$topic" 2>/dev/null |
      awk '/Subscription count:/ {print $3; exit}')"
    if [[ "$count" =~ ^[1-9][0-9]*$ ]]; then
      return 0
    fi
    if ! kill -0 "$record_pid" 2>/dev/null ||
        ! kill -0 "$reference_record_pid" 2>/dev/null; then
      printf 'Recorder exited while waiting for %s subscription\n' "$label" >&2
      return 1
    fi
    sleep 0.25
  done
  printf 'Timed out waiting for recorder subscription: %s (%s)\n' "$label" "$topic" >&2
  return 1
}

wait_for_subscription /gicp/localization/debug/fitness 'debug evidence' || exit 5
wait_for_subscription "$REFERENCE_TOPIC" 'reference evidence' || exit 5

declare -a play_args=()
for bag in "${BAGS[@]}"; do
  play_args+=(-i "$bag" "$STORAGE_ID")
done
play_args+=(
  --start-paused
  --read-ahead-queue-size "$READ_AHEAD_QUEUE_SIZE"
  --rate "$RATE"
  --start-offset "$START_OFFSET"
  --playback-duration "$DURATION"
  --clock-topics "$POINTCLOUD_TOPIC"
  --disable-keyboard-controls
  --topics
)
play_args+=("${PLAY_TOPICS[@]}")
if [[ -n "$QOS_OVERRIDES" ]]; then
  play_args+=(--qos-profile-overrides-path "$QOS_OVERRIDES")
fi

play_start_ns="$(date +%s%N)"
ros2 bag play "${play_args[@]}" >"$RUN_DIR/playback.log" 2>&1 &
playback_pid=$!
resume_ready=0
for _ in {1..120}; do
  if ros2 service list 2>/dev/null | grep -qx '/rosbag2_player/resume'; then
    resume_ready=1
    break
  fi
  if ! kill -0 "$playback_pid" 2>/dev/null; then
    break
  fi
  sleep 0.25
done
if [[ "$resume_ready" -ne 1 ]]; then
  printf 'rosbag player exited or never exposed the resume service\n' >&2
  playback_exit=7
  stop_pid "$playback_pid"
else
  if ! ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{}' \
      >"$RUN_DIR/resume.log" 2>&1; then
    printf 'Failed to resume paused rosbag playback\n' >&2
    playback_exit=8
    stop_pid "$playback_pid"
  else
    wait "$playback_pid"
    playback_exit=$?
  fi
fi
playback_pid=
play_end_ns="$(date +%s%N)"

sleep 3
launch_alive=0
if kill -0 "$launch_pid" 2>/dev/null; then
  launch_alive=1
fi

stop_launch
launch_pid=
sleep 2
stop_pid "$record_pid"
record_pid=
stop_pid "$reference_record_pid"
reference_record_pid=
stop_pid "$bridge_pid"
bridge_pid=
stop_pid "$resource_pid"
resource_pid=

play_wall_s="$(awk -v start="$play_start_ns" -v end="$play_end_ns" \
  'BEGIN { printf "%.6f", (end-start)/1000000000.0 }')"
{
  printf 'playback_exit=%s\n' "$playback_exit"
  printf 'localization_alive_after_playback=%s\n' "$launch_alive"
  printf 'completed_utc=%s\n' "$(date --utc --iso-8601=seconds)"
  printf 'dataset_root=%s\n' "$DATASET_ROOT"
  printf 'ros_domain_id=%s\n' "$ROS_DOMAIN_ID"
  printf 'map=%s\n' "$MAP"
  printf 'map_bytes=%s\n' "$(stat -c %s "$MAP")"
  printf 'map_sha256=%s\n' "$(sha256sum "$MAP" | awk '{print $1}')"
  printf 'bags=%s\n' "${BAGS[*]}"
  printf 'start_offset_s=%s\n' "$START_OFFSET"
  printf 'playback_duration_s=%s\n' "$DURATION"
  printf 'playback_rate=%s\n' "$RATE"
  printf 'playback_wall_s=%s\n' "$play_wall_s"
  printf 'lidar_concat_enabled=%s\n' "$LIDAR_CONCAT_ENABLED"
  printf 'require_all_aux=%s\n' "$REQUIRE_ALL_AUX"
  printf 'lidar_reliable_qos=%s\n' "$LIDAR_RELIABLE_QOS"
  printf 'future_aux_wait_timeout_s=%s\n' "$FUTURE_AUX_WAIT_TIMEOUT_S"
  printf 'primary_queue_size=%s\n' "$PRIMARY_QUEUE_SIZE"
  printf 'read_ahead_queue_size=%s\n' "$READ_AHEAD_QUEUE_SIZE"
  printf 'config_path=%s\n' "$CONFIG_PATH"
  if [[ -n "$CONFIG_PATH" ]]; then
    printf 'config_sha256=%s\n' "$(sha256sum "$CONFIG_PATH" | awk '{print $1}')"
  fi
  printf 'qos_overrides=%s\n' "$QOS_OVERRIDES"
  printf 'play_topics=%s\n' "${PLAY_TOPICS[*]}"
  printf 'bridge_script=%s\n' "$BRIDGE_SCRIPT"
  printf 'bridge_args=%s\n' "${BRIDGE_ARGS[*]}"
  printf 'pointcloud_topic=%s\n' "$POINTCLOUD_TOPIC"
  printf 'imu_topic=%s\n' "$IMU_TOPIC"
  printf 'gt_topic=%s\n' "$GT_TOPIC"
  printf 'reference_topic=%s\n' "$REFERENCE_TOPIC"
  printf 'mode=%s\n' "$MODE"
  printf 'reference_is_gt_ack=%s\n' "$REFERENCE_IS_GT_ACK"
  printf 'min_accept_rate=%s\n' "$MIN_ACCEPT_RATE"
  printf 'max_rejection_streak=%s\n' "$MAX_REJECTION_STREAK"
  printf 'min_debug_coverage=%s\n' "$MIN_DEBUG_COVERAGE"
  printf 'require_zero_drops=%s\n' "$REQUIRE_ZERO_DROPS"
} >"$RUN_DIR/run_status.env"

ros2 bag info "$RUN_DIR/debug_topics_bag" \
  >"$RUN_DIR/debug_topics_bag.info" 2>"$RUN_DIR/debug_topics_bag.info.err"
debug_info_exit=$?
ros2 bag info "$RUN_DIR/reference_topics_bag" \
  >"$RUN_DIR/reference_topics_bag.info" 2>"$RUN_DIR/reference_topics_bag.info.err"
reference_info_exit=$?
bag_message_count() {
  awk '/^Messages:/ {print $2; exit}' "$1"
}
debug_messages="$(bag_message_count "$RUN_DIR/debug_topics_bag.info")"
reference_messages="$(bag_message_count "$RUN_DIR/reference_topics_bag.info")"
debug_messages="${debug_messages:-0}"
reference_messages="${reference_messages:-0}"

declare -a analyzer_args=(
  "$RUN_DIR/localization.log"
  --json-out "$RUN_DIR/scan_debug_scorecard.json"
  --mode "$MODE"
  --min-accept-rate "$MIN_ACCEPT_RATE"
  --max-rejection-streak "$MAX_REJECTION_STREAK"
)
if [[ "$REQUIRE_ZERO_DROPS" == "true" ]]; then
  analyzer_args+=(--require-zero-drops)
fi
python3 "$SCRIPT_DIR/../GICP_plusplus/scripts/analyze_scan_debug_log.py" \
  "${analyzer_args[@]}" \
  >"$RUN_DIR/scan_debug_scorecard.md" \
  2>"$RUN_DIR/scan_debug_scorecard.err"
analyzer_exit=$?

debug_frames=0
if [[ -s "$RUN_DIR/scan_debug_scorecard.json" ]]; then
  debug_frames="$(python3 -c \
    'import json,sys; print(json.load(open(sys.argv[1]))["frames"])' \
    "$RUN_DIR/scan_debug_scorecard.json")"
fi
expected_input_frames=0
for index in "${!BAGS[@]}"; do
  bag="${BAGS[$index]}"
  bag_info="$RUN_DIR/input_$(printf '%02d' "$index").info"
  ros2 bag info "$bag" >"$bag_info" 2>"$bag_info.err" || continue
  bag_duration="$(sed -n 's/^Duration:[[:space:]]*\([0-9.]*\)s.*/\1/p' "$bag_info" | head -1)"
  topic_count="$(awk -v topic="$POINTCLOUD_TOPIC" \
    'index($0, "Topic: " topic " ") {
       if (match($0, /Count: [0-9]+/)) {
         value=substr($0, RSTART+7, RLENGTH-7); print value; exit
       }
     }' "$bag_info")"
  if [[ -n "$bag_duration" && -n "$topic_count" ]]; then
    estimate="$(awk -v count="$topic_count" -v total="$bag_duration" \
      -v start="$START_OFFSET" -v duration="$DURATION" \
      'BEGIN {
         available=total-start; if (available < 0) available=0;
         window=(duration < available ? duration : available);
         estimated=(total > 0 ? count*window/total : 0);
         printf "%d", estimated
       }')"
    expected_input_frames=$((expected_input_frames + estimate))
  fi
done
debug_coverage="$(awk -v actual="$debug_frames" -v expected="$expected_input_frames" \
  'BEGIN {
     coverage=(expected > 0 ? actual/expected : 0);
     printf "%.6f", coverage
   }')"
{
  printf 'debug_bag_info_exit=%s\n' "$debug_info_exit"
  printf 'reference_bag_info_exit=%s\n' "$reference_info_exit"
  printf 'debug_messages=%s\n' "$debug_messages"
  printf 'reference_messages=%s\n' "$reference_messages"
  printf 'analyzer_exit=%s\n' "$analyzer_exit"
  printf 'debug_frames=%s\n' "$debug_frames"
  printf 'expected_input_frames=%s\n' "$expected_input_frames"
  printf 'debug_coverage=%s\n' "$debug_coverage"
} >>"$RUN_DIR/run_status.env"

if [[ "$playback_exit" -ne 0 || "$launch_alive" -ne 1 ||
      "$debug_info_exit" -ne 0 || "$reference_info_exit" -ne 0 ||
      "$debug_messages" -eq 0 || "$reference_messages" -eq 0 ||
      "$analyzer_exit" -ne 0 || "$expected_input_frames" -eq 0 ]] ||
    ! awk -v actual="$debug_coverage" -v minimum="$MIN_DEBUG_COVERAGE" \
      'BEGIN { exit !(actual >= minimum) }'; then
  exit 6
fi
