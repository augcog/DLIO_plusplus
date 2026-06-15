# Error / sensor-fusion timeline analysis — how to build & read it

`scripts/analyze_fusion_timeline.py` joins, on ONE time axis: the localizer's
position error vs the RTK/INS reference + LiDAR matching quality + IMU
propagation burden + GNSS/RTK quality + the node's accept/reject/snap
decisions. It then auto-detects high-error segments, attributes each to a
cause, and grades the fusion strategy. Output: PNG figures + CSVs + a
`report_<name>.md`, one subfolder per analyzed run.

## How the analysis is built (what the script does)

Inputs (all produced by a localization replay — see commands below):
- `<loc-dir>/loc_eval/` — an mcap bag with `/gicp/localization/odom` (the
  estimate), `/gps_p1/filtered_odom_map` (the RTK/INS reference), per-scan
  `/gicp/localization/pose`, and the debug topics (fitness, gicp_elapsed_ms,
  hessian, correspondence_ratio, …).
- `<loc-dir>/localization.log` — the node's stderr, parsed for per-scan
  `SCAN DEBUG` lines (status, fitness, hessian, guess-to-solution, gt_err) and
  the snap / gate / defer events.
- `<prepped-bag>` — used for the RTK-FIXED gating stream and the raw IMU
  delivery cadence.

Pipeline inside the script:
1. Read est + reference odometry; optionally gate the reference to RTK-quality
   covariance (`--gt-var-gate`), except inside synthetic-denial windows.
2. Interpolate the reference to every estimate stamp → horizontal/Z error,
   plus an along-track / cross-track split (along-track ≈ latency; cross-track
   ≈ true lateral error).
3. Build a per-scan table (fitness, GICP ms, hessian, accept/reject + reason,
   snap, consecutive-failure streak) by merging the eval bag with the parsed
   log by millisecond-rounded stamp.
4. Tag every error sample with the decision state active at that instant
   (LiDAR-accepted / dead-reckon / GT-snap / no-scan).
5. Detect high-error segments (`--err-threshold`, `--seg-peak-factor`),
   classify each (degeneracy veto, scan starvation, pure-IMU drift, latency
   floor, …), and measure recovery.
6. Compute stats + the 5 strategy verdicts; render figures; write CSVs + report.

## How to read the figures

### `timeline_<name>.png` — the main figure (6 stacked panels, shared x = seconds since bag start)
Background everywhere: light-red = a detected high-error segment (S1, S2 … in
the report); purple hatch = a synthetic RTK-denial window.

1. **position error [m], log** — red line = horizontal error (0.2 s median),
   pink band = its max envelope (peaks), brown = |Z| error, dashed = segment
   threshold, blue ticks at bottom = snap-to-GT events. Where red rises above
   the dashed line → find the matching `Sx` in the report.
2. **GICP fitness, log** — one dot per processed scan: green = accepted, red =
   rejected; purple (right axis) = hessian condition vs its gate. Red dots at
   *low* fitness = a gate (degeneracy/veto) rejected an otherwise-good match;
   red at *high* fitness = the scene genuinely didn't match the map.
3. **latency [ms], log** — cyan = GICP solve time per scan vs the 50 ms scan
   budget (dashed); grey = how old each scan's result was when its pose
   published. Cyan above the line → node can't keep up; grey 100–500 ms streaks
   → corrections arrive stale (this is the along-track error floor).
4. **IMU burden** — orange = seconds since the last accepted-LiDAR/GNSS
   correction (how long the pose rode IMU alone); red steps = consecutive
   non-accepted scans; grey = IMU stream gaps (should stay ~10 ms). Each orange
   ramp is a dead-reckoning stretch; error grows ≈ ramp-height × speed.
5. **GNSS sigma [m], log** — green = reference reported σ_xy vs the rtk_gate
   (dashed, 0.5 m); red ▼ = gate drops, blue ticks = snaps, purple ✕ = snap
   deferred (GT stale); green/red bar = the bag's RTK-FIXED state. σ above the
   gate (or purple ✕) = GNSS effectively unavailable to the system.
6. **speed + decision strip** — grey line = speed; the colored strip on top is
   *who was steering the estimate each moment*: green = LiDAR accepted,
   orange = rejected→IMU dead-reckon, blue = rejected→GT snap, light-grey = no
   scan processed. For "LiDAR+IMU primary, GPS assist", this strip should be
   mostly GREEN with rare blue.

**Causal chain to read:** scan drop/reject (panel 2/3 + grey/blue in 6) →
correction-age ramp (4) → error ramp (1) → snap tick (1/5) → error drops.

### `track_map_<name>.png`
The lap in X/Y colored by error (log). Yellow labels mark where each segment
peaks; small grey numbers are time marks. Shows *where on track* (which
corners/straights) errors concentrate.

### `distributions_<name>.png`
Four panels: error CDF conditioned on decision state; fitness histogram
accepted-vs-rejected; fitness-vs-error scatter (does fitness predict error?);
GICP cost histogram vs the 50 ms budget.

### `segments_<name>/segNN.png`
Per high-error segment zoom: error + along/cross split (top), per-scan fitness
with snap lines (middle), correction-age vs speed (bottom).

### `report_<name>.md`
Key metrics table, error conditioned on decision state, a row per high-error
segment with its attributed cause + recovery, and the 5 strategy verdicts.

## Caveat
The error reference is the same Atlas RTK/INS stream that the GT snaps copy, so
"error during snap states" is low partly by construction. Only synthetic-denial
windows (positions stay RTK-true, covariance inflated) give an error measure
fully independent of the estimator's GNSS input.

## Commands

Run inside the ros2-jazzy distrobox (needs rosbag2_py + matplotlib):

```bash
distrobox enter ros2-jazzy
cd /run/host/home/dongc1/workspace/DLIO_plusplus
source /opt/ros/jazzy/setup.bash && source install/setup.bash
DATA=/home/dongc1/dlio_data

# A) Analyze an EXISTING replay (fast, no replay needed):
python3 scripts/analyze_fusion_timeline.py \
    --loc-dir     $DATA/run_5_loc \
    --prepped-bag $DATA/run_5_prepped \
    --out  $DATA/fusion_analysis/run_5  --name run_5

# B) From scratch: replay first (produces loc_eval + localization.log), then analyze:
scripts/run_localization_replay.sh \
    $DATA/run_5_prepped $DATA/run_5_map.pcd \
    $DATA/run_5_dump/T_world_utm.txt  $DATA/run_5_loc_new
python3 scripts/analyze_fusion_timeline.py \
    --loc-dir $DATA/run_5_loc_new --prepped-bag $DATA/run_5_prepped \
    --out $DATA/fusion_analysis/run_5_new --name run_5_new

# Useful flags:
#   --err-threshold 1.0     segment threshold (m)
#   --seg-peak-factor 3.0    only report segments peaking above thr*factor
#   --denial-windows "95:135,445:495"   annotate + exempt synthetic RTK-denial
#   --t-max 1570             analyze only the first N seconds (clip a bad tail)
#   --max-zooms 8            number of per-segment zoom plots

# View the PNGs (they are headless-rendered):
xdg-open $DATA/fusion_analysis/run_5/timeline_run_5.png
# or browse the whole folder:
python3 -m http.server 8800 -d $DATA/fusion_analysis    # http://localhost:8800
```
