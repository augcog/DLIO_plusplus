# Consolidated Action Plan — GICP/GLIM Turn Error (2026-07-04)

Merges the Claude investigation (`turn_error_investigation_20260704.md`) and the Codex 5.5
review, after cross-checking both against run 12/13 logs and the code on `art-jazzy`
(`9e7dbc0`). Cross-check verdicts first, then one prioritized plan.

## Cross-check verdicts

| Claim | Verdict | Evidence |
|---|---|---|
| Codex: wrong poses (35–120 m) accepted as `ok` during turns | **Confirmed, sharpened** | 359 accepted frames with gt_err>20 m. Only 1/359 within 5 frames of any rejection; 230 have none within 40 frames → independent wrong-basin lock-ins, not post-streak artifacts. Jump-vs-prior median only 1.87 m → the IMU prior drifts *with* the wrong solution, so a per-frame consistency gate catches entry, not the locked-in stretch. |
| Codex: no separate yaw gain; single `Kq` | **Confirmed** | `localization.cc:4837`, `Kq=4.0`. Turn yaw error is latency + gating, not a gain knob. |
| Codex: GT snap zeroes angular rate, hurting yaw continuity | **Partially confirmed** | All 1,860 snaps log `ω=[0,0,0]` — but via the GT odom itself lacking angular twist (adapter), not the zero-twist fallback (fired once). Severity lower than stated: `state.v.ang` is not consumed by prior integration (IMU gyro buffer is), and linear velocity IS preserved (e.g. `v=[-14.49,…]`). Real issue: adapter doesn't populate `twist.angular`. |
| Codex: multi-lidar merge not the sole cause | **Confirmed** | Accept rate turning 86.9% vs straight 86.4%; large outliers occur on full-size scans. Merge coverage is still poor (~35% 2/2, 24% 0/2 in throttled samples) and worth fixing as a conditioning improvement. |
| Claude: hessian gate collapses to hessian-alone on cross-run maps → 25 s rejection streaks | **Stands** | Fitness floor ~0.27 (p10 0.255) > `hessianFitnessWarnThreshold=0.15` on ~every frame; 16 streaks ≥10 frames = 1,648 of 1,860 rejections; longest 253 frames. |
| Claude: stale `prev_vel` on rejected scans (bug) | **Stands, with nuance** | `geo.prev_vel` only updates on accepted scans; BUT in these replays snap (min_failures=1) refreshed `prev_vel` every rejected frame, masking it. It bites whenever GT is absent/deferred — i.e. production, and any `max_dt` lookup miss. |
| Claude: observer corrects current state toward 0.1–0.3 s-stale measurement → yaw-rate-proportional error | **Stands** | Accepted-frame gt_pos median 1.01→2.06 m and gt_rot 1.07°→1.75° monotonically vs yaw-rate bucket (0–2 → 25–90 °/s). |
| New (this cross-check): per-map-normalized fitness separates bad accepts | **Measured** | fitness / rolling-median(201): good accepts median 1.00 (p90 1.17); bad accepts median 1.34. Threshold 1.3 catches 53% of bad accepts at 4.8% false-flag; 1.5 catches 41% at 2.4%. Useful only when paired with partial updates (P1) so added flags don't create new dead-reckon streaks. |

Unified diagnosis: the accept/reject system fails in **both directions** — it discards
~1,800 mostly-fine corner scans (streaks → dead reckoning) *and* passes 359 wrong-basin
matches (worst video excursions). Binary gating on absolute, same-run-calibrated
thresholds is the shared root cause; observer latency adds a yaw-rate-proportional bias on
top; GT snap currently papers over the streak half in replays.

## Action plan (priority order)

### P1 — Replace binary gating with confidence-weighted acceptance  [GICP, biggest lever]
**STATUS: IMPLEMENTED 2026-07-04** in `src/localization.cc` / `cfg/localization.yaml`
(params `gicp/fitnessBaseline/*`, `gicp/fitnessRatioRejectThreshold`,
`gicp/degeneracy/*`, `gicp/yawGate/*`; new statuses `ok_partial`,
`rejected_fitness_ratio`; new debug topics `fitness_ratio`, `degen_rot_axes`,
`degen_trans_axes`, `yaw_veto`). Offline replay of the new gates against the
run-12 log: acceptance 86.8% → ~98.2% (hessian streak rejects become partial
accepts), 87 of 359 wrong-basin accepts ratio-rejected outright, yaw veto
engages on 322 frames (covers the wrong-basin entries), only 0.78% of
previously-fine frames newly rejected (scattered — no streaks). Projection
quality itself requires a real replay to validate. NOTE: the hessian is
re-centered about the vehicle before eigen-analysis (nano_gicp parameterizes
rotation about the world origin — without re-centering, rotational degeneracy
is invisible at ~600 m from origin).

**Review follow-up 2026-07-04 (Codex P2/P3 comments):** the initial
implementation eigendecomposed the rot/trans 3x3 blocks independently, which
is blind to COUPLED rot/trans null directions. Now `gicp/degeneracy/full6d:
true` (default) performs the true 6D remapping: rotation coordinates are
scaled by `couplingLengthM` (20 m) to make rad/m commensurable, then one 6x6
eigendecomposition on the re-centered hessian. Synthetic coupled case
(side-facing patch 15 m ahead: yaw and t_y individually constrained, the
combination (2 deg, -0.52 m) null): blockwise flags nothing and passes the
whole slide; full6d suppresses it to 0.002 deg / 4 mm while a legitimate
constrained 0.4 m correction survives. Blockwise kept as `full6d: false` for
A/B. Also fixed: `yawGate` no longer silently depends on
`degeneracy/partialUpdate`, and the code default for
`gt_recovery/min_consecutive_failures` now matches the YAML (5, was 3).
Covers Codex finding 1 + Claude RC1 together; neither reviewer's half-fix suffices alone.
1. Normalize fitness signals per map: gates on `fitness / rolling_median(fitness)` instead
   of absolute values. Re-point `hessianFitnessWarnThreshold` (0.15 abs → ~1.2 ratio) and
   add a bad-accept ratio gate at ~1.3–1.5.
2. Eigendecompose the final 6×6 hessian; apply GICP corrections only along
   well-constrained directions (solution remapping), keep IMU prior along degenerate axes.
   This converts both "reject wholesale" and "accept wholesale" into partial updates.
3. Add the turn-aware consistency check (Codex): compare GICP Δyaw across the scan gap
   against IMU-integrated Δyaw; large disagreement at high yaw rate → distrust the yaw
   component (with #2, that means: don't apply it), not necessarily the whole scan.
Acceptance criteria: streak max < 20 frames; accepted-frame gt_err>20 m count → near 0;
turning-bucket p90 within 1.5× of straight bucket.

### P2 — Fix state continuity on the non-accepted path  [GICP, small diffs]
**STATUS: IMPLEMENTED 2026-07-04**
1. Stale `prev_vel`: rejection branch now seeds `prev_vel` from the
   IMU-propagated `state.v.lin.w` instead of `geo.prev_vel` (localization.cc).
2. Snap twist continuity: linear/angular sources resolved independently in
   `maybeSnapPoseToGT` — angular backfills from the live bias-corrected IMU
   gyro, linear from GT-pose finite difference (`getGtFiniteDiffVelWorld`),
   last resort keeps the current state velocity (never zeroes a moving
   vehicle). The adapter now also populates `/gps_p1/filtered_odom`
   `twist.angular` from the latest Atlas gyro (with rate covariance), fixing
   it at the source for future bags.
3. `gt_recovery/min_consecutive_failures` raised 1 → 5 (yaml + README);
   validation still needs one pass with `gt_recovery/enable=false`.
1. **Stale `prev_vel` bug** (Claude RC2): on rejected scans seed `prev_vel` from
   `state.v.lin.w` (IMU-propagated) instead of stale `geo.prev_vel`. Mandatory for
   production where snap doesn't exist.
2. **Snap angular-rate continuity** (Codex 2, corrected scope): populate `twist.angular`
   in the adapter's `/gps_p1/filtered_odom` (or derive yaw rate from neighboring GT poses
   in `maybeSnapPoseToGT`); keep IMU-measured ω through the snap rather than zero.
3. Raise `gt_recovery/min_consecutive_failures` (1 → ~5) once P1 lands, and run one
   validation pass with `gt_recovery/enable=false` — snap-every-frame currently hides
   streak damage and RC2 in all replay metrics.

### P3 — Remove observer measurement-latency bias  [GICP]
**STATUS: IMPLEMENTED 2026-07-04**
1. Delta-form correction (`odom/geo/delta_correction: true`): updateState now
   targets `T_corr (x) current_state` with `T_corr = T_meas * inv(T_prior)`
   (both at median scan time, paired via new `observer_prior_pose_`), instead
   of dragging the current state toward the stale absolute measurement.
   Numerical check: at 30 deg/s / 25 m/s / 150 ms latency, the legacy target
   injected 3.75 m / 4.5 deg of backward drag per update (matching the
   observed 2.06 m median gt_err at >25 deg/s after the dt*K gain); the delta
   target injects zero with a perfect IMU and recovers a drifted prior to
   <0.07 m. Gains untouched; implausible deltas (>45 deg / >100 m) fall back
   to the legacy target for that update.
2. Bias path unified: callbackImu now buffers bias-corrected IMU (raw values
   still feed calibration), propagateState no longer double-subtracts, and the
   snap helper uses the corrected imu_meas directly. The T_prior/deskew
   integration path previously ran on RAW gyro — after RTK calibration set a
   nonzero bias it permanently disagreed with propagateState, worst during
   high-yaw-rate sweeps.
Apply the GICP result as a delta: `T_corr = candidate · T_prior⁻¹` (both at median scan
time → time-free world-frame correction), left-applied with the existing gains to the
*current* state; or re-propagate from the corrected scan-time state through buffered IMU
(FAST-LIO style). Removes the yaw-rate-proportional error without touching `Kq`.
Bonus, same area: subtract `state.b.gyro/accel` when buffering IMU (prior/deskew path
currently integrates uncorrected gyro, unlike `propagateState`).

### P4 — Densify geometry: scan side then map side  [GICP + GLIM, A/B]
**STATUS: IMPLEMENTED 2026-07-04** (configs + diagnostics; the A/B replays
themselves still need to be run)
1. Scan voxel default 0.5 → 0.3 m in localization.yaml, with the A/B ladder
   (0.5 / 0.3 / 0.25) and the gicp_ms p99 watchpoint documented inline.
2. Dense GLIM localization-map profile: new
   `config_preprocess_dense_map.json` (0.4 m / 80k) and
   `config_sub_mapping_dense_map.json` (0.25 m voxel / 150k per submap),
   selectable via the commented-out lines in `config.json`. Mapping defaults
   unchanged. Re-baseline the GICP fitness floor after rebuilding maps —
   the P1 ratio thresholds assume it.
3. Merge diagnostics + parity: per-frame debug topics `merged_aux_count`,
   `aux<i>_merge_dt_s` (NaN = not merged), `aux<i>_points`,
   `scan_time_span_s`, plus the same fields in the SCAN DEBUG line — the
   run-12 audit could not reconstruct per-frame source sets. Concat
   `buffer_size` 20 → 200 (GLIM parity, ~20 s of aux history). Per-aux
   signed header-offset stats (mean/min/max vs primary) logged every 512
   merges, with an explicit warning when |mean| > 20 ms (constant-clock-
   offset signature worth absorbing upstream).
1. A/B scan voxel `dlio/preprocessing/voxelFilter/res` 0.5 → 0.3/0.25 (Codex 4); median
   scan is only ~6.9k pts, front-only frames 1–2k. Watch gicp_ms p99 (68 ms now).
2. Build a denser GLIM localization-map profile (Codex 5 + Claude): preprocess
   `downsample_resolution` 1.0 → 0.3–0.5 / higher `random_downsample_target`, submap voxel
   0.25–0.3, keep GICP runtime `map_voxel_size` as the only runtime knob. Re-measure the
   cross-run fitness floor after this — all P1 ratio thresholds assume re-baselining.
3. Merge diagnostics (both reviews): record per-frame merged-source count, per-aux merge
   dt, aux point counts, and scan time span into the debug bag (the audit could not
   reconstruct these); raise GICP concat `buffer_size` 20 → 200 (GLIM parity), and check
   for constant per-aux timestamp offset vs the P1 clock.

### P5 — GLIM map-quality follow-ups
1. Feed dual-antenna Atlas heading as attitude priors in the GNSS global module (position
   is currently the only constraint) — pins per-run map yaw at corners, shrinking cross-run
   disagreement (run 13's reverse pairing is worse: 80% acceptance, 204 fitness rejects).

   **STATUS: IMPLEMENTED 2026-07-04.** Chain audit correction: the tree already
   had `enable_orientation_prior: true` with yaw-only precisions
   ([1e-6, 1e-6, 1e2] ≈ 5.7° sigma) and the full data path was verified intact
   (adapter `rpyToQuat` deg→rad ✓, orientation on
   `/gps_p1/filtered_odom_rtk_fixed` ✓, `libgnss_global.so` loaded ✓,
   `PoseRotationPrior` body-frame error ≈ yaw for a level vehicle ✓) — so
   "position is the only constraint" was overstated for current HEAD. The
   REAL gap: the RTK filter qualifies POSITION quality only, so a
   position-FIXED sample with a degraded/unsolved dual-antenna heading
   (secondary-antenna outage, baseline multipath) fed a garbage yaw prior at
   full stiffness. Added a per-sample yaw-quality gate: `GNSSData` now carries
   `pose.covariance[35]` (populated by the adapter from Atlas rpy_covariance),
   interpolation takes the conservative max of the bracketing samples, and the
   heading prior is skipped (position prior kept, throttled warn) when
   reported yaw sigma exceeds `orientation_prior_max_yaw_sigma_deg` (3.0°;
   healthy Atlas heading is 0.1–0.3°). Unpopulated covariance passes, keeping
   covariance-less publishers on old behavior. NOTE: the 0630 run3/run5 maps
   predate validated heading priors — they take effect on the next map
   rebuild, which the P4 dense-profile rebuild requires anyway; validate the
   yaw sign convention on that rebuild before tightening the 1e2 precision.
2. A/B `fix_imu_bias: true → false` in odometry_gpu on one mapping run.
3. Optional exactness: re-point IMU frame to `pointonenav` in both stacks (lever-arm term
   currently dropped by design; small but free once tested).

**Codex review fixes 2026-07-04 (round 2):**
1. Debug evidence now ON by default: `localization/debug/enable_pub: true` and
   `verbose_scan_log: true` in localization.yaml — the P1/P4 diagnostics were
   implemented but gated behind flags the YAML disabled, so replay bags would
   have been empty of them (cost: 14 MB bag + ~10 log lines/s on run 12).
2. GLIM merge-diagnostics parity: `merge_clouds` now emits one parseable
   "CONCAT DEBUG | stamp=… merged=n/N dt<i>=…s pts<i>=… total_pts=…" INFO line
   per primary scan (`lidar_concat.frame_diag_log: true`, default ON — GLIM's
   offline tools have no node to publish topics from), plus per-aux running
   offset stats every 512 merges with the >20 ms constant-offset warning.
   All three merge_clouds call sites updated.
3. Warm-up window closed: `gicp/fitnessBaseline/seedBaseline` (set to 0.28,
   the run-12 sparse cross-run floor) makes the ratio gates and yaw veto live
   from frame 1; the rolling median takes over after minSamples. RE-MEASURE
   the seed after the dense-map rebuild.
4. Root README `min_consecutive_failures` drift fixed (1 → 5).

**P1 YAW-SAFETY ADDENDUM (2026-07-05, runs 19/20):** the P1–P4 build raised
acceptance to 91–95% but exposed a severe yaw hole: GICP rotation proposals of
45–47° passed the generic jump gate (30° + 60°/s·dt ⇒ 36–45° effective) and
the ratio-armed yaw veto (plausible fitness ⇒ veto never armed), producing
78.5°/98.6° max heading error vs RTK. Four fixes implemented:
1. **Unconditional hard yaw-veto tier** (`gicp/yawGate/hardMaxCorrDeg: 8.0`) —
   fires independent of fitness ratio; the IMU prior yaw cannot be 8° wrong
   over one scan gap on a ground vehicle. Soft tier (1.5° @ ratio>1.2) kept.
2. **Yaw split from the rotation jump gate** (`localization/jump/yaw_max_deg:
   10`, `yaw_dt_scale_deg: 10`, `yaw_total_max_deg: 15` absolute cap) — new
   status `rejected_yaw`; evaluated on the applied (post-veto) pose so vetoed
   frames keep their translation. Relocalization remains the GT snap's job.
3. **Observer orientation clamp** (`odom/geo/max_yaw_correction_deg: 5.0`,
   `max_rot_correction_deg: 0`) — the error's body-z component is clamped
   before the `dt_eff·Kq` gain, bounding one bad accepted scan to ≤3°/update
   while leaving >20°/s of legitimate correction authority.
4. **Yaw-failure-first scorecard**: new `## Yaw safety` section (accepted
   gt_rot by status, bad-yaw accept count [run-12 baseline: 317 at >10°],
   accepted yaw-innovation RAW vs FINAL percentiles, veto effectiveness,
   `rejected_yaw` count) + new debug topic `yaw_innovation_deg` and
   `yaw_innov=[raw,fin]` SCAN DEBUG field.
Defense-in-depth ordering: hard veto (zero yaw, keep translation) → yaw
innovation gate (reject if a big yaw still reaches the applied pose) →
observer clamp (bound damage if both miss). Trade-off: genuine yaw drift
beyond 8° recovers via GT snap / relocalization, not via GICP in normal
tracking — by design.

**ALGORITHMIC YAW-DEFECT FIXES (2026-07-05, deep-cause follow-up):** the gates
above bound damage after the fact; these remove the failure at the source.
1. **[P1a] In-optimizer attitude constraints** (`nano_gicp/lsq_registration`):
   the LM/GN solve gains (a) a **DoF mask** — `gicp/dof/mode: "4dof"` (new
   default) fixes roll/pitch to the IMU prior inside the optimizer, `"3dof"`
   also fixes yaw, with `full6dofEveryN: 10` unconstrained scans so roll/pitch
   re-anchor to the map; and (b) a **soft attitude prior**
   (`gicp/prior/yawInfo`, `rollPitchInfo`, rad⁻², included in both the normal
   equations and the LM error so ρ stays consistent) targeting the
   IMU-integrated initial guess — the missing IMU yaw term that let repeated
   map structure pull yaw into plausible wrong basins unopposed. The tangent
   is left/world so masking ω components constrains attitude exactly
   (rotations are center-independent); the mask pins masked diagonals at the
   unmasked scale so the downstream degeneracy analysis reads "externally
   constrained", not "null".
2. **[P2a] Honest final-pose fitness** (`NanoGICP::getFitnessScoreAtFinal`):
   on non-converged scans the cached score reflects the last LINEARIZATION
   pose; acceptance ("effectively converged") now gates on
   max(cached, recomputed-at-final) — one kd-tree pass, only on suspect frames.
3. **[P3] Constant aux clock-offset correction**
   (`localization/lidar_concat/aux_time_offsets`): applied to matching,
   rebasing, AND Luminar absolute per-point times, so a constant aux-vs-IMU
   clock offset (runs 19/20 measured 80–90 ms) no longer warps the merged
   scan during turns into false yaw pressure. Value comes straight off the
   per-aux offset diagnostic.
4. **Calibration instrument**: new `debug/yaw_marginal_stiffness` topic +
   `yaw_stiff=` SCAN DEBUG field — the Schur complement of the
   vehicle-re-centered hessian w.r.t. everything but yaw, i.e. the true
   per-scan yaw information. The scorecard reports its percentiles and
   suggests `gicp/prior/yawInfo ≈ 0.2× median`. Priors ship 0 (off) until
   calibrated from a replay; the 4-DoF mask is deterministic and ships ON.
[P1b] (plausible fitness in wrong basins) is addressed jointly by the ratio
gates (earlier) + the in-optimizer prior/DoF (this change); the earlier
yaw-safety gates remain as the outer defense layers.

**PR #6 incorporated (2026-07-06, "Bound non-converged GICP fitness
fallback"):** complementary to the above — the non-converged low-fitness
fallback ("effectively converged") is now bounded by correction-size gates
(`gicp/nonConvergedFitnessOkMaxTransM: 3.0`,
`nonConvergedFitnessOkMaxRotDeg: 5.0`; PR-validated: nonconv accepts
1145→397, accepts with INS err ≥50 m 99→3, p90 21.6→12.7 m). Out-of-bounds
non-converged candidates classify as `failed_to_converge`, so the
consecutive-failure counter keeps counting and GT recovery can engage. The
debug `converged` topic now publishes this same bounded decision, and the
build-breaking Eigen ternary around `initial_guess` is fixed in-tree (no
more per-worktree hot patches). Two deliberate deltas from the PR as posted:
the bound evaluates the APPLIED pose (post yaw-veto/projection — projection
only shrinks corrections, and the applied pose is what matters), and the
fitness it consults is the P2a honest max(cached, at-final) score, so the
two fixes compose.

## Validation (every step)
**Tooling committed 2026-07-04:** `scripts/analyze_scan_debug_log.py` computes the full
scorecard from a `localization.log` — status/acceptance/streaks, gicp_ms percentiles with
the P4#1 watchpoint, fitness floor + suggested P1 ratio thresholds, gt_err percentiles +
yaw-rate bucket table + bad-accept count, P1 partial/veto engagement, and per-frame
lidar_concat coverage/offsets (post-P4 logs). The pre-upgrade run-12 baseline is locked in
`docs/scorecard_baseline_run12_preP1-P4.md`; diff every new replay against it.
The dense GLIM profile is now the ACTIVE DEFAULT in `config.json` (upgraded from opt-in).

**Still requires real bag replays (cannot be produced offline):**
1. Rebuild the run3 + run5 maps with the dense profile; note map PCD point counts.
2. Replay both cross pairs on the P1–P4 build; run the scorecard on each log.
3. From the new scorecard: confirm gicp_ms p99 stays under the 80 ms watchpoint at scan
   voxel 0.3 (else 0.4); read the re-baselined fitness floor; adopt the script's suggested
   `yawGate/fitnessRatio` (p95) and `fitnessRatioRejectThreshold` (max(1.5, p99.9)) into
   localization.yaml.
4. One pass with `gt_recovery/enable=false` to expose true dead-reckoning behavior.
Plan gates: streak max < 20; bad accepts ~0; turning-bucket p90 ≤ 1.5× straight p90.

Replay both cross pairs (run3↔run5). Score: accepted-frame gt_pos p95/p99; yaw-rate-bucket
table (0–2/2–10/10–25/25–90 °/s); streak-length distribution; bad-accept count
(gt_err>20 m); one pass with GT recovery disabled.
