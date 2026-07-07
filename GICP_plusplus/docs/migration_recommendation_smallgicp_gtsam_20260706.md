# Migration Recommendation — gicp_localization Upstream Algorithms (2026-07-06)

Question under evaluation: migrate the localizer's registration/estimation core
away from the vendored NanoGICP fork, with two candidate paths —
**small_gicp** (short-term contained swap) and **gtsam_points + GTSAM**
(medium-term fixed-lag factor graph).

## Recommendation (summary)

**Adopt both, staged, with explicit decision gates — and do them in that
order.** The two are not alternatives: small_gicp is a *registration kernel*,
gtsam_points is an *estimator architecture*. The yaw saga of runs 12–20 showed
two distinct problem layers, and each path fixes one of them properly:

| Layer | Today (hardened NanoGICP) | small_gicp (Phase 1) | gtsam_points (Phase 2) |
|---|---|---|---|
| Registration kernel | frozen fork of fast_gicp lineage + our patched LM loop | maintained upstream, supported constraint hooks | gtsam_points VGICP/GICP factors |
| Yaw/DoF constraints | hand-rolled into `lsq_registration.cc` (2026-07-05) | **supported API** (`RestrictDoFFactor` + custom GeneralFactor) | native — priors are just factors |
| IMU/heading/GNSS fusion | outside the solve (prior → gates → observer) | still outside (same architecture) | **inside the MAP estimate** |
| Wrong-basin handling | veto/gate/clamp defense layers | same layers, better-conditioned kernel | robust loss + marginal covariance |

Phase 0 (this week) is non-negotiable either way: **replay-validate the
2026-07-05/06 fixes** (4-DoF mask, hard yaw veto, PR#6 bounded fallback). They
de-risk the current stack, and their scorecard gates become the acceptance
criteria for both migrations.

---

## Where we are, and why staying put is a liability

The current stack works but carries structural debt:

1. `nano_gicp` is a **frozen vendored fork** (VECTR DLIO → fast_gicp lineage).
   Every fix we've made — cached-fitness correctness, the honest final-pose
   score, the DoF mask, the attitude prior — is a patch to an LM loop nobody
   upstream maintains. The 2026-07-05 constraints work, but they live in
   *our* fork of *their* fork.
2. The estimator is a **hand-rolled geometric observer + gate stack**. After
   P1–P3 + yaw-safety + PR#6 it has ~10 interacting acceptance knobs. Each
   was individually justified by replay evidence, but the architecture is
   accumulating epicycles: the optimizer proposes unconstrained poses, then
   five defense layers repair the damage. The deep-cause report said it
   plainly: the IMU/heading information belongs *inside* the optimization.
3. Registration is the compute bottleneck (`gicp_ms` p99 = 68 ms against a
   100 ms budget), which is what forced the P4 density compromises.

## Phase 1 — small_gicp as the registration kernel (short-term, ~1–2 weeks)

**Verified facts** (upstream master, JOSS v1.0.0): MIT license; header-only;
deps = Eigen + bundled nanoflann/Sophus (no new workspace deps); PCL drop-in
interface (`RegistrationPCL`, PCL ≥ 1.11); ~2× faster than the fast_gicp
lineage single-threaded with better OMP/TBB scaling; parallel preprocessing
(voxel sampling, kd-tree build, covariance estimation); `RegistrationResult`
exposes `converged`, `iterations`, `num_inliers`, **final `H` (6×6), `b`,
`e`** — i.e., the exact quantities our gating machinery consumes. Critically,
the `Registration<PointFactor, Reduction, GeneralFactor>` template has a
**supported GeneralFactor hook**: the stock `RestrictDoFFactor` implements
soft DoF restriction via per-axis masks (rx, ry, rz, tx, ty, tz; λ-weighted
Tikhonov on the masked axes), and a custom GeneralFactor receives
`(T, H*, b*, e*)` per linearization — precisely the interface we hand-carved
into `lsq_registration.cc` for the attitude prior, but as a public API.

**What the port looks like:**

- Keep the node architecture unchanged: IMU prior → registration → ratio/
  yaw/jump gates → observer. Only the `this->gicp.*` kernel swaps, behind a
  thin interface (both expose pcl-style or direct-align APIs).
- 4-DoF mode → `RestrictDoFFactor` with rotation mask (1,1,0)… **verify the
  tangent convention during the port**: small_gicp perturbs with Sophus
  `se3_exp` — if right-multiplicative (body frame), the rotation mask is
  natively about the *vehicle*, and our world-frame re-centering machinery
  (hessian recentring, marginal-yaw Schur) simplifies or disappears. This is
  the single most important port detail to pin down first.
- Soft yaw prior → custom GeneralFactor (≈40 lines, mirrors our
  `apply_constraints`), calibrated by the same `yaw_marginal_stiffness`
  instrument.
- Honest fitness → `result.e / num_inliers` is already evaluated at the final
  pose; the P2a workaround becomes unnecessary.
- Map handling: optional upgrade from full-map kd-tree to
  `GaussianVoxelMap` + VGICP (bounded memory, no kd-tree build on the 1.6M+
  point map; helps the P4 dense-map plan directly).
- Everything else — fitness-ratio gates, yaw veto/innovation gates, observer,
  snap, scorecard — is backend-agnostic and stays. Thresholds tied to
  *fitness magnitude* must be re-baselined (error definitions differ);
  ratio-based gates and the seed/scorecard workflow absorb that by design.

**Effort:** ~1–2 weeks including a full cross-pair replay A/B.
**Risks:** threshold re-baselining (mitigated by the ratio-gate design);
tangent-convention mistakes (mitigate: port the Python-simulation test rig
first); losing our nano_gicp micro-optimizations (irrelevant — small_gicp is
faster wholesale).

## Phase 2 — gtsam_points fixed-lag localizer (medium-term, ~1–2 months)

**Verified facts:** GTSAM + gtsam_points **1.2.0 are already mandatory build
deps of the in-tree GLIM** (`GLIM/glim/CMakeLists.txt`), including the
optional CUDA path — Phase 2 adds no new external dependency to the
workspace, and GLIM's `odometry_estimation_{gpu,cpu,ct}` modules are working
in-tree reference implementations of exactly this estimator pattern.

**Architecture** ("GLIM-loc": GLIM's odometry front-end against a fixed map):
`IncrementalFixedLagSmoother` (lag 1–5 s) over scan-rate states
`X_i = {T, v, b}`, with the factor set the report calls for:

1. **IMU preintegration** — `gtsam::CombinedImuFactor` between consecutive
   states from `/gps_p1/imu` (Atlas imu_calibrated → tight bias random-walk
   priors, reusing GLIM's `config_sensors` noise model rationale). This IS
   the missing in-optimizer IMU term, done properly: yaw wrong-basins now
   fight the preintegrated rotation factor, not a post-hoc gate.
2. **Atlas dual-antenna heading factor** — yaw-only `PoseRotationPrior` per
   state, gated by reported yaw covariance. The P5 gnss_global work (yaw
   quality gate, precisions, frame conventions) ports verbatim.
3. **GNSS position factor** — RTK-gated unary translation prior from
   `filtered_odom`. *Evaluation caveat to record:* once INS position is a
   factor, `gt_err` against the same INS is no longer independent —
   validation must add held-out segments (factor disabled) or the scorecard
   claims become circular.
4. **Scan-to-map factor** — `IntegratedVGICPFactor` against a
   `GaussianVoxelMap` built once from the PCD (GPU variant available and
   already built for GLIM); this replaces align()+gates as the measurement.
5. **Scan-to-scan factor** — `IntegratedGICPFactor` between consecutive
   states for continuity through map-degenerate zones (the 253-frame-streak
   scenario stops being a cliff: the graph coasts on IMU + scan-to-scan).
6. **Robust loss + degeneracy-aware covariance** — Huber/Geman-McClure on
   the scan factors (wrong-basin residuals get downweighted instead of
   binary-gated), and per-state marginal covariance from the smoother
   replaces our recentered-Schur diagnostic as the honest confidence output
   (publish it; downstream consumers finally get a real covariance).

Output timing mirrors GLIM: IMU-rate prediction from the newest state
(replaces `propagateState`), smoother update per scan. The delta-form
observer, veto/clamp layers, and most gates **retire**; the yaw innovation
gate and scorecard stay as cheap independent watchdogs (defense in depth
against smoother divergence, which is Phase 2's characteristic new risk).

**Effort:** ~1–2 months to validated parity (state machine, initialization/
relocalization design, latency work, two full replay campaigns).
**Risks:** smoother divergence/indeterminate-system exceptions (mitigate:
GLIM's damping patterns + watchdog gates + snap-reset path); compute (VGICP
factor at 10 Hz with lag 3 s is proven by GLIM itself on this hardware, GPU
optional); a real architecture change during race season (mitigate: run
shadow-mode alongside the Phase 1 localizer before cutover).

## Decision gates

- **Gate 0 → 1:** current-stack replay shows yaw-safety fixes hold
  (bad-yaw accepts ≈ 0). Proceed with Phase 1 regardless of margin — the
  maintainability argument stands alone. If `gicp_ms` p99 is still >50 ms,
  Phase 1 also buys the density headroom P4 wants.
- **Gate 1 → 2:** Phase 1 A/B must be ≥ parity on the full scorecard
  (acceptance, gt_pos/gt_rot percentiles, streaks, yaw section) at lower
  `gicp_ms`. Start Phase 2 only after Phase 1 ships, since its replay corpus
  and thresholds become Phase 2's baseline.
- **Gate 2 cutover:** shadow-mode ≥ 3 full replays + 1 live session with the
  factor-graph localizer publishing alongside; cut over when its marginal-
  covariance-gated output beats the Phase 1 stack on gt_rot p99 and never
  diverges (no unbounded error without covariance blow-up flagging it).

## What survives every phase (deliberately backend-agnostic)

The evidence/validation layer: SCAN DEBUG schema + debug topics, the
scorecard and its plan gates, the per-aux clock-offset machinery, the
adapter, GT snap, and the yaw-innovation watchdog. That was the point of
building them as wrappers rather than into the solver.
