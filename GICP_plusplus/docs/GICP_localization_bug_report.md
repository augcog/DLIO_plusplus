# GICP Localization — Bug Report

| Field | Value |
|-------|-------|
| **Component** | `gicp_localization` (`LocalizationNode`, `NanoGICP`) |
| **Config reference** | `gicp_localization/cfg/localization.yaml` (AV-24 defaults) |
| **URDF reference** | `av24.urdf` |
| **Date** | 2026-05-26 |
| **Status** | Open |

---

## Executive summary

Review of lever-arm handling, IMU initialization, and kd-tree warm-start in `gicp_localization` found **one high-severity frame-consistency defect** on the default AV-24 configuration (`dlio/deskew: false`). The node registers **LiDAR-frame** point clouds against the map while using a **`base_frame` (`novatel_a`) pose** as the GICP initial guess and interpreting the optimizer output as a base pose—without applying the cached `base → lidar` extrinsic on that path.

Secondary issues include **inconsistent GT/RTK pose seeding** (no `T_base_gtbody` on bootstrap, unlike GT recovery snap) and **silent identity extrinsic fallback** when TF lookups fail.

Map kd-tree pre-build at startup and the IMU calibration state machine are **implemented correctly** for their intended roles. GNSS antenna lever-arm compensation is **intentionally absent** in this node (handled upstream).

---

## Affected configuration (defaults)

```yaml
localization/base_frame: "novatel_a"
localization/imu_frame: "novatel_a"
localization/lidar_frame: "luminar_front"
dlio/deskew: false
localization/use_odom_init: true
localization/rtk_init/enable: true
```

Approximate static offset `novatel_a` → `luminar_front` from `av24.urdf` (both mounted on `rear_axle_middle`):

| Axis | Δ (m) |
|------|-------|
| x | ~0.59 |
| y | ~0.08 |
| z | ~0.35 |
| **‖Δt‖** | **~0.69** |

---

## BUG-001 — GICP frame mismatch when deskew is disabled

| | |
|---|---|
| **Severity** | High |
| **Files** | `src/localization.cc` (`performLocalization`, `deskewPointcloud`) |
| **Trigger** | `dlio/deskew: false` and `base_frame` ≠ LiDAR `header.frame_id` |

### Description

With deskew disabled (default for Luminar due to collapsed per-point timestamps):

1. `current_scan` remains in the **sensor / LiDAR frame**.
2. `T_prior` is propagated from `lidarPose`, which stores the **`base_frame` pose in the map** (misleading name—see BUG-004).
3. GICP is called with `initial_guess = T_prior` and `candidate_pose = optimizer_solution` (no extrinsic chain).

With deskew **enabled**, the consistent path is used: transform points with `T_prior * baselink2lidar_T`, identity initial guess, `candidate_pose = optimizer_solution * T_prior`.

### Code references

```cpp
// performLocalization() — deskew OFF path
Eigen::Matrix4f initial_guess = this->deskew_ ? Eigen::Matrix4f::Identity() : this->T_prior;
// ...
const Eigen::Matrix4f candidate_pose = this->deskew_ ? optimizer_solution * this->T_prior : optimizer_solution;
```

```cpp
// deskewPointcloud() — deskew OFF: scan unchanged, T_prior from base pose integration
this->current_scan = this->original_scan;
auto frames = this->integrateImu(..., this->lidarPose.q, this->lidarPose.p, ...);
this->T_prior = frames[0];  // T_map_base
```

Extrinsic is cached for deskew but not applied to GICP when deskew is off:

```cpp
// callbackPointCloud — TF cached as baselink2lidar_T
this->extrinsics.baselink2lidar_T ...
```

### Expected behavior

GICP should estimate **`T_map_lidar`** when the source cloud is in the LiDAR frame, then convert to **`T_map_base`** for state and publications:

- `initial_guess = T_prior * T_base_lidar`
- `candidate_pose = T_map_lidar * inv(T_base_lidar)`  
  (equivalently: transform cloud to base/map before registration, matching the deskew-on pipeline).

### Actual behavior

GICP is seeded with **`T_map_base`** and the solution is stored as **`T_map_base`**, introducing a systematic error on the order of the base–LiDAR lever arm (~0.7 m on AV-24), plus rotation coupling in turns.

### Impact

- Biased localized pose vs ground truth (even when GT init is correct in `novatel_a`).
- Extra optimizer work; may be partially masked by large `gicp/maxCorrespondenceDistance` (4.0 m default).
- Jump / hessian rejection logic compares `T_prior` (base) to `candidate_pose` (effectively lidar)—inconsistent error metrics.

### Suggested fix

**Option A (minimal, deskew-off):** In `performLocalization()` when `!deskew_` and `extrinsics_cached_`:

```cpp
const Eigen::Matrix4f& T_bl = this->extrinsics.baselink2lidar_T;
initial_guess = this->T_prior * T_bl;
// after align:
candidate_pose = optimizer_solution * T_bl.inverse();
```

**Option B:** Transform `current_scan` to `base_frame` or map frame before GICP (mirror deskew-on math without per-point timing).

**Option C:** Re-enable deskew once Luminar per-point timestamps are valid (restores existing correct pipeline).

### Verification

1. Log `‖t(base_lidar)‖` from cached TF at startup.
2. Compare `gicp/localization/pose` to `/localization/global/odom` with deskew on vs off (same bag).
3. Expect deskew-off offset roughly constant in body frame, magnified in turns.

---

## BUG-002 — GT / RTK bootstrap skips `T_base_gtbody` transform

| | |
|---|---|
| **Severity** | Medium (conditional) |
| **Files** | `src/localization.cc` (`callbackGtOdom`, `tryRtkCalibrationStep`) |

### Description

`maybeSnapPoseToGT()` correctly maps GT pose into `base_frame`:

```cpp
// T_map_base = T_map_gtbody * inv(T_base_gtbody)
q_new = gt.q * q_gtbody_in_base.conjugate();
p_new = gt.p - q_new * t_base_gtbody;
```

The **first** GT message used for odometry init and RTK IMU seeding does **not** apply this transform:

```cpp
// callbackGtOdom — first GT only
this->applyInitialPose(s.p, s.q, ...);  // raw msg pose

// tryRtkCalibrationStep — state seed
this->state.p = this->latest_rtk_seed_.p;
this->state.q = this->latest_rtk_seed_.q;
```

### Expected behavior

Bootstrap and RTK seed poses should use the same `base_frame ← child_frame_id` composition as GT recovery snap.

### Actual behavior

Correct only when `msg->child_frame_id == localization/base_frame`. Wrong by the GT-body extrinsic otherwise until GICP or snap corrects.

### Impact

- **AV-24 default:** Usually OK if `/localization/global/odom` uses `child_frame_id: novatel_a` matching `base_frame`.
- **Failure mode:** Initial pose and RTK bias window seeded in wrong frame; transient error until first successful GICP or snap.

### Suggested fix

Factor a shared helper, e.g. `gtPoseToBaseFrame(const GtSample& gt) -> {p, q}`, used by `applyInitialPose`, `tryRtkCalibrationStep`, and `maybeSnapPoseToGT`.

---

## BUG-003 — Silent identity extrinsic when TF lookup fails

| | |
|---|---|
| **Severity** | Medium |
| **Files** | `src/localization.cc` (`callbackImu`, `callbackPointCloud`) |

### Description

If `lookupTransform(base_frame, imu_frame)` or `lookupTransform(base_frame, lidar_frame)` fails, the node logs a throttled warning and continues with **identity** extrinsics (`imu_extrinsics_cached_` may stay false for IMU; lidar callback **returns** without processing).

For IMU:

```cpp
} catch (const tf2::TransformException & ex) {
  RCLCPP_WARN_THROTTLE(..., "Cannot cache baselink->imu TF: %s (using identity)", ex.what());
}
// Lever-arm block skipped if !imu_extrinsics_cached_
```

### Expected behavior

Fail fast or refuse GICP until required transforms are available (same as lidar path blocking on missing TF).

### Actual behavior

IMU data processed without lever-arm rotation/centripetal correction; combined with BUG-001, wrong extrinsics prolong frame errors.

### Suggested fix

- Set a `extrinsics_required_` flag; reject scans / IMU propagation until both IMU and LiDAR TFs are cached.
- Or: retry TF lookup each message until success (no silent identity for LiDAR offset).

---

## BUG-004 — Misleading `lidarPose` identifier (maintainability)

| | |
|---|---|
| **Severity** | Low (documentation / maintenance) |
| **Files** | `src/localization.cc`, `include/gicp_localization/localization.h` |

### Description

`lidarPose` holds the tracked **`base_frame` pose** in the map, not the LiDAR pose. It is updated from `current_pose` after GICP and used as `integrateImu` seed on the deskew-off path—contributing to BUG-001 confusion.

### Suggested fix

Rename to `basePose` or `bodyPose`, or store true lidar pose and apply `T_base_lidar` at integration boundaries.

---

## BUG-005 — Unused `map_kdtree` member (dead code)

| | |
|---|---|
| **Severity** | Low |
| **Files** | `include/gicp_localization/localization.h` |

### Description

`std::shared_ptr<const nanoflann::KdTreeFLANN<PointType>> map_kdtree` is declared but never assigned. Map indexing is owned inside `NanoGICP::target_kdtree_` via `setInputTarget()` at node construction.

### Suggested fix

Remove the member or wire it for explicit map-tree sharing if `registerInputTarget()` is adopted later.

---

## Confirmed non-issues (for reviewers)

| Topic | Verdict |
|-------|---------|
| Map kd-tree warm-start | **OK** — `setInputTarget(map)` + `calculateTargetCovariances()` once at startup; per-scan `setInputSource` reuses nanoflann object (`nano_gicp.cc`). |
| IMU init (RTK + stationary fallback) | **OK** — State machine gates propagation on `imu_calibrated_`; `first_opt_done` from GT init does not skip bias calibration. |
| GNSS antenna lever arm in this node | **By design** — Not applied; upstream INS voter publishes `/localization/global/odom`. See repo `AGENTS.md` §6–7. |
| IMU centripetal lever arm | **Implemented** but **no-op on default YAML** (`imu_frame == base_frame == novatel_a`). |
| `callbackGtOdom` setting `first_opt_done` | **OK** — `applyInitialPose()` already initializes observer state (`AGENTS.md` §3). |
| Hessian rejection disjunction | **OK** — Matches documented truth table (`AGENTS.md` §4). |

---

## Design notes (not filed as bugs)

1. **`registerInputTarget()`** — API to register a cloud without rebuilding the kd-tree; localization uses `setInputTarget` instead. Not incorrect, only unused.
2. **`gt_recovery/min_consecutive_failures: 1`** — Aggressive snap after one rejected scan; intentional race-day tradeoff.
3. **`dlio/deskew: false`** — Reasonable for collapsed Luminar timestamps; does not remove the need for **static** lidar extrinsic handling in GICP (BUG-001).

---

## Recommended priority

| Priority | ID | Action |
|----------|-----|--------|
| P0 | BUG-001 | Fix base↔lidar frame handling on deskew-off path |
| P1 | BUG-002 | Unify GT pose composition for init, RTK seed, and snap |
| P2 | BUG-003 | Fail closed on missing TF |
| P3 | BUG-004, BUG-005 | Rename / cleanup |

---

## References

- `gicp_localization/src/localization.cc` — `performLocalization()` ~1837–1890, `deskewPointcloud()` ~1564–1594, `callbackGtOdom()` ~2347–2420, `maybeSnapPoseToGT()` ~2657–2687, `callbackImu()` ~2744–2794
- `gicp_localization/src/nano_gicp/nano_gicp.cc` — `setInputSource` / `setInputTarget` kd-tree reuse ~139–170
- `gicp_localization/cfg/localization.yaml` — frame IDs, deskew, RTK init
- `AGENTS.md` — confirmed non-issues and upstream GNSS assumptions
