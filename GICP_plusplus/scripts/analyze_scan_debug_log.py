#!/usr/bin/env python3
"""Turnkey replay scorecard from a gicp_localization SCAN DEBUG log.

This is the measurement half of the P1-P4 validation loop
(docs/action_plan_turn_error_20260704.md). Run it on localization.log after
every replay; diff the output between runs (old vs new gates, sparse vs dense
map, scan voxel A/B). It computes every quantity the action plan gates on:

  * status counts / acceptance (incl. ok_partial), rejection-streak histogram
  * gicp_ms percentiles + p99-vs-scan-period watchpoint          (P4#1)
  * fitness floor (p10/median) -> cross-run re-baseline          (P4#2)
  * fitness-ratio distribution + suggested P1 ratio thresholds   (P1)
  * gt_err percentiles, yaw-rate bucket table, bad-accept count  (P1/P3)
  * degeneracy/yaw-veto engagement rates                          (P1)
  * lidar_concat per-frame coverage, per-aux dt, sweep span       (P4#3)

Works on both pre- and post-P1 log formats (new fields are optional).

Usage:  analyze_scan_debug_log.py <localization.log> [--period 0.1] [--gt-bad 20]
"""

import argparse
import math
import re
import sys
from collections import Counter


def pct(sorted_vals, p):
    if not sorted_vals:
        return float("nan")
    i = min(len(sorted_vals) - 1, max(0, int(round(p / 100.0 * (len(sorted_vals) - 1)))))
    return sorted_vals[i]


def fmt(v, nd=3):
    return "n/a" if v is None or (isinstance(v, float) and math.isnan(v)) else f"{v:.{nd}f}"


ROW = re.compile(
    r"SCAN DEBUG \| status=(?P<status>\w+) stamp=(?P<stamp>[\d.]+)"
    r".*?guess=\{xyz=\[(?P<gx>[-\d.,]+)\] rpy_deg=\[(?P<grpy>[-\d.,]+)\]\}"
    r".*?gicp_ms=(?P<ms>[-\d.]+|n/a)"
    r".*?fitness=(?P<fit>[-\d.eE+]+|n/a)"
    r"(?:.*?fit_ratio=(?P<ratio>[-\d.]+|n/a))?"
    r"(?:.*?degen=\[r(?P<dr>\d+),t(?P<dt>\d+),yaw_veto=(?P<yv>\d),partial=(?P<pu>\d)\])?"
    r"(?:.*?yaw_innov=\[(?P<yi>-?[\d.]+|nan)deg,fin=(?P<yif>-?[\d.]+|nan)deg\])?"
    r"(?:.*?yaw_stiff=(?P<ys>-?[\d.]+|n/a))?"
    r"(?:.*?concat=\[(?P<cn>-?\d+)/(?P<ct>\d+)(?P<cdetail>[^\]]*)\])?"
    r".*?hessian_cond=(?P<hess>[-\d.eE+]+|n/a|inf)"
    r"(?:.*?gt_err=\[(?P<gtp>[\d.]+)m,(?P<gtr>[\d.]+)deg)?"
)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--period", type=float, default=0.1, help="scan period s (watchpoint)")
    ap.add_argument("--gt-bad", type=float, default=20.0, help="bad-accept gt_err threshold m")
    ap.add_argument("--gt-bad-rot", type=float, default=10.0, help="bad-accept gt_rot threshold deg")
    args = ap.parse_args()

    rows = []
    for line in open(args.log, errors="ignore"):
        m = ROW.search(line)
        if not m:
            continue
        d = m.groupdict()
        try:
            yaw = float(d["grpy"].split(",")[2])
        except (ValueError, IndexError):
            yaw = float("nan")
        rows.append(
            dict(
                st=d["status"],
                t=float(d["stamp"]),
                yaw=yaw,
                ms=float(d["ms"]) if d["ms"] not in (None, "n/a") else float("nan"),
                fit=float(d["fit"]) if d["fit"] not in (None, "n/a") else float("nan"),
                ratio=float(d["ratio"]) if d["ratio"] not in (None, "n/a") else float("nan"),
                yv=int(d["yv"]) if d["yv"] else 0,
                yi=float(d["yi"]) if d["yi"] not in (None, "nan") else float("nan"),
                yif=float(d["yif"]) if d["yif"] not in (None, "nan") else float("nan"),
                ys=float(d["ys"]) if d["ys"] not in (None, "n/a") else float("nan"),
                pu=int(d["pu"]) if d["pu"] else 0,
                dr=int(d["dr"]) if d["dr"] else 0,
                dtx=int(d["dt"]) if d["dt"] else 0,
                cn=int(d["cn"]) if d["cn"] else None,
                ct=int(d["ct"]) if d["ct"] else None,
                cdetail=d["cdetail"] or "",
                gtp=float(d["gtp"]) if d["gtp"] else float("nan"),
                gtr=float(d["gtr"]) if d["gtr"] else float("nan"),
            )
        )
    if not rows:
        sys.exit("no SCAN DEBUG rows found")

    n = len(rows)
    print(f"# SCAN DEBUG scorecard — {args.log}\nframes: {n}")

    # --- status / acceptance / streaks ---
    counts = Counter(r["st"] for r in rows)
    ok_like = {"ok", "ok_partial", "large_jump"}
    acc = sum(v for k, v in counts.items() if k in ok_like)
    print("\n## Status")
    for k, v in counts.most_common():
        print(f"  {k:24s} {v:6d}  ({v/n:.2%})")
    print(f"  acceptance: {acc/n:.2%}  (ok_partial share of accepts: "
          f"{counts.get('ok_partial',0)/max(1,acc):.2%})")
    streaks, cur = [], 0
    for r in rows:
        if r["st"] in ok_like:
            if cur:
                streaks.append(cur)
            cur = 0
        else:
            cur += 1
    if cur:
        streaks.append(cur)
    long_s = [s for s in streaks if s >= 10]
    print(f"  rejection streaks: n={len(streaks)} max={max(streaks) if streaks else 0} "
          f">=10: {len(long_s)} (frames in them: {sum(long_s)})   [plan gate: max < 20]")

    # --- gicp_ms ---
    ms = sorted(r["ms"] for r in rows if not math.isnan(r["ms"]))
    p99 = pct(ms, 99)
    print("\n## gicp_ms (P4#1 watchpoint)")
    print(f"  median={fmt(pct(ms,50),2)} p90={fmt(pct(ms,90),2)} p95={fmt(pct(ms,95),2)} "
          f"p99={fmt(p99,2)} max={fmt(ms[-1] if ms else float('nan'),2)}")
    if ms and p99 > 0.8 * args.period * 1e3:
        print(f"  WARNING: p99 > 80% of the {args.period*1e3:.0f} ms scan period — back off scan voxel")

    # --- fitness floor + ratio thresholds ---
    accf = sorted(r["fit"] for r in rows if r["st"] in ok_like and not math.isnan(r["fit"]))
    print("\n## Fitness floor (P4#2 re-baseline)")
    print(f"  accepted-frame fitness: p10={fmt(pct(accf,10))} median={fmt(pct(accf,50))} "
          f"p90={fmt(pct(accf,90))}")
    ratios = sorted(r["ratio"] for r in rows if r["st"] in ok_like and not math.isnan(r["ratio"]))
    if ratios:
        print(f"  fitness_ratio (accepted): median={fmt(pct(ratios,50),2)} p95={fmt(pct(ratios,95),2)} "
              f"p99={fmt(pct(ratios,99),2)} p99.9={fmt(pct(ratios,99.9),2)}")
        print(f"  suggested gicp/yawGate/fitnessRatio      ~ {fmt(pct(ratios,95),2)} (p95)")
        print(f"  suggested gicp/fitnessRatioRejectThreshold ~ {fmt(max(1.5, pct(ratios,99.9)),2)} "
              f"(max(1.5, p99.9))")
    else:
        print("  (no fit_ratio field — pre-P1 log)")

    # --- gt error ---
    gtrows = [r for r in rows if not math.isnan(r["gtp"])]
    accg = sorted(r["gtp"] for r in gtrows if r["st"] in ok_like)
    bad = [r for r in gtrows if r["st"] in ok_like and r["gtp"] > args.gt_bad]
    print("\n## GT error")
    print(f"  accepted gt_pos: median={fmt(pct(accg,50),2)} p90={fmt(pct(accg,90),2)} "
          f"p95={fmt(pct(accg,95),2)} p99={fmt(pct(accg,99),2)} max={fmt(accg[-1] if accg else float('nan'),1)}")
    print(f"  bad accepts (gt>{args.gt_bad:.0f} m): {len(bad)}   [plan gate: ~0]")

    # yaw-rate bucket table (turn-error signature; P3 gate)
    print("\n  yaw-rate buckets (accepted frames):")
    prev = None
    bucketed = {b: [] for b in ((0, 2), (2, 10), (10, 25), (25, 90))}
    for r in rows:
        if prev is not None:
            dt = r["t"] - prev["t"]
            dy = r["yaw"] - prev["yaw"]
            while dy > 180:
                dy -= 360
            while dy < -180:
                dy += 360
            if 0 < dt < 1 and r["st"] in ok_like and not math.isnan(r["gtp"]):
                yr = abs(dy / dt)
                for b in bucketed:
                    if b[0] <= yr < b[1]:
                        bucketed[b].append((r["gtp"], r["gtr"]))
        prev = r
    for b, vals in bucketed.items():
        if not vals:
            continue
        gp = sorted(v[0] for v in vals)
        gr = sorted(v[1] for v in vals)
        print(f"    {b[0]:>2}-{b[1]:<2} deg/s: n={len(vals):5d}  gt_pos median={fmt(pct(gp,50),2)} "
              f"p90={fmt(pct(gp,90),2)} | gt_rot median={fmt(pct(gr,50),2)} p90={fmt(pct(gr,90),2)}")
    print("    [plan gate: turning p90 within 1.5x of straight p90]")

    # --- Yaw safety (yaw-failure-first reporting; P1 yaw bug runs 19/20) ---
    print("\n## Yaw safety")
    for name, grp in (("ok", [r for r in rows if r["st"] == "ok"]),
                      ("ok_partial", [r for r in rows if r["st"] == "ok_partial"]),
                      ("all accepted", [r for r in rows if r["st"] in ok_like])):
        rg = sorted(r["gtr"] for r in grp if not math.isnan(r["gtr"]))
        if rg:
            print(f"  gt_rot [{name:12s}]: median={fmt(pct(rg,50),2)} p95={fmt(pct(rg,95),2)} "
                  f"p99={fmt(pct(rg,99),2)} max={fmt(rg[-1],1)} deg  (n={len(rg)})")
    bad_yaw = [r for r in rows if r["st"] in ok_like and not math.isnan(r["gtr"])
               and r["gtr"] > args.gt_bad_rot]
    print(f"  bad-yaw accepts (gt_rot>{args.gt_bad_rot:.0f} deg): {len(bad_yaw)}   [plan gate: ~0]")
    yi_acc = sorted(r["yi"] for r in rows if r["st"] in ok_like and not math.isnan(r["yi"]))
    yif_acc = sorted(r["yif"] for r in rows if r["st"] in ok_like and not math.isnan(r["yif"]))
    if yi_acc:
        print(f"  accepted yaw innovation RAW  : median={fmt(pct(yi_acc,50),2)} p99={fmt(pct(yi_acc,99),2)} "
              f"max={fmt(yi_acc[-1],1)} deg")
        print(f"  accepted yaw innovation FINAL: median={fmt(pct(yif_acc,50),2)} p99={fmt(pct(yif_acc,99),2)} "
              f"max={fmt(yif_acc[-1],1)} deg   [applied pose; large RAW->small FINAL = veto working]")
    vet = [r for r in rows if r["yv"]]
    if vet:
        vg = sorted(r["gtr"] for r in vet if not math.isnan(r["gtr"]))
        print(f"  yaw vetoes: {len(vet)}; vetoed-frame gt_rot median={fmt(pct(vg,50),2)} "
              f"p95={fmt(pct(vg,95),2)} deg  (low = veto correctly kept IMU yaw)")
    stiff = sorted(r["ys"] for r in rows if not math.isnan(r["ys"]) and r["ys"] >= 0)
    if stiff:
        print(f"  yaw marginal stiffness: p10={fmt(pct(stiff,10),1)} median={fmt(pct(stiff,50),1)} "
              f"p90={fmt(pct(stiff,90),1)}  -> suggested gicp/prior/yawInfo ~ {fmt(0.2*pct(stiff,50),1)} (0.2x median)")
    ry = counts.get("rejected_yaw", 0)
    print(f"  rejected_yaw frames: {ry}  (hard innovation gate; nonzero means the veto tier was bypassed)")

    # --- P1 engagement ---
    if any(r["pu"] or r["yv"] for r in rows):
        print("\n## P1 engagement")
        print(f"  partial updates: {sum(r['pu'] for r in rows)} "
              f"({sum(r['pu'] for r in rows)/n:.2%}); yaw vetoes: {sum(r['yv'] for r in rows)}; "
              f"mean degen axes r={sum(r['dr'] for r in rows)/n:.2f} t={sum(r['dtx'] for r in rows)/n:.2f}")

    # --- concat coverage (P4#3) ---
    cc = [r for r in rows if r["cn"] is not None and r["cn"] >= 0]
    if cc:
        total_aux = cc[0]["ct"]
        cov = Counter(r["cn"] for r in cc)
        print("\n## lidar_concat coverage (per frame, P4#3)")
        for k in sorted(cov, reverse=True):
            print(f"  merged {k}/{total_aux}: {cov[k]:6d} ({cov[k]/len(cc):.2%})")
        for i in range(total_aux):
            dts = []
            for r in cc:
                # tolerate platform NaN spellings: nan, -nan, -nan(ind)
                m = re.search(rf"dt{i}=(-?(?:[\d.]+|nan(?:\(ind\))?))s", r["cdetail"])
                if m and "nan" not in m.group(1):
                    dts.append(float(m.group(1)))
            if dts:
                dts.sort()
                mean = sum(dts) / len(dts)
                flag = "  <-- constant offset? consider per-aux time correction" if abs(mean) > 0.02 else ""
                print(f"  aux{i} merge dt: mean={mean*1e3:+.1f} ms  p5={pct(dts,5)*1e3:+.1f}  "
                      f"p95={pct(dts,95)*1e3:+.1f} ms  n={len(dts)}{flag}")
        spans = sorted(float(m.group(1)) for r in cc
                       for m in [re.search(r"span=([\d.]+)s", r["cdetail"])] if m)
        if spans:
            print(f"  sweep span: median={fmt(pct(spans,50))} p99={fmt(pct(spans,99))} s")
    else:
        print("\n## lidar_concat coverage: no per-frame concat fields (pre-P4 log)")


if __name__ == "__main__":
    main()
