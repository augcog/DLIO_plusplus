#!/usr/bin/env python3
"""Render a 3D "roller coaster" view of localization error on the map.

The XY position comes from the GNSS or GICP trajectory in map coordinates.
At each sample, a vertical colored bar rises from the physical trajectory by
the GICP-vs-GNSS error in meters, so high-error regions stand out without
moving the track laterally.

Example:
    python3 scripts/render_error_rollercoaster.py \
        --csv dlio_data/run_3_loc_gnss_live/live_error.csv \
        --map dlio_data/run_5_map.pcd \
        --out dlio_data/run_3_loc_gnss_live/error_rollercoaster.png
"""

import argparse
import csv
import re
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import colormaps
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d.art3d import Line3DCollection


def load_live_error_csv(path, position_source, error_field):
    cols = {
        "stamp": [],
        "x": [],
        "y": [],
        "z": [],
        "error": [],
    }
    px = f"{position_source}_x"
    py = f"{position_source}_y"
    pz = f"{position_source}_z"
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        missing = [k for k in (px, py, pz, error_field) if k not in reader.fieldnames]
        if missing:
            sys.exit(f"{path}: missing required columns: {', '.join(missing)}")
        for row in reader:
            try:
                cols["stamp"].append(float(row["stamp"]))
                cols["x"].append(float(row[px]))
                cols["y"].append(float(row[py]))
                cols["z"].append(float(row[pz]))
                cols["error"].append(float(row[error_field]))
            except (TypeError, ValueError):
                continue

    arr = {k: np.asarray(v, dtype=np.float64) for k, v in cols.items()}
    finite = np.isfinite(arr["x"]) & np.isfinite(arr["y"]) & np.isfinite(arr["z"]) & np.isfinite(arr["error"])
    finite &= arr["error"] >= 0
    return {k: v[finite] for k, v in arr.items()}


def load_pcd_xyz(path):
    """Minimal binary PCD reader for float32 FIELDS x y z [intensity]."""
    with open(path, "rb") as f:
        header = b""
        while not header.endswith(b"DATA binary\n"):
            line = f.readline()
            if not line:
                sys.exit(f"{path}: unsupported PCD, expected DATA binary")
            header += line
        text = header.decode(errors="replace")
        fields_match = re.search(r"^FIELDS (.+)$", text, re.M)
        points_match = re.search(r"^POINTS (\d+)$", text, re.M)
        if not fields_match or not points_match:
            sys.exit(f"{path}: invalid PCD header")
        fields = fields_match.group(1).split()
        if not {"x", "y", "z"}.issubset(fields):
            sys.exit(f"{path}: PCD must contain x y z fields")
        npts = int(points_match.group(1))
        data = np.fromfile(f, dtype=np.float32, count=npts * len(fields))
    data = data.reshape(-1, len(fields))
    return data[:, [fields.index("x"), fields.index("y"), fields.index("z")]].astype(np.float64)


def distance_resample(data, samples):
    xyz = np.column_stack([data["x"], data["y"], data["z"]])
    err = data["error"]
    if len(xyz) < 2:
        sys.exit("not enough trajectory samples")

    step = np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(step)])
    keep = np.concatenate([[True], np.diff(s) > 1e-4])
    s = s[keep]
    xyz = xyz[keep]
    err = err[keep]
    if len(s) < 2 or s[-1] <= 0:
        sys.exit("trajectory has no XY motion")

    n = min(max(2, samples), len(s))
    target = np.linspace(0.0, s[-1], n)
    out = np.column_stack([
        np.interp(target, s, xyz[:, 0]),
        np.interp(target, s, xyz[:, 1]),
        np.interp(target, s, xyz[:, 2]),
    ])
    out_err = np.interp(target, s, err)
    return out, out_err


def sample_map(map_path, traj_xyz, padding, max_points, seed):
    if not map_path:
        return np.empty((0, 3), dtype=np.float64)
    print(f"[error_rollercoaster] loading map {map_path} ...")
    xyz = load_pcd_xyz(map_path)
    lo = traj_xyz[:, :2].min(axis=0) - padding
    hi = traj_xyz[:, :2].max(axis=0) + padding
    m = (
        (xyz[:, 0] >= lo[0]) & (xyz[:, 0] <= hi[0]) &
        (xyz[:, 1] >= lo[1]) & (xyz[:, 1] <= hi[1])
    )
    xyz = xyz[m]
    print(f"[error_rollercoaster] {len(xyz):,} map points inside trajectory bbox")
    if len(xyz) > max_points:
        rng = np.random.default_rng(seed)
        idx = rng.choice(len(xyz), size=max_points, replace=False)
        xyz = xyz[idx]
    return xyz


def stats(values):
    return {
        "rms": float(np.sqrt(np.mean(values ** 2))),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def set_axes_equal_xy(ax, x, y, z):
    xr = float(np.ptp(x))
    yr = float(np.ptp(y))
    zr = float(np.ptp(z))
    ax.set_box_aspect((max(xr, 1.0), max(yr, 1.0), max(max(xr, yr) * 0.22, zr, 1.0)))


def render(args):
    data = load_live_error_csv(args.csv, args.position_source, args.error_field)
    raw_error = data["error"]
    raw_stats = stats(raw_error)
    traj, err = distance_resample(data, args.samples)
    map_xyz = sample_map(args.map, traj, args.map_padding, args.map_max_points, args.seed)

    base = traj
    shadow_base = traj.copy()
    shadow_base[:, 2] += args.baseline_z
    top = shadow_base.copy()
    top[:, 2] = shadow_base[:, 2] + err * args.z_scale

    fig = plt.figure(figsize=(args.width / args.dpi, args.height / args.dpi), dpi=args.dpi)
    ax = fig.add_subplot(111, projection="3d")
    fig.patch.set_facecolor("#101216")
    ax.set_facecolor("#101216")

    if len(map_xyz):
        zlo, zhi = np.percentile(map_xyz[:, 2], [2, 98])
        zn = np.clip((map_xyz[:, 2] - zlo) / max(zhi - zlo, 1e-6), 0, 1)
        gray = 0.28 + 0.42 * zn
        colors = np.column_stack([gray, gray, gray])
        ax.scatter(
            map_xyz[:, 0], map_xyz[:, 1], map_xyz[:, 2],
            c=colors, s=args.map_point_size, alpha=args.map_alpha,
            linewidths=0, depthshade=False, rasterized=True,
        )

    cmap = colormaps[args.cmap]
    color_max = args.color_max if args.color_max > 0 else np.percentile(raw_error, args.color_percentile)
    color_max = max(float(color_max), 1e-6)
    norm = Normalize(vmin=0.0, vmax=color_max)

    bar_segs = np.stack([shadow_base, top], axis=1)
    bar_colors = cmap(norm(err))
    bar_colors[:, 3] = args.curtain_alpha
    bars = Line3DCollection(bar_segs, colors=bar_colors, linewidth=args.line_width)
    ax.add_collection3d(bars)

    # Shadow/reference on the physical trajectory.
    ax.plot(
        shadow_base[:, 0], shadow_base[:, 1], shadow_base[:, 2],
        color="black", alpha=0.35, linewidth=args.line_width + 2,
    )
    ax.plot(
        shadow_base[:, 0], shadow_base[:, 1], shadow_base[:, 2],
        color="#f2f2f2", alpha=0.40, linewidth=0.8,
    )

    imax = int(np.argmax(err))
    ax.scatter([top[imax, 0]], [top[imax, 1]], [top[imax, 2]], s=58, c="white",
               edgecolors="#ff2d2d", linewidths=1.5, depthshade=False)
    ax.text(top[imax, 0], top[imax, 1], top[imax, 2] + max(0.5, raw_stats["max"] * 0.05),
            f"max {err[imax]:.1f} m", color="white", fontsize=9)

    all_x = np.concatenate([base[:, 0], map_xyz[:, 0] if len(map_xyz) else base[:, 0]])
    all_y = np.concatenate([base[:, 1], map_xyz[:, 1] if len(map_xyz) else base[:, 1]])
    all_z = np.concatenate([
        shadow_base[:, 2],
        top[:, 2],
        map_xyz[:, 2] if len(map_xyz) else shadow_base[:, 2],
    ])
    pad_xy = 30.0
    ax.set_xlim(all_x.min() - pad_xy, all_x.max() + pad_xy)
    ax.set_ylim(all_y.min() - pad_xy, all_y.max() + pad_xy)
    ax.set_zlim(all_z.min() - 2.0, all_z.max() + max(3.0, raw_stats["max"] * args.z_scale * 0.15))
    set_axes_equal_xy(ax, all_x, all_y, all_z)

    ax.view_init(elev=args.elev, azim=args.azim)
    ax.set_xlabel("map X [m]", color="#dddddd", labelpad=10)
    ax.set_ylabel("map Y [m]", color="#dddddd", labelpad=10)
    ax.set_zlabel("", color="#dddddd")
    ax.tick_params(colors="#cfcfcf", labelsize=8)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis._axinfo["grid"]["color"] = (1, 1, 1, 0.12)
        axis._axinfo["axisline"]["color"] = (1, 1, 1, 0.25)
    ax.xaxis.pane.set_facecolor((0.06, 0.07, 0.09, 0.70))
    ax.yaxis.pane.set_facecolor((0.06, 0.07, 0.09, 0.70))
    ax.zaxis.pane.set_facecolor((0.06, 0.07, 0.09, 0.70))

    title = args.title or Path(args.csv).parent.name
    ax.set_title(
        f"{title}\n"
        f"{args.error_field}: median {raw_stats['median']:.2f} m, "
        f"p95 {raw_stats['p95']:.2f} m, rms {raw_stats['rms']:.2f} m, max {raw_stats['max']:.2f} m",
        color="white", pad=18, fontsize=13,
    )
    mappable = plt.cm.ScalarMappable(norm=norm, cmap=cmap)
    cbar = fig.colorbar(mappable, ax=ax, shrink=0.58, pad=0.10)
    cbar.set_label(f"{args.error_field} GICP - GNSS error [m]", color="#dddddd")
    cbar.ax.yaxis.set_tick_params(color="#dddddd")
    plt.setp(cbar.ax.get_yticklabels(), color="#dddddd")

    height_note = f"Height = map z"
    if abs(args.baseline_z) > 1e-9:
        height_note += f" {args.baseline_z:+g} m"
    height_note += f" + {args.error_field} error"
    if abs(args.z_scale - 1.0) > 1e-9:
        height_note += f" x {args.z_scale:g}"
    if args.color_max <= 0:
        height_note += f"; color capped at p{args.color_percentile:g} = {color_max:.2f} m"
    fig.text(0.055, 0.045, height_note, color="#d8d8d8", fontsize=9)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, facecolor=fig.get_facecolor(), bbox_inches="tight", pad_inches=0.18)
    plt.close(fig)
    print(f"[error_rollercoaster] wrote {out}")
    print(
        "[error_rollercoaster] stats "
        f"n={len(raw_error)} rms={raw_stats['rms']:.3f} "
        f"median={raw_stats['median']:.3f} p95={raw_stats['p95']:.3f} "
        f"max={raw_stats['max']:.3f}"
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", required=True, help="live_error.csv from run_localization_replay.sh")
    ap.add_argument("--map", default="", help="optional PCD map to draw as the physical backdrop")
    ap.add_argument("--out", required=True, help="output PNG path")
    ap.add_argument("--position-source", choices=("gnss", "gicp"), default="gnss")
    ap.add_argument("--error-field", choices=("e2d", "e3d"), default="e2d")
    ap.add_argument("--samples", type=int, default=3200, help="distance-uniform trajectory samples to render")
    ap.add_argument("--z-scale", type=float, default=10.0, help="visual height multiplier for error meters")
    ap.add_argument(
        "--baseline-z",
        type=float,
        default=0.0,
        help="vertical offset added to the physical map z baseline before drawing error height",
    )
    ap.add_argument("--map-padding", type=float, default=80.0)
    ap.add_argument("--map-max-points", type=int, default=180000)
    ap.add_argument("--map-point-size", type=float, default=0.18)
    ap.add_argument("--map-alpha", type=float, default=0.18)
    ap.add_argument("--curtain-alpha", type=float, default=0.70, help="vertical bar alpha")
    ap.add_argument("--line-width", type=float, default=2.2)
    ap.add_argument("--cmap", default="inferno")
    ap.add_argument("--color-percentile", type=float, default=98.0)
    ap.add_argument("--color-max", type=float, default=0.0, help="override colorbar max; 0 uses percentile")
    ap.add_argument("--width", type=int, default=2200)
    ap.add_argument("--height", type=int, default=1500)
    ap.add_argument("--dpi", type=int, default=160)
    ap.add_argument("--elev", type=float, default=34.0)
    ap.add_argument("--azim", type=float, default=-58.0)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--title", default="")
    args = ap.parse_args()
    render(args)


if __name__ == "__main__":
    main()
