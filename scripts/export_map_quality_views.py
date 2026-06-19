#!/usr/bin/env python3
"""Export map-quality views for one GLIM mapping run.

Inputs:
  - a PCD map exported by glim_dump_to_pcd
  - a GLIM dump directory containing traj_imu.txt and T_world_utm.txt
  - a prepped rosbag containing /gps_p1/filtered_odom_rtk_fixed

Outputs:
  - traj_vs_rtk.csv: GLIM trajectory samples, interpolated RTK samples, errors
  - summary.txt: compact error statistics
  - top_glim_rtk.png: top-down map view with GLIM and RTK trajectories
  - interactive_error_3d/: draggable Three.js viewer with vertical error cylinders

The cylinder convention matches the existing RViz/live-error visualization:
the cylinder base is on the reference trajectory and height encodes error.
"""

import argparse
import csv
import json
import os
import struct
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import colormaps
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
import rosbag2_py

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_map import clean, colorize, load_pcd  # noqa: E402


HTML = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8"/>
<title>DLIO++ map quality viewer</title>
<style>
  body { margin:0; overflow:hidden; background:#090b10; font-family:Arial, sans-serif; }
  #hud { position:fixed; top:10px; left:10px; color:#d8dde8; font-size:13px;
         background:rgba(0,0,0,.58); padding:10px 12px; border-radius:6px; line-height:1.45; }
  #hud input { vertical-align:middle; }
  .sw { display:inline-block; width:12px; height:3px; margin:0 5px 3px 0; vertical-align:middle; }
  button { margin-left:8px; }
</style>
<script type="importmap">
{ "imports": {
    "three": "https://unpkg.com/three@0.160.0/build/three.module.js",
    "three/addons/": "https://unpkg.com/three@0.160.0/examples/jsm/"
} }
</script>
</head>
<body>
<div id="hud">
  <b id="title">DLIO++ map quality</b><br/>
  drag: rotate | wheel: zoom | right-drag: pan
  <button id="home">home</button><button id="top">top</button><br/>
  point size <input id="psize" type="range" min="0.4" max="4" step="0.1" value="1.3"/>
  cylinders <input id="cvis" type="checkbox" checked/><br/>
  <span class="sw" style="background:#ff8a1c"></span>GLIM trajectory
  <span class="sw" style="background:#32d7ff"></span>RTK trajectory
  <span id="stats"></span>
</div>
<script type="module">
const statsEl = () => document.getElementById('stats');
addEventListener('error', e => { statsEl().textContent = ' | ERROR: ' + e.message; });
addEventListener('unhandledrejection', e => { statsEl().textContent = ' | ERROR: ' + e.reason; });

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setSize(innerWidth, innerHeight);
renderer.setPixelRatio(devicePixelRatio);
document.body.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x090b10);
const camera = new THREE.PerspectiveCamera(55, innerWidth / innerHeight, 0.5, 60000);
camera.up.set(0, 0, 1);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

const meta = await (await fetch('meta.json')).json();
document.getElementById('title').textContent = meta.title;

statsEl().textContent = ' | loading map ...';
const buf = await (await fetch('points.bin')).arrayBuffer();
const dv = new DataView(buf);
let off = 0;
const n = dv.getUint32(off, true); off += 4;
const pos = new Float32Array(buf, off, n * 3); off += n * 12;
const col = new Uint8Array(buf, off, n * 3);

const geo = new THREE.BufferGeometry();
geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
geo.setAttribute('color', new THREE.BufferAttribute(col, 3, true));
const pmat = new THREE.PointsMaterial({size:1.3, vertexColors:true, sizeAttenuation:false});
scene.add(new THREE.Points(geo, pmat));

function makeLine(points, color, zLift) {
  if (!points || points.length < 6) return null;
  const arr = new Float32Array(points.length);
  for (let i = 0; i < points.length; i += 3) {
    arr[i] = points[i];
    arr[i + 1] = points[i + 1];
    arr[i + 2] = points[i + 2] + zLift;
  }
  const lineGeo = new THREE.BufferGeometry();
  lineGeo.setAttribute('position', new THREE.BufferAttribute(arr, 3));
  const line = new THREE.Line(lineGeo, new THREE.LineBasicMaterial({color}));
  scene.add(line);
  return line;
}
makeLine(meta.glim_line, 0xff8a1c, 2.0);
makeLine(meta.rtk_line, 0x32d7ff, 2.8);

let cylMesh = null;
if (meta.cylinders && meta.cylinders.count > 0) {
  const geom = new THREE.CylinderGeometry(0.5, 0.5, 1.0, 16, 1, true);
  geom.rotateX(Math.PI / 2.0);
  const mat = new THREE.MeshBasicMaterial({vertexColors:true, transparent:true, opacity:0.82});
  cylMesh = new THREE.InstancedMesh(geom, mat, meta.cylinders.count);
  cylMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
  const dummy = new THREE.Object3D();
  const centers = meta.cylinders.centers;
  const heights = meta.cylinders.heights;
  const colors = meta.cylinders.colors;
  const diameter = meta.cylinders.diameter;
  for (let i = 0; i < meta.cylinders.count; i++) {
    dummy.position.set(centers[i * 3], centers[i * 3 + 1], centers[i * 3 + 2]);
    dummy.scale.set(diameter, diameter, heights[i]);
    dummy.updateMatrix();
    cylMesh.setMatrixAt(i, dummy.matrix);
    cylMesh.setColorAt(i, new THREE.Color(colors[i * 3], colors[i * 3 + 1], colors[i * 3 + 2]));
  }
  scene.add(cylMesh);
}

geo.computeBoundingBox();
const bb = geo.boundingBox;
const center = new THREE.Vector3();
bb.getCenter(center);
const ext = bb.getSize(new THREE.Vector3()).length();
function home() {
  camera.position.set(center.x - 0.42 * ext, center.y - 0.50 * ext, center.z + 0.35 * ext);
  controls.target.copy(center);
  controls.update();
}
function top() {
  camera.position.set(center.x, center.y + 1, center.z + 0.92 * ext);
  controls.target.copy(center);
  controls.update();
}
home();

document.getElementById('psize').oninput = e => { pmat.size = parseFloat(e.target.value); };
document.getElementById('home').onclick = home;
document.getElementById('top').onclick = top;
document.getElementById('cvis').onchange = e => { if (cylMesh) cylMesh.visible = e.target.checked; };
statsEl().textContent =
  ` | ${n.toLocaleString()} pts | ` +
  `error median ${meta.stats.median.toFixed(3)} m, p95 ${meta.stats.p95.toFixed(3)} m, ` +
  `max ${meta.stats.max.toFixed(3)} m`;

addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});
renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });
</script>
</body>
</html>
"""


def load_tum(path):
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data[None, :]
    return data


def load_t_world_utm(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "T_world_utm" in line:
                continue
            vals = line.split()
            if len(vals) == 4:
                rows.append([float(v) for v in vals])
    if len(rows) != 4:
        raise RuntimeError(f"malformed {path}")
    return np.asarray(rows, dtype=np.float64)


def read_odom(bag_path, topic, storage_id):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    rows = []
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(data, Odometry)
        rows.append((
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ))
    if not rows:
        sys.exit(f"no messages on {topic} in {bag_path}")
    return np.asarray(rows, dtype=np.float64)


def interpolate_track(query_t, ref):
    return np.column_stack([
        np.interp(query_t, ref[:, 0], ref[:, i]) for i in range(1, 4)
    ])


def compare_traj_to_rtk(dump, bag, topic, storage_id, max_gap):
    traj = load_tum(Path(dump) / "traj_imu.txt")
    T = load_t_world_utm(Path(dump) / "T_world_utm.txt")
    odom = read_odom(bag, topic, storage_id)
    R, t = T[:3, :3], T[:3, 3]
    rtk_map = (R @ odom[:, 1:4].T).T + t
    rtk = np.column_stack([odom[:, 0], rtk_map])

    t0, t1 = rtk[0, 0], rtk[-1, 0]
    sel = (traj[:, 0] >= t0) & (traj[:, 0] <= t1)
    if sel.sum() < 10:
        sys.exit("trajectory and RTK stamps barely overlap")
    ts = traj[sel, 0]
    glim = traj[sel, 1:4]
    rtk_interp = interpolate_track(ts, rtk)

    idx = np.searchsorted(rtk[:, 0], ts).clip(1, len(rtk) - 1)
    gap = rtk[idx, 0] - rtk[idx - 1, 0]
    ok = gap <= max_gap
    ts, glim, rtk_interp = ts[ok], glim[ok], rtk_interp[ok]
    if len(ts) < 10:
        sys.exit("too few samples after RTK gap filtering")

    err = glim - rtk_interp
    e2d = np.linalg.norm(err[:, :2], axis=1)
    e3d = np.linalg.norm(err, axis=1)
    rows = np.column_stack([ts, glim, rtk_interp, err, e2d, e3d])
    return rows


def stats(values):
    return {
        "rms": float(np.sqrt(np.mean(values ** 2))),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def distance_resample_xyz(xyz, values, max_samples):
    if len(xyz) <= max_samples:
        return xyz, values
    step = np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(step)])
    keep = np.concatenate([[True], np.diff(s) > 1e-5])
    s, xyz, values = s[keep], xyz[keep], values[keep] if values is not None else None
    target = np.linspace(0.0, s[-1], max_samples)
    out = np.column_stack([
        np.interp(target, s, xyz[:, 0]),
        np.interp(target, s, xyz[:, 1]),
        np.interp(target, s, xyz[:, 2]),
    ])
    out_values = None
    if values is not None:
        out_values = np.interp(target, s, values)
    return out, out_values


def voxel_downsample(xyz, colors, max_points):
    if len(xyz) <= max_points:
        return xyz, colors
    res = 0.2
    idx = None
    for _ in range(14):
        key = np.floor(xyz / res).astype(np.int64)
        key = (key[:, 0] * 73856093) ^ (key[:, 1] * 19349663) ^ (key[:, 2] * 83492791)
        _, idx = np.unique(key, return_index=True)
        if len(idx) <= max_points:
            print(f"[map_quality] voxel {res:.2f} m -> {len(idx):,} points")
            return xyz[idx], colors[idx]
        res *= 1.35
    return xyz[idx], colors[idx]


def inferno_colors(errors, color_max):
    cmap = colormaps["inferno"]
    level = np.clip(errors / max(color_max, 1e-6), 0.0, 1.0)
    rgba = cmap(level)[:, :3]
    saturation = 0.18 + 0.82 * np.sqrt(level)[:, None]
    base = np.array([0.22, 0.22, 0.24])
    return base * (1.0 - saturation) + rgba * saturation


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    header = [
        "stamp",
        "glim_x", "glim_y", "glim_z",
        "rtk_x", "rtk_y", "rtk_z",
        "err_x", "err_y", "err_z",
        "e2d", "e3d",
    ]
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def render_top_view(out_path, xyz, colors, rows, title, max_points):
    if len(xyz) > max_points:
        rng = np.random.default_rng(7)
        idx = rng.choice(len(xyz), size=max_points, replace=False)
        xyz_plot, colors_plot = xyz[idx], colors[idx]
    else:
        xyz_plot, colors_plot = xyz, colors

    glim = rows[:, 1:4]
    rtk = rows[:, 4:7]
    e2d = rows[:, 10]
    st = stats(e2d)

    fig, ax = plt.subplots(figsize=(16, 16), dpi=180)
    fig.patch.set_facecolor("#111318")
    ax.set_facecolor("#111318")
    ax.scatter(
        xyz_plot[:, 0], xyz_plot[:, 1],
        c=np.clip(colors_plot, 0, 1),
        s=0.08,
        alpha=0.55,
        linewidths=0,
        rasterized=True,
    )
    ax.plot(rtk[:, 0], rtk[:, 1], color="#32d7ff", linewidth=1.9, label="RTK trajectory")
    ax.plot(glim[:, 0], glim[:, 1], color="#ff8a1c", linewidth=1.4, label="GLIM trajectory")
    ax.scatter([rtk[0, 0]], [rtk[0, 1]], c="#5cff9a", s=50, label="start", zorder=5)
    ax.scatter([rtk[-1, 0]], [rtk[-1, 1]], c="#ff4f5e", s=50, label="end", zorder=5)
    imax = int(np.argmax(e2d))
    ax.scatter([rtk[imax, 0]], [rtk[imax, 1]], c="white", edgecolors="#ff2d2d",
               s=70, zorder=6, label=f"max error {e2d[imax]:.2f} m")
    ax.set_aspect("equal", adjustable="box")
    pad = 35.0
    all_xy = np.vstack([xyz_plot[:, :2], glim[:, :2], rtk[:, :2]])
    ax.set_xlim(all_xy[:, 0].min() - pad, all_xy[:, 0].max() + pad)
    ax.set_ylim(all_xy[:, 1].min() - pad, all_xy[:, 1].max() + pad)
    ax.grid(color="white", alpha=0.10, linewidth=0.6)
    ax.tick_params(colors="#d8dde8", labelsize=8)
    ax.set_xlabel("map X [m]", color="#d8dde8")
    ax.set_ylabel("map Y [m]", color="#d8dde8")
    ax.set_title(
        f"{title} top view: GLIM vs RTK\n"
        f"horizontal error median {st['median']:.3f} m, p95 {st['p95']:.3f} m, "
        f"rms {st['rms']:.3f} m, max {st['max']:.3f} m",
        color="white",
        pad=14,
    )
    leg = ax.legend(loc="upper right", facecolor="#0b0d12", edgecolor="#3a3f4a", framealpha=0.92)
    for text in leg.get_texts():
        text.set_color("#e6e8ee")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, facecolor=fig.get_facecolor(), bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def write_interactive(out_dir, xyz, colors, rows, title, max_points, line_points,
                      cylinder_count, cylinder_z_scale, cylinder_diameter,
                      cylinder_baseline_z, color_percentile):
    out_dir.mkdir(parents=True, exist_ok=True)
    xyz_ds, colors_ds = voxel_downsample(xyz, colors, max_points)
    center = (xyz_ds.min(axis=0) + xyz_ds.max(axis=0)) * 0.5
    pos = (xyz_ds - center).astype(np.float32)
    col = (np.clip(colors_ds, 0, 1) * 255).astype(np.uint8)
    with open(out_dir / "points.bin", "wb") as f:
        f.write(struct.pack("<I", len(pos)))
        f.write(pos.tobytes())
        f.write(col.tobytes())

    glim_line, _ = distance_resample_xyz(rows[:, 1:4], None, line_points)
    rtk_line, _ = distance_resample_xyz(rows[:, 4:7], None, line_points)
    cyl_xyz, cyl_err = distance_resample_xyz(rows[:, 4:7], rows[:, 10], cylinder_count)
    color_max = float(np.percentile(rows[:, 10], color_percentile))
    cyl_height = np.maximum(cyl_err * cylinder_z_scale, 0.05)
    cyl_center = cyl_xyz.copy()
    cyl_center[:, 2] = cyl_xyz[:, 2] + cylinder_baseline_z + cyl_height * 0.5
    cyl_color = inferno_colors(cyl_err, color_max)

    e2d_stats = stats(rows[:, 10])
    e3d_stats = stats(rows[:, 11])
    meta = {
        "title": title,
        "center": center.tolist(),
        "stats": e2d_stats,
        "stats_3d": e3d_stats,
        "glim_line": (glim_line - center).astype(np.float32).reshape(-1).round(4).tolist(),
        "rtk_line": (rtk_line - center).astype(np.float32).reshape(-1).round(4).tolist(),
        "cylinders": {
            "count": int(len(cyl_center)),
            "diameter": float(cylinder_diameter),
            "z_scale": float(cylinder_z_scale),
            "color_max": color_max,
            "centers": (cyl_center - center).astype(np.float32).reshape(-1).round(4).tolist(),
            "heights": cyl_height.astype(np.float32).round(4).tolist(),
            "errors": cyl_err.astype(np.float32).round(4).tolist(),
            "colors": cyl_color.astype(np.float32).reshape(-1).round(4).tolist(),
        },
    }
    with open(out_dir / "meta.json", "w") as f:
        json.dump(meta, f)
    with open(out_dir / "index.html", "w") as f:
        f.write(HTML)


def write_summary(path, run_name, rows):
    e2d_stats = stats(rows[:, 10])
    e3d_stats = stats(rows[:, 11])
    z = rows[:, 9]
    text = (
        f"run: {run_name}\n"
        f"samples: {len(rows)}\n"
        f"window: {rows[0, 0]:.3f} .. {rows[-1, 0]:.3f} ({rows[-1, 0] - rows[0, 0]:.3f} s)\n"
        f"horizontal_error_m: rms={e2d_stats['rms']:.3f} median={e2d_stats['median']:.3f} "
        f"p95={e2d_stats['p95']:.3f} max={e2d_stats['max']:.3f}\n"
        f"3d_error_m: rms={e3d_stats['rms']:.3f} median={e3d_stats['median']:.3f} "
        f"p95={e3d_stats['p95']:.3f} max={e3d_stats['max']:.3f}\n"
        f"z_error_m: rms={np.sqrt(np.mean(z ** 2)):.3f} mean={np.mean(z):.3f}\n"
    )
    path.write_text(text)
    print(text.strip())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run-name", required=True)
    ap.add_argument("--map", required=True)
    ap.add_argument("--dump", required=True)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--rtk-topic", default="/gps_p1/filtered_odom_rtk_fixed")
    ap.add_argument("--storage-id", default="mcap")
    ap.add_argument("--max-gap", type=float, default=0.5)
    ap.add_argument("--interactive-max-points", type=int, default=2_000_000)
    ap.add_argument("--top-max-points", type=int, default=1_500_000)
    ap.add_argument("--line-points", type=int, default=9000)
    ap.add_argument("--cylinder-count", type=int, default=3500)
    ap.add_argument("--cylinder-z-scale", type=float, default=8.0)
    ap.add_argument("--cylinder-diameter", type=float, default=1.5)
    ap.add_argument("--cylinder-baseline-z", type=float, default=2.0)
    ap.add_argument("--color-percentile", type=float, default=98.0)
    ap.add_argument("--cmap", default="turbo")
    ap.add_argument("--no-clean", action="store_true")
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    print("[map_quality] comparing GLIM trajectory to RTK ...")
    rows = compare_traj_to_rtk(args.dump, args.bag, args.rtk_topic, args.storage_id, args.max_gap)
    write_csv(out / "traj_vs_rtk.csv", rows)
    write_summary(out / "summary.txt", args.run_name, rows)

    print(f"[map_quality] loading map {args.map} ...")
    xyz, inten = load_pcd(args.map)
    print(f"[map_quality] {len(xyz):,} map points")
    if not args.no_clean:
        xyz, inten = clean(xyz, inten)
        print(f"[map_quality] {len(xyz):,} map points after speckle removal")
    colors = colorize(xyz, inten, args.cmap)

    print("[map_quality] writing top-down PNG ...")
    render_top_view(out / "top_glim_rtk.png", xyz, colors, rows, args.run_name, args.top_max_points)

    print("[map_quality] writing interactive 3D viewer ...")
    write_interactive(
        out / "interactive_error_3d",
        xyz,
        colors,
        rows,
        f"{args.run_name} GLIM vs RTK error",
        args.interactive_max_points,
        args.line_points,
        args.cylinder_count,
        args.cylinder_z_scale,
        args.cylinder_diameter,
        args.cylinder_baseline_z,
        args.color_percentile,
    )
    print(f"[map_quality] wrote {out}")


if __name__ == "__main__":
    main()
