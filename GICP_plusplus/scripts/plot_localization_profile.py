#!/usr/bin/env python3
"""
Plot CSV outputs from profile_localization_resources.py.

The script accepts one profiler output directory, reads the CSV files created by
the profiler, and writes PNG time-series plots under a plots/ directory.
"""

import argparse
import csv
import math
import os
import sys
import warnings
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


os.environ.setdefault("MPLCONFIGDIR", str(Path(os.environ.get("TMPDIR", "/tmp")) / "gicp_profile_matplotlib"))

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError as exc:
    print(
        "matplotlib is required for plotting. Install it with: "
        "sudo apt install python3-matplotlib",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc


TIME_COLUMNS = ("elapsed_s", "ros_time_s", "scan_stamp_s", "pose_stamp_s")
NON_METRIC_COLUMNS = {
    "sample_index",
    "wall_time_iso",
    "elapsed_s",
    "ros_time_s",
    "scan_stamp_s",
    "pose_stamp_s",
    "role",
    "pid",
    "command",
    "core",
    "lap_number",
}
SUMMARY_STAT_COLUMNS = ("min", "mean", "p50", "p95", "p99", "max")


def finite(value: Optional[float]) -> bool:
    return value is not None and math.isfinite(value)


def parse_number(value: str) -> Optional[float]:
    text = value.strip()
    if text == "":
        return None
    lowered = text.lower()
    if lowered == "true":
        return 1.0
    if lowered == "false":
        return 0.0
    try:
        numeric = float(text)
    except ValueError:
        return None
    if not math.isfinite(numeric):
        return None
    return numeric


def parse_wall_time(value: str) -> Optional[float]:
    text = value.strip()
    if not text:
        return None
    try:
        return datetime.fromisoformat(text).timestamp()
    except ValueError:
        return None


def read_csv(path: Path) -> Tuple[List[str], List[Dict[str, str]]]:
    if not path.exists():
        return [], []
    with path.open("r", newline="", encoding="utf-8", errors="replace") as handle:
        reader = csv.DictReader(handle)
        return list(reader.fieldnames or []), list(reader)


def numeric_values(rows: Sequence[Dict[str, str]], column: str) -> List[Optional[float]]:
    return [parse_number(row.get(column, "")) for row in rows]


def has_numeric_data(rows: Sequence[Dict[str, str]], column: str) -> bool:
    return any(finite(value) for value in numeric_values(rows, column))


def numeric_columns(rows: Sequence[Dict[str, str]], columns: Sequence[str]) -> List[str]:
    result = []
    for column in columns:
        if column in NON_METRIC_COLUMNS:
            continue
        if has_numeric_data(rows, column):
            result.append(column)
    return result


def choose_time(rows: Sequence[Dict[str, str]], preferred: str = "elapsed_s") -> Tuple[str, List[Optional[float]]]:
    candidates = [preferred]
    candidates.extend(column for column in TIME_COLUMNS if column != preferred)
    for column in candidates:
        values = numeric_values(rows, column)
        if any(finite(value) for value in values):
            return column, values

    wall_times = [parse_wall_time(row.get("wall_time_iso", "")) for row in rows]
    finite_wall_times = [value for value in wall_times if finite(value)]
    if finite_wall_times:
        start = finite_wall_times[0]
        return "wall_elapsed_s", [
            None if value is None else value - start for value in wall_times
        ]

    return "row_index", [float(index) for index, _row in enumerate(rows)]


def compact_label(text: str, max_len: int = 48) -> str:
    if len(text) <= max_len:
        return text
    return text[: max_len - 3] + "..."


def safe_name(name: str) -> str:
    output = []
    for char in name.lower():
        if char.isalnum():
            output.append(char)
        elif char in ("_", "-", "."):
            output.append(char)
        else:
            output.append("_")
    return "".join(output).strip("_") or "plot"


def chunks(items: Sequence[str], size: int) -> Iterable[Sequence[str]]:
    for index in range(0, len(items), size):
        yield items[index : index + size]


def ensure_output_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def save_figure(fig, output_path: Path, dpi: int) -> None:
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        fig.tight_layout()
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)


def add_lap_markers(ax, lap_rows: Sequence[Dict[str, str]], label_first: bool = True) -> None:
    if not lap_rows:
        return
    _time_name, lap_times = choose_time(lap_rows)
    for index, (row, lap_time) in enumerate(zip(lap_rows, lap_times)):
        if not finite(lap_time):
            continue
        label = "lap" if label_first and index == 0 else None
        ax.axvline(lap_time, color="black", linewidth=0.8, alpha=0.25, linestyle="--", label=label)
        lap_number = row.get("lap_number", "")
        if lap_number:
            top = ax.get_ylim()[1]
            ax.text(lap_time, top, f" L{lap_number}", rotation=90, va="top", ha="left", fontsize=7)


def plot_series(
    ax,
    x_values: Sequence[Optional[float]],
    y_values: Sequence[Optional[float]],
    label: Optional[str] = None,
    marker: Optional[str] = None,
) -> bool:
    xs = []
    ys = []
    for x_value, y_value in zip(x_values, y_values):
        if finite(x_value) and finite(y_value):
            xs.append(float(x_value))
            ys.append(float(y_value))
    if not xs:
        return False
    ax.plot(xs, ys, linewidth=1.2, marker=marker, markersize=2.5 if marker else 0, label=label)
    return True


def plot_generic_csv(
    csv_path: Path,
    output_dir: Path,
    lap_rows: Sequence[Dict[str, str]],
    time_preference: str,
    dpi: int,
    max_subplots: int,
) -> List[Path]:
    columns, rows = read_csv(csv_path)
    if not rows:
        return []
    metrics = numeric_columns(rows, columns)
    if not metrics:
        return []

    group_column = None
    if csv_path.stem == "cpu_cores":
        group_column = "core"
    elif csv_path.stem == "process_resources":
        group_column = "role"

    written: List[Path] = []
    for chunk_index, metric_group in enumerate(chunks(metrics, max_subplots), start=1):
        fig, axes = plt.subplots(len(metric_group), 1, figsize=(14, max(3.0, 2.6 * len(metric_group))), sharex=True)
        if len(metric_group) == 1:
            axes = [axes]
        for ax, metric in zip(axes, metric_group):
            if group_column and group_column in columns:
                labels = sorted({row.get(group_column, "") or "unknown" for row in rows})
                for label in labels:
                    grouped_rows = [row for row in rows if (row.get(group_column, "") or "unknown") == label]
                    time_name, x_values = choose_time(grouped_rows, time_preference)
                    y_values = numeric_values(grouped_rows, metric)
                    plot_series(ax, x_values, y_values, compact_label(label))
            else:
                time_name, x_values = choose_time(rows, time_preference)
                y_values = numeric_values(rows, metric)
                plot_series(ax, x_values, y_values)

            add_lap_markers(ax, lap_rows, label_first=False)
            ax.set_ylabel(metric)
            ax.grid(True, alpha=0.25)
            if group_column:
                ax.legend(loc="best", fontsize=8)
        axes[-1].set_xlabel(time_name)
        fig.suptitle(f"{csv_path.name}: all numeric metrics vs time", y=1.0)
        suffix = "" if len(metrics) <= max_subplots else f"_{chunk_index:02d}"
        output_path = output_dir / f"{csv_path.stem}_all_metrics{suffix}.png"
        save_figure(fig, output_path, dpi)
        written.append(output_path)
    return written


def plot_dashboard(
    rows: Sequence[Dict[str, str]],
    metric_groups: Sequence[Tuple[str, Sequence[str]]],
    title: str,
    output_path: Path,
    lap_rows: Sequence[Dict[str, str]],
    time_preference: str,
    dpi: int,
) -> Optional[Path]:
    if not rows:
        return None
    active_groups = []
    for group_title, metrics in metric_groups:
        present = [metric for metric in metrics if has_numeric_data(rows, metric)]
        if present:
            active_groups.append((group_title, present))
    if not active_groups:
        return None

    fig, axes = plt.subplots(len(active_groups), 1, figsize=(14, max(4, 2.9 * len(active_groups))), sharex=True)
    if len(active_groups) == 1:
        axes = [axes]
    time_name, x_values = choose_time(rows, time_preference)
    for ax, (group_title, metrics) in zip(axes, active_groups):
        for metric in metrics:
            plot_series(ax, x_values, numeric_values(rows, metric), metric)
        add_lap_markers(ax, lap_rows, label_first=False)
        ax.set_ylabel(group_title)
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best", fontsize=8)
    axes[-1].set_xlabel(time_name)
    fig.suptitle(title, y=1.0)
    save_figure(fig, output_path, dpi)
    return output_path


def plot_process_dashboard(
    rows: Sequence[Dict[str, str]],
    output_path: Path,
    lap_rows: Sequence[Dict[str, str]],
    time_preference: str,
    dpi: int,
) -> Optional[Path]:
    if not rows:
        return None
    metrics = [
        ("CPU %", "cpu_percent"),
        ("Host CPU %", "cpu_percent_host"),
        ("RSS MB", "rss_mb"),
        ("PSS MB", "pss_mb"),
        ("Read MB/s", "read_mbps"),
        ("Write MB/s", "write_mbps"),
        ("Threads", "threads"),
    ]
    active_metrics = [(title, metric) for title, metric in metrics if has_numeric_data(rows, metric)]
    if not active_metrics:
        return None

    labels = sorted({row.get("role", "") or f"pid_{row.get('pid', 'unknown')}" for row in rows})
    fig, axes = plt.subplots(len(active_metrics), 1, figsize=(14, max(4, 2.8 * len(active_metrics))), sharex=True)
    if len(active_metrics) == 1:
        axes = [axes]
    for ax, (title, metric) in zip(axes, active_metrics):
        for label in labels:
            grouped_rows = [row for row in rows if (row.get("role", "") or f"pid_{row.get('pid', 'unknown')}") == label]
            time_name, x_values = choose_time(grouped_rows, time_preference)
            plot_series(ax, x_values, numeric_values(grouped_rows, metric), compact_label(label))
        add_lap_markers(ax, lap_rows, label_first=False)
        ax.set_ylabel(title)
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best", fontsize=8)
    axes[-1].set_xlabel(time_name)
    fig.suptitle("Process Resources", y=1.0)
    save_figure(fig, output_path, dpi)
    return output_path


def plot_cpu_heatmap(
    rows: Sequence[Dict[str, str]],
    output_path: Path,
    time_preference: str,
    dpi: int,
) -> Optional[Path]:
    if not rows or not has_numeric_data(rows, "cpu_percent"):
        return None
    sample_keys = []
    samples_seen = set()
    for row in rows:
        key = row.get("sample_index", "")
        if key and key not in samples_seen:
            samples_seen.add(key)
            sample_keys.append(key)

    cores = sorted({row.get("core", "") for row in rows if row.get("core", "")}, key=lambda value: int(value[3:]) if value.startswith("cpu") and value[3:].isdigit() else value)
    if not sample_keys or not cores:
        return None

    by_key_core: Dict[Tuple[str, str], Optional[float]] = {}
    time_by_key: Dict[str, Optional[float]] = {}
    for row in rows:
        sample_key = row.get("sample_index", "")
        core = row.get("core", "")
        if not sample_key or not core:
            continue
        by_key_core[(sample_key, core)] = parse_number(row.get("cpu_percent", ""))

    for sample_key in sample_keys:
        sample_rows = [row for row in rows if row.get("sample_index", "") == sample_key]
        _time_name, times = choose_time(sample_rows, time_preference)
        time_by_key[sample_key] = next((value for value in times if finite(value)), None)

    matrix = []
    for core in cores:
        core_values = []
        for sample_key in sample_keys:
            value = by_key_core.get((sample_key, core))
            core_values.append(float(value) if finite(value) else float("nan"))
        matrix.append(core_values)

    times = [time_by_key.get(sample_key) for sample_key in sample_keys]
    finite_times = [time for time in times if finite(time)]
    if finite_times:
        extent = [min(finite_times), max(finite_times), len(cores) - 0.5, -0.5]
        xlabel = "elapsed_s" if time_preference == "elapsed_s" else time_preference
    else:
        extent = [0, len(sample_keys) - 1, len(cores) - 0.5, -0.5]
        xlabel = "sample_index"

    fig, ax = plt.subplots(figsize=(14, max(5, 0.22 * len(cores))))
    image = ax.imshow(matrix, aspect="auto", interpolation="nearest", extent=extent, vmin=0.0, vmax=100.0)
    ax.set_title("Per-Core CPU Utilization")
    ax.set_xlabel(xlabel)
    ax.set_ylabel("CPU core")
    ax.set_yticks(range(len(cores)))
    ax.set_yticklabels(cores, fontsize=7)
    fig.colorbar(image, ax=ax, label="CPU %")
    save_figure(fig, output_path, dpi)
    return output_path


def plot_laps(
    rows: Sequence[Dict[str, str]],
    output_path: Path,
    time_preference: str,
    dpi: int,
) -> Optional[Path]:
    if not rows:
        return None
    time_name, x_values = choose_time(rows, time_preference)
    metrics = [metric for metric in ("distance_to_start_m", "path_length_m", "x", "y", "z") if has_numeric_data(rows, metric)]
    if not metrics:
        return None

    fig, axes = plt.subplots(len(metrics), 1, figsize=(14, max(4, 2.8 * len(metrics))), sharex=True)
    if len(metrics) == 1:
        axes = [axes]
    for ax, metric in zip(axes, metrics):
        plot_series(ax, x_values, numeric_values(rows, metric), marker="o")
        for row, x_value, y_value in zip(rows, x_values, numeric_values(rows, metric)):
            if finite(x_value) and finite(y_value):
                ax.text(float(x_value), float(y_value), f" L{row.get('lap_number', '')}", fontsize=8)
        ax.set_ylabel(metric)
        ax.grid(True, alpha=0.25)
    axes[-1].set_xlabel(time_name)
    fig.suptitle("Lap Events", y=1.0)
    save_figure(fig, output_path, dpi)
    return output_path


def plot_summary(summary_path: Path, output_path: Path, dpi: int, top_n: int) -> Optional[Path]:
    columns, rows = read_csv(summary_path)
    if not rows:
        return None
    if not all(column in columns for column in ("metric", "p95", "max")):
        return None

    scored = []
    for row in rows:
        max_value = parse_number(row.get("max", ""))
        if finite(max_value):
            scored.append((abs(float(max_value)), row))
    scored.sort(reverse=True, key=lambda item: item[0])
    selected = [row for _score, row in scored[:top_n]]
    if not selected:
        return None

    metrics = [compact_label(row.get("metric", ""), 34) for row in selected]
    y_positions = list(range(len(selected)))
    fig, ax = plt.subplots(figsize=(14, max(4, 0.38 * len(selected))))
    for stat in SUMMARY_STAT_COLUMNS:
        values = [parse_number(row.get(stat, "")) for row in selected]
        if any(finite(value) for value in values):
            ax.plot(
                [float(value) if finite(value) else float("nan") for value in values],
                y_positions,
                marker="o",
                linewidth=1.2,
                label=stat,
            )
    ax.set_yticks(y_positions)
    ax.set_yticklabels(metrics, fontsize=8)
    ax.invert_yaxis()
    ax.set_xlabel("metric value")
    ax.set_title(f"Summary Statistics: Top {len(selected)} Metrics by Absolute Max")
    ax.grid(True, axis="x", alpha=0.25)
    ax.legend(loc="best", fontsize=8)
    save_figure(fig, output_path, dpi)
    return output_path


def latest_profile_dir(root: Path) -> Path:
    if (root / "system_resources.csv").exists():
        return root
    candidates = [path for path in root.iterdir() if path.is_dir() and (path / "system_resources.csv").exists()]
    if not candidates:
        return root
    return max(candidates, key=lambda path: path.stat().st_mtime)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Plot CSV outputs produced by profile_localization_resources.py."
    )
    parser.add_argument(
        "profile_dir",
        type=Path,
        help="Profiler output directory, or a parent directory containing gicp_profile_* subdirectories.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory for generated plots. Default: PROFILE_DIR/plots.",
    )
    parser.add_argument(
        "--time-column",
        default="elapsed_s",
        choices=["elapsed_s", "ros_time_s", "scan_stamp_s", "pose_stamp_s"],
        help="Preferred x-axis time column when present.",
    )
    parser.add_argument("--dpi", type=int, default=150, help="Output image DPI.")
    parser.add_argument(
        "--max-subplots",
        type=int,
        default=8,
        help="Maximum subplots per generic all-metrics image.",
    )
    parser.add_argument(
        "--summary-top-n",
        type=int,
        default=40,
        help="Number of summary metrics to include in summary_statistics.png.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    profile_dir = latest_profile_dir(args.profile_dir.expanduser().resolve())
    if not profile_dir.exists():
        print(f"Profile directory does not exist: {profile_dir}", file=sys.stderr)
        return 1

    output_dir = args.output_dir.expanduser().resolve() if args.output_dir else profile_dir / "plots"
    ensure_output_dir(output_dir)

    _lap_columns, lap_rows = read_csv(profile_dir / "lap_events.csv")
    written: List[Path] = []

    system_columns, system_rows = read_csv(profile_dir / "system_resources.csv")
    process_columns, process_rows = read_csv(profile_dir / "process_resources.csv")
    cpu_columns, cpu_rows = read_csv(profile_dir / "cpu_cores.csv")
    scan_columns, scan_rows = read_csv(profile_dir / "ros_scan_metrics.csv")

    system_plot = plot_dashboard(
        system_rows,
        [
            ("CPU / load", ("cpu_percent", "load1", "load5", "load15")),
            ("Memory %", ("mem_used_percent", "swap_used_percent")),
            ("Memory MB", ("mem_used_mb", "mem_available_mb", "swap_used_mb")),
            ("Disk MB/s", ("disk_read_mbps", "disk_write_mbps")),
            ("Network MB/s", ("net_rx_mbps", "net_tx_mbps")),
            ("CPU freq / temp", ("cpu_freq_avg_mhz", "cpu_freq_max_mhz", "cpu_temp_c")),
        ],
        "System Resources",
        output_dir / "system_overview.png",
        lap_rows,
        args.time_column,
        args.dpi,
    )
    if system_plot:
        written.append(system_plot)

    process_plot = plot_process_dashboard(
        process_rows,
        output_dir / "process_overview.png",
        lap_rows,
        args.time_column,
        args.dpi,
    )
    if process_plot:
        written.append(process_plot)

    scan_plot = plot_dashboard(
        scan_rows,
        [
            ("GICP latency / dt", ("gicp_elapsed_ms", "scan_dt_s")),
            ("Fitness / error", ("fitness", "final_error", "hessian_condition_proxy")),
            ("Points", ("raw_points", "preprocessed_points", "num_correspondences")),
            ("Correspondence", ("correspondence_ratio", "converged")),
            ("Correction", ("guess_to_solution_trans_m", "guess_to_solution_rot_deg", "jump_trans_m", "jump_rot_deg")),
            ("IMU timing", ("imu_buffer_span_s", "scan_to_latest_imu_lag_s")),
            ("Final pose", ("final_x", "final_y", "final_z")),
        ],
        "ROS Scan Metrics",
        output_dir / "ros_scan_metrics_overview.png",
        lap_rows,
        args.time_column,
        args.dpi,
    )
    if scan_plot:
        written.append(scan_plot)

    cpu_heatmap = plot_cpu_heatmap(cpu_rows, output_dir / "cpu_cores_heatmap.png", args.time_column, args.dpi)
    if cpu_heatmap:
        written.append(cpu_heatmap)

    lap_plot = plot_laps(lap_rows, output_dir / "lap_events.png", args.time_column, args.dpi)
    if lap_plot:
        written.append(lap_plot)

    summary_plot = plot_summary(
        profile_dir / "summary.csv",
        output_dir / "summary_statistics.png",
        args.dpi,
        args.summary_top_n,
    )
    if summary_plot:
        written.append(summary_plot)

    for csv_path in sorted(profile_dir.glob("*.csv")):
        if csv_path.name == "summary.csv":
            continue
        written.extend(
            plot_generic_csv(
                csv_path,
                output_dir,
                lap_rows,
                args.time_column,
                args.dpi,
                args.max_subplots,
            )
        )

    if not written:
        print(f"No plots were generated from {profile_dir}", file=sys.stderr)
        return 1

    print(f"Read profiler CSVs from: {profile_dir}")
    print(f"Wrote {len(written)} plot files to: {output_dir}")
    for path in written:
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
