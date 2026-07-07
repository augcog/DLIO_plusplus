#!/usr/bin/env python3
"""
Attach-only resource profiler for the GICP localization replay workflow.

Run this as a third command while localization and rosbag playback are already
running. It samples Linux system/process counters, subscribes to localization
debug topics, writes CSV files continuously, and can stop automatically after a
configured number of laps.
"""

import argparse
import csv
import glob
import math
import os
import signal
import sys
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Set, Tuple


try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Bool, Float64
except ImportError:
    rclpy = None
    PoseStamped = None
    SingleThreadedExecutor = None
    Node = object
    Clock = None
    Bool = None
    Float64 = None


KIB = 1024.0
MIB = 1024.0 * 1024.0


def wall_iso(wall_time: float) -> str:
    return datetime.fromtimestamp(wall_time).isoformat(timespec="milliseconds")


def fmt_float(value: Optional[float], digits: int = 6) -> str:
    if value is None:
        return ""
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return ""
    if not math.isfinite(numeric):
        return ""
    return f"{numeric:.{digits}f}"


def fmt_int(value: Optional[int]) -> str:
    if value is None:
        return ""
    try:
        return str(int(value))
    except (TypeError, ValueError):
        return ""


def read_text(path: str) -> Optional[str]:
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as handle:
            return handle.read()
    except OSError:
        return None


def read_bytes(path: str) -> Optional[bytes]:
    try:
        with open(path, "rb") as handle:
            return handle.read()
    except OSError:
        return None


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def percentile(values: List[float], quantile: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    position = (len(ordered) - 1) * quantile
    low = int(math.floor(position))
    high = int(math.ceil(position))
    if low == high:
        return ordered[low]
    weight = position - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


class SummaryCollector:
    def __init__(self) -> None:
        self.values: Dict[str, List[float]] = defaultdict(list)

    def add(self, metric: str, value: Optional[float]) -> None:
        if value is None:
            return
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            return
        if math.isfinite(numeric):
            self.values[metric].append(numeric)

    def write_csv(self, path: Path) -> None:
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=["metric", "count", "min", "mean", "p50", "p95", "p99", "max"],
            )
            writer.writeheader()
            for metric in sorted(self.values):
                series = self.values[metric]
                if not series:
                    continue
                writer.writerow(
                    {
                        "metric": metric,
                        "count": len(series),
                        "min": fmt_float(min(series)),
                        "mean": fmt_float(sum(series) / len(series)),
                        "p50": fmt_float(percentile(series, 0.50)),
                        "p95": fmt_float(percentile(series, 0.95)),
                        "p99": fmt_float(percentile(series, 0.99)),
                        "max": fmt_float(max(series)),
                    }
                )


class CsvBundle:
    SCHEMAS: Dict[str, List[str]] = {
        "system_resources": [
            "sample_index",
            "wall_time_iso",
            "elapsed_s",
            "ros_time_s",
            "cpu_percent",
            "mem_total_mb",
            "mem_available_mb",
            "mem_used_mb",
            "mem_used_percent",
            "swap_total_mb",
            "swap_used_mb",
            "swap_used_percent",
            "load1",
            "load5",
            "load15",
            "disk_read_mbps",
            "disk_write_mbps",
            "net_rx_mbps",
            "net_tx_mbps",
            "cpu_freq_avg_mhz",
            "cpu_freq_max_mhz",
            "cpu_temp_c",
        ],
        "cpu_cores": [
            "sample_index",
            "wall_time_iso",
            "elapsed_s",
            "ros_time_s",
            "core",
            "cpu_percent",
        ],
        "process_resources": [
            "sample_index",
            "wall_time_iso",
            "elapsed_s",
            "ros_time_s",
            "role",
            "pid",
            "command",
            "cpu_percent",
            "cpu_percent_host",
            "rss_mb",
            "vms_mb",
            "pss_mb",
            "peak_rss_mb",
            "threads",
            "voluntary_ctxt_switches",
            "nonvoluntary_ctxt_switches",
            "read_bytes",
            "write_bytes",
            "read_mbps",
            "write_mbps",
        ],
        "ros_scan_metrics": [
            "wall_time_iso",
            "elapsed_s",
            "ros_time_s",
            "scan_stamp_s",
            "final_x",
            "final_y",
            "final_z",
            "gicp_elapsed_ms",
            "fitness",
            "converged",
            "scan_dt_s",
            "raw_points",
            "preprocessed_points",
            "num_correspondences",
            "correspondence_ratio",
            "final_error",
            "guess_to_solution_trans_m",
            "guess_to_solution_rot_deg",
            "guess_from_last_m",
            "guess_from_last_deg",
            "jump_trans_m",
            "jump_rot_deg",
            "imu_buffer_span_s",
            "scan_to_latest_imu_lag_s",
            "hessian_condition_proxy",
        ],
        "lap_events": [
            "wall_time_iso",
            "elapsed_s",
            "ros_time_s",
            "lap_number",
            "pose_stamp_s",
            "x",
            "y",
            "z",
            "distance_to_start_m",
            "path_length_m",
        ],
    }

    def __init__(self, output_dir: Path) -> None:
        self.output_dir = output_dir
        self.files = {}
        self.writers = {}
        for name, fields in self.SCHEMAS.items():
            handle = (output_dir / f"{name}.csv").open("w", newline="", encoding="utf-8")
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            handle.flush()
            self.files[name] = handle
            self.writers[name] = writer

    def write(self, name: str, row: Dict[str, object]) -> None:
        fields = self.SCHEMAS[name]
        self.writers[name].writerow({field: row.get(field, "") for field in fields})
        self.files[name].flush()

    def close(self) -> None:
        for handle in self.files.values():
            handle.flush()
            handle.close()


class LinuxResourceSampler:
    def __init__(self) -> None:
        self.sample_index = 0
        self.clock_ticks = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
        self.page_size = os.sysconf(os.sysconf_names["SC_PAGE_SIZE"])
        self.cpu_count = max(1, os.cpu_count() or 1)
        self.prev_cpu_times: Optional[Dict[str, Tuple[int, int]]] = None
        self.prev_disk: Optional[Tuple[float, Dict[str, int]]] = None
        self.prev_net: Optional[Tuple[float, Dict[str, int]]] = None
        self.prev_process: Dict[int, Dict[str, float]] = {}

    def sample(
        self,
        wall_time: float,
        start_wall_time: float,
        ros_time_s: Optional[float],
        csv_bundle: CsvBundle,
        summary: SummaryCollector,
    ) -> Set[str]:
        self.sample_index += 1
        elapsed_s = wall_time - start_wall_time

        cpu_times = self._read_cpu_times()
        cpu_percent = self._cpu_percent("cpu", cpu_times)
        mem = self._read_meminfo()
        load = self._read_loadavg()
        disk_rates = self._disk_rates(wall_time)
        net_rates = self._net_rates(wall_time)
        freq_avg, freq_max = self._read_cpu_freq_mhz()
        temp_c = self._read_temperature_c()

        system_row = {
            "sample_index": self.sample_index,
            "wall_time_iso": wall_iso(wall_time),
            "elapsed_s": fmt_float(elapsed_s, 3),
            "ros_time_s": fmt_float(ros_time_s, 6),
            "cpu_percent": fmt_float(cpu_percent, 3),
            "mem_total_mb": fmt_float(mem.get("mem_total_mb"), 3),
            "mem_available_mb": fmt_float(mem.get("mem_available_mb"), 3),
            "mem_used_mb": fmt_float(mem.get("mem_used_mb"), 3),
            "mem_used_percent": fmt_float(mem.get("mem_used_percent"), 3),
            "swap_total_mb": fmt_float(mem.get("swap_total_mb"), 3),
            "swap_used_mb": fmt_float(mem.get("swap_used_mb"), 3),
            "swap_used_percent": fmt_float(mem.get("swap_used_percent"), 3),
            "load1": fmt_float(load[0], 3),
            "load5": fmt_float(load[1], 3),
            "load15": fmt_float(load[2], 3),
            "disk_read_mbps": fmt_float(disk_rates[0], 3),
            "disk_write_mbps": fmt_float(disk_rates[1], 3),
            "net_rx_mbps": fmt_float(net_rates[0], 3),
            "net_tx_mbps": fmt_float(net_rates[1], 3),
            "cpu_freq_avg_mhz": fmt_float(freq_avg, 1),
            "cpu_freq_max_mhz": fmt_float(freq_max, 1),
            "cpu_temp_c": fmt_float(temp_c, 1),
        }
        csv_bundle.write("system_resources", system_row)

        summary.add("system_cpu_percent", cpu_percent)
        summary.add("system_mem_used_percent", mem.get("mem_used_percent"))
        summary.add("system_mem_used_mb", mem.get("mem_used_mb"))
        summary.add("system_swap_used_percent", mem.get("swap_used_percent"))
        summary.add("system_disk_read_mbps", disk_rates[0])
        summary.add("system_disk_write_mbps", disk_rates[1])
        summary.add("system_net_rx_mbps", net_rates[0])
        summary.add("system_net_tx_mbps", net_rates[1])
        summary.add("system_cpu_temp_c", temp_c)

        for core_name in sorted(
            [name for name in cpu_times if name != "cpu"],
            key=lambda name: int(name[3:]) if name[3:].isdigit() else 999999,
        ):
            csv_bundle.write(
                "cpu_cores",
                {
                    "sample_index": self.sample_index,
                    "wall_time_iso": wall_iso(wall_time),
                    "elapsed_s": fmt_float(elapsed_s, 3),
                    "ros_time_s": fmt_float(ros_time_s, 6),
                    "core": core_name,
                    "cpu_percent": fmt_float(self._cpu_percent(core_name, cpu_times), 3),
                },
            )

        self.prev_cpu_times = cpu_times
        return self._sample_processes(wall_time, elapsed_s, ros_time_s, csv_bundle, summary)

    def _read_cpu_times(self) -> Dict[str, Tuple[int, int]]:
        result: Dict[str, Tuple[int, int]] = {}
        text = read_text("/proc/stat")
        if text is None:
            return result
        for line in text.splitlines():
            if not line.startswith("cpu"):
                if result:
                    break
                continue
            parts = line.split()
            name = parts[0]
            if name != "cpu" and not name[3:].isdigit():
                continue
            values = [int(value) for value in parts[1:]]
            idle = values[3] + (values[4] if len(values) > 4 else 0)
            total = sum(values)
            result[name] = (idle, total)
        return result

    def _cpu_percent(self, name: str, current: Dict[str, Tuple[int, int]]) -> Optional[float]:
        if self.prev_cpu_times is None or name not in self.prev_cpu_times or name not in current:
            return None
        prev_idle, prev_total = self.prev_cpu_times[name]
        idle, total = current[name]
        total_delta = total - prev_total
        idle_delta = idle - prev_idle
        if total_delta <= 0:
            return None
        return max(0.0, min(100.0, 100.0 * (1.0 - idle_delta / total_delta)))

    def _read_meminfo(self) -> Dict[str, Optional[float]]:
        values_kb: Dict[str, float] = {}
        text = read_text("/proc/meminfo")
        if text is None:
            return {}
        for line in text.splitlines():
            if ":" not in line:
                continue
            key, raw_value = line.split(":", 1)
            parts = raw_value.strip().split()
            if not parts:
                continue
            try:
                values_kb[key] = float(parts[0])
            except ValueError:
                continue

        mem_total = values_kb.get("MemTotal")
        mem_available = values_kb.get("MemAvailable")
        swap_total = values_kb.get("SwapTotal", 0.0)
        swap_free = values_kb.get("SwapFree", 0.0)
        result: Dict[str, Optional[float]] = {}
        if mem_total:
            mem_used = mem_total - (mem_available or 0.0)
            result["mem_total_mb"] = mem_total / KIB
            result["mem_available_mb"] = (mem_available or 0.0) / KIB
            result["mem_used_mb"] = mem_used / KIB
            result["mem_used_percent"] = 100.0 * mem_used / mem_total
        if swap_total:
            swap_used = swap_total - swap_free
            result["swap_total_mb"] = swap_total / KIB
            result["swap_used_mb"] = swap_used / KIB
            result["swap_used_percent"] = 100.0 * swap_used / swap_total
        else:
            result["swap_total_mb"] = 0.0
            result["swap_used_mb"] = 0.0
            result["swap_used_percent"] = 0.0
        return result

    def _read_loadavg(self) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        text = read_text("/proc/loadavg")
        if text is None:
            return (None, None, None)
        parts = text.split()
        try:
            return (float(parts[0]), float(parts[1]), float(parts[2]))
        except (IndexError, ValueError):
            return (None, None, None)

    def _block_devices(self) -> set:
        devices = set()
        for path in glob.glob("/sys/block/*"):
            name = os.path.basename(path)
            if name.startswith(("loop", "ram", "zram")):
                continue
            devices.add(name)
        return devices

    def _read_disk_totals(self) -> Dict[str, int]:
        devices = self._block_devices()
        totals = {"read_bytes": 0, "write_bytes": 0}
        text = read_text("/proc/diskstats")
        if text is None:
            return totals
        for line in text.splitlines():
            parts = line.split()
            if len(parts) < 14:
                continue
            name = parts[2]
            if name not in devices:
                continue
            try:
                sectors_read = int(parts[5])
                sectors_written = int(parts[9])
            except ValueError:
                continue
            totals["read_bytes"] += sectors_read * 512
            totals["write_bytes"] += sectors_written * 512
        return totals

    def _disk_rates(self, wall_time: float) -> Tuple[Optional[float], Optional[float]]:
        current = self._read_disk_totals()
        rates: Tuple[Optional[float], Optional[float]] = (None, None)
        if self.prev_disk is not None:
            prev_wall, prev = self.prev_disk
            dt = wall_time - prev_wall
            if dt > 0:
                read_delta = current["read_bytes"] - prev.get("read_bytes", 0)
                write_delta = current["write_bytes"] - prev.get("write_bytes", 0)
                rates = (
                    max(0.0, read_delta / dt / MIB),
                    max(0.0, write_delta / dt / MIB),
                )
        self.prev_disk = (wall_time, current)
        return rates

    def _read_net_totals(self) -> Dict[str, int]:
        totals = {"rx_bytes": 0, "tx_bytes": 0}
        text = read_text("/proc/net/dev")
        if text is None:
            return totals
        for line in text.splitlines()[2:]:
            if ":" not in line:
                continue
            iface, raw_values = line.split(":", 1)
            if iface.strip() == "lo":
                continue
            parts = raw_values.split()
            if len(parts) < 16:
                continue
            try:
                totals["rx_bytes"] += int(parts[0])
                totals["tx_bytes"] += int(parts[8])
            except ValueError:
                continue
        return totals

    def _net_rates(self, wall_time: float) -> Tuple[Optional[float], Optional[float]]:
        current = self._read_net_totals()
        rates: Tuple[Optional[float], Optional[float]] = (None, None)
        if self.prev_net is not None:
            prev_wall, prev = self.prev_net
            dt = wall_time - prev_wall
            if dt > 0:
                rx_delta = current["rx_bytes"] - prev.get("rx_bytes", 0)
                tx_delta = current["tx_bytes"] - prev.get("tx_bytes", 0)
                rates = (
                    max(0.0, rx_delta / dt / MIB),
                    max(0.0, tx_delta / dt / MIB),
                )
        self.prev_net = (wall_time, current)
        return rates

    def _read_cpu_freq_mhz(self) -> Tuple[Optional[float], Optional[float]]:
        values = []
        for path in glob.glob("/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_cur_freq"):
            text = read_text(path)
            if text is None:
                continue
            try:
                values.append(float(text.strip()) / 1000.0)
            except ValueError:
                continue
        if not values:
            return (None, None)
        return (sum(values) / len(values), max(values))

    def _read_temperature_c(self) -> Optional[float]:
        values = []
        paths = glob.glob("/sys/class/thermal/thermal_zone*/temp")
        paths.extend(glob.glob("/sys/class/hwmon/hwmon*/temp*_input"))
        for path in paths:
            text = read_text(path)
            if text is None:
                continue
            try:
                raw = float(text.strip())
            except ValueError:
                continue
            values.append(raw / 1000.0 if raw > 200.0 else raw)
        if not values:
            return None
        return max(values)

    def _iter_pids(self) -> Iterable[int]:
        for name in os.listdir("/proc"):
            if name.isdigit():
                yield int(name)

    def _read_cmdline(self, pid: int) -> str:
        raw = read_bytes(f"/proc/{pid}/cmdline")
        if not raw:
            comm = read_text(f"/proc/{pid}/comm")
            return (comm or "").strip()
        parts = [part.decode("utf-8", errors="replace") for part in raw.split(b"\0") if part]
        return " ".join(parts)

    def _classify_process(self, command: str) -> Optional[str]:
        lowered = command.lower()
        padded = f" {lowered} "
        if "gicp_plusplus_node" in lowered or "gicp_localization_node" in lowered:
            return "localization"
        if "robot_state_publisher" in lowered:
            return "robot_state_publisher"
        if "rviz2" in lowered:
            return "rviz2"
        if "ros2" in lowered and " bag " in padded and " play " in padded:
            return "rosbag_play"
        if "rosbag2" in lowered and " play " in padded:
            return "rosbag_play"
        return None

    def _read_process_stat(self, pid: int) -> Optional[Dict[str, float]]:
        text = read_text(f"/proc/{pid}/stat")
        if text is None:
            return None
        end = text.rfind(")")
        if end < 0:
            return None
        fields = text[end + 2 :].split()
        if len(fields) < 22:
            return None
        try:
            return {
                "utime_ticks": float(fields[11]),
                "stime_ticks": float(fields[12]),
                "num_threads": float(fields[17]),
                "starttime_ticks": float(fields[19]),
                "vsize_bytes": float(fields[20]),
                "rss_bytes": float(fields[21]) * self.page_size,
            }
        except ValueError:
            return None

    def _read_status(self, pid: int) -> Dict[str, float]:
        result: Dict[str, float] = {}
        text = read_text(f"/proc/{pid}/status")
        if text is None:
            return result
        for line in text.splitlines():
            if ":" not in line:
                continue
            key, raw_value = line.split(":", 1)
            parts = raw_value.strip().split()
            if not parts:
                continue
            try:
                value = float(parts[0])
            except ValueError:
                continue
            if key in {"VmRSS", "VmHWM", "VmSize"}:
                result[key] = value * 1024.0
            elif key in {"Threads", "voluntary_ctxt_switches", "nonvoluntary_ctxt_switches"}:
                result[key] = value
        return result

    def _read_process_io(self, pid: int) -> Dict[str, float]:
        result = {"read_bytes": 0.0, "write_bytes": 0.0}
        text = read_text(f"/proc/{pid}/io")
        if text is None:
            return result
        for line in text.splitlines():
            if ":" not in line:
                continue
            key, raw_value = line.split(":", 1)
            if key not in result:
                continue
            try:
                result[key] = float(raw_value.strip())
            except ValueError:
                continue
        return result

    def _read_pss_bytes(self, pid: int) -> Optional[float]:
        text = read_text(f"/proc/{pid}/smaps_rollup")
        if text is None:
            return None
        for line in text.splitlines():
            if not line.startswith("Pss:"):
                continue
            parts = line.split()
            if len(parts) < 2:
                return None
            try:
                return float(parts[1]) * 1024.0
            except ValueError:
                return None
        return None

    def _sample_processes(
        self,
        wall_time: float,
        elapsed_s: float,
        ros_time_s: Optional[float],
        csv_bundle: CsvBundle,
        summary: SummaryCollector,
    ) -> Set[str]:
        current_pids = set()
        roles_present: Set[str] = set()
        role_totals: Dict[str, Dict[str, float]] = defaultdict(lambda: {"cpu": 0.0, "rss": 0.0, "pss": 0.0})

        for pid in self._iter_pids():
            if pid == os.getpid():
                continue
            command = self._read_cmdline(pid)
            role = self._classify_process(command)
            if role is None:
                continue
            roles_present.add(role)

            proc_stat = self._read_process_stat(pid)
            if proc_stat is None:
                continue
            status = self._read_status(pid)
            io = self._read_process_io(pid)
            pss_bytes = self._read_pss_bytes(pid)

            proc_time_s = (proc_stat["utime_ticks"] + proc_stat["stime_ticks"]) / self.clock_ticks
            previous = self.prev_process.get(pid)
            cpu_percent = None
            read_mbps = None
            write_mbps = None
            if previous and previous.get("starttime_ticks") == proc_stat["starttime_ticks"]:
                dt = wall_time - previous["wall_time"]
                if dt > 0:
                    cpu_delta = proc_time_s - previous["proc_time_s"]
                    cpu_percent = max(0.0, 100.0 * cpu_delta / dt)
                    read_delta = io["read_bytes"] - previous.get("read_bytes", 0.0)
                    write_delta = io["write_bytes"] - previous.get("write_bytes", 0.0)
                    read_mbps = max(0.0, read_delta / dt / MIB)
                    write_mbps = max(0.0, write_delta / dt / MIB)

            rss_bytes = status.get("VmRSS", proc_stat["rss_bytes"])
            vms_bytes = status.get("VmSize", proc_stat["vsize_bytes"])
            peak_rss_bytes = status.get("VmHWM")
            threads = status.get("Threads", proc_stat["num_threads"])
            voluntary = status.get("voluntary_ctxt_switches")
            nonvoluntary = status.get("nonvoluntary_ctxt_switches")

            csv_bundle.write(
                "process_resources",
                {
                    "sample_index": self.sample_index,
                    "wall_time_iso": wall_iso(wall_time),
                    "elapsed_s": fmt_float(elapsed_s, 3),
                    "ros_time_s": fmt_float(ros_time_s, 6),
                    "role": role,
                    "pid": pid,
                    "command": command,
                    "cpu_percent": fmt_float(cpu_percent, 3),
                    "cpu_percent_host": fmt_float(None if cpu_percent is None else cpu_percent / self.cpu_count, 3),
                    "rss_mb": fmt_float(rss_bytes / MIB, 3),
                    "vms_mb": fmt_float(vms_bytes / MIB, 3),
                    "pss_mb": fmt_float(None if pss_bytes is None else pss_bytes / MIB, 3),
                    "peak_rss_mb": fmt_float(None if peak_rss_bytes is None else peak_rss_bytes / MIB, 3),
                    "threads": fmt_int(threads),
                    "voluntary_ctxt_switches": fmt_int(voluntary),
                    "nonvoluntary_ctxt_switches": fmt_int(nonvoluntary),
                    "read_bytes": fmt_int(io["read_bytes"]),
                    "write_bytes": fmt_int(io["write_bytes"]),
                    "read_mbps": fmt_float(read_mbps, 3),
                    "write_mbps": fmt_float(write_mbps, 3),
                },
            )

            if cpu_percent is not None:
                role_totals[role]["cpu"] += cpu_percent
            role_totals[role]["rss"] += rss_bytes / MIB
            if pss_bytes is not None:
                role_totals[role]["pss"] += pss_bytes / MIB

            self.prev_process[pid] = {
                "wall_time": wall_time,
                "proc_time_s": proc_time_s,
                "starttime_ticks": proc_stat["starttime_ticks"],
                "read_bytes": io["read_bytes"],
                "write_bytes": io["write_bytes"],
            }
            current_pids.add(pid)

        self.prev_process = {pid: data for pid, data in self.prev_process.items() if pid in current_pids}
        for role, totals in role_totals.items():
            summary.add(f"process_{role}_cpu_percent", totals["cpu"])
            summary.add(f"process_{role}_rss_mb", totals["rss"])
            if totals["pss"] > 0.0:
                summary.add(f"process_{role}_pss_mb", totals["pss"])
        return roles_present


if rclpy is not None:

    class LocalizationProfilerNode(Node):
        FLOAT_TOPICS = {
            "/gicp/localization/debug/gicp_elapsed_ms": "gicp_elapsed_ms",
            "/gicp/localization/debug/fitness": "fitness",
            "/gicp/localization/debug/scan_dt": "scan_dt_s",
            "/gicp/localization/debug/raw_points": "raw_points",
            "/gicp/localization/debug/preprocessed_points": "preprocessed_points",
            "/gicp/localization/debug/num_correspondences": "num_correspondences",
            "/gicp/localization/debug/correspondence_ratio": "correspondence_ratio",
            "/gicp/localization/debug/final_error": "final_error",
            "/gicp/localization/debug/guess_to_solution_trans_m": "guess_to_solution_trans_m",
            "/gicp/localization/debug/guess_to_solution_rot_deg": "guess_to_solution_rot_deg",
            "/gicp/localization/debug/guess_from_last_m": "guess_from_last_m",
            "/gicp/localization/debug/guess_from_last_deg": "guess_from_last_deg",
            "/gicp/localization/debug/jump_trans": "jump_trans_m",
            "/gicp/localization/debug/jump_rot_deg": "jump_rot_deg",
            "/gicp/localization/debug/imu_buffer_span_s": "imu_buffer_span_s",
            "/gicp/localization/debug/scan_to_latest_imu_lag_s": "scan_to_latest_imu_lag_s",
            "/gicp/localization/debug/hessian_condition_proxy": "hessian_condition_proxy",
        }

        ROS_SUMMARY_FIELDS = [
            "gicp_elapsed_ms",
            "fitness",
            "scan_dt_s",
            "raw_points",
            "preprocessed_points",
            "num_correspondences",
            "correspondence_ratio",
            "final_error",
            "guess_to_solution_trans_m",
            "guess_to_solution_rot_deg",
            "guess_from_last_m",
            "guess_from_last_deg",
            "jump_trans_m",
            "jump_rot_deg",
            "imu_buffer_span_s",
            "scan_to_latest_imu_lag_s",
            "hessian_condition_proxy",
        ]

        def __init__(self, args, start_wall_time: float, csv_bundle: CsvBundle, summary: SummaryCollector) -> None:
            super().__init__("localization_resource_profiler")
            self.args = args
            self.start_wall_time = start_wall_time
            self.csv_bundle = csv_bundle
            self.summary = summary
            self.latest_metrics: Dict[str, object] = {}
            self.latest_clock_s: Optional[float] = None
            self.stop_requested = False

            self.start_pose: Optional[Tuple[float, float, float]] = None
            self.last_pose: Optional[Tuple[float, float, float]] = None
            self.path_length_m = 0.0
            self.lap_armed = False
            self.lap_count = 0
            self.last_lap_wall = 0.0

            self.create_subscription(Clock, "/clock", self._on_clock, 10)
            self.create_subscription(PoseStamped, "/gicp/localization/pose", self._on_localized_pose, 50)
            self.create_subscription(PoseStamped, "/gicp/localization/debug/final_pose", self._on_final_pose, 50)
            self.create_subscription(Bool, "/gicp/localization/debug/converged", self._on_converged, 50)

            for topic, key in self.FLOAT_TOPICS.items():
                self.create_subscription(Float64, topic, self._make_float_callback(key), 50)

            self.get_logger().info(f"Writing profiler CSVs to {csv_bundle.output_dir}")
            stop_text = (
                f"and stops after {args.stop_after_laps} lap returns"
                if args.stop_after_laps > 0
                else "and runs until interrupted"
            )
            if args.stop_after_seconds > 0.0:
                stop_text += f", or after {args.stop_after_seconds:.1f}s"
            if args.stop_when_rosbag_ends:
                stop_text += f", or {args.rosbag_end_grace_s:.1f}s after rosbag playback exits"
            self.get_logger().info(
                "Lap detector waits for the first /gicp/localization/pose, arms after moving away, "
                f"{stop_text}."
            )

        def _make_float_callback(self, key: str):
            def callback(msg) -> None:
                self.latest_metrics[key] = float(msg.data)

            return callback

        def _on_converged(self, msg) -> None:
            self.latest_metrics["converged"] = bool(msg.data)

        def _on_clock(self, msg) -> None:
            self.latest_clock_s = stamp_to_sec(msg.clock)

        def _on_final_pose(self, msg) -> None:
            wall_time = time.time()
            elapsed_s = wall_time - self.start_wall_time
            row = {
                "wall_time_iso": wall_iso(wall_time),
                "elapsed_s": fmt_float(elapsed_s, 3),
                "ros_time_s": fmt_float(self.latest_clock_s, 6),
                "scan_stamp_s": fmt_float(stamp_to_sec(msg.header.stamp), 6),
                "final_x": fmt_float(msg.pose.position.x, 6),
                "final_y": fmt_float(msg.pose.position.y, 6),
                "final_z": fmt_float(msg.pose.position.z, 6),
                "converged": "" if "converged" not in self.latest_metrics else str(self.latest_metrics["converged"]).lower(),
            }
            for key in self.FLOAT_TOPICS.values():
                value = self.latest_metrics.get(key)
                digits = 3 if key.endswith("_points") or key == "num_correspondences" else 6
                row[key] = fmt_float(value, digits)
                if key in self.ROS_SUMMARY_FIELDS:
                    self.summary.add(f"ros_{key}", value)
            self.csv_bundle.write("ros_scan_metrics", row)

        def _on_localized_pose(self, msg) -> None:
            wall_time = time.time()
            pose_stamp_s = stamp_to_sec(msg.header.stamp)
            x = float(msg.pose.position.x)
            y = float(msg.pose.position.y)
            z = float(msg.pose.position.z)
            current_pose = (x, y, z)

            if self.start_pose is None:
                self.start_pose = current_pose
                self.last_pose = current_pose
                self.get_logger().info(
                    f"Lap start pose set at stamp={pose_stamp_s:.3f}s x={x:.2f} y={y:.2f} z={z:.2f}"
                )
                return

            if self.last_pose is not None:
                step = math.dist(self.last_pose, current_pose)
                if step <= self.args.max_pose_step_m:
                    self.path_length_m += step
            self.last_pose = current_pose

            distance_to_start = math.dist(self.start_pose[:2], current_pose[:2])
            if not self.lap_armed and distance_to_start >= self.args.lap_away_radius_m:
                self.lap_armed = True
                self.get_logger().info(
                    f"Lap detector armed after moving {distance_to_start:.2f} m from the start pose."
                )

            if not self.lap_armed:
                return

            if distance_to_start > self.args.lap_radius_m:
                return

            if wall_time - self.last_lap_wall < self.args.lap_cooldown_s:
                return

            self.lap_count += 1
            self.last_lap_wall = wall_time
            self.lap_armed = False
            elapsed_s = wall_time - self.start_wall_time
            self.csv_bundle.write(
                "lap_events",
                {
                    "wall_time_iso": wall_iso(wall_time),
                    "elapsed_s": fmt_float(elapsed_s, 3),
                    "ros_time_s": fmt_float(self.latest_clock_s, 6),
                    "lap_number": self.lap_count,
                    "pose_stamp_s": fmt_float(pose_stamp_s, 6),
                    "x": fmt_float(x, 6),
                    "y": fmt_float(y, 6),
                    "z": fmt_float(z, 6),
                    "distance_to_start_m": fmt_float(distance_to_start, 6),
                    "path_length_m": fmt_float(self.path_length_m, 6),
                },
            )
            self.summary.add("lap_path_length_m", self.path_length_m)
            self.get_logger().info(
                f"Lap {self.lap_count} detected at stamp={pose_stamp_s:.3f}s "
                f"distance_to_start={distance_to_start:.2f} m path_length={self.path_length_m:.2f} m"
            )
            if self.args.stop_after_laps > 0 and self.lap_count >= self.args.stop_after_laps:
                self.get_logger().info(f"Reached stop-after-laps={self.args.stop_after_laps}; stopping profiler.")
                self.stop_requested = True

else:
    LocalizationProfilerNode = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Attach-only profiler for GICP localization resource and scan metrics."
    )
    parser.add_argument(
        "--output-dir",
        default="/tmp/gicp_profile",
        help="Root output directory. A timestamped run directory is created inside it.",
    )
    parser.add_argument(
        "--sample-period",
        type=float,
        default=0.5,
        help="System/process sampling period in seconds.",
    )
    parser.add_argument(
        "--stop-after-laps",
        type=int,
        default=2,
        help="Stop after this many returns near the start pose. Use 0 to disable automatic lap stop.",
    )
    parser.add_argument(
        "--stop-after-seconds",
        type=float,
        default=0.0,
        help="Stop after this many wall-clock seconds. Use 0 to disable duration-based stopping.",
    )
    parser.add_argument(
        "--lap-radius-m",
        type=float,
        default=10.0,
        help="Distance from the start pose that counts as a lap return.",
    )
    parser.add_argument(
        "--lap-away-radius-m",
        type=float,
        default=None,
        help="Distance from the start pose required before the next lap can be counted. Default: max(2*lap-radius, lap-radius+5).",
    )
    parser.add_argument(
        "--lap-cooldown-s",
        type=float,
        default=20.0,
        help="Minimum wall-clock seconds between counted lap events.",
    )
    parser.add_argument(
        "--max-pose-step-m",
        type=float,
        default=100.0,
        help="Ignore larger pose-to-pose jumps when integrating approximate path length.",
    )
    parser.add_argument(
        "--stop-when-rosbag-ends",
        dest="stop_when_rosbag_ends",
        action="store_true",
        default=True,
        help="Stop after a ros2 bag play process has been observed and then exits. Enabled by default.",
    )
    parser.add_argument(
        "--no-stop-when-rosbag-ends",
        dest="stop_when_rosbag_ends",
        action="store_false",
        help="Disable automatic stop on rosbag process exit.",
    )
    parser.add_argument(
        "--rosbag-end-grace-s",
        type=float,
        default=3.0,
        help="Seconds to keep sampling after the observed rosbag play process exits.",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Take one system/process sample, write summary.csv, and exit.",
    )
    parser.add_argument(
        "--no-ros",
        action="store_true",
        help="Do not import or subscribe to ROS. Useful with --once for smoke testing /proc parsing.",
    )
    return parser.parse_args()


def make_output_dir(root: str) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    root_dir = Path(root).expanduser().resolve()
    for suffix in [""] + [f"_{index:02d}" for index in range(1, 100)]:
        output_dir = root_dir / f"gicp_profile_{timestamp}{suffix}"
        try:
            output_dir.mkdir(parents=True, exist_ok=False)
            return output_dir
        except FileExistsError:
            continue
    raise RuntimeError(f"Could not create a unique output directory under {root_dir}")


def update_rosbag_end_state(
    args,
    roles_present: Set[str],
    wall_time: float,
    state: Dict[str, object],
) -> Tuple[bool, Optional[str]]:
    if not args.stop_when_rosbag_ends:
        return (False, None)

    if "rosbag_play" in roles_present:
        if not state.get("seen", False):
            state["seen"] = True
            state["missing_since"] = None
            return (False, "Detected rosbag playback process; profiler will stop after it exits.")
        state["missing_since"] = None
        return (False, None)

    if not state.get("seen", False):
        return (False, None)

    missing_since = state.get("missing_since")
    if missing_since is None:
        state["missing_since"] = wall_time
        return (False, "Rosbag playback process is no longer visible; waiting for grace period.")

    if wall_time - float(missing_since) >= args.rosbag_end_grace_s:
        return (True, f"Rosbag playback ended; stopping profiler after {args.rosbag_end_grace_s:.1f}s grace period.")

    return (False, None)


def run_no_ros(args, start_wall_time: float, csv_bundle: CsvBundle, summary: SummaryCollector) -> int:
    sampler = LinuxResourceSampler()
    stop = {"requested": False}
    rosbag_state: Dict[str, object] = {"seen": False, "missing_since": None}

    def handle_signal(signum, frame) -> None:
        del signum, frame
        stop["requested"] = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    while not stop["requested"]:
        wall_time = time.time()
        roles_present = sampler.sample(wall_time, start_wall_time, None, csv_bundle, summary)
        if args.stop_after_seconds > 0.0 and wall_time - start_wall_time >= args.stop_after_seconds:
            print(f"Reached stop-after-seconds={args.stop_after_seconds:.1f}; stopping profiler.", flush=True)
            break
        should_stop, message = update_rosbag_end_state(args, roles_present, wall_time, rosbag_state)
        if message:
            print(message, flush=True)
        if should_stop:
            break
        if args.once:
            break
        time.sleep(args.sample_period)
    return 0


def run_ros(args, start_wall_time: float, csv_bundle: CsvBundle, summary: SummaryCollector) -> int:
    if rclpy is None or LocalizationProfilerNode is None:
        print("ROS Python modules are not available. Source your ROS 2 environment or use --no-ros.", file=sys.stderr)
        return 2

    sampler = LinuxResourceSampler()
    stop = {"requested": False}
    rosbag_state: Dict[str, object] = {"seen": False, "missing_since": None}

    def handle_signal(signum, frame) -> None:
        del signum, frame
        stop["requested"] = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    rclpy.init()
    node = None
    executor = None
    try:
        node = LocalizationProfilerNode(args, start_wall_time, csv_bundle, summary)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        next_sample = time.monotonic()

        while rclpy.ok() and not stop["requested"] and not node.stop_requested:
            executor.spin_once(timeout_sec=0.05)
            now_monotonic = time.monotonic()
            if now_monotonic >= next_sample:
                wall_time = time.time()
                roles_present = sampler.sample(wall_time, start_wall_time, node.latest_clock_s, csv_bundle, summary)
                if args.stop_after_seconds > 0.0 and wall_time - start_wall_time >= args.stop_after_seconds:
                    node.get_logger().info(
                        f"Reached stop-after-seconds={args.stop_after_seconds:.1f}; stopping profiler."
                    )
                    node.stop_requested = True
                    break
                should_stop, message = update_rosbag_end_state(args, roles_present, wall_time, rosbag_state)
                if message:
                    node.get_logger().info(message)
                if should_stop:
                    node.stop_requested = True
                    break
                if args.once:
                    break
                next_sample = now_monotonic + args.sample_period
        return 0
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    args = parse_args()
    if args.sample_period <= 0.0:
        print("--sample-period must be > 0", file=sys.stderr)
        return 2
    if args.stop_after_laps < 0:
        print("--stop-after-laps must be >= 0", file=sys.stderr)
        return 2
    if args.stop_after_seconds < 0.0:
        print("--stop-after-seconds must be >= 0", file=sys.stderr)
        return 2
    if args.lap_radius_m <= 0.0:
        print("--lap-radius-m must be > 0", file=sys.stderr)
        return 2
    if args.rosbag_end_grace_s < 0.0:
        print("--rosbag-end-grace-s must be >= 0", file=sys.stderr)
        return 2
    if args.lap_away_radius_m is None:
        args.lap_away_radius_m = max(2.0 * args.lap_radius_m, args.lap_radius_m + 5.0)

    output_dir = make_output_dir(args.output_dir)
    start_wall_time = time.time()
    csv_bundle = CsvBundle(output_dir)
    summary = SummaryCollector()
    exit_code = 1
    try:
        if args.no_ros:
            exit_code = run_no_ros(args, start_wall_time, csv_bundle, summary)
        else:
            exit_code = run_ros(args, start_wall_time, csv_bundle, summary)
        return exit_code
    finally:
        summary.write_csv(output_dir / "summary.csv")
        csv_bundle.close()
        print(f"Profiler output: {output_dir}", flush=True)


if __name__ == "__main__":
    sys.exit(main())
