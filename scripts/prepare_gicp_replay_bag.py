#!/usr/bin/env python3
"""Build a topic-reduced rosbag for deterministic real-time GICP audits.

ros2 bag play still has to scan every message in each input bag even when
--topics is used.  Merging only the localization contract into one compressed
MCAP removes unrelated camera/aux-LiDAR traffic from the audit I/O path.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Iterable


DEFAULT_TOPICS = (
    "/luminar_front/points",
    "/gps_p1/imu",
    "/gps_p1/filtered_odom",
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def dataset_root_from_path(path: Path) -> Path | None:
    """Return .../rosbags/<dataset> when the path carries that contract."""
    parts = path.resolve().parts
    for index, part in enumerate(parts[:-1]):
        if part == "rosbags" and index + 1 < len(parts):
            return Path(*parts[: index + 2])
    return None


def is_relative_to(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def metadata_path(bag: Path) -> Path | None:
    candidate = bag / "metadata.yaml" if bag.is_dir() else bag.parent / "metadata.yaml"
    return candidate if candidate.is_file() else None


def source_record(bag: Path) -> dict[str, object]:
    metadata = metadata_path(bag)
    record: dict[str, object] = {
        "path": str(bag),
        "kind": "directory" if bag.is_dir() else "file",
    }
    if metadata is not None:
        record["metadata_yaml"] = str(metadata)
        record["metadata_sha256"] = sha256_file(metadata)
    if bag.is_file():
        record["bytes"] = bag.stat().st_size
    return record


def unique(values: Iterable[str]) -> list[str]:
    result: list[str] = []
    seen: set[str] = set()
    for value in values:
        if value not in seen:
            seen.add(value)
            result.append(value)
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Merge only the front-LiDAR/Atlas localization contract into one "
            "compressed MCAP under DATASET_ROOT/prep_bag."
        )
    )
    parser.add_argument(
        "--bag",
        action="append",
        required=True,
        help="Input rosbag2 directory/file; repeat for multiple sources.",
    )
    output_group = parser.add_mutually_exclusive_group()
    output_group.add_argument(
        "--out",
        help="Explicit output bag path, normally DATASET_ROOT/prep_bag/<name>.",
    )
    output_group.add_argument(
        "--prep-root",
        help="Explicit DATASET_ROOT/prep_bag; requires --name.",
    )
    parser.add_argument(
        "--name",
        help="Output directory name. Required unless --out supplies the full path.",
    )
    parser.add_argument(
        "--topic",
        action="append",
        help="Topic to retain; repeat to replace the default front/IMU/odom set.",
    )
    parser.add_argument("--storage-id", default="mcap")
    parser.add_argument("--storage-preset", default="zstd_fast")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate scope and write the conversion config without converting.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    bags = [Path(value).expanduser().resolve(strict=True) for value in args.bag]
    if len(set(bags)) != len(bags):
        raise SystemExit("Duplicate --bag inputs are not allowed")

    if args.out:
        output = Path(args.out).expanduser().resolve(strict=False)
        prep_root = output.parent
    else:
        if not args.name:
            raise SystemExit("--name is required unless --out supplies the full path")
        if "/" in args.name or args.name in {".", ".."}:
            raise SystemExit("--name must be one directory component")
        if args.prep_root:
            prep_root = Path(args.prep_root).expanduser().resolve(strict=False)
        else:
            inferred_roots = [
                root for bag in bags if (root := dataset_root_from_path(bag)) is not None
            ]
            if not inferred_roots:
                raise SystemExit(
                    "Could not derive DATASET_ROOT from --bag; pass --prep-root or --out"
                )
            if any(root != inferred_roots[0] for root in inferred_roots[1:]):
                raise SystemExit("Input bags resolve to more than one DATASET_ROOT")
            prep_root = inferred_roots[0] / "prep_bag"
        output = prep_root / args.name

    dataset_root = dataset_root_from_path(output)
    if dataset_root is None:
        raise SystemExit(
            "Output does not carry a .../rosbags/<dataset> DATASET_ROOT contract"
        )
    required_prep_root = dataset_root / "prep_bag"
    if not is_relative_to(output, required_prep_root):
        raise SystemExit(
            f"Output must remain under this dataset's prep_bag: {required_prep_root}"
        )

    for bag in bags:
        source_root = dataset_root_from_path(bag)
        if source_root is not None and source_root != dataset_root:
            raise SystemExit(
                f"Cross-dataset input refused: {bag} belongs to {source_root}, "
                f"output belongs to {dataset_root}"
            )
        if output == bag or is_relative_to(output, bag) or is_relative_to(bag, output):
            raise SystemExit(
                f"Input/output path collision refused: input={bag} output={output}"
            )

    if output.exists():
        raise SystemExit(f"Refusing to overwrite output bag: {output}")
    topics = unique(args.topic or DEFAULT_TOPICS)
    if not topics or any(not topic.startswith("/") for topic in topics):
        raise SystemExit("Every retained topic must be an absolute ROS topic")

    prep_root.mkdir(parents=True, exist_ok=True)
    # Dry-run is intentionally isolated so it cannot occupy the production
    # conversion-config pathname and block the subsequent real conversion.
    config_dir = prep_root / ("dry_run_configs" if args.dry_run else "configs")
    config_dir.mkdir(parents=True, exist_ok=True)
    config_path = config_dir / f"{output.name}.convert.yaml"
    if config_path.exists():
        raise SystemExit(f"Refusing to overwrite conversion config: {config_path}")

    config = {
        "output_bags": [
            {
                "uri": str(output),
                "storage_id": args.storage_id,
                "storage_preset_profile": args.storage_preset,
                "topics": topics,
            }
        ]
    }
    config_path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")

    command = ["ros2", "bag", "convert"]
    for bag in bags:
        command.extend(("-i", str(bag)))
    command.extend(("-o", str(config_path)))

    print(f"dataset_root={dataset_root}")
    print(f"output={output}")
    print(f"config={config_path}")
    print("topics=" + ",".join(topics))
    print("command=" + " ".join(command))
    if args.dry_run:
        return 0

    if shutil.which("ros2") is None:
        raise SystemExit("ros2 not found; source /opt/ros/jazzy/setup.bash first")
    completed = subprocess.run(command, check=False)
    if completed.returncode != 0:
        return completed.returncode
    if not output.is_dir() or not (output / "metadata.yaml").is_file():
        raise SystemExit("ros2 bag convert returned success without a readable output bag")

    bag_info = subprocess.run(
        ["ros2", "bag", "info", str(output)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    (output / "bag_info.txt").write_text(bag_info, encoding="utf-8")
    manifest = {
        "schema": 1,
        "dataset_root": str(dataset_root),
        "output": str(output),
        "output_metadata_sha256": sha256_file(output / "metadata.yaml"),
        "conversion_config": str(config_path),
        "conversion_config_sha256": sha256_file(config_path),
        "storage_id": args.storage_id,
        "storage_preset_profile": args.storage_preset,
        "topics": topics,
        "sources": [source_record(bag) for bag in bags],
        "command": command,
    }
    (output / "preparation_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    print(f"metadata_sha256={manifest['output_metadata_sha256']}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, subprocess.SubprocessError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
