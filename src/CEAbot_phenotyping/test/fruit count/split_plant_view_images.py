#!/usr/bin/env python3
"""Move or copy time-filtered plant-view images into dataset splits.

The scan timestamp is read from a scan directory name such as
``b1_r11_20260806_145201``. Each discovered plant (all ``view_*`` images and
its ``top`` image) is assigned as one group to train, validate, or test. A
matching ``color.json`` annotation is transferred with the image when present.

The command is a dry run unless ``--execute`` is supplied.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, time
import json
from pathlib import Path
import random
import shutil
import sys


SCAN_TIMESTAMP_FORMAT = "%Y%m%d_%H%M%S"
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".tif", ".tiff"}


@dataclass(frozen=True)
class PlantView:
    scan_dir: Path
    plant_dir: Path
    view_dir: Path
    image: Path

    @property
    def dataset_stem(self) -> str:
        """Return a unique, traceable name for the flattened dataset image."""
        return f"{self.scan_dir.name}__{self.plant_dir.name}__{self.view_dir.name}"


def parse_time(value: str, *, end_of_day: bool = False) -> datetime:
    """Parse scan-style timestamps or common ISO-style date/time values."""
    normalized = value.strip().replace("T", " ")
    formats = (
        SCAN_TIMESTAMP_FORMAT,
        "%Y-%m-%d %H:%M:%S",
        "%Y-%m-%d %H:%M",
        "%Y-%m-%d",
    )
    for timestamp_format in formats:
        try:
            parsed = datetime.strptime(normalized, timestamp_format)
            if timestamp_format == "%Y-%m-%d" and end_of_day:
                return datetime.combine(parsed.date(), time.max)
            return parsed
        except ValueError:
            continue
    raise argparse.ArgumentTypeError(
        f"invalid time {value!r}; use YYYYMMDD_HHMMSS or "
        "YYYY-MM-DD[ HH:MM[:SS]]"
    )


def scan_timestamp(scan_dir: Path) -> datetime | None:
    try:
        timestamp_text = scan_dir.name.rsplit("_", 2)[-2:]
        return datetime.strptime("_".join(timestamp_text), SCAN_TIMESTAMP_FORMAT)
    except (ValueError, IndexError):
        return None


def percentage(value: str) -> float:
    """Accept either a percentage (70) or a fraction (0.70)."""
    try:
        amount = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid percentage: {value!r}") from exc
    if 0.0 <= amount <= 1.0:
        amount *= 100.0
    if not 0.0 <= amount <= 100.0:
        raise argparse.ArgumentTypeError("percentage must be between 0 and 100")
    return amount


def find_views(
    source: Path,
    start: datetime | None,
    end: datetime | None,
    image_name: str,
) -> list[PlantView]:
    views: list[PlantView] = []
    for scan_dir in sorted(path for path in source.iterdir() if path.is_dir()):
        timestamp = scan_timestamp(scan_dir)
        if timestamp is None:
            continue
        if start is not None and timestamp < start:
            continue
        if end is not None and timestamp > end:
            continue

        for plant_dir in sorted(scan_dir.glob("plant_*")):
            if not plant_dir.is_dir():
                continue
            view_dirs = sorted(path for path in plant_dir.glob("view_*") if path.is_dir())
            if (plant_dir / "top").is_dir():
                view_dirs.append(plant_dir / "top")
            for view_dir in view_dirs:
                image = view_dir / image_name
                if image.is_file():
                    views.append(PlantView(scan_dir, plant_dir, view_dir, image))
    return views


def split_counts(total: int, percentages: list[float]) -> list[int]:
    """Allocate all items using the largest-remainder rounding method."""
    exact = [total * percent / 100.0 for percent in percentages]
    counts = [int(value) for value in exact]
    remaining = total - sum(counts)
    order = sorted(
        range(len(exact)), key=lambda index: exact[index] - counts[index], reverse=True
    )
    for index in order[:remaining]:
        counts[index] += 1
    return counts


def assign_splits(
    views: list[PlantView], train: float, validate: float, test: float, seed: int
) -> dict[str, list[PlantView]]:
    """Split whole (scan, plant) groups so related views cannot leak."""
    grouped: dict[tuple[Path, Path], list[PlantView]] = {}
    for view in views:
        grouped.setdefault((view.scan_dir, view.plant_dir), []).append(view)

    shuffled_groups = list(grouped.values())
    random.Random(seed).shuffle(shuffled_groups)
    train_count, validate_count, test_count = split_counts(
        len(shuffled_groups), [train, validate, test]
    )
    train_end = train_count
    validate_end = train_end + validate_count

    def flatten(groups: list[list[PlantView]]) -> list[PlantView]:
        return [view for group in groups for view in group]

    return {
        "train": flatten(shuffled_groups[:train_end]),
        "validate": flatten(shuffled_groups[train_end:validate_end]),
        "test": flatten(
            shuffled_groups[validate_end : validate_end + test_count]
        ),
    }


def related_files(view: PlantView) -> list[Path]:
    """Keep an existing X-AnyLabeling annotation beside its source image."""
    files = [view.image]
    annotation = view.image.with_suffix(".json")
    if annotation.is_file():
        files.append(annotation)
    return files


def transfer_file(
    source_file: Path,
    target: Path,
    image_target_name: str,
    operation: str,
) -> None:
    """Transfer a file, keeping X-AnyLabeling JSON linked to its image."""
    if source_file.suffix.lower() != ".json":
        if operation == "move":
            shutil.move(str(source_file), str(target))
        else:
            shutil.copy2(source_file, target)
        return

    try:
        with source_file.open("r", encoding="utf-8") as annotation_file:
            annotation = json.load(annotation_file)
        annotation["imagePath"] = image_target_name
        with target.open("x", encoding="utf-8") as annotation_file:
            json.dump(annotation, annotation_file, ensure_ascii=False, indent=2)
            annotation_file.write("\n")
        shutil.copystat(source_file, target)
        if operation == "move":
            source_file.unlink()
    except (OSError, TypeError, ValueError, json.JSONDecodeError):
        if target.exists():
            target.unlink()
        raise


def transfer(
    assignments: dict[str, list[PlantView]],
    destination: Path,
    operation: str,
    execute: bool,
) -> tuple[int, int]:
    transferred = 0
    conflicts = 0
    verb = "MOVE" if operation == "move" else "COPY"

    for split_name, views in assignments.items():
        split_dir = destination / split_name
        if execute:
            split_dir.mkdir(parents=True, exist_ok=True)
        for view in views:
            image_target_name = f"{view.dataset_stem}{view.image.suffix.lower()}"
            for source_file in related_files(view):
                target = split_dir / f"{view.dataset_stem}{source_file.suffix.lower()}"
                if target.exists():
                    print(f"CONFLICT: target already exists, skipped: {target}", file=sys.stderr)
                    conflicts += 1
                    continue
                print(f"{verb}: {source_file} -> {target}")
                if execute:
                    transfer_file(
                        source_file, target, image_target_name, operation
                    )
                transferred += 1
    return transferred, conflicts


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "destination", type=Path, help="dataset output directory containing the splits"
    )
    parser.add_argument(
        "--source", type=Path, default=Path("/home/thiwa/scan_data"), help="scan root"
    )
    parser.add_argument("--start", help="inclusive earliest scan timestamp")
    parser.add_argument("--end", help="inclusive latest scan timestamp")
    parser.add_argument("--train", type=percentage, default=70.0, help="train percent")
    parser.add_argument(
        "--validate", "--val", dest="validate", type=percentage, default=20.0,
        help="validation percent",
    )
    parser.add_argument("--test", type=percentage, default=10.0, help="test percent")
    parser.add_argument("--seed", type=int, default=42, help="random split seed")
    parser.add_argument(
        "--image-name", default="color.png", help="image filename inside every view"
    )
    parser.add_argument(
        "--operation", choices=("move", "copy"), default="copy",
        help="transfer operation (default: copy)",
    )
    parser.add_argument(
        "--execute", action="store_true", help="perform transfers; otherwise only preview"
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    source = args.source.expanduser().resolve()
    destination = args.destination.expanduser().resolve()

    if not source.is_dir():
        raise SystemExit(f"source directory does not exist: {source}")
    if destination == source or source in destination.parents:
        raise SystemExit("destination must not be the source directory or inside it")
    if Path(args.image_name).name != args.image_name:
        raise SystemExit("--image-name must be a filename, not a path")
    if Path(args.image_name).suffix.lower() not in IMAGE_SUFFIXES:
        raise SystemExit(f"unsupported image extension in --image-name: {args.image_name}")

    total_percentage = args.train + args.validate + args.test
    if abs(total_percentage - 100.0) > 1e-6:
        raise SystemExit(
            f"--train, --validate, and --test must total 100 (got {total_percentage:g})"
        )

    start = parse_time(args.start) if args.start else None
    end = parse_time(args.end, end_of_day=True) if args.end else None
    if start is not None and end is not None and start > end:
        raise SystemExit("--start must be earlier than or equal to --end")

    views = find_views(source, start, end, args.image_name)
    assignments = assign_splits(views, args.train, args.validate, args.test, args.seed)

    print(f"Source:      {source}")
    print(f"Destination: {destination}")
    print(f"Time range:  {start or 'earliest'} through {end or 'latest'}")
    print(f"Images:      {len(views)}")
    for split_name, split_views in assignments.items():
        percent = 100.0 * len(split_views) / len(views) if views else 0.0
        plant_groups = {
            (view.scan_dir.name, view.plant_dir.name) for view in split_views
        }
        print(
            f"  {split_name:<8} {len(split_views):>6} images "
            f"({percent:5.1f}%), {len(plant_groups):>4} plants"
        )
    print(f"Mode:        {args.operation}{'' if args.execute else ' (DRY RUN)'}")

    transferred, conflicts = transfer(
        assignments, destination, args.operation, args.execute
    )
    action = "Transferred" if args.execute else "Would transfer"
    print(f"{action} {transferred} file(s); conflicts: {conflicts}")
    if not args.execute:
        print("Dry run only. Add --execute to perform these operations.")
    return 1 if conflicts else 0


if __name__ == "__main__":
    raise SystemExit(main())
