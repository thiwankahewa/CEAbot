#!/usr/bin/env python3
"""Move or copy time-filtered plant-view images into dataset splits.

Camera selection:
- ``--camera end-effector`` (default): scan/plant_*/{view_*,top}/color.png.
- ``--camera top``: scan/color.png only (raw full-row RGB, not crop.png).
  Top-camera samples are grouped by location/row across dates, named
  ``SCAN__row__top_camera.png``, and always save/reuse plant_splits.json.
  Keep camera datasets separate; end-effector per-plant assignments cannot
  be reused for a full-row image containing multiple plants.

The scan timestamp is read from a scan directory name such as
``b1_r11_20260806_145201``. Each location/plant pair (for example,
``b1_r11/plant_01``) is assigned as one group to train, validate, or test,
keeping all ``view_*`` and ``top`` images across dates together. This assumes
plant IDs at a location remain stable throughout the selected date range.
Percentages apply to plant groups; image percentages can differ when plants
have different numbers of captures. Images are saved directly in each split
with scan date/time, location, plant, and view in their filenames. A
matching ``color.json`` annotation is transferred with the image when present.

Use ``--row-start 16 --row-end 21`` to include only rows r16 through r21
(inclusive, across all bed prefixes). Omitted row bounds are unrestricted.
Date-only bounds include the entire day, using the scan folder timestamp
(not the UTC capture timestamp in meta.yaml).
Use ``--reuse-splits EXISTING_DATASET`` to preserve earlier plant assignments
when preparing later scans. This records ``plant_splits.json`` in the output;
subsequent runs reuse it automatically. Percentages are targets for the total
plant population, so preserving existing assignments can prevent exact ratios.
The command copies by default and is a dry run unless ``--execute`` is supplied.
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
    plant_dir: Path | None
    view_dir: Path
    image: Path

    @property
    def plant_key(self) -> tuple[str, str]:
        """Identify a plant or whole row across scan dates."""
        location = self.scan_dir.name.rsplit("_", 2)[0]
        return location, self.plant_dir.name if self.plant_dir is not None else "row"

    @property
    def dataset_stem(self) -> str:
        """Return a unique, traceable name for the flattened dataset image."""
        if self.plant_dir is None:
            return f"{self.scan_dir.name}__row__top_camera"
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


def row_number(value: str) -> int:
    """Accept a positive row number as either 16 or r16."""
    text = value.strip().lower().removeprefix("r")
    if not text.isascii() or not text.isdigit() or int(text) < 1:
        raise argparse.ArgumentTypeError("row must be a positive integer, e.g. 16 or r16")
    return int(text)


def scan_row(scan_dir: Path) -> int | None:
    """Read the rNN location component before the scan date and time."""
    token = scan_dir.name.rsplit("_", 2)[0].rsplit("_", 1)[-1]
    if not token.lower().startswith("r"):
        return None
    try:
        return row_number(token)
    except argparse.ArgumentTypeError:
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
    row_start: int | None = None,
    row_end: int | None = None,
    camera: str = "end-effector",
) -> list[PlantView]:
    if camera not in ("end-effector", "top"):
        raise ValueError(f"unsupported camera: {camera}")
    views: list[PlantView] = []
    for scan_dir in sorted(path for path in source.iterdir() if path.is_dir()):
        timestamp = scan_timestamp(scan_dir)
        if timestamp is None:
            continue
        if start is not None and timestamp < start:
            continue
        if end is not None and timestamp > end:
            continue

        if row_start is not None or row_end is not None:
            row = scan_row(scan_dir)
            if row is None:
                continue
            if row_start is not None and row < row_start:
                continue
            if row_end is not None and row > row_end:
                continue

        if camera == "top":
            image = scan_dir / image_name
            if image.is_file():
                views.append(PlantView(scan_dir, None, scan_dir, image))
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
    views: list[PlantView], train: float, validate: float, test: float, seed: int,
    fixed_assignments: dict[str, str] | None = None,
) -> dict[str, list[PlantView]]:
    """Keep a location's plant together across all views and scan dates."""
    grouped: dict[tuple[str, str], list[PlantView]] = {}
    for view in views:
        grouped.setdefault(view.plant_key, []).append(view)

    if fixed_assignments is not None:
        # Retain historical plants even when they have no new images.
        mapping = dict(fixed_assignments)
        names = ("train", "validate", "test")
        new_keys = sorted("/".join(key) for key in grouped if "/".join(key) not in mapping)
        random.Random(seed).shuffle(new_keys)
        targets = split_counts(len(mapping) + len(new_keys), [train, validate, test])
        counts = [sum(value == name for value in mapping.values()) for name in names]
        for key in new_keys:
            index = max(range(3), key=lambda i: targets[i] - counts[i])
            mapping[key] = names[index]
            counts[index] += 1
        assignments = {name: [] for name in names}
        for key, items in grouped.items():
            assignments[mapping["/".join(key)]].extend(items)
        return assignments

    shuffled_groups = [grouped[key] for key in sorted(grouped)]
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


def load_split_assignments(dataset: Path, camera: str = "end-effector") -> dict[str, str]:
    """Read saved identities or infer them from our flattened image filenames."""
    mapping: dict[str, str] = {}

    def remember(key: str, split: str) -> None:
        split = "validate" if split == "val" else split
        if split not in ("train", "validate", "test") or "/" not in key:
            raise ValueError(f"invalid split assignment in {dataset}: {key!r}: {split!r}")
        identity = key.rsplit("/", 1)[-1]
        if (camera == "top" and identity != "row") or (
            camera == "end-effector" and not identity.startswith("plant_")
        ):
            raise ValueError(f"camera mode {camera!r} does not match assignments in {dataset}; use a separate dataset")
        if key in mapping and mapping[key] != split:
            raise ValueError(f"plant appears in different splits: {key} in {dataset}")
        mapping[key] = split

    manifest = dataset / "plant_splits.json"
    if manifest.is_file():
        saved = json.loads(manifest.read_text(encoding="utf-8"))
        if not isinstance(saved, dict):
            raise ValueError(f"expected a plant-to-split mapping: {manifest}")
        for key, split in saved.items():
            remember(key, split)

    image_root = dataset / "images" if (dataset / "images").is_dir() else dataset
    for split in ("train", "validate", "val", "test"):
        for image in sorted((image_root / split).glob("*")):
            if not image.is_file() or image.suffix.lower() not in IMAGE_SUFFIXES:
                continue
            parts = image.stem.split("__")
            if (len(parts) != 3 or scan_timestamp(Path(parts[0])) is None
                    or not (parts[1].startswith("plant_") or parts[1:] == ["row", "top_camera"])):
                raise ValueError(f"cannot infer plant identity from filename: {image}")
            remember(f"{parts[0].rsplit('_', 2)[0]}/{parts[1]}", split)
    return mapping


def merge_split_assignments(target: dict[str, str], extra: dict[str, str]) -> None:
    for key, split in extra.items():
        if key in target and target[key] != split:
            raise ValueError(f"conflicting existing split assignments for {key}")
        target[key] = split


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
    parser.add_argument(
        "--camera", choices=("end-effector", "top"), default="end-effector",
        help="end-effector: scan/plant_*/{view_*,top}/color.png; top: scan/color.png, grouped by row across dates",
    )
    parser.add_argument("--start", help="inclusive earliest scan date/time (YYYY-MM-DD)")
    parser.add_argument("--end", help="inclusive latest scan date/time (whole day for YYYY-MM-DD)")
    parser.add_argument("--row-start", type=row_number, help="inclusive minimum row (16 or r16)")
    parser.add_argument("--row-end", type=row_number, help="inclusive maximum row (21 or r21)")
    parser.add_argument("--train", type=percentage, default=70.0, help="train percent of groups (plants or rows)")
    parser.add_argument(
        "--validate", "--val", dest="validate", type=percentage, default=20.0,
        help="validation percent of groups (plants or rows)",
    )
    parser.add_argument("--test", type=percentage, default=10.0, help="test percent of groups (plants or rows)")
    parser.add_argument("--seed", type=int, default=42, help="random split seed")
    parser.add_argument(
        "--reuse-splits", type=Path,
        help="existing dataset from the same camera whose group assignments must be preserved; "
        "new groups fill percentage deficits, and plant_splits.json records assignments",
    )
    parser.add_argument(
        "--image-name", default="color.png", help="image filename inside each view or scan root, depending on --camera"
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

    if args.row_start is not None and args.row_end is not None and args.row_start > args.row_end:
        raise SystemExit("--row-start must be less than or equal to --row-end")

    views = find_views(source, start, end, args.image_name, args.row_start, args.row_end, args.camera)
    fixed = None
    if args.camera == "top" or args.reuse_splits or (destination / "plant_splits.json").is_file():
        fixed = {}
        try:
            if args.reuse_splits:
                previous = args.reuse_splits.expanduser().resolve()
                if not previous.is_dir():
                    raise ValueError(f"previous dataset does not exist: {previous}")
                inherited = load_split_assignments(previous, args.camera)
                if not inherited:
                    raise ValueError(f"no plant split assignments found: {previous}")
                merge_split_assignments(fixed, inherited)
            merge_split_assignments(fixed, load_split_assignments(destination, args.camera))
        except (ValueError, OSError) as exc:
            raise SystemExit(str(exc)) from exc
    assignments = assign_splits(views, args.train, args.validate, args.test, args.seed, fixed)

    print(f"Source:      {source}")
    print(f"Destination: {destination}")
    print(f"Time range:  {start or 'earliest'} through {end or 'latest'}")
    print(f"Row range:   {args.row_start or 'first'} through {args.row_end or 'last'} (inclusive)")
    print(f"Camera:      {args.camera}")
    print(f"Images:      {len(views)}")
    total_plants = len({view.plant_key for view in views})
    group_label = "rows" if args.camera == "top" else "plants"
    print(f"Groups:      {total_plants} {group_label} (grouped across dates)")
    for split_name, split_views in assignments.items():
        percent = 100.0 * len(split_views) / len(views) if views else 0.0
        plant_groups = {view.plant_key for view in split_views}
        plant_percent = 100.0 * len(plant_groups) / total_plants if total_plants else 0.0
        print(
            f"  {split_name:<8} {len(plant_groups):>4} {group_label} ({plant_percent:5.1f}%), "
            f"{len(split_views):>6} images ({percent:5.1f}%)"
        )
    print(f"Mode:        {args.operation}{'' if args.execute else ' (DRY RUN)'}")

    if fixed is not None:
        print(f"Preserving:  {len(fixed)} existing group assignments")
        for split, split_views in assignments.items():
            for view in split_views:
                fixed["/".join(view.plant_key)] = split
        if args.execute:
            destination.mkdir(parents=True, exist_ok=True)
            manifest = destination / "plant_splits.json"
            temporary = manifest.with_suffix(".json.tmp")
            temporary.write_text(json.dumps(fixed, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            temporary.replace(manifest)

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
