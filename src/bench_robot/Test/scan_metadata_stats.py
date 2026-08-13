#!/usr/bin/env python3
"""Print reliability and timing statistics from CEAbot row-scan metadata."""

from __future__ import annotations

import argparse
from collections import defaultdict
from datetime import datetime, time
from pathlib import Path
import re
import statistics

import yaml


TIMESTAMP_FORMAT = "%Y%m%d_%H%M%S"
FOLDER_TIMESTAMP_PATTERN = re.compile(r"(\d{8}_\d{6})$")


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as yaml_file:
        return yaml.safe_load(yaml_file) or {}


def parse_timestamp(value: object) -> datetime:
    return datetime.strptime(str(value), TIMESTAMP_FORMAT)


def parse_range_time(value: str, end_of_day: bool = False) -> datetime:
    """Parse scan-style or ISO-style timestamps; an end date includes its day."""
    normalized = value.strip().replace("T", " ")
    formats = (
        TIMESTAMP_FORMAT,
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
        f"invalid date/time {value!r}; use YYYYMMDD_HHMMSS or "
        "YYYY-MM-DD[ HH:MM[:SS]]"
    )


def scan_start_time(scan_dir: Path, metadata: dict) -> datetime | None:
    value = metadata.get("top_scan_timestamp") or metadata.get("row_segmentation_timestamp")
    if value:
        try:
            return parse_timestamp(value)
        except (TypeError, ValueError):
            pass

    match = FOLDER_TIMESTAMP_PATTERN.search(scan_dir.name)
    return parse_timestamp(match.group(1)) if match else None


def duration_seconds(start_value: object, end_value: object) -> float | None:
    if not start_value or not end_value:
        return None
    try:
        duration = (parse_timestamp(end_value) - parse_timestamp(start_value)).total_seconds()
        return duration if duration >= 0 else None
    except (TypeError, ValueError):
        return None


def find_scans(root: Path, start: datetime | None, end: datetime | None) -> tuple[list[dict], list[str]]:
    scans = []
    skipped = []
    for metadata_path in sorted(root.glob("*/metadata.yaml")):
        try:
            metadata = load_yaml(metadata_path)
            timestamp = scan_start_time(metadata_path.parent, metadata)
            if timestamp is None:
                skipped.append(f"{metadata_path.parent.name}: no valid scan timestamp")
                continue
            if start is not None and timestamp < start:
                continue
            if end is not None and timestamp > end:
                continue

            plant_scan = metadata.get("plant_scan")
            if not isinstance(plant_scan, dict):
                skipped.append(f"{metadata_path.parent.name}: missing plant_scan statistics")
                continue

            required = ("requested_poses", "reached_poses", "planning_failed", "execution_failed")
            missing = [field for field in required if field not in plant_scan]
            if missing:
                skipped.append(f"{metadata_path.parent.name}: missing {', '.join(missing)}")
                continue

            scans.append(
                {
                    "path": metadata_path.parent,
                    "timestamp": timestamp,
                    "plants": len(metadata.get("plants") or []),
                    "requested": int(plant_scan["requested_poses"]),
                    "reached": int(plant_scan["reached_poses"]),
                    "planning_failed": int(plant_scan["planning_failed"]),
                    "execution_failed": int(plant_scan["execution_failed"]),
                    "duration": duration_seconds(
                        metadata.get("top_scan_timestamp"), metadata.get("scan_end_time")
                    ),
                }
            )
        except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
            skipped.append(f"{metadata_path.parent.name}: {exc}")
    return scans, skipped


def find_view_durations(scans: list[dict]) -> tuple[list[float], list[str]]:
    durations = []
    skipped = []
    for scan in scans:
        for metadata_path in sorted(scan["path"].glob("plant_*/*/meta.yaml")):
            try:
                metadata = load_yaml(metadata_path)
                duration = duration_seconds(metadata.get("start_time"), metadata.get("end_time"))
                if duration is None:
                    skipped.append(f"{metadata_path}: invalid or missing start/end time")
                else:
                    durations.append(duration)
            except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
                skipped.append(f"{metadata_path}: {exc}")
    return durations, skipped


def format_duration(seconds: float) -> str:
    total = int(round(seconds))
    hours, remainder = divmod(total, 3600)
    minutes, secs = divmod(remainder, 60)
    return f"{hours:d}:{minutes:02d}:{secs:02d} ({seconds:.1f} s)"


def percentage(amount: int | float, denominator: int | float) -> float:
    return 100.0 * amount / denominator if denominator else 0.0


def print_report(root: Path, scans: list[dict], view_durations: list[float], start, end) -> None:
    total_plants = sum(scan["plants"] for scan in scans)
    requested = sum(scan["requested"] for scan in scans)
    reached = sum(scan["reached"] for scan in scans)
    planning_failed = sum(scan["planning_failed"] for scan in scans)
    execution_failed = sum(scan["execution_failed"] for scan in scans)
    known_row_durations = [scan["duration"] for scan in scans if scan["duration"] is not None]

    print("\nCEAbot scan metadata statistics")
    print("=" * 64)
    print(f"Root path:             {root}")
    print(f"Selected start:        {start or 'earliest available'}")
    print(f"Selected end:          {end or 'latest available'}")
    print(f"Plant rows:            {len(scans)}")
    print(f"Plants:                {total_plants}")
    print(f"Average plants/row:    {total_plants / len(scans):.2f}")

    grouped = defaultdict(list)
    for scan in scans:
        grouped[scan["requested"]].append(scan)

    print("\nReached poses grouped by requested poses")
    print("-" * 64)
    print(f"{'Requested':>9} {'Rows':>6} {'Avg reached':>13} {'Reached/total':>16} {'Percent':>9}")
    for group_requested in sorted(grouped):
        group = grouped[group_requested]
        group_reached = sum(scan["reached"] for scan in group)
        group_total_requested = sum(scan["requested"] for scan in group)
        print(
            f"{group_requested:>9} {len(group):>6} "
            f"{statistics.mean(scan['reached'] for scan in group):>13.2f} "
            f"{f'{group_reached}/{group_total_requested}':>16} "
            f"{percentage(group_reached, group_total_requested):>8.2f}%"
        )

    print("\nOverall reliability")
    print("-" * 64)
    print(f"Average reached/row:   {statistics.mean(scan['reached'] for scan in scans):.2f}")
    print(f"Reached poses:         {reached}/{requested} ({percentage(reached, requested):.2f}%)")
    print(f"Planning failures:     {planning_failed}/{requested} ({percentage(planning_failed, requested):.2f}%)")
    print(f"Execution failures:    {execution_failed}/{requested} ({percentage(execution_failed, requested):.2f}%)")

    print("\nRow scan timing grouped by requested poses")
    print("-" * 64)
    print(f"{'Requested':>9} {'Timed rows':>11} {'Average row duration':>23}")
    for group_requested in sorted(grouped):
        durations = [scan["duration"] for scan in grouped[group_requested] if scan["duration"] is not None]
        average = format_duration(statistics.mean(durations)) if durations else "unavailable"
        print(f"{group_requested:>9} {len(durations):>11} {average:>23}")

    print("\nOverall timing")
    print("-" * 64)
    if known_row_durations:
        print(f"Total scan duration:   {format_duration(sum(known_row_durations))}")
        print(f"Average scan/row:      {format_duration(statistics.mean(known_row_durations))}")
        if len(known_row_durations) != len(scans):
            print(f"Rows missing duration: {len(scans) - len(known_row_durations)}")
    else:
        print("Row scan durations:    unavailable")

    print("\nSuccessful view scan timing")
    print("-" * 64)
    print(f"Timed views:           {len(view_durations)}")
    if view_durations:
        print(f"Average:               {statistics.mean(view_durations):.2f} s")
        print(f"Median:                {statistics.median(view_durations):.2f} s")
        print(f"Minimum:               {min(view_durations):.2f} s")
        print(f"Maximum:               {max(view_durations):.2f} s")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Calculate plant-row reliability and timing statistics from scan metadata."
    )
    parser.add_argument("root", type=Path, help="Root containing timestamped row-scan folders")
    parser.add_argument(
        "--start-time",
        help="Inclusive start: YYYYMMDD_HHMMSS or YYYY-MM-DD[ HH:MM[:SS]]",
    )
    parser.add_argument(
        "--end-time",
        help="Inclusive end: YYYYMMDD_HHMMSS or YYYY-MM-DD[ HH:MM[:SS]]",
    )
    args = parser.parse_args()

    root = args.root.expanduser().resolve()
    if not root.is_dir():
        parser.error(f"root directory does not exist: {root}")

    try:
        start = parse_range_time(args.start_time) if args.start_time else None
        end = parse_range_time(args.end_time, end_of_day=True) if args.end_time else None
    except argparse.ArgumentTypeError as exc:
        parser.error(str(exc))
    if start and end and start > end:
        parser.error("--start-time must not be later than --end-time")

    scans, scan_warnings = find_scans(root, start, end)
    if not scans:
        parser.error("no complete row-scan metadata found in the selected time range")
    view_durations, view_warnings = find_view_durations(scans)
    print_report(root, scans, view_durations, start, end)

    warnings = scan_warnings + view_warnings
    if warnings:
        print(f"\nWarnings ({len(warnings)})")
        print("-" * 64)
        for warning in warnings:
            print(f"- {warning}")


if __name__ == "__main__":
    main()
