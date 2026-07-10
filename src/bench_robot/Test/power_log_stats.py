#!/usr/bin/env python3
"""Print useful summary statistics for CEAbot drive power CSV logs."""

import argparse
import csv
import math
from collections import defaultdict
from datetime import datetime
from pathlib import Path


DEFAULT_CSV = Path("/home/thiwa/CEAbot/power_logs/drive_power_log.csv")
NUMERIC_COLUMNS = (
    "duration_s",
    "sample_count",
    "avg_bus_voltage_v",
    "avg_left_current_a",
    "avg_right_current_a",
    "avg_total_current_a",
    "avg_left_power_w",
    "avg_right_power_w",
    "avg_total_power_w",
    "energy_wh",
)


def parse_float(value):
    if value is None or value == "":
        return None
    try:
        number = float(value)
    except ValueError:
        return None
    return number if math.isfinite(number) else None


def parse_timestamp(value):
    if not value:
        return None
    try:
        return datetime.fromisoformat(value)
    except ValueError:
        return None


def load_rows(csv_path):
    with csv_path.open("r", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        rows = []
        for row_number, row in enumerate(reader, start=2):
            cleaned = {
                "row_number": row_number,
                "timestamp": row.get("timestamp", ""),
                "timestamp_dt": parse_timestamp(row.get("timestamp", "")),
                "auto_state": row.get("auto_state", "unknown") or "unknown",
            }
            for column in NUMERIC_COLUMNS:
                cleaned[column] = parse_float(row.get(column))
            rows.append(cleaned)
    return rows


def collect_values(rows, column):
    return [row[column] for row in rows if row.get(column) is not None]


def mean(values):
    return sum(values) / len(values) if values else None


def percentile(values, percent):
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * percent / 100.0
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[int(index)]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def format_number(value, unit=""):
    if value is None:
        return "n/a"
    return f"{value:.3f}{unit}"


def format_timestamp(value):
    return value.isoformat(sep=" ") if value else "unknown"


def print_column_stats(rows):
    print("\nOverall numeric statistics")
    print("-" * 104)
    print(
        f"{'column':<24} {'count':>7} {'avg':>12} {'min':>12} {'max':>12} "
        f"{'p50':>12} {'p95':>12} {'sum':>12}"
    )
    print("-" * 104)
    for column in NUMERIC_COLUMNS:
        values = collect_values(rows, column)
        total = sum(values) if values else None
        print(
            f"{column:<24} {len(values):>7} "
            f"{format_number(mean(values)):>12} "
            f"{format_number(min(values) if values else None):>12} "
            f"{format_number(max(values) if values else None):>12} "
            f"{format_number(percentile(values, 50)):>12} "
            f"{format_number(percentile(values, 95)):>12} "
            f"{format_number(total):>12}"
        )


def print_state_stats(rows):
    states = defaultdict(list)
    for row in rows:
        states[row["auto_state"]].append(row)

    print("\nStats by auto_state")
    print("-" * 106)
    print(
        f"{'auto_state':<22} {'rows':>6} {'duration_s':>12} {'energy_wh':>12} "
        f"{'avg_power_w':>12} {'max_power_w':>12} {'avg_current_a':>14}"
    )
    print("-" * 106)
    for state, state_rows in sorted(
        states.items(),
        key=lambda item: sum(v for v in collect_values(item[1], "energy_wh")),
        reverse=True,
    ):
        duration = sum(collect_values(state_rows, "duration_s"))
        energy = sum(collect_values(state_rows, "energy_wh"))
        powers = collect_values(state_rows, "avg_total_power_w")
        currents = collect_values(state_rows, "avg_total_current_a")
        print(
            f"{state:<22} {len(state_rows):>6} "
            f"{format_number(duration):>12} "
            f"{format_number(energy):>12} "
            f"{format_number(mean(powers)):>12} "
            f"{format_number(max(powers) if powers else None):>12} "
            f"{format_number(mean(currents)):>14}"
        )


def print_peak_rows(rows, column, title, limit=5):
    ranked = [row for row in rows if row.get(column) is not None]
    ranked.sort(key=lambda row: row[column], reverse=True)
    show_energy = column != "energy_wh"
    print(f"\n{title}")
    print("-" * 112)
    energy_header = f"{'energy_wh':>12}" if show_energy else ""
    print(
        f"{'rank':>4} {'timestamp':<20} {'auto_state':<20} "
        f"{column:>16} {'duration_s':>12} {'avg_power_w':>12} {energy_header}"
    )
    print("-" * 112)
    for rank, row in enumerate(ranked[:limit], start=1):
        energy_value = f"{format_number(row.get('energy_wh')):>12}" if show_energy else ""
        print(
            f"{rank:>4} {row['timestamp']:<20} {row['auto_state']:<20} "
            f"{format_number(row[column]):>16} "
            f"{format_number(row.get('duration_s')):>12} "
            f"{format_number(row.get('avg_total_power_w')):>12} {energy_value}"
        )


def print_summary(rows, csv_path):
    timestamps = [row["timestamp_dt"] for row in rows if row["timestamp_dt"]]
    total_duration = sum(collect_values(rows, "duration_s"))
    total_energy = sum(collect_values(rows, "energy_wh"))
    average_power_from_energy = (
        total_energy * 3600.0 / total_duration if total_duration > 0 else None
    )

    print(f"CSV file: {csv_path}")
    print(f"Rows: {len(rows)}")
    print(f"Time range: {format_timestamp(min(timestamps) if timestamps else None)} to "
          f"{format_timestamp(max(timestamps) if timestamps else None)}")
    print(f"Total duration: {format_number(total_duration, ' s')}")
    print(f"Total energy: {format_number(total_energy, ' Wh')}")
    print(f"Average power from energy/duration: {format_number(average_power_from_energy, ' W')}")

    print_column_stats(rows)
    print_state_stats(rows)
    print_peak_rows(rows, "avg_total_power_w", "Top rows by average total power")
    print_peak_rows(rows, "energy_wh", "Top rows by energy used")


def main():
    parser = argparse.ArgumentParser(
        description="Calculate averages, min/max values, totals, percentiles, and per-state stats for a drive power CSV log."
    )
    parser.add_argument(
        "csv_path",
        nargs="?",
        default=DEFAULT_CSV,
        type=Path,
        help=f"CSV file to analyze. Default: {DEFAULT_CSV}",
    )
    args = parser.parse_args()

    if not args.csv_path.exists():
        raise SystemExit(f"CSV file not found: {args.csv_path}")

    rows = load_rows(args.csv_path)
    if not rows:
        raise SystemExit(f"No data rows found in: {args.csv_path}")

    print_summary(rows, args.csv_path)


if __name__ == "__main__":
    main()
