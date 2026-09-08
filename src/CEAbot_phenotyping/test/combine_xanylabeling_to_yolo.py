#!/usr/bin/env python3
"""Combine split X-AnyLabeling datasets into a YOLO segmentation dataset.

Each input dataset must contain ``train``, ``validate`` (or ``val``), and
``test`` directories with images and matching X-AnyLabeling JSON files. The
output layout is compatible with Ultralytics YOLO segmentation::

    output/
      data.yaml
      images/{train,val,test}/...
      labels/{train,val,test}/...

Only annotated images are included by default. The command is a dry run unless
``--execute`` is supplied.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import shutil
import sys


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".tif", ".tiff"}
INPUT_SPLITS = {
    "train": "train",
    "validate": "val",
    "validation": "val",
    "val": "val",
    "test": "test",
}
OUTPUT_SPLITS = ("train", "val", "test")


@dataclass(frozen=True)
class Sample:
    dataset: Path
    split: str
    image: Path
    annotation: Path | None


def file_digest(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as input_file:
        for chunk in iter(lambda: input_file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def discover_samples(
    datasets: list[Path], include_unlabeled: bool
) -> tuple[list[Sample], list[str]]:
    samples: list[Sample] = []
    warnings: list[str] = []
    seen_names: dict[str, Sample] = {}

    for dataset in datasets:
        for input_name, output_name in INPUT_SPLITS.items():
            split_dir = dataset / input_name
            if not split_dir.is_dir():
                continue
            for image in sorted(
                path for path in split_dir.iterdir()
                if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
            ):
                annotation = image.with_suffix(".json")
                if not annotation.is_file():
                    if not include_unlabeled:
                        warnings.append(f"unlabeled image skipped: {image}")
                        continue
                    annotation = None

                sample = Sample(dataset, output_name, image, annotation)
                previous = seen_names.get(image.name)
                if previous is not None:
                    same_image = file_digest(previous.image) == file_digest(image)
                    same_annotation = (
                        previous.annotation is None and annotation is None
                    ) or (
                        previous.annotation is not None
                        and annotation is not None
                        and file_digest(previous.annotation) == file_digest(annotation)
                    )
                    if same_image and same_annotation and previous.split == output_name:
                        warnings.append(f"identical duplicate skipped: {image}")
                        continue
                    raise ValueError(
                        "conflicting duplicate filename found:\n"
                        f"  {previous.image} ({previous.split})\n"
                        f"  {image} ({output_name})\n"
                        "Use unique filenames or make the duplicate files and split agree."
                    )
                seen_names[image.name] = sample
                samples.append(sample)

    return samples, warnings


def load_annotation(path: Path) -> dict:
    try:
        with path.open("r", encoding="utf-8") as annotation_file:
            data = json.load(annotation_file)
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"could not read annotation {path}: {exc}") from exc
    if not isinstance(data, dict) or not isinstance(data.get("shapes", []), list):
        raise ValueError(f"invalid X-AnyLabeling annotation: {path}")
    return data


def collect_classes(samples: list[Sample]) -> list[str]:
    classes = set()
    for sample in samples:
        if sample.annotation is None:
            continue
        for shape in load_annotation(sample.annotation).get("shapes", []):
            label = str(shape.get("label", "")).strip()
            if label:
                classes.add(label)
    return sorted(classes)


def parse_classes(value: str) -> list[str]:
    classes = [item.strip() for item in value.split(",") if item.strip()]
    if not classes:
        raise argparse.ArgumentTypeError("--classes must contain at least one class")
    if len(set(classes)) != len(classes):
        raise argparse.ArgumentTypeError("--classes contains duplicates")
    return classes


def image_size(annotation: dict, annotation_path: Path) -> tuple[float, float]:
    try:
        width = float(annotation["imageWidth"])
        height = float(annotation["imageHeight"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            f"annotation has no valid imageWidth/imageHeight: {annotation_path}"
        ) from exc
    if width <= 0 or height <= 0:
        raise ValueError(f"invalid image dimensions in {annotation_path}")
    return width, height


def rectangle_points(points: list) -> list[list[float]]:
    if len(points) < 2:
        return []
    xs = [float(point[0]) for point in points]
    ys = [float(point[1]) for point in points]
    left, right = min(xs), max(xs)
    top, bottom = min(ys), max(ys)
    return [[left, top], [right, top], [right, bottom], [left, bottom]]


def yolo_lines(annotation: dict, annotation_path: Path, class_ids: dict[str, int]) -> list[str]:
    width, height = image_size(annotation, annotation_path)
    lines: list[str] = []
    for index, shape in enumerate(annotation.get("shapes", []), start=1):
        label = str(shape.get("label", "")).strip()
        if label not in class_ids:
            raise ValueError(
                f"unknown or empty class {label!r} in {annotation_path}, shape {index}"
            )
        shape_type = str(shape.get("shape_type", "polygon")).lower()
        points = shape.get("points", [])
        if shape_type == "rectangle":
            points = rectangle_points(points)
        elif shape_type != "polygon":
            raise ValueError(
                f"unsupported shape type {shape_type!r} in {annotation_path}, shape {index}; "
                "YOLO segmentation requires polygon or rectangle shapes"
            )
        if len(points) < 3:
            raise ValueError(f"fewer than 3 points in {annotation_path}, shape {index}")

        coordinates: list[str] = []
        for point in points:
            try:
                x = min(1.0, max(0.0, float(point[0]) / width))
                y = min(1.0, max(0.0, float(point[1]) / height))
            except (IndexError, TypeError, ValueError) as exc:
                raise ValueError(
                    f"invalid point in {annotation_path}, shape {index}: {point!r}"
                ) from exc
            coordinates.extend((f"{x:.6f}", f"{y:.6f}"))
        lines.append(f"{class_ids[label]} {' '.join(coordinates)}")
    return lines


def yaml_text(output: Path, classes: list[str]) -> str:
    lines = [
        f"path: {json.dumps(str(output))}",
        "train: images/train",
        "val: images/val",
        "test: images/test",
        "names:",
    ]
    lines.extend(f"  {index}: {json.dumps(name)}" for index, name in enumerate(classes))
    return "\n".join(lines) + "\n"


def prepare_conversion(samples: list[Sample], classes: list[str]) -> dict[Sample, list[str]]:
    class_ids = {name: index for index, name in enumerate(classes)}
    converted: dict[Sample, list[str]] = {}
    for sample in samples:
        if sample.annotation is None:
            converted[sample] = []
        else:
            annotation = load_annotation(sample.annotation)
            converted[sample] = yolo_lines(annotation, sample.annotation, class_ids)
    return converted


def write_dataset(
    converted: dict[Sample, list[str]], output: Path, classes: list[str], execute: bool
) -> None:
    for split in OUTPUT_SPLITS:
        image_dir = output / "images" / split
        label_dir = output / "labels" / split
        if execute:
            image_dir.mkdir(parents=True, exist_ok=True)
            label_dir.mkdir(parents=True, exist_ok=True)

    for sample, lines in converted.items():
        target_image = output / "images" / sample.split / sample.image.name
        target_label = output / "labels" / sample.split / f"{sample.image.stem}.txt"
        print(f"COPY: {sample.image} -> {target_image}")
        if execute:
            shutil.copy2(sample.image, target_image)
            with target_label.open("x", encoding="utf-8") as label_file:
                if lines:
                    label_file.write("\n".join(lines) + "\n")

    if execute:
        (output / "data.yaml").write_text(yaml_text(output, classes), encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="new YOLO dataset directory")
    parser.add_argument(
        "datasets", type=Path, nargs="+", help="one or more split X-AnyLabeling datasets"
    )
    parser.add_argument(
        "--classes", type=parse_classes,
        help="comma-separated class order; default: infer and sort all labels",
    )
    parser.add_argument(
        "--include-unlabeled", action="store_true",
        help="include images without JSON as intentional negative/background samples",
    )
    parser.add_argument(
        "--execute", action="store_true", help="create the output; otherwise preview only"
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    datasets = [path.expanduser().resolve() for path in args.datasets]
    output = args.output.expanduser().resolve()

    for dataset in datasets:
        if not dataset.is_dir():
            raise SystemExit(f"input dataset does not exist: {dataset}")
    if len(set(datasets)) != len(datasets):
        raise SystemExit("the same input dataset was provided more than once")
    if output in datasets or any(dataset in output.parents for dataset in datasets):
        raise SystemExit("output must not be an input dataset or inside one")
    if output.exists() and any(output.iterdir()):
        raise SystemExit(f"output directory is not empty: {output}")

    try:
        samples, warnings = discover_samples(datasets, args.include_unlabeled)
        classes = args.classes or collect_classes(samples)
        if not classes:
            raise ValueError("no annotation classes found")
        converted = prepare_conversion(samples, classes)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    counts = {split: 0 for split in OUTPUT_SPLITS}
    instance_counts = {split: 0 for split in OUTPUT_SPLITS}
    for sample, lines in converted.items():
        counts[sample.split] += 1
        instance_counts[sample.split] += len(lines)

    print(f"Inputs:  {', '.join(str(path) for path in datasets)}")
    print(f"Output:  {output}")
    print(f"Classes: {', '.join(f'{i}={name}' for i, name in enumerate(classes))}")
    for split in OUTPUT_SPLITS:
        print(
            f"  {split:<5} {counts[split]:>6} images, "
            f"{instance_counts[split]:>6} instances"
        )
    print(f"Warnings: {len(warnings)}")
    for warning in warnings[:20]:
        print(f"WARNING: {warning}", file=sys.stderr)
    if len(warnings) > 20:
        print(f"WARNING: {len(warnings) - 20} additional warnings omitted", file=sys.stderr)

    write_dataset(converted, output, classes, args.execute)
    if args.execute:
        print(f"Created YOLO segmentation dataset: {output}")
    else:
        print("Dry run only. Add --execute to create the dataset.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
