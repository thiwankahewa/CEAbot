#!/usr/bin/env python3

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    parser = argparse.ArgumentParser(
        description="Validate a saved top-scan folder containing color.png, depth.npy, and metadata.yaml."
    )
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()

    folder = args.folder.expanduser()
    color_path = folder / "color.png"
    depth_path = folder / "depth.npy"
    metadata_path = folder / "metadata.yaml"

    require(folder.exists(), f"Folder does not exist: {folder}")
    require(color_path.exists(), f"Missing {color_path}")
    require(depth_path.exists(), f"Missing {depth_path}")
    require(metadata_path.exists(), f"Missing {metadata_path}")

    color = cv2.imread(str(color_path), cv2.IMREAD_COLOR)
    require(color is not None, f"Could not read {color_path}")

    depth = np.load(depth_path)
    require(depth.size > 0, f"Depth array is empty: {depth_path}")

    with open(metadata_path, "r") as f:
        metadata = yaml.safe_load(f) or {}

    camera_info = metadata.get("camera_info", {})
    rgb_info = camera_info.get("rgb")
    require(rgb_info is not None, "metadata.yaml missing camera_info.rgb")
    require(rgb_info.get("K") is not None, "metadata.yaml missing camera_info.rgb.K")
    require(len(rgb_info.get("K", [])) == 9, "camera_info.rgb.K must have 9 values")

    frames = metadata.get("frames", {})
    images = metadata.get("images", {})

    print(f"folder: {folder}")
    print(f"camera: {metadata.get('camera')}")
    print(f"location: {metadata.get('location')}")
    print(f"color shape: {color.shape}, dtype: {color.dtype}")
    print(f"depth shape: {depth.shape}, dtype: {depth.dtype}, min/max: {np.nanmin(depth):.3f}/{np.nanmax(depth):.3f}")
    print(f"color frame: {frames.get('color_frame_id')}")
    print(f"depth frame: {frames.get('depth_frame_id')}")
    print(f"rgb camera_info frame: {rgb_info.get('frame_id')}")
    print(f"color encoding: {images.get('color_encoding')}")
    print(f"depth encoding: {images.get('depth_encoding')}")
    print("validation: OK")


if __name__ == "__main__":
    main()
