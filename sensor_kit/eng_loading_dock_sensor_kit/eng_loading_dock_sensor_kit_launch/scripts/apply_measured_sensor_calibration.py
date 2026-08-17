#!/usr/bin/env python3
"""Apply measured Engineering Loading Dock sensor lever arms to calibration YAML.

Inputs are measured in the vehicle base_link frame:
  x forward, y left, z up, yaw positive counter-clockwise about +z.

The eng sensor kit keeps sensor_kit_base_link coincident with base_link for
bag replay, so measured base_link values can be written directly into the
sensor_kit_base_link child entries.
"""

from __future__ import annotations

import argparse
import datetime as dt
import math
import shutil
from pathlib import Path
from typing import Any

import yaml


FRAME_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")


def _float_arg(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise argparse.ArgumentTypeError(f"{value!r} is not finite")
    return parsed


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise RuntimeError(f"{path} did not parse as a YAML mapping")
    return data


def _backup(path: Path) -> Path:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = path.with_suffix(path.suffix + f".bak_{stamp}")
    shutil.copy2(path, backup)
    return backup


def _ensure_frame(data: dict[str, Any], parent: str, child: str) -> dict[str, Any]:
    parent_map = data.setdefault(parent, {})
    if not isinstance(parent_map, dict):
        raise RuntimeError(f"{parent} is not a mapping")
    frame = parent_map.setdefault(child, {})
    if not isinstance(frame, dict):
        raise RuntimeError(f"{parent}.{child} is not a mapping")
    for key in FRAME_KEYS:
        frame.setdefault(key, 0.0)
    return frame


def _set_frame(
    frame: dict[str, Any],
    *,
    x: float,
    y: float,
    z: float,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
) -> None:
    frame.update({"x": x, "y": y, "z": z, "roll": roll, "pitch": pitch, "yaw": yaw})


def _write_yaml(path: Path, data: dict[str, Any], header: str) -> None:
    with path.open("w", encoding="utf-8") as stream:
        stream.write(header)
        yaml.safe_dump(data, stream, sort_keys=False, default_flow_style=False)


def _default_description_root() -> Path:
    kit_root = Path(__file__).resolve().parents[2]
    return kit_root / "eng_loading_dock_sensor_kit_description"


def _lidar_yaw_rad(args: argparse.Namespace) -> float:
    if args.lidar_yaw_rad is not None and args.lidar_yaw_deg is not None:
        raise RuntimeError("Use either --lidar-yaw-rad or --lidar-yaw-deg, not both")
    if args.lidar_yaw_rad is not None:
        return args.lidar_yaw_rad
    if args.lidar_yaw_deg is not None:
        return math.radians(args.lidar_yaw_deg)
    return 0.0


def _format_frame(name: str, frame: dict[str, Any]) -> str:
    return (
        f"{name}: x={frame['x']:.3f} y={frame['y']:.3f} z={frame['z']:.3f} "
        f"roll={frame['roll']:.4f} pitch={frame['pitch']:.4f} yaw={frame['yaw']:.4f}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Apply measured base_link sensor calibration for the eng loading dock kit."
    )
    parser.add_argument("--description-root", type=Path, default=_default_description_root())
    parser.add_argument("--lidar-x", type=_float_arg, required=True)
    parser.add_argument("--lidar-y", type=_float_arg, required=True)
    parser.add_argument("--lidar-z", type=_float_arg, required=True)
    parser.add_argument("--lidar-yaw-rad", type=_float_arg)
    parser.add_argument("--lidar-yaw-deg", type=_float_arg)
    parser.add_argument("--gnss-x", type=_float_arg, required=True)
    parser.add_argument("--gnss-y", type=_float_arg, required=True)
    parser.add_argument("--gnss-z", type=_float_arg, required=True)
    parser.add_argument(
        "--apply",
        action="store_true",
        help="write YAML files. Without this flag, only prints the planned change.",
    )
    args = parser.parse_args()

    desc = args.description_root.resolve()
    sensors_path = desc / "config" / "sensors_calibration.yaml"
    sensor_kit_path = desc / "config" / "sensor_kit_calibration.yaml"

    sensors = _load_yaml(sensors_path)
    sensor_kit = _load_yaml(sensor_kit_path)

    # Keep sensor_kit_base_link identical to base_link for replay. Measured
    # base_link sensor coordinates are then represented directly as child TFs.
    kit_frame = _ensure_frame(sensors, "base_link", "sensor_kit_base_link")
    _set_frame(kit_frame, x=0.0, y=0.0, z=0.0)

    lidar_frame = _ensure_frame(sensor_kit, "sensor_kit_base_link", "lidar_vlp16_points_link")
    _set_frame(
        lidar_frame,
        x=args.lidar_x,
        y=args.lidar_y,
        z=args.lidar_z,
        yaw=_lidar_yaw_rad(args),
    )

    gnss_frame = _ensure_frame(sensor_kit, "sensor_kit_base_link", "gnss_oem7_link")
    _set_frame(gnss_frame, x=args.gnss_x, y=args.gnss_y, z=args.gnss_z)

    print("Planned calibration:")
    print(_format_frame("base_link -> sensor_kit_base_link", kit_frame))
    print(_format_frame("base_link -> lidar_vlp16_points_link", lidar_frame))
    print(_format_frame("base_link -> gnss_oem7_link", gnss_frame))

    if not args.apply:
        print("Dry run only. Re-run with --apply to write YAML backups and updates.")
        return

    sensors_backup = _backup(sensors_path)
    sensor_kit_backup = _backup(sensor_kit_path)

    generated_header = (
        "# Generated by apply_measured_sensor_calibration.py.\n"
        "# Values are measured in base_link unless noted. Replace only with measured calibration.\n"
        "# Units: x/y/z in meters; roll/pitch/yaw in radians.\n"
    )
    _write_yaml(sensors_path, sensors, generated_header)
    _write_yaml(sensor_kit_path, sensor_kit, generated_header)

    print(f"Wrote {sensors_path}")
    print(f"Wrote {sensor_kit_path}")
    print(f"Backups: {sensors_backup}, {sensor_kit_backup}")


if __name__ == "__main__":
    main()
