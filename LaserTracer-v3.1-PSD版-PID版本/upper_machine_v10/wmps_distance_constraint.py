from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from pseudo_intersection import euler_to_rotation


STAGE_GEOMETRY_PARAMS_PATH = (
    Path(__file__).resolve().with_name("stage123_nonorth_axis_params1.json")
)


def load_femtosecond_origin(stage_params_path=STAGE_GEOMETRY_PARAMS_PATH):
    """Load the single calibrated ranging origin used by this application."""
    path = Path(stage_params_path)
    with path.open("r", encoding="utf-8") as stream:
        params = json.load(stream)
    try:
        origin = np.asarray(params["origin_global_tracker"], dtype=float).reshape(-1)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"Invalid origin_global_tracker in {path}") from exc
    if origin.size != 3 or not np.all(np.isfinite(origin)):
        raise ValueError(f"Invalid origin_global_tracker in {path}: {origin}")
    return origin


# This is the omnidirectional femtosecond ranging origin expressed in the
# wMPS/global tracker frame.  Loading it from the stage geometry file prevents
# the ranging, motor and visualization paths from silently using different
# copies of the calibration.
FEMTOSECOND_DISTANCE_ORIGIN_MM = load_femtosecond_origin()


def _as_xyz(value, label):
    xyz = np.asarray(value, dtype=float).reshape(-1)
    if xyz.size < 3:
        raise ValueError(f"{label} must contain X, Y, Z")
    xyz = xyz[:3]
    if not np.all(np.isfinite(xyz)):
        raise ValueError(f"Invalid {label}: {xyz}")
    return xyz


def wmps_global_to_femtosecond(wmps_xyz, origin_mm=None):
    """Translate a wMPS-global point into the femtosecond-origin frame.

    The calibration defines only a new origin, not a second set of axis
    directions.  Consequently the local axes remain parallel to the wMPS
    global tracker axes.
    """
    point = _as_xyz(wmps_xyz, "wMPS coordinate")
    if origin_mm is None:
        origin_mm = FEMTOSECOND_DISTANCE_ORIGIN_MM
    origin = _as_xyz(origin_mm, "measurement origin")
    return point - origin


def femtosecond_to_wmps_global(local_xyz, origin_mm=None):
    """Translate a femtosecond-origin coordinate back to wMPS global."""
    point = _as_xyz(local_xyz, "femtosecond coordinate")
    if origin_mm is None:
        origin_mm = FEMTOSECOND_DISTANCE_ORIGIN_MM
    origin = _as_xyz(origin_mm, "measurement origin")
    return point + origin


def constrain_point_to_distance_sphere(wmps_xyz, distance_mm, origin_mm=None):
    """Project a wMPS point onto the laser-distance sphere in global space.

    The wMPS coordinate supplies the direction from the measurement origin.
    The femtosecond ranging result supplies the radius.
    """
    point = _as_xyz(wmps_xyz, "wMPS coordinate")
    if origin_mm is None:
        origin_mm = FEMTOSECOND_DISTANCE_ORIGIN_MM
    origin = _as_xyz(origin_mm, "measurement origin")
    distance = float(distance_mm)

    if not np.isfinite(distance) or distance <= 0.0:
        raise ValueError("Femtosecond distance must be a positive finite value")

    direction = point - origin
    norm = float(np.linalg.norm(direction))
    if norm < 1e-9:
        raise ValueError("wMPS coordinate is too close to the measurement origin")

    return origin + distance * direction / norm


def solve_coordinate_in_femtosecond_frame(wmps_corner_xyz, distance_mm, origin_mm=None):
    """Fuse a four-marker corner-cube estimate with the femtosecond range.

    ``wmps_corner_xyz`` is the corner-cube coordinate obtained from all four
    cooperative-target receivers.  The result is expressed with the
    omnidirectional femtosecond ranging origin at ``(0, 0, 0)``.
    """
    constrained_global = constrain_point_to_distance_sphere(
        wmps_corner_xyz,
        distance_mm,
        origin_mm,
    )
    return wmps_global_to_femtosecond(constrained_global, origin_mm)


def transmitter_origin_from_extrinsic(rotation, translation):
    """Return a wMPS transmitter origin in the global tracker frame.

    ``communication.rawResolveThread`` uses ``p_station = R @ p_global + t``.
    Hence the station origin is the point satisfying ``R @ p_global + t = 0``:
    ``p_global = -R.T @ t``.
    """
    rotation = np.asarray(rotation, dtype=float)
    translation = _as_xyz(translation, "external translation")
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError("External rotation must be a finite 3x3 matrix")
    return -rotation.T @ translation


def transmitter_origins_from_extpara(extpara):
    """Convert ``rawResolveThread.m_extPara`` into global station origins."""
    positions = {}
    for transmitter_id, (rotation, translation) in extpara.items():
        positions[int(transmitter_id)] = transmitter_origin_from_extrinsic(
            rotation,
            translation,
        )
    return positions


def load_transmitter_origins_from_extpara_file(file_path):
    """Read a seven-column wMPS external-parameter file for 3D display."""
    positions = {}
    path = Path(file_path)
    with path.open("r", encoding="utf-8", errors="ignore") as stream:
        for line_number, line in enumerate(stream, start=1):
            fields = line.split()
            if not fields:
                continue
            if len(fields) != 7:
                raise ValueError(
                    f"Invalid external-parameter row {line_number} in {path}: "
                    "expected 7 columns"
                )
            try:
                transmitter_id = int(fields[0])
                values = np.asarray(fields[1:7], dtype=float)
            except ValueError as exc:
                raise ValueError(
                    f"Invalid external-parameter row {line_number} in {path}"
                ) from exc
            rotation = euler_to_rotation(values[0], values[1], values[2])
            positions[transmitter_id] = transmitter_origin_from_extrinsic(
                rotation,
                values[3:6],
            )
    if not positions:
        raise ValueError(f"No transmitter external parameters found in {path}")
    return positions


def simulate_distance_constraint():
    """Deterministic smoke simulation for the spherical constraint method."""
    origin = FEMTOSECOND_DISTANCE_ORIGIN_MM
    true_point = np.array([420.0, -760.0, 1280.0], dtype=float)
    distance = float(np.linalg.norm(true_point - origin))

    # A synthetic wMPS reading with radial and small angular error.
    wmps_point = true_point + np.array([18.0, -7.0, 12.0], dtype=float)
    fused = constrain_point_to_distance_sphere(wmps_point, distance, origin)
    fused_local = wmps_global_to_femtosecond(fused, origin)

    radius_error = abs(float(np.linalg.norm(fused_local)) - distance)
    return {
        "origin": origin,
        "true_point": true_point,
        "wmps_point": wmps_point,
        "distance_mm": distance,
        "fused_point": fused,
        "fused_local": fused_local,
        "radius_error_mm": radius_error,
        "wmps_error_mm": float(np.linalg.norm(wmps_point - true_point)),
        "fused_error_mm": float(np.linalg.norm(fused - true_point)),
    }


if __name__ == "__main__":
    result = simulate_distance_constraint()
    fused = result["fused_point"]
    local = result["fused_local"]
    print(
        "fused_global = "
        f"X={fused[0]:.6f}, Y={fused[1]:.6f}, Z={fused[2]:.6f}; "
        "fused_local = "
        f"X={local[0]:.6f}, Y={local[1]:.6f}, Z={local[2]:.6f}; "
        f"distance={result['distance_mm']:.6f} mm; "
        f"radius_error={result['radius_error_mm']:.9f} mm; "
        f"wmps_error={result['wmps_error_mm']:.6f} mm; "
        f"fused_error={result['fused_error_mm']:.6f} mm"
    )
