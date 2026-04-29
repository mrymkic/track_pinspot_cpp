#!/usr/bin/env python3
"""Estimate aux-depth to base-depth extrinsics from paired checkerboard images.

This script detects a planar checkerboard in synchronized base/aux color images,
estimates the checkerboard pose in each color camera with solvePnP, converts each
pose into the device's depth-camera coordinate system, and then solves for the
rig transform expected by `track_test_2.cpp`:

    p_base_depth = aux_translation_mm + rotation_matrix * p_aux_depth

Input requirements
------------------
1. Two directories containing paired checkerboard images.
   Files are paired by stem, e.g.:

       base/0001.png  <->  aux/0001.png
       base/0002.png  <->  aux/0002.png

2. A rig calibration JSON describing:
   - each device's color intrinsics
   - either color->depth or depth->color extrinsics

Example rig JSON:

{
  "base": {
    "color_camera_matrix": [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
    "color_distortion_coeffs": [k1, k2, p1, p2, k3, k4, k5, k6],
    "color_to_depth_rotation": [[...], [...], [...]],
    "color_to_depth_translation_mm": [tx, ty, tz]
  },
  "aux": {
    "color_camera_matrix": [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
    "color_distortion_coeffs": [k1, k2, p1, p2, k3, k4, k5, k6],
    "color_to_depth_rotation": [[...], [...], [...]],
    "color_to_depth_translation_mm": [tx, ty, tz]
  }
}

If you only have depth->color extrinsics, the script can also consume:
  - depth_to_color_rotation
  - depth_to_color_translation_mm

and will invert them internally.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

np = None
cv2 = None


IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


def _import_dependencies() -> None:
    global np
    global cv2

    if np is not None and cv2 is not None:
        return

    try:
        import numpy as imported_numpy
        import cv2 as imported_cv2
    except ImportError as exc:  # pragma: no cover - dependency check
        raise SystemExit(
            "This script requires `numpy` and `opencv-python`. "
            "Install them before running checkerboard calibration."
        ) from exc

    np = imported_numpy
    cv2 = imported_cv2


@dataclass
class CameraCalibration:
    name: str
    color_camera_matrix: np.ndarray
    color_distortion_coeffs: np.ndarray
    color_to_depth_rotation: np.ndarray
    color_to_depth_translation_mm: np.ndarray


@dataclass
class PairEstimate:
    stem: str
    base_image: str
    aux_image: str
    base_reprojection_error_px: float
    aux_reprojection_error_px: float
    rotation_aux_to_base: np.ndarray
    translation_aux_to_base_mm: np.ndarray


@dataclass
class CalibrationRunSettings:
    base_dir: Path
    aux_dir: Path
    rig_calibration: Path
    board_cols: int
    board_rows: int
    square_size_mm: float
    output_json: Path | None
    config_in: Path | None
    config_out: Path | None


def _as_float_array(data: Any, shape: tuple[int, ...], label: str) -> np.ndarray:
    array = np.asarray(data, dtype=np.float64)
    if array.shape != shape:
        raise ValueError(f"{label} must have shape {shape}, got {array.shape}.")
    return array


def _load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _load_object_json(path: Path, label: str) -> dict[str, Any]:
    payload = _load_json(path)
    if not isinstance(payload, dict):
        raise ValueError(f"{label} must be a JSON object.")
    return payload


def _load_camera_calibration(payload: dict[str, Any], label: str) -> CameraCalibration:
    camera_matrix = _as_float_array(
        payload.get("color_camera_matrix", payload.get("camera_matrix")),
        (3, 3),
        f"{label}.color_camera_matrix",
    )
    distortion = np.asarray(
        payload.get("color_distortion_coeffs", payload.get("distortion_coeffs", payload.get("distortion_coefficients"))),
        dtype=np.float64,
    ).reshape(-1)
    if distortion.size == 0:
        raise ValueError(f"{label}.color_distortion_coeffs is required.")

    if "color_to_depth_rotation" in payload and "color_to_depth_translation_mm" in payload:
        color_to_depth_rotation = _as_float_array(
            payload["color_to_depth_rotation"],
            (3, 3),
            f"{label}.color_to_depth_rotation",
        )
        color_to_depth_translation_mm = _as_float_array(
            payload["color_to_depth_translation_mm"],
            (3,),
            f"{label}.color_to_depth_translation_mm",
        )
    elif "depth_to_color_rotation" in payload and "depth_to_color_translation_mm" in payload:
        depth_to_color_rotation = _as_float_array(
            payload["depth_to_color_rotation"],
            (3, 3),
            f"{label}.depth_to_color_rotation",
        )
        depth_to_color_translation_mm = _as_float_array(
            payload["depth_to_color_translation_mm"],
            (3,),
            f"{label}.depth_to_color_translation_mm",
        )
        color_to_depth_rotation = depth_to_color_rotation.T
        color_to_depth_translation_mm = -color_to_depth_rotation @ depth_to_color_translation_mm
    else:
        raise ValueError(
            f"{label} must define either color_to_depth_* or depth_to_color_* extrinsics."
        )

    return CameraCalibration(
        name=label,
        color_camera_matrix=camera_matrix,
        color_distortion_coeffs=distortion,
        color_to_depth_rotation=color_to_depth_rotation,
        color_to_depth_translation_mm=color_to_depth_translation_mm,
    )


def _load_rig_calibration(path: Path) -> tuple[CameraCalibration, CameraCalibration]:
    payload = _load_object_json(path, "Rig calibration JSON")
    if "base" not in payload or "aux" not in payload:
        raise ValueError("Rig calibration JSON must contain top-level `base` and `aux` objects.")
    return (
        _load_camera_calibration(payload["base"], "base"),
        _load_camera_calibration(payload["aux"], "aux"),
    )


def _build_image_index(directory: Path) -> dict[str, Path]:
    if not directory.is_dir():
        raise ValueError(f"Directory does not exist: {directory}")

    index: dict[str, Path] = {}
    for path in sorted(directory.iterdir()):
        if not path.is_file() or path.suffix.lower() not in IMAGE_EXTENSIONS:
            continue
        if path.stem in index:
            raise ValueError(
                f"Duplicate image stem `{path.stem}` in {directory}. "
                "Use unique stems so base/aux images can be paired safely."
            )
        index[path.stem] = path
    return index


def _make_board_object_points(board_cols: int, board_rows: int, square_size_mm: float) -> np.ndarray:
    object_points = np.zeros((board_rows * board_cols, 3), dtype=np.float32)
    grid_x, grid_y = np.meshgrid(np.arange(board_cols), np.arange(board_rows))
    object_points[:, 0] = grid_x.reshape(-1) * square_size_mm
    object_points[:, 1] = grid_y.reshape(-1) * square_size_mm
    return object_points


def _read_grayscale_image(path: Path) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"Failed to read image: {path}")
    return image


def _detect_checkerboard(gray: np.ndarray, pattern_size: tuple[int, int]) -> np.ndarray | None:
    sb_flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    found, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags=sb_flags)
    if found and corners is not None:
        return corners.reshape(-1, 2).astype(np.float32)

    standard_flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        | cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_FAST_CHECK
    )
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags=standard_flags)
    if not found or corners is None:
        return None

    termination = (
        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
        50,
        1e-4,
    )
    cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), termination)
    return corners.reshape(-1, 2).astype(np.float32)


def _solve_checkerboard_pose(
    object_points: np.ndarray,
    image_points: np.ndarray,
    calibration: CameraCalibration,
) -> tuple[np.ndarray, np.ndarray, float]:
    success, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        calibration.color_camera_matrix,
        calibration.color_distortion_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success:
        raise ValueError(f"solvePnP failed for {calibration.name}.")

    rotation_matrix, _ = cv2.Rodrigues(rvec)
    translation_mm = tvec.reshape(3).astype(np.float64)

    projected, _ = cv2.projectPoints(
        object_points,
        rvec,
        tvec,
        calibration.color_camera_matrix,
        calibration.color_distortion_coeffs,
    )
    projected = projected.reshape(-1, 2)
    reprojection_error_px = math.sqrt(
        float(np.mean(np.sum((projected - image_points) ** 2, axis=1)))
    )
    return rotation_matrix.astype(np.float64), translation_mm, reprojection_error_px


def _compose_color_pose_into_depth(
    rotation_color_board: np.ndarray,
    translation_color_board_mm: np.ndarray,
    calibration: CameraCalibration,
) -> tuple[np.ndarray, np.ndarray]:
    rotation_depth_board = calibration.color_to_depth_rotation @ rotation_color_board
    translation_depth_board_mm = (
        calibration.color_to_depth_translation_mm
        + calibration.color_to_depth_rotation @ translation_color_board_mm
    )
    return rotation_depth_board, translation_depth_board_mm


def _estimate_aux_to_base_for_pair(
    object_points: np.ndarray,
    base_image_path: Path,
    aux_image_path: Path,
    base_calibration: CameraCalibration,
    aux_calibration: CameraCalibration,
    pattern_size: tuple[int, int],
) -> PairEstimate | None:
    base_gray = _read_grayscale_image(base_image_path)
    aux_gray = _read_grayscale_image(aux_image_path)

    base_corners = _detect_checkerboard(base_gray, pattern_size)
    aux_corners = _detect_checkerboard(aux_gray, pattern_size)
    if base_corners is None or aux_corners is None:
        return None

    base_rot_color_board, base_t_color_board, base_reproj = _solve_checkerboard_pose(
        object_points, base_corners, base_calibration
    )
    aux_rot_color_board, aux_t_color_board, aux_reproj = _solve_checkerboard_pose(
        object_points, aux_corners, aux_calibration
    )

    base_rot_depth_board, base_t_depth_board = _compose_color_pose_into_depth(
        base_rot_color_board, base_t_color_board, base_calibration
    )
    aux_rot_depth_board, aux_t_depth_board = _compose_color_pose_into_depth(
        aux_rot_color_board, aux_t_color_board, aux_calibration
    )

    rotation_aux_to_base = base_rot_depth_board @ aux_rot_depth_board.T
    translation_aux_to_base_mm = base_t_depth_board - rotation_aux_to_base @ aux_t_depth_board

    return PairEstimate(
        stem=base_image_path.stem,
        base_image=str(base_image_path),
        aux_image=str(aux_image_path),
        base_reprojection_error_px=base_reproj,
        aux_reprojection_error_px=aux_reproj,
        rotation_aux_to_base=rotation_aux_to_base,
        translation_aux_to_base_mm=translation_aux_to_base_mm,
    )


def _average_rotation_matrices(rotations: list[np.ndarray]) -> np.ndarray:
    accumulator = np.zeros((3, 3), dtype=np.float64)
    for rotation in rotations:
        accumulator += rotation

    u_matrix, _, vt_matrix = np.linalg.svd(accumulator)
    average_rotation = u_matrix @ vt_matrix
    if np.linalg.det(average_rotation) < 0.0:
        u_matrix[:, -1] *= -1.0
        average_rotation = u_matrix @ vt_matrix
    return average_rotation


def _rotation_angle_deg(rotation_matrix: np.ndarray) -> float:
    cosine = (np.trace(rotation_matrix) - 1.0) * 0.5
    cosine = float(np.clip(cosine, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def _round_vector(values: np.ndarray, digits: int = 6) -> list[float]:
    return [round(float(value), digits) for value in values.reshape(-1)]


def _round_matrix(values: np.ndarray, digits: int = 6) -> list[list[float]]:
    return [[round(float(value), digits) for value in row] for row in values]


def _update_config_file(
    config_in: Path,
    config_out: Path,
    translation_mm: np.ndarray,
    rotation_matrix: np.ndarray,
) -> None:
    payload = _load_json(config_in)
    payload["aux_translation_mm"] = _round_vector(translation_mm)
    payload["rotation_matrix"] = _round_matrix(rotation_matrix)
    with config_out.open("w", encoding="utf-8", newline="\n") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=4)
        handle.write("\n")


def _parse_optional_path_from_config(value: Any, config_dir: Path, label: str) -> Path | None:
    if value is None:
        return None
    if not isinstance(value, str):
        raise ValueError(f"{label} must be a string path.")
    path = Path(value)
    if not path.is_absolute():
        path = config_dir / path
    return path


def _parse_optional_int(value: Any, label: str) -> int | None:
    if value is None:
        return None
    if isinstance(value, bool):
        raise ValueError(f"{label} must be an integer, got boolean.")
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be an integer.") from exc


def _parse_optional_float(value: Any, label: str) -> float | None:
    if value is None:
        return None
    if isinstance(value, bool):
        raise ValueError(f"{label} must be a number, got boolean.")
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a number.") from exc


def _resolve_run_settings(args: argparse.Namespace) -> CalibrationRunSettings:
    payload: dict[str, Any] = {}
    checkerboard_payload: dict[str, Any] = {}
    config_dir = Path.cwd()

    if args.calibration_config is not None:
        payload = _load_object_json(args.calibration_config, "Calibration config JSON")
        config_dir = args.calibration_config.parent
        checkerboard_value = payload.get("checkerboard")
        if checkerboard_value is not None:
            if not isinstance(checkerboard_value, dict):
                raise ValueError("`checkerboard` in calibration config must be a JSON object.")
            checkerboard_payload = checkerboard_value

    def _coalesce_path(cli_value: Path | None, config_key: str) -> Path | None:
        if cli_value is not None:
            return cli_value
        return _parse_optional_path_from_config(payload.get(config_key), config_dir, config_key)

    def _coalesce_board_setting(cli_value: int | float | None, key: str) -> int | float | None:
        if cli_value is not None:
            return cli_value
        if key in checkerboard_payload:
            return checkerboard_payload[key]
        return payload.get(key)

    base_dir = _coalesce_path(args.base_dir, "base_dir")
    aux_dir = _coalesce_path(args.aux_dir, "aux_dir")
    rig_calibration = _coalesce_path(args.rig_calibration, "rig_calibration")
    output_json = _coalesce_path(args.output_json, "output_json")
    config_in = _coalesce_path(args.config_in, "config_in")
    config_out = _coalesce_path(args.config_out, "config_out")
    board_cols = _parse_optional_int(_coalesce_board_setting(args.board_cols, "board_cols"), "board_cols")
    board_rows = _parse_optional_int(_coalesce_board_setting(args.board_rows, "board_rows"), "board_rows")
    square_size_mm = _parse_optional_float(
        _coalesce_board_setting(args.square_size_mm, "square_size_mm"),
        "square_size_mm",
    )

    missing: list[str] = []
    if base_dir is None:
        missing.append("base_dir / --base-dir")
    if aux_dir is None:
        missing.append("aux_dir / --aux-dir")
    if rig_calibration is None:
        missing.append("rig_calibration / --rig-calibration")
    if board_cols is None:
        missing.append("checkerboard.board_cols / --board-cols")
    if board_rows is None:
        missing.append("checkerboard.board_rows / --board-rows")
    if square_size_mm is None:
        missing.append("checkerboard.square_size_mm / --square-size-mm")
    if missing:
        raise SystemExit(
            "Missing required calibration settings: "
            + ", ".join(missing)
            + ". Provide them via CLI or --calibration-config."
        )

    return CalibrationRunSettings(
        base_dir=base_dir,
        aux_dir=aux_dir,
        rig_calibration=rig_calibration,
        board_cols=board_cols,
        board_rows=board_rows,
        square_size_mm=square_size_mm,
        output_json=output_json,
        config_in=config_in,
        config_out=config_out,
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Estimate aux-depth to base-depth extrinsics from paired checkerboard images."
    )
    parser.add_argument(
        "--calibration-config",
        type=Path,
        help="Optional JSON file containing calibration inputs such as paths and checkerboard settings.",
    )
    parser.add_argument("--base-dir", type=Path, help="Directory containing base checkerboard images.")
    parser.add_argument("--aux-dir", type=Path, help="Directory containing aux checkerboard images.")
    parser.add_argument(
        "--rig-calibration",
        type=Path,
        help="JSON file containing base/aux color intrinsics and color<->depth extrinsics.",
    )
    parser.add_argument(
        "--board-cols",
        type=int,
        help="Checkerboard inner-corner count along columns.",
    )
    parser.add_argument(
        "--board-rows",
        type=int,
        help="Checkerboard inner-corner count along rows.",
    )
    parser.add_argument(
        "--square-size-mm",
        type=float,
        help="Checkerboard square size in millimeters.",
    )
    parser.add_argument(
        "--output-json",
        type=Path,
        help="Optional path to write a detailed calibration report JSON.",
    )
    parser.add_argument(
        "--config-in",
        type=Path,
        help="Optional existing tracking config JSON to patch with aux_translation_mm and rotation_matrix.",
    )
    parser.add_argument(
        "--config-out",
        type=Path,
        help="Output path for the patched tracking config JSON. Requires --config-in.",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    _import_dependencies()
    settings = _resolve_run_settings(args)
    if (settings.config_in is None) != (settings.config_out is None):
        raise SystemExit("--config-in and --config-out must be provided together.")
    if settings.board_cols <= 1 or settings.board_rows <= 1:
        raise SystemExit("Checkerboard inner-corner counts must both be greater than 1.")
    if settings.square_size_mm <= 0.0:
        raise SystemExit("--square-size-mm must be positive.")

    base_calibration, aux_calibration = _load_rig_calibration(settings.rig_calibration)
    base_index = _build_image_index(settings.base_dir)
    aux_index = _build_image_index(settings.aux_dir)
    shared_stems = sorted(set(base_index) & set(aux_index))
    if not shared_stems:
        raise SystemExit("No paired checkerboard images were found. Match files by stem across both directories.")

    missing_in_aux = sorted(set(base_index) - set(aux_index))
    missing_in_base = sorted(set(aux_index) - set(base_index))
    if missing_in_aux:
        print(f"Skipping {len(missing_in_aux)} base-only images: {', '.join(missing_in_aux[:5])}", file=sys.stderr)
    if missing_in_base:
        print(f"Skipping {len(missing_in_base)} aux-only images: {', '.join(missing_in_base[:5])}", file=sys.stderr)

    object_points = _make_board_object_points(
        settings.board_cols,
        settings.board_rows,
        settings.square_size_mm,
    )
    pattern_size = (settings.board_cols, settings.board_rows)

    pair_estimates: list[PairEstimate] = []
    skipped_stems: list[str] = []
    for stem in shared_stems:
        estimate = _estimate_aux_to_base_for_pair(
            object_points,
            base_index[stem],
            aux_index[stem],
            base_calibration,
            aux_calibration,
            pattern_size,
        )
        if estimate is None:
            skipped_stems.append(stem)
            continue
        pair_estimates.append(estimate)

    if len(pair_estimates) < 1:
        raise SystemExit("Checkerboard detection failed for every image pair.")

    rotation_matrices = [estimate.rotation_aux_to_base for estimate in pair_estimates]
    translations_mm = np.stack([estimate.translation_aux_to_base_mm for estimate in pair_estimates], axis=0)
    average_rotation = _average_rotation_matrices(rotation_matrices)
    average_translation_mm = np.mean(translations_mm, axis=0)
    translation_std_mm = np.std(translations_mm, axis=0)

    pair_summaries = []
    for estimate in pair_estimates:
        delta_rotation = estimate.rotation_aux_to_base @ average_rotation.T
        pair_summaries.append(
            {
                "stem": estimate.stem,
                "base_image": estimate.base_image,
                "aux_image": estimate.aux_image,
                "base_reprojection_error_px": round(estimate.base_reprojection_error_px, 6),
                "aux_reprojection_error_px": round(estimate.aux_reprojection_error_px, 6),
                "rotation_delta_deg_from_average": round(_rotation_angle_deg(delta_rotation), 6),
                "translation_aux_to_base_mm": _round_vector(estimate.translation_aux_to_base_mm),
            }
        )

    result_payload = {
        "pairs_found": len(shared_stems),
        "pairs_used": len(pair_estimates),
        "pairs_skipped": skipped_stems,
        "aux_translation_mm": _round_vector(average_translation_mm),
        "rotation_matrix": _round_matrix(average_rotation),
        "translation_std_mm": _round_vector(translation_std_mm),
        "mean_base_reprojection_error_px": round(
            float(np.mean([estimate.base_reprojection_error_px for estimate in pair_estimates])),
            6,
        ),
        "mean_aux_reprojection_error_px": round(
            float(np.mean([estimate.aux_reprojection_error_px for estimate in pair_estimates])),
            6,
        ),
        "pair_details": pair_summaries,
    }

    print("Checkerboard extrinsic estimate")
    print(f"  pairs found   : {len(shared_stems)}")
    print(f"  pairs used    : {len(pair_estimates)}")
    print(f"  pairs skipped : {len(skipped_stems)}")
    print("  aux_translation_mm:")
    for value in _round_vector(average_translation_mm):
        print(f"    {value}")
    print("  rotation_matrix:")
    for row in _round_matrix(average_rotation):
        print(f"    {row}")
    print(f"  mean base reprojection error [px] : {result_payload['mean_base_reprojection_error_px']}")
    print(f"  mean aux reprojection error  [px] : {result_payload['mean_aux_reprojection_error_px']}")
    print()
    print("Config snippet:")
    print(
        json.dumps(
            {
                "aux_translation_mm": result_payload["aux_translation_mm"],
                "rotation_matrix": result_payload["rotation_matrix"],
            },
            ensure_ascii=False,
            indent=4,
        )
    )

    if settings.output_json is not None:
        with settings.output_json.open("w", encoding="utf-8", newline="\n") as handle:
            json.dump(result_payload, handle, ensure_ascii=False, indent=4)
            handle.write("\n")
        print(f"\nWrote report JSON: {settings.output_json}")

    if settings.config_in is not None and settings.config_out is not None:
        _update_config_file(settings.config_in, settings.config_out, average_translation_mm, average_rotation)
        print(f"Wrote patched config JSON: {settings.config_out}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
