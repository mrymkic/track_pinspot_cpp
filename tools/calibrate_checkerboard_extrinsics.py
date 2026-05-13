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

A plain checkerboard has a 180-degree corner-order ambiguity. If the estimated
transform projects the base ear near a clearly wrong place in the aux view, rerun
with --aux-corner-order reverse or --base-corner-order reverse and compare the
resulting runtime overlay and trace metrics.
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
    base_corner_order: str
    aux_corner_order: str
    output_json: Path | None
    config_in: Path | None
    config_out: Path | None


@dataclass
class OutlierRejectionResult:
    inlier_indices: list[int]
    rotation_reference_index: int | None
    rotation_threshold_deg: float | None
    translation_threshold_mm: float | None
    rotation_residuals_deg: list[float]
    translation_residuals_mm: list[float]
    rejection_reasons: list[str]
    iterations: int
    status: str
    minimum_inliers: int


def _as_float_array(data: Any, shape: tuple[int, ...], label: str) -> np.ndarray:
    array = np.asarray(data, dtype=np.float64)
    if array.shape != shape:
        raise ValueError(f"{label} must have shape {shape}, got {array.shape}.")
    return array


def _load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8-sig") as handle:
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
    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])
    cx = float(camera_matrix[0, 2])
    cy = float(camera_matrix[1, 2])
    if fx <= 0.0 or fy <= 0.0 or cx <= 0.0 or cy <= 0.0:
        raise ValueError(
            f"{label}.color_camera_matrix looks uninitialized. "
            "Populate the rig calibration JSON with the real Azure Kinect color intrinsics "
            "instead of leaving the checkerboard_rig_template placeholder values in place."
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


def _apply_corner_order(corners: np.ndarray, order: str) -> np.ndarray:
    if order == "normal":
        return corners
    if order == "reverse":
        return corners[::-1].copy()
    raise ValueError(f"Unsupported checkerboard corner order: {order}")


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
    base_corner_order: str,
    aux_corner_order: str,
) -> PairEstimate | None:
    base_gray = _read_grayscale_image(base_image_path)
    aux_gray = _read_grayscale_image(aux_image_path)

    base_corners = _detect_checkerboard(base_gray, pattern_size)
    aux_corners = _detect_checkerboard(aux_gray, pattern_size)
    if base_corners is None or aux_corners is None:
        return None
    base_corners = _apply_corner_order(base_corners, base_corner_order)
    aux_corners = _apply_corner_order(aux_corners, aux_corner_order)

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


def _median_absolute_deviation(values: list[float]) -> float:
    if not values:
        return 0.0
    array = np.asarray(values, dtype=np.float64)
    median = float(np.median(array))
    return float(np.median(np.abs(array - median)))


def _robust_residual_threshold(
    residuals: list[float],
    minimum_threshold: float,
    sigma_scale: float = 3.5,
) -> float:
    if not residuals:
        return minimum_threshold
    median = float(np.median(np.asarray(residuals, dtype=np.float64)))
    mad = _median_absolute_deviation(residuals)
    robust_sigma = 1.4826 * mad
    return max(minimum_threshold, median + sigma_scale * robust_sigma)


def _find_rotation_medoid_index(rotations: list[np.ndarray], candidate_indices: list[int]) -> int:
    best_index = candidate_indices[0]
    best_score = math.inf
    for candidate_index in candidate_indices:
        score = 0.0
        candidate_rotation = rotations[candidate_index]
        for other_index in candidate_indices:
            score += _rotation_angle_deg(candidate_rotation @ rotations[other_index].T)
        if score < best_score:
            best_score = score
            best_index = candidate_index
    return best_index


def _reject_transform_outliers(pair_estimates: list[PairEstimate]) -> OutlierRejectionResult:
    total_pairs = len(pair_estimates)
    all_indices = list(range(total_pairs))
    minimum_inliers = 2 if total_pairs >= 2 else 1
    if total_pairs < 3:
        return OutlierRejectionResult(
            inlier_indices=all_indices,
            rotation_reference_index=0 if all_indices else None,
            rotation_threshold_deg=None,
            translation_threshold_mm=None,
            rotation_residuals_deg=[0.0] * total_pairs,
            translation_residuals_mm=[0.0] * total_pairs,
            rejection_reasons=[""] * total_pairs,
            iterations=0,
            status="disabled_too_few_pairs",
            minimum_inliers=minimum_inliers,
        )

    rotations = [estimate.rotation_aux_to_base for estimate in pair_estimates]
    active_indices = all_indices[:]
    iterations = 0

    while True:
        iterations += 1
        rotation_reference_index = _find_rotation_medoid_index(rotations, active_indices)
        translation_center_mm = np.median(
            np.stack([pair_estimates[index].translation_aux_to_base_mm for index in active_indices], axis=0),
            axis=0,
        )

        rotation_residuals_deg = [
            _rotation_angle_deg(estimate.rotation_aux_to_base @ rotations[rotation_reference_index].T)
            for estimate in pair_estimates
        ]
        translation_residuals_mm = [
            float(np.linalg.norm(estimate.translation_aux_to_base_mm - translation_center_mm))
            for estimate in pair_estimates
        ]

        active_rotation_residuals = [rotation_residuals_deg[index] for index in active_indices]
        active_translation_residuals = [translation_residuals_mm[index] for index in active_indices]
        rotation_threshold_deg = _robust_residual_threshold(active_rotation_residuals, minimum_threshold=2.0)
        translation_threshold_mm = _robust_residual_threshold(active_translation_residuals, minimum_threshold=30.0)

        new_active_indices = [
            index
            for index in active_indices
            if rotation_residuals_deg[index] <= rotation_threshold_deg
            and translation_residuals_mm[index] <= translation_threshold_mm
        ]

        if len(new_active_indices) < minimum_inliers:
            ranked_indices = sorted(
                active_indices,
                key=lambda index: (
                    (rotation_residuals_deg[index] / max(rotation_threshold_deg, 1e-6)) ** 2
                    + (translation_residuals_mm[index] / max(translation_threshold_mm, 1e-6)) ** 2
                ),
            )
            new_active_indices = ranked_indices[:minimum_inliers]

        if new_active_indices == active_indices:
            break
        active_indices = new_active_indices

    final_rotation_reference_index = _find_rotation_medoid_index(rotations, active_indices)
    final_translation_center_mm = np.median(
        np.stack([pair_estimates[index].translation_aux_to_base_mm for index in active_indices], axis=0),
        axis=0,
    )
    final_rotation_residuals_deg = [
        _rotation_angle_deg(estimate.rotation_aux_to_base @ rotations[final_rotation_reference_index].T)
        for estimate in pair_estimates
    ]
    final_translation_residuals_mm = [
        float(np.linalg.norm(estimate.translation_aux_to_base_mm - final_translation_center_mm))
        for estimate in pair_estimates
    ]
    final_rotation_threshold_deg = _robust_residual_threshold(
        [final_rotation_residuals_deg[index] for index in active_indices],
        minimum_threshold=2.0,
    )
    final_translation_threshold_mm = _robust_residual_threshold(
        [final_translation_residuals_mm[index] for index in active_indices],
        minimum_threshold=30.0,
    )

    rejection_reasons: list[str] = []
    inlier_set = set(active_indices)
    for index in all_indices:
        if index in inlier_set:
            rejection_reasons.append("")
            continue
        reasons: list[str] = []
        if final_rotation_residuals_deg[index] > final_rotation_threshold_deg:
            reasons.append("rotation_outlier")
        if final_translation_residuals_mm[index] > final_translation_threshold_mm:
            reasons.append("translation_outlier")
        if not reasons:
            reasons.append("consensus_outlier")
        rejection_reasons.append("+".join(reasons))

    rejected_count = total_pairs - len(active_indices)
    status = "applied" if rejected_count > 0 else "no_outliers_detected"
    return OutlierRejectionResult(
        inlier_indices=active_indices,
        rotation_reference_index=final_rotation_reference_index,
        rotation_threshold_deg=final_rotation_threshold_deg,
        translation_threshold_mm=final_translation_threshold_mm,
        rotation_residuals_deg=final_rotation_residuals_deg,
        translation_residuals_mm=final_translation_residuals_mm,
        rejection_reasons=rejection_reasons,
        iterations=iterations,
        status=status,
        minimum_inliers=minimum_inliers,
    )


def _round_vector(values: np.ndarray, digits: int = 6) -> list[float]:
    return [round(float(value), digits) for value in values.reshape(-1)]


def _round_matrix(values: np.ndarray, digits: int = 6) -> list[list[float]]:
    return [[round(float(value), digits) for value in row] for row in values]


def _write_json_file(path: Path, payload: Any) -> None:
    with path.open("w", encoding="utf-8", newline="\n") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=4)
        handle.write("\n")


def _update_config_file(
    config_in: Path,
    config_out: Path,
    translation_mm: np.ndarray,
    rotation_matrix: np.ndarray,
) -> dict[str, Any]:
    payload = _load_json(config_in)
    payload["aux_translation_mm"] = _round_vector(translation_mm)
    payload["rotation_matrix"] = _round_matrix(rotation_matrix)
    _write_json_file(config_out, payload)
    return payload


def _build_rig_sync_payload(payload: dict[str, Any]) -> dict[str, Any]:
    keys = (
        "base_kinect_serial",
        "aux_kinect_serial",
        "aux_translation_mm",
        "rotation_matrix",
        "subordinate_delay_off_master_usec",
    )
    return {key: payload[key] for key in keys if key in payload}


def _collect_related_track_config_paths(config_out: Path) -> list[Path]:
    if config_out.name != "track_config_2.json":
        return []
    return sorted(
        candidate
        for candidate in config_out.parent.glob("track_config_2*.json")
        if candidate.name != config_out.name
    )


def _sync_related_track_configs(config_out: Path, source_payload: dict[str, Any]) -> list[Path]:
    sync_payload = _build_rig_sync_payload(source_payload)
    updated_paths: list[Path] = []
    for target_path in _collect_related_track_config_paths(config_out):
        target_payload = _load_json(target_path)
        if not isinstance(target_payload, dict):
            raise ValueError(f"Tracking config JSON must be an object: {target_path}")
        for key, value in sync_payload.items():
            target_payload[key] = value
        _write_json_file(target_path, target_payload)
        updated_paths.append(target_path)
    return updated_paths


def _parse_optional_path_from_config(value: Any, config_dir: Path, label: str) -> Path | None:
    if value is None:
        return None
    if not isinstance(value, str):
        raise ValueError(f"{label} must be a string path.")
    path = Path(value)
    if not path.is_absolute():
        path = config_dir / path
    return path


def _ensure_path_has_no_template_placeholder(path: Path | None, label: str) -> None:
    if path is None:
        return
    if "SESSION_NAME" in path.as_posix().split("/"):
        raise SystemExit(
            f"{label} still contains the template placeholder `SESSION_NAME`: {path}. "
            "Edit the calibration config JSON to point at a real capture session before running."
        )


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


def _parse_corner_order(value: Any, label: str) -> str:
    if value is None:
        return "normal"
    if not isinstance(value, str):
        raise ValueError(f"{label} must be a string.")
    normalized = value.strip().lower()
    if normalized not in {"normal", "reverse"}:
        raise ValueError(f"{label} must be either 'normal' or 'reverse'.")
    return normalized


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

    def _coalesce_board_setting(cli_value: Any, key: str) -> Any:
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
    base_corner_order = _parse_corner_order(
        _coalesce_board_setting(args.base_corner_order, "base_corner_order"),
        "base_corner_order",
    )
    aux_corner_order = _parse_corner_order(
        _coalesce_board_setting(args.aux_corner_order, "aux_corner_order"),
        "aux_corner_order",
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

    _ensure_path_has_no_template_placeholder(base_dir, "base_dir")
    _ensure_path_has_no_template_placeholder(aux_dir, "aux_dir")
    _ensure_path_has_no_template_placeholder(output_json, "output_json")

    return CalibrationRunSettings(
        base_dir=base_dir,
        aux_dir=aux_dir,
        rig_calibration=rig_calibration,
        board_cols=board_cols,
        board_rows=board_rows,
        square_size_mm=square_size_mm,
        base_corner_order=base_corner_order,
        aux_corner_order=aux_corner_order,
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
        "--base-corner-order",
        choices=("normal", "reverse"),
        help="Corner order to use for base images. Use 'reverse' for 180-degree checkerboard ordering ambiguity.",
    )
    parser.add_argument(
        "--aux-corner-order",
        choices=("normal", "reverse"),
        help="Corner order to use for aux images. Use 'reverse' for 180-degree checkerboard ordering ambiguity.",
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
            settings.base_corner_order,
            settings.aux_corner_order,
        )
        if estimate is None:
            skipped_stems.append(stem)
            continue
        pair_estimates.append(estimate)

    if len(pair_estimates) < 1:
        raise SystemExit("Checkerboard detection failed for every image pair.")

    outlier_rejection = _reject_transform_outliers(pair_estimates)
    inlier_estimates = [pair_estimates[index] for index in outlier_rejection.inlier_indices]
    if not inlier_estimates:
        raise SystemExit("Outlier rejection removed every checkerboard pair. Capture more images and retry.")

    inlier_rotation_matrices = [estimate.rotation_aux_to_base for estimate in inlier_estimates]
    inlier_translations_mm = np.stack([estimate.translation_aux_to_base_mm for estimate in inlier_estimates], axis=0)
    average_rotation = _average_rotation_matrices(inlier_rotation_matrices)
    average_translation_mm = np.mean(inlier_translations_mm, axis=0)
    translation_std_mm = np.std(inlier_translations_mm, axis=0)
    all_detected_translations_mm = np.stack([estimate.translation_aux_to_base_mm for estimate in pair_estimates], axis=0)
    all_detected_translation_std_mm = np.std(all_detected_translations_mm, axis=0)

    pair_summaries = []
    inlier_index_set = set(outlier_rejection.inlier_indices)
    for pair_index, estimate in enumerate(pair_estimates):
        delta_rotation = estimate.rotation_aux_to_base @ average_rotation.T
        pair_summaries.append(
            {
                "stem": estimate.stem,
                "base_image": estimate.base_image,
                "aux_image": estimate.aux_image,
                "included_in_final_estimate": pair_index in inlier_index_set,
                "rejection_reason": outlier_rejection.rejection_reasons[pair_index] or None,
                "base_reprojection_error_px": round(estimate.base_reprojection_error_px, 6),
                "aux_reprojection_error_px": round(estimate.aux_reprojection_error_px, 6),
                "rotation_delta_deg_from_average": round(_rotation_angle_deg(delta_rotation), 6),
                "rotation_residual_deg_from_consensus": round(outlier_rejection.rotation_residuals_deg[pair_index], 6),
                "translation_residual_mm_from_consensus": round(
                    outlier_rejection.translation_residuals_mm[pair_index],
                    6,
                ),
                "translation_aux_to_base_mm": _round_vector(estimate.translation_aux_to_base_mm),
            }
        )

    rejected_stems = [
        pair_estimates[index].stem
        for index in range(len(pair_estimates))
        if index not in inlier_index_set
    ]
    result_payload = {
        "pairs_found": len(shared_stems),
        "pairs_detected": len(pair_estimates),
        "pairs_used": len(inlier_estimates),
        "pairs_skipped": skipped_stems,
        "pairs_rejected_outliers": rejected_stems,
        "base_corner_order": settings.base_corner_order,
        "aux_corner_order": settings.aux_corner_order,
        "aux_translation_mm": _round_vector(average_translation_mm),
        "rotation_matrix": _round_matrix(average_rotation),
        "translation_std_mm": _round_vector(translation_std_mm),
        "translation_std_mm_all_detected": _round_vector(all_detected_translation_std_mm),
        "outlier_rejection": {
            "status": outlier_rejection.status,
            "iterations": outlier_rejection.iterations,
            "minimum_inliers": outlier_rejection.minimum_inliers,
            "rotation_reference_stem": (
                pair_estimates[outlier_rejection.rotation_reference_index].stem
                if outlier_rejection.rotation_reference_index is not None
                else None
            ),
            "rotation_threshold_deg": (
                round(outlier_rejection.rotation_threshold_deg, 6)
                if outlier_rejection.rotation_threshold_deg is not None
                else None
            ),
            "translation_threshold_mm": (
                round(outlier_rejection.translation_threshold_mm, 6)
                if outlier_rejection.translation_threshold_mm is not None
                else None
            ),
        },
        "mean_base_reprojection_error_px": round(
            float(np.mean([estimate.base_reprojection_error_px for estimate in inlier_estimates])),
            6,
        ),
        "mean_aux_reprojection_error_px": round(
            float(np.mean([estimate.aux_reprojection_error_px for estimate in inlier_estimates])),
            6,
        ),
        "mean_base_reprojection_error_px_all_detected": round(
            float(np.mean([estimate.base_reprojection_error_px for estimate in pair_estimates])),
            6,
        ),
        "mean_aux_reprojection_error_px_all_detected": round(
            float(np.mean([estimate.aux_reprojection_error_px for estimate in pair_estimates])),
            6,
        ),
        "pair_details": pair_summaries,
    }

    print("Checkerboard extrinsic estimate")
    print(f"  pairs found   : {len(shared_stems)}")
    print(f"  pairs detected: {len(pair_estimates)}")
    print(f"  pairs used    : {len(inlier_estimates)}")
    print(f"  pairs skipped : {len(skipped_stems)}")
    print(f"  pairs rejected: {len(rejected_stems)}")
    print(f"  base corner order : {settings.base_corner_order}")
    print(f"  aux corner order  : {settings.aux_corner_order}")
    if rejected_stems:
        print(f"  rejected stems    : {', '.join(rejected_stems)}")
    print(f"  outlier rejection : {outlier_rejection.status}")
    if outlier_rejection.rotation_threshold_deg is not None:
        print(f"  rotation inlier threshold [deg] : {round(outlier_rejection.rotation_threshold_deg, 6)}")
    if outlier_rejection.translation_threshold_mm is not None:
        print(f"  translation inlier threshold [mm]: {round(outlier_rejection.translation_threshold_mm, 6)}")
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
        patched_config_payload = _update_config_file(
            settings.config_in,
            settings.config_out,
            average_translation_mm,
            average_rotation,
        )
        print(f"Wrote patched config JSON: {settings.config_out}")
        synced_paths = _sync_related_track_configs(settings.config_out, patched_config_payload)
        if synced_paths:
            print("Synced related track configs:")
            for synced_path in synced_paths:
                print(f"  {synced_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
