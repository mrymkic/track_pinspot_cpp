#!/usr/bin/env python3
r"""Evaluate fused checkerboard coordinates against a known rotation schedule.

This script estimates checkerboard corner poses for paired base/aux color images,
transforms the aux pose into the base depth coordinate frame, and evaluates the
fused coordinates that would be produced by taking:

    fused = { base.x, base.y, aux_in_base.z }

The evaluation follows the method sketched in
`評価手法/kinect深度座標精度の評価手法（補足）.pdf`:

1. Capture paired checkerboard images at a reference angle.
2. Rotate the checkerboard by known angles around a fixed board axis.
3. Estimate the resulting depth-coordinate changes from Kinect images.
4. Compare the estimated changes against the theoretical changes.

Required inputs
---------------
- Paired `base` / `aux` checkerboard image directories.
- A rig calibration JSON for the checkerboard pose estimation step.
- A tracking config JSON (or equivalent JSON) containing `aux_translation_mm`
  and `rotation_matrix` for the fused-coordinate transform.
- A CSV mapping each image stem to the known checkerboard rotation angle.

Angle CSV example
-----------------
    stem,angle_deg
    0001,0
    0002,5
    0003,10

Example
-------
    python .\tools\evaluate_checkerboard_fusion.py ^
      --evaluation-config .\tools\checkerboard_fusion_eval_config_template.json
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import calibrate_checkerboard_extrinsics as checkerboard


@dataclass(frozen=True)
class EvaluationSettings:
    base_dir: Path
    aux_dir: Path
    rig_calibration: Path
    transform_config: Path
    angles_csv: Path
    board_cols: int
    board_rows: int
    square_size_mm: float
    base_corner_order: str
    aux_corner_order: str
    angle_stem_col: str
    angle_deg_col: str
    reference_stem: str | None
    rotation_axis: str
    pivot_coordinate_mm: float | None
    output_json: Path | None


@dataclass(frozen=True)
class AngleEntry:
    stem: str
    angle_deg: float


@dataclass
class CheckerboardObservation:
    stem: str
    angle_deg: float
    base_image: str
    aux_image: str
    base_reprojection_error_px: float
    aux_reprojection_error_px: float
    base_rotation_depth_board: Any
    base_translation_depth_board_mm: Any
    aux_rotation_depth_board: Any
    aux_translation_depth_board_mm: Any


@dataclass
class EvaluatedFrame:
    observation: CheckerboardObservation
    base_points: Any
    aux_transformed_points: Any
    fused_points: Any


def _load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8-sig") as handle:
        return json.load(handle)


def _load_object_json(path: Path, label: str) -> dict[str, Any]:
    payload = _load_json(path)
    if not isinstance(payload, dict):
        raise SystemExit(f"{label} must be a JSON object: {path}")
    return payload


def _resolve_optional_path(base_path: Path, raw_value: Any) -> Path | None:
    if raw_value is None:
        return None
    candidate = Path(str(raw_value))
    if candidate.is_absolute():
        return candidate
    return (base_path.parent / candidate).resolve()


def _config_value(payload: dict[str, Any], *keys: str) -> Any:
    current: Any = payload
    for key in keys:
        if not isinstance(current, dict):
            return None
        current = current.get(key)
    return current


def _resolve_settings(args: argparse.Namespace) -> EvaluationSettings:
    config_payload: dict[str, Any] = {}
    config_path = args.evaluation_config
    if config_path is not None:
        config_payload = _load_object_json(config_path, "evaluation_config")

    base_dir = args.base_dir or _resolve_optional_path(config_path, config_payload.get("base_dir"))
    aux_dir = args.aux_dir or _resolve_optional_path(config_path, config_payload.get("aux_dir"))
    rig_calibration = args.rig_calibration or _resolve_optional_path(
        config_path,
        config_payload.get("rig_calibration"),
    )
    transform_config = args.transform_config or _resolve_optional_path(
        config_path,
        config_payload.get("transform_config"),
    )
    angles_csv = args.angles_csv or _resolve_optional_path(config_path, config_payload.get("angles_csv"))

    checkerboard_payload = config_payload.get("checkerboard")
    if checkerboard_payload is not None and not isinstance(checkerboard_payload, dict):
        raise SystemExit("`checkerboard` in evaluation config must be a JSON object.")

    angles_payload = config_payload.get("angles")
    if angles_payload is not None and not isinstance(angles_payload, dict):
        raise SystemExit("`angles` in evaluation config must be a JSON object.")

    evaluation_payload = config_payload.get("evaluation")
    if evaluation_payload is not None and not isinstance(evaluation_payload, dict):
        raise SystemExit("`evaluation` in evaluation config must be a JSON object.")

    board_cols = args.board_cols
    if board_cols is None and checkerboard_payload is not None:
        board_cols = checkerboard_payload.get("board_cols")

    board_rows = args.board_rows
    if board_rows is None and checkerboard_payload is not None:
        board_rows = checkerboard_payload.get("board_rows")

    square_size_mm = args.square_size_mm
    if square_size_mm is None and checkerboard_payload is not None:
        square_size_mm = checkerboard_payload.get("square_size_mm")

    base_corner_order = args.base_corner_order
    if base_corner_order is None and checkerboard_payload is not None:
        base_corner_order = checkerboard_payload.get("base_corner_order")
    if base_corner_order is None:
        base_corner_order = "normal"

    aux_corner_order = args.aux_corner_order
    if aux_corner_order is None and checkerboard_payload is not None:
        aux_corner_order = checkerboard_payload.get("aux_corner_order")
    if aux_corner_order is None:
        aux_corner_order = "normal"

    angle_stem_col = args.angle_stem_col
    if angle_stem_col is None and angles_payload is not None:
        angle_stem_col = angles_payload.get("stem_col")
    if angle_stem_col is None:
        angle_stem_col = "stem"

    angle_deg_col = args.angle_deg_col
    if angle_deg_col is None and angles_payload is not None:
        angle_deg_col = angles_payload.get("angle_deg_col")
    if angle_deg_col is None:
        angle_deg_col = "angle_deg"

    reference_stem = args.reference_stem
    if reference_stem is None and angles_payload is not None:
        raw_reference_stem = angles_payload.get("reference_stem")
        if isinstance(raw_reference_stem, str) and raw_reference_stem.strip():
            reference_stem = raw_reference_stem.strip()

    rotation_axis = args.rotation_axis
    if rotation_axis is None and evaluation_payload is not None:
        rotation_axis = evaluation_payload.get("rotation_axis")
    if rotation_axis is None:
        rotation_axis = "board_y"

    pivot_coordinate_mm = args.pivot_coordinate_mm
    if pivot_coordinate_mm is None and evaluation_payload is not None:
        raw_pivot = evaluation_payload.get("pivot_coordinate_mm")
        if raw_pivot is not None:
            pivot_coordinate_mm = float(raw_pivot)

    output_json = args.output_json
    if output_json is None:
        output_json = _resolve_optional_path(config_path, config_payload.get("output_json"))

    missing: list[str] = []
    if base_dir is None:
        missing.append("base_dir / --base-dir")
    if aux_dir is None:
        missing.append("aux_dir / --aux-dir")
    if rig_calibration is None:
        missing.append("rig_calibration / --rig-calibration")
    if transform_config is None:
        missing.append("transform_config / --transform-config")
    if angles_csv is None:
        missing.append("angles_csv / --angles-csv")
    if board_cols is None:
        missing.append("checkerboard.board_cols / --board-cols")
    if board_rows is None:
        missing.append("checkerboard.board_rows / --board-rows")
    if square_size_mm is None:
        missing.append("checkerboard.square_size_mm / --square-size-mm")
    if missing:
        raise SystemExit(
            "Missing required evaluation settings: "
            + ", ".join(missing)
            + ". Provide them via CLI or --evaluation-config."
        )

    if base_corner_order not in {"normal", "reverse"}:
        raise SystemExit("--base-corner-order must be `normal` or `reverse`.")
    if aux_corner_order not in {"normal", "reverse"}:
        raise SystemExit("--aux-corner-order must be `normal` or `reverse`.")
    if rotation_axis not in {"board_x", "board_y"}:
        raise SystemExit("--rotation-axis must be `board_x` or `board_y`.")
    if board_cols <= 1 or board_rows <= 1:
        raise SystemExit("Checkerboard inner-corner counts must both be greater than 1.")
    if square_size_mm <= 0.0:
        raise SystemExit("--square-size-mm must be positive.")

    return EvaluationSettings(
        base_dir=base_dir,
        aux_dir=aux_dir,
        rig_calibration=rig_calibration,
        transform_config=transform_config,
        angles_csv=angles_csv,
        board_cols=int(board_cols),
        board_rows=int(board_rows),
        square_size_mm=float(square_size_mm),
        base_corner_order=base_corner_order,
        aux_corner_order=aux_corner_order,
        angle_stem_col=angle_stem_col,
        angle_deg_col=angle_deg_col,
        reference_stem=reference_stem,
        rotation_axis=rotation_axis,
        pivot_coordinate_mm=pivot_coordinate_mm,
        output_json=output_json,
    )


def _load_angle_entries(path: Path, stem_col: str, angle_deg_col: str) -> dict[str, AngleEntry]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise SystemExit(f"Angle CSV has no header row: {path}")
        if stem_col not in reader.fieldnames:
            raise SystemExit(f"Angle CSV is missing stem column `{stem_col}`: {path}")
        if angle_deg_col not in reader.fieldnames:
            raise SystemExit(f"Angle CSV is missing angle column `{angle_deg_col}`: {path}")

        entries: dict[str, AngleEntry] = {}
        for row_index, raw in enumerate(reader, start=2):
            stem = raw[stem_col].strip()
            if not stem:
                raise SystemExit(f"Angle CSV row {row_index} has an empty `{stem_col}`.")
            if stem in entries:
                raise SystemExit(f"Angle CSV contains duplicate stem `{stem}`.")
            angle_text = raw[angle_deg_col].strip()
            if not angle_text:
                raise SystemExit(f"Angle CSV row {row_index} has an empty `{angle_deg_col}`.")
            try:
                angle_deg = float(angle_text)
            except ValueError as exc:
                raise SystemExit(
                    f"Angle CSV row {row_index} has a non-numeric angle `{angle_text}`."
                ) from exc
            entries[stem] = AngleEntry(stem=stem, angle_deg=angle_deg)
    if not entries:
        raise SystemExit(f"Angle CSV is empty: {path}")
    return entries


def _load_aux_transform(path: Path) -> tuple[Any, Any]:
    payload = _load_object_json(path, "transform_config")
    if "aux_translation_mm" not in payload:
        raise SystemExit(f"`aux_translation_mm` is missing in transform config: {path}")
    if "rotation_matrix" not in payload:
        raise SystemExit(f"`rotation_matrix` is missing in transform config: {path}")
    translation = checkerboard._as_float_array(
        payload["aux_translation_mm"],
        (3,),
        "transform_config.aux_translation_mm",
    )
    rotation = checkerboard._as_float_array(
        payload["rotation_matrix"],
        (3, 3),
        "transform_config.rotation_matrix",
    )
    return translation, rotation


def _estimate_observation_for_pair(
    object_points: Any,
    pattern_size: tuple[int, int],
    base_image_path: Path,
    aux_image_path: Path,
    base_calibration: Any,
    aux_calibration: Any,
    base_corner_order: str,
    aux_corner_order: str,
    angle_deg: float,
) -> CheckerboardObservation | None:
    base_gray = checkerboard._read_grayscale_image(base_image_path)
    aux_gray = checkerboard._read_grayscale_image(aux_image_path)

    base_corners = checkerboard._detect_checkerboard(base_gray, pattern_size)
    aux_corners = checkerboard._detect_checkerboard(aux_gray, pattern_size)
    if base_corners is None or aux_corners is None:
        return None

    base_corners = checkerboard._apply_corner_order(base_corners, base_corner_order)
    aux_corners = checkerboard._apply_corner_order(aux_corners, aux_corner_order)

    base_rot_color_board, base_t_color_board, base_reproj = checkerboard._solve_checkerboard_pose(
        object_points,
        base_corners,
        base_calibration,
    )
    aux_rot_color_board, aux_t_color_board, aux_reproj = checkerboard._solve_checkerboard_pose(
        object_points,
        aux_corners,
        aux_calibration,
    )

    base_rot_depth_board, base_t_depth_board = checkerboard._compose_color_pose_into_depth(
        base_rot_color_board,
        base_t_color_board,
        base_calibration,
    )
    aux_rot_depth_board, aux_t_depth_board = checkerboard._compose_color_pose_into_depth(
        aux_rot_color_board,
        aux_t_color_board,
        aux_calibration,
    )

    return CheckerboardObservation(
        stem=base_image_path.stem,
        angle_deg=angle_deg,
        base_image=str(base_image_path),
        aux_image=str(aux_image_path),
        base_reprojection_error_px=base_reproj,
        aux_reprojection_error_px=aux_reproj,
        base_rotation_depth_board=base_rot_depth_board,
        base_translation_depth_board_mm=base_t_depth_board,
        aux_rotation_depth_board=aux_rot_depth_board,
        aux_translation_depth_board_mm=aux_t_depth_board,
    )


def _points_from_pose(rotation_matrix: Any, translation_mm: Any, object_points: Any) -> Any:
    return (rotation_matrix @ object_points.T).T + translation_mm.reshape(1, 3)


def _transform_points_to_base(points: Any, rotation_matrix: Any, translation_mm: Any) -> Any:
    return (rotation_matrix @ points.T).T + translation_mm.reshape(1, 3)


def _fuse_points(base_points: Any, aux_points_in_base: Any) -> Any:
    fused = base_points.copy()
    fused[:, 2] = aux_points_in_base[:, 2]
    return fused


def _default_pivot_coordinate_mm(settings: EvaluationSettings) -> float:
    if settings.rotation_axis == "board_y":
        return 0.5 * (settings.board_cols - 1) * settings.square_size_mm
    return 0.5 * (settings.board_rows - 1) * settings.square_size_mm


def _board_axis_rotation_matrix(rotation_axis: str, angle_rad: float) -> Any:
    np = checkerboard.np
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    if rotation_axis == "board_y":
        return np.array(
            [
                [cosine, 0.0, sine],
                [0.0, 1.0, 0.0],
                [-sine, 0.0, cosine],
            ],
            dtype=np.float64,
        )
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, cosine, -sine],
            [0.0, sine, cosine],
        ],
        dtype=np.float64,
    )


def _rotated_object_points(
    object_points: Any,
    angle_deg: float,
    rotation_axis: str,
    pivot_coordinate_mm: float,
) -> Any:
    np = checkerboard.np
    local_rotation = _board_axis_rotation_matrix(rotation_axis, math.radians(angle_deg))
    pivot = np.zeros(3, dtype=np.float64)
    pivot[0 if rotation_axis == "board_y" else 1] = pivot_coordinate_mm
    return (local_rotation @ (object_points - pivot).T).T + pivot


def _expected_points_from_reference(
    reference_observation: CheckerboardObservation,
    object_points: Any,
    angle_deg: float,
    rotation_axis: str,
    pivot_coordinate_mm: float,
) -> Any:
    rotated_local_points = _rotated_object_points(
        object_points,
        angle_deg - reference_observation.angle_deg,
        rotation_axis,
        pivot_coordinate_mm,
    )
    return _points_from_pose(
        reference_observation.base_rotation_depth_board,
        reference_observation.base_translation_depth_board_mm,
        rotated_local_points,
    )


def _pctl(sorted_values: list[float], q: float) -> float:
    if not sorted_values:
        raise ValueError("percentile requires at least one value")
    if len(sorted_values) == 1:
        return sorted_values[0]
    position = (len(sorted_values) - 1) * q
    low = math.floor(position)
    high = math.ceil(position)
    if low == high:
        return sorted_values[low]
    weight = position - low
    return sorted_values[low] * (1.0 - weight) + sorted_values[high] * weight


def _signed_error_summary(values: list[float]) -> dict[str, float] | None:
    if not values:
        return None
    abs_values = sorted(abs(value) for value in values)
    return {
        "count": len(values),
        "bias_mm": statistics.fmean(values),
        "mae_mm": statistics.fmean(abs(value) for value in values),
        "rmse_mm": math.sqrt(statistics.fmean(value * value for value in values)),
        "p95_abs_mm": _pctl(abs_values, 0.95),
        "max_abs_mm": abs_values[-1],
    }


def _magnitude_summary(values: list[float]) -> dict[str, float] | None:
    if not values:
        return None
    ordered = sorted(values)
    return {
        "count": len(values),
        "mean_mm": statistics.fmean(values),
        "median_mm": statistics.median(values),
        "rmse_mm": math.sqrt(statistics.fmean(value * value for value in values)),
        "p95_mm": _pctl(ordered, 0.95),
        "max_mm": ordered[-1],
    }


def _signed_error_summary_deg(values: list[float]) -> dict[str, float] | None:
    if not values:
        return None
    abs_values = sorted(abs(value) for value in values)
    return {
        "count": len(values),
        "bias_deg": statistics.fmean(values),
        "mae_deg": statistics.fmean(abs(value) for value in values),
        "rmse_deg": math.sqrt(statistics.fmean(value * value for value in values)),
        "p95_abs_deg": _pctl(abs_values, 0.95),
        "max_abs_deg": abs_values[-1],
    }


def _point_error_summary(
    observed_points: Any,
    reference_observed_points: Any,
    expected_points: Any,
    reference_expected_points: Any,
) -> tuple[dict[str, Any], dict[str, list[float]]]:
    measured_delta_z = (observed_points[:, 2] - reference_observed_points[:, 2]).tolist()
    expected_delta_z = (expected_points[:, 2] - reference_expected_points[:, 2]).tolist()
    z_change_errors = [measured - expected for measured, expected in zip(measured_delta_z, expected_delta_z)]
    absolute_z_errors = (observed_points[:, 2] - expected_points[:, 2]).tolist()
    absolute_3d_errors = checkerboard.np.linalg.norm(observed_points - expected_points, axis=1).tolist()

    payload = {
        "corner_count": len(z_change_errors),
        "z_change_error_mm": _signed_error_summary(z_change_errors),
        "absolute_z_error_mm": _signed_error_summary(absolute_z_errors),
        "absolute_3d_error_mm": _magnitude_summary(absolute_3d_errors),
    }
    arrays = {
        "z_change_error_mm": z_change_errors,
        "absolute_z_error_mm": absolute_z_errors,
        "absolute_3d_error_mm": absolute_3d_errors,
    }
    return payload, arrays


def _apply_rigid_transform(points: Any, rotation_matrix: Any, translation_mm: Any) -> Any:
    return (rotation_matrix @ points.T).T + translation_mm.reshape(1, 3)


def _invert_rigid_transform(points: Any, rotation_matrix: Any, translation_mm: Any) -> Any:
    return (rotation_matrix.T @ (points - translation_mm.reshape(1, 3)).T).T


def _fit_rigid_transform(source_points: Any, target_points: Any) -> tuple[Any, Any]:
    np = checkerboard.np
    source_centroid = source_points.mean(axis=0)
    target_centroid = target_points.mean(axis=0)
    source_centered = source_points - source_centroid
    target_centered = target_points - target_centroid
    covariance = source_centered.T @ target_centered
    u_matrix, _, vt_matrix = np.linalg.svd(covariance)
    rotation_matrix = vt_matrix.T @ u_matrix.T
    if np.linalg.det(rotation_matrix) < 0.0:
        vt_matrix[-1, :] *= -1.0
        rotation_matrix = vt_matrix.T @ u_matrix.T
    translation_mm = target_centroid - rotation_matrix @ source_centroid
    return rotation_matrix, translation_mm


def _wrap_angle_deg(angle_deg: float) -> float:
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    if wrapped == -180.0:
        return 180.0
    return wrapped


def _label_points(frame: EvaluatedFrame, label: str) -> Any:
    if label == "base":
        return frame.base_points
    if label == "aux_transformed":
        return frame.aux_transformed_points
    if label == "fused":
        return frame.fused_points
    raise KeyError(f"Unknown label: {label}")


def _fit_global_label_pose(
    frames: list[EvaluatedFrame],
    label: str,
    object_points: Any,
    rotation_axis: str,
    pivot_coordinate_mm: float,
) -> tuple[Any, Any]:
    np = checkerboard.np
    modeled_points: list[Any] = []
    observed_points: list[Any] = []
    for frame in frames:
        modeled_points.append(
            _rotated_object_points(
                object_points,
                frame.observation.angle_deg,
                rotation_axis,
                pivot_coordinate_mm,
            )
        )
        observed_points.append(_label_points(frame, label))
    return _fit_rigid_transform(
        np.concatenate(modeled_points, axis=0),
        np.concatenate(observed_points, axis=0),
    )


def _estimate_angle_from_local_points(
    local_points: Any,
    object_points: Any,
    rotation_axis: str,
    pivot_coordinate_mm: float,
) -> float:
    np = checkerboard.np
    if rotation_axis == "board_y":
        offsets = object_points[:, 0] - pivot_coordinate_mm
        observed_primary = local_points[:, 0] - pivot_coordinate_mm
        observed_depth = local_points[:, 2]
        sine = -float(np.dot(offsets, observed_depth))
    else:
        offsets = object_points[:, 1] - pivot_coordinate_mm
        observed_primary = local_points[:, 1] - pivot_coordinate_mm
        observed_depth = local_points[:, 2]
        sine = float(np.dot(offsets, observed_depth))

    denominator = float(np.dot(offsets, offsets))
    if denominator <= 1e-12:
        raise ValueError("Cannot estimate angle because the rotation axis offsets are degenerate.")

    cosine = float(np.dot(offsets, observed_primary))
    cosine /= denominator
    sine /= denominator
    scale = math.hypot(cosine, sine)
    if scale > 1e-12:
        cosine /= scale
        sine /= scale
    return math.degrees(math.atan2(sine, cosine))


def _compare_rigid_fit_summaries(
    baseline: dict[str, Any] | None,
    candidate: dict[str, Any] | None,
) -> dict[str, Any] | None:
    if baseline is None or candidate is None:
        return None
    return {
        "negative_means_candidate_is_better": True,
        "absolute_z_rmse_mm_delta": (
            candidate["absolute_z_error_mm"]["rmse_mm"] - baseline["absolute_z_error_mm"]["rmse_mm"]
        ),
        "absolute_3d_rmse_mm_delta": (
            candidate["absolute_3d_error_mm"]["rmse_mm"] - baseline["absolute_3d_error_mm"]["rmse_mm"]
        ),
    }


def _compare_angle_summaries(
    baseline: dict[str, Any] | None,
    candidate: dict[str, Any] | None,
) -> dict[str, Any] | None:
    if baseline is None or candidate is None:
        return None
    return {
        "negative_means_candidate_is_better": True,
        "angle_mae_deg_delta": candidate["angle_error_deg"]["mae_deg"] - baseline["angle_error_deg"]["mae_deg"],
        "angle_rmse_deg_delta": candidate["angle_error_deg"]["rmse_deg"] - baseline["angle_error_deg"]["rmse_deg"],
        "reconstructed_3d_rmse_mm_delta": (
            candidate["reconstructed_3d_error_mm"]["rmse_mm"]
            - baseline["reconstructed_3d_error_mm"]["rmse_mm"]
        ),
    }


def _build_rigid_fit_summary(
    frames: list[EvaluatedFrame],
    object_points: Any,
    rotation_axis: str,
    pivot_coordinate_mm: float,
    fit_transforms: dict[str, tuple[Any, Any]],
) -> dict[str, Any]:
    labels = ("base", "aux_transformed", "fused")
    aggregate_errors: dict[str, dict[str, list[float]]] = {
        label: {
            "absolute_z_error_mm": [],
            "absolute_3d_error_mm": [],
        }
        for label in labels
    }
    per_stem_payloads: list[dict[str, Any]] = []

    for frame in frames:
        modeled_points = _rotated_object_points(
            object_points,
            frame.observation.angle_deg,
            rotation_axis,
            pivot_coordinate_mm,
        )
        stem_payload: dict[str, Any] = {
            "stem": frame.observation.stem,
            "angle_deg": frame.observation.angle_deg,
        }
        for label in labels:
            rotation_matrix, translation_mm = fit_transforms[label]
            expected_points = _apply_rigid_transform(modeled_points, rotation_matrix, translation_mm)
            observed_points = _label_points(frame, label)
            absolute_z_errors = (observed_points[:, 2] - expected_points[:, 2]).tolist()
            absolute_3d_errors = checkerboard.np.linalg.norm(observed_points - expected_points, axis=1).tolist()
            stem_payload[label] = {
                "corner_count": len(absolute_z_errors),
                "absolute_z_error_mm": _signed_error_summary(absolute_z_errors),
                "absolute_3d_error_mm": _magnitude_summary(absolute_3d_errors),
            }
            aggregate_errors[label]["absolute_z_error_mm"].extend(absolute_z_errors)
            aggregate_errors[label]["absolute_3d_error_mm"].extend(absolute_3d_errors)
        per_stem_payloads.append(stem_payload)

    payload: dict[str, Any] = {
        "method": (
            "Known-angle rigid-fit evaluation. A single board pose at angle 0 is fitted per label "
            "using all frames, so no observed sensor is privileged as the reference frame."
        ),
        "by_stem": per_stem_payloads,
    }
    for label in labels:
        rotation_matrix, translation_mm = fit_transforms[label]
        payload[label] = {
            "fit_rotation_matrix": rotation_matrix.tolist(),
            "fit_translation_mm": translation_mm.tolist(),
            "absolute_z_error_mm": _signed_error_summary(aggregate_errors[label]["absolute_z_error_mm"]),
            "absolute_3d_error_mm": _magnitude_summary(aggregate_errors[label]["absolute_3d_error_mm"]),
        }
    payload["fused_minus_base"] = _compare_rigid_fit_summaries(payload["base"], payload["fused"])
    payload["fused_minus_aux_transformed"] = _compare_rigid_fit_summaries(
        payload["aux_transformed"],
        payload["fused"],
    )
    return payload


def _build_angle_only_summary(
    frames: list[EvaluatedFrame],
    object_points: Any,
    rotation_axis: str,
    pivot_coordinate_mm: float,
    fit_transforms: dict[str, tuple[Any, Any]],
    reference_stem: str,
    reference_angle_deg: float,
) -> dict[str, Any]:
    labels = ("base", "aux_transformed", "fused")
    aggregate_errors: dict[str, dict[str, list[float]]] = {
        label: {
            "angle_error_deg": [],
            "reconstructed_3d_error_mm": [],
        }
        for label in labels
    }
    raw_estimated_angles_by_stem: dict[str, dict[str, float]] = {label: {} for label in labels}

    for frame in frames:
        for label in labels:
            rotation_matrix, translation_mm = fit_transforms[label]
            observed_points = _label_points(frame, label)
            local_points = _invert_rigid_transform(observed_points, rotation_matrix, translation_mm)
            raw_estimated_angles_by_stem[label][frame.observation.stem] = _estimate_angle_from_local_points(
                local_points,
                object_points,
                rotation_axis,
                pivot_coordinate_mm,
            )

    reference_raw_angles = {
        label: raw_estimated_angles_by_stem[label][reference_stem]
        for label in labels
    }
    angle_direction_by_label: dict[str, float] = {}
    for label in labels:
        best_direction = 1.0
        best_score = float("inf")
        for direction in (1.0, -1.0):
            errors: list[float] = []
            for frame in frames:
                raw_delta_deg = _wrap_angle_deg(
                    raw_estimated_angles_by_stem[label][frame.observation.stem] - reference_raw_angles[label]
                )
                estimated_angle_deg = reference_angle_deg + direction * raw_delta_deg
                errors.append(_wrap_angle_deg(estimated_angle_deg - frame.observation.angle_deg))
            score = statistics.fmean(error * error for error in errors)
            if score < best_score:
                best_score = score
                best_direction = direction
        angle_direction_by_label[label] = best_direction
    per_stem_payloads: list[dict[str, Any]] = []

    for frame in frames:
        stem_payload: dict[str, Any] = {
            "stem": frame.observation.stem,
            "angle_deg": frame.observation.angle_deg,
        }
        for label in labels:
            rotation_matrix, translation_mm = fit_transforms[label]
            observed_points = _label_points(frame, label)
            raw_estimated_angle_deg = raw_estimated_angles_by_stem[label][frame.observation.stem]
            raw_delta_deg = _wrap_angle_deg(raw_estimated_angle_deg - reference_raw_angles[label])
            estimated_angle_deg = reference_angle_deg + angle_direction_by_label[label] * raw_delta_deg
            angle_error_deg = _wrap_angle_deg(estimated_angle_deg - frame.observation.angle_deg)
            reconstructed_points = _apply_rigid_transform(
                _rotated_object_points(
                    object_points,
                    raw_estimated_angle_deg,
                    rotation_axis,
                    pivot_coordinate_mm,
                ),
                rotation_matrix,
                translation_mm,
            )
            reconstructed_3d_errors = checkerboard.np.linalg.norm(
                observed_points - reconstructed_points,
                axis=1,
            ).tolist()
            stem_payload[label] = {
                "raw_estimated_angle_deg": raw_estimated_angle_deg,
                "estimated_angle_deg": estimated_angle_deg,
                "angle_error_deg": angle_error_deg,
                "reconstructed_3d_error_mm": _magnitude_summary(reconstructed_3d_errors),
            }
            aggregate_errors[label]["angle_error_deg"].append(angle_error_deg)
            aggregate_errors[label]["reconstructed_3d_error_mm"].extend(reconstructed_3d_errors)
        per_stem_payloads.append(stem_payload)

    payload: dict[str, Any] = {
        "method": (
            "Angle-only evaluation. After the global rigid-fit pose is estimated per label, each frame's "
            "rotation angle is re-estimated from the observed board shape. Because absolute angle has an "
            "unidentifiable constant offset, each label is aligned so that its own reference_stem matches the "
            "known reference angle."
        ),
        "reference_stem": reference_stem,
        "reference_angle_deg": reference_angle_deg,
        "reference_raw_angle_deg_by_label": reference_raw_angles,
        "angle_direction_by_label": angle_direction_by_label,
        "by_stem": per_stem_payloads,
    }
    for label in labels:
        rotation_matrix, translation_mm = fit_transforms[label]
        payload[label] = {
            "fit_rotation_matrix": rotation_matrix.tolist(),
            "fit_translation_mm": translation_mm.tolist(),
            "angle_error_deg": _signed_error_summary_deg(aggregate_errors[label]["angle_error_deg"]),
            "reconstructed_3d_error_mm": _magnitude_summary(
                aggregate_errors[label]["reconstructed_3d_error_mm"]
            ),
        }
    payload["fused_minus_base"] = _compare_angle_summaries(payload["base"], payload["fused"])
    payload["fused_minus_aux_transformed"] = _compare_angle_summaries(
        payload["aux_transformed"],
        payload["fused"],
    )
    return payload


def _compare_measurement_summaries(
    baseline: dict[str, Any] | None,
    candidate: dict[str, Any] | None,
) -> dict[str, Any] | None:
    if baseline is None or candidate is None:
        return None
    return {
        "negative_means_candidate_is_better": True,
        "z_change_mae_mm_delta": (
            candidate["z_change_error_mm"]["mae_mm"] - baseline["z_change_error_mm"]["mae_mm"]
        ),
        "z_change_rmse_mm_delta": (
            candidate["z_change_error_mm"]["rmse_mm"] - baseline["z_change_error_mm"]["rmse_mm"]
        ),
        "absolute_z_rmse_mm_delta": (
            candidate["absolute_z_error_mm"]["rmse_mm"] - baseline["absolute_z_error_mm"]["rmse_mm"]
        ),
        "absolute_3d_rmse_mm_delta": (
            candidate["absolute_3d_error_mm"]["rmse_mm"] - baseline["absolute_3d_error_mm"]["rmse_mm"]
        ),
    }


def _print_summary(title: str, payload: dict[str, Any], indent: str = "") -> None:
    if title:
        print(title)
    for key, value in payload.items():
        if isinstance(value, dict):
            print(f"{indent}{key}:")
            _print_summary("", value, indent + "  ")
        else:
            rendered = f"{value:.3f}" if isinstance(value, float) else str(value)
            print(f"{indent}- {key}: {rendered}")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Evaluate fused checkerboard coordinates against known rotation angles."
    )
    parser.add_argument(
        "--evaluation-config",
        type=Path,
        help="Optional JSON file containing evaluation inputs such as paths, checkerboard settings, and angle CSV paths.",
    )
    parser.add_argument("--base-dir", type=Path, help="Directory containing base checkerboard images.")
    parser.add_argument("--aux-dir", type=Path, help="Directory containing aux checkerboard images.")
    parser.add_argument(
        "--rig-calibration",
        type=Path,
        help="JSON file containing base/aux color intrinsics and color<->depth extrinsics.",
    )
    parser.add_argument(
        "--transform-config",
        type=Path,
        help="Tracking config JSON containing aux_translation_mm and rotation_matrix.",
    )
    parser.add_argument(
        "--angles-csv",
        type=Path,
        help="CSV mapping each checkerboard image stem to a known rotation angle in degrees.",
    )
    parser.add_argument("--board-cols", type=int, help="Checkerboard inner-corner count along columns.")
    parser.add_argument("--board-rows", type=int, help="Checkerboard inner-corner count along rows.")
    parser.add_argument("--square-size-mm", type=float, help="Checkerboard square size in millimeters.")
    parser.add_argument(
        "--base-corner-order",
        choices=("normal", "reverse"),
        help="Corner order to use for base images.",
    )
    parser.add_argument(
        "--aux-corner-order",
        choices=("normal", "reverse"),
        help="Corner order to use for aux images.",
    )
    parser.add_argument("--angle-stem-col", help="Stem column name inside the angle CSV. Default: stem")
    parser.add_argument(
        "--angle-deg-col",
        help="Angle column name inside the angle CSV. Default: angle_deg",
    )
    parser.add_argument(
        "--reference-stem",
        help="Optional reference capture stem. Default: the stem whose angle is closest to 0 degrees.",
    )
    parser.add_argument(
        "--rotation-axis",
        choices=("board_x", "board_y"),
        help="Fixed board axis used by the rotation rig. Default: board_y.",
    )
    parser.add_argument(
        "--pivot-coordinate-mm",
        type=float,
        help="Board-local coordinate of the fixed rotation axis. Defaults to the checkerboard center line.",
    )
    parser.add_argument(
        "--output-json",
        type=Path,
        help="Optional path to write a detailed evaluation JSON report.",
    )
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    checkerboard._import_dependencies()
    settings = _resolve_settings(args)

    base_calibration, aux_calibration = checkerboard._load_rig_calibration(settings.rig_calibration)
    aux_translation_mm, aux_rotation_matrix = _load_aux_transform(settings.transform_config)
    angle_entries = _load_angle_entries(
        settings.angles_csv,
        settings.angle_stem_col,
        settings.angle_deg_col,
    )

    base_index = checkerboard._build_image_index(settings.base_dir)
    aux_index = checkerboard._build_image_index(settings.aux_dir)
    shared_stems = sorted(set(base_index) & set(aux_index))
    if not shared_stems:
        raise SystemExit("No paired checkerboard images were found. Match files by stem across both directories.")

    missing_in_aux = sorted(set(base_index) - set(aux_index))
    missing_in_base = sorted(set(aux_index) - set(base_index))
    if missing_in_aux:
        print(
            f"Skipping {len(missing_in_aux)} base-only images: {', '.join(missing_in_aux[:5])}",
            file=sys.stderr,
        )
    if missing_in_base:
        print(
            f"Skipping {len(missing_in_base)} aux-only images: {', '.join(missing_in_base[:5])}",
            file=sys.stderr,
        )

    object_points = checkerboard._make_board_object_points(
        settings.board_cols,
        settings.board_rows,
        settings.square_size_mm,
    ).astype(checkerboard.np.float64)
    pattern_size = (settings.board_cols, settings.board_rows)

    stems_missing_angles = sorted(stem for stem in shared_stems if stem not in angle_entries)
    if stems_missing_angles:
        print(
            f"Skipping {len(stems_missing_angles)} paired images without angle metadata: "
            f"{', '.join(stems_missing_angles[:5])}",
            file=sys.stderr,
        )

    unused_angle_entries = sorted(stem for stem in angle_entries if stem not in set(shared_stems))
    if unused_angle_entries:
        print(
            f"Ignoring {len(unused_angle_entries)} angle rows without matching paired images: "
            f"{', '.join(unused_angle_entries[:5])}",
            file=sys.stderr,
        )

    observations: list[CheckerboardObservation] = []
    detection_failures: list[str] = []
    for stem in shared_stems:
        angle_entry = angle_entries.get(stem)
        if angle_entry is None:
            continue
        observation = _estimate_observation_for_pair(
            object_points,
            pattern_size,
            base_index[stem],
            aux_index[stem],
            base_calibration,
            aux_calibration,
            settings.base_corner_order,
            settings.aux_corner_order,
            angle_entry.angle_deg,
        )
        if observation is None:
            detection_failures.append(stem)
            continue
        observations.append(observation)

    if detection_failures:
        print(
            f"Skipping {len(detection_failures)} stems where checkerboard detection failed: "
            f"{', '.join(detection_failures[:5])}",
            file=sys.stderr,
        )

    if not observations:
        raise SystemExit("Checkerboard detection failed for every paired image.")

    observations.sort(key=lambda item: (item.angle_deg, item.stem))

    if settings.reference_stem is not None:
        reference_observation = next((item for item in observations if item.stem == settings.reference_stem), None)
        if reference_observation is None:
            raise SystemExit(
                f"Reference stem `{settings.reference_stem}` was not found among the evaluated observations."
            )
    else:
        reference_observation = min(observations, key=lambda item: (abs(item.angle_deg), item.stem))

    pivot_coordinate_mm = (
        settings.pivot_coordinate_mm
        if settings.pivot_coordinate_mm is not None
        else _default_pivot_coordinate_mm(settings)
    )

    reference_base_points = _points_from_pose(
        reference_observation.base_rotation_depth_board,
        reference_observation.base_translation_depth_board_mm,
        object_points,
    )
    reference_aux_points_in_base = _transform_points_to_base(
        _points_from_pose(
            reference_observation.aux_rotation_depth_board,
            reference_observation.aux_translation_depth_board_mm,
            object_points,
        ),
        aux_rotation_matrix,
        aux_translation_mm,
    )
    reference_fused_points = _fuse_points(reference_base_points, reference_aux_points_in_base)
    evaluated_frames: list[EvaluatedFrame] = []
    for observation in observations:
        base_points = _points_from_pose(
            observation.base_rotation_depth_board,
            observation.base_translation_depth_board_mm,
            object_points,
        )
        aux_points_in_base = _transform_points_to_base(
            _points_from_pose(
                observation.aux_rotation_depth_board,
                observation.aux_translation_depth_board_mm,
                object_points,
            ),
            aux_rotation_matrix,
            aux_translation_mm,
        )
        evaluated_frames.append(
            EvaluatedFrame(
                observation=observation,
                base_points=base_points,
                aux_transformed_points=aux_points_in_base,
                fused_points=_fuse_points(base_points, aux_points_in_base),
            )
        )

    aggregate_errors: dict[str, dict[str, list[float]]] = {
        "base": {
            "z_change_error_mm": [],
            "absolute_z_error_mm": [],
            "absolute_3d_error_mm": [],
        },
        "aux_transformed": {
            "z_change_error_mm": [],
            "absolute_z_error_mm": [],
            "absolute_3d_error_mm": [],
        },
        "fused": {
            "z_change_error_mm": [],
            "absolute_z_error_mm": [],
            "absolute_3d_error_mm": [],
        },
    }
    per_stem_payloads: list[dict[str, Any]] = []
    expected_reference_points = reference_base_points

    for frame in evaluated_frames:
        observation = frame.observation
        expected_points = _expected_points_from_reference(
            reference_observation,
            object_points,
            observation.angle_deg,
            settings.rotation_axis,
            pivot_coordinate_mm,
        )

        stem_payload: dict[str, Any] = {
            "stem": observation.stem,
            "angle_deg": observation.angle_deg,
            "base_image": observation.base_image,
            "aux_image": observation.aux_image,
            "base_reprojection_error_px": observation.base_reprojection_error_px,
            "aux_reprojection_error_px": observation.aux_reprojection_error_px,
            "expected_z_change_min_mm": float((expected_points[:, 2] - expected_reference_points[:, 2]).min()),
            "expected_z_change_max_mm": float((expected_points[:, 2] - expected_reference_points[:, 2]).max()),
        }

        for label, observed_points, reference_points in (
            ("base", frame.base_points, reference_base_points),
            ("aux_transformed", frame.aux_transformed_points, reference_aux_points_in_base),
            ("fused", frame.fused_points, reference_fused_points),
        ):
            summary, arrays = _point_error_summary(
                observed_points,
                reference_points,
                expected_points,
                expected_reference_points,
            )
            stem_payload[label] = summary
            for metric_name, values in arrays.items():
                aggregate_errors[label][metric_name].extend(values)

        per_stem_payloads.append(stem_payload)

    overall_summary: dict[str, Any] = {}
    for label, metric_values in aggregate_errors.items():
        overall_summary[label] = {
            "z_change_error_mm": _signed_error_summary(metric_values["z_change_error_mm"]),
            "absolute_z_error_mm": _signed_error_summary(metric_values["absolute_z_error_mm"]),
            "absolute_3d_error_mm": _magnitude_summary(metric_values["absolute_3d_error_mm"]),
        }

    overall_summary["fused_minus_base"] = _compare_measurement_summaries(
        overall_summary["base"],
        overall_summary["fused"],
    )
    overall_summary["fused_minus_aux_transformed"] = _compare_measurement_summaries(
        overall_summary["aux_transformed"],
        overall_summary["fused"],
    )
    fit_transforms = {
        label: _fit_global_label_pose(
            evaluated_frames,
            label,
            object_points,
            settings.rotation_axis,
            pivot_coordinate_mm,
        )
        for label in ("base", "aux_transformed", "fused")
    }
    rigid_fit_summary = _build_rigid_fit_summary(
        evaluated_frames,
        object_points,
        settings.rotation_axis,
        pivot_coordinate_mm,
        fit_transforms,
    )
    angle_only_summary = _build_angle_only_summary(
        evaluated_frames,
        object_points,
        settings.rotation_axis,
        pivot_coordinate_mm,
        fit_transforms,
        reference_observation.stem,
        reference_observation.angle_deg,
    )

    evaluation_payload = {
        "base_dir": str(settings.base_dir),
        "aux_dir": str(settings.aux_dir),
        "rig_calibration": str(settings.rig_calibration),
        "transform_config": str(settings.transform_config),
        "angles_csv": str(settings.angles_csv),
        "board_cols": settings.board_cols,
        "board_rows": settings.board_rows,
        "square_size_mm": settings.square_size_mm,
        "base_corner_order": settings.base_corner_order,
        "aux_corner_order": settings.aux_corner_order,
        "rotation_axis": settings.rotation_axis,
        "pivot_coordinate_mm": pivot_coordinate_mm,
        "paired_stems_found": len(shared_stems),
        "paired_stems_evaluated": len(observations),
        "paired_stems_detection_failed": detection_failures,
        "reference_stem": reference_observation.stem,
        "reference_angle_deg": reference_observation.angle_deg,
        "overall": overall_summary,
        "by_stem": per_stem_payloads,
        "base_anchored": {
            "method": (
                "Legacy base-anchored evaluation. The theoretical board coordinates are generated "
                "from the reference base pose, so base is implicitly privileged."
            ),
            "reference_stem": reference_observation.stem,
            "reference_angle_deg": reference_observation.angle_deg,
            "overall": overall_summary,
            "by_stem": per_stem_payloads,
        },
        "rigid_fit": rigid_fit_summary,
        "angle_only": angle_only_summary,
    }

    print("Checkerboard Fusion Evaluation")
    print(f"- base_dir: {settings.base_dir}")
    print(f"- aux_dir: {settings.aux_dir}")
    print(f"- rig_calibration: {settings.rig_calibration}")
    print(f"- transform_config: {settings.transform_config}")
    print(f"- angles_csv: {settings.angles_csv}")
    print(f"- paired_stems_found: {len(shared_stems)}")
    print(f"- paired_stems_evaluated: {len(observations)}")
    print(f"- reference_stem: {reference_observation.stem}")
    print(f"- reference_angle_deg: {reference_observation.angle_deg:.3f}")
    print(f"- rotation_axis: {settings.rotation_axis}")
    print(f"- pivot_coordinate_mm: {pivot_coordinate_mm:.3f}")
    print()
    _print_summary("Base-Anchored Error Summary (Legacy)", overall_summary)
    print()
    _print_summary("Rigid-Fit Summary", rigid_fit_summary)
    print()
    _print_summary("Angle-Only Summary", angle_only_summary)
    print()
    print("Per-stem base-anchored depth-change RMSE [mm]")
    for stem_payload in per_stem_payloads:
        print(
            "  - "
            f"{stem_payload['stem']} angle={stem_payload['angle_deg']:.3f}: "
            f"base={stem_payload['base']['z_change_error_mm']['rmse_mm']:.3f}, "
            f"aux_transformed={stem_payload['aux_transformed']['z_change_error_mm']['rmse_mm']:.3f}, "
            f"fused={stem_payload['fused']['z_change_error_mm']['rmse_mm']:.3f}"
        )
    print()
    print("Per-stem angle estimate error [deg]")
    for stem_payload in angle_only_summary["by_stem"]:
        print(
            "  - "
            f"{stem_payload['stem']} angle={stem_payload['angle_deg']:.3f}: "
            f"base={stem_payload['base']['angle_error_deg']:.3f}, "
            f"aux_transformed={stem_payload['aux_transformed']['angle_error_deg']:.3f}, "
            f"fused={stem_payload['fused']['angle_error_deg']:.3f}"
        )
    print()
    print("Per-stem rigid-fit 3D RMSE [mm]")
    for stem_payload in rigid_fit_summary["by_stem"]:
        print(
            "  - "
            f"{stem_payload['stem']} angle={stem_payload['angle_deg']:.3f}: "
            f"base={stem_payload['base']['absolute_3d_error_mm']['rmse_mm']:.3f}, "
            f"aux_transformed={stem_payload['aux_transformed']['absolute_3d_error_mm']['rmse_mm']:.3f}, "
            f"fused={stem_payload['fused']['absolute_3d_error_mm']['rmse_mm']:.3f}"
        )

    if settings.output_json is not None:
        settings.output_json.parent.mkdir(parents=True, exist_ok=True)
        with settings.output_json.open("w", encoding="utf-8", newline="\n") as handle:
            json.dump(evaluation_payload, handle, ensure_ascii=False, indent=2)
            handle.write("\n")
        print()
        print(f"Wrote JSON summary: {settings.output_json}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
