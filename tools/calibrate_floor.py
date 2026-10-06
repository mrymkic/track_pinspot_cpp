"""Fit a measured-height floor and a dual-Kinect transform from saved captures."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np


def vector(value, shape, label):
    result = np.asarray(value, dtype=np.float64)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"{label} must contain finite values with shape {shape}.")
    return result


def fit_floor(points, reference_height_mm, up_hint):
    points = vector(points, (len(points), 3), "Reference points")
    if len(points) < 3:
        raise ValueError("At least three horizontal positions are required.")
    if not np.isfinite(reference_height_mm) or reference_height_mm <= 0:
        raise ValueError("The measured reference height must be positive.")
    center = points.mean(axis=0)
    _, singular, vt = np.linalg.svd(points - center, full_matrices=False)
    if singular[1] < 50:
        raise ValueError("Positions are too close or nearly collinear; move in two floor directions.")
    up = vt[-1]
    if np.dot(up, up_hint) < 0:
        up = -up
    if np.dot(up, up_hint) < 0.2:
        raise ValueError("The fitted floor disagrees with the configured sensor mounting orientation.")
    height = float(reference_height_mm - up @ center)
    residuals = (points - center) @ up
    rms = float(np.sqrt(np.mean(residuals**2)))
    if rms > 10 or np.max(np.abs(residuals)) > 20:
        raise ValueError("Reference heights do not form a plane: keep the marked point at the measured height.")
    if height <= 0:
        raise ValueError("The fitted camera is below the floor; check the marked bottom reference and orientation.")
    forward = np.array([0., 0., 1.]) - up * up[2]
    if np.linalg.norm(forward) < 0.1:
        raise ValueError("Camera looks vertically at the floor; a separate forward direction is required.")
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, up)
    # Y-up with X-right/Z-forward changes handedness from the SDK's Y-down basis.
    basis = np.stack([right, up, forward])
    return dict(basis_matrix=basis.tolist(), translation_mm=[0., height, 0.],
                up_in_depth=up.tolist(), camera_height_mm=height,
                reference_plane_rms_mm=rms, reference_plane_residuals_mm=residuals.tolist(),
                position_singular_values_mm=singular.tolist())


def fit_rigid(source, target):
    source = vector(source, (len(source), 3), "Aux points")
    target = vector(target, source.shape, "Base points")
    centered_source, centered_target = source - source.mean(0), target - target.mean(0)
    u, _, vt = np.linalg.svd(centered_source.T @ centered_target)
    rotation = vt.T @ np.diag([1., 1., np.linalg.det(vt.T @ u.T)]) @ u.T
    translation = target.mean(0) - rotation @ source.mean(0)
    errors = np.linalg.norm(source @ rotation.T + translation - target, axis=1)
    return rotation, translation, errors


def load_dataset(directory):
    session = json.loads((directory / "session.json").read_text(encoding="utf-8"))
    if session.get("dataset_kind") != "floor_horizontal_translation":
        raise ValueError("Only measured-height photography sessions can calibrate the floor.")
    if session.get("status") != "closed":
        raise ValueError("Finish photography and close its window before calibration.")
    files = sorted((directory / "captures").glob("*.json"))
    if len(files) != session["captures_saved"] or len(files) < 3:
        raise ValueError("Capture count is inconsistent or fewer than three positions were saved.")
    captures = [json.loads(path.read_text(encoding="utf-8")) for path in files]
    return session, captures


def calibrate(session, captures, session_name):
    height = float(session["reference_height_mm"])
    board = session["checkerboard"]
    square = float(board["square_size_mm"])
    cols, rows = int(board["board_cols"]), int(board["board_rows"])
    if square <= 0 or not np.isfinite(square) or cols < 2 or rows < 2:
        raise ValueError("Invalid checkerboard dimensions.")
    local = np.array([[x * square, y * square, 0.] for y in range(rows) for x in range(cols)])
    ref_local = vector(session["reference"]["point_board_mm"], (3,), "Board reference")
    refs = {role: [] for role in ("base", "aux")}
    corners = {role: [] for role in refs}
    reprojection, sync = [], []
    for capture in captures:
        if abs(float(capture["reference_height_mm"]) - height) > 0.01:
            raise ValueError("Reference height changed within the session.")
        sync_error = float(capture["sync_error_us"])
        if not np.isfinite(sync_error) or abs(sync_error) > 2000:
            raise ValueError("Unsynchronized camera pair found.")
        sync.append(sync_error)
        for role in refs:
            observation = capture[role]["checkerboard"]
            error = float(observation["reprojection_error_px"])
            if not np.isfinite(error) or error > 2 or error < 0:
                raise ValueError("A checkerboard pose has excessive reprojection error.")
            rotation = vector(observation["board_to_depth_rotation"], (3, 3), "Board rotation")
            translation = vector(observation["board_to_depth_translation_mm"], (3,), "Board translation")
            if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5) or np.linalg.det(rotation) < 0.999:
                raise ValueError("Invalid board pose rotation.")
            if observation["reference_kind"] != session["reference"]["kind"] or not np.allclose(
                    observation["reference_point_board_mm"], ref_local, atol=0.01):
                raise ValueError("The physically marked reference changed within the session.")
            ref = vector(observation["reference_point_depth_mm"], (3,), "Depth reference")
            if not np.allclose(ref, rotation @ ref_local + translation, atol=0.01):
                raise ValueError("Stored reference disagrees with the board pose.")
            refs[role].append(ref)
            corners[role].extend(local @ rotation.T + translation)
            reprojection.append(error)
    floors = {}
    for role in refs:
        preview_rotation = session["settings"][f"{role}_preview_rotation_deg"]
        if preview_rotation not in (0, 180):
            raise ValueError("Unsupported sensor mounting rotation.")
        floors[role] = fit_floor(refs[role], height, [0., 1. if preview_rotation == 180 else -1., 0.])
    # Fit all matched physical inner corners, preserving a proper rigid transform.
    rotation, translation, corner_errors = fit_rigid(corners["aux"], corners["base"])
    reference_errors = np.linalg.norm(np.asarray(refs["aux"]) @ rotation.T + translation - refs["base"], axis=1)
    if np.max(reference_errors) > 30 or np.sqrt(np.mean(corner_errors**2)) > 20:
        raise ValueError("The two camera poses disagree; verify the same reference and corner order in both views.")
    basis = np.asarray(floors["base"]["basis_matrix"])
    offset = np.asarray(floors["base"]["translation_mm"])
    aux_origin = basis @ translation + offset
    virtual_origin = aux_origin * np.array([1., 0., 1.])
    # Columns are the virtual AUX axes expressed in the BASE floor frame.
    yaw90 = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
    aux_reference_floor = np.asarray(refs["aux"]) @ rotation.T @ basis.T + basis @ translation + offset
    return dict(
        schema="floor-coordinate-calibration-v1", created_at=datetime.now(timezone.utc).isoformat(),
        source_session=session_name, capture_stems=[capture["stem"] for capture in captures],
        base_kinect_serial=session["settings"]["base_kinect_serial"],
        aux_kinect_serial=session["settings"]["aux_kinect_serial"],
        reference_height_mm=height, checkerboard=board, reference=session["reference"],
        axes=dict(x="BASE right projected onto the floor", y="vertical height above floor", z="BASE forward projected onto the floor"),
        origin="Vertical projection of the BASE depth-camera origin onto the floor",
        base_depth_to_floor=floors["base"], aux_independent_floor_check=floors["aux"],
        aux_depth_to_base_depth=dict(rotation_matrix=rotation.tolist(), translation_mm=translation.tolist(),
                                    fit_method="Rigid least squares over matched inner corners from all board poses"),
        virtual_aux_90=dict(yaw_deg=90., origin_in_base_floor_mm=virtual_origin.tolist(),
                            rotation_to_base_floor=yaw90.tolist(),
                            aux_depth_basis_matrix=(yaw90.T @ basis @ rotation).tolist(),
                            aux_depth_translation_mm=(yaw90.T @ (basis @ translation + offset - virtual_origin)).tolist()),
        quality=dict(capture_count=len(captures), max_reprojection_error_px=max(reprojection),
                     max_sync_error_us=max(abs(error) for error in sync),
                     corner_alignment_rms_mm=float(np.sqrt(np.mean(corner_errors**2))),
                     reference_alignment_errors_mm=reference_errors.tolist(),
                     base_reference_floor_mm=(np.asarray(refs["base"]) @ basis.T + offset).tolist(),
                     aux_reference_floor_mm=aux_reference_floor.tolist()),
        sensor_orientation={role: "flip180" if session["settings"][f"{role}_preview_rotation_deg"] == 180 else "default" for role in refs},
    )


def tracking_profile(template, calibration):
    profile = dict(template)
    for role in ("base", "aux"):
        if profile.get(f"{role}_kinect_serial") not in (None, "", calibration[f"{role}_kinect_serial"]):
            raise ValueError("Template serials disagree with the photographed cameras.")
        profile[f"{role}_kinect_serial"] = calibration[f"{role}_kinect_serial"]
        profile[f"{role}_sensor_orientation"] = calibration["sensor_orientation"][role]
    profile.pop("aux_rotation_matrix", None)
    profile["rotation_matrix"] = calibration["aux_depth_to_base_depth"]["rotation_matrix"]
    profile["aux_translation_mm"] = calibration["aux_depth_to_base_depth"]["translation_mm"]
    profile["coordinate_output_frame"] = "base_floor"
    profile["floor_basis_matrix"] = calibration["base_depth_to_floor"]["basis_matrix"]
    profile["floor_translation_mm"] = calibration["base_depth_to_floor"]["translation_mm"]
    profile["virtual_aux_origin_floor_mm"] = calibration["virtual_aux_90"]["origin_in_base_floor_mm"]
    profile["coordinate_output_dir"] = "floor_tracking"
    profile["floor_calibration_session"] = calibration["source_session"]
    return profile


def write_json(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--template", type=Path)
    parser.add_argument("--config-out", type=Path)
    args = parser.parse_args()
    if bool(args.template) != bool(args.config_out):
        parser.error("--template and --config-out must be specified together")
    session, captures = load_dataset(args.session)
    calibration = calibrate(session, captures, args.session.name)
    profile = tracking_profile(json.loads(args.template.read_text(encoding="utf-8")), calibration) if args.template else None
    write_json(args.output, calibration)
    if profile is not None:
        write_json(args.config_out, profile)
    floor, quality = calibration["base_depth_to_floor"], calibration["quality"]
    print(f"{quality['capture_count']} positions; reference plane RMS {floor['reference_plane_rms_mm']:.3f} mm")
    print(f"Matched corner RMS {quality['corner_alignment_rms_mm']:.3f} mm; max sync error {quality['max_sync_error_us']:.0f} us")
    print(f"BASE camera height {floor['camera_height_mm']:.1f} mm; axes X right, Y height, Z forward")
    print(f"Calibration: {args.output}")
    if args.config_out:
        print(f"Tracking profile: {args.config_out}")


if __name__ == "__main__":
    main()
