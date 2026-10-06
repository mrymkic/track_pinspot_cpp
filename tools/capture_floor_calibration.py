"""Capture a constant-height checkerboard at multiple horizontal positions."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone, timedelta
import json
import math
from pathlib import Path
import subprocess
import time

import cv2
import numpy as np

from calibrate_checkerboard_extrinsics import (
    _apply_corner_order, _compose_color_pose_into_depth, _import_dependencies, _load_rig_calibration,
    _make_board_object_points, _solve_checkerboard_pose,
)
from kinect_capture import DualKinectCapture
from floor_capture_viewer import CaptureViewer

JST = timezone(timedelta(hours=9))
_import_dependencies()


def write_json(path, payload):
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    temporary.replace(path)


def read_settings(path):
    settings = json.loads(path.read_text(encoding="utf-8"))
    for key in ("output_root", "sdk_directory", "rig_exporter"):
        settings[key] = str((path.parent / settings[key]).resolve())
    for key in ("board_squares_cols", "board_squares_rows"):
        if not isinstance(settings[key], int) or isinstance(settings[key], bool) or settings[key] < 3:
            raise ValueError(f"{key} must be an integer of at least 3.")
    settings["board_cols"] = settings["board_squares_cols"] - 1
    settings["board_rows"] = settings["board_squares_rows"] - 1
    for key in ("square_size_mm", "max_reprojection_error_px"):
        if not math.isfinite(settings[key]) or settings[key] <= 0:
            raise ValueError(f"{key} must be positive and finite.")
    kind = settings.setdefault("reference_point_kind", "inner_corner")
    if kind == "inner_corner":
        index = settings["reference_corner_index"]
        if not isinstance(index, int) or isinstance(index, bool) or not 0 <= index < settings["board_cols"] * settings["board_rows"]:
            raise ValueError("reference_corner_index must identify an inner corner.")
    elif kind == "bottom_edge_center":
        margin = settings.setdefault("reference_bottom_margin_mm", 0.0)
        if margin is not None and (not math.isfinite(margin) or margin < 0):
            raise ValueError("reference_bottom_margin_mm must be nonnegative and finite.")
    else:
        raise ValueError("reference_point_kind must be inner_corner or bottom_edge_center.")
    for role in ("base", "aux"):
        if settings[f"{role}_preview_rotation_deg"] not in (0, 180):
            raise ValueError("Preview rotation must be 0 or 180 degrees.")
        if settings[f"{role}_corner_order"] not in ("normal", "reverse", "auto_bottom", "auto_bottom_reversed"):
            raise ValueError("Corner order must be normal, reverse, auto_bottom, or auto_bottom_reversed.")
    return settings


def validate_height(height):
    if height is None or not math.isfinite(height) or height <= 0:
        raise ValueError("Measure the marked reference point's vertical height from the floor in millimeters.")
    return float(height)


def reference_geometry(settings):
    if settings["reference_point_kind"] == "bottom_edge_center":
        margin = settings["reference_bottom_margin_mm"]
        if margin is None or not math.isfinite(margin) or margin < 0:
            raise ValueError("Measure the margin from the printed grid's lower edge to the board's lower edge in millimeters.")
        point = [(settings["board_cols"] - 1) * settings["square_size_mm"] / 2,
                 settings["board_rows"] * settings["square_size_mm"] + margin, 0.0]
        label = "REF BOTTOM"
        description = "白黒格子の下端中央" if margin == 0 else f"台紙・板の下端中央（格子の下端から{margin:g}mm）"
        definition = "Center of the physically marked bottom edge; vertical height measured from the floor."
    else:
        index = settings["reference_corner_index"]
        point = [(index % settings["board_cols"]) * settings["square_size_mm"],
                 (index // settings["board_cols"]) * settings["square_size_mm"], 0.0]
        label = f"REF {index}"
        description = f"内側コーナー{index}"
        definition = "The same physically marked inner corner; vertical height measured from the floor."
    return dict(kind=settings["reference_point_kind"], point_board_mm=point, label=label, description=description, definition=definition)


def preview_pixels(raw_bgra, rotation):
    bgr = cv2.cvtColor(raw_bgra, cv2.COLOR_BGRA2BGR)
    return cv2.rotate(bgr, cv2.ROTATE_180) if rotation == 180 else bgr


def preview_to_raw_corners(corners, width, height, rotation):
    result = corners.copy()
    if rotation == 180:
        result[:, 0] = width - 1 - result[:, 0]
        result[:, 1] = height - 1 - result[:, 1]
    return result


def detect_board(preview, settings, corner_order):
    height, width = preview.shape[:2]
    scale = min(1.0, 960 / width)
    small = cv2.resize(preview, (round(width * scale), round(height * scale))) if scale < 1 else preview
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    pattern = (settings["board_cols"], settings["board_rows"])
    found, corners = cv2.findChessboardCornersSB(gray, pattern, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        found, corners = cv2.findChessboardCorners(gray, pattern, flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK)
    if not found or corners is None:
        return None
    corners = ((corners.reshape(-1, 2) + 0.5) / scale - 0.5).astype(np.float32)
    full_gray = cv2.cvtColor(preview, cv2.COLOR_BGR2GRAY)
    cv2.cornerSubPix(full_gray, corners, (5, 5), (-1, -1),
                     (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    if corner_order.startswith("auto_bottom"):
        rows = corners.reshape(settings["board_rows"], settings["board_cols"], 2)
        if rows[0, :, 1].mean() > rows[-1, :, 1].mean():
            corners = corners[::-1].copy()
        corner_order = "reverse" if corner_order == "auto_bottom_reversed" else "normal"
    return _apply_corner_order(corners, corner_order)


def board_observation(frame, calibration, settings, role, order):
    rotation = settings[f"{role}_preview_rotation_deg"]
    preview = preview_pixels(frame.color, rotation)
    corners = detect_board(preview, settings, order)
    if corners is None:
        return preview, None, None
    raw_corners = preview_to_raw_corners(corners, preview.shape[1], preview.shape[0], rotation)
    points = _make_board_object_points(settings["board_cols"], settings["board_rows"], settings["square_size_mm"])
    color_r, color_t, error = _solve_checkerboard_pose(points, raw_corners, calibration)
    depth_r, depth_t = _compose_color_pose_into_depth(color_r, color_t, calibration)
    geometry = reference_geometry(settings)
    reference_board = np.asarray(geometry["point_board_mm"], dtype=np.float64)
    reference = depth_r @ reference_board + depth_t
    reference_raw_px, _ = cv2.projectPoints(reference_board.reshape(1, 3), cv2.Rodrigues(color_r)[0], color_t,
                                           calibration.color_camera_matrix, calibration.color_distortion_coeffs)
    observation = dict(
        corner_order=order, reprojection_error_px=error,
        raw_color_corners_px=raw_corners.tolist(),
        board_to_depth_rotation=depth_r.tolist(), board_to_depth_translation_mm=depth_t.tolist(),
        reference_point_depth_mm=reference.tolist(), reference_point_board_mm=reference_board.tolist(),
        reference_point_raw_color_px=reference_raw_px.reshape(2).tolist(), reference_kind=geometry["kind"],
    )
    return preview, corners, observation


def create_session(settings, height, kind="floor_horizontal_translation"):
    root = Path(settings["output_root"])
    root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(JST).strftime("%Y%m%d_%H%M%S_%f")
    prefix = {"floor_horizontal_translation": "", "hardware_smoke_test": "smoke_", "alignment_preview": "preview_"}[kind]
    session = root / (prefix + stamp)
    session.mkdir()
    for name in ("base", "aux_color", "base_depth", "aux_depth", "captures"):
        (session / name).mkdir()
    rig = session / "rig_calibration.json"
    exporter = Path(settings["rig_exporter"])
    if not exporter.is_file():
        raise FileNotFoundError(f"Build export_kinect_rig_calibration first: {exporter}")
    subprocess.run([
        str(exporter), "--output", str(rig),
        "--base-serial", settings["base_kinect_serial"], "--aux-serial", settings["aux_kinect_serial"],
        "--depth-mode", "wfov_2x2binned", "--color-resolution", "720p",
    ], check=True)
    payload = dict(
        schema="floor-capture-session-v2", dataset_kind=kind,
        created_at=datetime.now(JST).isoformat(), reference_height_mm=height,
        checkerboard=dict(board_cols=settings["board_cols"], board_rows=settings["board_rows"],
                          squares_cols=settings["board_squares_cols"], squares_rows=settings["board_squares_rows"],
                          square_size_mm=settings["square_size_mm"]),
        reference=reference_geometry(settings),
        original_images_preserved=True, rig_calibration="rig_calibration.json",
        settings=settings, captures_saved=0, status="prepared",
    )
    write_json(session / "session.json", payload)
    return session, payload


def frame_metadata(frame):
    return dict(sequence=frame.sequence, color_timestamp_us=frame.color_timestamp_us,
                depth_timestamp_us=frame.depth_timestamp_us, depth_system_timestamp_ns=frame.depth_system_timestamp_ns)


def save_capture(session, index, pair, observations, height):
    stem = f"{index:04d}"
    paths = {"base": session / "base" / (stem + ".png"),
             "aux": session / "aux_color" / (stem + ".png"),
             "base_depth": session / "base_depth" / (stem + ".png"),
             "aux_depth": session / "aux_depth" / (stem + ".png"),
             "metadata": session / "captures" / (stem + ".json")}
    if any(path.exists() for path in paths.values()):
        raise FileExistsError(f"Capture {stem} already exists; no files overwritten.")
    payload = dict(stem=stem, captured_at=datetime.now(JST).isoformat(), reference_height_mm=height,
                   expected_depth_delta_us=pair.expected_depth_delta_us, sync_error_us=pair.sync_error_us,
                   images={key: path.relative_to(session).as_posix() for key, path in paths.items() if key != "metadata"},
                   base={**frame_metadata(pair.base), "checkerboard": observations.get("base")},
                   aux={**frame_metadata(pair.aux), "checkerboard": observations.get("aux")})
    if all(observations.get(role) is not None for role in ("base", "aux")):
        base_r = np.asarray(observations["base"]["board_to_depth_rotation"])
        aux_r = np.asarray(observations["aux"]["board_to_depth_rotation"])
        base_t = np.asarray(observations["base"]["board_to_depth_translation_mm"])
        aux_t = np.asarray(observations["aux"]["board_to_depth_translation_mm"])
        rotation = base_r @ aux_r.T
        payload["aux_to_base"] = dict(rotation_matrix=rotation.tolist(),
                                      aux_translation_mm=(base_t - rotation @ aux_t).tolist())
    images = {"base": cv2.cvtColor(pair.base.color, cv2.COLOR_BGRA2BGR),
              "aux": cv2.cvtColor(pair.aux.color, cv2.COLOR_BGRA2BGR),
              "base_depth": pair.base.depth, "aux_depth": pair.aux.depth}
    try:
        for name, image in images.items():
            if not cv2.imwrite(str(paths[name]), image):
                raise OSError(f"Could not write {paths[name]}")
        write_json(paths["metadata"], payload)
    except BaseException:
        for path in (*paths.values(), paths["metadata"].with_name(paths["metadata"].name + ".tmp")):
            path.unlink(missing_ok=True)
        raise
    return payload


def cameras_for(settings):
    return DualKinectCapture(Path(settings["sdk_directory"]), settings["base_kinect_serial"],
                             settings["aux_kinect_serial"], settings["subordinate_delay_off_master_usec"],
                             settings["manual_exposure_usec"])


def annotate(preview, corners, observation, role, settings):
    display = preview.copy()
    if corners is not None:
        cv2.drawChessboardCorners(display, (settings["board_cols"], settings["board_rows"]), corners.reshape(-1, 1, 2), True)
        raw_point = np.asarray(observation["reference_point_raw_color_px"], dtype=np.float32).reshape(1, 2)
        preview_point = preview_to_raw_corners(raw_point, preview.shape[1], preview.shape[0], settings[f"{role}_preview_rotation_deg"])
        x, y = np.rint(preview_point[0]).astype(int)
        cv2.circle(display, (x, y), 14, (0, 255, 255), 3)
        cv2.putText(display, reference_geometry(settings)["label"], (x + 18, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    good = observation is not None and observation["reprojection_error_px"] <= settings["max_reprojection_error_px"]
    label = f"{role.upper()}  " + (f"OK {observation['reprojection_error_px']:.2f}px" if good else "CHECK BOARD")
    cv2.putText(display, label, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 0) if good else (0, 0, 255), 2)
    return cv2.resize(display, (640, 360))


def run_live(settings, height, session, manifest, duration=None):
    calibrations = dict(zip(("base", "aux"), _load_rig_calibration(session / "rig_calibration.json")))
    orders = {role: settings[f"{role}_corner_order"] for role in ("base", "aux")}
    status = "Mark the yellow REF point on the board. Keep its height and board tilt fixed."
    cameras = cameras_for(settings)
    index = 1
    first_display_at = None
    viewer = None
    failure = None
    try:
        with cameras:
            manifest.update(devices=cameras.device_info, status="capturing")
            write_json(session / "session.json", manifest)
            viewer = CaptureViewer()
            while True:
                try:
                    pair = cameras.next_pair(timeout=10 if cameras.offset_us is None else 0.5)
                except TimeoutError:
                    key = viewer.poll_key()
                    if key in (27, ord("q")):
                        break
                    continue
                observations, panels = {}, []
                for role in ("base", "aux"):
                    preview, corners, observation = board_observation(getattr(pair, role), calibrations[role], settings, role, orders[role])
                    observations[role] = observation
                    panels.append(annotate(preview, corners, observation, role, settings))
                canvas = np.vstack((np.hstack(panels), np.zeros((100, 1280, 3), np.uint8)))
                height_label = f"{height:g} mm" if height is not None else "PREVIEW ONLY"
                cv2.putText(canvas, f"Height {height_label} | {settings['board_cols']}x{settings['board_rows']} inner corners | square {settings['square_size_mm']:g} mm | saved {index - 1}",
                            (15, 390), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1)
                cv2.putText(canvas, status, (15, 430), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                viewer.show(canvas)
                if first_display_at is None:
                    first_display_at = time.monotonic()
                key = viewer.poll_key()
                if key in (27, ord("q")):
                    break
                if duration is not None and time.monotonic() - first_display_at >= duration:
                    break
                if key in (ord("a"), ord("b")):
                    if index > 1:
                        status = "Corner order is locked after the first capture. Start a new session to change it."
                    else:
                        role = "aux" if key == ord("a") else "base"
                        orders[role] = {"normal": "reverse", "reverse": "normal", "auto_bottom": "auto_bottom_reversed",
                                        "auto_bottom_reversed": "auto_bottom"}[orders[role]]
                        status = "Check that both yellow REF labels point to the same marked physical point."
                elif key in (ord("c"), ord(" ")):
                    if height is None:
                        status = "Preview only. Measure the marked REF height, then restart with --height-mm."
                        continue
                    if not all(o is not None and o["reprojection_error_px"] <= settings["max_reprojection_error_px"] for o in observations.values()):
                        status = "Not saved: show the full checkerboard to both cameras; both indicators must be OK."
                        continue
                    if max(time.monotonic() - pair.base.received_at, time.monotonic() - pair.aux.received_at) > 1.0:
                        status = "Not saved: image processing was too slow. Improve lighting or board visibility."
                        continue
                    record = save_capture(session, index, pair, observations, height)
                    manifest.update(captures_saved=index, corner_orders=orders.copy(), expected_depth_delta_us=cameras.offset_us)
                    write_json(session / "session.json", manifest)
                    status = f"Saved {record['stem']}. Move LEFT/RIGHT and FORWARD/BACK; stop before the next capture."
                    print(f"Saved {record['stem']} | sync error {pair.sync_error_us:.1f} us", flush=True)
                    index += 1
    except BaseException as exc:
        failure = str(exc)
        raise
    finally:
        if viewer is not None:
            viewer.close()
        manifest.update(status="failed" if failure is not None else "closed", captures_saved=index - 1,
                        closed_at=datetime.now(JST).isoformat(), cleanup_warnings=cameras.cleanup_warnings)
        if failure is not None:
            manifest["error"] = failure
        write_json(session / "session.json", manifest)
    return index - 1


def run_smoke(settings, seconds):
    session, manifest = create_session(settings, None, "hardware_smoke_test")
    errors, pair, count = [], None, 0
    cameras = cameras_for(settings)
    with cameras:
        manifest["devices"] = cameras.device_info
        deadline = time.monotonic() + seconds + 2
        while time.monotonic() < deadline:
            pair = cameras.next_pair(timeout=10 if count == 0 else 2)
            errors.append(pair.sync_error_us)
            count += 1
        save_capture(session, 1, pair, {}, None)
    manifest.update(status="hardware_test_passed", pairs_checked=count, captures_saved=1,
                    max_abs_sync_error_us=max(map(abs, errors)), cleanup_warnings=cameras.cleanup_warnings)
    write_json(session / "session.json", manifest)
    print(json.dumps(dict(session=str(session), pairs_checked=count, max_abs_sync_error_us=max(map(abs, errors))), indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("floor_capture_config.json"))
    parser.add_argument("--height-mm", type=float, help="Vertical floor-to-marked-reference-point height.")
    parser.add_argument("--bottom-margin-mm", type=float, help="Distance along the board from printed grid bottom to board bottom; zero uses the printed grid edge.")
    parser.add_argument("--check-devices", action="store_true", help="Check serial numbers and wired-sync jacks without starting cameras.")
    parser.add_argument("--smoke-test", type=float, metavar="SECONDS", help="Save a hardware-test pair without requiring a board or measured height.")
    parser.add_argument("--prepare-only", action="store_true", help="Create a session and export calibration, then exit without opening the viewer.")
    parser.add_argument("--preview-only", action="store_true", help="Find and mark the reference point before measuring its height; saving is disabled.")
    parser.add_argument("--duration", type=float, help="Close the preview after this many seconds (requires --preview-only).")
    args = parser.parse_args()
    cv2.setNumThreads(2)
    settings = read_settings(args.config.resolve())
    if args.bottom_margin_mm is not None:
        if settings["reference_point_kind"] != "bottom_edge_center":
            raise ValueError("--bottom-margin-mm requires the bottom-edge reference.")
        settings["reference_bottom_margin_mm"] = args.bottom_margin_mm
    if args.duration is not None and (not args.preview_only or not math.isfinite(args.duration) or args.duration <= 0):
        raise ValueError("Use a positive --duration with --preview-only.")
    if args.check_devices:
        cameras = cameras_for(settings)
        try:
            cameras.open()
            print(json.dumps(cameras.device_info, indent=2))
        finally:
            cameras.close()
        return 0
    if args.smoke_test is not None:
        if not math.isfinite(args.smoke_test) or args.smoke_test <= 0:
            raise ValueError("Smoke-test seconds must be positive and finite.")
        run_smoke(settings, args.smoke_test)
        return 0
    if settings["reference_point_kind"] == "bottom_edge_center" and settings["reference_bottom_margin_mm"] is None:
        settings["reference_bottom_margin_mm"] = float(input("白黒格子の下端から板の下端までの余白（mm）: "))
    geometry = reference_geometry(settings)
    print(f"高さの基準: {geometry['description']}（{geometry['label']}）", flush=True)
    if args.preview_only:
        session, manifest = create_session(settings, None, "alignment_preview")
        print(f"基準点の確認用プレビュー（画像保存なし）: {session}", flush=True)
        run_live(settings, None, session, manifest, args.duration)
        return 0
    height = args.height_mm if args.height_mm is not None else settings.get("reference_height_mm")
    if height is None:
        print("同じ基準点の高さを保ち、左右・前後へ移動して撮影します。")
        print("黄色のREFで示す基準点に印を付け、床からその点までの鉛直高さを測ってください。")
        height = float(input("基準点の高さ（mm）: "))
    height = validate_height(height)
    print(f"床から基準点までの高さ: {height:g} mm（{height / 10:g} cm）", flush=True)
    session, manifest = create_session(settings, height)
    print(f"保存先: {session}", flush=True)
    if not args.prepare_only:
        count = run_live(settings, height, session, manifest)
        print(f"保存した画像ペア: {count}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("撮影を終了しました。")
        raise SystemExit(130)
    except (ValueError, RuntimeError, OSError, subprocess.CalledProcessError, cv2.error) as exc:
        print(f"撮影エラー: {exc}")
        raise SystemExit(1)
