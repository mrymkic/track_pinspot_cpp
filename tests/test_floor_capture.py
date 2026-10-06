import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from capture_floor_calibration import annotate, board_observation, brighten_preview, read_settings, reference_geometry, run_live, save_capture, validate_height
from calibrate_checkerboard_extrinsics import CameraCalibration
from kinect_capture import CameraFrame, CapturePair


ROOT = Path(__file__).resolve().parents[1]


def checkerboard():
    image = np.full((480, 640, 4), 210, np.uint8)
    image[:, :, 3] = 255
    for row in range(7):
        for col in range(10):
            value = 255 if (row + col) % 2 else 0
            image[80 + row * 40:80 + (row + 1) * 40,
                  120 + col * 40:120 + (col + 1) * 40, :3] = value
    return image


def frame(image, timestamp=100000):
    return CameraFrame(1, time.monotonic(), image, np.full((24, 32), 1234, np.uint16),
                       timestamp, timestamp, timestamp * 1000)


def calibration(role):
    return CameraCalibration(role, np.array([[600., 0, 319.5], [0, 600., 239.5], [0, 0, 1.]]),
                             np.zeros(8), np.eye(3), np.zeros(3))


class FloorCaptureTests(unittest.TestCase):
    def setUp(self):
        cv2.setNumThreads(2)
        self.settings = read_settings(ROOT / "tools" / "floor_capture_config.json")

    def test_measured_square_count_becomes_correct_inner_corner_count(self):
        self.assertEqual((self.settings["board_cols"], self.settings["board_rows"]), (9, 6))
        self.assertEqual(self.settings["square_size_mm"], 22)

    def test_height_must_be_measured_not_invented(self):
        for invalid in (None, 0, -20, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                validate_height(invalid)
        self.assertEqual(validate_height(1123.5), 1123.5)

    def test_inverted_preview_keeps_reference_point_in_native_camera_coordinates(self):
        original = checkerboard()
        upside_down = cv2.rotate(original, cv2.ROTATE_180)
        _, _, base = board_observation(frame(original), calibration("base"), self.settings, "base", "auto_bottom")
        _, _, aux = board_observation(frame(upside_down), calibration("aux"), self.settings, "aux", "auto_bottom")
        self.assertIsNotNone(base)
        self.assertIsNotNone(aux)
        flip = np.diag([-1., -1., 1.])
        np.testing.assert_allclose(aux["reference_point_depth_mm"], flip @ base["reference_point_depth_mm"], atol=0.05)
        self.assertLess(base["reprojection_error_px"], 0.1)
        self.assertLess(aux["reprojection_error_px"], 0.1)
        self.assertEqual(base["reference_kind"], "bottom_edge_center")
        np.testing.assert_allclose(base["reference_point_raw_color_px"], [319.5, 359.5], atol=0.1)
        np.testing.assert_allclose(aux["reference_point_raw_color_px"], [319.5, 119.5], atol=0.1)

    def test_bottom_reference_follows_board_tilt_and_measured_margin(self):
        camera = calibration("base")
        rotation_vector = np.array([0.2, 0.15, 0.0])
        rotation = cv2.Rodrigues(rotation_vector)[0]
        translation = np.array([-88., -55., 500.])
        board_edges = np.array([[-22., -22., 0], [198., -22., 0], [198., 132., 0], [-22., 132., 0]])
        destination, _ = cv2.projectPoints(board_edges, rotation_vector, translation,
                                          camera.color_camera_matrix, camera.color_distortion_coeffs)
        source = np.array([[119.5, 79.5], [519.5, 79.5], [519.5, 359.5], [119.5, 359.5]], np.float32)
        homography = cv2.getPerspectiveTransform(source, destination.reshape(4, 2).astype(np.float32))
        tilted = cv2.warpPerspective(checkerboard(), homography, (640, 480), borderValue=(210, 210, 210, 255))
        for margin in (0., 11.):
            with self.subTest(margin=margin):
                settings = {**self.settings, "reference_bottom_margin_mm": margin}
                _, _, observation = board_observation(frame(tilted), camera, settings, "base", "auto_bottom")
                self.assertIsNotNone(observation)
                np.testing.assert_allclose(observation["reference_point_depth_mm"],
                                           rotation @ np.array([88., 132. + margin, 0.]) + translation, atol=1.0)
                self.assertLess(observation["reprojection_error_px"], 0.3)

    def test_negative_bottom_margin_is_rejected(self):
        with self.assertRaises(ValueError):
            reference_geometry({**self.settings, "reference_bottom_margin_mm": -1})

    def test_bottom_reference_is_stable_when_detector_reverses_corner_order(self):
        image = checkerboard()
        found, corners = cv2.findChessboardCornersSB(cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY), (9, 6),
                                                    flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
        self.assertTrue(found)
        for detected in (corners.copy(), corners[::-1].copy()):
            with patch("capture_floor_calibration.cv2.findChessboardCorners", return_value=(False, None)), \
                 patch("capture_floor_calibration.cv2.findChessboardCornersSB", return_value=(True, detected)):
                _, _, observation = board_observation(frame(image), calibration("base"), self.settings, "base", "auto_bottom")
            np.testing.assert_allclose(observation["reference_point_raw_color_px"], [319.5, 359.5], atol=0.1)

    def test_small_dark_board_is_detected_without_reducing_its_resolution(self):
        image = np.full((480, 640, 4), 30, np.uint8)
        image[:, :, 3] = 255
        image[:100, :200, :3] = 220
        small = cv2.resize(checkerboard(), (128, 96), interpolation=cv2.INTER_AREA)
        small[:, :, :3] = (small[:, :, :3].astype(float) * 0.12 + 5).astype(np.uint8)
        image[160:256, 256:384] = small
        _, _, observation = board_observation(frame(image), calibration("base"), self.settings, "base", "auto_bottom")
        self.assertIsNotNone(observation)
        self.assertLess(observation["reprojection_error_px"], 0.5)
        np.testing.assert_allclose(observation["reference_point_raw_color_px"], [319.5, 231.5], atol=0.3)

    def test_preview_brightness_does_not_modify_original_pixels(self):
        image = np.full((120, 160, 3), 30, np.uint8)
        before = image.copy()
        bright = brighten_preview(image, 1.8)
        self.assertGreater(float(bright.mean()), float(image.mean()))
        np.testing.assert_array_equal(image, before)

    def test_reference_name_is_visible_before_detection_without_a_fake_marker(self):
        image = np.full((480, 640, 3), 30, np.uint8)
        with patch("capture_floor_calibration.cv2.putText", wraps=cv2.putText) as text, \
             patch("capture_floor_calibration.cv2.circle", wraps=cv2.circle) as circle:
            annotate(image, None, None, "base", self.settings)
        labels = [call.args[1] for call in text.call_args_list]
        self.assertTrue(any(label.startswith("REF BOTTOM") for label in labels))
        circle.assert_not_called()

    def test_capture_preserves_raw_orientation_depth_units_and_height(self):
        original = checkerboard()
        upside_down = cv2.rotate(original, cv2.ROTATE_180)
        pair = CapturePair(frame(original), frame(upside_down, 100711), 711)
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary)
            for name in ("base", "aux_color", "base_depth", "aux_depth", "captures"):
                (session / name).mkdir()
            save_capture(session, 1, pair, {}, 1123.5)
            np.testing.assert_array_equal(cv2.imread(str(session / "aux_color/0001.png")), upside_down[:, :, :3])
            depth = cv2.imread(str(session / "base_depth/0001.png"), cv2.IMREAD_UNCHANGED)
            self.assertEqual(depth.dtype, np.uint16)
            self.assertTrue(np.all(depth == 1234))
            metadata = json.loads((session / "captures/0001.json").read_text())
            self.assertEqual(metadata["reference_height_mm"], 1123.5)
            self.assertEqual(metadata["sync_error_us"], 0)
            with self.assertRaises(FileExistsError):
                save_capture(session, 1, pair, {}, 900)
            self.assertEqual(json.loads((session / "captures/0001.json").read_text())["reference_height_mm"], 1123.5)

    def test_failed_save_does_not_leave_a_partial_capture(self):
        pair = CapturePair(frame(checkerboard()), frame(checkerboard(), 100711), 711)
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary)
            for name in ("base", "aux_color", "base_depth", "aux_depth", "captures"):
                (session / name).mkdir()
            original_write = cv2.imwrite
            calls = []
            def fail_third(path, image):
                calls.append(path)
                return original_write(path, image) if len(calls) < 3 else False
            with patch("capture_floor_calibration.cv2.imwrite", side_effect=fail_third):
                with self.assertRaises(OSError):
                    save_capture(session, 1, pair, {}, 1123.5)
            self.assertEqual(list(session.rglob("*.png")), [])
            self.assertEqual(list(session.rglob("*.json")), [])

    def run_simulated_capture(self, session, keys, height, aux_has_board=True):
        for name in ("base", "aux_color", "base_depth", "aux_depth", "captures"):
            (session / name).mkdir()
        original = checkerboard()
        aux = cv2.rotate(original, cv2.ROTATE_180) if aux_has_board else np.full_like(original, 210)
        cameras = Mock()
        cameras.__enter__ = Mock(return_value=cameras)
        cameras.__exit__ = Mock(return_value=False)
        cameras.device_info = {"base": {"wired_sync_mode": "subordinate"}, "aux": {"wired_sync_mode": "master"}}
        cameras.cleanup_warnings = []
        cameras.offset_us = 711
        cameras.next_pair.side_effect = lambda **kwargs: CapturePair(frame(original), frame(aux, 100711), 711)
        viewer = Mock()
        viewer.poll_key.side_effect = keys
        manifest = {"status": "prepared"}
        with patch("capture_floor_calibration.cameras_for", return_value=cameras), \
             patch("capture_floor_calibration._load_rig_calibration", return_value=(calibration("base"), calibration("aux"))), \
             patch("capture_floor_calibration.CaptureViewer", return_value=viewer):
            count = run_live(self.settings, height, session, manifest)
        cameras.__exit__.assert_called_once()
        viewer.close.assert_called_once()
        return count, json.loads((session / "session.json").read_text())

    def test_live_save_records_both_poses_and_locks_reference_order(self):
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary)
            count, manifest = self.run_simulated_capture(session, [ord("c"), ord("a"), ord("c"), ord("q")], 1123.5)
            self.assertEqual(count, 2)
            self.assertEqual(manifest["status"], "closed")
            self.assertEqual(manifest["corner_orders"], {"base": "auto_bottom", "aux": "auto_bottom"})
            first = json.loads((session / "captures/0001.json").read_text())
            second = json.loads((session / "captures/0002.json").read_text())
            self.assertEqual(first["reference_height_mm"], 1123.5)
            np.testing.assert_allclose(first["aux_to_base"]["rotation_matrix"], np.diag([-1., -1., 1.]), atol=0.001)
            np.testing.assert_allclose(first["aux"]["checkerboard"]["reference_point_depth_mm"],
                                       second["aux"]["checkerboard"]["reference_point_depth_mm"], atol=0.001)

    def test_live_save_requires_measured_height_and_both_boards(self):
        for height, aux_has_board in ((None, True), (1123.5, False)):
            with self.subTest(height=height, aux_has_board=aux_has_board), tempfile.TemporaryDirectory() as temporary:
                session = Path(temporary)
                count, manifest = self.run_simulated_capture(session, [ord("c"), ord("q")], height, aux_has_board)
                self.assertEqual(count, 0)
                self.assertEqual(manifest["captures_saved"], 0)
                self.assertEqual(list(session.rglob("*.png")), [])
                self.assertEqual(list((session / "captures").iterdir()), [])


if __name__ == "__main__":
    unittest.main()
