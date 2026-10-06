import copy
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from calibrate_floor import calibrate, fit_floor, tracking_profile


def synthetic_dataset():
    pitch = 0.55
    up = np.array([0., -np.cos(pitch), -np.sin(pitch)])
    forward = np.array([0., -np.sin(pitch), np.cos(pitch)])
    basis = np.stack([[1., 0., 0.], up, forward])
    rotation = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    translation = np.array([-1200., -950., 500.])
    reference = np.array([88., 132., 0.])
    height, camera_height = 1017., 2100.
    session = dict(reference_height_mm=height, checkerboard=dict(board_cols=9, board_rows=6, square_size_mm=22.),
                   reference=dict(kind="bottom_edge_center", point_board_mm=reference.tolist()),
                   settings=dict(base_preview_rotation_deg=0, aux_preview_rotation_deg=180,
                                 base_kinect_serial="base", aux_kinect_serial="aux"))
    captures = []
    for index, (x, z) in enumerate([(-400., 1400.), (400., 1400.), (400., 2100.), (-400., 2100.), (0., 1750.)]):
        point = basis.T @ (np.array([x, height, z]) - [0., camera_height, 0.])
        yaw = index * 0.15
        board_r = np.array([[np.cos(yaw), 0., np.sin(yaw)], [0., 1., 0.], [-np.sin(yaw), 0., np.cos(yaw)]])
        board_t = point - board_r @ reference
        capture = dict(stem=f"{index:04d}", reference_height_mm=height, sync_error_us=4.)
        for role, r, t in [("base", board_r, board_t), ("aux", rotation.T @ board_r, rotation.T @ (board_t - translation))]:
            capture[role] = dict(checkerboard=dict(reprojection_error_px=0.2, board_to_depth_rotation=r.tolist(),
                                                  board_to_depth_translation_mm=t.tolist(), reference_kind="bottom_edge_center",
                                                  reference_point_board_mm=reference.tolist(), reference_point_depth_mm=(r @ reference + t).tolist()))
        captures.append(capture)
    return session, captures, basis, rotation, translation


class FloorCalibrationTests(unittest.TestCase):
    def test_tilted_and_inverted_cameras_recover_floor_and_rigid_transform_with_changing_yaw(self):
        session, captures, basis, rotation, translation = synthetic_dataset()
        result = calibrate(session, captures, "synthetic")
        np.testing.assert_allclose(result["base_depth_to_floor"]["basis_matrix"], basis, atol=1e-9)
        self.assertAlmostEqual(result["base_depth_to_floor"]["camera_height_mm"], 2100.)
        np.testing.assert_allclose(result["aux_depth_to_base_depth"]["rotation_matrix"], rotation, atol=1e-9)
        np.testing.assert_allclose(result["aux_depth_to_base_depth"]["translation_mm"], translation, atol=1e-9)
        np.testing.assert_allclose(np.array(result["quality"]["base_reference_floor_mm"])[:, 1], 1017., atol=1e-9)
        np.testing.assert_allclose(result["quality"]["base_reference_floor_mm"], result["quality"]["aux_reference_floor_mm"], atol=1e-9)
        self.assertEqual(result["sensor_orientation"]["aux"], "flip180")

    def test_virtual_ninety_degree_frame_returns_same_physical_point(self):
        session, captures, _, _, _ = synthetic_dataset()
        result = calibrate(session, captures, "synthetic")
        virtual = result["virtual_aux_90"]
        basis = np.array(virtual["aux_depth_basis_matrix"])
        offset = np.array(virtual["aux_depth_translation_mm"])
        rotation = np.array(virtual["rotation_to_base_floor"])
        origin = np.array(virtual["origin_in_base_floor_mm"])
        for capture, expected in zip(captures, result["quality"]["base_reference_floor_mm"]):
            point = np.array(capture["aux"]["checkerboard"]["reference_point_depth_mm"])
            np.testing.assert_allclose(rotation @ (basis @ point + offset) + origin, expected, atol=1e-9)
        np.testing.assert_array_equal(rotation[:, 2], [1., 0., 0.])
        self.assertEqual(origin[1], 0.)

    def test_rejects_collinear_positions_and_changed_reference_height(self):
        with self.assertRaisesRegex(ValueError, "collinear"):
            fit_floor([[0., 0., 1000.], [100., 0., 1000.], [200., 0., 1000.]], 1000., [0., -1., 0.])
        session, captures, *_ = synthetic_dataset()
        captures[-1]["reference_height_mm"] += 10
        with self.assertRaisesRegex(ValueError, "height changed"):
            calibrate(session, captures, "synthetic")

    def test_rejects_wrong_corner_order_and_bad_sync(self):
        session, captures, *_ = synthetic_dataset()
        changed = copy.deepcopy(captures)
        changed[0]["aux"]["checkerboard"]["reference_point_board_mm"][1] -= 22
        with self.assertRaisesRegex(ValueError, "reference changed"):
            calibrate(session, changed, "synthetic")
        captures[0]["sync_error_us"] = 33333
        with self.assertRaisesRegex(ValueError, "Unsynchronized"):
            calibrate(session, captures, "synthetic")

    def test_generated_profile_preserves_actuator_setting_and_requires_matching_serials(self):
        session, captures, *_ = synthetic_dataset()
        calibration = calibrate(session, captures, "synthetic")
        profile = tracking_profile(dict(enable_ptu=False), calibration)
        self.assertFalse(profile["enable_ptu"])
        self.assertEqual(profile["coordinate_output_frame"], "base_floor")
        with self.assertRaisesRegex(ValueError, "serials"):
            tracking_profile(dict(base_kinect_serial="wrong"), calibration)


if __name__ == "__main__":
    unittest.main()
