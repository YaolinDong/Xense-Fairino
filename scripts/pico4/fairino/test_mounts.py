"""Check mounting transforms against an independent full-URDF kinematic chain."""
import math
import unittest
import numpy as np
import paths
from mounts import (BODY_FROM_BASE, BODY_FROM_DH, CHEST_FROM_DH,
                    tcp_in_chest, tcp_in_base)
from audit_mounts import FullModel, compare_models
from vr_input import Clutch
from lerobot.utils.rotation import Rotation
from vendor_kinematics import VendorKinematics
from test_tcp import hardware
from tcp_core import ARMS
from vr_plan import request
from vr_core import validate


class MountTests(unittest.TestCase):
    def test_keyboard_and_pico_use_same_installation_frame(self):
        from tcp_frames import CHEST_FROM_DH as keyboard_mounts, CHEST_FRAME_ID
        from mounts import MAPPING_ID
        self.assertEqual(CHEST_FRAME_ID, MAPPING_ID)
        for side in ("left", "right"):
            np.testing.assert_allclose(keyboard_mounts[side], CHEST_FROM_DH[side], atol=1e-12)

    def test_full_urdf_matches_selected_art7_model_across_400_poses(self):
        for side, result in compare_models().items():
            with self.subTest(side=side):
                self.assertLess(result["max_position_error_mm"], .02)
                self.assertLess(result["max_orientation_error_deg"], .01)

    def test_dh_origin_offsets_are_included_once_and_pose_roundtrips(self):
        for side, lateral in (("left", 48.), ("right", -48.)):
            mount = CHEST_FROM_DH[side]
            np.testing.assert_allclose(mount[:3, 3], [0., lateral, 330.], atol=.002)
            self.assertGreater(np.linalg.norm(BODY_FROM_DH[side][:3, 3]-BODY_FROM_BASE[side][:3, 3]), .94)
            tcp = np.eye(4)
            tcp[:3, 3] = [153., -82., 408.]
            tcp[:3, :3] = Rotation.from_rotvec([.23, -.64, 1.1]).as_matrix()
            np.testing.assert_allclose(tcp_in_base(side, tcp_in_chest(side, tcp)), tcp, atol=1e-10)

    def test_both_hands_all_signed_directions_match_full_model_and_vendor_ik(self):
        kin, model, hw = VendorKinematics(), FullModel(), hardware()
        raw_axes = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
        # Define chest axes independently from the runtime mount matrices.
        chest_from_body_rotation = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])
        for side, indices in ARMS.items():
            q0 = [hw["command_position"][j-1] for j in indices]
            tcp = np.array(kin.forward_tcp(q0)[0])
            start_full = model.flange_in_body(side, q0)
            for axis in range(6):
                for sign in (-1., 1.):
                    with self.subTest(side=side, axis=axis, sign=sign):
                        hand = {"pose": [.1, .2, -.3, 0., 0., 0., 1.], "grip": 1.}
                        clutch = Clutch(side)
                        np.testing.assert_allclose(clutch.target(hand, tcp, .5), tcp, atol=1e-9)
                        if axis < 3:
                            hand["pose"][:3] = np.array(hand["pose"][:3])+raw_axes[axis]*sign*.0008
                        else:
                            hand["pose"][3:] = Rotation.from_rotvec(raw_axes[axis-3]*sign*math.radians(.2)).as_quat()
                        goal = clutch.target(hand, tcp, .5)
                        targets = validate(hw, request(kin, hw, {side: goal}, 1, 10.), 10.)
                        end_full = model.flange_in_body(side, targets[side])
                        delta_position = chest_from_body_rotation @ (end_full[:3, 3]-start_full[:3, 3])
                        expected_position = np.eye(3)[axis]*sign*.4 if axis < 3 else np.zeros(3)
                        np.testing.assert_allclose(delta_position, expected_position, atol=.001)
                        delta_rotation = chest_from_body_rotation @ (end_full[:3, :3] @ start_full[:3, :3].T) @ chest_from_body_rotation.T
                        expected_rotation = np.eye(3) if axis < 3 else Rotation.from_rotvec(np.eye(3)[axis-3]*sign*math.radians(.1)).as_matrix()
                        np.testing.assert_allclose(delta_rotation, expected_rotation, atol=1e-5)


if __name__ == "__main__":
    unittest.main()
