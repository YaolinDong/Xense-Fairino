"""Offline adapter checks. Each native case gets a fresh Python process."""
from functools import wraps
import hashlib
import math
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np

from apps.fairino_tcp_probe.tcp_core import pose_error, tcp_forward_matrix
from apps.fairino_tcp_probe.tool_frames import for_arm, tool_config
from apps.workbench.model import RobotModel, URDF
from scripts.pico4.fairino.mounts import BODY_FROM_DH, CHEST_FROM_BODY
from tools.ik_experiments import fairino_qp_art7 as adapter
from tools.ik_experiments.evaluate_fairino_qp import body_pose, homes, solver_reference, world_frame
from tools.ik_experiments.setup_fairino_ik import ROOT, VENDOR

NATIVE_CHILD = __name__ == '__main__' and '--native-case' in sys.argv


class DefinitionTests(unittest.TestCase):
    @unittest.skipUnless((VENDOR / 'CMakeLists.txt').is_file(), 'Optional fairino_ik submodule absent')
    def test_rd53_assets_match_pinned_ik_submodule(self):
        packaged = VENDOR / 'models/fairino'
        self.assertEqual(URDF.read_bytes(), (ROOT / 'models/whole_body_20261030.urdf').read_bytes())
        local = ET.parse(URDF).getroot()
        upstream = ET.parse(packaged / 'fairino_wheel_legged_robot.urdf').getroot()
        # The manufacturer uses package:// URIs; the upstream wheel uses relative paths.
        for root in (local, upstream):
            for mesh in root.iter('mesh'):
                mesh.set('filename', Path(mesh.get('filename')).name)
        self.assertEqual(ET.tostring(local), ET.tostring(upstream))
        for name in {mesh.get('filename') for mesh in local.iter('mesh')}:
            mine = ROOT / 'models/whole_body_meshes' / name
            theirs = packaged / 'meshes' / name
            self.assertEqual(hashlib.sha256(mine.read_bytes()).digest(),
                             hashlib.sha256(theirs.read_bytes()).digest(), name)

    def test_fast_rigid_check_and_error_match_reference(self):
        rng = np.random.default_rng(2026)
        for tol in (1e-8, 1e-6):
            for delta in (0., tol/4, tol*.99, tol, tol*1.01, 2*tol):
                q, _ = np.linalg.qr(rng.normal(size=(3, 3)))
                t = np.eye(4)
                t[:3, :3] = q
                for i, j in ((0, 0), (0, 1), (3, 0), (3, 3)):
                    candidate = t.copy()
                    candidate[i, j] += delta
                    r = candidate[:3, :3]
                    valid = (np.allclose(candidate[3], t[3], atol=1e-9, rtol=0)
                             and np.allclose(r.T @ r, np.eye(3), atol=tol, rtol=0)
                             and abs(np.linalg.det(r)-1.) <= tol)
                    if valid:
                        np.testing.assert_array_equal(adapter.rigid_transform(candidate, tol), candidate)
                    else:
                        with self.assertRaises(ValueError):
                            adapter.rigid_transform(candidate, tol)
                    self.assertEqual(adapter.matrix_error(t, candidate), pose_error(t, candidate))
        for _ in range(100):
            a, b = rng.normal(size=(2, 4, 4))
            self.assertEqual(adapter.matrix_error(a, b), pose_error(a, b))
        for i in range(16):
            for value in (np.nan, np.inf):
                t = np.eye(4)
                t.flat[i] = value
                with self.assertRaises(ValueError):
                    adapter.rigid_transform(t)

    def test_project_joint_limits_and_exact_dh_arms_preserve_rd53_geometry(self):
        actual = ET.fromstring(adapter.model_xml())
        expected = ET.parse(URDF).getroot()
        for i, name in enumerate(adapter.ARM_NAMES):
            joint = actual.find(f"joint[@name='{name}']")
            vendor = expected.find(f"joint[@name='{name}']")
            limit = joint.find('limit')
            for key, col in [('lower', 0), ('upper', 1)]:
                self.assertEqual(float(limit.get(key)), adapter.ARM_LIMITS[i, col])
                limit.set(key, vendor.find('limit').get(key))
            # Each arm CAD axis and origin is restored to its nearest DH value.
            for tag, attr, step in [('origin', 'rpy', math.pi/2), ('axis', 'xyz', 1.)]:
                original = vendor.find(tag).get(attr)
                exact = [round(float(v) / step) * step for v in original.split()]
                self.assertEqual(list(map(float, joint.find(tag).get(attr).split())), exact)
                joint.find(tag).set(attr, original)
        head_torso = {'j15': adapter.QP_HEAD_LIMITS}
        head_torso.update({name: (lo, hi) for _, name, _, lo, hi in adapter.QP_TORSO_LIMITS})
        for name, bounds in head_torso.items():
            limit = actual.find(f"joint[@name='{name}']/limit")
            vendor = expected.find(f"joint[@name='{name}']/limit")
            self.assertEqual(tuple(float(limit.get(k)) for k in ('lower', 'upper')), bounds)
            for key in ('lower', 'upper'):
                limit.set(key, vendor.get(key))
        # All other geometry, axes, effort and velocity limits retain the vendor values.
        self.assertEqual(ET.tostring(actual), ET.tostring(expected))

    def test_elbow_lower_bound_only_raises_both_elbows(self):
        bounded = ET.fromstring(adapter.model_xml(elbow_min_deg=10.))
        plain = ET.fromstring(adapter.model_xml())
        for joint in plain.findall('joint'):
            name = joint.get('name')
            mine = bounded.find(f"joint[@name='{name}']")
            if name in adapter.ELBOW_JOINTS:
                self.assertAlmostEqual(float(mine.find('limit').get('lower')), math.radians(10.))
                self.assertEqual(mine.find('limit').get('upper'), joint.find('limit').get('upper'))
                mine.find('limit').set('lower', joint.find('limit').get('lower'))
            self.assertEqual(ET.tostring(mine), ET.tostring(joint), name)

    def test_missing_build_fails_with_actionable_setup(self):
        with (patch.object(adapter, '_check_process'),
              patch.object(adapter, 'extension_path', return_value=adapter.CACHE / 'missing-extension')):
            with self.assertRaisesRegex(RuntimeError, 'setup_fairino_ik.py --install'):
                adapter.Art7FairinoQP(tool_config())

    def test_detect_tron2_before_extension_loading(self):
        with patch.dict(sys.modules, {'tron2_ik._native': object()}):
            with self.assertRaisesRegex(RuntimeError, 'tron2_ik'):
                adapter.Art7FairinoQP(tool_config())

    def test_directions_from_chest_fk_agree_with_body_control_readme(self):
        model = RobotModel()
        def chest(height=100, forward=0, roll=0, yaw=0):
            q = body_pose(model.body, height, forward, roll, yaw)
            positions = adapter.urdf_coordinates(q, math.radians(model.body.neutral))
            raw, _ = model.transforms(positions)
            return world_frame(raw['body_Link']) @ CHEST_FROM_BODY.T
        # X forward, Y left, Z up. Infer chest heading and lean from two
        # independent physical directions (chest forward and chest up).
        for sign in (-1, 1):
            heading = chest(yaw=10 * sign)[:3, 0]
            self.assertAlmostEqual(math.degrees(math.atan2(heading[1], heading[0])), 10 * sign, places=2)
            up = chest(roll=10 * sign)[:3, 2]
            self.assertAlmostEqual(math.degrees(math.atan2(-up[1], up[2])), 10 * sign, places=2)
        up = chest(50, forward=25)[:3, 2]
        self.assertAlmostEqual(math.degrees(math.atan2(up[0], up[2])), 25, places=2)
        pivot = []
        for height in (0, 100):
            positions = adapter.urdf_coordinates(body_pose(model.body, height), math.radians(model.body.neutral))
            raw, _ = model.transforms(positions)
            pivot.append(world_frame(raw['hip_Link'])[:3, 3])
        np.testing.assert_allclose(pivot[1] - pivot[0], (0., 0., model.body.travel), rtol=0, atol=.002)


def native_case(method):
    @wraps(method)
    def isolated(self):
        if NATIVE_CHILD:
            return method(self)
        command = [sys.executable, '-B', '-m', 'tools.ik_experiments.test_fairino_qp_art7',
                   '--native-case', f'{type(self).__name__}.{method.__name__}']
        result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        self.assertNotIn('skipped=', result.stderr)
    return isolated


@unittest.skipUnless(adapter.extension_path().is_file(), 'Optional fairino_ik native build absent')
class NativeTests(unittest.TestCase):
    def setUp(self):
        if NATIVE_CHILD:
            self.model = solver_reference()
            self.solver = adapter.Art7FairinoQP(tool_config('taccap'))
            self.seed = body_pose(self.model.body, 100)
            self.seed[:14] = homes()['default']
            self.seed[14] = .91

    @native_case
    def test_native_head_lock_and_whole_chain_and_tool(self):
        for side, angles in zip(adapter.SIDES, self.seed[:14].reshape(2, 7)):
            q = self.seed.copy()
            q[14] = -2.
            frames = self.solver.frames(q)
            raw, _ = self.model.transforms(adapter.urdf_coordinates(self.seed, self.solver.upright_rad))
            for link in ('head_Link', 'body_Link', f'{side}_waist2_Link'):
                np.testing.assert_allclose(frames[link], world_frame(raw[link]), rtol=0, atol=1e-9)
            dh_tool = (world_frame(raw['body_Link']) @ BODY_FROM_DH[side]
                       @ tcp_forward_matrix(angles.tolist()) @ for_arm(tool_config('taccap'), side))
            # The solver's arm chain is the exact DH model, not merely within the gate.
            for candidate in (self.solver.forward(side, self.seed), self.solver.forward(side, q)):
                mm, deg = pose_error(candidate, dh_tool)
                self.assertLessEqual(mm, 1e-6)
                self.assertLessEqual(deg, 1e-4)
        self.assertNotIn('pinocchio', sys.modules)
        self.assertFalse(Path(np.__file__).resolve().is_relative_to(adapter.CACHE))

    @native_case
    def test_all_torso_limit_violations_rejected_before_fk_or_ik(self):
        calls = (lambda q: self.solver.forward('left', q), self.solver.frames,
                 lambda q: self.solver.solve_body(dict(left=np.eye(4), right=np.eye(4)), q),
                 lambda q: self.solver.solve_arms(dict(left=np.eye(4), right=np.eye(4)), q))
        np.testing.assert_array_equal(self.solver.torso_limits, [row[3:] for row in adapter.QP_TORSO_LIMITS])
        for (number, name, sign), (lo, hi) in zip(adapter.TORSO_MAPPING, self.solver.torso_limits):
            for outside in (lo - 1e-9, hi + 1e-9):
                q = self.seed.copy()
                q[number - 1] = sign * (outside + (self.solver.upright_rad if name == 'j18' else 0.))
                for call in calls:
                    with self.assertRaisesRegex(ValueError, name + ' outside Fairino QP'):
                        call(q)
        low = body_pose(self.model.body, 30)
        for side in adapter.SIDES:
            with self.assertRaisesRegex(ValueError, 'j18 outside Fairino QP'):
                self.solver.forward(side, low)

    @native_case
    def test_selected_chest_body_direction_and_mutable_goal(self):
        far = body_pose(self.model.body, 65, 10)
        far[:14] = self.seed[:14]
        far[15] += .12
        for q in (self.seed, far):
            np.testing.assert_array_equal(self.solver.body_frame(q), self.solver.frames(q)['body_Link'])
        goals = {s: self.solver.forward(s, far) for s in adapter.SIDES}
        kwargs = dict(max_iterations=6, arms_first_iterations=0)
        expected = self.solver.solve_body(goals, self.seed, **kwargs)
        actual = self.solver.solve_body(goals, self.seed, **kwargs, positions_only=True)
        np.testing.assert_array_equal(expected['q'], actual['q'])
        goals['left'][3, 0] = 2e-9
        with self.assertRaises(ValueError):
            self.solver.solve_body(goals, self.seed, **kwargs, positions_only=True)

    @native_case
    def test_fixed_torso_near_and_active_torso_far(self):
        near = self.seed.copy()
        near[[0, 7]] += .006
        far = body_pose(self.model.body, 65, 10)
        far[:14] = np.radians([45, 60, -80, 25, 75, 10, 0, -45, 60, 80, 25, -75, 10, 0])
        for solve, target in ((self.solver.solve_arms, near), (self.solver.solve_body, far)):
            goals = {s: self.solver.forward(s, target) for s in adapter.SIDES}
            res = solve(goals, self.seed)
            self.assertTrue(res['success'], res['status'])
            self.assertGreater(res['iterations'], 0)
            self.assertEqual(res['q'][14], self.seed[14])
            if target is near:
                np.testing.assert_array_equal(res['q'][15:], self.seed[15:])
            else:
                self.assertGreater(np.linalg.norm(res['q'][15:] - self.seed[15:]), .01)
                self.assertFalse(self.solver.solve_arms(goals, self.seed)['success'])
            raw, _ = self.model.transforms(adapter.urdf_coordinates(res['q'], self.solver.upright_rad))
            for side in adapter.SIDES:
                independent = world_frame(raw[side + '_waist2_Link']) @ self.solver.wrist_from_tool[side]
                mm, deg = pose_error(independent, goals[side])
                self.assertLessEqual(mm, adapter.POSITION_MM)
                self.assertLessEqual(deg, adapter.ORIENTATION_DEG)

    @native_case
    def test_posture_task_and_step_bound_reach_the_native_solver(self):
        solver = adapter.Art7FairinoQP(tool_config('taccap'), elbow_min_deg=10.)
        home = self.seed[:14].copy()
        seed, options = solver._seed(self.seed, False, 40, None, None, home, 3.6)
        self.assertEqual(options.posture_weight, 1.)
        np.testing.assert_allclose(options.posture_target, home)
        self.assertAlmostEqual(math.degrees(options.default_velocity_limit*options.dt)*40, 3.6)
        # Redundancy moved off Home with the same hand targets: the posture task pulls it back,
        # never further than the per-solve bound.
        off = self.seed.copy()
        off[[0, 2, 4]] += np.radians([6., -5., 5.])
        goals = {s: solver.forward(s, off) for s in adapter.SIDES}
        bounded = solver.solve_arms(goals, off, max_iterations=40, posture=home, step_deg=1.)
        self.assertLessEqual(np.degrees(np.abs(bounded['q'][:14]-off[:14])).max(), 1.+1e-6)
        # Same hand targets, so only the null space can move; the squared distance to Home drops.
        free = solver.solve_arms(goals, off, max_iterations=40, posture=home)
        self.assertLess(np.sum((free['q'][:14]-home)**2), np.sum((off[:14]-home)**2))
        for mm, deg in free['errors_mm_deg'].values():
            self.assertLessEqual(mm, adapter.POSITION_MM)
            self.assertLessEqual(deg, adapter.ORIENTATION_DEG)
        low = self.seed.copy()
        low[3] = math.radians(9.)
        with self.assertRaisesRegex(ValueError, 'J4/J11'):
            solver.forward('left', low)

    @native_case
    def test_bad_goals_and_joint_seeds(self):
        for bad in (np.zeros((4, 4)), np.full((4, 4), np.nan), np.ones((3, 3))):
            with self.assertRaises(ValueError):
                self.solver.solve_body(dict(left=np.eye(4), right=bad), self.seed)
        for bad in (self.seed[:-1], np.full(20, np.nan), np.full(20, 50.)):
            with self.assertRaises(ValueError):
                self.solver.forward('left', bad)

    @native_case
    def test_bounded_online_tool_frame_and_active_torso_budget(self):
        online = adapter.Art7FairinoQP(tool_config('taccap'), optimize_tool_tcp=True)
        goals = {}
        for side in adapter.SIDES:
            goal = online.forward(side, self.seed)
            np.testing.assert_allclose(goal, self.solver.forward(side, self.seed), atol=1e-9, rtol=0.)
            goal[:3, 3] += [300., 0., -300.]
            goals[side] = goal
        arm = online.solve_arms(goals, self.seed, max_iterations=1, damping=.3)
        self.assertEqual(arm['iterations'], 1)
        locked = online.solve_body(goals, arm['q'], max_iterations=1)
        free = online.solve_body(goals, arm['q'], max_iterations=1, arms_first_iterations=0)
        np.testing.assert_array_equal(locked['q'][15:], arm['q'][15:])
        self.assertGreater(np.linalg.norm(free['q'][15:] - arm['q'][15:]), .001)
        for kwargs in (dict(max_iterations=0), dict(arms_first_iterations=-1)):
            with self.assertRaises(ValueError):
                online.solve_body(goals, self.seed, **kwargs)


if __name__ == '__main__':
    if NATIVE_CHILD:
        sys.argv.remove('--native-case')
    unittest.main()
