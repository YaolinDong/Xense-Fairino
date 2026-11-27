"""Exercise the actual native adapter in fresh processes (Pinocchio 4 vs TRON2 3)."""
from functools import wraps
import hashlib
import json
import struct
import math
import os
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

import numpy as np

import paths
sys.path.insert(0, str(paths.ROOT))
from tcp_core import pose_error
from tool_frames import tool_config
from vendor_kinematics import VendorKinematics
from tools.ik_experiments.setup_fairino_ik import extension_path
from tools.ik_experiments.evaluate_fairino_qp import homes
from vr_core import direct_positions, validate
from vr_fairino_ik import AXES, FairinoQPIK, WORLD_FROM_V
from vr_fairino_input import FairinoInput
from vr_input import Clutch
from wrist_guard import WristGuard
from test_vr_fairino_packet import pose

CHILD = __name__ == '__main__' and '--native-case' in sys.argv
COMMITTED_TACCAP = dict(version=1, geometry_id='taccap-step-20261020-v1', tip_cad_mm=[0., 20.825, -152.712],
    arms=dict(left=dict(installed=True, roll_deg=180., confirmed=True),
              right=dict(installed=True, roll_deg=-180., confirmed=True)))


def native_case(fn):
    @wraps(fn)
    def run(self):
        if CHILD:
            return fn(self)
        proc = subprocess.run([sys.executable, '-B', str(Path(__file__).resolve()), '--native-case',
                               f'{type(self).__name__}.{fn.__name__}'], capture_output=True, text=True)
        self.assertEqual(proc.returncode, 0, proc.stdout+proc.stderr)
        self.assertNotIn('skipped=', proc.stderr)
    return run


@unittest.skipUnless(extension_path().is_file(), 'Fairino QP native extension absent')
class NativeTests(unittest.TestCase):
    def setUp(self):
        if CHILD:
            self.t = 4.
            # Committed TacCap geometry, independent of local edits to models/taccap/tool_config.json.
            self.tools = tool_config('taccap', COMMITTED_TACCAP)
            self.ik = FairinoQPIK(self.tools, clock=lambda: self.t)
            self.kin = VendorKinematics(tools=self.tools)
            self.q = pose(self.ik.body, roll=.05, x=3.55)
            self.q[:14] = homes()['default']
            self.q[14] = .22
            self.hw = dict(position=self.q.tolist(), command_position=self.q.tolist(), velocity=[0.]*20)
            self.goals = {s: WORLD_FROM_V[s].T @ self.ik.solver.forward(s,self.q) for s in WORLD_FROM_V}

    def send(self, goals=None, **kw):
        return self.ik.request(self.kin,self.hw,self.goals if goals is None else goals,1,123.,**kw)

    @native_case
    def test_command_bits_match_recorded_requests(self):
        # Recorded with RD53 geometry, the b80f539 position bounds and Pinocchio 4.1.0 (Home posture,
        # elbow lower bound, 40 iterations, per-cycle arm step bound), with fairino_ik 596f90d
        # (strict dual-wrist priority over the Home posture). Against the previous solver
        # (dd8cdc5) wrist errors match within 0.11 mm and 0.01 deg; only redundant arm joints
        # differ, by up to 1.8 deg. Include the side mask and every IEEE-754 target in wire
        # order. RECORD_DIGESTS=1 prints new values.
        for offset, sides, expected in (
                (.2, ('left', 'right'), 'aa60ef17be424fd2bc42de55b0b7afb3c429f5d6d899ee1a116728ab5cf07083'),
                (300., ('left', 'right'), '3ddf5e308e9a2fa7ed50638ba04bcf467b3b8113c64f6262f8e08b32f6f26663'),
                (2500., ('left',), '38f276d293fdc3c0ee8d5f2a0ad675ea29648a5a350f1cfbcd6d25fb21ef1fa9'),
                (1., ('right',), '4bf562bb57923c01174a24d4429eb331d7ac7282f2c9c7407659f8669febc3eb')):
            goals = {s: self.goals[s].copy() for s in sides}
            for side, goal in goals.items():
                goal[:3, 3] += WORLD_FROM_V[side][:3, :3].T @ [offset, 0., -offset]
            digest = hashlib.sha256()
            for i in range(12):
                self.t += .3 if i == 8 else 1/450
                if sides == ('right',):
                    guard = WristGuard()
                    with patch.object(guard, 'approach_scale', return_value=.5):
                        packet = self.send(goals, guard=guard)
                    self.assertLessEqual(packet['arms']['right']['wrist_scale'], .5)
                else:
                    packet = self.send(goals)
                self.assertEqual(tuple(packet['arms']), sides)
                digest.update(bytes([len(sides)]))
                next_q = self.hw['command_position'][:]
                for side, entry in packet['arms'].items():
                    digest.update(side.encode() + struct.pack('!7d', *entry['target']))
                    begin = 0 if side == 'left' else 7
                    next_q[begin:begin+7] = entry['target']
                torso = packet['torso']['target']
                digest.update(struct.pack('!4d', *torso))
                for axis, angle in zip(AXES, torso):
                    next_q[axis] = angle
                self.hw.update(command_position=next_q, position=next_q[:])
            if os.getenv('RECORD_DIGESTS'):
                print('DIGEST', offset, sides, digest.hexdigest())
                continue
            self.assertEqual(digest.hexdigest(), expected, (offset, sides))

    @native_case
    def test_normal_deceleration_may_increase_previous_tcp_error(self):
        self.send()
        self.ik.velocity[2] = .5
        self.t += 1/450
        p = self.send()
        self.assertAlmostEqual(math.degrees(p['torso']['target'][0]),(.5-10/450)/450,places=9)
        err = p['arms']['left']['native_ik']
        # With fairino_ik's posture task the arms follow the braking torso; only the bound matters.
        self.assertLess(err['position_error_mm'],.02)
        self.assertLess(err['orientation_error_deg'],.01)
        json.dumps(p,allow_nan=False)

    @native_case
    def test_single_grip_keeps_free_arm_and_session_offset(self):
        goal = self.goals['left'].copy()
        goal[:3,3] += WORLD_FROM_V['left'][:3,:3].T @ [300.,0.,-300.]
        prev_v = np.zeros(4)
        for _ in range(35):
            p = self.send({'left':goal})
            self.assertEqual(list(p['arms']),['left'])
            q = np.array(direct_positions(self.hw['command_position'],
                         validate(self.hw,p,123.,self.tools,self.ik.body),self.q.tolist(),p['torso'])[0][1])
            np.testing.assert_array_equal(q[7:15],self.q[7:15])
            self.assertEqual(q[16],self.q[16])
            self.assertAlmostEqual(p['torso']['path_offset_mm'],3.55)
            v = np.degrees(q[AXES]-np.array(self.hw['command_position'])[AXES])*450
            self.assertLessEqual(max(abs(v)),5.+1e-8)
            self.assertLessEqual(max(abs(v-prev_v))*450,20.+1e-7)
            prev_v = v
            self.hw.update(position=q.tolist(),command_position=q.tolist())
            self.t += 1/450
        self.assertGreater(max(abs(prev_v)),.001)

    @native_case
    def test_dt_cap_restart_and_path_not_redefined(self):
        self.send()
        self.ik.velocity[2] = 4.
        self.t += .08
        p = self.send()
        self.assertAlmostEqual(math.degrees(p['torso']['target'][0]),.02*(4.-10*.02),places=8)
        q = pose(self.ik.body,75.,1.,roll=.05,yaw=12.,x=3.55)
        q[:15] = self.q[:15]
        self.hw.update(position=q.tolist(),command_position=q.tolist())
        self.t += .251
        goals = {s: WORLD_FROM_V[s].T @ self.ik.solver.forward(s,q) for s in WORLD_FROM_V}
        restarted = self.send(goals)
        np.testing.assert_array_equal(restarted['torso']['target'],q[AXES])
        self.assertEqual(restarted['torso']['path_offset_mm'],p['torso']['path_offset_mm'])
        np.testing.assert_array_equal(self.ik.velocity, np.zeros(3))
        self.send({})
        q[19] += math.radians(.1)
        self.hw['command_position'] = q.tolist()
        with self.assertRaisesRegex(ValueError,'身体程序'):
            self.send(goals)

    @native_case
    def test_guard_paths_scaled_rebase_and_demo_only(self):
        with self.assertRaisesRegex(ValueError,'腕部联合限位'):
            self.send(guard=WristGuard(False))
        self.ik.demo_only = True
        self.assertTrue(self.send(guard=WristGuard(False))['demo_only'])
        self.ik.demo_only = False
        guard = WristGuard()
        self.hw['position'][0] += .001
        self.hw['velocity'][5] = .03
        goal = self.goals['left'].copy()
        goal[:3,3] += [.2,.3,.1]
        with patch.object(guard,'approach_scale',return_value=.5), patch.object(
                guard,'arm_segment',wraps=guard.arm_segment) as check:
            packet = self.send({'left':goal},guard=guard)
        self.assertEqual(check.call_count,2)
        self.assertEqual(check.call_args_list[0].args[0],self.q[:7].tolist())
        self.assertEqual(check.call_args_list[1].args[0],self.hw['position'][:7])
        self.assertEqual(check.call_args_list[1].args[2],self.hw['velocity'][:7])
        self.assertEqual(packet['arms']['left']['wrist_scale'],.5)
        hands = {s:s for s in WORLD_FROM_V}
        clutches = {s:Clutch(s) for s in hands}
        inp = FairinoInput(clutches,self.ik,delay_ms=0.)
        frame = {s:dict(pose=[0.,0.,0.,0.,0.,0.,1.],grip=1. if s=='left' else 0.,
                        primary=False,secondary=False) for s in hands}
        self.hw['velocity'] = [0.]*20
        anchored = inp.goals(frame,self.hw,self.kin,hands,1.,False,self.t)
        np.testing.assert_allclose(anchored['left'],self.goals['left'],atol=1e-8)
        frame['left']['pose'][2] -= .01  # Forward in PICO tracking coordinates.
        moved = inp.goals(frame,self.hw,self.kin,hands,1.,False,self.t+.01)
        delta = WORLD_FROM_V['left'] @ (moved['left']-anchored['left'])
        np.testing.assert_allclose(delta[:3,3],[10.,0.,0.],atol=1e-9)
        inp.accepted_plan(packet,frame,hands)
        expected = np.array(packet['arms']['left']['teleop_tcp']).reshape(4,4)
        rebased = inp.goals(frame,self.hw,self.kin,hands,1.,False,self.t+.02)
        np.testing.assert_allclose(rebased['left'],expected,atol=1e-9)
        frame['left']['grip'] = 0.
        self.assertEqual(inp.goals(frame,self.hw,self.kin,hands,1.,False,self.t+.03), {})
        self.assertIsNone(self.ik.last_time)
        inp.reset()
        self.assertIsNone(self.ik.last_time)
        self.assertFalse(inp.references)

    @native_case
    def test_mutated_tools_and_vendor_override_still_checked(self):
        self.send()
        row = self.kin.tools['transforms']['left'][0]
        original = row[1]
        for invalid in (False, 0, -0.):
            self.assertEqual(invalid, original)
            row[1] = invalid
            with self.assertRaises(ValueError):
                self.send()
        row[1] = original
        with patch.object(VendorKinematics, 'forward_tcp', autospec=True,
                          side_effect=ValueError('vendor failed')):
            with self.assertRaisesRegex(ValueError, 'vendor failed'):
                self.send()

    @native_case
    def test_wrong_fk_and_worsening_arm_rejected(self):
        original = self.ik.solver.solve_arms
        for mismatch in (True,False):
            def tampered(*args,**kwargs):
                result = original(*args,**kwargs)
                if mismatch:
                    result['tcp']['left'][0,3] += .021
                else:
                    result['q'][0] += .03
                    result['tcp']['left'] = self.ik.solver.forward('left',result['q'])
                return result
            with patch.object(self.ik.solver,'solve_arms',side_effect=tampered):
                with self.assertRaisesRegex(ValueError,'独立正解' if mismatch else '增大'):
                    self.send()
            self.assertIsNone(self.ik.last_time)

    @native_case
    def test_start_errors_mention_body_program(self):
        self.hw['command_position'][16] = math.radians(.201)
        with self.assertRaisesRegex(ValueError,'身体程序'):
            self.send()

    @native_case
    def test_body_direction_that_increases_error_does_not_accelerate(self):
        self.send()
        worse = self.q.copy()
        worse[15] -= math.radians(5.)
        nearer = self.q.copy()
        nearer[15] += math.radians(5.)
        goal = self.ik.solver.forward('left', nearer)
        baseline = self.ik.solver.forward('left', self.q)
        arms = dict(q=self.q.copy(), errors_mm_deg={'left': pose_error(goal, baseline)})
        with patch.object(self.ik.solver, 'solve_body', return_value={'q': worse}):
            velocity = self.ik._direction(self.ik.coords, self.q, arms, {'left': goal})
        np.testing.assert_array_equal(velocity, np.zeros(3))

    @native_case
    def test_both_tool_models_match_dh_above_arm_base(self):
        for mode in ('flange','taccap'):
            ik = FairinoQPIK(tool_config(mode))
            for side,angles in zip(WORLD_FROM_V,self.q[:14].reshape(2,7)):
                tcp = ik.teleop_tcp(side,self.q,ik.arm_tcp(side,angles))
                mm,deg = pose_error(WORLD_FROM_V[side]@tcp,ik.solver.forward(side,self.q))
                self.assertLess(mm,1e-7)
                self.assertLess(deg,1e-4)


if __name__ == '__main__':
    if CHILD:
        sys.argv.remove('--native-case')
    unittest.main()
