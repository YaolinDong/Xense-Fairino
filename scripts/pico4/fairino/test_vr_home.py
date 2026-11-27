"""Home planning/cancellation and independent position/rotation controls; no devices."""
import contextlib
import copy
import io
import json
import math
import queue
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
import paths
from lerobot.utils.rotation import Rotation
from mounts import tcp_in_chest
from tcp_core import ARMS, pose_error
from test_tcp import hardware
from test_vr_controls import hand
from vendor_kinematics import VendorKinematics
from tool_frames import load_geometry, tool_config
from wrist_guard import WristGuard
from vr_client import operate
from vr_controls import JoystickControls, sensitivity_step
from vr_home import DEFAULT_HOME, HomeMotion, load_home, resolve_home
from vr_input import Clutch, decode_packet, rotation_mapping
from vr_core import direct_positions as position_trajectory, validate
from apps.workbench.model import RobotModel


def frame(left=None, right=None):
    return {"stamp": 1, "left": left or hand(grip=0.), "right": right or hand(grip=0.)}


class RotationTests(unittest.TestCase):
    def test_room_translation_unchanged_but_rotation_is_anatomical_mirror_on_both_arms(self):
        directions = ([0., 0., -1.], [-1., 0., 0.], [0., 1., 0.])
        self.assertEqual(rotation_mapping("mirror_same_direction"), "mirror")
        self.assertEqual(rotation_mapping("first_person"), "first_person")
        self.assertEqual(rotation_mapping("mirror_same_direction", "mirror_same_direction"), "mirror_same_direction")
        for side in ARMS:
            for axis, direction in enumerate(directions):
                for sign in (-1, 1):
                    c = Clutch(side, "mirror_same_direction", "mirror")
                    tcp = np.eye(4); tcp[:3, 3] = [50., 100., 300.]
                    c.target(hand(), tcp, .5)
                    packet = hand(sign*.04*np.array(direction), sign*.2*np.array(direction))
                    out = c.target(packet, tcp, .5)
                    before, after = tcp_in_chest(side, tcp), tcp_in_chest(side, out)
                    # Same-room translation: operator forward/left/up -> robot back/right/up.
                    np.testing.assert_allclose(after[:3, 3]-before[:3, 3], sign*20.*np.diag([-1., -1., 1.])[:, axis], atol=1e-8)
                    # Mirrored angular vector: reflection flips the axial-vector parity.
                    expected = Rotation.from_rotvec(-sign*.1*np.diag([1., -1., 1.])[:, axis]).as_matrix()
                    np.testing.assert_allclose(after[:3, :3] @ before[:3, :3].T, expected, atol=1e-9)

    def test_mirrored_rotation_keeps_tcp_position_when_translation_is_frozen(self):
        c = Clutch("right", "mirror_same_direction", "mirror")
        tcp = np.eye(4); tcp[:3, 3] = [10., 20., 30.]
        c.target(hand(), tcp, .5)
        out = c.target(hand((.2, .1, .1), (.2, .3, .1)), tcp, .5, freeze_position=True)
        np.testing.assert_allclose(out[:3, 3], tcp[:3, 3])
        self.assertGreater(pose_error(tcp, out)[1], 1.)


class JoystickTests(unittest.TestCase):
    def test_decode_actual_sdk_axes_click_and_reject_invalid_values(self):
        controls = {s: {"pose": [0., 0., 0., 0., 0., 0., 1.], "grip": 0.,
                        "axisX": -.2, "axisY": .9, "axisClick": True} for s in ARMS}
        def decode(): return decode_packet(json.dumps({"value": {"timeStampNs": 1, "Controller": controls}}))
        self.assertTrue(decode()["right"]["axis_click"])
        self.assertTrue(decode()["left"]["joystick_available"])
        self.assertEqual(decode()["left"]["axis_y"], .9)
        for value in (float('nan'), 2., 'up', True):
            controls['left']['axisY'] = value
            with self.assertRaises(ValueError): decode()

    def test_stick_axes_are_reserved_for_motion_and_never_change_sensitivity(self):
        j = JoystickControls(); j.update(frame())
        up = frame({**hand(grip=0.), 'axis_y': .9})
        self.assertEqual(j.update(up)['sensitivity'], 0)
        for _ in range(20): self.assertEqual(j.update(up)['sensitivity'], 0)
        j.update(frame())
        down = frame({**hand(grip=0.), 'axis_y': -.9})
        self.assertEqual(j.update(down)['sensitivity'], 0)
        self.assertEqual(sensitivity_step(.25, 1), .5)
        self.assertEqual(sensitivity_step(.5, -1), .25)
        self.assertEqual(sensitivity_step(1., 1), 1.)

    def test_startup_or_reconnect_held_click_never_starts_home(self):
        j = JoystickControls(); click = frame(right={**hand(grip=0.), 'axis_click': True})
        self.assertFalse(j.update(click)['home'])
        j.update(frame()); self.assertTrue(j.update(click)['home'])
        self.assertFalse(j.update(click)['home'])
        j.reset(); self.assertFalse(j.update(click)['home'])
        j.update(frame()); self.assertTrue(j.update(click)['home'])
        j.update(frame())
        self.assertTrue(j.update(frame(left={**hand(grip=0.), 'axis_click': True}))['calibrate_heading'])

    def test_heading_click_is_edge_triggered_and_overrides_home_and_stick_deflection(self):
        j = JoystickControls()
        click = frame(left={**hand(grip=0.), 'axis_click': True, 'axis_y': .9},
                      right={**hand(grip=0.), 'axis_click': True})
        self.assertFalse(j.update(click)['calibrate_heading'])
        j.update(frame())
        self.assertEqual(j.update(click), {'calibrate_heading': True, 'sensitivity': 0, 'home': False})
        self.assertFalse(j.update(click)['calibrate_heading'])
        j.reset()
        self.assertFalse(j.update(click)['calibrate_heading'])


class HomeTests(unittest.TestCase):
    def setUp(self):
        self.home = load_home(); self.hw = hardware(); self.kin = VendorKinematics()

    def test_all_home_targets_pass_vendor_fk_ik(self):
        for q in self.home['target'].values(): self.assertLess(self.kin.verify_tcp(q)['roundtrip_deg'], .0001)

    def test_second_home_is_an_explicit_valid_mirror_with_default_and_override_preserved(self):
        second = load_home(resolve_home('right_mirrored'))
        self.assertEqual(resolve_home(), DEFAULT_HOME)
        self.assertEqual(resolve_home('right_mirrored', DEFAULT_HOME), DEFAULT_HOME)
        self.assertNotEqual(second['target'], self.home['target'])
        for q in second['target'].values():
            self.assertLess(self.kin.verify_tcp(q)['roundtrip_deg'], .0001)
        positions = {f'j{j}': value for side, joints in ARMS.items()
                     for j, value in zip(joints, second['target'][side])}
        transforms, _ = RobotModel().transforms(positions)
        # Torso mirror is world X; vendor wrist frames have mirrored local Y.
        mirrored = np.diag([-1., 1., 1., 1.]) @ transforms['right_waist2_Link'] @ np.diag([1., -1., 1., 1.])
        # Compare position and rotation in the same units as the ART7 FK gate.
        left = transforms['left_waist2_Link'].copy()
        left[:3, 3] *= 1000.
        mirrored[:3, 3] *= 1000.
        mm, deg = pose_error(left, mirrored)
        self.assertLess(mm, .02)
        self.assertLess(deg, .01)
        self.assertEqual(second['max_joint_speed_deg_s'], self.home['max_joint_speed_deg_s'])
        with self.assertRaises(ValueError):
            resolve_home('unknown')

    def test_cross_body_home_points_left_gripper_right_with_recorded_tool_mount(self):
        home = load_home(resolve_home('left_cross_body'))
        self.assertEqual(resolve_home(), DEFAULT_HOME)
        self.assertEqual(resolve_home('left_cross_body', DEFAULT_HOME), DEFAULT_HOME)
        self.assertEqual(home['target']['right'], self.home['target']['right'])
        # Verify the physical intent at the mounting angles used by the preview,
        # rather than depending on a later change to the user's tool settings.
        geometry = load_geometry()
        for side, roll in (('left', 180.), ('right', -180.)):
            geometry['arms'][side].update(installed=True, confirmed=True, roll_deg=roll)
        kin = VendorKinematics(tools=tool_config('taccap', geometry, True))
        for side, q in home['target'].items():
            with self.subTest(side=side):
                self.assertLess(kin.verify_tcp(q, side)['roundtrip_deg'], .0001)
        chest = tcp_in_chest('left', kin.forward_tcp(home['target']['left'], 'left')[0])
        np.testing.assert_allclose(chest[:3, 3], [300., -114.5, -99.], atol=.01)
        # The gripper projects along flange -X; chest -Y is robot right.
        direction = -chest[:3, 0]
        # Rounding the two CAD mount yaws to 1.5708 offsets the heading by 0.00042°.
        self.assertLess(math.degrees(math.atan2(math.hypot(direction[0], direction[2]),
                                                -direction[1])), .001)

    def test_cross_body_home_paths_from_both_presets_pass_wrist_margin_and_speed_reserve(self):
        home = load_home(resolve_home('left_cross_body'))
        for preset in ('default', 'right_mirrored'):
            with self.subTest(start=preset):
                source = load_home(resolve_home(preset))
                hw = hardware()
                for side, joints in ARMS.items():
                    for joint, value in zip(joints, source['target'][side]):
                        hw['position'][joint-1] = hw['command_position'][joint-1] = value
                # Constructor validates the full joint-space segment and adds
                # the peak-velocity stopping reserve, using the live planner.
                motion = HomeMotion(home, hw, 10., WristGuard())
                steps = math.ceil(motion.duration/.2)
                for step in range(steps+1):
                    targets = motion.sample(10.+motion.duration*step/steps)
                    for q in targets.values():
                        self.assertGreaterEqual(WristGuard.clearance(q[5:7]), 0.)
                for side in ARMS:
                    np.testing.assert_allclose(targets[side], home['target'][side], atol=1e-9)

    def test_path_has_bounded_home_speed_no_overshoot_and_never_moves_body(self):
        motion = HomeMotion(self.home, self.hw, 10.)
        last = motion.start
        samples = int(motion.duration/.02)+25
        max_speed = 0.
        for n in range(samples):
            now = 10.+n*.02
            req = motion.request(self.kin, self.hw, now, n+1, now)
            targets = validate(self.hw, req, now)
            for side in ARMS:
                for v, a, b, prev in zip(targets[side], motion.start[side], motion.target[side], last[side]):
                    self.assertLessEqual(min(a, b)-1e-12, v)
                    self.assertLessEqual(v, max(a, b)+1e-12)
                    max_speed = max(max_speed, abs(math.degrees(v-prev))/.02)
            trajectory = position_trajectory(self.hw['command_position'], targets, self.hw['command_position'])
            self.assertEqual(trajectory[-1][1][14:], self.hw['command_position'][14:])
            last = targets
        self.assertLessEqual(max_speed, self.home['max_joint_speed_deg_s']+1e-5)
        for side in ARMS: np.testing.assert_allclose(last[side], motion.target[side], atol=1e-9)

    def test_interrupted_home_cannot_jump_forward_after_long_stall(self):
        motion = HomeMotion(self.home, self.hw, 10.)
        motion.sample(10.1)
        with self.assertRaisesRegex(ValueError, '中断'): motion.sample(10.5)

    def test_reject_motion_and_invalid_model_or_limits(self):
        self.hw['velocity'][0] = .01
        with self.assertRaisesRegex(ValueError, '停止'): HomeMotion(self.home, self.hw, 0.)
        with tempfile.TemporaryDirectory() as d:
            f=Path(d)/'home.json'; raw=json.loads(DEFAULT_HOME.read_text())
            for change in ({'model':'wrong'}, {'joint_degrees':{'left':[200.]*7,'right':[0.]*7}}, {'max_joint_speed_deg_s':999.}):
                f.write_text(json.dumps({**raw,**change}))
                with self.assertRaises(ValueError): load_home(f)

    def test_arrival_requires_actual_feedback_settled_for_300ms(self):
        motion = HomeMotion(self.home, self.hw, 10.)
        end = 10.+motion.duration
        self.assertFalse(motion.complete(self.hw, end))
        for side, joints in ARMS.items():
            for j, v in zip(joints, motion.target[side]): self.hw['position'][j-1] = v
        self.assertFalse(motion.complete(self.hw, end+.1))
        self.assertFalse(motion.complete(self.hw, end+.3))
        self.assertTrue(motion.complete(self.hw, end+.5))


class StopSimulation(Exception): pass


class HomeClientTests(unittest.TestCase):
    def simulate(self, change=None, auto=True, loops=270):
        hw=hardware(); home=load_home(); kin=VendorKinematics()
        for side, joints in ARMS.items():
            for j,v in zip(joints,home['target'][side]): hw['position'][j-1]=hw['command_position'][j-1]=v
        hw['position'][0] += math.radians(4.); hw['command_position'][0]=hw['position'][0]
        clock=[10.]; sent=[]
        with tempfile.TemporaryDirectory() as d:
            c=NS(events=queue.Queue(),done=threading.Event(),last_status=10.,
                 latest={'hardware':hw,'stamp':10.,'ready':True},log_path=Path(d)/'client.jsonl')
            from test_probe import connected_wrist
            connected_wrist(c)
            def send(**kw):
                sent.append(copy.deepcopy(kw.get('request')))
                req=kw.get('request')
                if req:
                    for side,e in req['arms'].items():
                        for j,v in zip(ARMS[side],e['target']):hw['position'][j-1]=hw['command_position'][j-1]=v
                if len(sent)>=loops:raise StopSimulation()
            def sleep(dt):
                clock[0]+=.02;c.last_status=clock[0];c.latest['stamp']=clock[0]
            def read():
                f=frame()
                if change:change(f,len(sent))
                return f
            c.send=send
            args=NS(execute=True,sensitivity=.25,mode='mirror_same_direction',hand_mapping='crossed',
                    freeze_buttons=True,no_auto_home=not auto)
            with patch('vr_client.time.monotonic',lambda:clock[0]),patch('vr_client.time.sleep',sleep), \
                 patch('vr_client.key',return_value=''),patch('vr_client.keyboard',contextlib.nullcontext), \
                 contextlib.redirect_stdout(io.StringIO()),self.assertRaises(StopSimulation):
                operate(args,kin,NS(read=read),c)
            logs=[json.loads(x) for x in c.log_path.with_suffix('.actions.jsonl').read_text().splitlines()]
        return sent,logs

    def test_auto_home_completes_once_and_then_holds(self):
        sent,logs=self.simulate()
        self.assertEqual(sum(x.get('event')=='home_started' for x in logs),1)
        self.assertEqual(sum(x.get('event')=='home_completed' for x in logs),1)
        self.assertTrue(all(x is None or x.get('source')=='home' for x in sent))
        self.assertTrue(all(x is None for x in sent[-30:]))

    def test_grip_interrupts_home_and_release_does_not_restart_it(self):
        def change(f,n):
            if n==40:f['left']['grip']=1.
        sent,logs=self.simulate(change,loops=90)
        self.assertTrue(any(x and x.get('source')=='home' for x in sent[:40]))
        self.assertTrue(all(x is None for x in sent[40:]))
        self.assertTrue(any('中止 Home' in x.get('reason','') for x in logs))

    def test_right_stick_starts_home_and_second_press_cancels_it(self):
        def change(f,n):
            f['right']['axis_click']=n in (10,40)
        sent,logs=self.simulate(change,auto=False,loops=90)
        self.assertTrue(all(x is None for x in sent[:10]))
        self.assertTrue(any(x and x.get('source')=='home' for x in sent[10:40]))
        self.assertTrue(all(x is None for x in sent[40:]))

    def test_double_y_changes_sensitivity_and_cancels_pending_motion(self):
        def change(f,n):
            f['left']['secondary']=n in (20, 23)
        sent,logs=self.simulate(change,loops=70)
        self.assertTrue(any(x and x.get('source')=='home' for x in sent[20:24]))
        self.assertTrue(all(x is None for x in sent[24:]))
        self.assertEqual(sum('灵敏度已改为 0.5' in x.get('reason','') for x in logs),1)

    def test_pico_stream_loss_cancels_home_without_automatic_restart(self):
        def change(f,n):
            if n==40:raise ValueError('双手柄数据超过 0.25 秒未更新')
        sent,logs=self.simulate(change,loops=90)
        self.assertTrue(any(x and x.get('source')=='home' for x in sent[:40]))
        self.assertTrue(all(x is None for x in sent[40:]))
        self.assertEqual(sum(x.get('event')=='home_started' for x in logs),1)
        self.assertTrue(any('未更新' in x.get('reason','') for x in logs))


if __name__=='__main__':unittest.main()
