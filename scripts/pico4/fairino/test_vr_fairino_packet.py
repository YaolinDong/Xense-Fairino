"""Packet compatibility, session envelopes and the command filter (no native imports)."""
import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import unittest

import numpy as np

import paths
from body_core import load_config
from tcp_core import MODEL_ID, tcp_forward_matrix
from vr_core import (KIND, TORSO_JOINTS, TORSO_LIMITS, direct_positions, require_robot_request,
                     torso_in_limits, validate)
from vr_fairino_ik import AXES, BodyPath
from tools.ik_experiments.fairino_qp_art7 import TORSO_MAPPING, torso_angles


def pose(body, h=66.7, forward=0., roll=0., yaw=0., x=0.):
    model = copy.copy(body)
    model.x += x
    return np.asarray(model.curve(dict(height=h, forward=forward, roll=roll, yaw=yaw), [0.]*20)[0])


def arm_packet(q):
    arm = q[:7].tolist()
    return dict(kind=KIND, model=MODEL_ID, created=100., id=7,
                arms=dict(left=dict(reference=arm, target=arm, tcp=np.ravel(tcp_forward_matrix(arm)).tolist())))


class PacketContract(unittest.TestCase):
    def setUp(self):
        self.body = load_config()
        self.q = pose(self.body)
        self.hw = {'command_position': self.q.tolist()}
        self.packet = arm_packet(self.q)
        self.torso = dict(target=self.q[AXES].tolist(), path_offset_mm=0.)

    def test_old_outputs_remain_byte_identical(self):
        with self.assertRaisesRegex(ValueError, '协议'):
            validate(self.hw, dict(self.packet, kind='fairino_pico_cartesian_v4'), 100.)
        held = self.q.copy()
        held[14:] += .001
        for backend in (None, 'native', 'vendor', 'tron2'):
            if backend:
                self.packet['ik_backend'] = backend
            targets = validate({}, self.packet, 100.)
            self.assertEqual(json.dumps(targets), json.dumps({'left': self.q[:7].tolist()}))
            got = direct_positions(self.q.tolist(), targets, held.tolist())
            self.assertEqual(json.dumps(got), json.dumps([(0., self.q[:14].tolist()+held[14:].tolist())]))

    def test_torso_backend_required_in_both_directions(self):
        for backend in (None, 'native', 'vendor', 'tron2', 'unknown'):
            with self.assertRaises(ValueError):
                validate(self.hw, dict(self.packet, ik_backend=backend, torso=self.torso), 100., body=self.body)
        p = dict(self.packet, ik_backend='fairino_qp')
        for torso in (None, {}, dict(self.torso, target=[0.]*5), dict(self.torso, target=[math.nan]*4),
                      dict(self.torso, path_offset_mm=True), dict(self.torso, path_offset_mm=.051)):
            with self.assertRaises(ValueError):
                validate(self.hw, dict(p, torso=torso), 100., body=self.body)
        with self.assertRaises(ValueError):
            validate(self.hw, p, 100., body=self.body)
        p['torso'] = self.torso
        with self.assertRaises(ValueError):
            validate(self.hw, p, 100.)
        self.assertEqual(require_robot_request(p), 'fairino_qp')

    def test_legacy_held_head_and_roll_survive_torso_override(self):
        held = self.q.copy()
        held[14:] += .001
        old = direct_positions(self.q.tolist(), {}, held.tolist())[0][1]
        new = direct_positions(self.q.tolist(), {}, held.tolist(), self.torso)[0][1]
        self.assertEqual([i+1 for i, (a,b) in enumerate(zip(old,new)) if a != b], list(TORSO_JOINTS))
        self.assertEqual([new[14], new[16]], [held[14], held[16]])

    def test_same_hardware_positions_pass_as_before_rd53(self):
        # b80f539 receiver bounds in its original coordinate convention.
        old = ((20, -1, 0., .9599), (19, 1, 0., 2.2689), (18, -1, -1.0472, 1.0472),
               (17, 1, -.4887, .4887), (16, 1, -3.002, 3.002))
        self.assertEqual(tuple(row[:3] for row in TORSO_LIMITS), TORSO_MAPPING)
        for number, old_sign, lo, hi in old:
            offset = math.radians(self.body.neutral) if number == 18 else 0.
            for angle in (lo-.001, lo, lo+.001, hi-.001, hi, hi+.001):
                q = self.q.copy()
                q[number-1] = old_sign * (angle-offset)
                legacy_angle = old_sign*q[number-1] + offset
                with self.subTest(joint=number, angle=angle):
                    self.assertEqual(torso_in_limits(q, self.body), lo <= legacy_angle <= hi)

    def test_receiver_checks_both_path_offsets_jump_and_roll(self):
        def check(current, target, offset=0.):
            p = dict(self.packet, ik_backend='fairino_qp',
                     torso=dict(target=target[AXES].tolist(), path_offset_mm=offset))
            validate({'command_position': current.tolist()}, p, 100., body=self.body)

        off_path = pose(self.body, x=.1)
        for current, target in ((off_path, self.q), (self.q, off_path)):
            with self.assertRaisesRegex(ValueError, '偏离'):
                check(current, target)
        with self.assertRaisesRegex(ValueError, '跳变'):
            check(self.q, pose(self.body, yaw=1.51))
        with self.assertRaisesRegex(ValueError, '侧倾'):
            check(pose(self.body, roll=.21), self.q)
        bad = self.q.copy()
        bad[19] = math.radians(-60.)
        with self.assertRaisesRegex(ValueError, '软件限位'):
            check(bad, bad)

    def test_small_locked_roll_does_not_reject_mid_height(self):
        q = pose(self.body, forward=1., roll=.05)
        with self.assertRaises(ValueError):
            self.body.validate(self.body.state(q))
        path = BodyPath(self.body,q)
        p = dict(self.packet, ik_backend='fairino_qp', torso=dict(target=q[AXES].tolist(),
                                                                 path_offset_mm=path.offset))
        validate({'command_position':q.tolist()},p,100.,body=self.body)

    def test_35_to_100_percent_keeps_offset_at_every_receiver_step(self):
        start = pose(self.body,35.,1.,x=3.55)
        path = BodyPath(self.body,start)
        self.assertAlmostEqual(path.offset,3.55)
        previous = start
        for height in np.linspace(35.,100.,781):
            q = np.array(path.at_height(height,1.,0.,start)[0])
            p = dict(self.packet, ik_backend='fairino_qp', torso=dict(target=q[AXES].tolist(),
                                                                     path_offset_mm=path.offset))
            validate({'command_position':previous.tolist()},p,100.,body=self.body)
            self.assertAlmostEqual(self.body.state(q)['forward_offset_mm'],path.offset,places=10)
            previous = q

    def test_negative_starting_pitch_sets_worst_case_lower_height(self):
        q = pose(self.body,forward=-.8)
        path = BodyPath(self.body,q)
        self.assertTrue(34.45 < path.lower[0] < 34.55)
        self.assertFalse(torso_in_limits(pose(self.body,32.789909,-.8),self.body))
        for x in (-5.,0.,3.55,5.):
            for pitch in (-1.,0.,46.):
                q = pose(self.body,100.4,pitch,x=x)
                path = BodyPath(self.body,q)
                for h in np.linspace(path.lower[0],path.upper[0],33):
                    for p in np.linspace(path.lower[1],path.upper[1],5):
                        for y in (-90.,0.,90.):
                            full = path.at_height(h,p,y,q)[0]
                            native_angles = torso_angles(full,math.radians(self.body.neutral))
                            self.assertTrue(all(row[3] <= a <= row[4] for row,a in
                                                zip(TORSO_LIMITS,native_angles)))

    def test_braking_at_box_boundary_limits_command_deceleration(self):
        dt = 1/450
        q = pose(self.body,yaw=87.)
        path = BodyPath(self.body,q)
        coords, velocity = path.coordinates(q), np.zeros(3)
        samples = [q[AXES].copy()]
        for _ in range(900):
            coords,velocity,q = path.advance(coords,velocity,np.array([0.,0.,5.]),dt,q)
            samples.append(q[AXES].copy())
        speed = np.degrees(np.diff(samples,axis=0))/dt
        self.assertLessEqual(np.max(abs(speed)),5.+1e-8)
        self.assertLessEqual(np.max(abs(np.diff(speed,axis=0)))/dt,10.+1e-6)
        self.assertAlmostEqual(coords[2],90.,places=6)
        self.assertAlmostEqual(speed[-1,0],0.,places=5)

    def test_invalid_start_explains_body_controls(self):
        for q in (pose(self.body,30),pose(self.body,roll=.21),pose(self.body,x=5.1),
                  pose(self.body,forward=-1.1),pose(self.body,yaw=91.1)):
            with self.assertRaisesRegex(ValueError,'身体程序'):
                BodyPath(self.body,q)

    def test_robot_import_without_site_packages(self):
        code = ("import sys;sys.path[:0]="+repr([str(Path(__file__).parent),str(paths.TCP),
               str(paths.SHARED),str(paths.SHARED/'body_control')])+
                ";import vr_core;assert 'numpy' not in sys.modules")
        subprocess.run([sys.executable,'-B','-S','-c',code],check=True)
