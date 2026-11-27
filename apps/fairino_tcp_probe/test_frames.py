"""Chest-axis keyboard motion checked against the manufacturer's full URDF."""
import ast
import base64
import hashlib
import json
import math
from pathlib import Path
import shlex
import sys
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
from zipfile import ZipFile
import zlib

from tcp_core import ARMS, identity, pose_error, pose_matrix, target_matrix
from tcp_frames import (CHEST_FRAME_ID, SOURCE_ARCHIVE, SOURCE_SHA256,
                        CHEST_FROM_DH, tcp_in_chest, tcp_in_base)
from tcp_jog import make_request, validate_velocity, JOG_KIND
from vendor_kinematics import VendorKinematics
from test_tcp import hardware
from tcp_keyboard import remote_command
sys.path.insert(0, str(Path(__file__).parent/'analysis'))
from model_audit import axis_rotation, mmul


class ChestFrameTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kin = VendorKinematics()
        data = (Path(__file__).resolve().parents[2]/'models/whole_body_20261030.urdf').read_bytes()
        if hashlib.sha256(data).hexdigest() != SOURCE_SHA256:
            raise ValueError('厂家安装模型已改变，需重新核对')
        cls.joints = {j.get('name'): j for j in ET.fromstring(data).findall('joint')}

    def origin(self, joint):
        origin = joint.find('origin')
        r,p,y = map(float,origin.get('rpy').split())
        t = mmul(mmul(axis_rotation([0,0,1],y),axis_rotation([0,1,0],p)),axis_rotation([1,0,0],r))
        for i,x in enumerate(map(float,origin.get('xyz').split())): t[i][3] = x*1000.
        return t

    def full_flange(self, arm, q):
        # Chest forward = body Y, left = -body X, up = body Z.
        t = mmul([[0,1,0,0],[-1,0,0,0],[0,0,1,0],[0,0,0,1]],
                 self.origin(self.joints[arm+'_arm_fixed_j1']))
        for i,angle in enumerate(q,1 if arm=='left' else 8):
            joint = self.joints['j'+str(i)]
            axis = list(map(float,joint.find('axis').get('xyz').split()))
            t = mmul(mmul(t,self.origin(joint)),axis_rotation(axis,angle))
        offset = identity(); offset[0][3] = -106.
        return mmul(t,offset)

    def test_absolute_chest_pose_matches_full_model_and_inverse_roundtrips(self):
        for arm,joints in ARMS.items():
            q = [hardware()['position'][j-1] for j in joints]
            base,_ = self.kin.forward_tcp(q)
            chest = tcp_in_chest(arm,base)
            mm,deg = pose_error(chest,self.full_flange(arm,q))
            self.assertLess(mm,.02); self.assertLess(deg,.01)
            mm,deg = pose_error(base,tcp_in_base(arm,chest))
            self.assertLess(mm,1e-9); self.assertLess(deg,1e-5)
            self.assertAlmostEqual(CHEST_FROM_DH[arm][1][3],48. if arm=='left' else -48.,places=4)
            self.assertAlmostEqual(CHEST_FROM_DH[arm][2][3],330.,delta=.002)

    def test_all_24_signed_chest_targets_match_independent_full_urdf(self):
        hw = hardware()
        for arm,joints in ARMS.items():
            q = [hw['command_position'][j-1] for j in joints]
            start = self.full_flange(arm,q)
            for axis in range(6):
                for direction in (-1,1):
                    with self.subTest(arm=arm,axis=axis,direction=direction):
                        r = make_request(self.kin,hw,arm,axis,direction,8,2,4)
                        r.update(id=1,created=10.)
                        self.assertEqual(r['frame'],CHEST_FRAME_ID)
                        velocities = validate_velocity(hw,r,10.)
                        self.assertTrue(all(v==0. for i,v in enumerate(velocities,1) if i not in joints))
                        actual = self.full_flange(arm,r['target'])
                        expected = target_matrix(start,axis,direction,r['step'])
                        mm,deg = pose_error(expected,actual)
                        self.assertLess(mm,.002); self.assertLess(deg,.001)

    def test_base_axis_packets_and_mislabeled_targets_are_rejected(self):
        hw = hardware()
        r = make_request(self.kin,hw,'left',0,1,8,2,4)
        r.update(id=1,created=10.)
        for mutation in ({'kind':'tcp_hold_v1'},{'frame':'base'},{'frame':'art7-both-chest-urdf-20260731-v1'},{'frame':None}):
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):
                validate_velocity(hw,{**r,**mutation},10.)
        base,angle = self.kin.forward_tcp(r['reference'])
        r['target'] = self.kin.inverse_tcp(target_matrix(base,0,1,r['step']),angle,r['reference'])
        with self.assertRaisesRegex(ValueError,'正解'):
            validate_velocity(hw,r,10.)

    def test_chest_targets_do_not_depend_on_current_flange_orientation(self):
        from tcp_frames import chest_target_in_base
        for arm in ARMS:
            for pose in ([123.,234.,345.,0.,0.,0.],[123.,234.,345.,35.,-51.,77.]):
                base = tcp_in_base(arm,pose_matrix(pose))
                changed = tcp_in_chest(arm,chest_target_in_base(arm,base,0,1,.8))
                self.assertAlmostEqual(changed[0][3],123.8)
                self.assertAlmostEqual(changed[1][3],234.)
                self.assertAlmostEqual(changed[2][3],345.)

    def test_remote_source_bundle_loads_chest_math_without_files_or_ros(self):
        args = types.SimpleNamespace(check=True,prepare_check=False)
        setup = shlex.split(remote_command(args))[2]
        loader = shlex.split(setup)[-1]
        tree = ast.parse(loader)
        encoded = next(n.args[0].value for n in ast.walk(tree)
                       if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='b64decode')
        bundle = json.loads(zlib.decompress(base64.b64decode(encoded)))
        order = ast.literal_eval(next(n.iter for n in ast.walk(tree) if isinstance(n,ast.For)))
        self.assertLess(order.index('tcp_core'),order.index('tcp_frames'))
        self.assertLess(order.index('tcp_frames'),order.index('tcp_jog'))
        with patch.dict(sys.modules):
            for name in order:
                module = types.ModuleType(name)
                sys.modules[name] = module
                exec(compile(bundle[name+'.py'],name+'.py','exec'),module.__dict__)
            self.assertEqual(sys.modules['tcp_jog'].JOG_KIND,JOG_KIND)
            self.assertEqual(sys.modules['tcp_frames'].CHEST_FRAME_ID,CHEST_FRAME_ID)


if __name__ == '__main__': unittest.main()
