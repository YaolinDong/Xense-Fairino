"""Offline regression checks. No SSH, ROS or motor commands."""
import io
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'apps/fairino_tcp_probe'))
sys.path.insert(0, str(ROOT/'scripts/pico4/fairino'))
from apps.workbench.model import RobotModel
from apps.workbench.network import route_info
from apps.workbench.odom_guard import input_problem
from apps.workbench.telemetry import fresh_hardware
from apps.workbench.telemetry_remote import feedback_packet
from apps.map_viewer import map_control_remote as mapping
from tools.ros_environment import feedback_environment
from vr_gui_ipc import GuiInput


class FeedbackTests(unittest.TestCase):
    def test_empty_ssh_heartbeats_do_not_hide_missing_hardware(self):
        waiting = feedback_packet({}, None, 11., 10.)
        self.assertIn('SSH 已连接', waiting['status'])
        self.assertNotIn('error', waiting)
        missing = feedback_packet({}, None, 16., 10.)
        self.assertIn('/hardware_states', missing['error'])
        self.assertIsNone(fresh_hardware(dict(missing, received=time.monotonic())))

    def test_stale_or_rejected_hardware_recovers_only_after_new_valid_sample(self):
        hardware = {'position': [1.]}
        for packet in (feedback_packet(hardware, 10., 11., 9.),
                       feedback_packet(hardware, 10., 10.1, 9., 'bad data')):
            self.assertIn('error', packet)
            self.assertIsNone(fresh_hardware(dict(packet, received=time.monotonic())))
        recovered = feedback_packet(hardware, 11., 11.01, 9.)
        self.assertNotIn('error', recovered)
        self.assertEqual(fresh_hardware(dict(recovered, received=time.monotonic())), hardware)

    def test_upper_observer_uses_existing_udp_profile_and_base_keeps_own_domain(self):
        upper = feedback_environment('upper')
        self.assertIn('export ROS_DOMAIN_ID=0', upper)
        self.assertIn('export FASTRTPS_DEFAULT_PROFILES_FILE=/home/fairino/fastrtps_udp.xml', upper)
        self.assertNotIn('unset FASTRTPS_DEFAULT_PROFILES_FILE', upper)
        base = feedback_environment('base')
        self.assertIn('export ROS_DOMAIN_ID=1', base)
        self.assertIn('unset FASTRTPS_DEFAULT_PROFILES_FILE', base)
        self.assertNotIn('/home/fairino/fastrtps_udp.xml', base)
        with self.assertRaises(ValueError):
            feedback_environment('unknown')

    def test_stale_transport_or_hardware_never_appears_live(self):
        now = time.monotonic()
        state = {'hardware': {'position': [1]}, 'age': .1, 'received': now}
        self.assertIsNotNone(fresh_hardware(state))
        for bad in ({**state, 'received': now-1}, {**state, 'age': 1}, {**state, 'error': 'disconnected'}):
            self.assertIsNone(fresh_hardware(bad))

    def test_guard_requires_real_fresh_wheel_data_even_if_filter_is_publishing(self):
        raw = {'received': 10, 'stamp_age': .01, 'values': [0]*9}
        hardware = {'received': 10, 'fault': False}
        self.assertIsNone(input_problem(raw, hardware, 10.1))
        self.assertIn('断流', input_problem(raw, hardware, 10.5))
        self.assertIn('故障', input_problem(raw, {**hardware, 'fault': True}, 10.1))
        self.assertIn('时间戳', input_problem({**raw, 'stamp_age': -2}, hardware, 10.1))
        self.assertIn('非有限', input_problem({**raw, 'values': [float('nan')]}, hardware, 10.1))

    def test_new_ekf_epoch_cannot_resume_old_map(self):
        with tempfile.TemporaryDirectory() as directory:
            odom, saved = Path(directory)/'odom', Path(directory)/'map'
            odom.write_text('session-A'); saved.write_text('session-A')
            with patch.object(mapping, 'ODOMETRY_EPOCH', odom), patch.object(mapping, 'MAPPING_EPOCH', saved):
                mapping.require_same_epoch()
                odom.write_text('session-B')
                with self.assertRaisesRegex(RuntimeError, '重新初始化'):
                    mapping.require_same_epoch()


class PanelFocusTests(unittest.TestCase):
    def test_mapping_page_routes_keys_to_base_and_releases_them_on_other_pages(self):
        from apps.workbench.app import Workbench
        events = []
        base = NS(input_enabled=False, dispatch=lambda kind, event: events.append(kind) or True)
        app = NS(pages={'map': None, 'body': None}, controls={'base': NS(window=base)})
        Workbench.switched(app, None, None, 0)
        self.assertTrue(base.input_enabled)
        self.assertTrue(Workbench.key_event(app, None, NS(keyval=1), 'key-press-event'))
        Workbench.switched(app, None, None, 1)
        self.assertFalse(base.input_enabled)
        self.assertEqual(events[-1], 'focus-out-event')
        self.assertFalse(Workbench.key_event(app, None, NS(keyval=1), 'key-press-event'))
        Workbench.key_event(app, None, NS(keyval=1), 'key-release-event')
        self.assertEqual(events[-1], 'key-release-event')

    def test_selected_panel_requires_real_window_focus(self):
        from apps.workbench.embedding import PanelHost
        # Exercise the adapter policy without creating a display or robot session.
        window = NS(is_active=lambda: False)
        panel = NS(owner=window, input_enabled=True)
        self.assertFalse(PanelHost.is_active(panel))
        window.is_active = lambda: True
        self.assertTrue(PanelHost.is_active(panel))
        panel.input_enabled = False
        self.assertFalse(PanelHost.is_active(panel))


class ObserverConnectionTests(unittest.TestCase):
    def test_missing_plotting_dependency_preserves_connection_without_starting_ssh(self):
        from apps.workbench.app import Workbench
        app = NS(closing=False, args=NS(demo=False), message=Mock(),
                 disconnect_observers=Mock(), credentials=Mock())
        real_import = __import__

        def missing_backend(name, *args, **kwargs):
            if name == 'apps.map_viewer.panel':
                raise ImportError("No module named 'gi._gi_cairo'")
            return real_import(name, *args, **kwargs)

        with patch('builtins.__import__', side_effect=missing_backend), \
                patch('apps.workbench.app.Feedback') as feedback:
            Workbench.connect_observers(app)
        app.disconnect_observers.assert_not_called()
        app.credentials.assert_not_called()
        feedback.assert_not_called()
        self.assertIn('bash setup.sh console', app.message.set_text.call_args.args[0])


class NetworkTests(unittest.TestCase):
    def test_actual_route_selects_wifi_or_wired_not_interface_name_guess(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            device = root/'arbitrary0'; device.mkdir()
            (device/'type').write_text('1'); (device/'device').mkdir()
            run = lambda *a, **k: NS(returncode=0, stdout=json.dumps([{'dev':'arbitrary0','prefsrc':'192.168.58.9'}]))
            self.assertEqual(route_info('192.168.58.3', run, root)['kind'], '有线')
            (device/'wireless').mkdir()
            result = route_info('192.168.58.3', run, root)
            self.assertEqual((result['kind'],result['source']), ('无线','192.168.58.9'))

    def test_missing_route_is_not_a_connection(self):
        result = route_info('192.168.58.3', lambda *a, **k: NS(returncode=2, stderr='unreachable'))
        self.assertIn('error', result)


class ModelTests(unittest.TestCase):
    def test_rd53_measured_joints_include_head_and_waist(self):
        model = RobotModel()
        hardware = {'joint_names':[f'j{i}' for i in range(1,21)], 'position':[.1]*14+[9.]*6}
        positions = model.measured_positions(hardware)
        self.assertEqual(len(positions), 20)
        self.assertEqual(positions['j15'], 9.)
        self.assertEqual(positions['j16'], 9.)
        self.assertEqual(positions['j17'], 9.)
        self.assertEqual(positions['j14'], .1)
        self.assertAlmostEqual(positions['j19'], 9.)
        self.assertAlmostEqual(positions['j20'], 9.)

    def test_demo_and_feedback_match_rd53_joint_order(self):
        import math
        from tools.ik_experiments.fairino_qp_art7 import TORSO_MAPPING, torso_angles
        model = RobotModel()
        q = [.01*i for i in range(20)]
        names = [f'j{i}' for i in range(1, 21)]
        hardware = dict(joint_names=names[::-1], position=q[::-1])
        default = model.measured_positions(hardware)
        demo = model.measured_positions(hardware, demo=True)
        self.assertEqual(default, demo)
        self.assertEqual(demo['j15'], q[14])
        for (_, name, _), angle in zip(TORSO_MAPPING, torso_angles(q, math.radians(model.body.neutral))):
            self.assertEqual(demo[name], angle)

    def test_saved_high_low_calibration_drives_same_height_and_chest_attitude(self):
        import numpy as np
        model = RobotModel()
        poses = []
        for height in (0, 100):
            q = model.body.pose(dict(height=height, forward=0, roll=0, yaw=0), [0.]*20)
            hw = {'joint_names': [f'j{i}' for i in range(1, 21)], 'position': q}
            positions = model.measured_positions(hw)
            transforms, _ = model.transforms(positions)
            poses.append(transforms)
            self.assertAlmostEqual(model.body_state(hw)['height'], height, places=7)
            np.testing.assert_allclose(transforms['body_Link'][:3, :3], np.diag([-1., -1., 1.]), atol=3e-5)
        self.assertAlmostEqual((poses[1]['hip_Link'][2,3]-poses[0]['hip_Link'][2,3])*1000,
                               model.body.travel, places=6)

    def test_urdf_arm_flange_matches_existing_verified_fk(self):
        import numpy as np
        from tcp_core import forward_matrix, wrist_to_flange
        from tcp_frames import CHEST_FROM_DH, CHEST_FROM_BODY
        model = RobotModel()
        angles = [.5, 1.2, -1.4, 1.6, .2, .1, -.2]
        positions = {f'j{i}': angles[(i-1)%7] for i in range(1,15)}
        transforms, _ = model.transforms(positions)
        body_inv = np.linalg.inv(transforms['body_Link'])
        for side, number in [('left',7),('right',14)]:
            joint = next(j for j in model.joints if j.get('name') == f'j{number}')
            link = joint.find('child').get('link')
            tip = np.eye(4); tip[0,3] = -.106
            measured = np.asarray(CHEST_FROM_BODY) @ body_inv @ transforms[link] @ tip
            measured[:3,3] *= 1000
            reference = np.asarray(CHEST_FROM_DH[side]) @ np.asarray(wrist_to_flange(forward_matrix(angles)))
            np.testing.assert_allclose(measured, reference, atol=.02)


class GuiIpcTests(unittest.TestCase):
    def test_only_explicit_ui_commands_are_translated(self):
        receiver = GuiInput(io.StringIO('{"command":"pause"}\n{"command":"home"}\n'))
        with patch('vr_gui_ipc.os.kill') as kill:
            receiver.read()
            self.assertEqual(receiver.key(), ' ')
            self.assertEqual(receiver.key(), 'h')
            kill.assert_called_once()  # EOF disconnect triggers existing cleanup.

    def test_arbitrary_robot_request_is_rejected(self):
        receiver = GuiInput(io.StringIO('{"request":{"joints":[1,2]}}\n'))
        with patch('vr_gui_ipc.os.kill') as kill:
            receiver.read()
            self.assertEqual(receiver.key(), '')
            kill.assert_called_once()


if __name__ == '__main__':
    unittest.main()
