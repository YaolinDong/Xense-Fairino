"""Read-only URDF kinematics. Mapping is explicit; unverified axes stay static."""
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from apps.fairino_upper_probe.body_control.body_core import load_config

ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / 'scripts/pico4/fairino/models/fairino_wheel_legged_robot_description.urdf'
SHA256 = '96d4b4fb6e6a584bc55e1e9f1cf538ff9b02cc2093f1cda1cc983cd48199be9c'
UPPER_JOINTS = tuple(f'j{i}' for i in range(1, 18))


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*(skew @ skew)


def origin(joint):
    node = joint.find('origin')
    transform = np.eye(4)
    if node is None:
        return transform
    r, p, y = map(float, node.get('rpy', '0 0 0').split())
    transform[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
    transform[:3, 3] = np.fromstring(node.get('xyz', '0 0 0'), sep=' ')
    return transform


class RobotModel:
    def __init__(self, path=URDF):
        self.body = load_config()
        data = Path(path).read_bytes()
        if hashlib.sha256(data).hexdigest() != SHA256:
            raise ValueError('整机 URDF 已变化，请重新核对关节映射')
        self.joints = ET.fromstring(data).findall('joint')
        children = {j.find('child').get('link') for j in self.joints}
        parents = {j.find('parent').get('link') for j in self.joints}
        self.root, = parents - children

    def transforms(self, positions):
        transforms = {self.root: np.eye(4)}
        remaining = self.joints[:]
        segments = []
        while remaining:
            progressed = False
            for joint in remaining[:]:
                parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
                if parent not in transforms:
                    continue
                local = origin(joint)
                name = joint.get('name')
                angle = positions.get(name, 0.)
                if not math.isfinite(angle):
                    raise ValueError('非有限关节角')
                if joint.get('type') in ('revolute', 'continuous'):
                    turn = np.eye(4)
                    turn[:3, :3] = rotation(np.fromstring(joint.find('axis').get('xyz'), sep=' '), angle)
                    local = local @ turn
                transforms[child] = transforms[parent] @ local
                segments.append((name, transforms[parent][:3, 3], transforms[child][:3, 3]))
                remaining.remove(joint)
                progressed = True
            if not progressed:
                raise ValueError('URDF 关节树不完整')
        for number in (7, 14):
            joint = next(j for j in self.joints if j.get('name') == f'j{number}')
            transform = transforms[joint.find('child').get('link')]
            flange = transform @ np.array([-.106, 0, 0, 1])
            segments.append((f'j{number}_flange', transform[:3, 3], flange[:3]))
        return transforms, segments

    def measured_positions(self, hardware, *, demo=False):
        if not hardware:
            return {}
        names, values = hardware.get('joint_names', []), hardware.get('position', [])
        if len(names) != 20 or len(values) != 20 or len(set(names)) != 20:
            raise ValueError('上半身关节反馈格式不符')
        measured = dict(zip(names, values))
        result = {name: measured[name] for name in UPPER_JOINTS}
        if self.body.config.get('upright_confirmed'):
            result.update({name: measured[name] for name in ('j18', 'j19', 'j20')})
            # Display convention only: the saved user-declared vertical reference
            # defines zero chest pitch. Do not alter encoders or motion targets.
            result['j18'] -= math.radians(self.body.neutral)
        return result

    def body_state(self, hardware):
        if not hardware:
            return None
        measured = dict(zip(hardware['joint_names'], hardware['position']))
        return self.body.state([measured[f'j{i}'] for i in range(1, 21)])


class ModelPlot:
    def __init__(self):
        from matplotlib.figure import Figure
        self.model = RobotModel()
        self.figure = Figure(figsize=(4.2, 5), dpi=100, facecolor='#f6f8fa')
        self.ax = self.figure.add_subplot(111, projection='3d')
        self.ax.view_init(elev=18, azim=-60)
        self.ax.set(xlabel='X / m', ylabel='Y / m', zlabel='Z / m',
                    xlim=(-1, 1), ylim=(-1, 1), zlim=(-.3, 1.8))
        self.ax.set_box_aspect((2, 2, 2.1))
        self.figure.subplots_adjust(left=0, right=1, top=.89, bottom=.03)
        self.title = self.figure.suptitle('运动学模型 · 等待实测反馈', fontsize=11)
        self.lines = {}
        _, segments = self.model.transforms({})
        for name, a, b in segments:
            color = '#b2b9c2'
            line, = self.ax.plot(*np.array([a, b]).T, '-o', linewidth=5 if name in UPPER_JOINTS else 3,
                                 markersize=3, color=color)
            self.lines[name] = line
        self.last_positions = {}

    def update(self, hardware, live_arms=True):
        fresh = hardware is not None and live_arms
        if fresh:
            self.last_positions = self.model.measured_positions(hardware)
        _, segments = self.model.transforms(self.last_positions)
        for name, a, b in segments:
            line = self.lines[name]
            values = np.array([a, b]).T
            line.set_data_3d(*values)
            source = name.split('_')[0]
            color = '#168caf' if source in {f'j{i}' for i in range(1, 8)} else '#ef852b'
            if source in ('j15', 'j16', 'j17', 'j18', 'j19', 'j20'):
                color = '#6b8e64'
            line.set_color(color if fresh and source in self.last_positions else '#b2b9c2')
        self.title.set_text('实测双臂 / 头部 / 躯干 · 身体几何为估算' if fresh else
                            '实时模型尚未启用 · 静态参考' if not live_arms else
                            '模型数据未更新 · 灰色为参考 / 最后姿态')
