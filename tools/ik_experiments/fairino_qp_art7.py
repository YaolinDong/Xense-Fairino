"""Offline ART7 primitives. Poses use mm in a fixed x-forward, y-left, z-up frame;
joint vectors are hardware J1..J20 in radians. The head is locked to model zero.
"""
from functools import lru_cache
import hashlib
import importlib.util
import math
from pathlib import Path
import struct
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

from apps.fairino_tcp_probe.tcp_core import BOUNDS_DEG, FLANGE_OFFSET_MM
from apps.fairino_tcp_probe.tool_frames import for_arm
from apps.fairino_upper_probe.body_control.body_core import QP_HEAD_LIMITS, QP_TORSO_LIMITS, load_config
from apps.workbench.model import SHA256, URDF
from tools.ik_experiments.setup_fairino_ik import CACHE, PREFIX, check_linkage, extension_path

# Hardware joint number, URDF name, multiplier. J18 is expressed relative to the
# calibrated upright joint sum. J15 is measured but the solver never sees it.
ARM_NAMES = ('j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'j7',
             'j8', 'j9', 'j10', 'j11', 'j12', 'j13', 'j14')
HEAD_MAPPING = (15, 'j15', 0.)
TORSO_MAPPING = tuple(row[:3] for row in QP_TORSO_LIMITS)
WORLD_FROM_BASE = np.array([[0., -1., 0., 0.], [1., 0., 0., 0.],
                            [0., 0., 1., 0.], [0., 0., 0., 1.]])
SIDES = ('left', 'right')
POSITION_MM, ORIENTATION_DEG = .02, .01
NATIVE_DT = .02  # fairino_ik IKOptions.dt default
ARM_LIMITS = np.tile(np.radians(BOUNDS_DEG), (2, 1))
CAD_ANGLE_TOLERANCE = 1e-4
ELBOW_JOINTS = ('j4', 'j11')
ELBOW_INDEX = (3, 10)


def model_xml(elbow_min_deg=None):
    """RD53 geometry with project joint limits and exact ART7 DH arms.

    The RD53 arm origins and axes contain small CAD export residuals.
    Restore the nominal cardinal axes and quarter turns for comparison with
    the independent ART7 DH solver. Torso and mount geometry come from CAD.
    """
    original = URDF.read_bytes()
    if hashlib.sha256(original).hexdigest() != SHA256:
        raise ValueError('Whole-body URDF changed; recheck the joint mapping')
    root = ET.fromstring(original)
    bounds = dict(zip(ARM_NAMES, ARM_LIMITS, strict=True))
    bounds[HEAD_MAPPING[1]] = QP_HEAD_LIMITS
    bounds.update({name: (lower, upper) for _, name, _, lower, upper in QP_TORSO_LIMITS})
    for name, (lower, upper) in bounds.items():
        limit = root.find(f"joint[@name='{name}']/limit")
        limit.set('lower', str(float(lower)))
        limit.set('upper', str(float(upper)))
    for name in ARM_NAMES:
        joint = root.find(f"joint[@name='{name}']")
        origin = joint.find('origin')
        rpy = [float(value) for value in origin.get('rpy').split()]
        nominal = [round(value / (math.pi / 2)) * (math.pi / 2) for value in rpy]
        if any(abs(value - exact) > CAD_ANGLE_TOLERANCE for value, exact in zip(rpy, nominal)):
            raise ValueError(f'{name}: unexpected ART7 arm origin')
        origin.set('rpy', ' '.join(map(str, nominal)))
        axis = joint.find('axis')
        values = [float(value) for value in axis.get('xyz').split()]
        cardinal = [round(value) for value in values]
        if (sum(abs(v) for v in cardinal) != 1 or
                any(abs(v - n) > CAD_ANGLE_TOLERANCE for v, n in zip(values, cardinal))):
            raise ValueError(f'{name}: unexpected ART7 joint axis')
        axis.set('xyz', ' '.join(map(str, cardinal)))
    if elbow_min_deg is not None:
        # Solver model only: keep both elbows on the Home (positive) branch.
        # fairino_ik's posture task acts in the hands' null space and cannot
        # move an elbow back across the straight-arm singularity.
        for name in ELBOW_JOINTS:
            limit = root.find(f"joint[@name='{name}']/limit")
            limit.set('lower', str(max(float(limit.get('lower')), math.radians(elbow_min_deg))))
    return ET.tostring(root, encoding='utf-8', xml_declaration=True)


def model_path(tool_offsets=None, elbow_min_deg=None):
    xml = model_xml(elbow_min_deg)
    if tool_offsets is not None:
        root = ET.fromstring(xml)
        for side, offset in tool_offsets.items():
            name = f'{side}_teleop_tcp'
            ET.SubElement(root, 'link', name=name)
            joint = ET.SubElement(root, 'joint', name=name+'_fixed', type='fixed')
            ET.SubElement(joint, 'parent', link=f'{side}_waist2_Link')
            ET.SubElement(joint, 'child', link=name)
            ET.SubElement(joint, 'origin', xyz=' '.join(str(x/1000.) for x in offset[:3, 3]))
        xml = ET.tostring(root, encoding='utf-8', xml_declaration=True)
    directory = CACHE / 'models'
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / (hashlib.sha256(xml).hexdigest() + '.urdf')
    if not path.exists() or path.read_bytes() != xml:
        with tempfile.NamedTemporaryFile(dir=directory, delete=False) as stream:
            stream.write(xml)
        Path(stream.name).replace(path)
    return path


def _mapped_libraries():
    for row in Path('/proc/self/maps').read_text().splitlines():
        parts = row.split(maxsplit=5)
        if len(parts) == 6 and parts[5].startswith('/'):
            yield Path(parts[5].removesuffix(' (deleted)'))


def _check_process():
    if 'tron2_ik._native' in sys.modules:
        raise RuntimeError('tron2_ik native module already loaded; use a separate process')
    for path in set(_mapped_libraries()):
        if ('tron2_ik' in path.parts or 'tron2-ik' in path.parts) and '.so' in path.name:
            raise RuntimeError(f'tron2_ik native library already loaded: {path}')
        if path.name.startswith(('libpinocchio', 'liburdfdom', 'libboost_', 'libcoal',
                                 'libassimp', 'libocto', 'libqhull', 'libtinyxml2',
                                 'libconsole_bridge')) and not path.is_relative_to(PREFIX):
            raise RuntimeError(f'Conflicting native library already loaded: {path}')


@lru_cache(maxsize=1)
def _native(path):
    if not path.is_file():
        raise RuntimeError('Build fairino_ik with .local/pico4/env/bin/python '
                           'tools/ik_experiments/setup_fairino_ik.py --install')
    check_linkage(path)
    # Import just the pybind extension by absolute path. The upstream Python
    # package initializer probes other site-packages and globally preloads v3.
    spec = importlib.util.spec_from_file_location('fairino_art7._native', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    _check_process()
    return module


def _hardware(values, *, copy=False):
    q = np.asarray(values)
    if q.shape != (20,) or q.dtype.kind not in 'ifu' or not np.isfinite(q).all():
        raise ValueError('Expected 20 finite hardware joint angles in radians')
    if (q[:14] < ARM_LIMITS[:, 0]).any() or (q[:14] > ARM_LIMITS[:, 1]).any():
        raise ValueError('Arm seed exceeds BOUNDS_DEG')
    return q.astype(float, copy=copy)


def _torso_values(hardware, upright_rad):
    return [sign * hardware[number - 1] - (upright_rad if name == 'j18' else 0.)
            for number, name, sign in TORSO_MAPPING]


def torso_angles(hardware, upright_rad):
    return np.array(_torso_values(hardware, upright_rad))


def urdf_coordinates(hardware, upright_rad):
    """Mapped coordinates for independent FK, including out-of-range model audits."""
    q = _hardware(hardware)
    positions = dict(zip(ARM_NAMES, q[:14], strict=True))
    positions[HEAD_MAPPING[1]] = 0.
    positions.update({name: angle for (_, name, _), angle in
                      zip(TORSO_MAPPING, torso_angles(q, upright_rad), strict=True)})
    return positions


def _mm(flat):
    transform = np.array(flat, dtype=float).reshape(4, 4)
    transform[:3, 3] *= 1000.
    return transform


IDENTITY_3 = np.eye(3)


def matrix_error(a, b):
    """tcp_core.pose_error for arrays, preserving its row-major summation order."""
    left, right = a.ravel().tolist(), b.ravel().tolist()
    dx, dy, dz = a[:3, 3] - b[:3, 3]
    distance = math.sqrt(dx**2 + dy**2 + dz**2)
    # Built-in sum(float) uses compensated summation on Python 3.12.
    # The original sums NumPy scalars from left to right.
    trace = 0.
    for i in (0, 1, 2, 4, 5, 6, 8, 9, 10):
        trace += left[i]*right[i]
    return distance, math.degrees(math.acos(max(-1., min(1., (trace-1.)/2.))))


def rigid_transform(value, rotation_tol=1e-8):
    """Validate a 4x4 pose at the adapter boundary without allocating isclose masks."""
    t = np.asarray(value, dtype=float)
    if t.shape != (4, 4):
        raise ValueError('Expected a rigid 4x4 world TCP transform in mm')
    a, b, c, x, d, e, f, y, g, h, i, z, u, v, w, k = t.ravel().tolist()
    if not (abs(u) <= 1e-9 and abs(v) <= 1e-9 and abs(w) <= 1e-9 and abs(k-1.) <= 1e-9
            and math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
        raise ValueError('Expected a rigid 4x4 world TCP transform in mm')
    # Interior fast path. These dot products bound all coefficients by ~1,
    # so their rounding error is far below the half-tolerance margin. Use
    # the original matrix product and LAPACK determinant near either limit.
    margin = rotation_tol / 2
    if (abs(a*a + d*d + g*g - 1.) < margin and abs(b*b + e*e + h*h - 1.) < margin
            and abs(c*c + f*f + i*i - 1.) < margin and abs(a*b + d*e + g*h) < margin
            and abs(a*c + d*f + g*i) < margin and abs(b*c + e*f + h*i) < margin
            and abs(a*(e*i-f*h) - b*(d*i-f*g) + c*(d*h-e*g) - 1.) < margin):
        return t
    r = t[:3, :3]
    if (not np.isfinite(r).all() or not (abs(r.T @ r - IDENTITY_3) <= rotation_tol).all()
            or abs(np.linalg.det(r) - 1.) > rotation_tol):
        raise ValueError('Expected a rigid 4x4 world TCP transform in mm')
    return t


@lru_cache(maxsize=4)
def _native_target(values, inverse):
    # Key by the complete pose and calibration, including signed zero.
    frame = rigid_transform(np.frombuffer(values).reshape(4, 4))
    frame = frame.copy() if inverse is None else frame @ np.frombuffer(inverse).reshape(4, 4)
    frame[:3, 3] /= 1000.
    return tuple(frame.ravel().tolist())


@lru_cache(maxsize=64)
def _options(api, locked, iterations, arms_first, damping, posture=None, velocity=None):
    options = api.Options()
    options.head_position = 0.
    options.position_tolerance = POSITION_MM / 1000.
    options.orientation_tolerance = math.radians(ORIENTATION_DEG)
    if locked is not None:
        options.torso_position = struct.unpack('5d', locked)
    if iterations is not None:
        options.max_iterations = iterations
    if arms_first is not None:
        options.arms_first_iterations = arms_first
    if damping is not None:
        options.damping = damping
    if posture is not None:
        options.posture_weight = 1.
        options.posture_target = list(posture)
    if velocity is not None:
        options.default_velocity_limit = velocity
    return options


class Art7FairinoQP:
    def __init__(self, tools, *, optimize_tool_tcp=False, elbow_min_deg=None):
        _check_process()
        self._api = _native(extension_path())
        self.upright_rad = math.radians(load_config(live=True).neutral)
        self.optimize_tool_tcp = optimize_tool_tcp
        self.wrist_from_tool = {}
        for side in SIDES:
            offset = np.array(for_arm(tools, side), dtype=float)
            offset[:3, 3] += FLANGE_OFFSET_MM
            offset.setflags(write=False)
            self.wrist_from_tool[side] = offset
        # At the reach boundary a lower wrist cost can increase fingertip
        # error. Give Pinocchio the actual TCP Jacobian for bounded online IK.
        frame = 'teleop_tcp' if optimize_tool_tcp else 'waist2_Link'
        self.elbow_min = None if elbow_min_deg is None else math.radians(elbow_min_deg)
        path = model_path(self.wrist_from_tool if optimize_tool_tcp else None, elbow_min_deg)
        args = (str(path), 'left_'+frame, 'right_'+frame, WORLD_FROM_BASE.ravel().tolist())
        self._arms = self._api.Model(*args, False)
        self._body = self._api.Model(*args, True)
        # A selected native frame avoids marshalling every URDF link just
        # to independently verify the torso once per command.
        self._chest = self._api.Model(str(path), 'body_Link', 'body_Link', args[3], True)
        self._tool_inverse = {side: None if optimize_tool_tcp else np.linalg.inv(offset).tobytes()
                              for side, offset in self.wrist_from_tool.items()}
        names = list(ARM_NAMES) + [row[1] for row in TORSO_MAPPING]
        if list(self._body.joint_names) != names or self._body.head_joint != HEAD_MAPPING[1]:
            raise RuntimeError('Upstream joint order or head joint differs from the RD53 hardware mapping')
        self.torso_limits = np.asarray(self._body.torso_limits)
        self._torso_bounds = self.torso_limits.tolist()

    def _torso(self, q):
        torso = _torso_values(q, self.upright_rad)
        for (_, name, _), value, (lo, hi) in zip(TORSO_MAPPING, torso, self._torso_bounds, strict=True):
            if not lo <= value <= hi:
                raise ValueError(f'{name} outside Fairino QP limits: {value} not in [{lo}, {hi}] rad')
        return torso

    def _seed(self, q, active, max_iterations=None, arms_first_iterations=None, damping=None,
              posture=None, step_deg=None):
        if damping is not None:
            if not math.isfinite(damping) or damping < 0.:
                raise ValueError('damping must be finite and nonnegative')
            damping = float(damping)
        if max_iterations is not None:
            if type(max_iterations) is not int or max_iterations < 1:
                raise ValueError('max_iterations must be a positive integer')
        if arms_first_iterations is not None:
            if type(arms_first_iterations) is not int or arms_first_iterations < 0:
                raise ValueError('arms_first_iterations must be a nonnegative integer')
        values = q.tolist()
        if self.elbow_min is not None and min(values[i] for i in ELBOW_INDEX) < self.elbow_min - 1e-9:
            # The native model would clamp the elbow and return a wrong pose.
            raise ValueError(f'肘部 J4/J11 低于 {math.degrees(self.elbow_min):g}°；请先回 Home 或用关节程序把肘部调到 '
                             f'{math.degrees(self.elbow_min):g}° 以上')
        torso = self._torso(values)
        # Active models take the torso from the seed. Locked models take it
        # from options. Share only fully configured, never mutated options.
        locked = None if active else struct.pack('5d', *torso)
        if posture is not None:
            posture = tuple(float(v) for v in posture)
            if len(posture) != (19 if active else 14) or not all(map(math.isfinite, posture)):
                raise ValueError('posture must hold the model\'s 14 or 19 public joint values')
        velocity = None
        if step_deg is not None:
            # fairino_ik bounds every QP iteration by velocity * dt; spread the
            # per-solve joint step evenly over the iteration budget.
            if max_iterations is None or not math.isfinite(step_deg) or step_deg <= 0.:
                raise ValueError('step_deg needs a positive value and max_iterations')
            velocity = math.radians(step_deg) / (max_iterations * NATIVE_DT)
        options = _options(self._api, locked, max_iterations, arms_first_iterations, damping, posture, velocity)
        seed = values[:14]
        if active:
            seed.extend(torso)
        return seed, options

    def forward(self, side, hardware):
        """World-from-tool, with the torso taken from the same hardware reference."""
        if side not in SIDES:
            raise ValueError('Unknown arm side')
        seed, options = self._seed(_hardware(hardware), False)
        return self._forward(side, seed, options)

    def _forward(self, side, seed, options):
        frame = _mm(self._arms.forward_kinematics(seed, side, options))
        return frame if self.optimize_tool_tcp else frame @ self.wrist_from_tool[side]

    def frames(self, hardware):
        """Full native URDF placements in the same world frame, in mm."""
        seed, options = self._seed(_hardware(hardware), True)
        return {key: _mm(flat) for key, flat in self._body.frame_placements(seed, options).items()}

    def body_frame(self, hardware):
        seed, options = self._seed(_hardware(hardware), True)
        return _mm(self._chest.forward_kinematics(seed, 'left', options))

    def solve_arms(self, goals, hardware, *, max_iterations=None, damping=None, posture=None, step_deg=None):
        return self._solve(goals, hardware, False, max_iterations, damping=damping, posture=posture, step_deg=step_deg)

    def solve_body(self, goals, hardware, *, max_iterations=None, arms_first_iterations=None, positions_only=False,
                   posture=None, step_deg=None):
        return self._solve(goals, hardware, True, max_iterations, arms_first_iterations, positions_only=positions_only,
                           posture=posture, step_deg=step_deg)

    def _solve(self, goals, hardware, active, max_iterations, arms_first_iterations=None, damping=None,
               positions_only=False, posture=None, step_deg=None):
        if set(goals) != set(SIDES):
            raise ValueError('The dual-arm primitive requires both world TCP targets')
        model = self._body if active else self._arms
        q = _hardware(hardware, copy=True)
        seed, options = self._seed(q, active, max_iterations, arms_first_iterations, damping, posture, step_deg)
        targets = []
        for side in SIDES:
            frame = np.asarray(goals[side], dtype=float)
            if frame.shape != (4, 4):
                raise ValueError('Expected a rigid 4x4 world TCP transform in mm')
            targets.append(_native_target(frame.tobytes(), self._tool_inverse[side]))
        result = model.solve_ik(*targets, seed, options)
        solution = result.q
        q[:14] = solution[:14]
        if active:
            for (number, name, sign), value in zip(TORSO_MAPPING, solution[14:], strict=True):
                q[number - 1] = sign * (value + (self.upright_rad if name == 'j18' else 0.))
        # Check the native result once, then use the same validated seed
        # and options for both sides. A body direction only consumes joints.
        _hardware(q)
        if positions_only:
            self._torso(q.tolist())
            return dict(q=q)
        if active:
            arm_seed, options = self._seed(q, False)
        else:
            arm_seed = solution
        poses = {side: self._forward(side, arm_seed, options) for side in SIDES}
        errors = {side: matrix_error(np.asarray(goals[side], dtype=float), pose) for side, pose in poses.items()}
        reached = all(mm <= POSITION_MM and deg <= ORIENTATION_DEG for mm, deg in errors.values())
        return dict(q=q, tcp=poses, errors_mm_deg=errors, success=result.success and reached,
                    status='tcp_tolerance' if result.success and not reached else result.status,
                    iterations=result.iterations, qp_iterations=result.qp_iterations,
                    native_ms=result.solve_time_ms, collision_free=result.body_collision_free,
                    clearance_mm=result.body_min_distance * 1000.,
                    nearest=result.body_nearest_segment)
