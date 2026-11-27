"""Fixed-base ART7 whole-body teleoperation. The command clock is local and monotonic."""
import copy
import math
import marshal
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

from paths import ROOT
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from body_core import load_config
from mounts import BODY_FROM_DH, CHEST_FROM_DH, MODEL_PATH, origin_matrix
from tcp_core import ARMS, MODEL_ID, check_arm_limits, finite_vector, multiply, tcp_forward_matrix, wrist_to_flange
from tool_frames import for_arm, validate_runtime
from tools.ik_experiments.fairino_qp_art7 import (
    Art7FairinoQP, ORIENTATION_DEG, POSITION_MM, WORLD_FROM_BASE,
    TORSO_MAPPING, matrix_error, rigid_transform, torso_angles,
)
from vendor_kinematics import VendorKinematics
from vr_core import (BODY_EPS, KIND, PATH_LIMIT_MM, PATH_TOL_MM, TORSO_JOINTS, TORSO_ROLL_DEG,
                     torso_in_limits, validate)
from vr_home import load_home, resolve_home
from vr_plan import interpolate_tcp
from wrist_guard import WristBlocked, WristGuard

# fairino_ik as configured upstream: Home posture task, its default damping,
# and enough iterations for the null-space pull to settle.
N_ARMS, N_BODY = 40, 6
ELBOW_MIN_DEG = 10.  # Solver model only; keeps both elbows on the Home branch.
# The posture target is moved towards Home by at most this rate, so the
# null-space return is spread over cycles instead of snapping in one solve.
HOME_RETURN_DEG_S = 30.
# Per-cycle arm joint step, enforced inside fairino_ik's QP (velocity * dt per
# iteration). Without it a near-singular solve can swing a redundant joint by
# tens of degrees in one command.
ARM_STEP_DEG_S = 360.
W_MM, W_DEG = 30., 8.
SPEED_DEG_S, ACCEL_DEG_S2 = 5., 10.
MAX_DT, RESTART_GAP = .020, .25
HEIGHT_MARGIN = .5
PROBE_STEP_DEG = .01
COST_EPS = 1e-12
ROUNDING_RAD = 2e-14
AXES = [j-1 for j in TORSO_JOINTS]
_VENDOR_FORWARD_TCP = VendorKinematics.forward_tcp
ARM_SLICE = {'left': slice(0, 7), 'right': slice(7, 14)}
WORLD_FROM_V = {}
for _side, _mount in CHEST_FROM_DH.items():
    WORLD_FROM_V[_side] = np.eye(4)
    WORLD_FROM_V[_side][:3, :3] = _mount[:3, :3]


def _tool_signature(config):
    # Compare only the fields consumed by validate_runtime. marshal v2
    # distinguishes lists, tuples, ints, bools, floats and signed zero; dict
    # equality alone would miss changes that invalidate the JSON fingerprint.
    if type(config) is dict:
        try:
            return marshal.dumps((config.get('mode'), config.get('id'), config.get('transforms')), 2)
        except (TypeError, ValueError, OverflowError):
            pass
    return None


def _cost(error):
    mm, deg = error
    return (mm/1000.)**2 + math.radians(deg)**2


def _worse_than(error, reference):
    """A solution may not move the TCP further from its goal than holding.

    Inside the validation tolerance it is accepted: fairino_ik's Home posture
    task works in the hands' null space and its linearized steps change an
    already-reached TCP by far less than 0.02 mm / 0.01 deg.
    """
    if error[0] <= POSITION_MM and error[1] <= ORIENTATION_DEG:
        return False
    return _cost(error) > _cost(reference) + COST_EPS*(1.+_cost(reference))


def _assert_fk(a, b, label):
    mm, deg = matrix_error(a, b)
    if not (mm <= POSITION_MM and deg <= ORIENTATION_DEG):
        raise ValueError(f'{label} 独立正解不一致 ({mm:.5g} mm, {deg:.5g}°)')


def _body_error(message):
    return ValueError(f'{message}；请用身体程序 bash run.sh body 回正侧倾、对齐升降路径，'
                      '前倾与腰旋转回零，高度可选 66.7% 后重试')


class BodyPath:
    """Constant pivot x, positive-knee branch. Coordinates are knee, pitch and yaw (degrees).

    Knee angle parametrizes height monotonically, avoiding the large dq/dheight
    close to a straight knee. Targets and packet bounds still use physical height.
    """
    def __init__(self, body, q):
        self.body = body
        start = body.state(q)
        self.offset = start['forward_offset_mm']
        self.start_target = tuple(q[i] for i in AXES)
        self.model = copy.copy(body)
        self.model.x += self.offset
        h, pitch, yaw = (start[k] for k in ('height', 'forward', 'yaw'))
        if (abs(start['roll']) > TORSO_ROLL_DEG+BODY_EPS or abs(self.offset) > PATH_LIMIT_MM+BODY_EPS
                or h > 101.+BODY_EPS or not -1.-BODY_EPS <= pitch <= 46.+BODY_EPS
                or abs(yaw) > 91.+BODY_EPS):
            raise _body_error('启动躯干姿态超出范围')
        self.lower = np.array([0., min(0., pitch), min(-90., yaw)])
        self.upper = np.array([max(h, 100.), max(45., pitch), max(90., yaw)])
        left, right = 0., 100.
        for _ in range(48):
            mid = (left+right)/2
            candidate = self.at_height(mid, self.lower[1], yaw, q)[0]
            if torso_in_limits(candidate, body):
                right = mid
            else:
                left = mid
        self.lower[0] = right+HEIGHT_MARGIN
        if h < self.lower[0]-BODY_EPS:
            raise _body_error(f'当前路径请升至 {math.ceil(self.lower[0]*100)/100:.2f}% 以上')
        if not torso_in_limits(q, body):
            raise _body_error('启动躯干姿态超出 Fairino QP 软件限位')
        slopes = []
        for height in np.linspace(self.lower[0], self.upper[0], 33):
            for p in (self.lower[1], self.upper[1]):
                for y in (self.lower[2], self.upper[2]):
                    candidate, dz, _ = self.at_height(height, p, y, q)
                    if not torso_in_limits(candidate, body):
                        raise _body_error('本次身体包络超出 Fairino QP 软件限位')
            slopes.append(abs(dz[17]/dz[18]))
        self.coord_min = self.lower.copy()
        self.coord_max = self.upper.copy()
        self.coord_min[0] = math.degrees(self.at_height(self.upper[0], pitch, yaw, q)[0][18])
        self.coord_max[0] = math.degrees(self.at_height(self.lower[0], pitch, yaw, q)[0][18])
        # Pitch and knee acceleration share the J18 budget. Keep a margin for
        # the curvature of q(knee) and the final joint-speed scaling.
        knee_share = min(.9, .45/max(slopes))
        self.accel = ACCEL_DEG_S2 * np.array([knee_share, .45, 1.])

    def at_height(self, height, forward, yaw, q):
        state = dict(height=height, forward=forward, yaw=yaw, roll=math.degrees(q[16]))
        return self.model.curve(state, q)

    def coordinates(self, q):
        state = self.body.state(q)
        if (abs(state['forward_offset_mm']-self.offset) > PATH_TOL_MM
                or abs(state['roll']) > TORSO_ROLL_DEG+BODY_EPS or not torso_in_limits(q, self.body)):
            raise _body_error('当前躯干指令已偏离本次会话')
        result = np.array([math.degrees(q[18]), state['forward'], state['yaw']])
        if np.any(result < self.coord_min-BODY_EPS) or np.any(result > self.coord_max+BODY_EPS):
            raise _body_error('当前躯干指令超出本次包络')
        return result

    def height(self, knee):
        m = self.model
        r2 = m.l1**2 + m.l2**2 + 2*m.l1*m.l2*math.cos(math.radians(knee))
        return (math.sqrt(r2-m.x**2)-m.z_low)*100/m.travel

    def joints(self, coords, reference):
        q, _, _ = self.at_height(self.height(coords[0]), *coords[1:], reference)
        return np.asarray(q)

    def desired(self, coords, body_solution, reference, gain):
        state = self.body.state(body_solution)
        target = np.clip([state['height'], state['forward'], state['yaw']], self.lower, self.upper)
        h = self.height(coords[0])
        _, dz, _ = self.at_height(h, *coords[1:], reference)
        dh_mm = (target[0]-h)*self.model.travel/100.
        dp, dy = target[1:] - coords[1:]
        dk = math.degrees(dz[18]*dh_mm)
        joint_dir = np.array([dy, dp+math.degrees(dz[17]*dh_mm), dk, math.degrees(dz[19]*dh_mm)])
        norm = max(abs(joint_dir))
        if norm <= BODY_EPS:
            return np.zeros(3)
        return np.array([dk, dp, dy])*(SPEED_DEG_S*gain/norm)

    def advance(self, coords, velocity, desired, dt, reference):
        if dt == 0.:
            return coords.copy(), velocity.copy(), reference.copy()
        # Each sample consumes v*dt before the remaining continuous braking
        # distance. Leave extra braking authority for variable command intervals.
        distance = np.where(desired >= 0., self.coord_max-coords, coords-self.coord_min)
        braking = .8*self.accel
        bound = np.sqrt((braking*dt)**2+2*braking*np.maximum(distance, 0.))-braking*dt
        desired = np.sign(desired)*np.minimum(abs(desired), bound)
        speed = velocity + np.clip(desired-velocity, -self.accel*dt, self.accel*dt)
        next_coords = coords + speed*dt
        if np.any(next_coords < self.coord_min-BODY_EPS) or np.any(next_coords > self.coord_max+BODY_EPS):
            raise ValueError('躯干滤波结果超出会话包络')
        if np.array_equal(next_coords, coords):
            return next_coords, speed, reference.copy()
        next_q = self.joints(next_coords, reference)
        limit = math.radians(SPEED_DEG_S)*dt
        # Rescale in body coordinates so x stays fixed even on a curved path.
        for _ in range(3):
            step = max(abs(next_q[AXES]-reference[AXES]))
            if step <= limit+ROUNDING_RAD:
                break
            speed *= (limit/step)*(1.-1e-9)
            next_coords = coords+speed*dt
            next_q = self.joints(next_coords, reference)
        if max(abs(next_q[AXES]-reference[AXES])) > limit+ROUNDING_RAD:
            raise ValueError('躯干指令超出本周期速度限制')
        return next_coords, speed, next_q


class _CycleFK:
    """Reuse the independent FK for identical joint bits within one request."""
    def __init__(self, backend):
        self.backend = backend
        self.bodies = {}
        self.arms = {}

    def world(self, q):
        key = q[15:].tobytes()
        if key not in self.bodies:
            self.bodies[key] = self.backend.world_from_body(q)
        return self.bodies[key]

    def tcp(self, side, q):
        key = side, q.tobytes()
        if key not in self.arms:
            self.arms[key] = self.backend.arm_tcp(side, q)
        return self.arms[key]


class FairinoQPIK:
    demo_only = False

    def __init__(self, tools, *, clock=time.monotonic):
        self.tools = copy.deepcopy(validate_runtime(tools))
        self._tool_signature = _tool_signature(self.tools)
        self.tool = {side: np.asarray(for_arm(self.tools, side)) for side in ARMS}
        self.solver = Art7FairinoQP(self.tools, optimize_tool_tcp=True, elbow_min_deg=ELBOW_MIN_DEG)
        self.set_home(load_home(resolve_home('default'))['target'])
        self.body = load_config(live=True)
        self.clock = clock
        self.path = None
        self.reset()
        root = ET.parse(MODEL_PATH)
        self.chain = []
        for _, name, _ in TORSO_MAPPING:
            joint = root.find(f"./joint[@name='{name}']")
            axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
            # The pinned torso URDF contains only +Z and -Z revolutes.
            if tuple(axis) not in ((0., 0., 1.), (0., 0., -1.)):
                raise ValueError(f'{name} 躯干 URDF 旋转轴已变化')
            self.chain.append((origin_matrix(joint), float(axis[2])))
        seed = np.array(self.body.pose(dict(height=100., forward=0., roll=0., yaw=0.), [0.]*20))
        for angles in (np.radians([0., 0., 0., 30., 0., 0., 0.]), np.radians([45., 75., -100., 85., 85., 20., -10.])):
            seed[:14] = np.tile(angles, 2)
            body = self.world_from_body(seed)
            _assert_fk(body, rigid_transform(self.solver.body_frame(seed), 1e-6), '启动躯干')
            for side, arm in ARM_SLICE.items():
                full = body @ BODY_FROM_DH[side] @ self.arm_tcp(side, seed[arm])
                _assert_fk(full, self.solver.forward(side, seed), f'启动 {side}')

    def set_home(self, target):
        """Home arms (left 7, right 7, radians): fairino_ik's posture target."""
        home = np.array(list(target['left'])+list(target['right']), dtype=float)
        if home.shape != (14,) or not np.isfinite(home).all() or min(home[[3, 10]]) < math.radians(ELBOW_MIN_DEG):
            raise ValueError('Fairino QP 的 Home 必须是 14 个有限关节角，肘部不低于 %g°' % ELBOW_MIN_DEG)
        self.home = home

    def _solve_arms(self, goals, q, dt):
        """fairino_ik's arm QP with the Home posture task.

        The posture pull is a null-space task. Its target is an intermediate
        point towards Home; if it cannot be met without moving the hands, the
        plain solve is used for that cycle only.
        """
        step = math.radians(HOME_RETURN_DEG_S) * (dt if dt > 0. else MAX_DT)
        posture = q[:14] + np.clip(self.home - q[:14], -step, step)
        bound = ARM_STEP_DEG_S * (dt if dt > 0. else MAX_DT)
        result = self.solver.solve_arms(goals, q, max_iterations=N_ARMS, posture=posture, step_deg=bound)
        if result['collision_free'] and not self._worse(goals, result, q):
            return result
        return self.solver.solve_arms(goals, q, max_iterations=N_ARMS, step_deg=bound)

    def _worse(self, goals, result, q):
        for side in ARMS:
            hold = self.solver.forward(side, q)
            if _worse_than(matrix_error(goals[side], result['tcp'][side]), matrix_error(goals[side], hold)):
                return True
        return False

    def _validated_tools(self, config):
        if self._tool_signature is not None and _tool_signature(config) == self._tool_signature:
            return config
        return validate_runtime(config)

    def reset(self):
        # Grip release/rebase restarts the filter but retains the session path.
        self.last_time = None
        self.velocity = np.zeros(3)
        self.last_q = self.coords = None

    def start_path(self, hardware):
        """Verify the activated controller's hold before releasing the enable handshake."""
        q = np.array(finite_vector(hardware.get('command_position'), 20, '躯干启动指令'))
        path = BodyPath(self.body, q)
        measured = finite_vector(hardware.get('position'), 20, '躯干启动实测')
        if not np.array_equal(measured[15:20], q[15:20]):
            try:
                BodyPath(self.body, measured)
            except ValueError as exc:
                raise ValueError(f'实测躯干不满足启动条件：{exc}') from exc
        self.path = path

    def world_from_body(self, q):
        matrix = WORLD_FROM_BASE
        angles = torso_angles(q, math.radians(self.body.neutral))
        r = np.eye(3)
        for (origin, sign), angle in zip(self.chain, angles, strict=True):
            matrix = matrix @ origin
            # Keep the Rodrigues diagonal rounding used by model.rotation.
            r[0, 0] = r[1, 1] = 1.-(1.-math.cos(angle))
            sine = sign*math.sin(angle)
            r[1, 0] = 0. + sine
            r[0, 1] = 0. - sine
            matrix[:3, :3] = matrix[:3, :3] @ r
        return matrix

    def arm_tcp(self, side, q):
        return np.asarray(tcp_forward_matrix(q.tolist())) @ self.tool[side]

    def teleop_tcp(self, side, q, arm_tcp):
        return WORLD_FROM_V[side].T @ self.world_from_body(q) @ BODY_FROM_DH[side] @ arm_tcp

    def _all_goals(self, goals, seed):
        return {s: goals[s] if s in goals else self.solver.forward(s, seed) for s in ARMS}

    def _descends(self, goals, frozen_arms, before, after, fk):
        old_world = fk.world(before)
        new_world = fk.world(after)
        for side, goal in goals.items():
            body_tcp = BODY_FROM_DH[side] @ fk.tcp(side, frozen_arms[ARM_SLICE[side]])
            if _cost(matrix_error(goal, new_world @ body_tcp)) >= _cost(matrix_error(goal, old_world @ body_tcp)):
                return False
        return True

    def _direction(self, coords, q, arms, goals, fk=None):
        fk = _CycleFK(self) if fk is None else fk
        desired = np.zeros(3)
        errors = [arms['errors_mm_deg'][side] for side in goals]
        if all(mm <= POSITION_MM and deg <= ORIENTATION_DEG for mm, deg in errors):
            return desired
        gain = min(1., max(max(mm/W_MM, deg/W_DEG) for mm, deg in errors))
        best = arms['q'].copy()
        for side in ARMS.keys()-goals.keys():
            best[ARM_SLICE[side]] = q[ARM_SLICE[side]]
        result = self.solver.solve_body(self._all_goals(goals, best), best,
                                        max_iterations=N_BODY, arms_first_iterations=0, positions_only=True)
        direction = self.path.desired(coords, result['q'], best, gain)
        if not np.any(direction):
            return desired
        probe = np.clip(coords + direction*(PROBE_STEP_DEG/SPEED_DEG_S),
                        self.path.coord_min, self.path.coord_max)
        return direction if self._descends(goals, best, q, self.path.joints(probe, best), fk) else desired

    def _verified(self, kin, side, q, reported, world_from_dh, fk):
        target = check_arm_limits(q.tolist())
        tcp = fk.tcp(side, q)
        # The manufacturer's forward() includes its own independent wrist
        # check. Its TCP composition can use the tools validated at entry.
        vendor_tcp = kin.forward_tcp
        if getattr(vendor_tcp, '__func__', None) is _VENDOR_FORWARD_TCP:
            wrist, _ = kin.forward(target)
            tool = self._validated_tools(kin.tools)['transforms'][side]
            vendor = multiply(wrist_to_flange(wrist), tool)
        else:
            vendor, _ = vendor_tcp(target, side)
        _assert_fk(tcp, rigid_transform(vendor, 1e-6), side)
        world = world_from_dh @ tcp
        _assert_fk(world, rigid_transform(reported, 1e-6), side)
        return target, tcp, world

    def _entry(self, kin, side, goals, result, q, world_from_dh, hw, guard, fk):
        arm = ARM_SLICE[side]
        ref = q[arm].tolist()
        measured, velocity = hw['position'][arm], hw['velocity'][arm]
        goal = goals[side]
        origin = world_from_dh @ fk.tcp(side, q[arm])
        before = matrix_error(goal, origin)
        scale = guard.approach_scale(ref, result['q'][arm].tolist())
        for fraction in (1., .5, .25, .125, .0625, .03125, 0.):
            amount = scale*fraction
            if amount == 0.:
                candidate, reported = q[arm], self.solver.forward(side, q)
                status = 'wrist_hold'
            else:
                trial = result
                if amount != 1.:
                    limited = {**goals, side: interpolate_tcp(origin, goal, amount)}
                    trial = self._solve_arms(self._all_goals(limited, q), q, self._dt)
                candidate, reported = trial['q'][arm], trial['tcp'][side]
                status = trial['status']
            target, tcp, world = self._verified(kin, side, candidate, reported, world_from_dh, fk)
            try:
                guard.arm_segment(ref, target, velocity)
                guard.arm_segment(measured, target, velocity)
            except WristBlocked:
                continue
            err = matrix_error(goal, world)
            if _worse_than(err, before):
                raise ValueError(f'{side} 手臂解增大了固定躯干下的完整目标误差')
            return dict(reference=ref, target=target, tcp=tcp.ravel().tolist(), wrist_scale=amount,
                        teleop_tcp=(WORLD_FROM_V[side].T @ world).ravel().tolist(),
                        native_ik=dict(backend='fairino_qp', status=status,
                                       success=err[0] <= POSITION_MM and err[1] <= ORIENTATION_DEG,
                                       position_error_mm=err[0],
                                       orientation_error_deg=err[1], requested_tcp=goal.ravel().tolist()))
        raise WristBlocked(f'{side} 腕部路径不通过，请松开 Grip 后向内收腕')

    def request(self, kin, hardware, goals, sequence, stamp, references=None, guard=None):
        try:
            return self._request(kin, hardware, goals, sequence, stamp, references, guard)
        except ValueError:
            self.reset()
            raise

    def _request(self, kin, hardware, goals, sequence, stamp, references, guard):
        if not isinstance(goals, dict) or not set(goals) <= set(ARMS):
            raise ValueError('Fairino QP 目标须为 left/right TCP')
        if not goals:
            self.reset()
            return None
        if type(sequence) is not int or sequence < 1 or type(stamp) not in (int, float) or not math.isfinite(stamp):
            raise ValueError('Fairino QP 请求编号或时间无效')
        if not isinstance(hardware, dict):
            raise ValueError('Fairino QP 需要当前二十轴反馈')
        if self._validated_tools(kin.tools) != self.tools:
            raise ValueError('Fairino QP TCP 配置不一致')
        guard = WristGuard() if guard is None else guard
        if not self.demo_only and not guard.enabled:
            raise ValueError('Fairino QP 实机腕部联合限位保护必须保持开启')
        hw = {key: finite_vector(hardware.get(key), 20, key)
              for key in ('position', 'command_position', 'velocity')}
        q = np.array(hw['command_position'], dtype=float)
        if self.path is None:
            self.start_path(hw)
        now = self.clock()
        elapsed = 0. if self.last_time is None else now-self.last_time
        if not math.isfinite(now) or elapsed < 0.:
            raise ValueError('本地单调时钟无效')
        restart = self.last_time is None or elapsed > RESTART_GAP
        dt = 0. if restart else min(MAX_DT, elapsed)
        if restart:
            coords = self.path.coordinates(q)
            velocity = np.zeros(3)
        else:
            q[AXES] = self.last_q[AXES]
            coords, velocity = self.coords, self.velocity
        references = {} if references is None else references
        if not isinstance(references, dict) or not set(references) <= set(ARMS):
            raise ValueError('Fairino QP 参考关节无效')
        for side, arm in ARM_SLICE.items():
            check_arm_limits(hw['position'][arm])
            check_arm_limits(hw['command_position'][arm])
            if side in goals and side in references:
                q[arm] = check_arm_limits(references[side])
        # The adapter validates these world poses at the stricter 1e-8
        # tolerance. A fixed proper rotation preserves rigidity and the bottom row.
        goals = {s: WORLD_FROM_V[s] @ np.asarray(value, dtype=float) for s, value in goals.items()}
        self._dt = dt
        arms = self._solve_arms(self._all_goals(goals, q), q, dt)
        fk = _CycleFK(self)
        desired = self._direction(coords, q, arms, goals, fk) if dt else np.zeros(3)
        next_state = self.path.advance(coords, velocity, desired, dt, q)
        if np.any(desired):
            # Boundary braking and per-axis acceleration limits can rotate the
            # filtered step. Certify its extra motion relative to pure braking.
            braking = self.path.advance(coords, velocity, np.zeros(3), dt, q)
            if not self._descends(goals, arms['q'], braking[2], next_state[2], fk):
                next_state = braking
        coords, velocity, new_q = next_state
        if not np.array_equal(q[AXES], new_q[AXES]):
            arms = self._solve_arms(self._all_goals(goals, new_q), new_q, dt)
        world = fk.world(new_q)
        _assert_fk(world, rigid_transform(self.solver.body_frame(new_q), 1e-6), '躯干')
        entries = {s: self._entry(kin, s, goals, arms, new_q, world @ BODY_FROM_DH[s], hw, guard, fk)
                   for s in goals}
        final_q = new_q.copy()
        for s, entry in entries.items():
            final_q[ARM_SLICE[s]] = entry['target']
        if np.array_equal(final_q[:14], arms['q'][:14]):
            clear = arms['collision_free']
        else:
            # Recheck the actual combination after a wrist retreat or discarded
            # free-arm solution. A collision-recovery motion cannot certify its seed.
            hold = self.solver.solve_arms(self._all_goals({}, final_q), final_q, max_iterations=1)
            clear = hold['collision_free'] and np.array_equal(hold['q'], final_q)
        if not clear:
            raise ValueError('Fairino QP 最终姿态未通过机身避障检查')
        packet = self.checked_packet(hw, entries, new_q, sequence, stamp)
        self.last_time, self.coords, self.velocity, self.last_q = now, coords, velocity, new_q
        return packet

    def checked_packet(self, hw, entries, q, sequence, stamp, *, source=None):
        packet = dict(kind=KIND, model=MODEL_ID, id=sequence, created=stamp,
                      tool_id=self.tools['id'], arms=entries, ik_backend='fairino_qp',
                      torso=dict(target=q[AXES].tolist(), path_offset_mm=self.path.offset))
        if source is not None:
            packet['source'] = source
        if self.demo_only:
            packet['demo_only'] = True
        validate(hw, packet, stamp, self.tools, self.body)
        return packet
