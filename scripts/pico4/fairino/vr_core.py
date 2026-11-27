"""Robot-side, standard-library-only validation of bimanual IK requests."""
import math
from body_core import QP_TORSO_LIMITS as TORSO_LIMITS
from tcp_core import MODEL_ID, ARMS, check_arm_limits, finite_vector, tcp_forward_matrix, pose_error, multiply
from tool_frames import tool_config, validate_runtime, for_arm
from tcp_jog import LEASE
from vr_lift import lift_axis

KIND = "fairino_pico_cartesian_v5"
LIFT_MODE = "preset_v1"
ROBOT_IK_BACKENDS = ('native', 'vendor', 'tron2', 'fairino_qp')
TORSO_JOINTS = (16, 18, 19, 20)
TORSO_ROLL_DEG = .2
PATH_LIMIT_MM = 5.
PATH_TOL_MM = .05
TORSO_JUMP_RAD = math.radians(1.5)
BODY_EPS = 1e-9  # Roundoff at an inclusive body-state boundary (mm, %, degrees).


def torso_in_limits(q, body):
    neutral = math.radians(body.neutral)
    return all(lo <= sign*q[j-1] - (neutral if name == 'j18' else 0.) <= hi
               for j, name, sign, lo, hi in TORSO_LIMITS)


def torso_target(torso):
    if not isinstance(torso, dict):
        raise ValueError('躯干目标格式无效')
    return finite_vector(torso.get('target'), len(TORSO_JOINTS), '躯干四轴目标')


def validate_torso(hw, torso, body):
    if body is None:
        raise ValueError('Fairino QP 需要身体标定配置')
    target = torso_target(torso)
    offset = torso.get('path_offset_mm')
    if type(offset) not in (float, int) or not math.isfinite(offset) or abs(offset) > PATH_LIMIT_MM+BODY_EPS:
        raise ValueError('躯干升降路径偏移超限')
    current = finite_vector(hw.get('command_position'), 20, '二十轴指令位置')
    if abs(math.degrees(current[16])) > TORSO_ROLL_DEG+BODY_EPS:
        raise ValueError('请用身体程序回正侧倾')
    q = current[:]
    for j, value in zip(TORSO_JOINTS, target):
        if abs(value-current[j-1]) > TORSO_JUMP_RAD:
            raise ValueError('躯干指令跳变过大')
        q[j-1] = value
    if not torso_in_limits(q, body):
        raise ValueError('躯干目标超出 Fairino QP 软件限位')
    state = body.state(q)
    if not (-1.-BODY_EPS <= state['height'] <= 101.+BODY_EPS
            and -1.-BODY_EPS <= state['forward'] <= 46.+BODY_EPS
            and abs(state['yaw']) <= 91.+BODY_EPS):
        raise ValueError('躯干目标超出身体包络')
    if any(abs(value-offset) > PATH_TOL_MM for value in
           (state['forward_offset_mm'], body.state(current)['forward_offset_mm'])):
        raise ValueError('躯干目标或指令偏离本次升降路径')


def require_robot_backend(backend):
    """Real control has an explicit allowlist, independent of Demo choices."""
    if backend not in ROBOT_IK_BACKENDS:
        raise ValueError('实机 IK 只允许 native、vendor、tron2 或 fairino_qp')
    return backend


def require_robot_request(request):
    """Reject model-only packets before any real target can be accepted."""
    if request.get('demo_only'):
        raise ValueError('模型 Demo 请求不能用于实机控制')
    # Old vendor/Home/lift packets do not carry an IK backend field.
    return require_robot_backend(request.get('ik_backend', 'vendor'))


def direct_positions(current, targets, held, torso=None):
    """One t=0 target: JTC holds this sample, with no added joint ramp."""
    end = finite_vector(current, 20, "当前二十轴指令")
    end[14:] = finite_vector(held, 20, "二十轴保持位置")[14:]
    if torso is not None:
        for j, value in zip(TORSO_JOINTS, torso_target(torso)):
            end[j-1] = value
    for side, joints in ARMS.items():
        goal = check_arm_limits(targets.get(side, [end[j-1] for j in joints]))
        for j, value in zip(joints, goal):
            end[j-1] = value
    return [(0., end)]


def matrix(value):
    values = finite_vector(value, 16, "TCP 矩阵")
    t = [values[i:i+4] for i in range(0, 16, 4)]
    if max(abs(a-b) for a, b in zip(t[3], [0., 0., 0., 1.])) > 1e-9:
        raise ValueError("无效齐次矩阵")
    r = [row[:3] for row in t[:3]]
    if any(abs(sum(r[i][k]*r[j][k] for k in range(3))-(i == j)) > 1e-6
           for i in range(3) for j in range(3)):
        raise ValueError("TCP 旋转矩阵不正交")
    det = sum(r[0][i]*(r[1][(i+1)%3]*r[2][(i+2)%3]-r[1][(i+2)%3]*r[2][(i+1)%3]) for i in range(3))
    if abs(det-1.) > 1e-6:
        raise ValueError("TCP 旋转矩阵不是正常旋转")
    return t


def validate(hw, request, now, tools=None, body=None):
    if not isinstance(request, dict) or request.get("kind") != KIND or request.get("model") != MODEL_ID:
        raise ValueError("双臂协议或运动学模型不匹配")
    tools = validate_runtime(tools if tools is not None else tool_config())
    if request.get('tool_id', 'flange') != tools['id']:
        raise ValueError('客户端与机器人端 TCP 工具配置不一致')
    if type(request.get("id")) is not int or request["id"] <= 0:
        raise ValueError("无效遥操请求序号")
    created = request.get("created")
    if type(created) not in (int, float) or not math.isfinite(created) or not 0 <= now-created <= LEASE:
        raise ValueError("PICO 目标超过 0.25 秒未更新")
    arms = request.get("arms")
    lift = lift_axis(request)
    mode = request.get('lift_mode')
    if 'lift_mode' in request and mode != LIFT_MODE:
        raise ValueError('不支持的身体升降协议，请更新 PICO 客户端与接收端')
    if mode == LIFT_MODE and 'lift' not in request:
        raise ValueError('预设升降请求必须包含标量摇杆值；中性心跳使用 lift=0')
    if mode == LIFT_MODE and (type(request.get('lift_epoch')) is not int or request['lift_epoch'] <= 0):
        raise ValueError('预设升降请求必须包含正整数 lift_epoch 会话标识')
    if mode == LIFT_MODE and 'lift_gesture' in request and (
            type(request['lift_gesture']) is not int or request['lift_gesture'] <= 0):
        raise ValueError('预设升降 lift_gesture 必须是正整数手势标识')
    if mode != LIFT_MODE and lift != 0.:
        raise ValueError('旧版身体点动协议已停用，请更新为 preset_v1 预设升降')
    if not isinstance(arms, dict) or (not arms and mode != LIFT_MODE) or not set(arms) <= set(ARMS):
        raise ValueError("无效的双臂目标")
    targets = {}
    for side, entry in arms.items():
        if not isinstance(entry, dict):
            raise ValueError("无效手臂目标")
        check_arm_limits(entry.get("reference"))
        target = check_arm_limits(entry.get("target"))
        tcp = matrix(entry.get("tcp"))
        mm, deg = pose_error(tcp, multiply(tcp_forward_matrix(target), for_arm(tools, side)))
        if mm > .02 or deg > .01:
            raise ValueError("双臂目标未通过机器人端独立 TCP 正解校验")
        targets[side] = target
    if request.get('ik_backend') == 'fairino_qp':
        if 'lift_mode' in request:
            raise ValueError('Fairino QP 与身体预设升降互斥')
        validate_torso(hw, request.get('torso'), body)
    elif 'torso' in request:
        raise ValueError('仅 Fairino QP 允许躯干目标')
    return targets


