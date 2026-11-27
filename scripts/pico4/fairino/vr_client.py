#!/usr/bin/env python3
"""PICO -> selected constrained IK -> one guarded FAIRINO ROS trajectory stream."""
import argparse
import base64
import contextlib
import getpass
import json
import math
import queue
import select
import shlex
import signal
import sys
import termios
import time
import tty
import zlib
from pathlib import Path

from paths import ROOT, TCP, SHARED
sys.path.insert(0, str(ROOT))
from apps.gripper_control.gripper_core import TELEOP_SPEED, INSTALLATION_SERIALS, number
from apps.fairino_keyboard.control_core import (DEFAULT_LINEAR, DEFAULT_ANGULAR,
                                                MIN_SPEED, MAX_LINEAR, MAX_ANGULAR, check_speeds)
from upper_keyboard import Connection
from tcp_keyboard import verify_both
from tcp_core import ARMS, MODEL_ID
from vendor_kinematics import VendorKinematics
from tool_frames import DEFAULT_CONFIG, load_geometry, tool_config, packet_tool_id
from vr_core import validate, KIND, LIFT_MODE, ROBOT_IK_BACKENDS, require_robot_backend
from vr_lift import LiftInput
from vr_lift_presets import HIGH_PERCENT, HOLD_SECONDS, LOW_PERCENT, SPEED_DEG_S
from vr_input import Clutch, Stream, MAPPING_ID, POSE_FRAMES, controller_sides, rotation_mapping
from vr_controls import JoystickControls, sensitivity_step
from vr_heading import heading_alignment
from vr_home import DEFAULT_HOME, load_home, HomeMotion, HOME_PRESETS, resolve_home
from vr_timing import Timing, Deadline, RateMeter, add_timing_arguments
from mounts import tcp_in_chest
from vr_plan import request
from vr_cartesian import CartesianInput
from vr_exit import ExitLog, connection_state, exit_notice
from control_modes import CHASSIS_DIRECTIONS

HERE = Path(__file__).resolve().parent
GUARDED_NAMES = {'native': '联合限位 IK', 'tron2': 'TRON2 QP IK', 'fairino_qp': 'Fairino QP IK'}
GUARDED_IK_BACKENDS = tuple(GUARDED_NAMES)
IK_LABELS = {'native': '联合限位 IK · 完整 TCP 目标直接求解', 'vendor': '原厂 IK',
             'tron2': 'TRON2 QP IK · ART7 模型 · 外层腕部路径保护',
             'fairino_qp': 'Fairino QP IK · 双臂与躯干 · 外层腕部路径保护'}


def remote_command(args):
    require_robot_backend(getattr(args, 'ik_backend', 'vendor'))
    bundle = {name: (SHARED/(name+".py")).read_text() for name in ("wrist_guard", "probe_core", "robot_probe")}
    bundle.update({name: (TCP/(name+".py")).read_text() for name in ("tcp_core", "tool_frames", "tcp_frames", "tcp_jog", "tcp_robot")})
    bundle.update({name: (HERE/(name+".py")).read_text() for name in ("vr_timing", "vr_core", "vr_link_check", "vr_robot")})
    bundle['body_core'] = (SHARED/'body_control/body_core.py').read_text()
    bundle['vr_lift'] = (HERE/'vr_lift.py').read_text()
    bundle['vr_lift_presets'] = (HERE/'vr_lift_presets.py').read_text()
    encoded = base64.b64encode(zlib.compress(json.dumps(bundle).encode())).decode()
    argv = ["vr_robot.py", *Timing.from_args(args).argv()]
    if getattr(args, 'no_held_speed_check', False):
        argv.append('--no-held-speed-check')
    argv.extend(['--tools-json', json.dumps(getattr(args, 'tools', tool_config()), allow_nan=False)])
    if getattr(args, "link_check", False):
        argv.extend(["--link-check", "--duration", str(args.duration)])
        if getattr(args, "observe_ros", False):
            argv.append("--observe-ros")
    elif not args.execute:
        argv.append("--prepare-check" if args.prepare_check else "--check")
    if getattr(args, 'ik_backend', None) == 'fairino_qp':
        if getattr(args, 'body_lift', False):
            raise ValueError('Fairino QP 与 --body-lift 互斥')
        planner = getattr(args, 'fairino_ik', None)
        config = planner.body.config if planner is not None else json.loads((SHARED/'body_control/body_config.json').read_text())
        argv.extend(['--ik-backend', 'fairino_qp', '--body-config-json', json.dumps(config)])
    elif getattr(args, 'body_lift', False):
        config = json.loads((SHARED/'body_control/body_config.json').read_text())
        argv.extend(['--body-lift', '--body-config-json', json.dumps(config)])
    loader = ("import base64,zlib,json,sys,types;"
        f"bundle=json.loads(zlib.decompress(base64.b64decode({encoded!r})));\n"
        "for name in ('wrist_guard','probe_core','robot_probe','tcp_core','tool_frames','tcp_frames','tcp_jog','tcp_robot','body_core','vr_lift','vr_lift_presets','vr_timing','vr_core','vr_link_check'):\n"
        " m=types.ModuleType(name);sys.modules[name]=m;exec(compile(bundle[name],name+'.py','exec'),m.__dict__)\n"
        f"sys.argv={argv!r}\n"
        "exec(compile(bundle['vr_robot'],'vr_robot.py','exec'),{'__name__':'__main__'})")
    setup = "\n".join(["set -e", "source /opt/ros/humble/setup.bash",
        "source /opt/fairino_art_plugin/plugin/install/setup.bash",
        "export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
        "export FASTRTPS_DEFAULT_PROFILES_FILE=/home/fairino/fastrtps_udp.xml",
        "echo UPPERPROBE_CONNECTED", "exec python3 -u -c "+shlex.quote(loader)])
    return "bash -c "+shlex.quote(setup)


@contextlib.contextmanager
def keyboard():
    old = None
    if sys.stdin.isatty():
        old = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin)
    try:
        yield
    finally:
        if old is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


def key():
    if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return ""


def wait_for_input(stream, duration=30., require_triggers=False, require_lift=False,
                   require_chassis=False):
    deadline = time.monotonic()+duration
    previous = None
    while time.monotonic() < deadline:
        try:
            frame = stream.read()
            triggers_ready = not require_triggers or all(frame[s].get('trigger_available')
                                                       and frame[s]['trigger'] <= .02 for s in ARMS)
            lift_ready = not require_lift or (frame['right'].get('joystick_available')
                         and abs(frame['right']['axis_y']) <= .2 and not frame['right']['axis_click'])
            chassis_ready = not require_chassis or (
                all(frame[s].get('joystick_available') and not frame[s]['axis_click'] for s in ARMS)
                and math.hypot(frame['left']['axis_x'], frame['left']['axis_y']) <= .2
                and abs(frame['right']['axis_x']) <= .2)
            if all(frame[s]["grip"] <= .3 for s in ARMS) and triggers_ready and lift_ready and chassis_ready:
                return frame
            message = "启动前请松开左右 Grip" + (' 和 Trigger，确认扳机数据可用。' if require_triggers else '。')
            if require_lift:
                message += '右摇杆需回中并松开按压。'
            if require_chassis:
                message += '左摇杆及右摇杆横轴需回中，两侧摇杆松开按压。'
        except ValueError as exc:
            message = str(exc)
        if message != previous:
            print(message, flush=True)
            previous = message
        time.sleep(.02)
    raise RuntimeError("未确认新鲜双手柄数据及 Grip 已释放；没有启动机器人控制。")


def heading_controls_released(frame, chassis_enabled):
    """Require a new neutral input after a calibration attempt, even a rejection."""
    left, right = frame['left'], frame['right']
    if left.get('axis_click') or right.get('axis_click'):
        return False
    if not right.get('joystick_available') or abs(right.get('axis_y', 1.)) > .2:
        return False
    if chassis_enabled:
        return (left.get('joystick_available') is True
                and math.hypot(left.get('axis_x', 1.), left.get('axis_y', 1.)) <= .2
                and abs(right.get('axis_x', 1.)) <= .2)
    return True


def wait_for_body_start(connection, planner):
    """Keep the receiver unarmed while its controller acquires the measured hold."""
    deadline = time.monotonic()+60.
    while True:
        status = connection.latest
        if connection.done.is_set() or status.get('error') or time.monotonic() > deadline:
            raise RuntimeError(status.get('error') or '未收到使能前的身体保持姿态')
        if status.get('body_start_ready') and time.monotonic()-connection.last_status <= .25:
            planner.start_path(status['hardware'])
            return
        connection.send()
        time.sleep(.01)


def operate(args, kin, stream, connection):
    ik_backend = require_robot_backend(getattr(args, 'ik_backend', 'vendor'))
    timing = Timing.from_args(args)
    cadence = Deadline(timing.command_hz, time.monotonic())
    targets_rate, feedback_rate = RateMeter(), RateMeter()
    control_mode = getattr(args, "mode", "first_person")
    rotation_mode = rotation_mapping(control_mode, getattr(args, "rotation_mode", "auto"))
    hands = controller_sides(control_mode, getattr(args, "hand_mapping", "auto"))
    freeze_buttons = getattr(args, "freeze_buttons", False)
    whole_body = ik_backend == 'fairino_qp'
    # Existing direct callers may still supply the old vendor-only namespace.
    # The CLI explicitly selects native by default. Both C++ backends load
    # before opening PICO or the robot connection, and share continuous input.
    if ik_backend in GUARDED_IK_BACKENDS and args.execute:
        from vr_native_input import NativeClutch
        planner = getattr(args, {'native': 'native_ik', 'tron2': 'tron_ik', 'fairino_qp': 'fairino_ik'}[ik_backend])
        clutch_type, plan_request = NativeClutch, planner.request
    else:
        clutch_type, plan_request = Clutch, request
    clutches = {side: clutch_type(side, mode=control_mode, rotation_mode=rotation_mode) for side in ARMS}
    if whole_body and args.execute:
        from vr_fairino_home import FairinoHomeMotion
        from vr_fairino_ik import WORLD_FROM_V
        from vr_fairino_input import FairinoInput
        cartesian = FairinoInput(clutches, planner, timing.interpolation_delay_ms)
    else:
        cartesian = CartesianInput(clutches, timing.interpolation_delay_ms)
    joysticks = JoystickControls(right_a_home=whole_body and not freeze_buttons)
    heading_yaw_deg = None
    heading_message = '正前方未校准；沿用 PICO 追踪方向'
    heading_release_required = False
    home = load_home(resolve_home(getattr(args, 'home_preset', 'default'),
                                 getattr(args, 'home_file', None))) if args.execute else None
    if home:
        for side, q in home["target"].items():
            kin.verify_tcp(q, side)
    home_pending = args.execute and not getattr(args, "no_auto_home", False)
    home_motion = None
    sensitivity = args.sensitivity
    paused = True
    proof = None
    sequence = 0
    outgoing = None
    last_sides = ()
    last_message = None
    last_display = 0.
    last_gripper_display = 0.
    last_input_stop = None
    stream_bad_since = None
    checked = False
    startup = time.monotonic()
    grippers = getattr(args, 'gripper_control', None)
    chassis = getattr(args, 'chassis_control', None)
    lift_input = LiftInput() if getattr(args, 'body_lift', False) else None
    lift_value = 0.
    lift_epoch = 0
    last_lift_session_active = False
    last_lift_value, last_lift_log = 0., 0.
    action_path = connection.log_path.with_suffix(".actions.jsonl")
    print("日志：", connection.log_path, flush=True)
    tools = getattr(kin, 'tools', tool_config())
    print('TCP：'+('法兰中心' if tools['mode'] == 'flange' else '夹爪固定指尖中心')+
          ' · '+tools['id'], flush=True)
    print('IK：'+IK_LABELS[ik_backend], flush=True)
    held_speed_check = not getattr(args, 'no_held_speed_check', False)
    print('保持关节速度检测（1°/s）：'+('已开启' if held_speed_check else '已关闭')+
          '；保持位置偏差、停止确认及其他保护仍保留。', flush=True)
    if args.execute:
        print('所选起始位姿：'+home.get('description', home['name']), flush=True)
        print("即将使能 J1–J20；" + ("先自动回双臂 Home，完成后握 Grip 遥操。" if home_pending else "保持当前姿态，握 Grip 遥操。"), flush=True)
        print("左手 X 双击降低灵敏度、Y 双击提高；右摇杆按压：回 Home（再按中止）；左摇杆按压：校准双臂正前方。", flush=True)
        print("方向校准：头显打开 Head / Controller / Send，松开两侧 Grip、两摇杆回中，机器人停稳后面向正前方按左摇杆；空格暂停。", flush=True)
        if chassis:
            print(f'底盘将回正并使能：{CHASSIS_DIRECTIONS[control_mode]}；'
                  f'最高 {args.chassis_linear:g} m/s、{args.chassis_angular:g} rad/s。回中停止，暂停后先回中。', flush=True)
        if lift_input:
            print(f'右摇杆持续推 {HOLD_SECONDS:g} 秒：上→高位{HIGH_PERCENT:g}%，下→下位{LOW_PERCENT:.1f}%；'
                  f'关节速度上限 {SPEED_DEG_S:g}°/s。触发后回中继续到位，暂停/Home可中止；完成后先回中再触发。', flush=True)
        if grippers:
            print('夹爪遥操已显式启用：控制某臂的 Trigger 连续控制同侧夹爪；Grip 仍控制手臂跟随。', flush=True)
            print(f'夹爪速度 {grippers.speed:g} rad/s（电机轴），目标传输最多 100 Hz；'
                  '自动使用各爪允许的最大持续力矩（当前 EL05 为 1.1 Nm）。', flush=True)
        if whole_body:
            print("Home 先移动双臂，再恢复会话起始躯干姿态；握 Grip 或按空格中止。", flush=True)
            if not freeze_buttons:
                print("物理右 A 键也可触发或中止 Home。", flush=True)
        else:
            print("Home 仅移动双臂；请松开两侧 Grip，途中握任一 Grip 或按空格可中止。", flush=True)
        if freeze_buttons:
            print("按住 A/X 冻结旋转（只平移）；按住 B/Y 冻结位移（只旋转）；同时按住冻结两者。", flush=True)
            print("按键仅影响该手柄控制的手臂；松开不补走；仍须握 Grip。+/- 调灵敏度，空格暂停。", flush=True)
        else:
            print("双击 X/Y 或 +/- 调灵敏度 0.25/0.5/1；左摇杆按压校准双臂正前方，空格暂停。", flush=True)
        print("Esc/Ctrl+C 退出并全部下使能。", flush=True)
        print(GUARDED_NAMES[ik_backend]+
              ' 的腕部保护始终开启，不能关闭。' if ik_backend in GUARDED_IK_BACKENDS else
              '腕部联合限位默认开启；g 启用 / G 停用，切换暂停并保持使能，松开 Grip 后重握。', flush=True)
        labels = {"left": "左", "right": "右"}
        print(f"位移 {control_mode} / 旋转 {rotation_mode}；" + "，".join(
            f"{labels[hand]}手柄 → 机器人{labels[arm]}臂" for arm, hand in hands.items()), flush=True)
        print(f"TCP 插值 v3：位移线性 / 姿态 SLERP → IK；采样/下发 {timing.command_hz:g} Hz，缓冲 {timing.interpolation_delay_ms:g} ms。", flush=True)
        print(f"机器人单点保持，无额外关节追赶段；接收轮询 {timing.receiver_hz:g} Hz / 回传 {timing.feedback_hz:g} Hz。", flush=True)
        if whole_body:
            print("手柄位移和旋转固定在底盘坐标系；腰和折叠结构运动时映射方向保持。", flush=True)
        else:
            print(f"坐标映射 {MAPPING_ID}：双臂按厂家整机 URDF 对齐胸部，X 前 / Y 左 / Z 上。", flush=True)
            print("方向随胸部变化；腰部或折叠结构倾斜时，胸部轴不保证与地面对齐。", flush=True)
        print("保留模型关节限位、独立正解校验、急停和断流停止。", flush=True)
    with action_path.open("a", buffering=1) as log, keyboard():
        log.write(json.dumps({"local_time": time.time(), "event": "timing_config", **timing.payload()})+"\n")
        log.write(json.dumps({"local_time": time.time(), "event": "tcp_tool_config", **tools})+"\n")
        log.write(json.dumps({'local_time': time.time(), 'event': 'held_speed_check_config',
            'held_speed_check_enabled': held_speed_check, 'held_speed_limit_deg_s': 1.0})+'\n')
        log.write(json.dumps({'local_time': time.time(), 'event': 'ik_config',
            'ik_backend': ik_backend, 'command_tracking': {
                'native': 'direct_constrained_ik', 'vendor': 'vendor_wrist_scaled_ik',
                'tron2': 'tron2_qp_external_wrist_checked_ik',
                'fairino_qp': 'fairino_qp_fixed_base_arm_torso'}[ik_backend]})+'\n')
        while True:
            now = time.monotonic()
            stop_reason = None
            while True:
                try:
                    event = connection.events.get_nowait()
                except queue.Empty:
                    break
                if "stamp" in event:
                    feedback_rate.mark(time.monotonic())
                if "error" in event:
                    raise RuntimeError(event["error"])
                if "cleanup_warning" in event:
                    print(event["cleanup_warning"], flush=True)
                if "jog_rejected" in event:
                    stop_reason = event["jog_rejected"]
                if event.get("checked"):
                    checked = True
                    proof, details = verify_both(kin, event["hardware"])
                    if whole_body:
                        hw = event['hardware']
                        # Before controller activation, the previous command can be stale.
                        args.fairino_ik.start_path(hw if args.prepare_check else dict(hw, command_position=hw['position']))
                    print("机器人检查通过，未使能、未移动。", flush=True)
                    print(json.dumps(details, ensure_ascii=False), flush=True)
            if connection.done.is_set():
                if not args.execute and checked:
                    return 0
                raise RuntimeError("机器人控制通道已结束，请查看日志中的最后状态")
            status = connection.latest
            status_received = connection.last_status
            now = time.monotonic()
            hw = status.get("hardware")
            if hw is None:
                if now-startup > 12:
                    raise RuntimeError("等待机器人状态超时")
                time.sleep(.02)
                continue
            if not args.execute:
                # Controller unloading reports its final state after ROS calls;
                # no motion lease is running in either inspection mode.
                if now-startup > 60.:
                    raise RuntimeError("机器人接口检查未在 60 秒内结束")
                connection.send()
                time.sleep(.02)
                continue
            if now-status_received > .25:
                raise RuntimeError("机器人状态反馈超过 0.25 秒未更新")
            character = args.gui_input.key() if getattr(args, "gui_input", None) else key()
            if character in ("\x1b", "q", "Q"):
                args.exit_reason = 'keyboard_stop'
                return 0
            if character in ("g", "G"):
                if ik_backend in GUARDED_IK_BACKENDS and character == 'G':
                    stop_reason = GUARDED_NAMES[ik_backend]+\
                        ' 的腕部保护必须保持开启；已暂停，请松开 Grip 后重握'
                else:
                    connection.wrist.switch(character == "g")
                    stop_reason = "切换腕部模块：暂停并等待静止；确认后松开 Grip 再重握"
            wrist_state = status.get("wrist_guard", {})
            wrist_ready = connection.wrist.ready(wrist_state) and connection.wrist.released
            frame = None
            try:
                frame = stream.read()
                # Input callbacks can arrive after this iteration began.
                # Compute input ages and sampling times after the snapshot.
                now = time.monotonic()
                stream_bad_since = None
            except ValueError as exc:
                stop_reason = str(exc)
                joysticks.reset()
                if stream_bad_since is None:
                    stream_bad_since = now
                elif now-stream_bad_since > 3.:
                    raise RuntimeError("PICO 数据中断超过 3 秒，结束本次控制")
            if frame:
                controls = joysticks.update(frame, now)
                if controls["sensitivity"]:
                    sensitivity = sensitivity_step(sensitivity, controls["sensitivity"])
                    stop_reason = f"灵敏度已改为 {sensitivity:g}，请松开两侧 Grip"
                elif character in ("+", "=", "-"):
                    sensitivity = sensitivity_step(sensitivity, -1 if character == "-" else 1)
                    stop_reason = f"灵敏度已改为 {sensitivity:g}，请松开两侧 Grip"
                if character == " ":
                    stop_reason = "用户暂停，请松开两侧 Grip 后重握"
                if controls["calibrate_heading"]:
                    # Change only the arms' tracking frame. Both success and
                    # rejection enter the shared hold path for every actuator.
                    heading_release_required = True
                    try:
                        if stop_reason:
                            raise ValueError('当前正在暂停，请停止后重新按左摇杆校准')
                        if home_pending or home_motion is not None:
                            raise ValueError('已中止 Home；等待机器人停止后重新按左摇杆校准')
                        if (not status.get('ready') or not wrist_ready
                                or max(abs(v) for v in hw['velocity']) >= .005):
                            raise ValueError('请等待机器人停止并就绪后重新按左摇杆校准')
                        calibration_now = time.monotonic()
                        alignment, yaw_deg = heading_alignment(
                            frame, calibration_now, require_chassis_neutral=chassis is not None)
                        if chassis is not None:
                            chassis.require_stationary(calibration_now)
                        cartesian.reset()
                        for clutch in clutches.values():
                            clutch.set_heading(alignment)
                        heading_yaw_deg = yaw_deg
                        heading_message = f'已校准正前方（{yaw_deg:+.1f}°）；松开左摇杆按键后重新握 Grip'
                        log.write(json.dumps({'local_time': time.time(), 'event': 'heading_calibrated',
                            'yaw_deg': yaw_deg, 'tracking_to_heading': alignment.tolist(),
                            'head': frame['head'], 'mode': control_mode, 'rotation_mode': rotation_mode},
                            ensure_ascii=False, allow_nan=False)+'\n')
                    except ValueError as exc:
                        heading_message = '正前方校准未执行：'+str(exc)
                        log.write(json.dumps({'local_time': time.time(), 'event': 'heading_calibration_rejected',
                            'reason': str(exc)}, ensure_ascii=False)+'\n')
                    stop_reason = heading_message
                    last_display = 0.
                if controls.get('cancel_home'):
                    stop_reason = '用户中止 Home；松开 Grip 后可继续遥操'
                if (controls["home"] or character == "h") and not stop_reason:
                    if home_motion is not None or home_pending:
                        stop_reason = "用户中止 Home；松开 Grip 后可继续遥操"
                    else:
                        home_pending = True
                        paused, outgoing, last_sides = True, None, ()
                        cartesian.reset()
                        print("准备回 Home：松开两侧 Grip，等待双臂停止。", flush=True)
                        log.write(json.dumps({"local_time": time.time(), "event": "home_requested"})+"\n")
                samples = (frame.get('samples') or [frame]) if whole_body else [frame]
                if home_motion is not None and any(f[s]["grip"] > .3 for f in samples for s in ARMS):
                    stop_reason = "Grip 已握下，中止 Home；松开后可继续遥操"
            if proof is None and frame is not None and (not whole_body or status.get('body_start_ready')):
                if whole_body:
                    planner.start_path(hw)
                proof, _ = verify_both(kin, hw)
                if whole_body:
                    proof['body_reference'] = list(hw['command_position'][15:20])
            now = time.monotonic()
            if frame is not None and not 0. <= now-frame.get('received_monotonic', now) <= .25:
                stop_reason = 'PICO 输入超过 0.25 秒未更新；请松开 Grip 后重试'
            if stop_reason:
                home_pending, home_motion = False, None
                paused = True
                outgoing = None
                last_sides = ()
                cartesian.reset()
                joysticks.reset()
                if stop_reason != last_message:
                    print(stop_reason, flush=True)
                    log.write(json.dumps({"local_time": time.time(), "event": "input_paused", "reason": stop_reason,
                        "input": {s: {k: v for k, v in frame[s].items() if k != "pose"} for s in ARMS} if frame else None}, ensure_ascii=False)+"\n")
                    last_message = stop_reason
            elif frame and status.get("ready") and wrist_ready and (home_pending or home_motion is not None):
                try:
                    if home_pending:
                        outgoing, last_sides = None, ()
                        if (all(frame[s]["grip"] <= .3 for s in ARMS)
                                and max(abs(v) for v in hw["velocity"]) < .005):
                            if whole_body:
                                home_motion = FairinoHomeMotion(home, hw, now, connection.wrist.guard(), planner)
                                duration = home_motion.arms.duration
                            else:
                                home_motion = HomeMotion(home, hw, now, connection.wrist.guard())
                                duration = home_motion.duration
                            home_pending = False
                            print(f"双臂回 Home，预计 {duration:.1f} 秒；头腰保持。", flush=True)
                            log.write(json.dumps({"local_time": time.time(), "event": "home_started",
                                "duration": duration, "joint_degrees": home["joint_degrees"]})+"\n")
                    if home_motion is not None:
                        if home_motion.complete(hw, now):
                            home_motion = None
                            paused, outgoing, last_sides = True, None, ()
                            print("双臂与躯干 Home 已到位；握 Grip 开始遥操。" if whole_body else
                                  "双臂 Home 已到位；握 Grip 开始遥操。", flush=True)
                            log.write(json.dumps({"local_time": time.time(), "event": "home_completed"})+"\n")
                        else:
                            sequence += 1
                            outgoing = home_motion.request(kin, hw, now, sequence, status["stamp"])
                            last_sides = tuple(ARMS)
                            targets_rate.mark(now)
                            log.write(json.dumps({"local_time": time.time(), "event": "home_target", "request": outgoing})+"\n")
                except ValueError as exc:
                    home_pending, home_motion = False, None
                    if whole_body:
                        cartesian.reset()
                    paused, outgoing, last_sides = True, None, ()
                    print(f"Home 已中止：{exc}", flush=True)
                    log.write(json.dumps({"local_time": time.time(), "event": "home_aborted", "reason": str(exc)}, ensure_ascii=False)+"\n")
            elif frame and status.get("ready") and wrist_ready:
                if paused:
                    if (all(frame[s]["grip"] <= .3 for s in ARMS)
                            and (not heading_release_required or
                                 heading_controls_released(frame, chassis is not None))
                            and max(abs(x) for x in hw["velocity"][:14]) < .005):
                        paused = False
                        heading_release_required = False
                        print(f"双臂就绪 · 灵敏度 {sensitivity:g}；握住对应 Grip 移动", flush=True)
                        last_message = None
                else:
                    try:
                        goals = cartesian.goals(frame, hw, kin, hands, sensitivity, freeze_buttons, now)
                        sides = tuple(goals)
                        if not goals:
                            outgoing = None
                            reason = ("grip_released" if all(frame[s]["grip"] <= .3 for s in ARMS)
                                      else "waiting_for_stationary_or_full_grip")
                            if reason != last_input_stop:
                                log.write(json.dumps({"local_time": time.time(), "event": reason,
                                    "input": {s: {k: v for k, v in frame[s].items() if k != "pose"} for s in ARMS}})+"\n")
                            last_input_stop = reason
                        else:
                            last_input_stop = None
                            sequence += 1
                            ik_started = time.monotonic()
                            outgoing = plan_request(kin, hw, goals, sequence, status["stamp"],
                                                    cartesian.references, connection.wrist.guard())
                            ik_ms = 1000*(time.monotonic()-ik_started)
                            if whole_body:
                                validate(hw, outgoing, status["stamp"], tools, planner.body)
                            else:
                                validate(hw, outgoing, status["stamp"], tools)
                            cartesian.accepted_plan(outgoing, frame, hands)
                            targets_rate.mark(now)
                            log.write(json.dumps({"local_time": time.time(), "plan_monotonic": now, "vr_stamp": frame["stamp"],
                                "sensitivity": sensitivity, "mapping": f'fairino_qp_fixed_base_{MAPPING_ID}' if whole_body else MAPPING_ID,
                                "ik_backend": ik_backend,
                                "mode": control_mode, "rotation_mode": rotation_mode, "controller_sides": hands,
                                "heading_yaw_deg": heading_yaw_deg,
                                "input": {s: {k: v for k, v in frame[s].items() if k != "pose"} for s in ARMS},
                                "robot_status_age_ms": 1000*(now-status_received),
                                "compute_ms": 1000*(time.monotonic()-now),
                                "ik_ms": ik_ms, "cartesian_sampling": cartesian.buffer.last_metadata,
                                "pico_pc_sample_age_ms": 1000*(now-frame.get("received_monotonic", now)),
                                "freeze": {s: {"position": c.frozen_position, "rotation": c.frozen_rotation}
                                           for s, c in clutches.items()},
                                ("tcp_world_mm" if whole_body else "tcp_chest_mm"): {
                                    side: (WORLD_FROM_V[side] @ goal if whole_body else
                                           tcp_in_chest(side, goal)).flatten().tolist() for side, goal in goals.items()},
                                "request": outgoing}, allow_nan=False)+"\n")
                        last_sides = sides
                    except ValueError as exc:
                        paused = True
                        outgoing = None
                        last_sides = ()
                        cartesian.reset()
                        joysticks.reset()
                        print(f"已停止：{exc}；请松开两侧 Grip 后重试", flush=True)
            else:
                paused = True
                outgoing = None
                cartesian.reset()
            auxiliary_active = bool(frame and status.get('ready') and wrist_ready and not paused
                                    and not home_pending and home_motion is None and not stop_reason)
            if chassis is not None:
                chassis.update(frame, auxiliary_active)
            if lift_input is not None:
                right = frame.get('right', {}) if frame else {}
                lift_session_active = (auxiliary_active and right.get('joystick_available', False)
                                       and not right.get('axis_click', False))
                lift_value = lift_input.value(frame, lift_session_active)
                lift_session_active = lift_session_active and not lift_input.interrupted
                if lift_session_active and not last_lift_session_active:
                    lift_epoch += 1
                last_lift_session_active = bool(lift_session_active)
                # Neutral is a live heartbeat for an already-triggered preset.
                # Missing input, pause and Home omit the mode to cancel it; never
                # carry a previous lift field into an unrelated arm/Home target.
                if outgoing is not None:
                    outgoing = {k: v for k, v in outgoing.items()
                                if k not in ('lift', 'lift_mode', 'lift_epoch', 'lift_gesture')}
                if lift_session_active:
                    if outgoing is None:
                        sequence += 1
                        outgoing = dict(kind=KIND, model=MODEL_ID, id=sequence, created=status['stamp'],
                                        arms={}, tool_id=packet_tool_id(kin))
                        targets_rate.mark(now)
                    outgoing = {**outgoing, 'lift': lift_value, 'lift_mode': LIFT_MODE,
                                'lift_epoch': lift_epoch, 'lift_gesture': lift_input.gesture_id}
                if ((lift_value and now-last_lift_log >= .1)
                        or bool(lift_value) != bool(last_lift_value)):
                    log.write(json.dumps({'local_time': time.time(), 'event': 'body_lift_input',
                        'axis': lift_value, 'request_id': outgoing.get('id') if outgoing else None,
                        'mode': LIFT_MODE, 'lift_epoch': lift_epoch,
                        'lift_gesture': lift_input.gesture_id, 'body': status.get('body'),
                        'preset': status.get('body_lift_preset')}, allow_nan=False)+'\n')
                    last_lift_log = now
                last_lift_value = lift_value
            if grippers is not None:
                grippers.update(frame, hands, auxiliary_active)
            connection.send(request=outgoing, kinematics=proof if frame else None)
            if grippers is not None and getattr(args, 'gui_input', None) and now-last_gripper_display >= .1:
                from vr_gui_ipc import gripper_status
                gripper_status(grippers.snapshot())
                last_gripper_display = now
            if now-last_display > 1.:
                sides = "/".join(last_sides) if outgoing else "保持"
                frozen = "; ".join(f"{s}:" + "/".join(name for name, active in
                    (("位移", c.frozen_position), ("旋转", c.frozen_rotation)) if active)
                    for s, c in clutches.items() if c.frozen_position or c.frozen_rotation)
                phase = "回 Home 中" if home_motion is not None else "等待回 Home" if home_pending else status.get('phase', '')
                rate_data = {"send_hz": targets_rate.hz(now) if outgoing else 0.,
                    "accepted_hz": status.get("target_rate_hz", 0.),
                    "feedback_hz": feedback_rate.hz(now), "missed_loop_slots": cadence.skipped,
                    "hardware_observed_hz": status.get("hardware_observed_hz", 0.)}
                log.write(json.dumps({"local_time": time.time(), "event": "rates", **rate_data})+"\n")
                print(f"{phase} | 灵敏度 {sensitivity:g} | {sides}"
                      + f" | 下发 {rate_data['send_hz']:.1f} / 接收 {rate_data['accepted_hz']:.1f} / 回传 {rate_data['feedback_hz']:.1f} Hz"
                      + f" / 硬件反馈 {rate_data['hardware_observed_hz']:.1f} Hz"
                      + (f" | 冻结 {frozen}" if frozen else ""), flush=True)
                if lift_input and status.get('body'):
                    body = status['body']
                    preset = status.get('body_lift_preset') or {}
                    print(f'身体 {body["height"]:.1f}% / 相对低位 {body["height_mm"]:.1f} mm · '
                          + preset.get('message', '长推1秒：上高位 / 下到66.7%'), flush=True)
                if grippers:
                    gs = grippers.snapshot()
                    units = ' / '.join(f'{s}: {u["position"]*100:.1f}%' for s,u in gs['devices'].items())
                    print(f'夹爪：{gs["message"]}'+(' · '+units if units else ''), flush=True)
                chassis_state = chassis.snapshot() if chassis is not None else None
                if chassis_state:
                    log.write(json.dumps({'local_time': time.time(), 'event': 'chassis_state',
                                          **chassis_state}, allow_nan=False)+'\n')
                    vx, vy, wz = chassis_state['command']
                    print(f'底盘：{chassis_state["message"]} · 使能 {chassis_state["enabled"]}/8 · '
                          f'vx {vx:+.3f} / vy {vy:+.3f} m/s / wz {wz:+.3f} rad/s', flush=True)
                if getattr(args, "gui_input", None):
                    from vr_gui_ipc import status as gui_status
                    gui_status(phase=phase, receiver_phase=status.get('phase', ''),
                        sensitivity=sensitivity, sides=sides,
                        wrist_guard=wrist_state, wrist_pending=not wrist_ready,
                        held_speed_check_enabled=status.get('held_speed_check_enabled'),
                        held_speed_check_active=status.get('held_speed_check_active'),
                        wrist_limited=any(a.get('wrist_scale', 1.) < 1.
                                          for a in (outgoing or {}).get('arms', {}).values()),
                        ik_partial=any(a.get('native_ik', {}).get('success') is False
                                       for a in (outgoing or {}).get('arms', {}).values()),
                        mode=control_mode, ik_backend=ik_backend,
                        tcp_tool=tools['mode'], tool_id=tools['id'],
                        home_name=home['name'] if home else '',
                        grips={s: frame[s]["grip"] for s in ARMS} if frame else {},
                        triggers={s: frame[s].get('trigger') for s in ARMS} if frame else {},
                        grippers=grippers.snapshot() if grippers is not None else None,
                        chassis=chassis_state,
                        body=status.get('body'), lift_axis=lift_value,
                        body_lift_preset=status.get('body_lift_preset'),
                        body_lift_enabled=bool(lift_input),
                        heading_calibrated=heading_yaw_deg is not None,
                        heading_yaw_deg=heading_yaw_deg, heading_message=heading_message,
                        enabled=sum(hw.get("enabled", [])), **rate_data)
                last_display = now
            # Both target generation and heartbeat follow one absolute cadence.
            # A late iteration skips expired slots and samples the latest input.
            time.sleep(cadence.advance(time.monotonic()))


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("必须是有限正数")
    return number


def configure_ik(args):
    """Resolve backend timing before any connection or device initialization."""
    require_robot_backend(args.ik_backend)
    if args.ik_backend in GUARDED_IK_BACKENDS and args.no_wrist_guard:
        raise ValueError(GUARDED_NAMES[args.ik_backend]+
                         ' 要求腕部保护开启，不能使用 --no-wrist-guard')
    if args.ik_backend == 'fairino_qp' and getattr(args, 'body_lift', False):
        raise ValueError('Fairino QP 与 --body-lift 互斥')
    if args.command_hz is None:
        args.command_hz = {'fairino_qp': 200., 'vendor': 450.}.get(args.ik_backend, 500.)
    if args.receiver_hz is None:
        args.receiver_hz = 500.
    Timing.from_args(args)


def main(argv=None):
    parser = argparse.ArgumentParser(description="法奥双臂 PICO 相对遥操；默认只读检查")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true", help="使能全部 20 轴，默认先回双臂 Home，然后握 Grip 遥操")
    mode.add_argument("--check", action="store_true", help="只读检查（默认）")
    mode.add_argument("--prepare-check", action="store_true", help="检查临时控制器接口，不使能、不运动")
    mode.add_argument("--link-check", action="store_true", help="通信压测：不启用 ROS 控制器、不使能、不运动")
    parser.add_argument("--duration", type=positive, default=10., help="通信压测持续秒数")
    parser.add_argument("--observe-ros", action="store_true", help="仅用于 --link-check：增加只读硬件状态订阅；不发布运动目标")
    add_timing_arguments(parser, command_default=None, receiver_default=None,
        command_help='TCP 线性/SLERP 插值、IK 和下发频率：native/tron2 默认 500 Hz，vendor 450 Hz，fairino_qp 200 Hz',
        receiver_help='机器人接收轮询频率：各 IK 后端默认 500 Hz')
    parser.add_argument('--ik-backend', choices=ROBOT_IK_BACKENDS, default='native',
                        help='native 联合限位 IK（默认），vendor 原厂 IK，tron2 双臂 QP，fairino_qp 手臂与躯干 QP')
    parser.add_argument("--host", default="192.168.58.1")
    parser.add_argument("--user", default="fairino")
    parser.add_argument("--no-wrist-guard", action="store_true",
                        help="仅 vendor 后端可用；停用外部腕部保护，运行中 g 启用、G 停用")
    parser.add_argument('--no-held-speed-check', action='store_true',
                        help='本次会话关闭保持关节的 1°/s 反馈速度检测；保留位置偏差及其他保护')
    parser.add_argument("--sensitivity", type=positive, default=.25, help="位移和旋转灵敏度")
    parser.add_argument("--mode", choices=tuple(POSE_FRAMES), default="first_person",
                        help="first_person 同向；mirror 面对面镜像；mirror_same_direction 面对面且保持房间方向")
    parser.add_argument("--rotation-mode", choices=("auto", *POSE_FRAMES), default="auto",
                        help="独立旋转映射；auto 在面对面模式使用镜像旋转，保持位移模式不变")
    parser.add_argument("--hand-mapping", choices=("auto", "same", "crossed"), default="auto",
                        help="手柄到手臂：auto 在面对面模式交换左右；same 同侧；crossed 交叉")
    parser.add_argument("--freeze-buttons", action="store_true",
                        help="A/X 按住冻结旋转，B/Y 按住冻结位移；双击 X 降/Y 升灵敏度，左摇杆按压校准双臂正前方，空格暂停")
    parser.add_argument('--home-preset', choices=tuple(HOME_PRESETS), default='default',
                        help='双臂起始位姿；默认原对称屈肘，其他预设需显式选择')
    parser.add_argument("--home-file", type=Path, help="自定义双臂 Home 七轴配置文件，覆盖 --home-preset")
    parser.add_argument("--no-auto-home", action="store_true", help="本次启动保持姿态，不自动回 Home；右摇杆回 Home 仍可用")
    parser.add_argument('--grippers', action=argparse.BooleanOptionalAction, default=False,
                        help='显式使能 TacCap 夹爪遥操；控制某臂的 Trigger 连续控制对应夹爪，Grip 仍控制双臂跟随')
    parser.add_argument('--left-gripper-sn', default='',
                        help='机器人左爪固件 SN；留空使用安装绑定 '+INSTALLATION_SERIALS['left'])
    parser.add_argument('--right-gripper-sn', default='',
                        help='机器人右爪固件 SN；留空使用安装绑定 '+INSTALLATION_SERIALS['right'])
    parser.add_argument('--gripper-speed-rad-s', type=float, default=TELEOP_SPEED,
                        help='夹爪电机轴开合速度 0.01–1.0 rad/s，默认 0.75；仅 --grippers 时使用')
    for side in ('left', 'right'):
        parser.add_argument(f'--{side}-gripper-max-open-percent', type=float, default=100.,
                            help=f'{side} 夹爪最大开度 0–100%%；Trigger 从该上限线性映射至闭合')
    parser.add_argument('--tcp-tool', choices=('flange', 'taccap'), default='flange',
                        help='TCP：法兰中心或夹爪固定指尖中心；夹爪开合控制另用 --grippers')
    parser.add_argument('--tool-config', type=Path, default=DEFAULT_CONFIG,
                        help='左右夹爪安装角度和固定指尖中心配置')
    parser.add_argument('--body-lift', action=argparse.BooleanOptionalAction, default=False,
                        help=f'右摇杆持续推1秒：上到高位{HIGH_PERCENT:g}%、下到下位{LOW_PERCENT:.1f}%，30°/s；回中继续到位，暂停/Home中止')
    parser.add_argument('--chassis', action=argparse.BooleanOptionalAction, default=False,
                        help='同时回正并使能底盘：左杆平移、右杆左右旋转；方向随 --mode，回中停止')
    parser.add_argument('--chassis-host', default='192.168.58.3', help='底盘 SSH 主机，与上半身共用用户和密码')
    parser.add_argument('--chassis-linear', type=positive, default=DEFAULT_LINEAR,
                        help=f'底盘最大平移速度 m/s，范围 {MIN_SPEED:g}–{MAX_LINEAR:g}')
    parser.add_argument('--chassis-angular', type=positive, default=DEFAULT_ANGULAR,
                        help=f'底盘最大旋转速度 rad/s，范围 {MIN_SPEED:g}–{MAX_ANGULAR:g}')
    parser.add_argument('--chassis-rotation-priority', action='store_true',
                        help='右摇杆左右偏转时只旋转、停止平移；默认允许同时平移旋转')
    parser.add_argument("--gui-stdio", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.observe_ros and not args.link_check:
        parser.error("--observe-ros 只能与 --link-check 一起使用")
    try:
        configure_ik(args)
        try:
            check_speeds(args.chassis_linear, args.chassis_angular)
        except RuntimeError as exc:
            raise ValueError(str(exc)) from exc
        number(args.gripper_speed_rad_s, .01, 1., '夹爪速度 rad/s')
        for side in ('left', 'right'):
            number(getattr(args, side+'_gripper_max_open_percent'), 0., 100., side+' 夹爪最大开度 %')
        args.home_file = resolve_home(args.home_preset, args.home_file)
        if args.execute:
            load_home(args.home_file)
        args.tools = tool_config(args.tcp_tool,
            load_geometry(args.tool_config) if args.tcp_tool == 'taccap' else None,
            require_confirmed=args.execute)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    stream = connection = grippers = chassis = None
    code = 0
    logs = ROOT/".local/pico4/fairino-logs"
    exit_log = ExitLog(logs)
    exit_log.write('started', execute=args.execute, link_check=args.link_check,
                   grippers_requested=args.grippers, body_lift_requested=args.body_lift,
                   chassis_requested=args.chassis, mode=args.mode,
                   ik_backend=args.ik_backend,
                   controller_sides=controller_sides(args.mode, args.hand_mapping),
                   rotation_mode=rotation_mapping(args.mode, args.rotation_mode))
    termination = dict(reason='completed')
    signal_reason = {}
    def interrupted(signum, _frame):
        signal_reason['signal'] = signal.Signals(signum).name
        raise KeyboardInterrupt()
    previous_sigterm = signal.signal(signal.SIGTERM, interrupted)
    try:
        password = None
        if args.gui_stdio:
            from vr_gui_ipc import GuiInput
            args.gui_input = GuiInput()
            password = args.gui_input.authenticate()
        kin = VendorKinematics(tools=args.tools)
        if args.ik_backend == 'fairino_qp':
            from vr_fairino_ik import FairinoQPIK
            args.fairino_ik = FairinoQPIK(args.tools)
            args.fairino_ik.set_home(load_home(args.home_file)['target'])  # fairino_ik posture target
        if args.execute and args.ik_backend == 'native':
            # Compile/load locally before opening PICO or any robot connection.
            from vr_native_ik import NativeIK
            args.native_ik = NativeIK(tools=args.tools)
        elif args.execute and args.ik_backend == 'tron2':
            # Optional native dependencies must be ready before any device or
            # robot connection; failure must never silently choose another IK.
            from vr_tron_ik import TronIK
            args.tron_ik = TronIK(tools=args.tools)
        if args.execute:
            stream = Stream()
            stream.open()
            wait_for_input(stream, require_triggers=args.grippers, require_lift=args.body_lift,
                           require_chassis=args.chassis)
        if (args.grippers or args.chassis) and args.execute and password is None:
            password = getpass.getpass(f'SSH 密码（{args.user}@{args.host}）：')
        connection = Connection(args, worker_command=remote_command(args), log_directory=logs, password=password)
        exit_log.write('connected', connection=connection_state(connection))
        if args.execute and args.ik_backend == 'fairino_qp' and (args.grippers or args.chassis):
            # These optional sessions can enable independently of the upper-body proof.
            wait_for_body_start(connection, args.fairino_ik)
        if args.execute and args.grippers:
            from vr_grippers import GripperTeleop
            grippers = GripperTeleop(args, password)
            grippers.start()
            args.gripper_control = grippers
        if args.execute and args.chassis:
            from vr_chassis import ChassisTeleop
            chassis = ChassisTeleop(args, password)
            chassis.start()
            args.chassis_control = chassis
        password = None
        if args.link_check:
            from vr_link_check import run_client
            code = run_client(args, kin, connection)
        else:
            code = operate(args, kin, stream, connection)
        termination['reason'] = getattr(args, 'exit_reason', 'completed')
    except KeyboardInterrupt:
        gui = getattr(args, 'gui_input', None)
        termination = dict(reason=getattr(gui, 'exit_reason', None) or
                           ('signal' if signal_reason else 'keyboard_interrupt'),
                           **signal_reason)
        if gui is not None:
            termination['gui_exit_monotonic'] = getattr(gui, 'exit_monotonic', None)
        exit_notice("正在结束通信压测。" if args.link_check else "正在退出并等待机器人确认下使能。")
    except Exception as exc:
        termination = dict(reason='exception', error_type=type(exc).__name__, error=str(exc))
        exit_notice("停止："+str(exc), error=True)
        code = 1
    finally:
        # Record the first cause before any blocking or failing cleanup calls.
        exit_log.write('stopping', **termination, return_code=code,
                       connection=connection_state(connection))
        cleanup_errors = {}
        def cleanup(name, action):
            try:
                return action()
            except (Exception, KeyboardInterrupt) as exc:
                cleanup_errors[name] = dict(error_type=type(exc).__name__, error=str(exc))
                exit_notice(f'{name} 清理异常：{exc}', error=True)
                return None
        if chassis is not None:
            cleanup('chassis_stop', chassis.close)
        if grippers is not None:
            cleanup('grippers_stop', grippers.close)
        upper_confirmed = None
        if connection is not None:
            cleanup('upper_connection', connection.finish)
            if args.execute:
                upper_confirmed = bool(connection.latest.get("disabled_confirmed"))
                if upper_confirmed:
                    exit_notice("已确认上半身全部下使能。")
                else:
                    exit_notice("未收到全部下使能确认，请现场检查。", error=True)
                    code = 1
        grippers_confirmed = None
        if grippers is not None:
            grippers_confirmed = bool(cleanup('grippers_finish', grippers.finish))
            if not grippers_confirmed:
                exit_notice('夹爪退出或下使能未确认，请查看夹爪状态与 logs/gripper。', error=True)
                code = 1
            else:
                exit_notice('已确认本会话夹爪下使能。')
        chassis_confirmed = None
        chassis_disabled = None
        chassis_final = None
        if chassis is not None:
            chassis_confirmed = bool(cleanup('chassis_finish', chassis.finish))
            state = cleanup('chassis_final_state', chassis.snapshot)
            chassis_final = state if isinstance(state, dict) else None
            chassis_disabled = (bool(chassis_final.get('disabled_confirmed'))
                                if chassis_final is not None else chassis_confirmed)
            if chassis_final and chassis_final.get('error'):
                exit_notice('底盘退出记录：'+str(chassis_final['error']), error=True)
                code = 1
            if chassis_disabled and not chassis_confirmed:
                exit_notice('底盘已确认下使能，但控制器或里程计清理未确认，请查看底盘日志。', error=True)
                code = 1
            elif not chassis_confirmed:
                exit_notice('底盘退出或下使能未确认，请查看底盘状态与 apps/fairino_keyboard/logs。', error=True)
                code = 1
            else:
                exit_notice('已确认本会话底盘下使能。')
        if stream is not None:
            cleanup('pico_stream', stream.close)
        if cleanup_errors:
            code = 1
        exit_log.write('cleanup_finished', first_reason=termination['reason'], return_code=code,
                       upper_disabled_confirmed=upper_confirmed,
                       grippers_disabled_confirmed=grippers_confirmed,
                       chassis_disabled_confirmed=chassis_disabled,
                       chassis_cleanup_confirmed=chassis_confirmed, chassis=chassis_final,
                       cleanup_errors=cleanup_errors,
                       connection=connection_state(connection))
        signal.signal(signal.SIGTERM, previous_sigterm)
        exit_notice('退出日志：'+str(exit_log.path))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
