"""Plan torso height and attitude in the measured joint convention.

No ROS, networking or actuator access. Height is a model estimate at the upper
folding pivot, relative to the user-observed low posture, not floor height.
"""
import json
import math
from pathlib import Path

BODY = (16, 17, 18, 19, 20)
# Fairino QP position bounds from b80f539, expressed in RD53 joint coordinates.
# Each torso row holds hardware ID, model joint, direction, lower and upper (rad).
# J18 is relative to the calibrated upright sum. J20 now follows the encoder sign.
QP_TORSO_LIMITS = (
    (20, 'j20', 1, -.9599, 0.),
    (19, 'j19', 1, 0., 2.2689),
    (18, 'j18', 1, -1.0472, 1.0472),
    (17, 'j17', 1, -.4887, .4887),
    (16, 'j16', 1, -3.002, 3.002),
)
QP_HEAD_LIMITS = (-3.002, 3.002)  # The dual-arm solver locks the head at model zero.
DEG = 180.0 / math.pi
EPS = 1e-7


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name}必须是有限数值")
    return float(value)


def load_config(path=None, live=False):
    path = Path(path) if path else Path(__file__).with_name('body_config.json')
    config = json.loads(path.read_text())
    model = BodyModel(config)
    if live and not config.get('upright_confirmed', False):
        raise ValueError('胸部竖直参考姿态尚未确认；当前只能运行 --demo 或 --check')
    return model


class BodyModel:
    def __init__(self, config):
        self.config = dict(config)
        self.l1 = finite(config['lower_link_mm'], '下连杆长度')
        self.l2 = finite(config['upper_link_mm'], '上连杆长度')
        self.neutral = finite(config['upright_joint_sum_deg'], '竖直角度和')
        self.low = [finite(v, '低位角度') for v in config['low_joint_deg']]
        if len(self.low) != 3 or min(self.l1, self.l2) <= 0:
            raise ValueError('连杆长度或低位姿态无效')
        self.x, self.z_low = self.fold_position(self.low[1], self.low[2])
        knee = math.radians(finite(config['high_knee_deg'], '高位中间关节角'))
        radius2 = self.l1**2 + self.l2**2 + 2*self.l1*self.l2*math.cos(knee)
        self.z_high = math.sqrt(radius2-self.x**2)
        self.travel = self.z_high-self.z_low
        if not 0 < knee < math.pi or self.travel <= 0:
            raise ValueError('高低参考姿态不能构成有效升降区间')
        for key in ('max_forward_deg', 'max_roll_deg', 'max_yaw_deg',
                    'roll_min_height_percent', 'roll_max_forward_deg', 'default_speed_deg_s'):
            if finite(config[key], key) < 0:
                raise ValueError(f'{key}不能小于零')
        if not 0 <= config['roll_min_height_percent'] <= 100:
            raise ValueError('侧倾高度门槛必须在 0～100%')

    def fold_position(self, knee_deg, lower_deg):
        a, b = math.radians(lower_deg), math.radians(lower_deg+knee_deg)
        return (self.l1*math.sin(a)+self.l2*math.sin(b),
                self.l1*math.cos(a)+self.l2*math.cos(b))

    def roll_allowed(self, state):
        return (state['height'] >= self.config['roll_min_height_percent']-EPS
                and abs(state['forward']) <= self.config['roll_max_forward_deg']+EPS)

    def validate(self, state):
        s = {k: finite(state[k], k) for k in ('height', 'forward', 'roll', 'yaw')}
        bounds = {'height': (0, 100), 'forward': (0, self.config['max_forward_deg']),
                  'roll': (-self.config['max_roll_deg'], self.config['max_roll_deg']),
                  'yaw': (-self.config['max_yaw_deg'], self.config['max_yaw_deg'])}
        labels = {'height': '高度比例', 'forward': '前倾', 'roll': '侧倾', 'yaw': '腰旋转'}
        for k, (lo, hi) in bounds.items():
            if not lo-EPS <= s[k] <= hi+EPS:
                raise ValueError(f'{labels[k]}必须在 {lo:g}～{hi:g} 范围内')
        if abs(s['roll']) > EPS and not self.roll_allowed(s):
            raise ValueError('只有高度≥90%且前倾≤5°时才允许侧倾；请先回正侧倾')
        return s

    def state(self, q):
        self.check_q(q)
        d = [math.degrees(v) for v in q]
        x, z = self.fold_position(d[18], d[19])
        return {'height': (z-self.z_low)/self.travel*100,
                'forward': sum(d[17:20])-self.neutral,
                'roll': d[16], 'yaw': d[15],
                'height_mm': z-self.z_low, 'forward_offset_mm': x-self.x}

    @staticmethod
    def check_q(q):
        if len(q) != 20 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in q):
            raise ValueError('需要完整的 20 轴有效关节角度')

    def curve(self, state, hold):
        """q and its derivatives with respect to vertical pivot position (mm)."""
        z = self.z_low + state['height']/100*self.travel
        r2 = self.x*self.x+z*z
        c = (r2-self.l1*self.l1-self.l2*self.l2)/(2*self.l1*self.l2)
        if not -1 < c < 1:
            raise ValueError('该高度没有当前折叠分支的有效解')
        k = math.acos(c)
        a = math.atan2(self.x, z)-math.atan2(self.l2*math.sin(k), self.l1+self.l2*math.cos(k))
        kd = -z/(self.l1*self.l2*math.sin(k))
        kdd = (-1/(self.l1*self.l2*math.sin(k))
               -z*z*math.cos(k)/(self.l1*self.l1*self.l2*self.l2*math.sin(k)**3))
        bd = self.l2*(self.l1*math.cos(k)+self.l2)/r2
        bdd = self.l1*self.l2*math.sin(k)*(self.l2*self.l2-self.l1*self.l1)/r2**2
        ad = -self.x/r2-bd*kd
        add = 2*self.x*z/r2**2-bdd*kd*kd-bd*kdd
        q = list(hold)
        q[15], q[16] = math.radians(state['yaw']), math.radians(state['roll'])
        q[17], q[18], q[19] = math.radians(self.neutral+state['forward'])-k-a, k, a
        dq, ddq = [0.]*20, [0.]*20
        dq[17:20] = [-kd-ad, kd, ad]
        ddq[17:20] = [-kdd-add, kdd, add]
        return q, dq, ddq

    def pose(self, state, hold):
        return self.curve(self.validate(state), hold)[0]

    def references(self):
        return {name: {'height_percent': h,
                       'height_mm': self.travel*h/100,
                       'fold_joint_deg': [math.degrees(v) for v in self.pose(
                           dict(height=h, forward=0, roll=0, yaw=0), [0.]*20)[17:20]]}
                for name, h in [('低', 0), ('中', 50), ('高', 100)]}


def ease(s):
    return 10*s**3-15*s**4+6*s**5, 30*s*s*(1-s)**2, 60*s-180*s*s+120*s**3


def bezier(q0, v0, a0, q1, v1, a1, dt):
    return [q0, q0+dt*v0/5, q0+2*dt*v0/5+dt*dt*a0/20,
            q1-2*dt*v1/5+dt*dt*a1/20, q1-dt*v1/5, q1]


def sample_interval(left, right, time):
    """Evaluate exactly the quintic Hermite segment used by ROS JTC."""
    dt = right['time']-left['time']
    u = max(0., min(1., (time-left['time'])/dt))
    q, v = [], []
    for j in range(20):
        b = bezier(left['positions'][j], left['velocities'][j], left['accelerations'][j],
                   right['positions'][j], right['velocities'][j], right['accelerations'][j], dt)
        q.append(sum(math.comb(5, i)*u**i*(1-u)**(5-i)*b[i] for i in range(6)))
        v.append(sum(math.comb(4, i)*u**i*(1-u)**(4-i)*5*(b[i+1]-b[i])/dt for i in range(5)))
    return q, v


def _segment(model, start, target, hold, speed, linear_joint=False):
    """100 polynomial pieces; retime using a rigorous Bezier velocity bound."""
    n = 100
    points = []
    if linear_joint:
        diff = [b-a for a, b in zip(start, target)]
    else:
        diff = {k: target[k]-start[k] for k in ('height', 'forward', 'roll', 'yaw')}
        dz = diff['height']/100*model.travel
    for i in range(n+1):
        s = i/n
        u, du, ddu = ease(s)
        if linear_joint:
            q = [a+d*u for a, d in zip(start, diff)]
            v, a = [d*du for d in diff], [d*ddu for d in diff]
        else:
            state = {k: start[k]+diff[k]*u for k in diff}
            q, qz, qzz = model.curve(state, hold)
            qs = [w*dz for w in qz]
            qss = [w*dz*dz for w in qzz]
            qs[15], qs[16] = math.radians(diff['yaw']), math.radians(diff['roll'])
            qs[17] += math.radians(diff['forward'])
            v = [w*du for w in qs]
            a = [x*du*du+y*ddu for x, y in zip(qss, qs)]
        points.append(dict(time=s, positions=q, velocities=v, accelerations=a))
    bound = 0.
    for left, right in zip(points, points[1:]):
        dt = right['time']-left['time']
        for j in BODY:
            k = j-1
            b = bezier(left['positions'][k], left['velocities'][k], left['accelerations'][k],
                       right['positions'][k], right['velocities'][k], right['accelerations'][k], dt)
            bound = max(bound, max(abs(5*(y-x)/dt) for x, y in zip(b, b[1:])))
    duration = max(.1, math.ceil(bound*DEG/speed*1000)/1000)
    if not math.isfinite(duration) or duration > 2147483000:
        raise ValueError('运动时间超出 ROS 时间戳的表示范围')
    for p in points:
        p['time'] *= duration
        p['velocities'] = [v/duration for v in p['velocities']]
        p['accelerations'] = [a/duration**2 for a in p['accelerations']]
    return points


def plan(model, actual, hold, target, speed):
    model.check_q(actual)
    model.check_q(hold)
    speed = finite(speed, '速度')
    if speed <= 0:
        raise ValueError('速度必须大于零，单位 °/s')
    goal = model.validate(target)
    q = list(hold)
    q[15:20] = actual[15:20]
    first = q[:]
    stages = []

    def append_joint(end, label):
        nonlocal q
        if max(abs(x-y) for x, y in zip(q, end)) > 1e-10:
            stages.append((label, _segment(model, q, end, hold, speed, True)))
        q = end[:]

    current = model.state(q)
    # Side tilt is removed BEFORE leaving its allowed height/pitch region.
    if abs(current['roll']) > EPS and (not model.roll_allowed(current) or not model.roll_allowed(goal)):
        end = q[:]; end[16] = 0.
        append_joint(end, '先回正侧倾')
        current = model.state(q)
    # Existing manually chosen poses may be off the vertical path. Align once
    # after an explicit Execute, preserving attitude when it is inside bounds.
    aligned = dict(height=max(0., min(100., current['height'])),
                   forward=max(0., min(model.config['max_forward_deg'], current['forward'])),
                   roll=current['roll'], yaw=max(-model.config['max_yaw_deg'],
                                                min(model.config['max_yaw_deg'], current['yaw'])))
    aligned['roll'] = max(-model.config['max_roll_deg'], min(model.config['max_roll_deg'], aligned['roll']))
    if not model.roll_allowed(aligned):
        aligned['roll'] = 0.
    append_joint(model.pose(aligned, hold), '对齐升降路径')
    travel_goal = dict(goal)
    if not model.roll_allowed(aligned):
        travel_goal['roll'] = 0.
    if max(abs(travel_goal[k]-aligned[k]) for k in travel_goal) > 1e-9:
        stages.append(('升降与姿态调节', _segment(model, aligned, travel_goal, hold, speed)))
        q = stages[-1][1][-1]['positions'][:]
    if abs(goal['roll']-travel_goal['roll']) > EPS:
        stages.append(('到达高位后侧倾', _segment(model, travel_goal, goal, hold, speed)))
        q = stages[-1][1][-1]['positions'][:]
    if not stages:
        stages.append(('保持当前目标', _segment(model, q, q, hold, speed, True)))
    points = [dict(time=.1, positions=first, velocities=[0.]*20, accelerations=[0.]*20)]
    elapsed = .1
    labels = []
    for label, segment in stages:
        labels.append(label)
        for p in segment[1:]:
            points.append({**p, 'time': p['time']+elapsed})
        elapsed += segment[-1]['time']
    if elapsed > 2147483000:
        raise ValueError('总运动时间超出 ROS 时间戳范围')
    return dict(target=goal, speed_deg_s=speed, points=points, duration=elapsed,
                start=first, final=q, stages=labels, references=model.references())
