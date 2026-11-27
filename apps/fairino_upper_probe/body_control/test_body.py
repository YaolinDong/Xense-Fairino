import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from body_core import BODY, load_config, plan, sample_interval
import body_robot


class BodyTests(unittest.TestCase):
    def setUp(self):
        self.m = load_config()
        self.hold = [math.radians(2*i-20) for i in range(20)]

    def s(self, h=0, f=0, r=0, y=0):
        return dict(height=h, forward=f, roll=r, yaw=y)

    def q(self, **kw):
        return self.m.pose(self.s(**kw), self.hold)

    def samples(self, result, count=7):
        for left, right in zip(result['points'], result['points'][1:]):
            for i in range(count+1):
                t = left['time']+(right['time']-left['time'])*i/count
                yield sample_interval(left, right, t)

    def test_low_and_reference_heights(self):
        self.assertEqual(len(self.m.references()), 3)
        q = self.q()
        for a, b in zip(map(math.degrees, q[17:20]), [self.m.neutral-75, 130, -55]):
            self.assertAlmostEqual(a, b, places=9)
        self.assertAlmostEqual(math.degrees(self.q(h=100)[18]), 15)
        self.assertAlmostEqual(self.m.state(self.q(h=50))['height_mm'], self.m.travel/2)

    def test_constant_attitude_and_horizontal_position(self):
        for h in range(101):
            q = self.q(h=h)
            s = self.m.state(q)
            self.assertAlmostEqual(s['forward'], 0, places=10)
            self.assertAlmostEqual(s['height'], h, places=9)
            self.assertAlmostEqual(s['forward_offset_mm'], 0, places=9)

    def test_user_limits_and_side_tilt_gate(self):
        for s in [self.s(h=100,f=5,r=15,y=90), self.s(h=90,f=5,r=-15,y=-90), self.s(f=45)]:
            self.m.validate(s)
        for s in [self.s(h=-.01), self.s(h=100.01), self.s(f=-.01), self.s(f=45.01),
                  self.s(y=90.01), self.s(y=-90.01), self.s(h=100,r=15.01),
                  self.s(h=89.99,r=1), self.s(h=100,f=5.01,r=1), self.s(h=math.nan)]:
            with self.assertRaises(ValueError):
                self.m.validate(s)

    def test_derivatives_against_central_difference(self):
        for h in (0, 20, 50, 90, 100):
            s = self.s(h=h,f=12)
            q, qz, qzz = self.m.curve(s, self.hold)
            dz = .001
            a = self.m.curve({**s, 'height': h-dz/self.m.travel*100}, self.hold)[0]
            b = self.m.curve({**s, 'height': h+dz/self.m.travel*100}, self.hold)[0]
            for i in (17,18,19):
                self.assertAlmostEqual(qz[i], (b[i]-a[i])/(2*dz), delta=1e-8)
                self.assertAlmostEqual(qzz[i], (b[i]-2*q[i]+a[i])/dz**2, delta=1e-8)

    def test_quintic_full_height_path_and_speed(self):
        for start, end in ((0,100),(100,0)):
            result = plan(self.m, self.q(h=start), self.q(h=start), self.s(h=end), 5)
            peak = 0
            for q, v in self.samples(result):
                s = self.m.state(q)
                self.assertAlmostEqual(s['forward'], 0, delta=1e-8)
                self.assertAlmostEqual(s['forward_offset_mm'], 0, delta=.001)
                self.assertGreaterEqual(s['height'], -1e-6)
                self.assertLessEqual(s['height'], 100+1e-6)
                peak = max(peak, *(abs(math.degrees(x)) for x in v))
                self.assertLess(max(abs(a-b) for a,b in zip(q[:15],result['start'][:15])),1e-14)
            self.assertLessEqual(peak, 5+1e-8)
            self.assertGreater(peak, 4.8)
            for key in ('velocities','accelerations'):
                self.assertTrue(all(abs(v)<1e-10 for v in result['points'][-1][key]))

    def test_center_roll_before_lowering_or_pitching(self):
        for target in [self.s(h=0,f=0), self.s(h=100,f=45)]:
            q = self.q(h=100,f=3,r=15,y=50)
            result = plan(self.m,q,q,target,5)
            self.assertEqual(result['stages'][0], '先回正侧倾')
            for qq,v in self.samples(result):
                s = self.m.state(qq)
                if abs(s['roll'])>1e-5:
                    self.assertTrue(self.m.roll_allowed(s), s)
                self.assertLessEqual(max(abs(math.degrees(x)) for x in v), 5+1e-8)

    def test_side_tilt_only_after_ascending(self):
        q = self.q()
        result = plan(self.m,q,q,self.s(h=100,f=5,r=-15,y=-90),5)
        self.assertEqual(result['stages'][-1], '到达高位后侧倾')
        for qq,v in self.samples(result):
            s = self.m.state(qq)
            if abs(s['roll'])>1e-5:
                self.assertTrue(self.m.roll_allowed(s), s)

    def test_existing_off_path_pose_gets_explicit_alignment(self):
        q=self.q(h=50)
        q[19]+=math.radians(1)
        q[17]-=math.radians(1)
        result=plan(self.m,q,q,self.s(h=100),5)
        self.assertEqual(result['stages'][0],'对齐升降路径')
        self.assertEqual(result['points'][0]['positions'], q)
        self.assertAlmostEqual(self.m.state(result['final'])['forward_offset_mm'],0,places=8)

    def test_speed_is_adjustable_without_previous_cap(self):
        q=self.q()
        slow=plan(self.m,q,q,self.s(h=100),5)
        fast=plan(self.m,q,q,self.s(h=100),20)
        self.assertAlmostEqual((slow['duration']-.1)/(fast['duration']-.1),4,delta=.002)
        for bad in (0,-1,math.nan,math.inf,True):
            with self.assertRaises(ValueError):plan(self.m,q,q,self.s(h=100),bad)

    def test_arms_and_head_keep_command_targets(self):
        actual=self.q(h=0)
        actual[3]+=.0001
        result=plan(self.m,actual,self.hold,self.s(h=100),5)
        for p in result['points']:
            self.assertEqual(p['positions'][:15],self.hold[:15])

    def test_goal_has_all_twenty_axes_and_no_old_error_trips(self):
        class Goal:
            def __init__(self):
                self.trajectory=NS(joint_names=[],points=[])
                self.path_tolerance=[];self.goal_tolerance=[]
        class Duration:
            def __init__(self,seconds):self.seconds=seconds
            def to_msg(self):return self.seconds
        q=self.q()
        result=plan(self.m,q,q,self.s(h=100),5)
        with patch.multiple(body_robot.rp,FollowJointTrajectory=NS(Goal=Goal),
                            JointTrajectoryPoint=NS,JointTolerance=NS,Duration=Duration,create=True):
            goal=body_robot.BodyProbe.goal_for(result)
        self.assertEqual(len(goal.trajectory.joint_names),20)
        self.assertEqual(goal.trajectory.points[-1].positions,result['final'])
        self.assertTrue(all(t.position==-1 for t in goal.path_tolerance+goal.goal_tolerance))
        self.assertFalse(body_robot.BodyProbe.feedback_speed_check)
        self.assertFalse(body_robot.BodyProbe.path_tracking_check)
        self.assertTrue(body_robot.BodyProbe.allow_unfocused)


class IndependentURDFTests(unittest.TestCase):
    def test_torso_stays_upright_relative_to_reference_in_full_urdf(self):
        import xml.etree.ElementTree as E
        path=Path(__file__).resolve().parents[3]/'scripts/pico4/fairino/models/fairino_wheel_legged_robot_description.urdf'
        joints={j.get('name'):j for j in E.parse(path).getroot().findall('joint')}
        def eye(n):return [[float(i==j) for j in range(n)] for i in range(n)]
        def mul(a,b):return [[sum(x*y for x,y in zip(row,col)) for col in zip(*b)] for row in a]
        def rot(axis,theta):
            norm=math.sqrt(sum(x*x for x in axis));x,y,z=[v/norm for v in axis]
            K=[[0,-z,y],[z,0,-x],[-y,x,0]];KK=mul(K,K)
            return [[float(i==j)+math.sin(theta)*K[i][j]+(1-math.cos(theta))*KK[i][j]
                     for j in range(3)] for i in range(3)]
        def fk(q):
            T=eye(4)
            for name,theta in [('j20',q[19]),('j19',q[18]),('j18',q[17]-math.radians(m.neutral)),('j17',q[16]),('j16',q[15])]:
                j=joints[name];o=j.find('origin')
                xyz=[float(v) for v in o.get('xyz').split()]
                r,p,y=[float(v) for v in o.get('rpy').split()]
                A=eye(4);R=mul(mul(rot([0,0,1],y),rot([0,1,0],p)),rot([1,0,0],r))
                B=eye(4);Q=rot([float(v) for v in j.find('axis').get('xyz').split()],theta)
                for i in range(3):
                    A[i][:3]=R[i];A[i][3]=xyz[i];B[i][:3]=Q[i]
                T=mul(mul(T,A),B)
            return T
        m=load_config();base=None
        for height in range(0,101,5):
            q=m.pose(dict(height=height,forward=0,roll=0,yaw=0),[0.]*20)
            T=fk(q)
            if base is None:base=T
            self.assertLess(max(abs(T[i][j]-base[i][j]) for i in range(3) for j in range(3)),1e-5)
            self.assertLess(math.sqrt(sum((T[i][3]-base[i][3])**2 for i in range(2))),1e-5)
            self.assertAlmostEqual((T[2][3]-base[2][3])*1000,m.travel*height/100,delta=.01)


if __name__=='__main__':unittest.main()
