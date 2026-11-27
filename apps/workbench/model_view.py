"""Desktop-only OpenGL renderer fed by the shared measured-joint model."""
import ctypes
import math
import os
import time

import gi
gi.require_version('Gtk', '3.0')
gi.require_version('Gdk', '3.0')
from gi.repository import Gdk, Gtk
import numpy as np

from .mesh_geometry import load_visuals, load_tool_visuals, tool_local, jaw_transform, look_at, perspective
from .model import RobotModel
from apps.fairino_tcp_probe.tool_frames import load_geometry, validate_geometry


VERTEX = '''#version 330 core
layout(location=0) in vec3 position;
layout(location=1) in vec3 normal;
uniform mat4 model, view_projection;
out vec3 world_position, world_normal;
void main() {
    vec4 world = model * vec4(position, 1.0);
    world_position = world.xyz;
    world_normal = mat3(model) * normal;
    gl_Position = view_projection * world;
}
'''
FRAGMENT = '''#version 330 core
in vec3 world_position, world_normal;
uniform vec3 color, eye;
uniform int floor_mode;
out vec4 fragment;
void main() {
    if (floor_mode == 1) {
        vec2 q = world_position.xy / 0.2;
        vec2 edge = abs(fract(q - 0.5) - 0.5) / max(fwidth(q), vec2(0.001));
        float grid = 1.0 - min(min(edge.x, edge.y), 1.0);
        float fade = exp(-0.18 * dot(world_position.xy, world_position.xy));
        vec3 base = vec3(0.075, 0.105, 0.15);
        base += vec3(0.07, 0.10, 0.13) * grid * fade;
        float shadow = exp(-dot(world_position.xy / vec2(0.5, 0.65), world_position.xy / vec2(0.5, 0.65)));
        fragment = vec4(base * (1.0 - 0.35 * shadow), 1.0);
        return;
    }
    vec3 n = normalize(world_normal);
    if (!gl_FrontFacing) n = -n;
    vec3 key = normalize(vec3(-2.0, -3.0, 5.0));
    vec3 fill = normalize(vec3(3.0, 1.0, 2.0));
    float light = 0.28 + 0.58 * max(dot(n, key), 0.0) + 0.22 * max(dot(n, fill), 0.0);
    vec3 view = normalize(eye - world_position);
    float shine = pow(max(dot(n, normalize(key+view)), 0.0), 48.0) * 0.18;
    float rim = pow(1.0 - max(dot(n, view), 0.0), 3.0) * 0.10;
    fragment = vec4(color * light + vec3(shine) + vec3(0.25,0.55,0.65)*rim, 1.0);
}
'''

DEMO_ARM_COLOR = (.56, .88, .65)
DEMO_COLLISION_COLOR = (1., .13, .10)
WRIST_CENTER_LINKS = {'left_waist1_Link': 'left', 'right_waist1_Link': 'right',
                     'left_waist2_Link': 'left', 'right_waist2_Link': 'right'}


class ModelView(Gtk.Box):
    def __init__(self):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        self.model = RobotModel()
        self.tool_geometry = load_geometry()
        self.visuals = load_visuals()+load_tool_visuals(self.tool_geometry)
        self.positions = {}
        self.gripper_positions = {'left': 1., 'right': 1.}
        self.gripper_fresh = set()
        self.fresh = False
        self.demo = False
        self.wrist_collision = {}
        self.program = None
        self.buffers = []
        self.render_error = None
        self.render_ms = 0.
        self.received_pose = False
        self.drag = None
        self.title = Gtk.Label(label='实体模型 · 等待实测反馈', xalign=0)
        self.title.set_line_wrap(True)
        self.pack_start(self.title, False, False, 0)
        self.area = Gtk.GLArea()
        self.area.set_required_version(3, 3)
        self.area.set_use_es(False)
        self.area.set_has_depth_buffer(True)
        self.area.set_auto_render(False)
        self.area.set_size_request(320, 320)
        self.area.set_hexpand(True)
        self.area.set_vexpand(True)
        self.area.add_events(Gdk.EventMask.BUTTON_PRESS_MASK | Gdk.EventMask.BUTTON_RELEASE_MASK |
                             Gdk.EventMask.POINTER_MOTION_MASK | Gdk.EventMask.SCROLL_MASK |
                             Gdk.EventMask.SMOOTH_SCROLL_MASK)
        self.area.connect('realize', self.realize_gl)
        self.area.connect('unrealize', self.unrealize_gl)
        self.area.connect('render', self.render)
        self.area.connect('button-press-event', self.press)
        self.area.connect('button-release-event', self.release)
        self.area.connect('motion-notify-event', self.motion)
        self.area.connect('scroll-event', self.scroll)
        self.pack_start(self.area, True, True, 0)
        bar = Gtk.Box(spacing=6)
        for title, callback in [('复位视角', self.reset_camera), ('正面', lambda *_: self.preset(-90)),
                                ('侧面', lambda *_: self.preset(0))]:
            button = Gtk.Button(label=title)
            button.set_can_focus(False)
            button.connect('clicked', callback)
            bar.pack_start(button, False, False, 0)
        self.tint = Gtk.CheckButton(label='分组着色')
        self.tint.set_can_focus(False)
        self.tint.connect('toggled', lambda *_: self.area.queue_render())
        bar.pack_start(self.tint, False, False, 0)
        self.pack_start(bar, False, False, 0)
        tool_bar = Gtk.Box(spacing=6)
        self.show_grippers = Gtk.CheckButton(label='显示夹爪 / 转接件')
        self.show_grippers.set_active(True)
        self.show_grippers.set_can_focus(False)
        self.show_grippers.connect('toggled', lambda *_: self.area.queue_render())
        tool_bar.pack_start(self.show_grippers, False, False, 0)
        tool_bar.pack_start(Gtk.Label(label='绿点：夹爪参考 TCP'), False, False, 0)
        self.pack_start(tool_bar, False, False, 0)
        self.tool_note = Gtk.Label(xalign=0)
        self.tool_note.set_line_wrap(True)
        self.pack_start(self.tool_note, False, False, 0)
        self.set_tool_geometry(self.tool_geometry)
        self.reset_camera()

    def set_tool_geometry(self, data):
        self.tool_geometry = validate_geometry(data)
        for visual in self.visuals:
            if visual.tool_part:
                visual.local = tool_local(data, visual.side, visual.tool_part)
        self.update_tool_note()
        self.area.queue_render()

    def update_tool_note(self):
        values = []
        for side, title in [('left', '左爪'), ('right', '右爪')]:
            if self.tool_geometry['arms'][side]['installed']:
                source = '模拟' if self.demo else '实测'
                values.append(f'{title}{source} {self.gripper_positions[side]*100:.1f}%'
                              if side in self.gripper_fresh else title+'开度未更新 · 灰色参考')
        pending = any(s['installed'] and not s['confirmed'] for s in self.tool_geometry['arms'].values())
        self.tool_note.set_text('；'.join(values)+('。\n开度来自模型演示，指节几何为近似；绿色固定 TCP 不随开合移动。'
                               if self.demo else '。\n开度来自夹爪反馈，指节几何为近似；绿色固定 TCP 不随开合移动。')
                               + (' 工具 TCP 待核对。' if pending else ''))

    def visual_color(self, visual):
        """Demo colors are stable even when input stops; red means wrist envelope only."""
        color = visual.color[:3]
        if self.demo and not visual.tool_part and visual.link.startswith(('left_', 'right_')):
            side = WRIST_CENTER_LINKS.get(visual.link)
            return DEMO_COLLISION_COLOR if self.wrist_collision.get(side, False) else DEMO_ARM_COLOR
        if self.tint.get_active() and self.fresh and not visual.tool_part:
            if visual.link.startswith('left_'):
                color = (.16, .66, .76)
            elif visual.link.startswith('right_'):
                color = (.96, .48, .19)
        if not self.fresh and not visual.tool_part:
            color = (.48, .54, .62)
        if visual.tool_part and visual.tool_part != 'tcp' and visual.side not in self.gripper_fresh:
            color = (.39, .42, .46)
        return color

    def visual_local(self, visual):
        return visual.local @ jaw_transform(visual, self.gripper_positions.get(visual.side, 1.))

    def visible(self, visual):
        return (not visual.tool_part or (self.show_grippers.get_active()
                and self.tool_geometry['arms'][visual.side]['installed']))

    def reset_camera(self, *_):
        self.azimuth, self.elevation = -65., 12.
        transforms, _ = self.model.transforms(self.positions)
        corners = []
        for visual in self.visuals:
            if not self.visible(visual):
                continue
            low, high = visual.vertices[:, :3].min(axis=0), visual.vertices[:, :3].max(axis=0)
            local = np.array([[x,y,z,1.] for x in (low[0], high[0]) for y in (low[1], high[1]) for z in (low[2], high[2])])
            corners.extend((transforms[visual.link] @ self.visual_local(visual) @ local.T).T[:, :3])
        points = np.asarray(corners)
        self.target = (points.min(axis=0)+points.max(axis=0))/2
        a, e = math.radians(self.azimuth), math.radians(self.elevation)
        direction = np.array([math.cos(e)*math.cos(a), math.cos(e)*math.sin(a), math.sin(e)])
        local = (look_at(self.target+direction, self.target)[:3, :3] @ (points-self.target).T).T
        aspect = self.area.get_allocated_width()/max(1, self.area.get_allocated_height())
        if self.area.get_allocated_width() < 10:
            aspect = .85
        tangent = math.tan(math.radians(36)/2)
        self.distance = max(.8, float(np.maximum(np.abs(local[:, 0])/(tangent*aspect)+local[:, 2],
                                                 np.abs(local[:, 1])/tangent+local[:, 2]).max())*1.16)
        self.area.queue_render()

    def preset(self, azimuth):
        self.azimuth, self.elevation = float(azimuth), 4.
        self.area.queue_render()

    def press(self, _, event):
        if event.button in (1, 2, 3):
            self.drag = (event.button, event.x, event.y)
            return True
        return False

    def release(self, *_):
        self.drag = None
        return True

    def motion(self, _, event):
        if not self.drag:
            return False
        button, x, y = self.drag
        self.drag = button, event.x, event.y
        if button == 1:
            self.azimuth -= (event.x-x)*.45
            self.elevation = float(np.clip(self.elevation+(event.y-y)*.35, -20, 80))
        else:
            angle = math.radians(self.azimuth)
            self.target += np.array([-math.sin(angle), math.cos(angle), 0.]) * (x-event.x)*self.distance*.001
            self.target[2] += (event.y-y)*self.distance*.001
        self.area.queue_render()
        return True

    def scroll(self, _, event):
        delta = -1 if event.direction == Gdk.ScrollDirection.UP else 1
        if event.direction == Gdk.ScrollDirection.SMOOTH:
            _, _, delta = event.get_scroll_deltas()
        self.distance = float(np.clip(self.distance*math.exp(delta*.1), .7, 10))
        self.area.queue_render()
        return True

    def upload(self, data):
        from OpenGL import GL
        vao, vbo = GL.glGenVertexArrays(1), GL.glGenBuffers(1)
        self.buffers.append((vao, vbo, len(data)))
        GL.glBindVertexArray(vao)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo)
        GL.glBufferData(GL.GL_ARRAY_BUFFER, data.nbytes, data, GL.GL_STATIC_DRAW)
        for index in (0, 1):
            GL.glEnableVertexAttribArray(index)
            GL.glVertexAttribPointer(index, 3, GL.GL_FLOAT, False, 24, ctypes.c_void_p(index*12))
        return self.buffers[-1]

    def fail(self, error):
        self.render_error = str(error)
        self.title.set_text('实体渲染不可用：'+self.render_error+'；请检查电脑 OpenGL / 运行 bash setup.sh console')

    def realize_gl(self, area):
        area.make_current()
        if area.get_error():
            self.fail(area.get_error())
            return
        try:
            # GTK can use EGL on X11 as well as Wayland. PyOpenGL defaults to
            # GLX, whose context lookup would then fail on vertex attributes.
            try:
                egl = ctypes.CDLL('libEGL.so.1')
                egl.eglGetCurrentContext.restype = ctypes.c_void_p
                if egl.eglGetCurrentContext():
                    os.environ.setdefault('PYOPENGL_PLATFORM', 'egl')
                else:
                    glx = ctypes.CDLL('libGLX.so.0')
                    glx.glXGetCurrentContext.restype = ctypes.c_void_p
                    if glx.glXGetCurrentContext():
                        os.environ.setdefault('PYOPENGL_PLATFORM', 'glx')
            except OSError:
                pass
            from OpenGL import GL
            from OpenGL.GL.shaders import compileProgram, compileShader
            self.program = compileProgram(compileShader(VERTEX, GL.GL_VERTEX_SHADER),
                                          compileShader(FRAGMENT, GL.GL_FRAGMENT_SHADER))
            self.uniforms = {name: GL.glGetUniformLocation(self.program, name)
                             for name in ('model', 'view_projection', 'color', 'eye', 'floor_mode')}
            for visual in self.visuals:
                self.upload(visual.vertices)
            transforms, _ = self.model.transforms({})
            floor = min((transforms[v.link] @ v.local @ np.c_[v.vertices[:, :3], np.ones(len(v.vertices))].T)[2].min()
                        for v in self.visuals if not v.tool_part) - .003
            points = [(-8,-8), (8,-8), (8,8), (-8,-8), (8,8), (-8,8)]
            self.upload(np.array([[x,y,floor,0,0,1] for x,y in points], dtype=np.float32))
            GL.glBindVertexArray(0)
        except Exception as exc:
            self.fail(exc)

    def unrealize_gl(self, area):
        area.make_current()
        if not area.get_error() and (self.buffers or self.program):
            from OpenGL import GL
            for vao, vbo, _ in self.buffers:
                GL.glDeleteBuffers(1, [vbo])
                GL.glDeleteVertexArrays(1, [vao])
            if self.program:
                GL.glDeleteProgram(self.program)
        self.buffers, self.program = [], None

    def matrix(self, name, value):
        from OpenGL import GL
        GL.glUniformMatrix4fv(self.uniforms[name], 1, GL.GL_TRUE, np.ascontiguousarray(value, dtype=np.float32))

    def render(self, area, _context):
        if not self.program or self.render_error:
            return True
        try:
            from OpenGL import GL
            start = time.perf_counter()
            width = max(1, area.get_allocated_width())
            height = max(1, area.get_allocated_height())
            scale = area.get_scale_factor()
            GL.glViewport(0, 0, width*scale, height*scale)
            GL.glClearColor(.055, .075, .11, 1.)
            GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
            GL.glEnable(GL.GL_DEPTH_TEST)
            GL.glDisable(GL.GL_CULL_FACE)
            GL.glUseProgram(self.program)
            a, e = math.radians(self.azimuth), math.radians(self.elevation)
            eye = self.target + self.distance*np.array([math.cos(e)*math.cos(a), math.cos(e)*math.sin(a), math.sin(e)])
            self.matrix('view_projection', perspective(width/height) @ look_at(eye, self.target))
            GL.glUniform3fv(self.uniforms['eye'], 1, eye.astype(np.float32))
            transforms, _ = self.model.transforms(self.positions)
            for visual, (vao, _, count) in zip(self.visuals, self.buffers):
                if not self.visible(visual):
                    continue
                self.matrix('model', transforms[visual.link] @ self.visual_local(visual))
                color = np.array(self.visual_color(visual))
                GL.glUniform3fv(self.uniforms['color'], 1, color.astype(np.float32))
                GL.glUniform1i(self.uniforms['floor_mode'], 0)
                GL.glBindVertexArray(vao)
                GL.glDrawArrays(GL.GL_TRIANGLES, 0, count)
            self.matrix('model', np.eye(4))
            GL.glUniform1i(self.uniforms['floor_mode'], 1)
            vao, _, count = self.buffers[-1]
            GL.glBindVertexArray(vao)
            GL.glDrawArrays(GL.GL_TRIANGLES, 0, count)
            GL.glBindVertexArray(0)
            GL.glUseProgram(0)
            self.render_ms = (time.perf_counter()-start)*1000
        except Exception as exc:
            self.fail(exc)
        return True

    def update(self, hardware, live_arms=True, grippers=None, *, demo=False, demo_active=False,
               wrist_collision=None):
        if self.demo != demo:
            # Neither source may inherit the other source's pose or be labelled as its feedback.
            self.positions = {}
            self.gripper_positions = {'left': 1., 'right': 1.}
            self.received_pose = False
            self.wrist_collision = {}
        self.demo = demo
        if demo and wrist_collision is not None:
            self.wrist_collision = dict(wrist_collision)
        self.gripper_fresh = set(grippers or {})
        self.gripper_positions.update(grippers or {})
        self.update_tool_note()
        self.fresh = hardware is not None and live_arms
        if self.fresh:
            self.positions = self.model.measured_positions(hardware, demo=demo)
            if not self.received_pose:
                self.received_pose = True
                self.reset_camera()
        if not self.render_error:
            if demo:
                title = ('模型 Demo · 模拟双臂 / 头部 / 躯干' if self.fresh else
                         '模型 Demo · 等待新数据，保留姿态' if demo_active else
                         '模型 Demo · 已停止，保留姿态' if self.received_pose else '模型 Demo · 等待启动')
                warning = [name for side, name in [('left', '左腕'), ('right', '右腕')]
                           if self.wrist_collision.get(side)]
                title += '\n浅绿：演示双臂；红色腕部：联合限位干涉区（近似）'
                if warning:
                    title += ' · '+ ' / '.join(warning)
                self.title.set_text(title)
            else:
                self.title.set_text('实体模型 · 实测双臂 / 头部 / 躯干' if self.fresh else
                                    '实体模型 · 静态参考' if not live_arms else '实体模型 · 无新反馈，保留最后姿态')
        self.area.queue_render()
