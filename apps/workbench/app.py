#!/usr/bin/env python3
"""Composable GTK desktop. Opening it does not enable a robot."""
import argparse
from pathlib import Path
import queue
import re
import signal
import sys
import threading
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
for folder in (ROOT, ROOT/'apps/fairino_keyboard', ROOT/'apps/fairino_upper_probe',
               ROOT/'apps/fairino_upper_probe/body_control', ROOT/'apps/fairino_tcp_probe'):
    sys.path.insert(0, str(folder))

import gi
from apps.workbench import gtk_support
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk, GLib

from apps.workbench.embedding import PanelHost
from apps.workbench.network import route_info, describe
from apps.workbench.telemetry import Feedback, fresh_hardware
from apps.workbench.teleop import TeleopPanel
from apps.gripper_control.panel import GripperPanel
from apps.gripper_control.feedback import measured_openings
from apps.workbench.slam_support import ensure_slam

ensure_slam()

# Several independent features may share one page without sharing a controller.
CONTROL_PAGES = {'base': 'map', 'joints': 'joints', 'body': 'body', 'tcp': 'tcp'}


class Workbench:
    def __init__(self, args):
        self.args = args
        self.closing = False
        self.leases = {}
        self.controls = {}
        self.previews = {}
        self.control_buttons = {}
        self.control_unlocked = set()
        self.starting = set()
        self.observers = {}
        self.retired_threads = []
        self.map_panel = None
        self.model_panel = None
        self.map_model_visible = False
        self.syncing_model_toggle = False
        self.events = queue.Queue()
        self.window = Gtk.Window(title='法奥机器人工作台' + (' · 离线演示' if args.demo else ''))
        self.window.set_name('robot-workbench')
        self.window_style = Gtk.CssProvider()
        self.window_style.load_from_data(b'#robot-workbench { background-color: #f4f6f9; color: #223248; }')
        self.window.get_style_context().add_provider(self.window_style, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)
        self.window.set_default_size(1500, 940)
        self.window.set_border_width(10)
        self.window.connect('delete-event', self.close)
        self.window.connect('key-press-event', self.key_event, 'key-press-event')
        self.window.connect('key-release-event', self.key_event, 'key-release-event')
        self.window.connect('focus-out-event', self.key_event, 'focus-out-event')
        self.window.connect('focus-in-event', self.key_event, 'focus-in-event')
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        self.window.add(box)
        heading = Gtk.Label(xalign=0)
        heading.set_markup('<span size="19000" weight="bold">机器人工作台</span>  ·  查看状态 / 控制 / 建图 / 遥操')
        box.pack_start(heading, False, False, 0)
        header = Gtk.Box(spacing=8)
        self.entries = {}
        for key, title, default in [('base', '底盘 IP', args.base_host), ('upper', '上身 IP', args.upper_host),
                                    ('user', '用户', args.user), ('password', 'SSH 密码', '')]:
            header.pack_start(Gtk.Label(label=title), False, False, 0)
            entry = Gtk.Entry(text=default)
            entry.set_width_chars(14 if key in ('base', 'upper') else 9)
            if key == 'password':
                entry.set_visibility(False)
                entry.set_input_purpose(Gtk.InputPurpose.PASSWORD)
            self.entries[key] = entry
            header.pack_start(entry, False, False, 0)
        connect = Gtk.Button(label='连接 / 刷新只读状态')
        connect.connect('clicked', self.connect_observers)
        header.pack_start(connect, False, False, 0)
        disconnect = Gtk.Button(label='断开查看')
        disconnect.connect('clicked', self.disconnect_observers)
        header.pack_start(disconnect, False, False, 0)
        self.model_toggle = Gtk.CheckButton(label='显示机器人模型')
        self.model_toggle.connect('toggled', self.toggle_model)
        header.pack_end(self.model_toggle, False, False, 0)
        box.pack_start(header, False, False, 0)
        self.routes = Gtk.Label(label='正在检查连接路由…', xalign=0)
        self.routes.set_line_wrap(True)
        box.pack_start(self.routes, False, False, 0)
        self.message = Gtk.Label(label='打开工作台不会使能；请先连接查看状态。', xalign=0)
        self.message.set_line_wrap(True)
        box.pack_start(self.message, False, False, 0)
        self.hardware_strip = Gtk.Box(spacing=18)
        box.pack_start(self.hardware_strip, False, False, 0)
        split = Gtk.Paned(orientation=Gtk.Orientation.HORIZONTAL)
        self.content_split = split
        box.pack_start(split, True, True, 0)
        self.notebook = Gtk.Notebook()
        self.notebook.set_scrollable(True)
        self.notebook.connect('switch-page', self.switched)
        split.pack1(self.notebook, resize=True, shrink=False)
        self.pages = {}
        self.page_scrolls = {}
        self.page_controls = {}
        for name, title in [('map', '建图与底盘'), ('joints', '关节'), ('body', '身体'), ('tcp', 'TCP'), ('gripper', '夹爪调试'), ('teleop', 'PICO 遥操'), ('maintenance', '维护')]:
            page = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
            page.set_border_width(10)
            self.pages[name] = page
            # Gtk.Notebook requests the tallest page even while it is hidden.
            # Long option panels must scroll instead of enlarging the map window.
            # Joint/body/TCP pages already scroll their embedded controls.
            if name in ('gripper', 'teleop', 'maintenance'):
                scroll = Gtk.ScrolledWindow()
                scroll.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)
                scroll.set_propagate_natural_height(False)
                scroll.set_propagate_natural_width(False)
                scroll.add(page)
                self.page_scrolls[name] = scroll
                self.notebook.append_page(scroll, Gtk.Label(label=title))
            else:
                self.notebook.append_page(page, Gtk.Label(label=title))
        self.map_container = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        self.pages['map'].pack_start(self.map_container, True, True, 0)
        self.map_placeholder()
        for name in ('base', 'joints', 'body', 'tcp'):
            page = self.pages[CONTROL_PAGES[name]]
            if name == 'base':
                self.base_controls_widget = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
                self.map_container.pack_end(self.base_controls_widget, False, False, 0)
                page = self.base_controls_widget
            row = Gtk.Box(spacing=8)
            label = '启动底盘 · 回正并使能' if name == 'base' else '启动本控制 · 使能上半身'
            if args.demo:
                label = '打开离线演示'
            button = Gtk.Button(label=label)
            button.connect('clicked', lambda _, n=name: self.start_control(n))
            row.pack_start(button, False, False, 0)
            stop = Gtk.Button(label='结束本控制并下使能')
            stop.connect('clicked', lambda _, n=name: self.stop_control(n))
            row.pack_start(stop, False, False, 0)
            stop.set_sensitive(False)
            self.control_buttons[name] = (button, stop)
            page.pack_start(row, False, False, 0)
            container = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
            if name == 'base':
                page.pack_start(container, False, False, 0)
            else:
                scroll = Gtk.ScrolledWindow()
                scroll.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)
                scroll.add(container)
                page.pack_start(scroll, True, True, 0)
            self.page_controls[name] = container
            self.show_control_preview(name)
        self.teleop = TeleopPanel(self.credentials, self.acquire, self.release, args.demo)
        self.pages['teleop'].pack_start(self.teleop, True, True, 0)
        self.gripper = GripperPanel(self.credentials, self.acquire, self.release, args.demo)
        self.pages['gripper'].pack_start(self.gripper, True, True, 0)
        self.add_maintenance()
        self.add_model(split)
        split.set_position(1000)
        self.window.show_all()
        GLib.timeout_add(100, self.tick)
        GLib.timeout_add_seconds(5, self.refresh_routes)
        self.refresh_routes()
        page = CONTROL_PAGES.get(args.page, args.page)
        if page in self.pages:
            self.notebook.set_current_page(list(self.pages).index(page))
        self.update_model_visibility()
        if args.demo:
            self.connect_observers()

    def map_placeholder(self):
        self.map_container.pack_start(Gtk.Label(
            label='点击「连接 / 刷新只读状态」显示地图和双雷达。\n底盘启动后，可在本页按键移动并实时查看建图。'), True, True, 0)
        if hasattr(self, 'base_controls_widget'):
            self.unparent_base_controls()
            self.map_container.pack_end(self.base_controls_widget, False, False, 0)
        self.map_container.show_all()

    def unparent_base_controls(self):
        parent = self.base_controls_widget.get_parent()
        if parent is not None:
            parent.remove(self.base_controls_widget)

    def control_args(self, name, credentials=None, preview=False):
        part = 'base' if name == 'base' else 'upper'
        host, user = credentials[:2] if credentials else (self.entries[part].get_text(), self.entries['user'].get_text())
        return SimpleNamespace(host=host, user=user, demo=self.args.demo and not preview,
                               check=False, prepare_check=False, linear=.06, angular=.10,
                               group='body', arm='left', calibrate_upright=False,
                               config=ROOT/'apps/fairino_upper_probe/body_control/body_config.json')

    @staticmethod
    def control_feature(name, args, preview=False):
        if name == 'base':
            import keyboard_control as feature
            extra = ()
        elif name == 'joints':
            import upper_keyboard as feature
            extra = ()
        elif name == 'body':
            import body_keyboard as feature
            extra = (feature.load_config(args.config, live=not (args.demo or preview)),)
        else:
            import tcp_keyboard as feature
            extra = (None if preview else feature.VendorKinematics(),)
        return feature, extra

    def show_control_preview(self, name, message='未启动 · 控件仅供查看；点击上方启动按钮后可操作。'):
        args = self.control_args(name, preview=True)
        feature, extra = self.control_feature(name, args, preview=True)
        host = PanelHost(self.window)
        host.set_sensitive(False)
        options = {'compact': True} if name == 'base' else {}
        control = feature.Window(args, *extra, connection=None, window_factory=lambda **_: host,
                                 on_closed=lambda: None, start_timer=False, **options)
        control.phase.set_text(message)
        host.set_tooltip_text(message)
        old = self.previews.pop(name, None)
        if old:
            old.window.destroy()
        self.previews[name] = control
        self.page_controls[name].pack_start(host, True, True, 0)
        host.show_all()

    def refresh_control_access(self):
        for name, (start, stop) in self.control_buttons.items():
            part = 'base' if name == 'base' else 'upper'
            control = self.controls.get(name)
            start.set_sensitive(not self.closing and part not in self.leases)
            stop.set_sensitive(bool(control and not control.closing))
            if control:
                # ready may temporarily be false during a trajectory. Keep the
                # session open so release/cancel events can still be delivered.
                if control.ready:
                    self.control_unlocked.add(name)
                control.window.set_sensitive(name in self.control_unlocked
                                             and not control.stopped and not control.closing)

    def credentials(self, part):
        host, user = self.entries[part].get_text().strip(), self.entries['user'].get_text().strip()
        if not re.fullmatch(r'[A-Za-z0-9_.:-]+', host) or not re.fullmatch(r'[A-Za-z0-9_-]+', user):
            self.message.set_text('请填写有效 IP / 主机名和 SSH 用户名')
            return None
        password = self.entries['password'].get_text()
        if not password and not self.args.demo:
            self.message.set_text('请在顶部填写 SSH 密码；仅在内存中使用，不保存')
            return None
        return host, user, password

    def acquire(self, part, owner):
        if self.closing:
            return False
        if part in self.leases:
            self.message.set_text(f'{part} 正由 {self.leases[part]} 使用，请先结束该控制并等待退出')
            return False
        self.leases[part] = owner
        return True

    def release(self, part, owner):
        if self.leases.get(part) == owner:
            self.leases.pop(part)

    def refresh_routes(self):
        if self.closing:
            return False
        if self.args.demo:
            self.routes.set_text('离线演示 · 使用本机示例地图和模拟状态，不连接机器人')
            return True
        hosts = {part: self.entries[part].get_text().strip() for part in ('base', 'upper')}
        def work():
            values = {part: describe(route_info(host)) for part, host in hosts.items()}
            self.events.put(('routes', values))
        threading.Thread(target=work, daemon=True).start()
        return True

    def connect_observers(self, *_):
        if self.closing:
            return
        # Check the lazily loaded plotting backend before replacing observers:
        # an incomplete GUI environment must not leave a half-connected session.
        try:
            from apps.map_viewer.viewer import Stream
            from apps.map_viewer.panel import MapPanel
        except ImportError as exc:
            self.message.set_text(
                f'地图绘图依赖未就绪：{exc}。请在项目目录运行 bash setup.sh console，然后重启工作台。')
            return
        if not self.disconnect_observers():
            return
        if self.args.demo:
            from apps.map_viewer.demo import DemoStream
            stream = DemoStream()
            self.mount_map_panel(MapPanel, stream, start_stream=False)
            self.message.set_text('离线演示 · 可查看地图图层和选择目标；不会连接或移动机器人')
            self.refresh_routes()
            return
        for part in ('base', 'upper'):
            creds = self.credentials(part)
            if creds:
                observer = Feedback(*creds, part)
                self.observers[part] = observer
                observer.thread.start()
        creds = self.credentials('base')
        if creds:
            host, user, password = creds
            stream = Stream(SimpleNamespace(host=host, user=user, check=False, duration=0), password)
            self.mount_map_panel(MapPanel, stream)
        self.message.set_text('只读状态连接中；控制功能需分别显式启动')
        self.refresh_routes()

    def mount_map_panel(self, panel_class, stream, start_stream=True):
        self.unparent_base_controls()
        for child in self.map_container.get_children():
            child.destroy()
        self.map_panel = panel_class(stream, ROOT/'logs/map-viewer', compact=True,
                                     start_stream=start_stream, acquire=self.acquire, release=self.release)
        self.map_panel.attach_base_controls(self.base_controls_widget)
        if 'base' in self.controls:
            self.map_panel.base_expander.set_expanded(True)
        self.map_container.pack_start(self.map_panel, True, True, 0)
        self.map_container.show_all()

    def disconnect_observers(self, *_):
        if self.map_panel:
            if not self.map_panel.close():
                self.message.set_text('请等待地图操作完成后再断开或刷新连接')
                return False
            self.retired_threads.append(self.map_panel.stream.thread)
            self.map_panel.detach_base_controls()
            self.unparent_base_controls()
            self.map_panel.destroy()
            self.map_panel = None
            self.map_placeholder()
        for observer in self.observers.values():
            observer.stop.set()
            self.retired_threads.append(observer.thread)
        self.observers.clear()
        for part, label in getattr(self, 'hardware_labels', {}).items():
            label.set_text(('底盘' if part == 'base' else '上半身') + '：未连接')
        self.message.set_text('只读查看已断开；正在运行的控制由其各自的结束按钮关闭')
        return True

    def start_control(self, name):
        if name == 'base' and self.map_panel:
            self.map_panel.base_expander.set_expanded(True)
        part = 'base' if name == 'base' else 'upper'
        creds = self.credentials(part)
        if creds is None or not self.acquire(part, name):
            return
        self.starting.add(name)
        host, user, password = creds
        args = self.control_args(name, creds)
        if name in self.previews:
            self.previews[name].phase.set_text('正在启动，控件暂不可操作；等待控制器就绪。')
        self.refresh_control_access()
        self.message.set_text(f'正在启动 {name}；运动启动过程见控制面板')
        def work():
            connection = None
            try:
                feature, extra = self.control_feature(name, args)
                if not args.demo:
                    if name in ('base', 'joints'):
                        connection = feature.Connection(args, password=password)
                    elif name == 'body':
                        connection = feature.Connection(args, worker_command=feature.remote_command(args, extra[0]),
                                                        log_directory=feature.HERE/'logs', password=password)
                    else:
                        connection = feature.Connection(args, worker_command=feature.remote_command(args),
                                                        log_directory=feature.HERE/'logs', password=password)
                self.events.put(('control_ready', (name, args, feature, extra, connection)))
            except Exception as exc:
                if connection:
                    connection.finish()
                self.events.put(('control_error', (name, str(exc))))
        threading.Thread(target=work, daemon=True).start()

    def attach_control(self, name, args, feature, extra, connection):
        if self.closing:
            def discard():
                if connection:
                    connection.finish()
                self.events.put(('control_error', (name, '关闭期间已取消启动')))
            threading.Thread(target=discard, daemon=True).start()
            return
        host = PanelHost(self.window)
        host.set_sensitive(False)
        try:
            options = {'compact': True} if name == 'base' else {}
            control = feature.Window(args, *extra, connection=connection, window_factory=lambda **_: host,
                                     on_closed=lambda: self.control_closed(name), **options)
            self.controls[name] = control
            self.control_unlocked.discard(name)
            self.page_controls[name].pack_start(host, True, True, 0)
            old = self.previews.pop(name, None)
            if old:
                old.window.destroy()
            self.pages[CONTROL_PAGES[name]].show_all()
            self.starting.discard(name)
            self.refresh_control_access()
            self.switched(self.notebook, None, self.notebook.get_current_page())
            self.message.set_text(('底盘' if name == 'base' else name) + '控制面板已连接，启动进度和停止结果见面板状态。')
        except Exception as exc:
            host.destroy()
            def discard(message=str(exc)):
                if connection:
                    connection.finish()
                self.events.put(('control_error', (name, message)))
            threading.Thread(target=discard, daemon=True).start()

    def control_closed(self, name):
        control = self.controls.pop(name, None)
        if control:
            control.window.destroy()
        self.release('base' if name == 'base' else 'upper', name)
        self.control_unlocked.discard(name)
        if not self.closing:
            self.show_control_preview(name, '控制已结束 · 控件仅供查看；再次启动后可操作。')
            self.refresh_control_access()
        return False

    def stop_control(self, name):
        if name in self.controls:
            self.controls[name].close()

    def switched(self, _, page, page_number):
        names = list(self.pages)
        current = names[page_number] if page_number < len(names) else None
        for name, control in self.controls.items():
            was_enabled = control.window.input_enabled
            control.window.input_enabled = CONTROL_PAGES[name] == current
            if was_enabled and not control.window.input_enabled:
                control.window.dispatch('focus-out-event', None)
            elif not was_enabled and control.window.input_enabled:
                control.window.dispatch('focus-in-event', None)
        if hasattr(self, 'model_toggle'):
            self.update_model_visibility(current)

    def toggle_model(self, *_):
        if self.syncing_model_toggle or self.model_panel is None:
            return
        if self.notebook.get_current_page() == list(self.pages).index('map'):
            self.map_model_visible = self.model_toggle.get_active()
        self.model_panel.set_visible(self.model_toggle.get_active())

    def update_model_visibility(self, current=None):
        if self.model_panel is None:
            return
        if current is None:
            current = list(self.pages)[max(0, self.notebook.get_current_page())]
        visible = self.map_model_visible if current == 'map' else True
        self.syncing_model_toggle = True
        self.model_toggle.set_active(visible)
        self.syncing_model_toggle = False
        self.model_panel.set_visible(visible)

    def key_event(self, _, event, kind):
        # Releases go to every control so changing tabs never strands a key.
        result = False
        for control in list(self.controls.values()):
            if kind != 'key-press-event' or control.window.input_enabled:
                result = control.window.dispatch(kind, event) or result
        return result

    def add_model(self, split):
        from apps.workbench.model_view import ModelView
        panel = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        self.model_panel = panel
        panel.set_no_show_all(True)
        panel.set_size_request(340, -1)
        self.hardware_labels = {}
        for part, title in [('base', '底盘'), ('upper', '上半身')]:
            label = Gtk.Label(label=title+'：未连接', xalign=0)
            label.set_line_wrap(True)
            self.hardware_labels[part] = label
            label.set_max_width_chars(44)
            self.hardware_strip.pack_start(label, True, True, 0)
        self.model = ModelView()
        self.teleop.tool_preview = self.model.set_tool_geometry
        self.body_readout = Gtk.Label(label='身体：等待实测角度', xalign=0)
        self.body_readout.set_line_wrap(True)
        self.body_readout.set_max_width_chars(48)
        self.hardware_strip.pack_start(self.body_readout, True, True, 0)
        panel.pack_start(self.model, True, True, 0)
        config = self.model.model.body.config
        calibration = ('复用身体竖直标定：' + config.get('upright_saved_at', '日期未记录')
                       if config.get('upright_confirmed') else '身体竖直参考尚未确认')
        hint = Gtk.Label(label='电脑端渲染 · 左键旋转 / 右键平移 / 滚轮缩放。\n' + calibration + '。\n升降毫米数为模型估算；头部和腰部跟随关节反馈，轮组为静态参考。', xalign=0)
        hint.set_line_wrap(True)
        panel.pack_start(hint, False, False, 0)
        split.pack2(panel, resize=True, shrink=False)
        for child in panel.get_children():
            child.show_all()

    def add_maintenance(self):
        page = self.pages['maintenance']
        label = Gtk.Label(label='先结束对应部位的控制并确认停稳；允许部分轴仍使能。\n单次清错 → 故障归零后下使能 → 最终复查；完成后保持停用。', xalign=0)
        label.set_line_wrap(True)
        page.pack_start(label, False, False, 0)
        for part, title in [('base', '底盘'), ('upper', '上半身')]:
            for execute in (False, True):
                button = Gtk.Button(label=title + ('：单次清错并下使能' if execute else '：只读故障检查'))
                button.connect('clicked', lambda _, p=part, e=execute: self.maintenance(p, e))
                page.pack_start(button, False, False, 0)
        self.maintenance_result = Gtk.Label(xalign=0, yalign=0)
        self.maintenance_result.set_selectable(True)
        self.maintenance_result.set_line_wrap(True)
        page.pack_start(self.maintenance_result, True, True, 0)

    def maintenance(self, part, execute):
        # Uses the same remote implementation as the standalone reset command.
        if self.args.demo:
            self.maintenance_result.set_text('离线演示：未连接、未清错')
            return
        creds = self.credentials(part)
        if creds is None or not self.acquire(part, '维护'):
            return
        self.maintenance_result.set_text('读取 / 处理故障中…')
        def work():
            try:
                from apps.workbench.maintenance import reset
                result = reset(part, *creds, execute=execute)
            except Exception as exc:
                result = str(exc)
            self.events.put(('maintenance', (part, result)))
        threading.Thread(target=work, daemon=True).start()

    def tick(self):
        while not self.events.empty():
            kind, value = self.events.get_nowait()
            if kind == 'routes':
                self.routes.set_text('底盘：' + value['base'] + '\n上半身：' + value['upper'])
            elif kind == 'control_ready':
                self.attach_control(*value)
            elif kind == 'control_error':
                name, error = value
                self.starting.discard(name)
                self.release('base' if name == 'base' else 'upper', name)
                self.message.set_text(f'{name}：{error}')
                if name in self.previews:
                    self.previews[name].phase.set_text('启动未完成，控件保持禁用；原因见上方提示。')
            elif kind == 'maintenance':
                part, result = value
                self.release(part, '维护')
                self.maintenance_result.set_text(result)
        upper = None
        for part, observer in self.observers.items():
            state = observer.snapshot()
            hardware = fresh_hardware(state)
            title = '底盘' if part == 'base' else '上半身'
            if hardware:
                faults = ', '.join(f'{n.upper()}={e}' for n, e in zip(hardware['joint_names'], hardware['errorcode']) if e) or '无'
                self.hardware_labels[part].set_text(f'{title} · 使能 {sum(hardware["enabled"])}/{len(hardware["enabled"])}\n'
                    f'故障 0x{hardware["error"]:08X} · {faults}')
                if part == 'upper':
                    upper = hardware
            else:
                self.hardware_labels[part].set_text(f'{title}：反馈未更新 / 状态未知\n'
                    + state.get('error', state.get('status', '等待硬件状态')))
        try:
            gripper_state = (self.gripper.session.snapshot() if self.gripper.session
                             and not self.gripper.session.finished() else {})
            jaws = measured_openings(gripper_state, self.teleop.gripper_feedback)
            demo_state = self.teleop.model_snapshot()
            if demo_state is not None:
                model_hardware = demo_state['hardware']
                self.model.update(model_hardware, live_arms=demo_state['fresh'],
                    grippers=demo_state['grippers'], demo=True,
                    demo_active=demo_state['active'],
                    wrist_collision=demo_state.get('wrist_collision', {}))
                body_prefix = '身体（模型演示）'
            else:
                model_hardware = upper
                self.model.update(upper, live_arms=self.args.live_arms, grippers=jaws)
                body_prefix = '身体'
            state = self.model.model.body_state(model_hardware)
            self.body_readout.set_text(
                f'{body_prefix}：{state["height"]:.1f}% · 相对低位约 {state["height_mm"]:.1f} mm\n'
                f'前倾 {state["forward"]:.1f}° · 侧倾 {state["roll"]:.1f}° · 腰转 {state["yaw"]:.1f}°'
                if state else body_prefix + '：等待数据')
        except (ValueError, KeyError) as exc:
            self.message.set_text('模型反馈不可用：' + str(exc))
        for key, entry in self.entries.items():
            entry.set_sensitive(not self.leases and not self.observers and not self.closing)
        self.refresh_control_access()
        if self.closing and not self.controls and not self.starting and not self.leases and self.teleop.finished() and self.gripper.finished():
            if all(not o.thread.is_alive() for o in self.observers.values()) and not any(t.is_alive() for t in self.retired_threads) and (not self.map_panel or not self.map_panel.stream.thread.is_alive()):
                Gtk.main_quit()
                return False
        self.retired_threads = [t for t in self.retired_threads if t.is_alive()]
        return True

    def retry_close(self):
        if self.closing:
            self.close_pending = False
            return False
        self.close()
        return not self.closing

    def close(self, *_):
        if self.closing:
            return True
        if self.map_panel and not self.map_panel.close():
            self.message.set_text('等待地图操作及导航停止、下使能完成后关闭工作台')
            if not getattr(self, 'close_pending', False):
                self.close_pending = True
                GLib.timeout_add(300, self.retry_close)
            return True
        self.closing = True
        self.message.set_text('正在结束本工作台启动的控制，等待下使能及连接退出…')
        for name in list(self.controls):
            self.stop_control(name)
        self.teleop.close()
        self.gripper.close()
        for observer in self.observers.values():
            observer.stop.set()
        return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo', action='store_true', help='离线界面，不连接机器人')
    parser.add_argument('--base-host', default='192.168.58.3')
    parser.add_argument('--upper-host', default='192.168.58.1')
    parser.add_argument('--user', default='fairino')
    parser.add_argument('--page', choices=('map', 'base', 'joints', 'body', 'tcp', 'gripper', 'teleop', 'maintenance'), default='map')
    parser.add_argument('--live-arms', action='store_true', default=True, help='兼容选项；默认显示双臂及已标定折叠关节')
    parser.add_argument('--static-model', action='store_false', dest='live_arms', help='只显示静态模型参考')
    args = parser.parse_args()
    settings = Gtk.Settings.get_default()
    if settings:
        settings.set_property('gtk-font-name', 'Noto Sans CJK SC 10')
    app = Workbench(args)
    signal.signal(signal.SIGINT, app.close)
    signal.signal(signal.SIGTERM, app.close)
    Gtk.main()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
