#!/usr/bin/env python3
# Copyright 2024 Apache License, Version 2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
"""
tui_joint_trajectory_controller
--------------------------------
Terminal User Interface equivalent of rqt_joint_trajectory_controller.

Provides the same Monitor / Control modes, joint-position sliders, speed
scaling, and automatic controller discovery as the rqt plugin — all inside
a terminal window.

Requirements (apt):
    sudo apt install python3-urwid

Usage:
    tui_joint_trajectory_controller [--ros-args ...]
    ros2 run rqt_joint_trajectory_controller tui_joint_trajectory_controller

Keyboard shortcuts:
    Q          Quit
    E          Toggle Enable / Disable control
    C          Cycle to next JointTrajectoryController
    R          Force immediate UI refresh
    ↑ / ↓      Navigate between joints
    ← / →      Coarse adjust selected joint (2 % of range per press)
    Shift+←/→  Fine adjust selected joint  (0.1 % of range per press)
    Home / End  Move joint to limit
    0 / Z       Zero selected joint
"""

# Make apt-installed system packages (urwid) accessible from the ROS 2 venv.
import sys
sys.path.insert(0, '/usr/lib/python3/dist-packages')

import math
import threading
import xml.dom.minidom
from typing import Callable, Dict, List, Optional, Tuple

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.srv import ListControllers
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import urwid

# ─── Timing constants (match rqt plugin defaults) ─────────────────────────────
CMD_HZ        = 10.0   # trajectory command publish frequency
DISPLAY_HZ    = 30.0   # UI refresh frequency
CTRL_POLL_HZ  = 1.0    # controller-manager polling frequency
MIN_TRAJ_S    = 0.5    # minimum trajectory point duration (seconds)

# ─── Colour palette ───────────────────────────────────────────────────────────
PALETTE: List[Tuple] = [
    ('header',   'white,bold',       'dark blue'),
    ('footer',   'white',            'dark blue'),
    ('key',      'yellow,bold',      'dark blue'),
    ('normal',   'light gray',       'default'),
    ('focus',    'black',            'light cyan'),
    ('monitor',  'dark cyan,bold',   'default'),
    ('control',  'light green,bold', 'default'),
    ('dim',      'dark gray',        'default'),
]


# ─── URDF helpers ─────────────────────────────────────────────────────────────

def parse_urdf_limits(urdf: str) -> Dict[str, Dict]:
    """Return {joint_name: {lower, upper, velocity}} parsed from a URDF string."""
    result: Dict[str, Dict] = {}
    try:
        doc = xml.dom.minidom.parseString(urdf)
        for jel in doc.getElementsByTagName('joint'):
            name  = jel.getAttribute('name')
            jtype = jel.getAttribute('type')
            if jtype in ('fixed', 'floating'):
                continue
            lims = jel.getElementsByTagName('limit')
            if not lims:
                if jtype == 'continuous':
                    result[name] = {'lower': -math.pi, 'upper': math.pi, 'velocity': 1.0}
                continue
            l     = lims[0]
            lower = float(l.getAttribute('lower') or '-3.14159')
            upper = float(l.getAttribute('upper') or  '3.14159')
            vel   = float(l.getAttribute('velocity') or '1.0')
            # Optional safety limits narrow the position range
            for s in jel.getElementsByTagName('safety_controller'):
                if s.hasAttribute('soft_lower_limit'):
                    lower = max(lower, float(s.getAttribute('soft_lower_limit')))
                if s.hasAttribute('soft_upper_limit'):
                    upper = min(upper, float(s.getAttribute('soft_upper_limit')))
            result[name] = {'lower': lower, 'upper': upper, 'velocity': max(vel, 1e-6)}
    except Exception:
        pass
    return result


# ─── urwid widgets ────────────────────────────────────────────────────────────

class JointSlider(urwid.Widget):
    """
    Single-row selectable widget for one joint:

      ▶ joint_name  -3.14 [████████░░░░░░░░░░░░] +3.14  cmd:+1.234  pos:+0.567

    In Monitor mode (active=False) the slider tracks the feedback position.
    In Control mode (active=True)  the slider sends commands on keypress.
    """
    _sizing = frozenset(['flow'])

    COARSE = 0.02    # fraction of range per coarse keypress
    FINE   = 0.001   # fraction of range per fine   keypress

    def __init__(self, name: str, low: float, high: float,
                 value: float = 0.0,
                 on_change: Optional[Callable] = None):
        super().__init__()
        self.name    = name
        self.low     = low
        self.high    = high
        self._cmd    = self._clamp(value)
        self._pos    = value
        self._active = False
        self._cb     = on_change

    # ── urwid interface ───────────────────────────────────────────────────────

    def selectable(self) -> bool:
        return True

    def rows(self, size, focus: bool = False) -> int:
        return 1

    def render(self, size, focus: bool = False):
        (maxcol,) = size
        row  = self._build_row(maxcol, focus)
        attr = 'focus' if focus else 'normal'
        return urwid.Text((attr, row)).render(size, focus=False)

    def keypress(self, size, key: str):
        if not self._active:
            return key
        rng = self.high - self.low
        if   key == 'left':        self._adjust(-rng * self.COARSE)
        elif key == 'right':       self._adjust( rng * self.COARSE)
        elif key == 'shift left':  self._adjust(-rng * self.FINE)
        elif key == 'shift right': self._adjust( rng * self.FINE)
        elif key == 'home':        self._do_set(self.low)
        elif key == 'end':         self._do_set(self.high)
        elif key in ('0', 'z'):    self._do_set(0.0)
        else:                      return key

    # ── public API ────────────────────────────────────────────────────────────

    def set_feedback(self, pos: float) -> None:
        """Update the displayed feedback position.
        In monitor mode the command slider also tracks this value."""
        self._pos = pos
        if not self._active:
            self._cmd = pos
        self._invalidate()

    def set_active(self, active: bool) -> None:
        self._active = active
        self._invalidate()

    def get_command(self) -> float:
        return self._cmd

    # ── internals ─────────────────────────────────────────────────────────────

    def _adjust(self, delta: float) -> None:
        self._do_set(self._cmd + delta)

    def _do_set(self, val: float) -> None:
        self._cmd = self._clamp(val)
        self._invalidate()
        if self._cb:
            self._cb(self.name, self._cmd)

    def _clamp(self, v: float) -> float:
        return max(self.low, min(self.high, v))

    def _build_row(self, maxcol: int, focus: bool) -> str:
        NAME_W = 22
        NUMS_W = 26   # '  cmd:±x.xxx  pos:±x.xxx'
        LIMS_W = 14   # '±x.xx[…]±x.xx'
        bar_w  = max(6, maxcol - NAME_W - LIMS_W - NUMS_W - 2)

        rng    = self.high - self.low
        frac   = (self._cmd - self.low) / rng if rng > 0 else 0.5
        frac   = max(0.0, min(1.0, frac))
        filled = int(frac * bar_w)
        bar    = '█' * filled + '░' * (bar_w - filled)

        cursor = '▶' if (focus and self._active) else ' '
        name   = self.name[:NAME_W].ljust(NAME_W)
        row = (
            f'{cursor}{name} {self.low:+.2f}[{bar}]{self.high:+.2f}'
            f'  cmd:{self._cmd:+7.3f}  pos:{self._pos:+7.3f}'
        )
        return row[:maxcol].ljust(maxcol)


class SpeedSlider(urwid.Widget):
    """Single-row interactive speed-scaling slider (1–100 %)."""
    _sizing = frozenset(['flow'])

    def __init__(self, value: float = 50.0,
                 on_change: Optional[Callable] = None):
        super().__init__()
        self._val    = value
        self._active = False
        self._cb     = on_change

    def selectable(self) -> bool:
        return True

    def rows(self, size, focus: bool = False) -> int:
        return 1

    def render(self, size, focus: bool = False):
        (maxcol,) = size
        row  = self._build_row(maxcol, focus)
        attr = 'focus' if focus else 'normal'
        return urwid.Text((attr, row)).render(size, focus=False)

    def keypress(self, size, key: str):
        if not self._active:
            return key
        if   key == 'left':        self._set(self._val - 5.0)
        elif key == 'right':       self._set(self._val + 5.0)
        elif key == 'shift left':  self._set(self._val - 1.0)
        elif key == 'shift right': self._set(self._val + 1.0)
        else:                      return key

    @property
    def value(self) -> float:
        return self._val

    def set_active(self, active: bool) -> None:
        self._active = active
        self._invalidate()

    def _set(self, v: float) -> None:
        self._val = max(1.0, min(100.0, v))
        self._invalidate()
        if self._cb:
            self._cb(self._val)

    def _build_row(self, maxcol: int, focus: bool) -> str:
        PREFIX  = ' Speed: '
        SUFFIX  = f' {self._val:5.1f}%  '
        HINT    = '(←→ adjust)' if self._active else '(disabled) '
        bar_w   = max(6, maxcol - len(PREFIX) - len(SUFFIX) - len(HINT) - 2)
        frac    = (self._val - 1.0) / 99.0
        filled  = int(frac * bar_w)
        bar     = '█' * filled + '░' * (bar_w - filled)
        row     = f'{PREFIX}[{bar}]{SUFFIX}{HINT}'
        return row[:maxcol].ljust(maxcol)


# ─── ROS 2 node ───────────────────────────────────────────────────────────────

class JTCNode(Node):
    """
    ROS 2 backend that mirrors the rqt_joint_trajectory_controller logic:

    - Subscribes to robot_description (latched) and parses URDF joint limits.
    - Polls /controller_manager/list_controllers at CTRL_POLL_HZ.
    - Subscribes to <ctrl_ns>/controller_state for joint feedback.
    - Publishes to <ctrl_ns>/joint_trajectory at CMD_HZ when enabled.
    """

    def __init__(self) -> None:
        super().__init__('tui_joint_trajectory_controller')
        self._lock = threading.Lock()

        # Configuration / state
        self._cm_ns        : str                    = '/controller_manager'
        self._ctrl_name    : str                    = ''
        self._ctrl_ns      : str                    = ''
        self._enabled      : bool                   = False
        self._speed        : float                  = 50.0   # percent
        self._joint_names  : List[str]              = []
        self._joint_pos    : Dict[str, float]       = {}     # feedback
        self._joint_cmd    : Dict[str, float]       = {}     # TUI commands
        self._joint_limits : Dict[str, Dict]        = {}
        self._avail_jtcs   : List[Tuple[str, str]]  = []    # (name, ns)

        # ROS objects (only modified from executor thread – no lock needed)
        self._state_sub   = None
        self._cmd_pub     = None
        self._list_cli    = None
        self._list_future = None

        # Latched robot_description
        rdqos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String, 'robot_description', self._robot_desc_cb, rdqos)

        self.create_timer(1.0 / CTRL_POLL_HZ, self._poll_controllers)
        self.create_timer(1.0 / CMD_HZ,       self._publish_cmd)

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _robot_desc_cb(self, msg: String) -> None:
        limits = parse_urdf_limits(msg.data)
        with self._lock:
            self._joint_limits = limits

    def _state_cb(self, msg: JointTrajectoryControllerState) -> None:
        with self._lock:
            new_names = list(msg.joint_names)
            # Initialise command/position tracking on first message or name change
            if new_names != self._joint_names:
                self._joint_names = new_names
                for n in new_names:
                    lim = self._joint_limits.get(
                        n, {'lower': -math.pi, 'upper': math.pi})
                    mid = (lim['lower'] + lim['upper']) / 2.0
                    self._joint_pos.setdefault(n, mid)
                    self._joint_cmd.setdefault(n, mid)
            for name, pos in zip(msg.joint_names, msg.feedback.positions):
                self._joint_pos[name] = pos

    def _poll_controllers(self) -> None:
        """Non-blocking 1-Hz discovery of active JointTrajectoryControllers."""
        svc = f'{self._cm_ns}/list_controllers'
        try:
            # Re-create client when CM namespace changes
            if self._list_cli is None or self._list_cli.srv_name != svc:
                if self._list_cli is not None:
                    self.destroy_client(self._list_cli)
                self._list_cli   = self.create_client(ListControllers, svc)
                self._list_future = None

            if self._list_future is None:
                if self._list_cli.service_is_ready():
                    self._list_future = self._list_cli.call_async(
                        ListControllers.Request())
            elif self._list_future.done():
                result            = self._list_future.result()
                self._list_future = None
                if result is not None:
                    jtcs = [
                        (c.name, f'{self._cm_ns}/{c.name}')
                        for c in result.controller
                        if c.state == 'active'
                        and 'JointTrajectoryController' in c.type
                    ]
                    with self._lock:
                        self._avail_jtcs = jtcs
        except Exception as exc:
            self.get_logger().warn(
                f'Controller poll failed: {exc}', throttle_duration_sec=5.0)

    def _publish_cmd(self) -> None:
        """Publish a JointTrajectory at CMD_HZ when control is enabled."""
        with self._lock:
            if not self._enabled or self._cmd_pub is None:
                return
            if not self._joint_names:
                return
            names  = list(self._joint_names)
            cmds   = {n: self._joint_cmd.get(n, self._joint_pos.get(n, 0.0))
                      for n in names}
            poses  = dict(self._joint_pos)
            limits = dict(self._joint_limits)
            speed  = self._speed / 100.0

        msg              = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names  = names
        pt               = JointTrajectoryPoint()
        max_dur          = MIN_TRAJ_S

        for n in names:
            cmd     = cmds[n]
            pos     = poses.get(n, cmd)
            vel     = limits.get(n, {}).get('velocity', 1.0)
            dur     = abs(cmd - pos) / max(vel * speed, 1e-6)
            max_dur = max(max_dur, dur)
            pt.positions.append(cmd)

        pt.time_from_start.sec     = int(max_dur)
        pt.time_from_start.nanosec = int((max_dur % 1.0) * 1e9)
        msg.points = [pt]

        with self._lock:
            pub = self._cmd_pub
        if pub:
            pub.publish(msg)

    # ── TUI-facing API (called from main/urwid thread) ────────────────────────

    def set_cm_ns(self, ns: str) -> None:
        with self._lock:
            if self._cm_ns != ns:
                self._cm_ns      = ns
                self._avail_jtcs = []
                self._list_future = None
                if self._list_cli:
                    self.destroy_client(self._list_cli)
                    self._list_cli = None

    def select_controller(self, name: str, ctrl_ns: str) -> None:
        """Switch to a new JointTrajectoryController."""
        self.set_enabled(False)
        if self._state_sub:
            self.destroy_subscription(self._state_sub)
            self._state_sub = None
        if self._cmd_pub:
            self.destroy_publisher(self._cmd_pub)
            self._cmd_pub = None
        with self._lock:
            self._ctrl_name   = name
            self._ctrl_ns     = ctrl_ns
            self._joint_names = []
            self._joint_pos   = {}
            self._joint_cmd   = {}
        if name:
            self._state_sub = self.create_subscription(
                JointTrajectoryControllerState,
                f'{ctrl_ns}/controller_state', self._state_cb, 10)
            self._cmd_pub = self.create_publisher(
                JointTrajectory,
                f'{ctrl_ns}/joint_trajectory', 10)

    def set_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._enabled = enabled

    def set_joint_cmd(self, name: str, val: float) -> None:
        with self._lock:
            if name in self._joint_cmd:
                self._joint_cmd[name] = val

    def set_speed(self, pct: float) -> None:
        with self._lock:
            self._speed = max(1.0, min(100.0, pct))

    def snapshot(self) -> Dict:
        """Thread-safe state snapshot consumed by the TUI refresh loop."""
        with self._lock:
            return {
                'enabled'     : self._enabled,
                'cm_ns'       : self._cm_ns,
                'ctrl_name'   : self._ctrl_name,
                'ctrl_ns'     : self._ctrl_ns,
                'joint_names' : list(self._joint_names),
                'joint_pos'   : dict(self._joint_pos),
                'joint_cmd'   : dict(self._joint_cmd),
                'joint_limits': dict(self._joint_limits),
                'avail_jtcs'  : list(self._avail_jtcs),
            }


# ─── TUI application ──────────────────────────────────────────────────────────

class JTCApp:
    """
    urwid-based TUI for JointTrajectoryController.

    Layout:
        ┌─ header (status) ──────────────────────────────────┐
        │ CM: /controller_manager                            │
        │ Ctrl: arm_controller  [1/2]  (C: cycle)           │
        │ Mode: MONITOR   E: toggle  |  N joints            │
        ├─ body (scrollable ListBox) ────────────────────────┤
        │  Speed: [████████████░░░░░░] 50.0%  (←→ adjust)  │
        │  joint1  -3.14[████████░░░░]+3.14  cmd:… pos:…   │
        │  joint2  …                                         │
        └─ footer (help) ────────────────────────────────────┘
    """

    def __init__(self, node: JTCNode) -> None:
        self.node              = node
        self._ctrl_idx         = 0
        self._joint_widgets    : Dict[str, JointSlider] = {}
        self._last_joint_names : List[str] = []
        self._loop             = None
        self._build_ui()

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        # ── Header (static, updated each refresh cycle) ──
        self._hdr_cm   = urwid.Text('')
        self._hdr_ctrl = urwid.Text('')
        self._hdr_mode = urwid.Text('')
        header = urwid.AttrMap(
            urwid.Padding(
                urwid.Pile([self._hdr_cm, self._hdr_ctrl, self._hdr_mode]),
                left=1, right=1,
            ),
            'header',
        )

        # ── Footer (keybinding help) ──
        footer = urwid.AttrMap(
            urwid.Text([
                ('key', ' Q'), ':Quit ',
                ('key', ' E'), ':Enable/Disable ',
                ('key', ' C'), ':Next controller ',
                ('key', ' ↑↓'), ':Navigate ',
                ('key', ' ←→'), ':Adjust ',
                ('key', ' Shift+←→'), ':Fine ',
                ('key', ' 0'), ':Zero ',
            ]),
            'footer',
        )

        # ── Body (scrollable joints + speed) ──
        self._speed_widget = SpeedSlider(50.0, on_change=self.node.set_speed)
        self._walker  = urwid.SimpleFocusListWalker([
            self._speed_widget,
            urwid.Text('  No controller selected.', align='center'),
        ])
        self._listbox = urwid.ListBox(self._walker)
        body = urwid.LineBox(
            self._listbox,
            title=' Joints   ←→:coarse   Shift+←→:fine   0:zero   Home/End:limits ',
        )

        self._frame = urwid.Frame(body, header=header, footer=footer)

    # ── Global key handler ────────────────────────────────────────────────────

    def _handle_input(self, key: str) -> None:
        if key in ('q', 'Q'):
            raise urwid.ExitMainLoop()

        elif key in ('e', 'E'):
            state       = self.node.snapshot()
            new_enabled = not state['enabled']
            if new_enabled and not state['ctrl_name']:
                return   # cannot enable without a controller
            self.node.set_enabled(new_enabled)
            self._speed_widget.set_active(new_enabled)
            for w in self._joint_widgets.values():
                w.set_active(new_enabled)
            if self._loop:
                self._loop.draw_screen()

        elif key in ('c', 'C'):
            state = self.node.snapshot()
            jtcs  = state['avail_jtcs']
            if jtcs:
                self._ctrl_idx = (self._ctrl_idx + 1) % len(jtcs)
                name, ns = jtcs[self._ctrl_idx]
                self.node.select_controller(name, ns)
            if self._loop:
                self._loop.draw_screen()

        elif key in ('r', 'R'):
            if self._loop:
                self._refresh(self._loop, None)

    # ── Periodic refresh (30 Hz) ──────────────────────────────────────────────

    def _refresh(self, loop, _data) -> None:
        state = self.node.snapshot()
        self._update_ui(state)
        loop.set_alarm_in(1.0 / DISPLAY_HZ, self._refresh)

    def _update_ui(self, state: Dict) -> None:
        # ── Header text ──
        jtcs    = state['avail_jtcs']
        n_jtcs  = len(jtcs)
        if self._ctrl_idx >= n_jtcs and n_jtcs > 0:
            self._ctrl_idx = 0

        ctrl_disp = state['ctrl_name'] or '(none selected – press C to cycle)'
        idx_disp  = f'{self._ctrl_idx + 1}/{n_jtcs}' if n_jtcs else '0/0'
        mode_attr = 'control' if state['enabled'] else 'monitor'
        mode_str  = 'CONTROL' if state['enabled'] else 'MONITOR'

        self._hdr_cm.set_text(f'CM:    {state["cm_ns"]}')
        self._hdr_ctrl.set_text(f'Ctrl:  {ctrl_disp}  [{idx_disp}]  (C: cycle)')
        self._hdr_mode.set_text([
            (mode_attr, f' {mode_str} '),
            ('header', f'  E: toggle  |  {len(state["joint_names"])} joints'),
        ])

        # ── Auto-select first available controller if none selected ──
        if not state['ctrl_name'] and jtcs:
            name, ns = jtcs[0]
            self._ctrl_idx = 0
            self.node.select_controller(name, ns)
            return

        # ── Rebuild joint sliders if joint set changed ──
        if state['joint_names'] != self._last_joint_names:
            self._rebuild_joints(state)
            self._last_joint_names = list(state['joint_names'])

        # ── Update feedback position on each slider ──
        for name, widget in self._joint_widgets.items():
            if name in state['joint_pos']:
                widget.set_feedback(state['joint_pos'][name])

    def _rebuild_joints(self, state: Dict) -> None:
        """Recreate JointSlider widgets whenever the active joint set changes."""
        self._joint_widgets = {}
        enabled = state['enabled']
        rows: List[urwid.Widget] = []

        for name in state['joint_names']:
            lim = state['joint_limits'].get(
                name, {'lower': -math.pi, 'upper': math.pi})
            pos = state['joint_pos'].get(name, 0.0)

            def _make_cb(joint_name: str) -> Callable:
                def cb(_, val: float) -> None:
                    self.node.set_joint_cmd(joint_name, val)
                return cb

            w = JointSlider(
                name=name,
                low=lim['lower'],
                high=lim['upper'],
                value=pos,
                on_change=_make_cb(name),
            )
            w.set_active(enabled)
            self._joint_widgets[name] = w
            rows.append(w)

        placeholder = [] if rows else [
            urwid.Text('  Waiting for joint state…', align='center')]
        self._walker[:] = [self._speed_widget] + rows + placeholder

    # ── Entry point ───────────────────────────────────────────────────────────

    def run(self) -> None:
        self._loop = urwid.MainLoop(
            self._frame,
            palette=PALETTE,
            unhandled_input=self._handle_input,
        )
        self._loop.set_alarm_in(1.0 / DISPLAY_HZ, self._refresh)
        self._loop.run()


# ─── Entry point ──────────────────────────────────────────────────────────────

def main() -> None:
    rclpy.init(args=sys.argv)
    node     = JTCNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        app = JTCApp(node)
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
