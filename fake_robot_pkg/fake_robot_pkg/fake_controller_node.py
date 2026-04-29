"""
fake_controller_node.py
=======================
Fake ROS 2 node for GUI development — no physical robot needed.

Published topics
----------------
  /controller/connection_status   mecanumbot_msgs/msg/ControllerStatus   1 Hz
  /controller/button_events       mecanumbot_msgs/msg/ButtonEvent         on demand
  /controller/joystick_events     mecanumbot_msgs/msg/JoystickEvent       on demand
  /opencr_state                   mecanumbot_msgs/msg/OpenCRState         10 Hz

Subscribed topics  (motor / accessory simulation)
--------------------------------------------------
  /cmd_vel               geometry_msgs/msg/Twist          → updates wheel velocities in OpenCR state
  /cmd_accessory_pos     mecanumbot_msgs/msg/AccessMotorCmd → updates servo positions in OpenCR state

Service servers
---------------
  /get_led_status   mecanumbot_msgs/srv/GetLedStatus
  /set_led_status   mecanumbot_msgs/srv/SetLedStatus

Interactive CLI
---------------
Button
  b   <button>                     press + release (50 ms)
  bh  <button>                     start holding (HOLD at 10 Hz)
  br  <button>                     release held button
  bra                              release ALL held buttons

Joystick
  j   <axis> <x> [<y>]             single event
  ja  <axis> <x> [<y>] <n> [dt]    burst of n events, dt s apart (default 0.05 s)
  Axis aliases: ls  rs  lt  rt

OpenCR state  (published at 10 Hz; values persist until changed)
  opencr battery <v>               set battery voltage  (e.g. 11.5)
  opencr vel <bl> <br> <fl> <fr>   set wheel velocities
  opencr imu ax <x> <y> <z>        set linear acc  x y z
  opencr imu av <x> <y> <z>        set angular vel x y z
  opencr show                      print current fake OpenCR values

LED  (stored in memory; served back by /get_led_status)
  led                              show current LED state
  led set <corner> <mode> <color>  set one corner
    corners: FL FR BL BR
    modes:   0=OFF 1=WAVE_RIGHT 2=WAVE_LEFT 3=PULSE 4=SOLID
    colors:  0=BLACK 1=WHITE 2=GREEN 3=RED 4=BLUE 5=CYAN 6=PINK 7=YELLOW
  led reset                        set all corners to mode=0 color=0

General
  status / buttons / joysticks / help / quit
"""

import threading
import time
import textwrap

import rclpy
from rclpy.node import Node

from mecanumbot_msgs.msg import ButtonEvent, JoystickEvent, ControllerStatus, OpenCRState, AccessMotorCmd
from mecanumbot_msgs.srv import GetLedStatus, SetLedStatus
from geometry_msgs.msg import Twist


# ── Constants ────────────────────────────────────────────────────────────────

BUTTONS: list[str] = [
    "A", "B", "X", "Y", "LB", "RB", "BACK", "START", "HOME",
    "LS", "RS", "DPadUp", "DPadDown", "DPadLeft", "DPadRight",
]
BUTTONS_LOWER: dict[str, str] = {b.lower(): b for b in BUTTONS}

JOYSTICK_ALIASES: dict[str, str] = {
    "ls": "Left Stick",
    "rs": "Right Stick",
    "lt": "Left Trigger (LT)",
    "rt": "Right Trigger (RT)",
}
JOYSTICKS: list[str] = list(JOYSTICK_ALIASES.values())

LED_COLOR_NAMES = {0: 'BLACK', 1: 'WHITE', 2: 'GREEN', 3: 'RED',
                   4: 'BLUE',  5: 'CYAN',  6: 'PINK',  7: 'YELLOW'}
LED_MODE_NAMES  = {0: 'OFF', 1: 'WAVE_RIGHT', 2: 'WAVE_LEFT', 3: 'PULSE', 4: 'SOLID'}
LED_CORNERS     = ('FL', 'FR', 'BL', 'BR')

HELP_TEXT = textwrap.dedent("""
    ╔═══════════════════════════════════════════════════════════════════╗
    ║               Fake Robot CLI – commands                          ║
    ╠═══════════════════════════════════════════════════════════════════╣
    ║  b   <button>                  press + release                  ║
    ║  bh  <button>                  start holding                    ║
    ║  br  <button>                  release held button              ║
    ║  bra                           release ALL held buttons         ║
    ║                                                                   ║
    ║  j   <axis> <x> [<y>]          single joystick event            ║
    ║  ja  <axis> <x> [<y>] <n> [dt] burst  (axis: ls rs lt rt)       ║
    ║                                                                   ║
    ║  opencr battery <v>            set battery voltage               ║
    ║  opencr vel <bl><br><fl><fr>   set wheel velocities              ║
    ║  opencr imu ax <x> <y> <z>     set linear acceleration          ║
    ║  opencr imu av <x> <y> <z>     set angular velocity             ║
    ║  opencr show                   print OpenCR values               ║
    ║                                                                   ║
    ║  led                           show LED state                    ║
    ║  led set <corner> <mode> <clr> set one corner                    ║
    ║  led reset                     all corners off                   ║
    ║                                                                   ║
    ║  status / buttons / joysticks / help / quit                      ║
    ║                                                                   ║
    ║  (Subscribing to /cmd_vel and /cmd_accessory_pos automatically)  ║
    ╚═══════════════════════════════════════════════════════════════════╝
""").strip()


# ── Default fake OpenCR state ────────────────────────────────────────────────

def _default_opencr() -> dict:
    return {
        'vel_bl': 0, 'vel_br': 0, 'vel_fl': 0, 'vel_fr': 0,
        'pos_bl': 0, 'pos_br': 0, 'pos_fl': 0, 'pos_fr': 0,
        'curr_bl': 0, 'curr_br': 0, 'curr_fl': 0, 'curr_fr': 0,
        'acc_bl': 0, 'acc_br': 0, 'acc_fl': 0, 'acc_fr': 0,
        'err_bl': 0, 'err_br': 0, 'err_fl': 0, 'err_fr': 0,
        'pos_n': 0, 'pos_gl': 0, 'pos_gr': 0,
        'battery_voltage': 12.0,
        'imu_angular_vel_x': 0.0, 'imu_angular_vel_y': 0.0, 'imu_angular_vel_z': 0.0,
        'imu_linear_acc_x': 0.0,  'imu_linear_acc_y': 0.0,  'imu_linear_acc_z': 9.81,
        'imu_magnetic_x': 0.0,    'imu_magnetic_y': 0.0,    'imu_magnetic_z': 0.0,
        'imu_orientation_w': 1.0,
        'imu_orientation_x': 0.0, 'imu_orientation_y': 0.0, 'imu_orientation_z': 0.0,
    }


# ── Node ─────────────────────────────────────────────────────────────────────

class FakeControllerNode(Node):
    """Publishes controller topics, serves LED services, and provides CLI helpers."""

    HOLD_INTERVAL = 0.10   # seconds between HOLD messages
    OPENCR_RATE   = 10.0   # Hz

    def __init__(self) -> None:
        super().__init__('fake_controller_node')

        # ── publishers ────────────────────────────────────────────────────
        self._btn_pub    = self.create_publisher(ButtonEvent,      '/controller/button_events',     10)
        self._joy_pub    = self.create_publisher(JoystickEvent,    '/controller/joystick_events',   10)
        self._stat_pub   = self.create_publisher(ControllerStatus, '/controller/connection_status', 10)
        self._opencr_pub = self.create_publisher(OpenCRState,      '/opencr_state',                 10)

        # ── timers ────────────────────────────────────────────────────────
        self.create_timer(1.0,                    self._publish_status)
        self.create_timer(1.0 / self.OPENCR_RATE, self._publish_opencr)

        # ── LED service servers ───────────────────────────────────────────
        self._led_state      = {c: {'mode': 0, 'color': 0} for c in LED_CORNERS}
        self._led_state_lock = threading.Lock()
        self.create_service(GetLedStatus, '/get_led_status', self._svc_get_led)
        self.create_service(SetLedStatus, '/set_led_status', self._svc_set_led)

        # ── OpenCR state ──────────────────────────────────────────────────
        self._opencr      = _default_opencr()
        self._opencr_lock = threading.Lock()

        # ── motor / accessory subscriptions ──────────────────────────────
        # Simulate physical robot: reflect received commands back into
        # the OpenCR state so the GUI sees the effect.
        self.create_subscription(Twist,          '/cmd_vel',            self._on_cmd_vel,       10)
        self.create_subscription(AccessMotorCmd, '/cmd_accessory_pos',  self._on_accessory_pos, 10)

        # ── hold threads ──────────────────────────────────────────────────
        self._hold_threads: dict[str, threading.Event] = {}
        self._hold_lock = threading.Lock()

        self.get_logger().info(
            'Fake robot node ready — /controller/* topics, /opencr_state, '
            '/get_led_status, /set_led_status, '
            'subscribed to /cmd_vel and /cmd_accessory_pos'
        )

    # ── status heartbeat ──────────────────────────────────────────────────

    def _now(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1e9)

    def _publish_status(self) -> None:
        msg = ControllerStatus()
        msg.connected = True
        msg.controller_type = 'Generic'
        msg.time_since_last_input = 0.0
        msg.timestamp = self._now()
        self._stat_pub.publish(msg)

    # ── OpenCR heartbeat ──────────────────────────────────────────────────

    def _publish_opencr(self) -> None:
        msg = OpenCRState()
        with self._opencr_lock:
            d = self._opencr.copy()
        now = self.get_clock().now().to_msg()
        msg.header.stamp    = now
        msg.header.frame_id = 'fake_opencr'
        msg.vel_bl = d['vel_bl']; msg.vel_br = d['vel_br']
        msg.vel_fl = d['vel_fl']; msg.vel_fr = d['vel_fr']
        msg.pos_bl = d['pos_bl']; msg.pos_br = d['pos_br']
        msg.pos_fl = d['pos_fl']; msg.pos_fr = d['pos_fr']
        msg.curr_bl = d['curr_bl']; msg.curr_br = d['curr_br']
        msg.curr_fl = d['curr_fl']; msg.curr_fr = d['curr_fr']
        msg.acc_bl = d['acc_bl']; msg.acc_br = d['acc_br']
        msg.acc_fl = d['acc_fl']; msg.acc_fr = d['acc_fr']
        msg.err_bl = d['err_bl']; msg.err_br = d['err_br']
        msg.err_fl = d['err_fl']; msg.err_fr = d['err_fr']
        msg.pos_n = d['pos_n']; msg.pos_gl = d['pos_gl']; msg.pos_gr = d['pos_gr']
        msg.battery_voltage    = d['battery_voltage']
        msg.imu_angular_vel_x  = d['imu_angular_vel_x']
        msg.imu_angular_vel_y  = d['imu_angular_vel_y']
        msg.imu_angular_vel_z  = d['imu_angular_vel_z']
        msg.imu_linear_acc_x   = d['imu_linear_acc_x']
        msg.imu_linear_acc_y   = d['imu_linear_acc_y']
        msg.imu_linear_acc_z   = d['imu_linear_acc_z']
        msg.imu_magnetic_x     = d['imu_magnetic_x']
        msg.imu_magnetic_y     = d['imu_magnetic_y']
        msg.imu_magnetic_z     = d['imu_magnetic_z']
        msg.imu_orientation_w  = d['imu_orientation_w']
        msg.imu_orientation_x  = d['imu_orientation_x']
        msg.imu_orientation_y  = d['imu_orientation_y']
        msg.imu_orientation_z  = d['imu_orientation_z']
        self._opencr_pub.publish(msg)

    # ── LED service callbacks ─────────────────────────────────────────────

    def _svc_get_led(self, _request, response):
        with self._led_state_lock:
            s = {c: dict(v) for c, v in self._led_state.items()}
        response.fl_mode  = s['FL']['mode'];  response.fl_color = s['FL']['color']
        response.fr_mode  = s['FR']['mode'];  response.fr_color = s['FR']['color']
        response.bl_mode  = s['BL']['mode'];  response.bl_color = s['BL']['color']
        response.br_mode  = s['BR']['mode'];  response.br_color = s['BR']['color']
        return response

    def _svc_set_led(self, request, response):
        with self._led_state_lock:
            self._led_state['FL'] = {'mode': request.fl_mode, 'color': request.fl_color}
            self._led_state['FR'] = {'mode': request.fr_mode, 'color': request.fr_color}
            self._led_state['BL'] = {'mode': request.bl_mode, 'color': request.bl_color}
            self._led_state['BR'] = {'mode': request.br_mode, 'color': request.br_color}
        response.success = True
        # Print LED change to CLI so user can see the effect
        self.get_logger().info(
            f'[LED] FL:{request.fl_mode}/{request.fl_color}  '
            f'FR:{request.fr_mode}/{request.fr_color}  '
            f'BL:{request.bl_mode}/{request.bl_color}  '
            f'BR:{request.br_mode}/{request.br_color}'
        )
        return response

    # ── motor / accessory simulation callbacks ────────────────────────────

    def _on_cmd_vel(self, msg: Twist) -> None:
        """
        Simulate mecanum drive kinematics from a Twist message.
        Converts linear.x / linear.y / angular.z → approximate wheel velocities
        and reflects them into the OpenCR state so the GUI shows movement.
        Scale factor 100 maps m/s → integer ticks (adjust as needed).
        """
        SCALE = 100
        vx  = msg.linear.x
        vy  = msg.linear.y
        wz  = msg.angular.z
        # Mecanum wheel mixing (simplified, equal wheel geometry)
        vel_fl = int((vx - vy - wz) * SCALE)
        vel_fr = int((vx + vy + wz) * SCALE)
        vel_bl = int((vx + vy - wz) * SCALE)
        vel_br = int((vx - vy + wz) * SCALE)
        with self._opencr_lock:
            self._opencr['vel_fl'] = vel_fl
            self._opencr['vel_fr'] = vel_fr
            self._opencr['vel_bl'] = vel_bl
            self._opencr['vel_br'] = vel_br
        self.get_logger().debug(
            f'[cmd_vel] vx={vx:.3f} vy={vy:.3f} wz={wz:.3f} → '
            f'fl={vel_fl} fr={vel_fr} bl={vel_bl} br={vel_br}'
        )

    def _on_accessory_pos(self, msg: AccessMotorCmd) -> None:
        """Reflect accessory motor commands into the OpenCR state."""
        with self._opencr_lock:
            self._opencr['pos_n']  = int(msg.n_pos)
            self._opencr['pos_gl'] = int(msg.gl_pos)
        self.get_logger().debug(
            f'[cmd_accessory_pos] n_pos={msg.n_pos:.3f} gl_pos={msg.gl_pos:.3f}'
        )

    # ── button helpers ────────────────────────────────────────────────────

    def _make_btn(self, event: str, button_name: str) -> ButtonEvent:
        msg = ButtonEvent()
        msg.event = event
        msg.button_name = button_name
        msg.button_id = -1
        msg.timestamp = self._now()
        return msg

    def press_release(self, button_name: str) -> None:
        self._btn_pub.publish(self._make_btn('PRESSED',  button_name))
        time.sleep(0.05)
        self._btn_pub.publish(self._make_btn('RELEASED', button_name))

    def start_hold(self, button_name: str) -> bool:
        with self._hold_lock:
            if button_name in self._hold_threads:
                return False
            stop_evt = threading.Event()
            self._hold_threads[button_name] = stop_evt

        def _loop() -> None:
            self._btn_pub.publish(self._make_btn('PRESSED', button_name))
            while not stop_evt.wait(self.HOLD_INTERVAL):
                self._btn_pub.publish(self._make_btn('HOLD', button_name))

        threading.Thread(target=_loop, daemon=True, name=f'hold-{button_name}').start()
        return True

    def stop_hold(self, button_name: str) -> bool:
        with self._hold_lock:
            stop_evt = self._hold_threads.pop(button_name, None)
        if stop_evt is None:
            return False
        stop_evt.set()
        time.sleep(0.02)
        self._btn_pub.publish(self._make_btn('RELEASED', button_name))
        return True

    def stop_all_holds(self) -> list[str]:
        with self._hold_lock:
            names = list(self._hold_threads.keys())
        return [n for n in names if self.stop_hold(n)]

    def held_buttons(self) -> list[str]:
        with self._hold_lock:
            return list(self._hold_threads.keys())

    # ── joystick helpers ──────────────────────────────────────────────────

    def send_joystick(self, joystick_name: str, x: float, y: float = 0.0) -> None:
        msg = JoystickEvent()
        msg.joystick_name = joystick_name
        msg.x = float(x)
        msg.y = float(y)
        msg.timestamp = self._now()
        self._joy_pub.publish(msg)

    # ── OpenCR CLI helpers ────────────────────────────────────────────────

    def set_opencr(self, key: str, value) -> None:
        with self._opencr_lock:
            self._opencr[key] = value

    def get_opencr_snapshot(self) -> dict:
        with self._opencr_lock:
            return self._opencr.copy()

    # ── LED CLI helpers ───────────────────────────────────────────────────

    def set_led_corner(self, corner: str, mode: int, color: int) -> None:
        with self._led_state_lock:
            self._led_state[corner] = {'mode': mode, 'color': color}

    def reset_led(self) -> None:
        with self._led_state_lock:
            self._led_state = {c: {'mode': 0, 'color': 0} for c in LED_CORNERS}

    def get_led_snapshot(self) -> dict:
        with self._led_state_lock:
            return {c: dict(v) for c, v in self._led_state.items()}


# ── CLI helpers ───────────────────────────────────────────────────────────────

def _resolve_button(raw: str) -> str | None:
    return BUTTONS_LOWER.get(raw.lower())

def _resolve_joystick(raw: str) -> str | None:
    alias = JOYSTICK_ALIASES.get(raw.lower())
    if alias:
        return alias
    return raw if raw in JOYSTICKS else None

def _parse_float(s: str, label: str) -> float | None:
    try:
        return float(s)
    except ValueError:
        print(f'  ✗  {label} must be a number, got: {s!r}')
        return None

def _parse_int(s: str, label: str) -> int | None:
    try:
        v = int(s)
        if v < 1:
            raise ValueError
        return v
    except ValueError:
        print(f'  ✗  {label} must be a positive integer, got: {s!r}')
        return None

def _print_led(state: dict) -> None:
    print('  LED state:')
    for corner in LED_CORNERS:
        d = state[corner]
        cname = LED_COLOR_NAMES.get(d['color'], str(d['color']))
        mname = LED_MODE_NAMES.get(d['mode'],   str(d['mode']))
        print(f'    {corner}: mode={d["mode"]} ({mname:10s})  color={d["color"]} ({cname})')

def _print_opencr(d: dict) -> None:
    print('  OpenCR state:')
    print(f'    battery_voltage : {d["battery_voltage"]:.3f} V')
    print(f'    wheel vel       : bl={d["vel_bl"]:5d}  br={d["vel_br"]:5d}  fl={d["vel_fl"]:5d}  fr={d["vel_fr"]:5d}')
    print(f'    wheel pos       : bl={d["pos_bl"]:5d}  br={d["pos_br"]:5d}  fl={d["pos_fl"]:5d}  fr={d["pos_fr"]:5d}')
    print(f'    wheel curr      : bl={d["curr_bl"]:5d}  br={d["curr_br"]:5d}  fl={d["curr_fl"]:5d}  fr={d["curr_fr"]:5d}')
    print(f'    servo pos       : n={d["pos_n"]}  gl={d["pos_gl"]}  gr={d["pos_gr"]}')
    print(f'    imu lin acc     : x={d["imu_linear_acc_x"]:.4f}  y={d["imu_linear_acc_y"]:.4f}  z={d["imu_linear_acc_z"]:.4f}')
    print(f'    imu ang vel     : x={d["imu_angular_vel_x"]:.4f}  y={d["imu_angular_vel_y"]:.4f}  z={d["imu_angular_vel_z"]:.4f}')
    print(f'    imu orientation : w={d["imu_orientation_w"]:.5f}  x={d["imu_orientation_x"]:.5f}  y={d["imu_orientation_y"]:.5f}  z={d["imu_orientation_z"]:.5f}')


# ── REPL ─────────────────────────────────────────────────────────────────────

def _repl(node: FakeControllerNode) -> None:
    print('\n' + HELP_TEXT + '\n')
    print('  Type "help" for commands.  ROS 2 node running in background.\n')

    while True:
        try:
            raw = input('robot> ').strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not raw:
            continue

        parts = raw.split()
        cmd   = parts[0].lower()
        args  = parts[1:]

        # ── quit ──────────────────────────────────────────────────────────
        if cmd in ('quit', 'exit', 'q'):
            node.stop_all_holds()
            break

        # ── help ──────────────────────────────────────────────────────────
        elif cmd in ('help', '?'):
            print(HELP_TEXT)

        # ── listings ──────────────────────────────────────────────────────
        elif cmd == 'buttons':
            print('  ' + '  '.join(BUTTONS))

        elif cmd == 'joysticks':
            for alias, full in JOYSTICK_ALIASES.items():
                print(f'  {alias:4s}  →  {full}')

        # ── status ────────────────────────────────────────────────────────
        elif cmd == 'status':
            held = node.held_buttons()
            print(f'  Held: {", ".join(held)}' if held else '  No buttons held.')

        # ── b: press + release ────────────────────────────────────────────
        elif cmd == 'b':
            if len(args) < 1:
                print('  Usage: b <button>')
                continue
            btn = _resolve_button(args[0])
            if btn is None:
                print(f'  ✗  Unknown button: {args[0]!r}  (type "buttons" to list)')
                continue
            node.press_release(btn)
            print(f'  ✓  {btn}: PRESSED → RELEASED')

        # ── bh: hold ──────────────────────────────────────────────────────
        elif cmd == 'bh':
            if len(args) < 1:
                print('  Usage: bh <button>')
                continue
            btn = _resolve_button(args[0])
            if btn is None:
                print(f'  ✗  Unknown button: {args[0]!r}')
                continue
            if node.start_hold(btn):
                print(f'  ✓  {btn}: holding  (use "br {btn}" to release)')
            else:
                print(f'  !  {btn} is already being held')

        # ── br: release ───────────────────────────────────────────────────
        elif cmd == 'br':
            if len(args) < 1:
                print('  Usage: br <button>')
                continue
            btn = _resolve_button(args[0])
            if btn is None:
                print(f'  ✗  Unknown button: {args[0]!r}')
                continue
            if node.stop_hold(btn):
                print(f'  ✓  {btn}: RELEASED')
            else:
                print(f'  !  {btn} was not being held')

        # ── bra: release all ──────────────────────────────────────────────
        elif cmd == 'bra':
            released = node.stop_all_holds()
            print(f'  ✓  Released: {", ".join(released)}' if released else '  !  Nothing was held')

        # ── j: single joystick ────────────────────────────────────────────
        elif cmd == 'j':
            if len(args) < 2:
                print('  Usage: j <axis> <x> [<y>]')
                continue
            axis = _resolve_joystick(args[0])
            if axis is None:
                print(f'  ✗  Unknown axis: {args[0]!r}  (use ls / rs / lt / rt)')
                continue
            x = _parse_float(args[1], 'x')
            if x is None:
                continue
            y = 0.0
            if len(args) >= 3:
                y = _parse_float(args[2], 'y')
                if y is None:
                    continue
            node.send_joystick(axis, x, y)
            print(f'  ✓  {axis}: x={x:.3f}  y={y:.3f}')

        # ── ja: burst of joystick events ──────────────────────────────────
        elif cmd == 'ja':
            if len(args) < 3:
                print('  Usage: ja <axis> <x> [<y>] <n> [<delay_s>]')
                continue
            axis = _resolve_joystick(args[0])
            if axis is None:
                print(f'  ✗  Unknown axis: {args[0]!r}')
                continue
            x = _parse_float(args[1], 'x')
            if x is None:
                continue
            ptr = 2
            y = 0.0
            try:
                n = int(args[ptr])
                ptr += 1
            except ValueError:
                y_val = _parse_float(args[ptr], 'y')
                if y_val is None:
                    continue
                y = y_val
                ptr += 1
                if ptr >= len(args):
                    print('  Missing <n>')
                    continue
                n = _parse_int(args[ptr], 'n')
                if n is None:
                    continue
                ptr += 1
            delay = 0.05
            if ptr < len(args):
                d = _parse_float(args[ptr], 'delay')
                if d is None:
                    continue
                delay = max(0.0, d)

            def _burst(axis=axis, x=x, y=y, n=n, delay=delay):
                for _ in range(n):
                    node.send_joystick(axis, x, y)
                    if delay > 0:
                        time.sleep(delay)

            threading.Thread(target=_burst, daemon=True).start()
            print(f'  ✓  {axis}: x={x:.3f} y={y:.3f}  ×{n}  (dt={delay:.3f} s)')

        # ── opencr ────────────────────────────────────────────────────────
        elif cmd == 'opencr':
            if not args:
                print('  Subcommands: battery  vel  imu  show')
                continue
            sub = args[0].lower()

            if sub == 'show':
                _print_opencr(node.get_opencr_snapshot())

            elif sub == 'battery':
                if len(args) < 2:
                    print('  Usage: opencr battery <voltage>')
                    continue
                v = _parse_float(args[1], 'voltage')
                if v is None:
                    continue
                node.set_opencr('battery_voltage', v)
                print(f'  ✓  battery_voltage = {v:.3f} V')

            elif sub == 'vel':
                if len(args) < 5:
                    print('  Usage: opencr vel <bl> <br> <fl> <fr>')
                    continue
                vals = []
                ok = True
                for i, name in enumerate(['bl', 'br', 'fl', 'fr'], 1):
                    try:
                        vals.append(int(args[i]))
                    except ValueError:
                        print(f'  ✗  {name} must be an integer')
                        ok = False
                        break
                if not ok:
                    continue
                for key, v in zip(['vel_bl', 'vel_br', 'vel_fl', 'vel_fr'], vals):
                    node.set_opencr(key, v)
                print(f'  ✓  wheel vel: bl={vals[0]}  br={vals[1]}  fl={vals[2]}  fr={vals[3]}')

            elif sub == 'imu':
                if len(args) < 5:
                    print('  Usage: opencr imu ax|av <x> <y> <z>')
                    continue
                kind = args[1].lower()
                if kind not in ('ax', 'av'):
                    print('  ✗  type must be ax (linear acc) or av (angular vel)')
                    continue
                xyz = []
                ok = True
                for label in ('x', 'y', 'z'):
                    v = _parse_float(args[2 + len(xyz)], label)
                    if v is None:
                        ok = False
                        break
                    xyz.append(v)
                if not ok:
                    continue
                if kind == 'ax':
                    for key, v in zip(['imu_linear_acc_x', 'imu_linear_acc_y', 'imu_linear_acc_z'], xyz):
                        node.set_opencr(key, v)
                    print(f'  ✓  imu linear acc: x={xyz[0]}  y={xyz[1]}  z={xyz[2]}')
                else:
                    for key, v in zip(['imu_angular_vel_x', 'imu_angular_vel_y', 'imu_angular_vel_z'], xyz):
                        node.set_opencr(key, v)
                    print(f'  ✓  imu angular vel: x={xyz[0]}  y={xyz[1]}  z={xyz[2]}')
            else:
                print(f'  ✗  Unknown opencr subcommand: {sub!r}  (battery / vel / imu / show)')

        # ── led ───────────────────────────────────────────────────────────
        elif cmd == 'led':
            if not args:
                _print_led(node.get_led_snapshot())
                continue
            sub = args[0].lower()

            if sub == 'reset':
                node.reset_led()
                print('  ✓  All LEDs reset to OFF')

            elif sub == 'set':
                # led set <corner> <mode> <color>
                if len(args) < 4:
                    print('  Usage: led set <corner> <mode> <color>')
                    print(f'  corners: {" ".join(LED_CORNERS)}')
                    print(f'  modes  : {LED_MODE_NAMES}')
                    print(f'  colors : {LED_COLOR_NAMES}')
                    continue
                corner = args[1].upper()
                if corner not in LED_CORNERS:
                    print(f'  ✗  Unknown corner: {args[1]!r}  (FL FR BL BR)')
                    continue
                try:
                    mode  = int(args[2])
                    color = int(args[3])
                except ValueError:
                    print('  ✗  mode and color must be integers')
                    continue
                node.set_led_corner(corner, mode, color)
                mname = LED_MODE_NAMES.get(mode,  str(mode))
                cname = LED_COLOR_NAMES.get(color, str(color))
                print(f'  ✓  {corner}: mode={mode} ({mname})  color={color} ({cname})')
            else:
                print(f'  ✗  Unknown led subcommand: {sub!r}  (set / reset)')

        else:
            print(f'  ✗  Unknown command: {cmd!r}  (type "help")')


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeControllerNode()

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True, name='rclpy-spin'
    )
    spin_thread.start()

    try:
        _repl(node)
    finally:
        print('Shutting down fake robot node…')
        node.stop_all_holds()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
