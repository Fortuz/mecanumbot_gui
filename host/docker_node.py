import os
import time
import json
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

try:
    from rosidl_runtime_py import message_to_ordereddict
    _MSG_TO_DICT_AVAILABLE = True
except ImportError:
    _MSG_TO_DICT_AVAILABLE = False

try:
    from mecanumbot_msgs.srv import SyncActions, RegisterAction, SyncMappings
    from mecanumbot_msgs.msg import (ActionTuple, ActionDescriptor,
                                     ControllerStatus, ButtonEvent, JoystickEvent)
    SYNC_SRV_AVAILABLE = True
except ImportError:
    SYNC_SRV_AVAILABLE = False
    ControllerStatus = None
    ButtonEvent      = None
    JoystickEvent    = None
    print("[DockerNode] mecanumbot_msgs not found — SyncActions/RegisterAction services disabled")

try:
    from mecanumbot_msgs.srv import SetLedStatus, GetLedStatus
    LED_SRV_AVAILABLE = True
except ImportError:
    LED_SRV_AVAILABLE = False
    SetLedStatus = None
    GetLedStatus = None
    print("[DockerNode] mecanumbot_msgs not found — SetLedStatus/GetLedStatus services disabled")

try:
    from mecanumbot_msgs.msg import OpenCRState
    _OPENCR_MSG_AVAILABLE = True
except ImportError:
    OpenCRState = None
    _OPENCR_MSG_AVAILABLE = False
    print("[DockerNode] mecanumbot_msgs/OpenCRState not available — robot state topic disabled")

# ──────────────────────────────────────────────────────────────
# Shared globals — written by DockerNode thread, read by Flask.
# ──────────────────────────────────────────────────────────────

LATEST_CONTROLLER_STATUS = {
    "connected": False,
    "controller_type": "Unknown",
    "time_since_last_input": 0,
    "timestamp": 0,
    "last_updated": None
}

LATEST_SYNC_RESULT = {
    "attempted": False,
    "success": None,
    "accepted_count": 0,
    "conflicts": [],
    "timestamp": None
}

LATEST_MAPPING_SYNC_RESULT = {
    "attempted": False,
    "success": None,
    "loaded_button_count": 0,
    "loaded_joystick_count": 0,
    "loaded_button_names": [],
    "loaded_joystick_names": [],
    "timestamp": None
}

ROBOT_ACTIONS = {}

LOG_FILE       = "/host_docs/controller_status.log"
RECORDINGS_DIR = "/host_docs/recordings"

_opencr_lock        = threading.Lock()
LATEST_OPENCR_STATE = None

_led_lock        = threading.Lock()
LATEST_LED_STATE = None

ROBOT_ACTIVE = False

RECORDING_STATE = {
    "active":     False,
    "user_name":  None,
    "topics":     [],
    "dir":        None,
    "started_at": None,
}

# Sentinel value used as the "topic" key for the LED service recording entry.
# It is not a real ROS2 topic — start_recording() detects it and handles it
# by polling get_led_status() instead of creating a topic subscription.
LED_RECORD_KEY = '__led_service__'

# LED display name maps — used in recording and LED status logging.
_LED_COLOR_NAMES = {0:'BLACK', 1:'WHITE', 2:'GREEN', 3:'RED', 4:'BLUE', 5:'CYAN', 6:'PINK', 7:'YELLOW'}
_LED_MODE_NAMES  = {1:'WAVE_RIGHT', 2:'WAVE_LEFT', 3:'PULSE', 4:'SOLID'}

RECORDABLE_TOPICS = [
    {'topic': '/xbox_controller/button_events',    'msg_type': 'mecanumbot_msgs/msg/ButtonEvent',    'description': 'Button press/release events'},
    {'topic': '/xbox_controller/joystick_events',  'msg_type': 'mecanumbot_msgs/msg/JoystickEvent',  'description': 'Joystick position events'},
    {'topic': '/xbox_controller/connection_status','msg_type': 'mecanumbot_msgs/msg/ControllerStatus','description': 'Controller connection status'},
    {'topic': '/cmd_vel',                          'msg_type': 'geometry_msgs/msg/Twist',          'description': 'Velocity commands sent to robot'},
    {'topic': '/joy',                              'msg_type': 'sensor_msgs/msg/Joy',              'description': 'Raw joystick input from joy_node'},
    {'topic': '/mecanumbot/opencr_state',          'msg_type': 'mecanumbot_msgs/msg/OpenCRState',  'description': 'Raw OpenCR state: wheel vel/pos/curr, servos, IMU, battery'},
    {'topic': '/odom',                             'msg_type': 'nav_msgs/msg/Odometry',            'description': 'Odometry: integrated position + velocity from wheel encoders'},
    {'topic': '/imu',                              'msg_type': 'sensor_msgs/msg/Imu',              'description': 'IMU: orientation, angular velocity, linear acceleration'},
    {'topic': '/joint_states',                     'msg_type': 'sensor_msgs/msg/JointState',       'description': 'Joint states: 4 wheel + head + 2 grabber positions (rad)'},
    {'topic': '/battery_state',                    'msg_type': 'sensor_msgs/msg/BatteryState',     'description': 'Battery voltage and charge percentage'},
    # ── Service-based entry (not a real ROS2 topic) ──────────────────────────
    {'topic': LED_RECORD_KEY,                      'msg_type': 'service',                          'description': 'LED corner state (polled via set_led_status service)', 'is_service': True},
]

_REC_MSG_CLASSES = {
    'std_msgs/msg/String':     String,
    'geometry_msgs/msg/Twist': Twist,
    'sensor_msgs/msg/Joy':     Joy,
}

try:
    from nav_msgs.msg import Odometry
    _REC_MSG_CLASSES['nav_msgs/msg/Odometry'] = Odometry
except ImportError:
    pass

try:
    from sensor_msgs.msg import Imu, JointState, BatteryState
    _REC_MSG_CLASSES['sensor_msgs/msg/Imu']          = Imu
    _REC_MSG_CLASSES['sensor_msgs/msg/JointState']   = JointState
    _REC_MSG_CLASSES['sensor_msgs/msg/BatteryState'] = BatteryState
except ImportError:
    pass

if _OPENCR_MSG_AVAILABLE:
    _REC_MSG_CLASSES['mecanumbot_msgs/msg/OpenCRState'] = OpenCRState

if SYNC_SRV_AVAILABLE:
    _REC_MSG_CLASSES['mecanumbot_msgs/msg/ButtonEvent']     = ButtonEvent
    _REC_MSG_CLASSES['mecanumbot_msgs/msg/JoystickEvent']   = JoystickEvent
    _REC_MSG_CLASSES['mecanumbot_msgs/msg/ControllerStatus'] = ControllerStatus


class DockerNode(Node):
    """
    ROS2 node running inside Docker / on the host machine.

    Two responsibilities:
    1. Subscribe to /xbox_controller/connection_status and update
       LATEST_CONTROLLER_STATUS in-process for the Flask UI to display.
    2. On startup, read all actions from the host database and push them to
       the robot via the /mecanumbot/sync_actions ROS2 service.
       The service response is stored in LATEST_SYNC_RESULT so the Flask UI
       can report conflicts to the user.
    """

    def __init__(self, db):
        super().__init__('docker_node')
        self._db = db
        os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)

        if SYNC_SRV_AVAILABLE:
            self.create_subscription(
                ControllerStatus,
                '/xbox_controller/connection_status',
                self._status_callback,
                10
            )
        else:
            self.create_subscription(
                String,
                '/xbox_controller/connection_status',
                self._status_callback_str,
                10
            )
        self.get_logger().info(
            'Docker Node started — subscribed to /xbox_controller/connection_status')
        self._write_log("=== Docker Node Started ===")
        self._write_log(f"Start time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")

        self._initial_sync_timer = self.create_timer(3.0, self._initial_sync)
        self._sync_done = False

        self._last_opencr_time   = None
        self._robot_sync_lock    = threading.Lock()
        self._robot_gone_timeout = 8.0
        self.create_timer(5.0, self._robot_watchdog)

        if SYNC_SRV_AVAILABLE:
            self._register_client    = self.create_client(RegisterAction, '/mecanumbot/register_action')
            self._sync_client        = self.create_client(SyncActions,    '/mecanumbot/sync_actions')
            self._sync_mappings_client = self.create_client(SyncMappings, '/mecanumbot/sync_mappings')
        else:
            self._register_client      = None
            self._sync_client          = None
            self._sync_mappings_client = None

        if LED_SRV_AVAILABLE:
            self._led_set_client = self.create_client(SetLedStatus, 'mecanumbot/set_led_status')
            self._led_get_client = self.create_client(GetLedStatus, 'mecanumbot/get_led_status')
        else:
            self._led_set_client = None
            self._led_get_client = None

        self._rec_subs     = {}
        self._rec_files    = {}
        self._rec_file     = None
        self._rec_led_file = None   # distinct file for LED state when selected

        if _OPENCR_MSG_AVAILABLE:
            self.create_subscription(OpenCRState, 'mecanumbot/opencr_state', self._opencr_callback, 10)

    # ── Controller status ─────────────────────────────────────────────────────

    def _status_callback(self, msg):
        """Callback for typed ControllerStatus messages."""
        global LATEST_CONTROLLER_STATUS
        try:
            LATEST_CONTROLLER_STATUS = {
                "connected":             msg.connected,
                "controller_type":       msg.controller_type,
                "time_since_last_input": msg.time_since_last_input,
                "timestamp":             msg.timestamp,
                "last_updated":          datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            }
            ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            self._write_log(
                f"[{ts}] Connected: {LATEST_CONTROLLER_STATUS['connected']} | "
                f"Type: {LATEST_CONTROLLER_STATUS['controller_type']}"
            )
        except Exception as e:
            self.get_logger().error(f'Status callback error: {e}')

    def _status_callback_str(self, msg):
        """Fallback: parse connection status from a JSON String message."""
        global LATEST_CONTROLLER_STATUS
        try:
            data = json.loads(msg.data)
            LATEST_CONTROLLER_STATUS = {
                "connected":             data.get('connected', False),
                "controller_type":       data.get('controller_type', 'Unknown'),
                "time_since_last_input": data.get('time_since_last_input', 0),
                "timestamp":             data.get('timestamp', 0),
                "last_updated":          datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            }
        except Exception as e:
            self.get_logger().error(f'Status callback (str) error: {e}')

    def _write_log(self, message):
        try:
            with open(LOG_FILE, 'a') as f:
                f.write(message + '\n')
                f.flush()
        except Exception as e:
            self.get_logger().error(f'Log write failed: {e}')

    # ── OpenCR State ──────────────────────────────────────────────────────────

    def _opencr_callback(self, msg):
        """Store the latest OpenCR state as a plain dict for the Flask API.
        Also detects when the robot comes (back) online and triggers a re-sync."""
        global LATEST_OPENCR_STATE, ROBOT_ACTIVE
        now = time.monotonic()
        self._last_opencr_time = now

        if not ROBOT_ACTIVE:
            ROBOT_ACTIVE = True
            self.get_logger().info(
                'Robot came online (/opencr_state received). Triggering action re-sync …')
            if self._robot_sync_lock.acquire(blocking=False):
                def _do_resync():
                    try:
                        self.sync_actions_to_robot()
                        self.sync_mappings_to_robot()
                    finally:
                        self._robot_sync_lock.release()
                threading.Thread(target=_do_resync, daemon=True).start()

        try:
            state = {
                'vel_bl': msg.vel_bl, 'vel_br': msg.vel_br,
                'vel_fl': msg.vel_fl, 'vel_fr': msg.vel_fr,
                'pos_bl': msg.pos_bl, 'pos_br': msg.pos_br,
                'pos_fl': msg.pos_fl, 'pos_fr': msg.pos_fr,
                'curr_bl': msg.curr_bl, 'curr_br': msg.curr_br,
                'curr_fl': msg.curr_fl, 'curr_fr': msg.curr_fr,
                'pos_n': msg.pos_n, 'pos_gl': msg.pos_gl, 'pos_gr': msg.pos_gr,
                'battery_voltage': round(msg.battery_voltage, 3),
                'imu_angular_vel_x': round(msg.imu_angular_vel_x, 4),
                'imu_angular_vel_y': round(msg.imu_angular_vel_y, 4),
                'imu_angular_vel_z': round(msg.imu_angular_vel_z, 4),
                'imu_linear_acc_x': round(msg.imu_linear_acc_x, 4),
                'imu_linear_acc_y': round(msg.imu_linear_acc_y, 4),
                'imu_linear_acc_z': round(msg.imu_linear_acc_z, 4),
                'imu_orientation_w': round(msg.imu_orientation_w, 5),
                'imu_orientation_x': round(msg.imu_orientation_x, 5),
                'imu_orientation_y': round(msg.imu_orientation_y, 5),
                'imu_orientation_z': round(msg.imu_orientation_z, 5),
            }
            with _opencr_lock:
                LATEST_OPENCR_STATE = state
        except Exception as e:
            self.get_logger().error(f'OpenCR state callback error: {e}')

    # ── Recording ─────────────────────────────────────────────────────────────

    def start_recording(self, user_name: str, topics: list) -> str:
        global RECORDING_STATE

        if RECORDING_STATE['active']:
            self.stop_recording()

        ts        = datetime.now().strftime('%Y%m%d_%H%M%S')
        safe_user = "".join(c if c.isalnum() or c in '-_' else '_' for c in user_name)
        session_dir = os.path.join(RECORDINGS_DIR, f"{ts}_{safe_user}")

        try:
            os.makedirs(session_dir, exist_ok=True)
        except Exception as e:
            self.get_logger().error(f'Cannot create session directory: {e}')
            return ''

        session_file_path = os.path.join(session_dir, '_session.txt')
        try:
            sf = open(session_file_path, 'w', encoding='utf-8')
        except Exception as e:
            self.get_logger().error(f'Cannot open session file: {e}')
            return ''

        sf.write(f"=== MecanumBot Recording Session ===\n")
        sf.write(f"User      : {user_name}\n")
        sf.write(f"Started   : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        sf.write(f"Topics    : {', '.join(topics)}\n")
        sf.write(f"{'=' * 40}\n\n")

        try:
            all_actions       = self._db.get_all_actions()
            button_mappings   = self._db.get_all_button_mappings()
            joystick_mappings = self._db.get_all_joystick_mappings()
        except Exception as e:
            all_actions = {}
            button_mappings = {}
            joystick_mappings = {}
            sf.write(f"[WARNING] Could not read DB snapshot: {e}\n\n")

        sf.write(f"--- Actions ({len(all_actions)}) ---\n")
        for name, data in all_actions.items():
            sf.write(f"  [{data.get('type', '?')}] {name}\n")
            for t in data.get('tuples', []):
                pub_type = t[7] if len(t) > 7 else 'topic'
                svc_name = t[8] if len(t) > 8 else ''
                if pub_type == 'service':
                    sf.write(f"    service={svc_name}  msg={t[1]}\n")
                else:
                    sf.write(
                        f"    topic={t[0]}  msg={t[1]}"
                        f"  type={t[2] if len(t) > 2 else 'std_msgs/msg/String'}"
                        f"  offset=({t[5] if len(t) > 5 else 0.0}, {t[6] if len(t) > 6 else 0.0})\n"
                    )
        sf.write("\n")

        sf.write(f"--- Button Mappings ({len(button_mappings)}) ---\n")
        for btn, mapping in button_mappings.items():
            sf.write(f"  {btn} → {mapping.get('action', '?')}  (mode={mapping.get('trigger_mode', '?')})\n")
        sf.write("\n")

        sf.write(f"--- Joystick Mappings ({len(joystick_mappings)}) ---\n")
        for joy, action in joystick_mappings.items():
            sf.write(f"  {joy} → {action}\n")
        sf.write("\n")
        sf.flush()

        self._rec_files    = {}
        self._rec_file     = sf
        self._rec_subs     = {}
        self._rec_led_file = None

        for entry in RECORDABLE_TOPICS:
            topic = entry['topic']
            if topic not in topics:
                continue

            # ── LED service entry — not a real ROS2 topic ─────────────────────
            if topic == LED_RECORD_KEY:
                led_path = os.path.join(session_dir, 'led_state.txt')
                try:
                    lf = open(led_path, 'w', encoding='utf-8')
                    lf.write(f"# source   : set_led_status service (polled on each LED query)\n")
                    lf.write(f"# started  : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                    lf.write(f"# {'─' * 36}\n")
                    # Write initial snapshot if we already have a cached state
                    with _led_lock:
                        snap = LATEST_LED_STATE
                    if snap:
                        ts_now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                        lf.write(f"[{ts_now}] INITIAL STATE\n")
                        for corner in ['FL', 'FR', 'BL', 'BR']:
                            d = snap.get(corner, {})
                            cname = _LED_COLOR_NAMES.get(d.get('color'), str(d.get('color', '?')))
                            mname = _LED_MODE_NAMES.get(d.get('mode'),   str(d.get('mode',  '?')))
                            lf.write(f"    {corner}: color={cname}  mode={mname}\n")
                        lf.write("\n")
                    lf.flush()
                    self._rec_led_file = lf
                    self.get_logger().info('Recording: LED state file opened → led_state.txt')
                except Exception as e:
                    self.get_logger().error(f'Cannot open LED state file: {e}')
                continue

            # ── Normal ROS2 topic ─────────────────────────────────────────────
            safe_topic = topic.lstrip('/').replace('/', '_')
            topic_path = os.path.join(session_dir, f"{safe_topic}.txt")
            try:
                tf = open(topic_path, 'w', encoding='utf-8')
                tf.write(f"# topic    : {topic}\n")
                tf.write(f"# msg_type : {entry['msg_type']}\n")
                tf.write(f"# started  : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                tf.write(f"# {'─' * 36}\n")
                tf.flush()
                self._rec_files[topic] = tf
            except Exception as e:
                self.get_logger().error(f'Cannot open topic file {topic_path}: {e}')
                continue

            msg_class = _REC_MSG_CLASSES.get(entry['msg_type'])
            if msg_class is None:
                self.get_logger().warn(
                    f'Recording: no message class for {entry["msg_type"]} — skipping {topic}')
                self._rec_files.pop(topic, None)
                continue
            sub = self.create_subscription(msg_class, topic, self._make_rec_callback(topic), 10)
            self._rec_subs[topic] = sub
            self.get_logger().info(
                f'Recording: subscribed to {topic} [{entry["msg_type"]}] → {safe_topic}.txt')

        RECORDING_STATE = {
            'active':     True,
            'user_name':  user_name,
            'topics':     list(topics),
            'dir':        session_dir,
            'started_at': datetime.now().isoformat(),
        }
        self.get_logger().info(f'Recording started → {session_dir}/')
        return session_dir

    def stop_recording(self):
        global RECORDING_STATE

        for topic, sub in self._rec_subs.items():
            self.destroy_subscription(sub)
            self.get_logger().info(f'Recording: unsubscribed from {topic}')
        self._rec_subs = {}

        for topic, tf in self._rec_files.items():
            try:
                tf.write(f"# stopped  : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                tf.close()
            except Exception:
                pass
        self._rec_files = {}

        if self._rec_file:
            try:
                self._rec_file.write(
                    f"{'=' * 40}\n"
                    f"Stopped   : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
                )
                self._rec_file.close()
            except Exception:
                pass
            self._rec_file = None

        if self._rec_led_file:
            try:
                self._rec_led_file.write(
                    f"# stopped  : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
                )
                self._rec_led_file.close()
            except Exception:
                pass
            self._rec_led_file = None

        RECORDING_STATE = {
            'active':     False,
            'user_name':  RECORDING_STATE.get('user_name'),
            'topics':     [],
            'dir':        RECORDING_STATE.get('dir'),
            'started_at': RECORDING_STATE.get('started_at'),
        }
        self.get_logger().info('Recording stopped')

    def record_new_action(self, name: str, action_type: str, tuples: list):
        if not self._rec_file:
            return
        try:
            ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            self._rec_file.write(f"[{ts}] NEW ACTION DEFINED: [{action_type}] {name}\n")
            for t in tuples:
                pub_type = t[7] if len(t) > 7 else 'topic'
                svc_name = t[8] if len(t) > 8 else ''
                if pub_type == 'service':
                    self._rec_file.write(
                        f"    service={svc_name}  msg={t[1]}\n"
                    )
                else:
                    self._rec_file.write(
                        f"    topic={t[0]}  msg={t[1]}"
                        f"  type={t[2] if len(t) > 2 else 'std_msgs/msg/String'}"
                        f"  offset=({t[5] if len(t) > 5 else 0.0}, {t[6] if len(t) > 6 else 0.0})\n"
                    )
            self._rec_file.write("\n")
            self._rec_file.flush()
        except Exception as e:
            self.get_logger().error(f'record_new_action write error: {e}')

    def record_led_command(self, values: dict, success: bool, message: str):
        if not self._rec_file:
            return
        try:
            ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            self._rec_file.write(f"[{ts}] LED COMMAND  success={success}  message={message!r}\n")
            for corner in ['FL', 'FR', 'BL', 'BR']:
                d = values.get(corner, {})
                cname = _LED_COLOR_NAMES.get(d.get('color'), str(d.get('color', '?')))
                mname = _LED_MODE_NAMES.get(d.get('mode'),   str(d.get('mode',  '?')))
                self._rec_file.write(f"    {corner}: color={cname}  mode={mname}\n")
            self._rec_file.write("\n")
            self._rec_file.flush()
        except Exception as e:
            self.get_logger().error(f'record_led_command write error: {e}')

    def record_sync_result(self, actions_sent: list, result: dict):
        if not self._rec_file:
            return
        try:
            ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            self._rec_file.write(f"[{ts}] SYNC TO ROBOT\n")
            self._rec_file.write(f"    Actions sent : {len(actions_sent)}\n")
            self._rec_file.write(f"    Success      : {result.get('success')}\n")
            self._rec_file.write(f"    Accepted     : {result.get('accepted_count', 0)}\n")
            conflicts = result.get('conflicts', [])
            if conflicts:
                self._rec_file.write(f"    Conflicts    : {len(conflicts)}\n")
                for c in conflicts:
                    self._rec_file.write(
                        f"      - {c.get('name', '?')} [{c.get('category', '?')}]: {c.get('reason', '?')}\n")
            else:
                self._rec_file.write(f"    Conflicts    : none\n")
            self._rec_file.write(f"    Actions sent detail:\n")
            for a in actions_sent:
                # actions_sent is a list of ActionDescriptor ROS2 message objects
                self._rec_file.write(f"      [{a.action_type}] {a.name}\n")
                for t in a.tuples:
                    if t.publish_type == 'service':
                        self._rec_file.write(
                            f"        service={t.service_name}  msg={t.message}\n"
                        )
                    else:
                        self._rec_file.write(
                            f"        topic={t.topic}  msg={t.message}"
                            f"  type={t.message_type}"
                            f"  scale=({t.scale_x}, {t.scale_y})"
                            f"  offset=({t.offset_x}, {t.offset_y})\n"
                        )
            self._rec_file.write("\n")
            self._rec_file.flush()
        except Exception as e:
            self.get_logger().error(f'record_sync_result write error: {e}')

    def _make_rec_callback(self, topic: str):
        def _callback(msg):
            tf = self._rec_files.get(topic)
            if not tf:
                return
            ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            try:
                if _MSG_TO_DICT_AVAILABLE:
                    data = json.dumps(message_to_ordereddict(msg))
                elif hasattr(msg, 'data'):
                    data = str(msg.data)
                else:
                    data = str(msg)
            except Exception:
                data = str(msg)
            try:
                tf.write(f"[{ts}] {data}\n")
                tf.flush()
            except Exception as e:
                self.get_logger().error(f'Recording write error ({topic}): {e}')
        return _callback

    # ── Action sync ───────────────────────────────────────────────────────────

    def _initial_sync(self):
        if self._sync_done:
            return
        if not SYNC_SRV_AVAILABLE:
            self._sync_done = True
            self._initial_sync_timer.cancel()
            return
        self._sync_done = True
        self._initial_sync_timer.cancel()
        self.get_logger().info('Running initial action sync to robot …')
        if not self._robot_sync_lock.acquire(blocking=False):
            self.get_logger().info(
                '_initial_sync: sync already in progress (robot came online simultaneously) — skipping')
            return
        # Run sync in a plain thread so the executor stays free to process the response
        def _do_sync():
            try:
                self.sync_actions_to_robot()
                self.sync_mappings_to_robot()
            finally:
                self._robot_sync_lock.release()
        threading.Thread(target=_do_sync, daemon=True).start()

    def _robot_watchdog(self):
        global ROBOT_ACTIVE
        if self._last_opencr_time is None:
            return
        elapsed = time.monotonic() - self._last_opencr_time
        if elapsed > self._robot_gone_timeout and ROBOT_ACTIVE:
            ROBOT_ACTIVE = False
            self.get_logger().warn(
                f'Robot went offline (no /opencr_state for {elapsed:.1f} s). '
                'Will re-sync when it comes back.')

    def sync_actions_to_robot(self):
        global LATEST_SYNC_RESULT, ROBOT_ACTIONS

        if not SYNC_SRV_AVAILABLE:
            self.get_logger().warn('SyncActions service type not available — skipping sync')
            LATEST_SYNC_RESULT = {
                "attempted": True, "success": False, "accepted_count": 0,
                "conflicts": [{"name": "", "category": "",
                               "reason": "mecanumbot_msgs not installed in Docker"}],
                "timestamp": datetime.now().isoformat()
            }
            return

        try:
            all_actions = self._db.get_all_actions()
        except Exception as e:
            self.get_logger().error(f'Failed to read host DB: {e}')
            LATEST_SYNC_RESULT = {
                "attempted": True, "success": False, "accepted_count": 0,
                "conflicts": [{"name": "", "category": "",
                               "reason": f"Host DB read error: {e}"}],
                "timestamp": datetime.now().isoformat()
            }
            return

        if not all_actions:
            self.get_logger().info('No actions in host DB — nothing to sync')
            # Don't mark as "attempted" — the robot was never contacted.
            # Badges should stay "⊘ Not synced" rather than "✓ Synced".
            LATEST_SYNC_RESULT = {
                "attempted": False, "success": None,
                "accepted_count": 0, "conflicts": [],
                "timestamp": datetime.now().isoformat()
            }
            self.record_sync_result([], LATEST_SYNC_RESULT)
            return

        actions_list = []
        for name, data in all_actions.items():
            desc             = ActionDescriptor()
            desc.name        = name
            desc.action_type = data.get("type", "button_once")
            desc.tuples      = []
            for t in data.get("tuples", []):
                at              = ActionTuple()
                at.topic        = t[0]
                at.message      = t[1]
                at.message_type = t[2] if len(t) > 2 else "std_msgs/msg/String"
                at.scale_x      = float(t[3]) if len(t) > 3 else 1.0
                at.scale_y      = float(t[4]) if len(t) > 4 else 1.0
                at.offset_x     = float(t[5]) if len(t) > 5 else 0.0
                at.offset_y     = float(t[6]) if len(t) > 6 else 0.0
                at.publish_type = t[7] if len(t) > 7 else "topic"
                at.service_name = t[8] if len(t) > 8 else ""
                desc.tuples.append(at)
            actions_list.append(desc)

        self.get_logger().info(
            f'Calling /mecanumbot/sync_actions with {len(actions_list)} actions …')

        req         = SyncActions.Request()
        req.actions = actions_list
        future = self._sync_client.call_async(req)

        elapsed = 0.0
        timeout = 15.0
        poll    = 0.05
        while not future.done():
            if elapsed >= timeout:
                self.get_logger().warn('SyncActions timed out — robot not running or mecanumbot_msgs not built there')
                LATEST_SYNC_RESULT = {
                    "attempted": True, "success": False, "accepted_count": 0,
                    "conflicts": [{"name": "", "category": "",
                                   "reason": "Service /mecanumbot/sync_actions not reachable. "
                                             "Is the robot running and is mecanumbot_msgs built?"}],
                    "timestamp": datetime.now().isoformat()
                }
                self.record_sync_result(actions_list, LATEST_SYNC_RESULT)
                return
            time.sleep(poll)
            elapsed += poll

        if future.result() is None:
            self.get_logger().error('SyncActions call timed out or failed')
            LATEST_SYNC_RESULT = {
                "attempted": True, "success": False, "accepted_count": 0,
                "conflicts": [{"name": "", "category": "", "reason": "Service call timed out"}],
                "timestamp": datetime.now().isoformat()
            }
            self.record_sync_result(actions_list, LATEST_SYNC_RESULT)
            return

        result = future.result()
        conflicts = [
            {
                "name":     result.conflict_names[i],
                "category": "",
                "reason":   result.conflict_reasons[i] if i < len(result.conflict_reasons) else ''
            }
            for i in range(len(result.conflict_names))
        ]

        ROBOT_ACTIONS = {
            desc.name: {
                'type':   desc.action_type,
                'tuples': [(t.topic, t.message, t.message_type)
                           for t in desc.tuples]
            }
            for desc in result.robot_actions
        }

        LATEST_SYNC_RESULT = {
            "attempted":      True,
            "success":        result.success,
            "accepted_count": result.accepted_count,
            "conflicts":      conflicts,
            "timestamp":      datetime.now().isoformat()
        }
        self.record_sync_result(actions_list, LATEST_SYNC_RESULT)

        if result.success:
            self.get_logger().info(f'Sync OK — {result.accepted_count} actions accepted by robot')
        else:
            self.get_logger().warn(
                f'Sync completed with {len(conflicts)} conflict(s): '
                + ', '.join(c["name"] for c in conflicts))

    def sync_mappings_to_robot(self):
        global LATEST_MAPPING_SYNC_RESULT

        if not SYNC_SRV_AVAILABLE:
            self.get_logger().warn('SyncMappings service type not available — skipping mapping sync')
            LATEST_MAPPING_SYNC_RESULT = {
                "attempted": True, "success": False,
                "loaded_button_count": 0, "loaded_joystick_count": 0,
                "loaded_button_names": [], "loaded_joystick_names": [],
                "timestamp": datetime.now().isoformat()
            }
            return

        try:
            button_mappings   = self._db.get_all_button_mappings()
            joystick_mappings = self._db.get_all_joystick_mappings()
        except Exception as e:
            self.get_logger().error(f'SyncMappings: failed to read host DB: {e}')
            LATEST_MAPPING_SYNC_RESULT = {
                "attempted": True, "success": False,
                "loaded_button_count": 0, "loaded_joystick_count": 0,
                "loaded_button_names": [], "loaded_joystick_names": [],
                "timestamp": datetime.now().isoformat()
            }
            return

        req = SyncMappings.Request()
        req.button_names   = list(button_mappings.keys())
        req.action_names   = [v['action']       for v in button_mappings.values()]
        req.trigger_modes  = [v['trigger_mode']  for v in button_mappings.values()]
        req.joystick_names         = list(joystick_mappings.keys())
        req.joystick_action_names  = list(joystick_mappings.values())

        self.get_logger().info(
            f'Calling /mecanumbot/sync_mappings with '
            f'{len(req.button_names)} button mapping(s), '
            f'{len(req.joystick_names)} joystick mapping(s) …')

        future = self._sync_mappings_client.call_async(req)

        elapsed = 0.0
        timeout = 15.0
        poll    = 0.05
        while not future.done():
            if elapsed >= timeout:
                self.get_logger().warn('SyncMappings timed out — robot offline?')
                LATEST_MAPPING_SYNC_RESULT = {
                    "attempted": True, "success": False,
                    "loaded_button_count": 0, "loaded_joystick_count": 0,
                    "loaded_button_names": [], "loaded_joystick_names": [],
                    "timestamp": datetime.now().isoformat()
                }
                return
            time.sleep(poll)
            elapsed += poll

        result = future.result()
        if result is None:
            self.get_logger().error('SyncMappings call returned None')
            LATEST_MAPPING_SYNC_RESULT = {
                "attempted": True, "success": False,
                "loaded_button_count": 0, "loaded_joystick_count": 0,
                "loaded_button_names": [], "loaded_joystick_names": [],
                "timestamp": datetime.now().isoformat()
            }
            return

        LATEST_MAPPING_SYNC_RESULT = {
            "attempted":            True,
            "success":              result.success,
            "loaded_button_count":  result.loaded_button_count,
            "loaded_joystick_count": result.loaded_joystick_count,
            "loaded_button_names":  list(result.loaded_button_names),
            "loaded_joystick_names": list(result.loaded_joystick_names),
            "timestamp":            datetime.now().isoformat()
        }

        self.get_logger().info(
            f'SyncMappings OK — {result.loaded_button_count} button(s), '
            f'{result.loaded_joystick_count} joystick(s) loaded on robot')

    def register_action_with_robot(self, name, action_type, tuples):
        if not SYNC_SRV_AVAILABLE:
            self.get_logger().warn('RegisterAction: mecanumbot_msgs not available — cannot validate')
            return False, 'Robot interface (mecanumbot_msgs) is not available in this environment.'

        desc             = ActionDescriptor()
        desc.name        = name
        desc.action_type = action_type
        desc.tuples      = []
        for t in tuples:
            at              = ActionTuple()
            at.topic        = t[0]
            at.message      = t[1]
            at.message_type = t[2] if len(t) > 2 else 'std_msgs/msg/String'
            at.scale_x      = float(t[3]) if len(t) > 3 else 1.0
            at.scale_y      = float(t[4]) if len(t) > 4 else 1.0
            at.offset_x     = float(t[5]) if len(t) > 5 else 0.0
            at.offset_y     = float(t[6]) if len(t) > 6 else 0.0
            at.publish_type = t[7] if len(t) > 7 else 'topic'
            at.service_name = t[8] if len(t) > 8 else ''
            desc.tuples.append(at)

        req        = RegisterAction.Request()
        req.action = desc
        future = self._register_client.call_async(req)

        timeout = 10.0
        elapsed = 0.0
        poll    = 0.05
        while not future.done():
            if elapsed >= timeout:
                self.get_logger().error('RegisterAction timed out — robot offline?')
                return False, 'Robot did not respond. Is the robot node running?'
            time.sleep(poll)
            elapsed += poll

        result = future.result()
        if result is None:
            return False, 'Robot did not respond. Is the robot node running?'

        if result.success:
            self.get_logger().info(f'RegisterAction: "{name}" accepted by robot')
        else:
            self.get_logger().warn(f'RegisterAction: "{name}" rejected — {result.conflict}')
        return result.success, result.conflict

    # ── LED control ───────────────────────────────────────────────────────────

    def get_led_status(self):
        if not LED_SRV_AVAILABLE or self._led_get_client is None:
            return None, 'LED service not available (mecanumbot_msgs not built)'
        req = GetLedStatus.Request()
        future = self._led_get_client.call_async(req)
        elapsed, timeout, poll = 0.0, 8.0, 0.05
        while not future.done():
            if elapsed >= timeout:
                return None, 'get_led_status timed out — robot offline?'
            time.sleep(poll)
            elapsed += poll
        result = future.result()
        if result is None:
            return None, 'get_led_status call returned None'
        state = {
            'FL': {'mode': result.fl_mode, 'color': result.fl_color},
            'FR': {'mode': result.fr_mode, 'color': result.fr_color},
            'BL': {'mode': result.bl_mode, 'color': result.bl_color},
            'BR': {'mode': result.br_mode, 'color': result.br_color},
        }
        # If LED recording is active, write a timestamped snapshot to led_state.txt
        if self._rec_led_file:
            try:
                ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                self._rec_led_file.write(f"[{ts}]\n")
                for corner in ['FL', 'FR', 'BL', 'BR']:
                    d = state[corner]
                    cname = _LED_COLOR_NAMES.get(d['color'], str(d['color']))
                    mname = _LED_MODE_NAMES.get(d['mode'],   str(d['mode']))
                    self._rec_led_file.write(f"    {corner}: color={cname}  mode={mname}\n")
                self._rec_led_file.write("\n")
                self._rec_led_file.flush()
            except Exception as e:
                self.get_logger().error(f'LED recording write error: {e}')
        return state, ''

    def set_led_status(self, values):
        if not LED_SRV_AVAILABLE or self._led_set_client is None:
            return False, 'LED service not available (mecanumbot_msgs not built)'
        req = SetLedStatus.Request()
        req.fl_mode  = int(values['FL']['mode'])
        req.fl_color = int(values['FL']['color'])
        req.fr_mode  = int(values['FR']['mode'])
        req.fr_color = int(values['FR']['color'])
        req.bl_mode  = int(values['BL']['mode'])
        req.bl_color = int(values['BL']['color'])
        req.br_mode  = int(values['BR']['mode'])
        req.br_color = int(values['BR']['color'])
        future = self._led_set_client.call_async(req)
        elapsed, timeout, poll = 0.0, 8.0, 0.05
        while not future.done():
            if elapsed >= timeout:
                return False, 'set_led_status timed out — robot offline?'
            time.sleep(poll)
            elapsed += poll
        result = future.result()
        if result is None:
            return False, 'set_led_status call returned None'
        msg = getattr(result, 'message', '') or ('OK' if result.success else 'Service returned failure')
        return result.success, msg


_docker_node_instance = None  # set once the thread creates the node


def _start_docker_node(db):
    """Spin the DockerNode in a background daemon thread using a MultiThreadedExecutor."""
    global _docker_node_instance
    try:
        rclpy.init()
        node = DockerNode(db)
        _docker_node_instance = node
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
        node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(f"[DockerNode] Failed to start: {e}")
