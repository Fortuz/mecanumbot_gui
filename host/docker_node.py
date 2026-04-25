import os
import time
import json
import logging
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

try:
    from rosidl_runtime_py import message_to_ordereddict
    _MSG_TO_DICT_AVAILABLE = True
except ImportError:
    _MSG_TO_DICT_AVAILABLE = False

from mecanumbot_msgs.srv import GetRobotActions
from mecanumbot_msgs.srv import SetLedStatus, GetLedStatus
from mecanumbot_msgs.srv import (
    GetMappingNames, GetMappingDetails,
    SaveMapping, DeleteRobotMapping, ApplyMapping,
    SaveRobotAction, DeleteRobotAction,
)
from mecanumbot_msgs.srv import (
    GetRecordingSchemes, SaveRecordingScheme, DeleteRecordingScheme,
    GetActionUsages,
)
from mecanumbot_msgs.msg import (ActionTuple, ActionDescriptor,
                                 ControllerStatus, ButtonEvent, JoystickEvent,
                                 OpenCRState)

# ──────────────────────────────────────────────────────────────
# Module logger (used outside the ROS2 node, e.g. _start_docker_node)
# ──────────────────────────────────────────────────────────────
_log = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s')

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

# ── Current user — set when the user submits the landing-page form ────────────
CURRENT_USER = {"name": None, "id": None}

def set_current_user(name: str, user_id: int):
    """Called by app.py /save route once the DB user row is resolved."""
    global CURRENT_USER
    CURRENT_USER["name"]   = name
    CURRENT_USER["id"]     = user_id

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

_REC_MSG_CLASSES['mecanumbot_msgs/msg/OpenCRState']      = OpenCRState
_REC_MSG_CLASSES['mecanumbot_msgs/msg/ButtonEvent']      = ButtonEvent
_REC_MSG_CLASSES['mecanumbot_msgs/msg/JoystickEvent']    = JoystickEvent
_REC_MSG_CLASSES['mecanumbot_msgs/msg/ControllerStatus'] = ControllerStatus


class DockerNode(Node):
    """
    ROS2 node running inside Docker / on the host machine.

    Two responsibilities:
    1. Subscribe to /xbox_controller/connection_status and update
       LATEST_CONTROLLER_STATUS in-process for the Flask UI to display.
    2. Provide service clients to interact with the robot (GetRobotActions,
       SaveRobotAction, ApplyMapping, etc.) for the Flask UI.
    """

    def __init__(self, db):
        super().__init__('docker_node')
        self._db = db
        os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)

        self.create_subscription(
            ControllerStatus,
            '/xbox_controller/connection_status',
            self._status_callback,
            10
        )
        self.get_logger().info(
            'Docker Node started — subscribed to /xbox_controller/connection_status')
        self._write_log("=== Docker Node Started ===")
        self._write_log(f"Start time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")

        self._last_opencr_time   = None
        self._robot_gone_timeout = 8.0
        self.create_timer(5.0, self._robot_watchdog)

        self._get_robot_actions_client = self.create_client(GetRobotActions, '/mecanumbot/get_robot_actions')

        # ── New mapping/action management service clients ─────────────────────
        self._get_mapping_names_client   = self.create_client(GetMappingNames,   '/mecanumbot/get_mapping_names')
        self._get_mapping_details_client = self.create_client(GetMappingDetails, '/mecanumbot/get_mapping_details')
        self._save_mapping_client    = self.create_client(SaveMapping,        '/mecanumbot/save_mapping')
        self._del_mapping_client     = self.create_client(DeleteRobotMapping, '/mecanumbot/delete_mapping')
        self._apply_mapping_client   = self.create_client(ApplyMapping,       '/mecanumbot/apply_mapping')
        self._save_action_client     = self.create_client(SaveRobotAction,    '/mecanumbot/save_action')
        self._del_action_client      = self.create_client(DeleteRobotAction,  '/mecanumbot/delete_action')

        self._get_schemes_client = self.create_client(GetRecordingSchemes,   '/mecanumbot/get_recording_schemes')
        self._save_scheme_client = self.create_client(SaveRecordingScheme,   '/mecanumbot/save_recording_scheme')
        self._del_scheme_client  = self.create_client(DeleteRecordingScheme, '/mecanumbot/delete_recording_scheme')
        self._get_action_usages_client = self.create_client(GetActionUsages, '/mecanumbot/get_action_usages')

        self._led_set_client = self.create_client(SetLedStatus, '/set_led_status')
        self._led_get_client = self.create_client(GetLedStatus, '/get_led_status')
        self._rec_subs     = {}
        self._rec_files    = {}
        self._rec_file     = None
        self._rec_led_file = None   # distinct file for LED state when selected

        # ── Thread-safe subscription queue ────────────────────────────────────
        # create_subscription / destroy_subscription must be called from the
        # executor thread, not from Flask request threads.  We enqueue work
        # here and flush it on the executor thread via a fast timer.
        self._sub_create_queue  = []   # list of (msg_class, topic, callback)
        self._sub_destroy_queue = []   # list of Subscription objects
        self._sub_queue_lock    = threading.Lock()
        self.create_timer(0.05, self._flush_sub_queue)   # 50 ms flush interval

        # Executor reference — set by _start_docker_node after creation.
        self._executor = None

        self.create_subscription(OpenCRState, '/opencr_state', self._opencr_callback, 10)

    # ── Subscription queue flush (runs on executor thread) ───────────────────

    def _flush_sub_queue(self):
        """Timer callback — safe to create/destroy subscriptions here because
        this runs on the ROS2 executor thread, not a Flask request thread."""
        with self._sub_queue_lock:
            to_create  = list(self._sub_create_queue);  self._sub_create_queue.clear()
            to_destroy = list(self._sub_destroy_queue); self._sub_destroy_queue.clear()

        for sub in to_destroy:
            try:
                self.destroy_subscription(sub)
            except Exception as e:
                self.get_logger().error(f'destroy_subscription error: {e}')

        for msg_class, topic, callback in to_create:
            try:
                sub = self.create_subscription(msg_class, topic, callback, 10)
                self._rec_subs[topic] = sub
                safe = topic.lstrip('/').replace('/', '_')
                self.get_logger().info(f'Recording: subscribed to {topic} → {safe}.txt')
            except Exception as e:
                self.get_logger().error(f'create_subscription error for {topic}: {e}')


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
                'Robot came online (/opencr_state received).')

        try:
            state = {
                'vel_bl': msg.vel_bl, 'vel_br': msg.vel_br,
                'vel_fl': msg.vel_fl, 'vel_fr': msg.vel_fr,
                'pos_bl': msg.pos_bl, 'pos_br': msg.pos_br,
                'pos_fl': msg.pos_fl, 'pos_fr': msg.pos_fr,
                'curr_bl': msg.curr_bl, 'curr_br': msg.curr_br,
                'curr_fl': msg.curr_fl, 'curr_fr': msg.curr_fr,
                'err_bl': msg.err_bl, 'err_br': msg.err_br,
                'err_fl': msg.err_fl, 'err_fr': msg.err_fr,
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

        sf.write("=== MecanumBot Recording Session ===\n")
        sf.write(f"User      : {user_name}\n")
        sf.write(f"Started   : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        sf.write(f"Topics    : {', '.join(topics)}\n")
        sf.write("=" * 40 + "\n\n")

        try:
            uid = CURRENT_USER.get("id")
            all_actions = self._db.get_all_actions(uid) if uid is not None else {}
            # Snapshot of named mappings (new schema)
            all_mappings = self._db.get_all_mappings(uid) if uid is not None else []
        except Exception as e:
            all_actions = {}
            all_mappings = []
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

        sf.write(f"--- Mappings ({len(all_mappings)}) ---\n")
        for m in all_mappings:
            btns = ', '.join(f"{b['button']}→{b['action']}" for b in m.get('buttons', []))
            joys = ', '.join(f"{j['joystick']}→{j['action']}" for j in m.get('joysticks', []))
            sf.write(f"  [{m.get('name','?')}]  buttons: {btns or 'none'}  joysticks: {joys or 'none'}\n")
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
                    lf.write("# source   : set_led_status service (polled on each LED query)\n")
                    lf.write(f"# started  : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                    lf.write("# " + "─" * 36 + "\n")
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
            # Enqueue — actual create_subscription happens on the executor thread
            with self._sub_queue_lock:
                self._sub_create_queue.append(
                    (msg_class, topic, self._make_rec_callback(topic))
                )

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

        # Enqueue all active subscriptions for destruction on the executor thread
        with self._sub_queue_lock:
            self._sub_destroy_queue.extend(self._rec_subs.values())
        logged = list(self._rec_subs.keys())
        self._rec_subs = {}
        for topic in logged:
            self.get_logger().info(f'Recording: unsubscribed from {topic}')

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

    # ── Robot watchdog ────────────────────────────────────────────────────────

    def _robot_watchdog(self):
        global ROBOT_ACTIVE, LATEST_CONTROLLER_STATUS
        if self._last_opencr_time is None:
            return
        elapsed = time.monotonic() - self._last_opencr_time
        if elapsed > self._robot_gone_timeout and ROBOT_ACTIVE:
            ROBOT_ACTIVE = False
            self.get_logger().warn(
                f'Robot went offline (no /opencr_state for {elapsed:.1f} s). '
                'Will re-sync when it comes back.')
            # Mark the controller as disconnected too — the robot is the
            # only source of controller status, so if the robot is gone the
            # controller status is stale and meaningless.
            LATEST_CONTROLLER_STATUS = {
                **LATEST_CONTROLLER_STATUS,
                "connected":             False,
                "time_since_last_input": elapsed,
                "last_updated":          datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            }

    def get_robot_actions(self, user_name: str) -> tuple:
        """Fetch all actions stored in the robot's local DB for the given user.
        Returns (actions_dict | None, error_str | None).
        """
        req           = GetRobotActions.Request()
        req.user_name = user_name

        result = self._call_service(self._get_robot_actions_client, req, timeout=5.0)
        if result is None:
            return None, 'GetRobotActions service timed out'

        if not result.success:
            return None, result.message or 'Robot returned failure for GetRobotActions'

        actions = {}
        for name, atype, aj in zip(result.action_names, result.action_types, result.actions_json):
            try:
                data = json.loads(aj)
                actions[name] = {'type': atype, 'tuples': data.get('tuples', [])}
            except Exception:
                actions[name] = {'type': atype, 'tuples': []}

        return actions, None

    # ── New mapping/action management services ────────────────────────────────

    def _call_service(self, client, request, timeout=15.0):
        """Generic helper: call a service client and return (result | None).
        Returns None on timeout or exception; both are logged.
        """
        event  = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: event.set())
        # If the future completed before the callback was registered, set manually
        if future.done():
            event.set()
        if not event.wait(timeout=timeout):
            svc = getattr(client, 'srv_name', str(client))
            self.get_logger().warning(
                f'Service call timed out after {timeout:.1f}s: {svc}')
            return None
        try:
            return future.result()
        except Exception:
            svc = getattr(client, 'srv_name', str(client))
            self.get_logger().exception(f'Service call raised an exception: {svc}')
            return None

    def get_robot_mapping_names(self):
        """Return (list_of_name_strings, error_str)."""
        req           = GetMappingNames.Request()
        req.user_name = CURRENT_USER.get("name") or ""
        result = self._call_service(self._get_mapping_names_client, req)
        if result is None:
            return [], 'GetMappingNames timed out — robot offline?'
        if not result.success:
            return [], result.message
        return list(result.mapping_names), ''

    def get_robot_mapping_details(self, mapping_name: str):
        """Return (mapping_dict, error_str).
        mapping_dict: {name, buttons:[{button,action,trigger_mode}], joysticks:[{joystick,action}]}
        """
        req              = GetMappingDetails.Request()
        req.user_name    = CURRENT_USER.get("name") or ""
        req.mapping_name = mapping_name
        result = self._call_service(self._get_mapping_details_client, req)
        if result is None:
            return None, 'GetMappingDetails timed out — robot offline?'
        if not result.success:
            return None, result.message
        try:
            mapping = json.loads(result.mapping_json) if result.mapping_json else {}
        except Exception:
            mapping = {}
        return mapping, ''

    def save_mapping_to_robot(self, mapping_name: str,
                              button_entries: list,
                              joystick_entries: list,
                              actions_dict: dict,
                              overwrite: bool = False):
        """
        Save a named mapping to the robot's DB.

        actions_dict: {name: {type, tuples}} — only actions referenced by
        this mapping that the robot may not already have.
        button_entries:   [(btn, action_name, trigger_mode), ...]
        joystick_entries: [(joy, action_name), ...]
        overwrite: if False and mapping exists, returns conflict error
        Returns (success, message).
        """
        req              = SaveMapping.Request()
        req.user_name    = CURRENT_USER.get("name") or ""
        req.mapping_name = mapping_name
        req.overwrite    = overwrite
        req.button_names         = [e[0] for e in button_entries]
        req.button_action_names  = [e[1] for e in button_entries]
        req.button_trigger_modes = [e[2] for e in button_entries]
        req.joystick_names         = [e[0] for e in joystick_entries]
        req.joystick_action_names  = [e[1] for e in joystick_entries]
        req.actions = self._build_action_descriptors(actions_dict)

        result = self._call_service(self._save_mapping_client, req)
        if result is None:
            return False, 'SaveMapping timed out — robot offline?'
        return result.success, result.message

    def delete_robot_mapping(self, mapping_name: str):
        """Delete a mapping from the robot's DB. Returns (success, message)."""
        req              = DeleteRobotMapping.Request()
        req.user_name    = CURRENT_USER.get("name") or ""
        req.mapping_name = mapping_name
        result           = self._call_service(self._del_mapping_client, req)
        if result is None:
            return False, 'DeleteRobotMapping timed out — robot offline?'
        return result.success, result.message

    def apply_mapping(self, mapping_name: str,
                      from_robot_db: bool,
                      button_entries: list = None,
                      joystick_entries: list = None,
                      actions_dict: dict = None):
        """
        Activate a named mapping on the robot.
        If from_robot_db=True the robot loads it from its own DB.
        Otherwise the host sends the full data.
        Returns (success, message).
        """
        req                = ApplyMapping.Request()
        req.user_name      = CURRENT_USER.get("name") or ""
        req.mapping_name   = mapping_name
        req.from_robot_db  = from_robot_db
        if not from_robot_db:
            req.button_names         = [e[0] for e in (button_entries or [])]
            req.button_action_names  = [e[1] for e in (button_entries or [])]
            req.button_trigger_modes = [e[2] for e in (button_entries or [])]
            req.joystick_names         = [e[0] for e in (joystick_entries or [])]
            req.joystick_action_names  = [e[1] for e in (joystick_entries or [])]
            req.actions = self._build_action_descriptors(actions_dict or {})
        result = self._call_service(self._apply_mapping_client, req)
        if result is None:
            return False, 'ApplyMapping timed out — robot offline?', 0, 0
        return result.success, result.message, result.loaded_button_count, result.loaded_joystick_count

    def save_action_to_robot(self, name: str, action_type: str, tuples: list,
                             overwrite: bool = False):
        """Save a single action to the robot's DB. Returns (success, message)."""
        req           = SaveRobotAction.Request()
        req.user_name = CURRENT_USER.get("name") or ""
        req.action    = self._build_action_descriptor(name, action_type, tuples)
        req.overwrite = overwrite
        result     = self._call_service(self._save_action_client, req)
        if result is None:
            return False, 'SaveRobotAction timed out — robot offline?'
        return result.success, result.message

    def delete_robot_action(self, action_name: str):
        """Delete an action from the robot's DB. Returns (success, message)."""
        req             = DeleteRobotAction.Request()
        req.user_name   = CURRENT_USER.get("name") or ""
        req.action_name = action_name
        result          = self._call_service(self._del_action_client, req)
        if result is None:
            return False, 'DeleteRobotAction timed out — robot offline?'
        return result.success, result.message

    def get_robot_action_usages(self, action_name: str):
        """Return list of mapping names that use the given action on the robot. Returns (list, error_str)."""
        req             = GetActionUsages.Request()
        req.user_name   = CURRENT_USER.get("name") or ""
        req.action_name = action_name
        result          = self._call_service(self._get_action_usages_client, req)
        if result is None:
            return [], 'GetActionUsages timed out — robot offline?'
        if not result.success:
            return [], result.message
        return list(result.mapping_names), ''

    def _build_action_descriptor(self, name, action_type, tuples):
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
        return desc

    def _build_action_descriptors(self, actions_dict: dict):
        descs = []
        for name, data in actions_dict.items():
            descs.append(self._build_action_descriptor(
                name, data.get('type', 'button_once'), data.get('tuples', [])
            ))
        return descs

    # ── Recording scheme services ─────────────────────────────────────────────

    def get_robot_recording_schemes(self, user_name: str):
        """Return ({name: [topics]}, error_str)."""
        req           = GetRecordingSchemes.Request()
        req.user_name = user_name
        result = self._call_service(self._get_schemes_client, req)
        if result is None:
            return None, 'GetRecordingSchemes timed out — robot offline?'
        if not result.success:
            return None, result.message or 'GetRecordingSchemes returned failure'
        schemes = {}
        for name, topics_json in zip(result.scheme_names, result.scheme_topics_json):
            try:
                schemes[name] = json.loads(topics_json)
            except Exception:
                schemes[name] = []
        return schemes, None

    def save_recording_scheme_to_robot(self, scheme_name: str, topics: list,
                                       overwrite: bool = False):
        """Save a recording scheme to the robot's DB. Returns (success, message)."""
        req             = SaveRecordingScheme.Request()
        req.user_name   = CURRENT_USER.get("name") or ""
        req.scheme_name = scheme_name
        req.topics      = list(topics)
        req.overwrite   = overwrite
        result = self._call_service(self._save_scheme_client, req)
        if result is None:
            return False, 'SaveRecordingScheme timed out — robot offline?'
        return result.success, result.message

    def delete_robot_recording_scheme(self, scheme_name: str):
        """Delete a recording scheme from the robot's DB. Returns (success, message)."""
        req             = DeleteRecordingScheme.Request()
        req.user_name   = CURRENT_USER.get("name") or ""
        req.scheme_name = scheme_name
        result = self._call_service(self._del_scheme_client, req)
        if result is None:
            return False, 'DeleteRecordingScheme timed out — robot offline?'
        return result.success, result.message

    # ── LED control ───────────────────────────────────────────────────────────

    def get_led_status(self):
        result = self._call_service(self._led_get_client, GetLedStatus.Request(), timeout=8.0)
        if result is None:
            return None, 'get_led_status timed out — robot offline?'
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
        req = SetLedStatus.Request()
        req.fl_mode  = int(values['FL']['mode'])
        req.fl_color = int(values['FL']['color'])
        req.fr_mode  = int(values['FR']['mode'])
        req.fr_color = int(values['FR']['color'])
        req.bl_mode  = int(values['BL']['mode'])
        req.bl_color = int(values['BL']['color'])
        req.br_mode  = int(values['BR']['mode'])
        req.br_color = int(values['BR']['color'])
        result = self._call_service(self._led_set_client, req, timeout=8.0)
        if result is None:
            return False, 'set_led_status timed out — robot offline?'
        msg = getattr(result, 'message', '') or ('OK' if result.success else 'Service returned failure')
        return result.success, msg


_docker_node_instance = None  # set once the thread creates the node


def _start_docker_node(db):
    """Spin the DockerNode in a background daemon thread using a MultiThreadedExecutor."""
    global _docker_node_instance
    try:
        rclpy.init()
        node     = DockerNode(db)
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        node._executor       = executor   # give the node a back-reference for future use
        _docker_node_instance = node
        executor.spin()
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        _log.exception('[DockerNode] Failed to start')
