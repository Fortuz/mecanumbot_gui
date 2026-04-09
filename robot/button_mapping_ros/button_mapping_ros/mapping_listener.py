#!/usr/bin/env python3
"""
Mapping Listener Node  (robot-side)
====================================

Responsibilities
----------------
1. Owns the **robot's local SQLite database** (~/Documents/actions.db).
   On startup it loads the robot's own actions and mappings from that DB.
   These are the actions defined directly on the robot.

2. Exposes the **/mecanumbot/sync_actions** ROS2 service
   (mecanumbot_msgs/srv/SyncActions).
   The Docker/GUI node calls this service to push the host's actions to the
   robot node.  The received actions are:
     - Checked for name conflicts against the robot's own DB actions.
     - If accepted: stored IN MEMORY ONLY — never written to the robot's DB.
     - The robot's database is never modified by the sync.
   The service returns: success flag, accepted count, conflict list (JSON).

3. Subscribes to Xbox controller button/joystick events and publishes the
   mapped ROS2 messages.  Both robot-local and host-synced actions are
   available for execution.

Two sources of actions (in memory at runtime)
---------------------------------------------
  robot DB actions  — persistent, survive restarts, defined on the robot
  host-synced       — ephemeral, live only while the node is running,
                      re-sent by the Docker node on every startup

Conflict rule
-------------
An incoming action conflicts if its name already exists in the robot's loaded
actions (from the local DB).  Names must be unique across both sources.
The error response tells you the existing type and incoming type.

Database
--------
  ~/Documents/actions.db  (robot's own actions only — host actions never here)
"""

import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

from database_interface import IDatabase
from database import Database

# ── Optional imports ──────────────────────────────────────────────────────────
try:
    from mecanumbot_msgs.msg import AccessMotorCmd, ButtonEvent, JoystickEvent
    MECANUMBOT_MSGS_AVAILABLE = True
except ImportError:
    MECANUMBOT_MSGS_AVAILABLE = False
    print("Warning: mecanumbot_msgs not available — AccessMotorCmd/ButtonEvent/JoystickEvent will not work")

try:
    from mecanumbot_msgs.srv import SyncActions, RegisterAction, SetLedStatus, SyncMappings
    from mecanumbot_msgs.msg import ActionTuple, ActionDescriptor
    MECANUMBOT_SRV_AVAILABLE = True
except ImportError:
    MECANUMBOT_SRV_AVAILABLE = False
    print("Warning: mecanumbot_msgs services not available — SyncActions/RegisterAction services disabled")

# ─────────────────────────────────────────────────────────────────────────────
class MappingListener(Node):
    """
    ROS2 Node that:
    - serves the SyncActions service (receives host actions, checks conflicts)
    - loads its own persistent actions from the local SQLite database
    - keeps host-synced actions in memory ONLY (never written to the robot DB)
    - subscribes to controller events and publishes mapped actions
    """

    def __init__(self, db: IDatabase):
        super().__init__('mapping_listener')

        # ── Database ──────────────────────────────────────────────────────────
        self._db = db

        # ── In-memory state ───────────────────────────────────────────────────
        # self.actions holds BOTH robot-local actions (from DB) and
        # host-synced actions (from SyncActions service).
        # Host-synced actions are kept in memory only — never written to the DB.
        self.button_mappings   = {}
        self.joystick_mappings = {}
        self.actions           = {}   # all actions available for execution

        # self._db_actions is a snapshot of only the robot's own DB actions.
        # Used for conflict checking — host-synced actions never go in here.
        self._db_actions       = {}

        # ── Publisher caches ──────────────────────────────────────────────────
        self._publishers_dict      = {}

        # ── Message-type map ──────────────────────────────────────────────────
        self.message_type_map = {
            'std_msgs/msg/String':     String,
            'std_msgs/msg/Bool':       Bool,
            'geometry_msgs/msg/Twist': Twist,
        }
        if MECANUMBOT_MSGS_AVAILABLE:
            self.message_type_map['mecanumbot_msgs/msg/AccessMotorCmd'] = AccessMotorCmd

        # ── Controller subscribers ────────────────────────────────────────────
        if MECANUMBOT_MSGS_AVAILABLE:
            self.create_subscription(
                ButtonEvent, '/xbox_controller/button_events',
                self.xbox_button_callback, 10)
            self.create_subscription(
                JoystickEvent, '/xbox_controller/joystick_events',
                self.xbox_joystick_callback, 10)
        else:
            self.get_logger().warn(
                'mecanumbot_msgs not available — controller subscriptions disabled')

        # ── SyncActions service server ────────────────────────────────────────
        if MECANUMBOT_SRV_AVAILABLE:
            self._sync_srv = self.create_service(
                SyncActions,
                '/mecanumbot/sync_actions',
                self._sync_actions_callback
            )
            self._register_srv = self.create_service(
                RegisterAction,
                '/mecanumbot/register_action',
                self._register_action_callback
            )
            self._sync_mappings_srv = self.create_service(
                SyncMappings,
                '/mecanumbot/sync_mappings',
                self._sync_mappings_callback
            )
            self.get_logger().info(
                'Services ready: /mecanumbot/sync_actions, /mecanumbot/register_action, /mecanumbot/sync_mappings')

            # ── LED set service client ─────────────────────────────────────────
            self._led_set_client = self.create_client(
                SetLedStatus, 'set_led_status')
            self.get_logger().info('LED service client created: set_led_status')
        else:
            self._led_set_client = None
            self.get_logger().warn(
                'SyncActions/RegisterAction services NOT available — mecanumbot_msgs not installed. '
                'Actions will only be loaded from the local database.')

        # ── Load from local DB ────────────────────────────────────────────────
        self._load_from_db()

        # ── Periodic state log (every 30 s) ──────────────────────────────────
        self.create_timer(30.0, self._log_state)

        self.get_logger().info('Mapping Listener Node started')

    # ══════════════════════════════════════════════════════════════════════════
    # Database helpers
    # ══════════════════════════════════════════════════════════════════════════

    def _load_from_db(self):
        """Load all actions and mappings from the local database into memory."""
        try:
            all_actions       = self._db.get_all_actions()
            button_mappings   = self._db.get_all_button_mappings()
            joystick_mappings = self._db.get_all_joystick_mappings()
        except Exception as e:
            self.get_logger().error(f'DB load error: {e}')
            return

        for name, data in all_actions.items():
            tuples = [tuple(t) for t in data['tuples']]
            self.actions[name]     = {'type': data['type'], 'tuples': tuples}
            self._db_actions[name] = {'type': data['type'], 'tuples': tuples}

        for btn, info in button_mappings.items():
            self.button_mappings[btn] = {
                'action': info['action'], 'trigger_mode': info['trigger_mode']}

        self.joystick_mappings = dict(joystick_mappings)

        self.get_logger().info(
            f'Loaded from local DB: {len(self.actions)} actions, '
            f'{len(self.button_mappings)} button mappings, '
            f'{len(self.joystick_mappings)} joystick mappings')

    # ══════════════════════════════════════════════════════════════════════════
    # SyncActions service callback
    # ══════════════════════════════════════════════════════════════════════════

    def _sync_actions_callback(self, request, response):
        """
        Receive a list of actions from the host, check for name conflicts
        against the robot's own actions (loaded from local DB), and load
        accepted actions into memory ONLY — they are NOT written to the
        robot's database.

        Conflict rule: any action whose name already exists in self.actions
        (i.e. was loaded from the robot's local DB) is a conflict.
        Host-to-host duplicates within the same sync call are also caught.
        """
        self.get_logger().info('SyncActions service called')

        conflicts_names   = []
        conflicts_reasons = []
        accepted          = []

        # Remove all previously host-synced actions before loading the new batch.
        host_synced_names = [n for n in self.actions if n not in self._db_actions]
        for n in host_synced_names:
            del self.actions[n]
        if host_synced_names:
            self.get_logger().info(
                f'SyncActions: cleared {len(host_synced_names)} previously synced host action(s)')

        for action_desc in request.actions:
            name        = action_desc.name.strip()
            action_type = action_desc.action_type

            if not name:
                continue

            if name in self.actions:
                existing_type = self.actions[name]['type']
                conflicts_names.append(name)
                conflicts_reasons.append(
                    f'Action "{name}" already exists on the robot '
                    f'(existing type: {existing_type}, '
                    f'incoming type: {action_type}). '
                    'Action names must be unique.'
                )
            else:
                accepted.append(action_desc)

        # Load accepted actions into memory ONLY — no DB write
        for action_desc in accepted:
            name        = action_desc.name
            action_type = action_desc.action_type
            self.actions[name] = {
                'type': action_type,
                'tuples': [
                    (t.topic, t.message, t.message_type,
                     float(t.scale_x), float(t.scale_y),
                     float(t.offset_x), float(t.offset_y),
                     t.publish_type, t.service_name)
                    for t in action_desc.tuples
                ]
            }
            self.get_logger().info(f'  + Loaded (memory-only) action "{name}" ({action_type})')

        response.success        = len(conflicts_names) == 0
        response.accepted_count = len(accepted)
        response.conflict_names   = conflicts_names
        response.conflict_reasons = conflicts_reasons

        # Build robot_actions: the robot's own DB-loaded actions
        robot_actions = []
        for name, data in self._db_actions.items():
            desc        = ActionDescriptor()
            desc.name   = name
            desc.action_type = data['type']
            desc.tuples = []
            for t in data.get('tuples', []):
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
            robot_actions.append(desc)
        response.robot_actions = robot_actions

        if conflicts_names:
            self.get_logger().warn(
                f'SyncActions: {len(accepted)} loaded, '
                f'{len(conflicts_names)} conflict(s):\n' +
                '\n'.join(f'  * {n}: {r}' for n, r in zip(conflicts_names, conflicts_reasons)))
        else:
            self.get_logger().info(
                f'SyncActions: all {len(accepted)} actions loaded into memory')

        return response

    def _register_action_callback(self, request, response):
        """
        Service handler for /mecanumbot/register_action.

        Called when the user creates a single new action in the GUI.
        Checks for a name conflict against the robot's own DB actions only.
        If no conflict: loads the action into memory (not the DB) and returns
        success=true so the Docker node can save it to the host DB.
        """
        name        = (request.action.name or '').strip()
        action_type = request.action.action_type or 'button_once'

        self.get_logger().info(f'RegisterAction called: "{name}" ({action_type})')

        if not name:
            response.success  = False
            response.conflict = 'Action name cannot be empty.'
            return response

        if name in self.actions:
            existing_type = self.actions[name]['type']
            origin = 'robot DB' if name in self._db_actions else 'host (already registered)'
            response.success  = False
            response.conflict = (
                f'Action "{name}" already exists ({origin}, '
                f'type: {existing_type}). Choose a different name.'
            )
            self.get_logger().warn(f'  ! Conflict: {response.conflict}')
            return response

        # No conflict — load into memory
        self.actions[name] = {
            'type': action_type,
            'tuples': [
                (t.topic, t.message, t.message_type,
                 float(t.scale_x), float(t.scale_y),
                 float(t.offset_x), float(t.offset_y),
                 t.publish_type, t.service_name)
                for t in request.action.tuples
            ]
        }
        response.success  = True
        response.conflict = ''
        self.get_logger().info(
            f'  + RegisterAction: loaded "{name}" into memory ({action_type})')
        return response

    # ══════════════════════════════════════════════════════════════════════════
    # SyncMappings service callback
    # ══════════════════════════════════════════════════════════════════════════

    def _sync_mappings_callback(self, request, response):
        """
        Receive the full button and joystick mapping tables from the host and
        overwrite the in-memory mappings.  The robot's local database is never
        touched — mappings are stored in memory only.
        """
        self.get_logger().info('SyncMappings service called')

        # Always overwrite — clear existing in-memory mappings first
        self.button_mappings   = {}
        self.joystick_mappings = {}

        loaded_btn_names = []
        loaded_joy_names = []

        for i in range(len(request.button_names)):
            btn    = request.button_names[i]
            action = request.action_names[i] if i < len(request.action_names) else ''
            mode   = request.trigger_modes[i] if i < len(request.trigger_modes) else 'once'
            if btn:
                self.button_mappings[btn] = {'action': action, 'trigger_mode': mode}
                loaded_btn_names.append(btn)

        for j in range(len(request.joystick_names)):
            joy    = request.joystick_names[j]
            action = request.joystick_action_names[j] if j < len(request.joystick_action_names) else ''
            if joy:
                self.joystick_mappings[joy] = action
                loaded_joy_names.append(joy)

        response.success               = True
        response.loaded_button_count   = len(loaded_btn_names)
        response.loaded_joystick_count = len(loaded_joy_names)
        response.loaded_button_names   = loaded_btn_names
        response.loaded_joystick_names = loaded_joy_names

        self.get_logger().info(
            f'SyncMappings: loaded {len(loaded_btn_names)} button mapping(s), '
            f'{len(loaded_joy_names)} joystick mapping(s) into memory')
        return response

    # ══════════════════════════════════════════════════════════════════════════
    # Publisher helpers
    # ══════════════════════════════════════════════════════════════════════════

    def get_or_create_publisher(self, topic, message_type_str):
        if topic in self._publishers_dict:
            return self._publishers_dict[topic]
        msg_class = self.message_type_map.get(message_type_str)
        if msg_class is None:
            self.get_logger().warn(f'Unsupported message type: {message_type_str}')
            return None
        pub = self.create_publisher(msg_class, topic, 10)
        self._publishers_dict[topic] = pub
        self.get_logger().info(f'Created publisher for {topic} ({message_type_str})')
        return pub

    def parse_message_data(self, message_str, message_type_str):
        try:
            data = json.loads(message_str)
            if message_type_str == 'std_msgs/msg/String':
                msg = String()
                msg.data = data.get('data', message_str)
                return msg
            if message_type_str == 'std_msgs/msg/Bool':
                msg = Bool()
                msg.data = bool(data.get('data', False))
                return msg
            if message_type_str == 'geometry_msgs/msg/Twist':
                msg = Twist()
                lin = data.get('linear', {})
                ang = data.get('angular', {})
                msg.linear.x  = float(lin.get('x', 0.0))
                msg.linear.y  = float(lin.get('y', 0.0))
                msg.linear.z  = float(lin.get('z', 0.0))
                msg.angular.x = float(ang.get('x', 0.0))
                msg.angular.y = float(ang.get('y', 0.0))
                msg.angular.z = float(ang.get('z', 0.0))
                return msg
            if (message_type_str == 'mecanumbot_msgs/msg/AccessMotorCmd'
                    and MECANUMBOT_MSGS_AVAILABLE):
                msg = AccessMotorCmd()
                msg.n_pos  = float(data.get('n_pos',  5.3))
                msg.gl_pos = float(data.get('gl_pos', 5.12))
                msg.gr_pos = float(data.get('gr_pos', 5.12))
                return msg
            self.get_logger().error(
                f'Message type {message_type_str} not implemented')
            return None
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse message JSON: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'Error creating message: {e}')
            return None

    # ══════════════════════════════════════════════════════════════════════════
    # Controller callbacks
    # ══════════════════════════════════════════════════════════════════════════

    def xbox_button_callback(self, msg: 'ButtonEvent'):
        try:
            button_name = msg.button_name
            event_type  = msg.event

            if button_name not in self.button_mappings:
                return

            mapping_info = self.button_mappings[button_name]
            action_name  = mapping_info['action']
            trigger_mode = mapping_info['trigger_mode']

            if action_name not in self.actions:
                self.get_logger().warn(f'Action "{action_name}" not configured')
                return

            if trigger_mode == 'once' and event_type == 'PRESSED':
                self.publish_action(action_name, button_name)
            elif trigger_mode == 'hold' and event_type in ('PRESSED', 'HOLD'):
                self.publish_action(action_name, button_name)

        except Exception as e:
            self.get_logger().error(f'Button event error: {e}')

    def xbox_joystick_callback(self, msg: 'JoystickEvent'):
        try:
            joystick_name = msg.joystick_name
            x_value       = msg.x
            y_value       = msg.y

            if joystick_name not in self.joystick_mappings:
                return

            action_name = self.joystick_mappings[joystick_name]
            if action_name not in self.actions:
                self.get_logger().warn(f'Action "{action_name}" not configured')
                return

            self.publish_joystick_action(action_name, joystick_name, x_value, y_value)

        except Exception as e:
            self.get_logger().error(f'Joystick event error: {e}')

    # ══════════════════════════════════════════════════════════════════════════
    # Action publishing
    # ══════════════════════════════════════════════════════════════════════════

    def publish_action(self, action_name, trigger_control):
        action_data = self.actions.get(action_name)
        if not action_data:
            self.get_logger().warn(f'Action "{action_name}" not in memory')
            return
        self.get_logger().info(
            f'[BUTTON] Action "{action_name}" triggered by {trigger_control}')
        for tup in action_data.get('tuples', []):
            # tuples: (topic, message, type, scale_x, scale_y, offset_x, offset_y, publish_type, service_name)
            topic        = tup[0]
            message_str  = tup[1]
            message_type = tup[2] if len(tup) >= 3 else 'std_msgs/msg/String'
            publish_type = tup[7] if len(tup) >= 8 else 'topic'
            service_name = tup[8] if len(tup) >= 9 else ''

            if publish_type == 'service':
                self._execute_service_action(service_name, message_str, action_name)
            else:
                self._publish_single(topic, message_str, message_type)

    def _execute_service_action(self, service_name: str, cfg_json: str, action_name: str):
        """Dispatch a service call based on service_name."""
        if service_name == 'set_led_status':
            self._execute_led_service(cfg_json, action_name)
        else:
            self.get_logger().warn(
                f'[SERVICE] Unknown service "{service_name}" for action "{action_name}"')

    def _execute_led_service(self, cfg_json: str, action_name: str):
        """Parse a LED JSON config and call the SetLedStatus service."""
        if self._led_set_client is None:
            self.get_logger().warn(
                f'[LED] Cannot execute "{action_name}" — LED service client not available')
            return
        if not self._led_set_client.service_is_ready():
            self.get_logger().warn(
                f'[LED] Cannot execute "{action_name}" — '
                'set_led_status service not ready')
            return
        try:
            cfg = json.loads(cfg_json)
        except (json.JSONDecodeError, TypeError) as e:
            self.get_logger().error(f'[LED] Failed to parse LED config JSON: {e}')
            return

        req = SetLedStatus.Request()
        corners = {
            'FL': ('fl_color', 'fl_mode'),
            'FR': ('fr_color', 'fr_mode'),
            'BL': ('bl_color', 'bl_mode'),
            'BR': ('br_color', 'br_mode'),
        }
        for corner, (color_field, mode_field) in corners.items():
            corner_data = cfg.get(corner, {})
            setattr(req, color_field, int(corner_data.get('color') or 0))
            setattr(req, mode_field,  int(corner_data.get('mode')  or 4))

        future = self._led_set_client.call_async(req)
        future.add_done_callback(
            lambda f: self._on_led_response(f, action_name))
        self.get_logger().info(f'  -> LED action "{action_name}" sent to set_led_status')

    def _on_led_response(self, future, action_name: str):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f'  [LED] "{action_name}" applied OK')
            else:
                self.get_logger().warn(
                    f'  [LED] "{action_name}" service returned failure')
        except Exception as e:
            self.get_logger().error(f'  [LED] "{action_name}" call error: {e}')

    def publish_joystick_action(self, action_name, joystick_name, x_value, y_value):
        action_data = self.actions.get(action_name)
        if not action_data:
            self.get_logger().warn(f'Action "{action_name}" not in memory')
            return
        self.get_logger().info(
            f'[JOYSTICK] Action "{action_name}" '
            f'({joystick_name}, X={x_value:.2f}, Y={y_value:.2f})')
        for tup in action_data.get('tuples', []):
            topic        = tup[0]
            msg_tmpl     = tup[1]
            message_type = tup[2] if len(tup) >= 3 else 'std_msgs/msg/String'
            scale_x      = float(tup[3]) if len(tup) >= 4 else 1.0
            scale_y      = float(tup[4]) if len(tup) >= 5 else 1.0
            offset_x     = float(tup[5]) if len(tup) >= 6 else 0.0
            offset_y     = float(tup[6]) if len(tup) >= 7 else 0.0
            publish_type = tup[7] if len(tup) >= 8 else 'topic'

            if publish_type == 'service':
                # Service calls are not applicable for joystick actions — skip
                self.get_logger().warn(
                    f'[JOYSTICK] Tuple in "{action_name}" has publish_type=service — '
                    'services cannot be driven by joystick; skipping')
                continue

            scaled_x = round(x_value * scale_x + offset_x, 4)
            scaled_y = round(y_value * scale_y + offset_y, 4)
            # Replace quoted "X" / "Y" placeholders only — bare replace('X', ...)
            # would corrupt any JSON key or string value that contains these letters.
            message_str = msg_tmpl.replace('"X"', str(scaled_x)).replace('"Y"', str(scaled_y))
            self._publish_single(topic, message_str, message_type)

    def _publish_single(self, topic, message_str, message_type):
        pub = self.get_or_create_publisher(topic, message_type)
        if pub is None:
            self.get_logger().error(
                f'  [!] No publisher for {topic} ({message_type})')
            return
        msg = self.parse_message_data(message_str, message_type)
        if msg is None:
            self.get_logger().error(
                f'  [!] Could not parse message for {topic}')
            return
        pub.publish(msg)
        preview = message_str[:80] + ('...' if len(message_str) > 80 else '')
        self.get_logger().info(f'  -> {topic}: {preview}')

    # ══════════════════════════════════════════════════════════════════════════
    # Periodic logging
    # ══════════════════════════════════════════════════════════════════════════

    def _log_state(self):
        self.get_logger().info(
            f'State: {len(self.actions)} actions | '
            f'{len(self.button_mappings)} button mappings | '
            f'{len(self.joystick_mappings)} joystick mappings')


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None, db: IDatabase = None):
    rclpy.init(args=args)

    if db is None:
        db_path = os.path.expanduser('~/Documents/actions.db')
        db = Database(db_path)

    node = MappingListener(db)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
