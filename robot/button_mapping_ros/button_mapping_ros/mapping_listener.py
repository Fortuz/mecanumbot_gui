#!/usr/bin/env python3
"""
Mapping Listener Node  (robot-side)
====================================

Responsibilities
----------------
1. Owns the **robot's local SQLite database** (~/Documents/actions.db).
   Actions saved directly on the robot live here permanently.
   No actions are pre-loaded into memory at startup.

2. Exposes the **/mecanumbot/get_robot_actions** ROS2 service
   (mecanumbot_msgs/srv/GetRobotActions).
   The Docker/GUI node calls this service to retrieve all actions stored in
   the robot's local DB for a given user.  The host uses this to let the
   user browse and select "robot" actions in the UI.

3. Exposes the **/mecanumbot/apply_mapping** ROS2 service.
   When the user selects a mapping in the dashboard this service is called:
     - from_robot_db=True  → load the mapping and its referenced actions
                             directly from the robot's local database.
     - from_robot_db=False → the host sends the full mapping data (button /
                             joystick assignments + action definitions) inline.
   Either way, self.actions is fully replaced with only the actions needed
   by the newly selected mapping.

4. Subscribes to Xbox controller button/joystick events and publishes the
   mapped ROS2 messages using the currently active mapping's actions.

Active-mapping memory model
---------------------------
  self.actions        — actions for the CURRENTLY ACTIVE mapping only.
                        Fully replaced on every ApplyMapping call.
                        Empty until the first mapping is applied.
  self.button_mappings   — {button_name: {action, trigger_mode}}
  self.joystick_mappings — {joystick_name: action_name}

Conflict rule (SaveRobotAction)
--------------------------------
At save time, if an action with the same name already exists in the robot's
DB for the requesting user, the service returns success=False unless the
request has overwrite=True.

Database
--------
  ~/Documents/actions.db  (robot's own actions only)
"""

import json
import logging
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

from database_interface import IDatabase
from database import Database
from database import (
    DatabaseDirectoryError,
    DatabasePermissionError,
    DatabaseSchemaError,
    DatabaseConnectionError,
)

# ── Optional imports ──────────────────────────────────────────────────────────
try:
    from mecanumbot_msgs.msg import AccessMotorCmd, ButtonEvent, JoystickEvent
    MECANUMBOT_MSGS_AVAILABLE = True
except ImportError:
    MECANUMBOT_MSGS_AVAILABLE = False
    print("Warning: mecanumbot_msgs not available — AccessMotorCmd/ButtonEvent/JoystickEvent will not work")

try:
    from mecanumbot_msgs.srv import GetRobotActions, SetLedStatus
    from mecanumbot_msgs.srv import (
        GetMappingNames, GetMappingDetails,
        SaveMapping, DeleteRobotMapping, ApplyMapping,
        SaveRobotAction, DeleteRobotAction,
        GetRecordingSchemes, SaveRecordingScheme, DeleteRecordingScheme,
        GetActionUsages,
    )
    MECANUMBOT_SRV_AVAILABLE = True
except ImportError:
    MECANUMBOT_SRV_AVAILABLE = False
    print("Warning: mecanumbot_msgs services not available — GetRobotActions and mapping services disabled")

# ─────────────────────────────────────────────────────────────────────────────
class MappingListener(Node):
    """
    ROS2 Node that:
    - serves GetRobotActions (returns all robot DB actions for a user)
    - on ApplyMapping: loads only the active mapping's actions into memory
      (from robot DB when from_robot_db=True, from host inline data otherwise)
    - subscribes to controller events and publishes mapped actions
    """

    def __init__(self, db: IDatabase):
        super().__init__('mapping_listener')

        # ── Callback groups ───────────────────────────────────────────────────
        # Service callbacks use a reentrant group so they can execute
        # concurrently with each other and with subscription callbacks.
        # Controller subscriptions use a separate exclusive group to ensure
        # button/joystick events are processed in order.
        self._svc_cbg  = ReentrantCallbackGroup()
        self._ctrl_cbg = MutuallyExclusiveCallbackGroup()

        # ── Database ──────────────────────────────────────────────────────────
        self._db = db

        # Each service request carries the username; user_id is resolved
        # on demand via _uid_for(username).  No actions are pre-loaded.

        # ── Active-mapping in-memory state ────────────────────────────────────
        # self.actions holds ONLY the actions for the currently active mapping.
        # It is fully replaced on every ApplyMapping call and is empty until
        # the first mapping is applied.
        self.button_mappings   = {}
        self.joystick_mappings = {}
        self.actions           = {}   # current mapping's actions only

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
                self.xbox_button_callback, 10,
                callback_group=self._ctrl_cbg)
            self.create_subscription(
                JoystickEvent, '/xbox_controller/joystick_events',
                self.xbox_joystick_callback, 10,
                callback_group=self._ctrl_cbg)
        else:
            self.get_logger().warn(
                'mecanumbot_msgs not available — controller subscriptions disabled')

        # ── GetRobotActions service server ────────────────────────────────────
        if MECANUMBOT_SRV_AVAILABLE:
            self._get_robot_actions_srv = self.create_service(
                GetRobotActions,
                '/mecanumbot/get_robot_actions',
                self._get_robot_actions_callback,
                callback_group=self._svc_cbg,
            )
            # ── New persistent mapping/action management services ──────────────
            self._get_mapping_names_srv = self.create_service(
                GetMappingNames,    '/mecanumbot/get_mapping_names',   self._get_mapping_names_callback,   callback_group=self._svc_cbg)
            self._get_mapping_details_srv = self.create_service(
                GetMappingDetails,  '/mecanumbot/get_mapping_details', self._get_mapping_details_callback, callback_group=self._svc_cbg)
            self._save_mapping_srv  = self.create_service(
                SaveMapping,        '/mecanumbot/save_mapping',   self._save_mapping_callback,   callback_group=self._svc_cbg)
            self._del_mapping_srv   = self.create_service(
                DeleteRobotMapping, '/mecanumbot/delete_mapping', self._delete_mapping_callback, callback_group=self._svc_cbg)
            self._apply_mapping_srv = self.create_service(
                ApplyMapping,       '/mecanumbot/apply_mapping',  self._apply_mapping_callback,  callback_group=self._svc_cbg)
            self._save_action_srv   = self.create_service(
                SaveRobotAction,    '/mecanumbot/save_action',    self._save_robot_action_callback, callback_group=self._svc_cbg)
            self._del_action_srv    = self.create_service(
                DeleteRobotAction,  '/mecanumbot/delete_action',  self._delete_robot_action_callback, callback_group=self._svc_cbg)
            self._get_schemes_srv = self.create_service(
                GetRecordingSchemes, '/mecanumbot/get_recording_schemes',   self._get_recording_schemes_callback,   callback_group=self._svc_cbg)
            self._save_scheme_srv = self.create_service(
                SaveRecordingScheme, '/mecanumbot/save_recording_scheme',  self._save_recording_scheme_callback,  callback_group=self._svc_cbg)
            self._del_scheme_srv  = self.create_service(
                DeleteRecordingScheme, '/mecanumbot/delete_recording_scheme', self._delete_recording_scheme_callback, callback_group=self._svc_cbg)
            self._get_action_usages_srv = self.create_service(
                GetActionUsages, '/mecanumbot/get_action_usages', self._get_action_usages_callback, callback_group=self._svc_cbg)
            self.get_logger().info(
                'Services ready: /mecanumbot/get_robot_actions, '
                'get_mapping_names, get_mapping_details, save_mapping, delete_mapping, apply_mapping, '
                'save_action, delete_action, get_action_usages, '
                'get_recording_schemes, save_recording_scheme, delete_recording_scheme')

            # ── LED set service client ─────────────────────────────────────────
            self._led_set_client = self.create_client(
                SetLedStatus, 'set_led_status')
            self.get_logger().info('LED service client created: set_led_status')
        else:
            self._led_set_client = None
            self.get_logger().warn(
                'GetRobotActions and mapping services NOT available — mecanumbot_msgs not installed. '
                'Actions will only be loaded from the local database.')

        # ── Periodic state log (every 30 s) ──────────────────────────────────
        self.create_timer(30.0, self._log_state)

        self.get_logger().info('Mapping Listener Node started')

    # ══════════════════════════════════════════════════════════════════════════
    # Database helpers
    # ══════════════════════════════════════════════════════════════════════════

    def _uid_for(self, user_name: str) -> int:
        """Return the DB user_id for the given username, creating the row if needed."""
        name = (user_name or "").strip() or "unknown"
        return self._db.get_or_create_user(name)

    # ══════════════════════════════════════════════════════════════════════════
    # GetRobotActions service callback
    # ══════════════════════════════════════════════════════════════════════════

    def _get_robot_actions_callback(self, request, response):
        """Return all actions stored in the robot's local DB for the requesting user."""
        self.get_logger().info('GetRobotActions service called')
        try:
            uid = self._uid_for(request.user_name)
            db_actions = self._db.get_all_actions(uid)
        except Exception as e:
            self.get_logger().error(f'GetRobotActions: failed to read robot DB: {e}')
            response.success      = False
            response.action_names = []
            response.action_types = []
            response.actions_json = []
            response.message      = str(e)
            return response

        action_names = []
        action_types = []
        actions_json = []
        for aname, data in db_actions.items():
            action_names.append(aname)
            action_types.append(data.get('type', 'button'))
            actions_json.append(json.dumps({
                'name':   aname,
                'type':   data.get('type', 'button'),
                'tuples': data.get('tuples', []),
            }))

        response.success      = True
        response.action_names = action_names
        response.action_types = action_types
        response.actions_json = actions_json
        response.message      = ''
        self.get_logger().info(
            f'GetRobotActions: returning {len(action_names)} action(s) for "{request.user_name}"')
        return response

    # ══════════════════════════════════════════════════════════════════════════
    # GetMappingNames — return only the names of mappings stored in robot DB
    # ══════════════════════════════════════════════════════════════════════════

    def _get_mapping_names_callback(self, request, response):
        self.get_logger().info('GetMappingNames service called')
        try:
            uid      = self._uid_for(request.user_name)
            mappings = self._db.get_all_mappings(uid)
            response.success       = True
            response.mapping_names = [m['name'] for m in mappings]
            response.message       = ''
        except Exception as e:
            response.success       = False
            response.mapping_names = []
            response.message       = str(e)
            self.get_logger().error(f'GetMappingNames error: {e}')
        return response

    # ══════════════════════════════════════════════════════════════════════════
    # GetMappingDetails — return full button/joystick data for one mapping
    # ══════════════════════════════════════════════════════════════════════════

    def _get_mapping_details_callback(self, request, response):
        name = request.mapping_name.strip()
        self.get_logger().info(f'GetMappingDetails service called: "{name}"')
        try:
            uid     = self._uid_for(request.user_name)
            mapping = self._db.get_mapping(uid, name)
            if mapping is None:
                response.success      = False
                response.mapping_json = ''
                response.message      = f'Mapping "{name}" not found'
            else:
                response.success      = True
                response.mapping_json = json.dumps(mapping)
                response.message      = ''
        except Exception as e:
            response.success      = False
            response.mapping_json = ''
            response.message      = str(e)
            self.get_logger().error(f'GetMappingDetails error: {e}')
        return response

    # ══════════════════════════════════════════════════════════════════════════
    # SaveMapping — persist a named mapping (+ required actions) in robot DB
    # ══════════════════════════════════════════════════════════════════════════

    def _save_mapping_callback(self, request, response):
        name = request.mapping_name.strip()
        self.get_logger().info(f'SaveMapping service called: "{name}"')
        try:
            uid = self._uid_for(request.user_name)

            # Conflict check at save time
            if not request.overwrite:
                existing = self._db.get_mapping(uid, name)
                if existing is not None:
                    response.success = False
                    response.message = (
                        f'Mapping "{name}" already exists in the robot DB. '
                        'Use overwrite=true to replace it.')
                    self.get_logger().warn(
                        f'SaveMapping: conflict — "{name}" already exists')
                    return response

            # Save any bundled actions first
            for action_desc in request.actions:
                aname = action_desc.name.strip()
                if not aname:
                    continue
                tuples = [
                    (t.topic, t.message, t.message_type,
                     float(t.scale_x), float(t.scale_y),
                     float(t.offset_x), float(t.offset_y),
                     t.publish_type, t.service_name)
                    for t in action_desc.tuples
                ]
                self._db.save_action(uid, aname, tuples, action_desc.action_type)
                self.get_logger().info(f'  + Saved action "{aname}" to robot DB')

            # Build button / joystick entry lists
            btn_entries = []
            for i in range(len(request.button_names)):
                btn    = request.button_names[i]
                action = (request.button_action_names[i]
                          if i < len(request.button_action_names) else '')
                mode   = (request.button_trigger_modes[i]
                          if i < len(request.button_trigger_modes) else 'once')
                if btn and action:
                    btn_entries.append((btn, action, mode))

            joy_entries = []
            for j in range(len(request.joystick_names)):
                joy    = request.joystick_names[j]
                action = (request.joystick_action_names[j]
                          if j < len(request.joystick_action_names) else '')
                if joy and action:
                    joy_entries.append((joy, action))

            ok = self._db.save_mapping(uid, name, btn_entries, joy_entries)
            response.success = ok
            response.message = '' if ok else 'DB write failed'
            self.get_logger().info(
                f'SaveMapping "{name}": {len(btn_entries)} btn, '
                f'{len(joy_entries)} joy — success={ok}')
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'SaveMapping error: {e}')
        return response

    # ══════════════════════════════════════════════════════════════════════════
    # DeleteRobotMapping — remove a named mapping from robot DB
    # ══════════════════════════════════════════════════════════════════════════

    def _delete_mapping_callback(self, request, response):
        name = request.mapping_name.strip()
        self.get_logger().info(f'DeleteRobotMapping service called: "{name}"')
        try:
            ok = self._db.delete_mapping(self._uid_for(request.user_name), name)
            response.success = ok
            response.message = '' if ok else 'Delete failed'
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'DeleteRobotMapping error: {e}')
        return response

    # ══════════════════════════════════════════════════════════════════════════
    # ApplyMapping — activate a named mapping in robot memory
    # ══════════════════════════════════════════════════════════════════════════

    def _apply_mapping_callback(self, request, response):
        name          = request.mapping_name.strip()
        from_robot_db = request.from_robot_db
        self.get_logger().info(
            f'ApplyMapping service called: "{name}" from_robot_db={from_robot_db}')

        try:
            # Always start with a clean slate — replace the entire active
            # mapping state rather than merging with whatever was loaded before.
            self.actions           = {}
            self.button_mappings   = {}
            self.joystick_mappings = {}

            btn_map = {}
            joy_map = {}

            if from_robot_db:
                # ── Load mapping and its actions from the robot's own DB ──────
                uid     = self._uid_for(request.user_name)
                mapping = self._db.get_mapping(uid, name)
                if mapping is None:
                    response.success = False
                    response.message = f'Mapping "{name}" not found in robot DB'
                    return response

                # Collect the action names this mapping references
                needed_names = {b['action'] for b in mapping['buttons']}
                needed_names |= {j['action'] for j in mapping['joysticks']}

                # Load only the referenced actions from DB (not all actions)
                all_db_actions = self._db.get_all_actions(uid)
                for aname, data in all_db_actions.items():
                    if aname in needed_names:
                        self.actions[aname] = {
                            'type':   data['type'],
                            'tuples': [tuple(t) for t in data['tuples']],
                        }
                        self.get_logger().info(f'  + Loaded action "{aname}" from robot DB')

                for b in mapping['buttons']:
                    btn_map[b['button']] = {
                        'action':       b['action'],
                        'trigger_mode': b.get('trigger_mode', 'once'),
                    }
                for j in mapping['joysticks']:
                    joy_map[j['joystick']] = j['action']

            else:
                # ── Host sends the full mapping data inline ───────────────────
                for action_desc in request.actions:
                    aname = action_desc.name.strip()
                    if not aname:
                        continue
                    tuples = [
                        (t.topic, t.message, t.message_type,
                         float(t.scale_x), float(t.scale_y),
                         float(t.offset_x), float(t.offset_y),
                         t.publish_type, t.service_name)
                        for t in action_desc.tuples
                    ]
                    self.actions[aname] = {'type': action_desc.action_type, 'tuples': tuples}
                    self.get_logger().info(f'  + Loaded action "{aname}" from host (inline)')

                for i in range(len(request.button_names)):
                    btn    = request.button_names[i]
                    action = (request.button_action_names[i]
                              if i < len(request.button_action_names) else '')
                    mode   = (request.button_trigger_modes[i]
                              if i < len(request.button_trigger_modes) else 'once')
                    if btn and action:
                        btn_map[btn] = {'action': action, 'trigger_mode': mode}

                for j in range(len(request.joystick_names)):
                    joy    = request.joystick_names[j]
                    action = (request.joystick_action_names[j]
                              if j < len(request.joystick_action_names) else '')
                    if joy and action:
                        joy_map[joy] = action

            self.button_mappings   = btn_map
            self.joystick_mappings = joy_map

            response.success               = True
            response.message               = f'Applied mapping "{name}"'
            response.loaded_button_count   = len(btn_map)
            response.loaded_joystick_count = len(joy_map)
            self.get_logger().info(
                f'ApplyMapping "{name}": {len(self.actions)} action(s), '
                f'{len(btn_map)} buttons, {len(joy_map)} joysticks loaded into memory')
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'ApplyMapping error: {e}')
        return response

    # ══════════════════════════════════════════════════════════════════════════
    # SaveRobotAction — persist an action in robot DB
    # ══════════════════════════════════════════════════════════════════════════

    def _save_robot_action_callback(self, request, response):
        desc  = request.action
        name  = desc.name.strip()
        atype = desc.action_type
        self.get_logger().info(f'SaveRobotAction service called: "{name}" ({atype})')
        try:
            uid = self._uid_for(request.user_name)

            # Conflict check at save time
            if not request.overwrite:
                existing = self._db.get_all_actions(uid)
                if name in existing:
                    response.success = False
                    response.message = (
                        f'Action "{name}" already exists in the robot DB. '
                        'Use overwrite=true to replace it.')
                    self.get_logger().warn(
                        f'SaveRobotAction: conflict — "{name}" already exists')
                    return response

            tuples = [
                (t.topic, t.message, t.message_type,
                 float(t.scale_x), float(t.scale_y),
                 float(t.offset_x), float(t.offset_y),
                 t.publish_type, t.service_name)
                for t in desc.tuples
            ]
            ok = self._db.save_action(uid, name, tuples, atype)
            response.success = ok
            response.message = '' if ok else 'DB write failed'
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'SaveRobotAction error: {e}')
        return response

    # ══════════════════════════════════════════════════════════════════════════
    # DeleteRobotAction — remove an action from robot DB
    # ══════════════════════════════════════════════════════════════════════════

    def _delete_robot_action_callback(self, request, response):
        name = request.action_name.strip()
        self.get_logger().info(f'DeleteRobotAction service called: "{name}"')
        try:
            ok = self._db.delete_action(self._uid_for(request.user_name), name)
            if ok:
                # Also remove from the active mapping's memory if present,
                # so controller events for this action fail cleanly.
                self.actions.pop(name, None)
            response.success = ok
            response.message = '' if ok else 'Delete failed'
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'DeleteRobotAction error: {e}')
        return response

    # ══════════════════════════════════════════════════════════════════════════
    # Action usages service
    # ══════════════════════════════════════════════════════════════════════════

    def _get_action_usages_callback(self, request, response):
        self.get_logger().info(f'GetActionUsages service called for action "{request.action_name}"')
        try:
            uid = self._uid_for(request.user_name)
            mapping_names = self._db.get_mappings_using_action(uid, request.action_name)
            response.success = True
            response.mapping_names = list(mapping_names)
            response.message = ''
        except Exception as e:
            response.success = False
            response.mapping_names = []
            response.message = str(e)
            self.get_logger().error(f'GetActionUsages error: {e}')
        return response

    # ══════════════════════════════════════════════════════════════════════════
    # Recording scheme services
    # ══════════════════════════════════════════════════════════════════════════

    def _get_recording_schemes_callback(self, request, response):
        self.get_logger().info('GetRecordingSchemes service called')
        try:
            uid     = self._uid_for(request.user_name)
            schemes = self._db.get_all_recording_schemes(uid)
            response.scheme_names       = list(schemes.keys())
            response.scheme_topics_json = [
                json.dumps(topics) for topics in schemes.values()
            ]
            response.success = True
            response.message = ''
        except Exception as e:
            response.success            = False
            response.scheme_names       = []
            response.scheme_topics_json = []
            response.message            = str(e)
            self.get_logger().error(f'GetRecordingSchemes error: {e}')
        return response

    def _save_recording_scheme_callback(self, request, response):
        name = request.scheme_name.strip()
        self.get_logger().info(f'SaveRecordingScheme service called: "{name}"')
        try:
            uid = self._uid_for(request.user_name)
            if not request.overwrite:
                existing = self._db.get_all_recording_schemes(uid)
                if name in existing:
                    response.success = False
                    response.message = (
                        f'Recording scheme "{name}" already exists in the robot DB. '
                        'Use overwrite=true to replace it.')
                    return response
            ok = self._db.save_recording_scheme(uid, name, list(request.topics))
            response.success = ok
            response.message = '' if ok else 'DB write failed'
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'SaveRecordingScheme error: {e}')
        return response

    def _delete_recording_scheme_callback(self, request, response):
        name = request.scheme_name.strip()
        self.get_logger().info(f'DeleteRecordingScheme service called: "{name}"')
        try:
            uid = self._uid_for(request.user_name)
            ok  = self._db.delete_recording_scheme(uid, name)
            response.success = ok
            response.message = '' if ok else 'Delete failed'
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'DeleteRecordingScheme error: {e}')
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

            if action_name not in self.actions:
                self.get_logger().warn(f'Action "{action_name}" not configured')
                return

            # Use the per-button trigger_mode stored when the mapping was applied.
            # This respects the user-configured value (e.g. 'once' or 'hold')
            # rather than inferring it from the action type alone.
            trigger_mode = mapping_info.get('trigger_mode', 'once')

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
        try:
            db = Database(db_path)
        except DatabaseDirectoryError as e:
            logging.critical(
                '[FATAL] Database directory missing or could not be created.\n'
                '  %s\n'
                '  Make sure ~/Documents exists on the robot:\n'
                '      mkdir -p ~/Documents', e,
            )
            rclpy.shutdown()
            return
        except DatabasePermissionError as e:
            logging.critical(
                '[FATAL] Database directory is not writable.\n'
                '  %s\n'
                '  Fix permissions:\n'
                '      chmod 755 ~/Documents', e,
            )
            rclpy.shutdown()
            return
        except DatabaseSchemaError as e:
            logging.critical(
                '[FATAL] Database schema is outdated or corrupt.\n'
                '  %s\n'
                '  A fresh database will be created automatically on next start.', e,
            )
            rclpy.shutdown()
            return
        except DatabaseConnectionError as e:
            logging.critical(
                '[FATAL] Could not connect to the database.\n'
                '  %s', e,
            )
            rclpy.shutdown()
            return

    node = MappingListener(db)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
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
