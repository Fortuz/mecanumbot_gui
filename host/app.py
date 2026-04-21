from flask import Flask, request, render_template, redirect, url_for, jsonify
import os
import threading

from database import IDatabase
from database import Database
from database import (
    DatabaseDirectoryError,
    DatabasePermissionError,
    DatabaseSchemaError,
    DatabaseConnectionError,
)
import database as _db_constants
import docker_node

# ──────────────────────────────────────────────────────────────
# Controller / Button layout
# ──────────────────────────────────────────────────────────────

XBOX360_BUTTONS = [
    "A", "B", "X", "Y",
    "LB", "RB",
    "BACK", "START", "XBOX",
    "LS", "RS",
    "DPadUp", "DPadDown", "DPadLeft", "DPadRight"
]

CONTROLLER_JOYSTICKS = [
    "Left Stick",
    "Right Stick",
    "Left Trigger (LT)",
    "Right Trigger (RT)"
]

# ──────────────────────────────────────────────────────────────
# Topic schemas
# ──────────────────────────────────────────────────────────────

TOPIC_SCHEMAS = {
    '/cmd_vel': {
        'msg_type': 'geometry_msgs/msg/Twist',
        'description': 'Drive: robot velocity command (wheels)',
        'fields': [
            {'name': 'linear.x',  'label': 'linear.x  — forward / back (m/s)',      'type': 'float', 'default': '0.0', 'placeholder': '0.0', 'joystick_role': 'Y'},
            {'name': 'linear.y',  'label': 'linear.y  — strafe left / right (m/s)', 'type': 'float', 'default': '0.0', 'placeholder': '0.0', 'joystick_role': 'X'},
            {'name': 'linear.z',  'label': 'linear.z  — vertical (m/s)',             'type': 'float', 'default': '0.0', 'placeholder': '0.0'},
            {'name': 'angular.x', 'label': 'angular.x — roll (rad/s)',               'type': 'float', 'default': '0.0', 'placeholder': '0.0'},
            {'name': 'angular.y', 'label': 'angular.y — pitch (rad/s)',              'type': 'float', 'default': '0.0', 'placeholder': '0.0'},
            {'name': 'angular.z', 'label': 'angular.z — yaw / turn (rad/s)',         'type': 'float', 'default': '0.0', 'placeholder': '0.0', 'joystick_role': 'X'},
        ],
    },
    '/cmd_accessory_pos': {
        'msg_type': 'mecanumbot_msgs/msg/AccessMotorCmd',
        'description': 'Accessories: neck + both grabbers together',
        'fields': [
            {'name': 'n_pos',  'label': 'n_pos  — neck tilt (2.0 – 8.6)',          'type': 'float', 'default': '5.3',  'placeholder': '5.3',  'joystick_role': 'Y'},
            {'name': 'gl_pos', 'label': 'gl_pos — left grabber (1.6 – 8.54)',       'type': 'float', 'default': '5.12', 'placeholder': '5.12', 'joystick_role': 'X'},
            {'name': 'gr_pos', 'label': 'gr_pos — right grabber (1.6 – 8.54)',      'type': 'float', 'default': '5.12', 'placeholder': '5.12'},
        ],
    },
    '/cmd_accessory_pos [neck only]': {
        'msg_type': 'mecanumbot_msgs/msg/AccessMotorCmd',
        'description': 'Accessories: neck tilt only',
        '_topic_override': '/cmd_accessory_pos',
        'fields': [
            {'name': 'n_pos',  'label': 'n_pos — neck tilt (2.0 – 8.6)',            'type': 'float', 'default': '5.3',  'placeholder': '5.3',  'joystick_role': 'Y'},
            {'name': 'gl_pos', 'label': 'gl_pos — left grabber (fixed default)',     'type': 'float', 'default': '5.12', 'placeholder': '5.12', 'readonly': True},
            {'name': 'gr_pos', 'label': 'gr_pos — right grabber (fixed default)',    'type': 'float', 'default': '5.12', 'placeholder': '5.12', 'readonly': True},
        ],
    },
    '/cmd_accessory_pos [left grabber only]': {
        'msg_type': 'mecanumbot_msgs/msg/AccessMotorCmd',
        'description': 'Accessories: left grabber only',
        '_topic_override': '/cmd_accessory_pos',
        'fields': [
            {'name': 'n_pos',  'label': 'n_pos  — neck tilt (fixed default)',        'type': 'float', 'default': '5.3',  'placeholder': '5.3',  'readonly': True},
            {'name': 'gl_pos', 'label': 'gl_pos — left grabber (1.6 – 8.54)',        'type': 'float', 'default': '5.12', 'placeholder': '5.12', 'joystick_role': 'Y'},
            {'name': 'gr_pos', 'label': 'gr_pos — right grabber (fixed default)',    'type': 'float', 'default': '5.12', 'placeholder': '5.12', 'readonly': True},
        ],
    },
    '/cmd_accessory_pos [right grabber only]': {
        'msg_type': 'mecanumbot_msgs/msg/AccessMotorCmd',
        'description': 'Accessories: right grabber only',
        '_topic_override': '/cmd_accessory_pos',
        'fields': [
            {'name': 'n_pos',  'label': 'n_pos  — neck tilt (fixed default)',        'type': 'float', 'default': '5.3',  'placeholder': '5.3',  'readonly': True},
            {'name': 'gl_pos', 'label': 'gl_pos — left grabber (fixed default)',     'type': 'float', 'default': '5.12', 'placeholder': '5.12', 'readonly': True},
            {'name': 'gr_pos', 'label': 'gr_pos — right grabber (1.6 – 8.54)',       'type': 'float', 'default': '5.12', 'placeholder': '5.12', 'joystick_role': 'Y'},
        ],
    },
    '/cmd_accessory_pos [both grabbers]': {
        'msg_type': 'mecanumbot_msgs/msg/AccessMotorCmd',
        'description': 'Accessories: both grabbers move together',
        '_topic_override': '/cmd_accessory_pos',
        'fields': [
            {'name': 'n_pos',  'label': 'n_pos  — neck tilt (fixed default)',        'type': 'float', 'default': '5.3',  'placeholder': '5.3',  'readonly': True},
            {'name': 'gl_pos', 'label': 'gl_pos — left grabber (1.6 – 8.54)',        'type': 'float', 'default': '5.12', 'placeholder': '5.12', 'joystick_role': 'Y'},
            {'name': 'gr_pos', 'label': 'gr_pos — right grabber (mirrors gl_pos)',   'type': 'float', 'default': '5.12', 'placeholder': '5.12', 'joystick_role': 'Y'},
        ],
    },
    '/cmd_vel [stop]': {
        'msg_type': 'geometry_msgs/msg/Twist',
        'description': 'Drive: emergency stop',
        '_topic_override': '/cmd_vel',
        'fields': [
            {'name': 'linear.x',  'label': 'linear.x  (fixed 0.0)', 'type': 'float', 'default': '0.0', 'placeholder': '0.0', 'readonly': True},
            {'name': 'linear.y',  'label': 'linear.y  (fixed 0.0)', 'type': 'float', 'default': '0.0', 'placeholder': '0.0', 'readonly': True},
            {'name': 'linear.z',  'label': 'linear.z  (fixed 0.0)', 'type': 'float', 'default': '0.0', 'placeholder': '0.0', 'readonly': True},
            {'name': 'angular.x', 'label': 'angular.x (fixed 0.0)', 'type': 'float', 'default': '0.0', 'placeholder': '0.0', 'readonly': True},
            {'name': 'angular.y', 'label': 'angular.y (fixed 0.0)', 'type': 'float', 'default': '0.0', 'placeholder': '0.0', 'readonly': True},
            {'name': 'angular.z', 'label': 'angular.z (fixed 0.0)', 'type': 'float', 'default': '0.0', 'placeholder': '0.0', 'readonly': True},
        ],
    },
    '/cmd_accessory_pos [reset all]': {
        'msg_type': 'mecanumbot_msgs/msg/AccessMotorCmd',
        'description': 'Accessories: reset to neutral defaults',
        '_topic_override': '/cmd_accessory_pos',
        'fields': [
            {'name': 'n_pos',  'label': 'n_pos  (neutral 5.3)',  'type': 'float', 'default': '5.3',  'placeholder': '5.3',  'readonly': True},
            {'name': 'gl_pos', 'label': 'gl_pos (neutral 5.12)', 'type': 'float', 'default': '5.12', 'placeholder': '5.12', 'readonly': True},
            {'name': 'gr_pos', 'label': 'gr_pos (neutral 5.12)', 'type': 'float', 'default': '5.12', 'placeholder': '5.12', 'readonly': True},
        ],
    },
    '/chatter': {
        'msg_type': 'std_msgs/msg/String',
        'description': 'Generic: plain string message on /chatter',
        'fields': [
            {'name': 'data', 'label': 'data — string payload', 'type': 'str', 'default': '', 'placeholder': 'hello'},
        ],
    },
}


class FlaskApp:
    """Encapsulates the Flask application, database, and ROS2 background thread."""

    def __init__(self, db: IDatabase):
        self._db = db

        self.ACTION_TYPE_BUTTON_ONCE = _db_constants.ACTION_BUTTON_ONCE
        self.ACTION_TYPE_BUTTON_HOLD = _db_constants.ACTION_BUTTON_HOLD
        self.ACTION_TYPE_JOYSTICK    = _db_constants.ACTION_JOYSTICK

        self.CONTROLLER_TYPE    = "Xbox 360"
        self.CONTROLLER_BUTTONS = XBOX360_BUTTONS.copy()
        print(f"Using controller layout: {self.CONTROLLER_TYPE}")

        self.app = Flask(__name__)
        self.app.secret_key = os.environ.get('FLASK_SECRET', 'mecanumbot-secret-2025')
        self._register_routes()

        self._ros_thread = threading.Thread(
            target=docker_node._start_docker_node, args=(self._db,), daemon=True
        )
        self._ros_thread.start()

    # ──────────────────────────────────────────────────────────
    # Internal helpers
    # ──────────────────────────────────────────────────────────

    def _node(self):
        return docker_node._docker_node_instance

    def _uid(self):
        """Current user_id — stored in docker_node global after login."""
        return docker_node.CURRENT_USER.get("id")

    def _uname(self):
        return docker_node.CURRENT_USER.get("name") or "User"

    # ──────────────────────────────────────────────────────────
    # Route registration
    # ──────────────────────────────────────────────────────────

    def _register_routes(self):
        app = self.app

        # ── Status / utility API ───────────────────────────────

        @app.route('/api/controller-status')
        def api_controller_status():
            return jsonify(docker_node.LATEST_CONTROLLER_STATUS)

        @app.route('/api/robot-status')
        def api_robot_status():
            if self._node() is None:
                return jsonify({"available": False, "reason": "ROS2 node is still starting up"})
            if not docker_node.ROBOT_ACTIVE:
                return jsonify({"available": False, "reason": "Robot offline"})
            return jsonify({"available": True, "reason": "Robot online"})

        @app.get('/api/opencr-state')
        def api_opencr_state():
            with docker_node._opencr_lock:
                state = docker_node.LATEST_OPENCR_STATE
            if state is None:
                return jsonify({"available": False})
            return jsonify({"available": True, **state})

        # ── Action APIs ────────────────────────────────────────

        @app.get('/api/actions')
        def api_get_actions():
            """Return actions for the current user.
            Query param: ?source=host (default) | robot
                         ?include_tuples=1 to include full tuple data
            """
            source         = request.args.get('source', 'host')
            include_tuples = request.args.get('include_tuples', '') == '1'
            uid            = self._uid()

            if source == 'robot':
                node = self._node()
                if node is None:
                    return jsonify({'error': 'ROS2 node not running'}), 503
                user_name = self._uname()
                robot_actions, err = node.get_robot_actions(user_name)
                if robot_actions is None:
                    return jsonify({'error': err or 'Failed to fetch robot actions'}), 502
                result = []
                for name, data in robot_actions.items():
                    item = {'name': name, 'type': data['type']}
                    if include_tuples:
                        item['tuples'] = data.get('tuples', [])
                    result.append(item)
                return jsonify(result)

            # source == 'host' (default)
            if uid is None:
                return jsonify([])
            actions = self._db.get_all_actions(uid)
            result = []
            for name, data in actions.items():
                item = {'name': name, 'type': data['type']}
                if include_tuples:
                    # Convert tuple list to list of lists for JSON
                    item['tuples'] = [list(t) for t in data.get('tuples', [])]
                result.append(item)
            return jsonify(result)

        @app.post('/api/actions/save')
        def api_save_action():
            """Save (create or update) an action.
            JSON body:
              { name, action_type, tuple_list (TAB-delimited lines), destination ("host"|"robot"|"both") }
            """
            uid = self._uid()
            if uid is None:
                return jsonify({"success": False, "error": "Not logged in"}), 401
            body         = request.get_json(silent=True) or {}
            name         = (body.get('name') or '').strip()
            action_type  = (body.get('action_type') or self.ACTION_TYPE_BUTTON_ONCE).strip()
            tuples_raw   = body.get('tuples') or []   # list of tuple-lists from JSON
            destination  = (body.get('destination') or 'host').strip()
            overwrite    = bool(body.get('overwrite', False))

            if not name:
                return jsonify({"success": False, "error": "Action name is required"}), 400
            if not tuples_raw:
                return jsonify({"success": False, "error": "At least one step is required"}), 400

            # tuples_raw is already a list of lists (from JS)
            tuples = [tuple(t) for t in tuples_raw]

            saved_host   = False
            saved_robot  = False
            robot_msg    = ''

            if destination in ('host', 'both'):
                # Conflict check for host DB
                if not overwrite:
                    existing = self._db.get_all_actions(uid)
                    if name in existing:
                        return jsonify({
                            "success": False,
                            "error": f'Action "{name}" already exists in the host DB. Use overwrite to replace it.'
                        }), 409
                ok = self._db.save_action(uid, name, tuples, action_type)
                if not ok:
                    return jsonify({"success": False, "error": "Failed to save to host database"}), 500
                saved_host = True
                if self._node() is not None:
                    self._node().record_new_action(name, action_type, tuples)

            if destination in ('robot', 'both'):
                if self._node() is None:
                    robot_msg = 'ROS2 node not running — could not save to robot'
                else:
                    ok, msg = self._node().save_action_to_robot(name, action_type, tuples, overwrite=overwrite)
                    saved_robot = ok
                    robot_msg   = msg if not ok else ''
                    if not ok and msg and 'already exists' in msg:
                        return jsonify({"success": False, "error": msg}), 409

            return jsonify({
                "success":     saved_host or saved_robot,
                "saved_host":  saved_host,
                "saved_robot": saved_robot,
                "robot_msg":   robot_msg,
            })

        @app.post('/api/actions/delete')
        def api_delete_action():
            uid  = self._uid()
            if uid is None:
                return jsonify({"success": False, "error": "Not logged in"}), 401
            body = request.get_json(silent=True) or {}
            name = (body.get('name') or '').strip()
            destination = (body.get('destination') or 'host').strip()
            if not name:
                return jsonify({"success": False, "error": "Name required"}), 400

            ok_host  = True
            ok_robot = True
            msg      = ''

            if destination in ('host', 'both'):
                ok_host = self._db.delete_action(uid, name)

            if destination in ('robot', 'both'):
                if self._node() is None:
                    ok_robot = False
                    msg = 'ROS2 node not running'
                else:
                    ok_robot, msg = self._node().delete_robot_action(name)

            return jsonify({"success": ok_host and ok_robot, "message": msg})

        # ── Mapping APIs ───────────────────────────────────────

        @app.get('/api/mappings')
        def api_get_mappings():
            """Return host mappings for the current user."""
            uid = self._uid()
            if uid is None:
                return jsonify([])
            return jsonify(self._db.get_all_mappings(uid))

        @app.get('/api/robot/mapping-names')
        def api_get_robot_mapping_names():
            """Return just the names of mappings stored on the robot."""
            if self._node() is None:
                return jsonify({"success": False, "names": [], "error": "ROS2 node not running"}), 503
            names, err = self._node().get_robot_mapping_names()
            return jsonify({"success": not bool(err), "names": names, "error": err})

        @app.get('/api/robot/mapping-details/<path:mapping_name>')
        def api_get_robot_mapping_details(mapping_name):
            """Return full button/joystick data for a single named robot mapping."""
            if self._node() is None:
                return jsonify({"success": False, "mapping": None, "error": "ROS2 node not running"}), 503
            mapping, err = self._node().get_robot_mapping_details(mapping_name)
            return jsonify({"success": not bool(err), "mapping": mapping, "error": err})

        @app.post('/api/mappings/save')
        def api_save_mapping():
            """Create or update a named mapping.
            JSON body:
              {
                name,
                destination: "host" | "robot" | "both",
                overwrite: bool (optional, default false),
                buttons:   [{button, action, trigger_mode}, ...],
                joysticks: [{joystick, action}, ...],
              }
            """
            uid  = self._uid()
            if uid is None:
                return jsonify({"success": False, "error": "Not logged in"}), 401
            body = request.get_json(silent=True) or {}
            name        = (body.get('name') or '').strip()
            destination = (body.get('destination') or 'host').strip()
            overwrite   = bool(body.get('overwrite', False))
            buttons_raw   = body.get('buttons')   or []
            joysticks_raw = body.get('joysticks') or []

            if not name:
                return jsonify({"success": False, "error": "Mapping name is required"}), 400

            btn_entries = [
                (b['button'], b['action'], b.get('trigger_mode', 'once'))
                for b in buttons_raw if b.get('button') and b.get('action')
            ]
            joy_entries = [
                (j['joystick'], j['action'])
                for j in joysticks_raw if j.get('joystick') and j.get('action')
            ]

            saved_host  = False
            saved_robot = False
            robot_msg   = ''

            if destination in ('host', 'both'):
                # Conflict check for host DB
                if not overwrite:
                    existing = self._db.get_mapping(uid, name)
                    if existing is not None:
                        return jsonify({
                            "success": False,
                            "error": f'Mapping "{name}" already exists in the host DB. Use overwrite to replace it.'
                        }), 409
                ok = self._db.save_mapping(uid, name, btn_entries, joy_entries)
                if not ok:
                    return jsonify({"success": False, "error": "Failed to save to host database"}), 500
                saved_host = True

            if destination in ('robot', 'both'):
                if self._node() is None:
                    robot_msg = 'ROS2 node not running'
                else:
                    # Gather actions needed by this mapping that live only on the host
                    actions_needed = {}
                    all_actions = self._db.get_all_actions(uid)
                    needed_names = {e[1] for e in btn_entries} | {e[1] for e in joy_entries}
                    actions_needed = {n: d for n, d in all_actions.items() if n in needed_names}
                    ok, msg = self._node().save_mapping_to_robot(
                        name, btn_entries, joy_entries, actions_needed, overwrite=overwrite
                    )
                    saved_robot = ok
                    robot_msg   = msg if not ok else ''
                    if not ok and msg and 'already exists' in msg:
                        return jsonify({"success": False, "error": msg}), 409

            return jsonify({
                "success":     saved_host or saved_robot,
                "saved_host":  saved_host,
                "saved_robot": saved_robot,
                "robot_msg":   robot_msg,
            })

        @app.post('/api/mappings/delete')
        def api_delete_mapping():
            uid  = self._uid()
            if uid is None:
                return jsonify({"success": False, "error": "Not logged in"}), 401
            body = request.get_json(silent=True) or {}
            name        = (body.get('name') or '').strip()
            destination = (body.get('destination') or 'host').strip()
            if not name:
                return jsonify({"success": False, "error": "Name required"}), 400

            ok_host  = True
            ok_robot = True
            msg      = ''

            if destination in ('host', 'both'):
                ok_host = self._db.delete_mapping(uid, name)

            if destination in ('robot', 'both'):
                if self._node() is None:
                    ok_robot = False
                    msg = 'ROS2 node not running'
                else:
                    ok_robot, msg = self._node().delete_robot_mapping(name)

            return jsonify({"success": ok_host and ok_robot, "message": msg})

        @app.post('/api/mappings/apply')
        def api_apply_mapping():
            """Apply a named mapping to the robot (load into robot memory).
            JSON body:
              {
                name,
                from_robot_db: true | false,
                // if from_robot_db=false, supply full data:
                buttons:   [{button, action, trigger_mode}, ...],
                joysticks: [{joystick, action}, ...],
              }
            """
            uid  = self._uid()
            if uid is None:
                return jsonify({"success": False, "error": "Not logged in"}), 401
            body = request.get_json(silent=True) or {}
            name          = (body.get('name') or '').strip()
            from_robot_db = bool(body.get('from_robot_db', False))

            if not name:
                return jsonify({"success": False, "error": "Mapping name required"}), 400
            if self._node() is None:
                return jsonify({"success": False, "error": "ROS2 node not running"}), 503

            btn_entries = []
            joy_entries = []
            actions_dict = {}

            if not from_robot_db:
                buttons_raw   = body.get('buttons')   or []
                joysticks_raw = body.get('joysticks') or []
                btn_entries = [
                    (b['button'], b['action'], b.get('trigger_mode', 'once'))
                    for b in buttons_raw if b.get('button') and b.get('action')
                ]
                joy_entries = [
                    (j['joystick'], j['action'])
                    for j in joysticks_raw if j.get('joystick') and j.get('action')
                ]
                # Include all actions referenced by this mapping
                all_actions  = self._db.get_all_actions(uid)
                needed_names = {e[1] for e in btn_entries} | {e[1] for e in joy_entries}
                actions_dict = {n: d for n, d in all_actions.items() if n in needed_names}

            ok, msg = self._node().apply_mapping(
                name, from_robot_db, btn_entries, joy_entries, actions_dict
            )
            return jsonify({"success": ok, "message": msg})

        # ── Recording APIs ─────────────────────────────────────

        @app.get('/api/recording/status')
        def api_recording_status():
            return jsonify(docker_node.RECORDING_STATE)

        @app.post('/api/recording/start')
        def api_recording_start():
            if self._node() is None:
                return jsonify({"started": False, "error": "ROS2 node not running"}), 503
            body      = request.get_json(silent=True) or {}
            user_name = (body.get('user_name') or self._uname()).strip()
            topics    = body.get('topics') or []
            if not topics:
                return jsonify({"started": False, "error": "No topics selected"}), 400
            try:
                session_dir = self._node().start_recording(user_name, topics)
            except Exception as e:
                return jsonify({"started": False, "error": str(e)}), 500
            if not session_dir:
                return jsonify({"started": False,
                                "error": "Could not create recording directory"}), 500
            return jsonify({"started": True, "dir": session_dir,
                            "dir_name": os.path.basename(session_dir)})

        @app.post('/api/recording/stop')
        def api_recording_stop():
            if self._node() is None:
                return jsonify({"stopped": False, "error": "ROS2 node not running"}), 503
            session_dir = docker_node.RECORDING_STATE.get('dir') or ''
            self._node().stop_recording()
            return jsonify({"stopped": True, "dir": session_dir,
                            "dir_name": os.path.basename(session_dir)})

        # ── LED APIs ───────────────────────────────────────────

        @app.get('/api/led/status')
        def api_led_status():
            with docker_node._led_lock:
                state = docker_node.LATEST_LED_STATE
            if state is None:
                return jsonify({"available": False})
            return jsonify({"available": True, "corners": state})

        @app.post('/api/led/get')
        def api_led_get():
            if self._node() is None:
                return jsonify({"success": False, "error": "ROS2 node not running"}), 503
            state, err = self._node().get_led_status()
            if state is None:
                return jsonify({"success": False, "error": err}), 500
            with docker_node._led_lock:
                docker_node.LATEST_LED_STATE = state
            return jsonify({"success": True, "corners": state})

        @app.post('/api/led/set')
        def api_led_set():
            if self._node() is None:
                return jsonify({"success": False, "error": "ROS2 node not running"}), 503
            body = request.get_json(silent=True) or {}
            values = {}
            for c in ['FL', 'FR', 'BL', 'BR']:
                if c not in body:
                    return jsonify({"success": False, "error": f"Missing corner: {c}"}), 400
                values[c] = {'mode': int(body[c].get('mode', 4)),
                             'color': int(body[c].get('color', 0))}
            ok, msg = self._node().set_led_status(values)
            if ok:
                with docker_node._led_lock:
                    docker_node.LATEST_LED_STATE = values
            self._node().record_led_command(values, ok, msg)
            return jsonify({"success": ok, "message": msg})

        # ── Recording schemes APIs ─────────────────────────────

        @app.post('/api/recording-schemes/save')
        def api_save_scheme():
            uid  = self._uid()
            body = request.get_json(silent=True) or {}
            name        = (body.get('name') or '').strip()
            topics      = body.get('topics') or []
            overwrite   = body.get('overwrite', False)
            destination = (body.get('destination') or 'host').strip()
            if uid is None:
                return jsonify({"success": False, "error": "Not logged in"}), 401
            if not name:
                return jsonify({"success": False, "error": "Name required"}), 400
            if not topics:
                return jsonify({"success": False, "error": "At least one topic required"}), 400

            saved_host  = False
            saved_robot = False
            robot_msg   = ''

            if destination in ('host', 'both'):
                if not overwrite:
                    existing = self._db.get_all_recording_schemes(uid)
                    if name in existing:
                        return jsonify({"success": False,
                            "error": f'Recording scheme "{name}" already exists in the host DB. Use overwrite to replace.'}), 409
                ok = self._db.save_recording_scheme(uid, name, topics)
                if not ok:
                    return jsonify({"success": False, "error": "Failed to save to host database"}), 500
                saved_host = True

            if destination in ('robot', 'both'):
                if self._node() is None:
                    robot_msg = 'ROS2 node not running — could not save to robot'
                else:
                    ok, msg = self._node().save_recording_scheme_to_robot(name, topics, overwrite=overwrite)
                    saved_robot = ok
                    robot_msg   = msg if not ok else ''
                    if not ok and msg and 'already exists' in msg:
                        return jsonify({"success": False, "error": msg}), 409

            return jsonify({
                "success":     saved_host or saved_robot,
                "saved_host":  saved_host,
                "saved_robot": saved_robot,
                "robot_msg":   robot_msg,
            })

        @app.post('/api/recording-schemes/delete')
        def api_delete_scheme():
            uid  = self._uid()
            body = request.get_json(silent=True) or {}
            name        = (body.get('name') or '').strip()
            destination = (body.get('destination') or 'host').strip()
            if uid is None:
                return jsonify({"success": False, "error": "Not logged in"}), 401
            if not name:
                return jsonify({"success": False, "error": "Name required"}), 400

            ok_host  = True
            ok_robot = True
            msg      = ''

            if destination in ('host', 'both'):
                ok_host = self._db.delete_recording_scheme(uid, name)

            if destination in ('robot', 'both'):
                if self._node() is None:
                    ok_robot = False
                    msg = 'ROS2 node not running'
                else:
                    ok_robot, msg = self._node().delete_robot_recording_scheme(name)

            return jsonify({"success": ok_host and ok_robot, "message": msg})

        # ── Page routes ────────────────────────────────────────

        @app.get("/")
        def index():
            return render_template("landing.html", msg="")

        @app.post("/save")
        def save():
            user_name = (request.form.get("user_name") or "").strip()
            if not user_name:
                return render_template("landing.html", msg="Please enter a user name.")
            # Resolve user in DB and store in docker_node global
            try:
                uid = self._db.get_or_create_user(user_name)
                docker_node.set_current_user(user_name, uid)
            except Exception as e:
                return render_template("landing.html",
                                       msg=f"Database error: {e}. Please try again.")
            return redirect(url_for("main_dashboard"))

        @app.get("/main-dashboard")
        def main_dashboard():
            if self._uid() is None:
                return redirect(url_for("index"))
            uid     = self._uid()
            host_schemes = self._db.get_all_recording_schemes(uid)
            topics       = docker_node.RECORDABLE_TOPICS
            return render_template("main_dashboard.html",
                                   user_name=self._uname(),
                                   controller_type=self.CONTROLLER_TYPE,
                                   host_recording_schemes=host_schemes,
                                   recordable_topics=topics)

        @app.get('/api/recording-schemes/robot')
        def api_get_robot_schemes():
            """Return recording schemes from the robot's DB (non-blocking async endpoint)."""
            if self._uid() is None:
                return jsonify({"available": False, "schemes": {}}), 401
            node = self._node()
            if node is None:
                return jsonify({"available": False, "schemes": {}})
            rs, err = node.get_robot_recording_schemes(self._uname())
            if rs is None:
                return jsonify({"available": False, "schemes": {}, "error": err})
            return jsonify({"available": True, "schemes": rs})

        @app.get("/button-mapping")
        def button_mapping():
            if self._uid() is None:
                return redirect(url_for("index"))
            uid     = self._uid()
            actions = self._db.get_all_actions(uid)
            # Strip internal DB id before exposing to the template / JS
            actions_view = {
                name: {'type': d['type'], 'tuples': d['tuples']}
                for name, d in actions.items()
            }
            return render_template(
                "button_mapping.html",
                buttons=self.CONTROLLER_BUTTONS,
                controller_type=self.CONTROLLER_TYPE,
                actions=actions_view,
                user_name=self._uname(),
                topic_schemas=TOPIC_SCHEMAS,
            )

        @app.get("/map-controls")
        def map_controls():
            if self._uid() is None:
                return redirect(url_for("index"))
            uid      = self._uid()
            actions  = self._db.get_all_actions(uid)
            btn_acts = {n: {'type': d['type']} for n, d in actions.items()
                        if d.get('type') in (self.ACTION_TYPE_BUTTON_ONCE, self.ACTION_TYPE_BUTTON_HOLD)}
            joy_acts = {n: {'type': d['type']} for n, d in actions.items()
                        if d.get('type') == self.ACTION_TYPE_JOYSTICK}
            return render_template(
                "map_controls.html",
                buttons=self.CONTROLLER_BUTTONS,
                joysticks=CONTROLLER_JOYSTICKS,
                controller_type=self.CONTROLLER_TYPE,
                button_actions=btn_acts,
                joystick_actions=joy_acts,
                user_name=self._uname(),
            )

    # ──────────────────────────────────────────────────────────
    # Run
    # ──────────────────────────────────────────────────────────

    def run(self, host="0.0.0.0", port=8080, debug=False):
        print("Starting Flask GUI server...")
        self.app.run(host=host, port=port, debug=debug)


if __name__ == "__main__":
    import sys

    _db_path = "/host_docs/actions.db"

    try:
        db = Database(_db_path)
    except DatabaseDirectoryError as e:
        print(
            f"\n  [FATAL] Database directory missing or could not be created.\n\n"
            f"  {e}\n\n"
            f"  Start the container with:\n\n"
            f"      ./start_docker.sh\n",
            flush=True,
        )
        sys.exit(1)
    except DatabasePermissionError as e:
        print(
            f"\n  [FATAL] Database directory is not writable.\n\n"
            f"  {e}\n\n"
            f"  Fix permissions:  chmod 755 ~/Documents\n",
            flush=True,
        )
        sys.exit(1)
    except DatabaseSchemaError as e:
        print(
            f"\n  [FATAL] Database schema is outdated or corrupt.\n\n"
            f"  {e}\n\n"
            f"  Delete the file and restart to create a fresh database.\n",
            flush=True,
        )
        sys.exit(1)
    except DatabaseConnectionError as e:
        print(
            f"\n  [FATAL] Could not connect to the database.\n\n"
            f"  {e}\n",
            flush=True,
        )
        sys.exit(1)

    flask_app = FlaskApp(db)
    flask_app.run()
