from flask import Flask, request, render_template, redirect, url_for, jsonify
import os
import threading
import json

from database import IDatabase
from database import Database
import database as _db_constants  # for ACTION_* / PUBLISH_* module-level constants
import docker_node  # single import — all globals accessed as docker_node.X

# ──────────────────────────────────────────────────────────────
# Controller / Button layout  (module-level — shared constants)
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
# Topic schemas  (unchanged from original)
# ──────────────────────────────────────────────────────────────

TOPIC_SCHEMAS = {
    '/cmd_vel': {
        'msg_type': 'geometry_msgs/msg/Twist',
        'description': 'Drive: robot velocity command (wheels)',
        'fields': [
            {'name': 'linear.x',  'label': 'linear.x  — forward / back (m/s)',     'type': 'float', 'default': '0.0', 'placeholder': '0.0', 'joystick_role': 'Y'},
            {'name': 'linear.y',  'label': 'linear.y  — strafe left / right (m/s)','type': 'float', 'default': '0.0', 'placeholder': '0.0', 'joystick_role': 'X'},
            {'name': 'linear.z',  'label': 'linear.z  — vertical (m/s)',            'type': 'float', 'default': '0.0', 'placeholder': '0.0'},
            {'name': 'angular.x', 'label': 'angular.x — roll (rad/s)',              'type': 'float', 'default': '0.0', 'placeholder': '0.0'},
            {'name': 'angular.y', 'label': 'angular.y — pitch (rad/s)',             'type': 'float', 'default': '0.0', 'placeholder': '0.0'},
            {'name': 'angular.z', 'label': 'angular.z — yaw / turn (rad/s)',        'type': 'float', 'default': '0.0', 'placeholder': '0.0', 'joystick_role': 'X'},
        ],
    },
    '/cmd_accessory_pos': {
        'msg_type': 'mecanumbot_msgs/msg/AccessMotorCmd',
        'description': 'Accessories: neck + both grabbers together',
        'fields': [
            {'name': 'n_pos',  'label': 'n_pos  — neck tilt (2.0 – 8.6)',         'type': 'float', 'default': '5.3',  'placeholder': '5.3',  'joystick_role': 'Y'},
            {'name': 'gl_pos', 'label': 'gl_pos — left grabber (1.6 – 8.54)',      'type': 'float', 'default': '5.12', 'placeholder': '5.12', 'joystick_role': 'X'},
            {'name': 'gr_pos', 'label': 'gr_pos — right grabber (1.6 – 8.54)',     'type': 'float', 'default': '5.12', 'placeholder': '5.12'},
        ],
    },
    '/cmd_accessory_pos [neck only]': {
        'msg_type': 'mecanumbot_msgs/msg/AccessMotorCmd',
        'description': 'Accessories: neck tilt only (grabbers stay at default)',
        '_topic_override': '/cmd_accessory_pos',
        'fields': [
            {'name': 'n_pos',  'label': 'n_pos — neck tilt (2.0 – 8.6)',           'type': 'float', 'default': '5.3',  'placeholder': '5.3',  'joystick_role': 'Y'},
            {'name': 'gl_pos', 'label': 'gl_pos — left grabber (fixed default)',    'type': 'float', 'default': '5.12', 'placeholder': '5.12', 'readonly': True},
            {'name': 'gr_pos', 'label': 'gr_pos — right grabber (fixed default)',   'type': 'float', 'default': '5.12', 'placeholder': '5.12', 'readonly': True},
        ],
    },
    '/cmd_accessory_pos [left grabber only]': {
        'msg_type': 'mecanumbot_msgs/msg/AccessMotorCmd',
        'description': 'Accessories: left grabber only (neck + right grabber stay at default)',
        '_topic_override': '/cmd_accessory_pos',
        'fields': [
            {'name': 'n_pos',  'label': 'n_pos  — neck tilt (fixed default)',       'type': 'float', 'default': '5.3',  'placeholder': '5.3',  'readonly': True},
            {'name': 'gl_pos', 'label': 'gl_pos — left grabber (1.6 – 8.54)',       'type': 'float', 'default': '5.12', 'placeholder': '5.12', 'joystick_role': 'Y'},
            {'name': 'gr_pos', 'label': 'gr_pos — right grabber (fixed default)',   'type': 'float', 'default': '5.12', 'placeholder': '5.12', 'readonly': True},
        ],
    },
    '/cmd_accessory_pos [right grabber only]': {
        'msg_type': 'mecanumbot_msgs/msg/AccessMotorCmd',
        'description': 'Accessories: right grabber only (neck + left grabber stay at default)',
        '_topic_override': '/cmd_accessory_pos',
        'fields': [
            {'name': 'n_pos',  'label': 'n_pos  — neck tilt (fixed default)',       'type': 'float', 'default': '5.3',  'placeholder': '5.3',  'readonly': True},
            {'name': 'gl_pos', 'label': 'gl_pos — left grabber (fixed default)',    'type': 'float', 'default': '5.12', 'placeholder': '5.12', 'readonly': True},
            {'name': 'gr_pos', 'label': 'gr_pos — right grabber (1.6 – 8.54)',      'type': 'float', 'default': '5.12', 'placeholder': '5.12', 'joystick_role': 'Y'},
        ],
    },
    '/cmd_accessory_pos [both grabbers]': {
        'msg_type': 'mecanumbot_msgs/msg/AccessMotorCmd',
        'description': 'Accessories: both grabbers move together (neck stays at default)',
        '_topic_override': '/cmd_accessory_pos',
        'fields': [
            {'name': 'n_pos',  'label': 'n_pos  — neck tilt (fixed default)',       'type': 'float', 'default': '5.3',  'placeholder': '5.3',  'readonly': True},
            {'name': 'gl_pos', 'label': 'gl_pos — left grabber (1.6 – 8.54)',       'type': 'float', 'default': '5.12', 'placeholder': '5.12', 'joystick_role': 'Y'},
            {'name': 'gr_pos', 'label': 'gr_pos — right grabber (mirrors gl_pos)',  'type': 'float', 'default': '5.12', 'placeholder': '5.12', 'joystick_role': 'Y'},
        ],
    },
    '/cmd_vel [stop]': {
        'msg_type': 'geometry_msgs/msg/Twist',
        'description': 'Drive: emergency stop — all velocities zeroed',
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
        'description': 'Accessories: reset neck + grabbers to neutral defaults',
        '_topic_override': '/cmd_accessory_pos',
        'fields': [
            {'name': 'n_pos',  'label': 'n_pos  — neck tilt (neutral 5.3)',        'type': 'float', 'default': '5.3',  'placeholder': '5.3',  'readonly': True},
            {'name': 'gl_pos', 'label': 'gl_pos — left grabber (neutral 5.12)',     'type': 'float', 'default': '5.12', 'placeholder': '5.12', 'readonly': True},
            {'name': 'gr_pos', 'label': 'gr_pos — right grabber (neutral 5.12)',    'type': 'float', 'default': '5.12', 'placeholder': '5.12', 'readonly': True},
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

        # ── Action type constants ──────────────────────────────
        self.ACTION_TYPE_BUTTON_ONCE = _db_constants.ACTION_BUTTON_ONCE
        self.ACTION_TYPE_BUTTON_HOLD = _db_constants.ACTION_BUTTON_HOLD
        self.ACTION_TYPE_JOYSTICK    = _db_constants.ACTION_JOYSTICK

        # ── Controller layout ──────────────────────────────────
        self.CONTROLLER_TYPE    = "Xbox 360"
        self.CONTROLLER_BUTTONS = XBOX360_BUTTONS.copy()
        print(f"Using controller layout: {self.CONTROLLER_TYPE}")

        # ── Flask app ──────────────────────────────────────────
        self.app = Flask(__name__)
        self._register_routes()

        # ── Start ROS2 node in background ──────────────────────
        self._ros_thread = threading.Thread(
            target=docker_node._start_docker_node, args=(self._db,), daemon=True
        )
        self._ros_thread.start()

    # ──────────────────────────────────────────────────────────
    # Internal helpers
    # ──────────────────────────────────────────────────────────

    def _node(self):
        return docker_node._docker_node_instance

    def _save_action_to_db(self, action_name, tuples, action_type=None):
        """Save action to DB and attempt to register it with the robot.

        Returns:
            (saved: bool, error: str, robot_warning: str)
            - saved=False means the DB write itself failed (hard error).
            - robot_warning is non-empty when the action was saved locally
              but the robot rejected it or was unreachable (soft warning).
        """
        if action_type is None:
            action_type = self.ACTION_TYPE_BUTTON_ONCE
        if self._node() is None:
            return False, 'ROS2 node is still starting up. Please wait a moment and try again.', ''

        ok = self._db.save_action(action_name, tuples, action_type)
        if not ok:
            return False, 'Failed to save action to database.', ''

        self._node().record_new_action(action_name, action_type, tuples)

        accepted, conflict_reason = self._node().register_action_with_robot(action_name, action_type, tuples)
        if not accepted:
            self._node().get_logger().warn(
                f'Action "{action_name}" saved to host DB but not registered with robot: {conflict_reason}')
            return True, '', conflict_reason or 'Robot did not accept the action.'

        return True, '', ''

    def _get_trigger_mode_for_action(self, action_name):
        all_actions = self._db.get_all_actions()
        data = all_actions.get(action_name)
        if data and data.get('type') == self.ACTION_TYPE_BUTTON_HOLD:
            return 'hold'
        return 'once'

    # ──────────────────────────────────────────────────────────
    # Route registration
    # ──────────────────────────────────────────────────────────

    def _register_routes(self):
        app = self.app

        # ── API routes ─────────────────────────────────────────

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

        @app.route('/api/sync-status')
        def api_sync_status():
            return jsonify(docker_node.LATEST_SYNC_RESULT)

        @app.route('/api/mapping-sync-status')
        def api_mapping_sync_status():
            return jsonify(docker_node.LATEST_MAPPING_SYNC_RESULT)

        @app.route('/api/robot-actions')
        def api_robot_actions():
            return jsonify(docker_node.ROBOT_ACTIONS)

        @app.post('/api/sync-actions')
        def api_trigger_sync():
            if self._node() is None:
                return jsonify({"triggered": False, "error": "ROS2 node not running"}), 503
            if not self._node()._robot_sync_lock.acquire(blocking=False):
                return jsonify({"triggered": False, "error": "Sync already in progress"}), 409
            def _do_sync():
                try:
                    self._node().sync_actions_to_robot()
                finally:
                    self._node()._robot_sync_lock.release()
            threading.Thread(target=_do_sync, daemon=True).start()
            return jsonify({"triggered": True})

        @app.get('/api/recording/status')
        def api_recording_status():
            return jsonify(docker_node.RECORDING_STATE)

        @app.get('/api/recording/topics')
        def api_recording_topics():
            return jsonify(docker_node.RECORDABLE_TOPICS)

        @app.post('/api/recording/start')
        def api_recording_start():
            if self._node() is None:
                return jsonify({"started": False, "error": "ROS2 node not running"}), 503
            body      = request.get_json(silent=True) or {}
            user_name = (body.get('user_name') or 'User').strip()
            topics    = body.get('topics') or []
            if not topics:
                return jsonify({"started": False, "error": "No topics selected"}), 400
            try:
                session_dir = self._node().start_recording(user_name, topics)
            except Exception as e:
                return jsonify({"started": False, "error": str(e)}), 500
            if not session_dir:
                return jsonify({"started": False,
                                "error": "Could not create recording directory — check that ~/Dokumentumok is mounted correctly"}), 500
            return jsonify({"started": True, "dir": session_dir, "dir_name": os.path.basename(session_dir)})

        @app.post('/api/recording/stop')
        def api_recording_stop():
            if self._node() is None:
                return jsonify({"stopped": False, "error": "ROS2 node not running"}), 503
            session_dir = docker_node.RECORDING_STATE.get('dir') or ''
            self._node().stop_recording()
            return jsonify({"stopped": True, "dir": session_dir, "dir_name": os.path.basename(session_dir)})

        @app.get('/api/opencr-state')
        def api_opencr_state():
            with docker_node._opencr_lock:
                state = docker_node.LATEST_OPENCR_STATE
            if state is None:
                return jsonify({"available": False})
            return jsonify({"available": True, **state})

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
                values[c] = {'mode': int(body[c].get('mode', 4)), 'color': int(body[c].get('color', 0))}
            ok, msg = self._node().set_led_status(values)
            if ok:
                with docker_node._led_lock:
                    docker_node.LATEST_LED_STATE = values
            self._node().record_led_command(values, ok, msg)
            return jsonify({"success": ok, "message": msg})

        # ── Page routes ────────────────────────────────────────

        @app.get("/")
        def index():
            return render_template("landing.html", msg="")

        @app.post("/save")
        def save():
            user_name = (request.form.get("user_name") or "").strip()
            if not user_name:
                return render_template("landing.html", msg="Please enter a user name.")
            return redirect(url_for("main_dashboard", user_name=user_name))

        @app.get("/main-dashboard")
        def main_dashboard():
            user_name = request.args.get("user_name", "User")
            return render_template("main_dashboard.html", user_name=user_name,
                                   controller_type=self.CONTROLLER_TYPE)

        @app.get("/button-mapping")
        def button_mapping():
            user_name = request.args.get("user_name", "User")
            actions   = self._db.get_all_actions()
            return render_template(
                "button_mapping.html",
                buttons=self.CONTROLLER_BUTTONS,
                controller_type=self.CONTROLLER_TYPE,
                mappings=actions,
                robot_actions=docker_node.ROBOT_ACTIONS,
                user_name=user_name,
                topic_schemas=TOPIC_SCHEMAS,
            )

        @app.post("/button-mapping/create")
        def create_button_mapping():
            action_name    = request.form.get("mapping_name", "").strip()
            tuple_list_raw = request.form.get("tuple_list", "").strip()
            action_type    = request.form.get("action_type", self.ACTION_TYPE_BUTTON_ONCE).strip()
            user_name      = request.form.get("user_name", "User").strip()

            if not action_name or not tuple_list_raw:
                return redirect(url_for("button_mapping", user_name=user_name))

            # Each line: TAB-delimited "topic\tmessage_json\tmsg_type\tpublish_type\tservice_name"
            tuples  = []
            row_idx = 0
            for line in tuple_list_raw.split('\n'):
                line = line.strip()
                if not line:
                    continue
                parts = [p.strip() for p in line.split('\t')]
                if len(parts) < 2:
                    continue
                topic        = parts[0]
                message      = parts[1]
                message_type = (parts[2] if len(parts) > 2 else '') or 'std_msgs/msg/String'
                publish_type = parts[3] if len(parts) > 3 else 'topic'
                service_name = parts[4] if len(parts) > 4 else ''

                if not topic and not service_name:
                    continue
                if publish_type == 'service' and not message:
                    continue

                try:
                    offset_x = float(request.form.get(f"offset_x_{row_idx}", "0.0") or "0.0")
                except ValueError:
                    offset_x = 0.0
                try:
                    offset_y = float(request.form.get(f"offset_y_{row_idx}", "0.0") or "0.0")
                except ValueError:
                    offset_y = 0.0

                tuples.append((topic, message, message_type, 1.0, 1.0, offset_x, offset_y,
                               publish_type, service_name))
                row_idx += 1

            if not tuples:
                return redirect(url_for("button_mapping", user_name=user_name))

            saved, error, robot_warning = self._save_action_to_db(action_name, tuples, action_type)
            if not saved:
                return render_template(
                    "button_mapping.html",
                    buttons=self.CONTROLLER_BUTTONS, controller_type=self.CONTROLLER_TYPE,
                    mappings=self._db.get_all_actions(), robot_actions=docker_node.ROBOT_ACTIONS,
                    user_name=user_name, topic_schemas=TOPIC_SCHEMAS,
                    error=error or "Failed to save action to database. Please try again."
                )
            if robot_warning:
                return render_template(
                    "button_mapping.html",
                    buttons=self.CONTROLLER_BUTTONS, controller_type=self.CONTROLLER_TYPE,
                    mappings=self._db.get_all_actions(), robot_actions=docker_node.ROBOT_ACTIONS,
                    user_name=user_name, topic_schemas=TOPIC_SCHEMAS,
                    robot_warning=f'Action "{action_name}" saved locally, but not sent to robot: {robot_warning}'
                )
            return redirect(url_for("button_mapping", user_name=user_name))

        @app.post("/button-mapping/delete")
        def delete_button_mapping():
            action_name = request.form.get("action_name", "").strip()
            user_name   = request.form.get("user_name", "User").strip()
            if action_name:
                self._db.delete_action(action_name)
            return redirect(url_for("button_mapping", user_name=user_name))

        @app.get("/map-controls")
        def map_controls():
            user_name        = request.args.get("user_name", "User")
            all_actions      = self._db.get_all_actions()
            button_types     = {self.ACTION_TYPE_BUTTON_ONCE, self.ACTION_TYPE_BUTTON_HOLD}
            button_actions   = {n: d for n, d in all_actions.items() if d.get('type') in button_types}
            joystick_actions = {n: d for n, d in all_actions.items() if d.get('type') == self.ACTION_TYPE_JOYSTICK}
            return render_template(
                "map_controls.html",
                buttons=self.CONTROLLER_BUTTONS,
                joysticks=CONTROLLER_JOYSTICKS,
                controller_type=self.CONTROLLER_TYPE,
                button_actions=button_actions,
                button_mappings=self._db.get_all_button_mappings(),
                joystick_actions=joystick_actions,
                joystick_mappings=self._db.get_all_joystick_mappings(),
                user_name=user_name,
                mapping_sync=docker_node.LATEST_MAPPING_SYNC_RESULT,
            )

        @app.post("/map-controls/save-button")
        def save_button_action_mapping():
            button_name = request.form.get("button_name", "").strip()
            action_name = request.form.get("action_name", "").strip()
            user_name   = request.form.get("user_name", "User").strip()
            if button_name and action_name:
                self._db.save_button_mapping(
                    button_name, action_name,
                    self._get_trigger_mode_for_action(action_name)
                )
                if self._node() is not None:
                    threading.Thread(
                        target=self._node().sync_mappings_to_robot, daemon=True
                    ).start()
            return redirect(url_for("map_controls", user_name=user_name))

        @app.post("/map-controls/save-joystick")
        def save_joystick_action_mapping():
            joystick_name = request.form.get("joystick_name", "").strip()
            action_name   = request.form.get("action_name", "").strip()
            user_name     = request.form.get("user_name", "User").strip()
            if joystick_name and action_name:
                self._db.save_joystick_mapping(joystick_name, action_name)
                if self._node() is not None:
                    threading.Thread(
                        target=self._node().sync_mappings_to_robot, daemon=True
                    ).start()
            return redirect(url_for("map_controls", user_name=user_name))

        @app.post("/map-controls/delete-joystick")
        def delete_joystick_action_mapping():
            joystick_name = request.form.get("joystick_name", "").strip()
            user_name     = request.form.get("user_name", "User").strip()
            if joystick_name:
                self._db.delete_joystick_mapping(joystick_name)
            return redirect(url_for("map_controls", user_name=user_name))

        @app.post("/map-controls/delete-button")
        def delete_button_action_mapping():
            button_name = request.form.get("button_name", "").strip()
            user_name   = request.form.get("user_name", "User").strip()
            if button_name:
                self._db.delete_button_mapping(button_name)
            return redirect(url_for("map_controls", user_name=user_name))

    # ──────────────────────────────────────────────────────────
    # Run
    # ──────────────────────────────────────────────────────────

    def run(self, host="0.0.0.0", port=8080, debug=False):
        print("Starting Flask GUI server...")
        self.app.run(host=host, port=port, debug=debug)


if __name__ == "__main__":
    db = Database("/host_docs/actions.db")
    flask_app = FlaskApp(db)
    flask_app.run()
