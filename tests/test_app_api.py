"""
test_app_api.py — Flask API integration tests.

Strategy
--------
* The real SQLite Database is replaced with an InMemoryDB mock that
  implements IDatabase and stores data in plain dicts.
* The docker_node module (which spins up a ROS2 thread) is replaced
  by a MockDockerNode / module-level monkeypatching before FlaskApp
  is imported / instantiated, so no real threads or ROS2 are needed.
* Every test group focuses on one API surface area.

Run with:
    pytest tests/test_app_api.py -v
"""

import json
import sys
import threading
import types
import unittest.mock as mock
from typing import Optional

import pytest

# ── sys.path is extended by conftest.py (shared/ and host/) ──────────────────

# ─────────────────────────────────────────────────────────────────────────────
# InMemoryDB — a lightweight IDatabase implementation for tests
# ─────────────────────────────────────────────────────────────────────────────

from database_interface import IDatabase, ACTION_BUTTON_ONCE


class InMemoryDB(IDatabase):
    """Minimal in-memory IDatabase for test isolation."""

    def __init__(self):
        self._users: dict[str, int]    = {}
        self._uid_seq                   = 1
        self._actions: dict[int, dict] = {}   # {uid: {name: {type, tuples}}}
        self._mappings: dict[int, dict] = {}  # {uid: {name: {buttons, joysticks}}}
        self._schemes: dict[int, dict]  = {}  # {uid: {name: [topics]}}

    # ── users ────────────────────────────────────────────────────────────────

    def get_or_create_user(self, username: str) -> int:
        if username not in self._users:
            self._users[username] = self._uid_seq
            self._uid_seq += 1
        return self._users[username]

    # ── actions ──────────────────────────────────────────────────────────────

    def save_action(self, user_id, name, tuples, action_type=ACTION_BUTTON_ONCE) -> bool:
        self._actions.setdefault(user_id, {})[name] = {
            "id": id(name), "type": action_type, "tuples": tuples
        }
        return True

    def get_all_actions(self, user_id, action_type=None) -> dict:
        all_a = dict(self._actions.get(user_id, {}))
        if action_type:
            all_a = {k: v for k, v in all_a.items() if v["type"] == action_type}
        return all_a

    def delete_action(self, user_id, name) -> bool:
        self._actions.get(user_id, {}).pop(name, None)
        return True

    def get_mappings_using_action(self, user_id, action_name) -> list:
        refs = []
        for mname, mdata in self._mappings.get(user_id, {}).items():
            for b in mdata["buttons"]:
                if b["action"] == action_name:
                    refs.append(mname)
                    break
            else:
                for j in mdata["joysticks"]:
                    if j["action"] == action_name:
                        refs.append(mname)
                        break
        return refs

    # ── mappings ─────────────────────────────────────────────────────────────

    def save_mapping(self, user_id, name, button_entries, joystick_entries) -> bool:
        buttons   = [{"button": b, "action": a, "trigger_mode": t} for b, a, t in button_entries]
        joysticks = [{"joystick": j, "action": a} for j, a in joystick_entries]
        self._mappings.setdefault(user_id, {})[name] = {
            "name": name, "buttons": buttons, "joysticks": joysticks
        }
        return True

    def get_all_mappings(self, user_id) -> list:
        return [{"name": k, **v} for k, v in self._mappings.get(user_id, {}).items()]

    def get_mapping(self, user_id, name) -> Optional[dict]:
        return self._mappings.get(user_id, {}).get(name)

    def delete_mapping(self, user_id, name) -> bool:
        self._mappings.get(user_id, {}).pop(name, None)
        return True

    # ── recording schemes ─────────────────────────────────────────────────────

    def save_recording_scheme(self, user_id, name, topics) -> bool:
        self._schemes.setdefault(user_id, {})[name] = list(topics)
        return True

    def get_all_recording_schemes(self, user_id) -> dict:
        return dict(self._schemes.get(user_id, {}))

    def delete_recording_scheme(self, user_id, name) -> bool:
        self._schemes.get(user_id, {}).pop(name, None)
        return True


# ─────────────────────────────────────────────────────────────────────────────
# MockRosNode — stands in for docker_node._docker_node_instance
# ─────────────────────────────────────────────────────────────────────────────

class MockRosNode:
    """
    Configurable mock of the ROS2 node.  All methods return sensible
    defaults; individual tests override what they need.
    """
    def __init__(self):
        self.online              = True
        self._robot_actions      = {}
        self._robot_mapping_names = []
        self._robot_mapping_detail = None
        self._robot_schemes      = {}

    def get_robot_actions(self, user_name):
        if not self.online:
            return None, "Robot offline"
        return self._robot_actions, None

    def get_robot_mapping_names(self):
        if not self.online:
            return [], "Robot offline"
        return list(self._robot_mapping_names), None

    def get_robot_mapping_details(self, name):
        if not self.online:
            return None, "Robot offline"
        if self._robot_mapping_detail is None:
            return None, "Not found"
        return self._robot_mapping_detail, None

    def save_action_to_robot(self, name, action_type, tuples, overwrite=False):
        return True, ""

    def save_mapping_to_robot(self, name, btn_entries, joy_entries, actions_needed, overwrite=False):
        return True, ""

    def apply_mapping(self, name, from_robot_db, btn_entries, joy_entries, actions_dict):
        return True, "ok", len(btn_entries), len(joy_entries)

    def delete_robot_action(self, name):
        return True, ""

    def delete_robot_mapping(self, name):
        return True, ""

    def get_robot_action_usages(self, name):
        return [], ""

    def get_robot_recording_schemes(self, user_name):
        return self._robot_schemes, None

    def save_recording_scheme_to_robot(self, name, topics, overwrite=False):
        return True, ""

    def delete_robot_recording_scheme(self, name):
        return True, ""

    def record_new_action(self, name, action_type, tuples):
        pass

    def record_led_command(self, values, ok, msg):
        pass

    def get_led_status(self):
        return None, "Not available"

    def set_led_status(self, values):
        return True, ""

    def start_recording(self, user_name, topics):
        return "/tmp/recording_session"

    def stop_recording(self):
        pass


# ─────────────────────────────────────────────────────────────────────────────
# Fixture: build a Flask test client with a fresh InMemoryDB + mock node
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture
def client(monkeypatch):
    """
    Returns (flask_test_client, in_memory_db, mock_ros_node).

    docker_node is injected as a synthetic module so no real ROS2
    thread is ever started.
    """
    db       = InMemoryDB()
    ros_node = MockRosNode()

    # Build a fake docker_node module
    fake_dn = types.ModuleType("docker_node")
    fake_dn.CURRENT_USER              = {"id": None, "name": None}
    fake_dn.ROBOT_ACTIVE              = True
    fake_dn.LATEST_CONTROLLER_STATUS  = {"connected": False, "controller_type": "Unknown"}
    fake_dn.RECORDING_STATE           = {"active": False, "dir": ""}
    fake_dn._opencr_lock              = threading.Lock()
    fake_dn.LATEST_OPENCR_STATE       = None
    fake_dn._led_lock                 = threading.Lock()
    fake_dn.LATEST_LED_STATE          = None
    fake_dn.RECORDABLE_TOPICS         = [{"topic": "/cmd_vel"}, {"topic": "/joy"}]
    fake_dn._docker_node_instance     = ros_node
    fake_dn._start_docker_node        = lambda db: None   # no-op
    fake_dn.set_current_user = lambda name, uid: fake_dn.CURRENT_USER.update({"id": uid, "name": name})

    # Inject before import
    sys.modules["docker_node"] = fake_dn

    # Import *after* patching so FlaskApp picks up the fake module
    import importlib
    import app as app_module
    importlib.reload(app_module)

    flask_app_obj = app_module.FlaskApp(db)
    flask_app_obj.app.config["TESTING"] = True
    test_client = flask_app_obj.app.test_client()

    return test_client, db, fake_dn, ros_node


@pytest.fixture
def authed_client(client):
    """client fixture with a logged-in user (uid=1, name='tester')."""
    test_client, db, fake_dn, ros_node = client
    uid = db.get_or_create_user("tester")
    fake_dn.CURRENT_USER = {"id": uid, "name": "tester"}
    return test_client, db, fake_dn, ros_node


# ─────────────────────────────────────────────────────────────────────────────
# Helper: create an action in the DB for the logged-in user
# ─────────────────────────────────────────────────────────────────────────────

def _add_action(db: InMemoryDB, uid: int, name: str,
                atype: str = ACTION_BUTTON_ONCE) -> None:
    db.save_action(uid, name, [("/cmd_vel", "{}", "geometry_msgs/msg/Twist")], atype)


# ─────────────────────────────────────────────────────────────────────────────
# Tests: /api/controller-status and /api/robot-status
# ─────────────────────────────────────────────────────────────────────────────

class TestStatusEndpoints:
    def test_controller_status(self, client):
        c, *_ = client
        r = c.get("/api/controller-status")
        assert r.status_code == 200
        assert "connected" in r.get_json()

    def test_robot_status_online(self, authed_client):
        c, db, fake_dn, ros_node = authed_client
        fake_dn.ROBOT_ACTIVE = True
        r = c.get("/api/robot-status")
        assert r.status_code == 200
        assert r.get_json()["available"] is True

    def test_robot_status_offline(self, authed_client):
        c, db, fake_dn, ros_node = authed_client
        fake_dn.ROBOT_ACTIVE = False
        r = c.get("/api/robot-status")
        data = r.get_json()
        assert data["available"] is False

    def test_robot_status_node_not_ready(self, authed_client):
        c, db, fake_dn, ros_node = authed_client
        fake_dn._docker_node_instance = None
        r = c.get("/api/robot-status")
        assert r.get_json()["available"] is False


# ─────────────────────────────────────────────────────────────────────────────
# Tests: /api/actions
# ─────────────────────────────────────────────────────────────────────────────

class TestActionsAPI:
    def test_get_actions_no_login_returns_empty(self, client):
        c, *_ = client
        r = c.get("/api/actions")
        assert r.status_code == 200
        assert r.get_json() == []

    def test_get_host_actions(self, authed_client):
        c, db, fake_dn, _ = authed_client
        uid = fake_dn.CURRENT_USER["id"]
        _add_action(db, uid, "Forward")
        r = c.get("/api/actions?source=host")
        assert r.status_code == 200
        names = [a["name"] for a in r.get_json()]
        assert "Forward" in names

    def test_get_robot_actions_node_offline(self, authed_client):
        c, db, fake_dn, ros_node = authed_client
        fake_dn._docker_node_instance = None
        r = c.get("/api/actions?source=robot")
        assert r.status_code == 503

    def test_get_robot_actions_returns_list(self, authed_client):
        c, db, fake_dn, ros_node = authed_client
        ros_node._robot_actions = {"Drive": {"type": "joystick"}}
        r = c.get("/api/actions?source=robot")
        assert r.status_code == 200
        names = [a["name"] for a in r.get_json()]
        assert "Drive" in names

    def test_save_action_host(self, authed_client):
        c, db, fake_dn, _ = authed_client
        uid = fake_dn.CURRENT_USER["id"]
        payload = {
            "name": "Spin",
            "action_type": "button_once",
            "tuples": [["/cmd_vel", "{}", "geometry_msgs/msg/Twist"]],
            "destination": "host",
        }
        r = c.post("/api/actions/save",
                   data=json.dumps(payload),
                   content_type="application/json")
        assert r.status_code == 200
        assert r.get_json()["success"] is True
        assert "Spin" in db.get_all_actions(uid)

    def test_save_action_conflict_returns_409(self, authed_client):
        c, db, fake_dn, _ = authed_client
        uid = fake_dn.CURRENT_USER["id"]
        _add_action(db, uid, "Dup")
        payload = {
            "name": "Dup",
            "action_type": "button_once",
            "tuples": [["/t", "{}", "std_msgs/msg/String"]],
            "destination": "host",
        }
        r = c.post("/api/actions/save",
                   data=json.dumps(payload),
                   content_type="application/json")
        assert r.status_code == 409

    def test_save_action_overwrite(self, authed_client):
        c, db, fake_dn, _ = authed_client
        uid = fake_dn.CURRENT_USER["id"]
        _add_action(db, uid, "Dup")
        payload = {
            "name": "Dup",
            "action_type": "button_hold",
            "tuples": [["/t", "{}", "std_msgs/msg/String"]],
            "destination": "host",
            "overwrite": True,
        }
        r = c.post("/api/actions/save",
                   data=json.dumps(payload),
                   content_type="application/json")
        assert r.status_code == 200
        assert r.get_json()["success"] is True

    def test_save_action_missing_name_returns_400(self, authed_client):
        c, *_ = authed_client
        r = c.post("/api/actions/save",
                   data=json.dumps({"tuples": [["/t", "{}"]]}),
                   content_type="application/json")
        assert r.status_code == 400

    def test_save_action_missing_tuples_returns_400(self, authed_client):
        c, *_ = authed_client
        r = c.post("/api/actions/save",
                   data=json.dumps({"name": "X", "destination": "host"}),
                   content_type="application/json")
        assert r.status_code == 400

    def test_save_action_not_logged_in_returns_401(self, client):
        c, *_ = client
        r = c.post("/api/actions/save",
                   data=json.dumps({"name": "X", "tuples": [["/t", "{}"]]}),
                   content_type="application/json")
        assert r.status_code == 401

    def test_delete_action(self, authed_client):
        c, db, fake_dn, _ = authed_client
        uid = fake_dn.CURRENT_USER["id"]
        _add_action(db, uid, "ToDelete")
        r = c.post("/api/actions/delete",
                   data=json.dumps({"name": "ToDelete", "destination": "host"}),
                   content_type="application/json")
        assert r.status_code == 200
        assert r.get_json()["success"] is True
        assert "ToDelete" not in db.get_all_actions(uid)

    def test_delete_action_not_logged_in(self, client):
        c, *_ = client
        r = c.post("/api/actions/delete",
                   data=json.dumps({"name": "X"}),
                   content_type="application/json")
        assert r.status_code == 401

    def test_action_usages(self, authed_client):
        c, db, fake_dn, _ = authed_client
        uid = fake_dn.CURRENT_USER["id"]
        _add_action(db, uid, "Honk")
        db.save_mapping(uid, "MapA", [("A", "Honk", "once")], [])
        r = c.get("/api/actions/usages?name=Honk&destination=host")
        assert r.status_code == 200
        data = r.get_json()
        assert "MapA" in data["host_mappings"]

    def test_action_usages_missing_name_returns_400(self, authed_client):
        c, *_ = authed_client
        r = c.get("/api/actions/usages")
        assert r.status_code == 400


# ─────────────────────────────────────────────────────────────────────────────
# Tests: /api/mappings
# ─────────────────────────────────────────────────────────────────────────────

class TestMappingsAPI:
    def test_get_mappings_empty(self, authed_client):
        c, *_ = authed_client
        r = c.get("/api/mappings")
        assert r.status_code == 200
        assert r.get_json() == []

    def test_get_mappings_no_login_returns_empty(self, client):
        c, *_ = client
        r = c.get("/api/mappings")
        assert r.status_code == 200
        assert r.get_json() == []

    def test_save_mapping_host(self, authed_client):
        c, db, fake_dn, _ = authed_client
        uid = fake_dn.CURRENT_USER["id"]
        _add_action(db, uid, "Drive")
        payload = {
            "name": "Teleop",
            "destination": "host",
            "buttons": [{"button": "A", "action": "Drive", "trigger_mode": "once"}],
            "joysticks": [],
        }
        r = c.post("/api/mappings/save",
                   data=json.dumps(payload),
                   content_type="application/json")
        assert r.status_code == 200
        d = r.get_json()
        assert d["success"] is True
        assert d["saved_host"] is True

    def test_save_mapping_conflict_returns_409(self, authed_client):
        c, db, fake_dn, _ = authed_client
        uid = fake_dn.CURRENT_USER["id"]
        _add_action(db, uid, "Drive")
        db.save_mapping(uid, "Dup", [("A", "Drive", "once")], [])
        payload = {
            "name": "Dup",
            "destination": "host",
            "buttons": [{"button": "A", "action": "Drive", "trigger_mode": "once"}],
            "joysticks": [],
        }
        r = c.post("/api/mappings/save",
                   data=json.dumps(payload),
                   content_type="application/json")
        assert r.status_code == 409

    def test_save_mapping_overwrite(self, authed_client):
        c, db, fake_dn, _ = authed_client
        uid = fake_dn.CURRENT_USER["id"]
        _add_action(db, uid, "Drive")
        db.save_mapping(uid, "Dup", [("A", "Drive", "once")], [])
        payload = {
            "name": "Dup",
            "destination": "host",
            "buttons": [{"button": "B", "action": "Drive", "trigger_mode": "once"}],
            "joysticks": [],
            "overwrite": True,
        }
        r = c.post("/api/mappings/save",
                   data=json.dumps(payload),
                   content_type="application/json")
        assert r.status_code == 200
        assert r.get_json()["success"] is True

    def test_save_mapping_missing_name_returns_400(self, authed_client):
        c, *_ = authed_client
        r = c.post("/api/mappings/save",
                   data=json.dumps({"destination": "host", "buttons": [], "joysticks": []}),
                   content_type="application/json")
        assert r.status_code == 400

    def test_save_mapping_not_logged_in(self, client):
        c, *_ = client
        r = c.post("/api/mappings/save",
                   data=json.dumps({"name": "X", "destination": "host"}),
                   content_type="application/json")
        assert r.status_code == 401

    def test_delete_mapping(self, authed_client):
        c, db, fake_dn, _ = authed_client
        uid = fake_dn.CURRENT_USER["id"]
        _add_action(db, uid, "Drive")
        db.save_mapping(uid, "ToDelete", [("A", "Drive", "once")], [])
        r = c.post("/api/mappings/delete",
                   data=json.dumps({"name": "ToDelete", "destination": "host"}),
                   content_type="application/json")
        assert r.status_code == 200
        assert r.get_json()["success"] is True
        assert db.get_mapping(uid, "ToDelete") is None

    def test_delete_mapping_not_logged_in(self, client):
        c, *_ = client
        r = c.post("/api/mappings/delete",
                   data=json.dumps({"name": "X"}),
                   content_type="application/json")
        assert r.status_code == 401

    def test_delete_mapping_missing_name_returns_400(self, authed_client):
        c, *_ = authed_client
        r = c.post("/api/mappings/delete",
                   data=json.dumps({"destination": "host"}),
                   content_type="application/json")
        assert r.status_code == 400


# ─────────────────────────────────────────────────────────────────────────────
# Tests: /api/mappings/apply
# ─────────────────────────────────────────────────────────────────────────────

class TestApplyMappingAPI:
    def test_apply_host_mapping(self, authed_client):
        c, db, fake_dn, ros_node = authed_client
        uid = fake_dn.CURRENT_USER["id"]
        _add_action(db, uid, "Drive")
        db.save_mapping(uid, "TeleOp", [("A", "Drive", "once")], [])
        m = db.get_mapping(uid, "TeleOp")
        payload = {
            "name": "TeleOp",
            "from_robot_db": False,
            "buttons":   m["buttons"],
            "joysticks": m["joysticks"],
        }
        r = c.post("/api/mappings/apply",
                   data=json.dumps(payload),
                   content_type="application/json")
        assert r.status_code == 200
        d = r.get_json()
        assert d["success"] is True

    def test_apply_robot_mapping(self, authed_client):
        c, db, fake_dn, ros_node = authed_client
        payload = {"name": "RobotMap", "from_robot_db": True}
        r = c.post("/api/mappings/apply",
                   data=json.dumps(payload),
                   content_type="application/json")
        assert r.status_code == 200
        assert r.get_json()["success"] is True

    def test_apply_missing_name_returns_400(self, authed_client):
        c, *_ = authed_client
        r = c.post("/api/mappings/apply",
                   data=json.dumps({"from_robot_db": False}),
                   content_type="application/json")
        assert r.status_code == 400

    def test_apply_node_offline_returns_503(self, authed_client):
        c, db, fake_dn, _ = authed_client
        fake_dn._docker_node_instance = None
        r = c.post("/api/mappings/apply",
                   data=json.dumps({"name": "X", "from_robot_db": True}),
                   content_type="application/json")
        assert r.status_code == 503

    def test_apply_not_logged_in(self, client):
        c, *_ = client
        r = c.post("/api/mappings/apply",
                   data=json.dumps({"name": "X", "from_robot_db": True}),
                   content_type="application/json")
        assert r.status_code == 401


# ─────────────────────────────────────────────────────────────────────────────
# Tests: /api/robot/mapping-names and /api/robot/mapping-details
# ─────────────────────────────────────────────────────────────────────────────

class TestRobotMappingEndpoints:
    def test_mapping_names_node_offline(self, authed_client):
        c, db, fake_dn, _ = authed_client
        fake_dn._docker_node_instance = None
        r = c.get("/api/robot/mapping-names")
        assert r.status_code == 503

    def test_mapping_names_success(self, authed_client):
        c, db, fake_dn, ros_node = authed_client
        ros_node._robot_mapping_names = ["Map1", "Map2"]
        r = c.get("/api/robot/mapping-names")
        assert r.status_code == 200
        d = r.get_json()
        assert d["success"] is True
        assert "Map1" in d["names"]

    def test_mapping_details_node_offline(self, authed_client):
        c, db, fake_dn, _ = authed_client
        fake_dn._docker_node_instance = None
        r = c.get("/api/robot/mapping-details/Map1")
        assert r.status_code == 503

    def test_mapping_details_not_found(self, authed_client):
        c, db, fake_dn, ros_node = authed_client
        ros_node._robot_mapping_detail = None  # get_robot_mapping_details returns error
        r = c.get("/api/robot/mapping-details/Ghost")
        assert r.status_code == 200
        assert r.get_json()["success"] is False

    def test_mapping_details_found(self, authed_client):
        c, db, fake_dn, ros_node = authed_client
        ros_node._robot_mapping_detail = {
            "name": "Map1",
            "buttons": [{"button": "A", "action": "Drive", "trigger_mode": "once"}],
            "joysticks": [],
        }
        r = c.get("/api/robot/mapping-details/Map1")
        assert r.status_code == 200
        d = r.get_json()
        assert d["success"] is True
        assert d["mapping"]["name"] == "Map1"


# ─────────────────────────────────────────────────────────────────────────────
# Tests: recording schemes API
# ─────────────────────────────────────────────────────────────────────────────

class TestRecordingSchemesAPI:
    def test_save_scheme_host(self, authed_client):
        c, db, fake_dn, _ = authed_client
        uid = fake_dn.CURRENT_USER["id"]
        payload = {
            "name": "Full", "topics": ["/cmd_vel", "/joy"],
            "destination": "host",
        }
        r = c.post("/api/recording-schemes/save",
                   data=json.dumps(payload),
                   content_type="application/json")
        assert r.status_code == 200
        assert r.get_json()["success"] is True
        assert "Full" in db.get_all_recording_schemes(uid)

    def test_save_scheme_conflict_returns_409(self, authed_client):
        c, db, fake_dn, _ = authed_client
        uid = fake_dn.CURRENT_USER["id"]
        db.save_recording_scheme(uid, "Dup", ["/cmd_vel"])
        payload = {"name": "Dup", "topics": ["/joy"], "destination": "host"}
        r = c.post("/api/recording-schemes/save",
                   data=json.dumps(payload),
                   content_type="application/json")
        assert r.status_code == 409

    def test_save_scheme_overwrite(self, authed_client):
        c, db, fake_dn, _ = authed_client
        uid = fake_dn.CURRENT_USER["id"]
        db.save_recording_scheme(uid, "Dup", ["/old"])
        payload = {
            "name": "Dup", "topics": ["/new"],
            "destination": "host", "overwrite": True,
        }
        r = c.post("/api/recording-schemes/save",
                   data=json.dumps(payload),
                   content_type="application/json")
        assert r.status_code == 200

    def test_save_scheme_missing_name_returns_400(self, authed_client):
        c, *_ = authed_client
        r = c.post("/api/recording-schemes/save",
                   data=json.dumps({"topics": ["/t"]}),
                   content_type="application/json")
        assert r.status_code == 400

    def test_save_scheme_missing_topics_returns_400(self, authed_client):
        c, *_ = authed_client
        r = c.post("/api/recording-schemes/save",
                   data=json.dumps({"name": "S", "topics": []}),
                   content_type="application/json")
        assert r.status_code == 400

    def test_delete_scheme(self, authed_client):
        c, db, fake_dn, _ = authed_client
        uid = fake_dn.CURRENT_USER["id"]
        db.save_recording_scheme(uid, "Del", ["/t"])
        r = c.post("/api/recording-schemes/delete",
                   data=json.dumps({"name": "Del", "destination": "host"}),
                   content_type="application/json")
        assert r.status_code == 200
        assert r.get_json()["success"] is True
        assert "Del" not in db.get_all_recording_schemes(uid)

    def test_get_robot_schemes(self, authed_client):
        c, db, fake_dn, ros_node = authed_client
        ros_node._robot_schemes = {"RobScheme": ["/cmd_vel"]}
        r = c.get("/api/recording-schemes/robot")
        assert r.status_code == 200
        d = r.get_json()
        assert d["available"] is True
        assert "RobScheme" in d["schemes"]

    def test_get_robot_schemes_node_offline(self, authed_client):
        c, db, fake_dn, ros_node = authed_client
        fake_dn._docker_node_instance = None
        r = c.get("/api/recording-schemes/robot")
        assert r.status_code == 200
        assert r.get_json()["available"] is False


# ─────────────────────────────────────────────────────────────────────────────
# Tests: recording start / stop
# ─────────────────────────────────────────────────────────────────────────────

class TestRecordingAPI:
    def test_recording_status(self, client):
        c, *_ = client
        r = c.get("/api/recording/status")
        assert r.status_code == 200

    def test_start_recording(self, authed_client):
        c, db, fake_dn, _ = authed_client
        r = c.post("/api/recording/start",
                   data=json.dumps({"topics": ["/cmd_vel"]}),
                   content_type="application/json")
        assert r.status_code == 200
        assert r.get_json()["started"] is True

    def test_start_recording_no_topics_returns_400(self, authed_client):
        c, *_ = authed_client
        r = c.post("/api/recording/start",
                   data=json.dumps({"topics": []}),
                   content_type="application/json")
        assert r.status_code == 400

    def test_start_recording_node_offline(self, authed_client):
        c, db, fake_dn, _ = authed_client
        fake_dn._docker_node_instance = None
        r = c.post("/api/recording/start",
                   data=json.dumps({"topics": ["/cmd_vel"]}),
                   content_type="application/json")
        assert r.status_code == 503

    def test_stop_recording(self, authed_client):
        c, db, fake_dn, _ = authed_client
        r = c.post("/api/recording/stop")
        assert r.status_code == 200
        assert r.get_json()["stopped"] is True


# ─────────────────────────────────────────────────────────────────────────────
# Tests: page routes
# ─────────────────────────────────────────────────────────────────────────────

class TestPageRoutes:
    def test_landing_page(self, client):
        c, *_ = client
        r = c.get("/")
        assert r.status_code == 200

    def test_main_dashboard_redirects_when_not_logged_in(self, client):
        c, *_ = client
        r = c.get("/main-dashboard")
        assert r.status_code in (301, 302)

    def test_main_dashboard_accessible_when_logged_in(self, authed_client):
        c, *_ = authed_client
        r = c.get("/main-dashboard")
        assert r.status_code == 200

    def test_button_mapping_redirects_when_not_logged_in(self, client):
        c, *_ = client
        r = c.get("/button-mapping")
        assert r.status_code in (301, 302)

    def test_button_mapping_accessible_when_logged_in(self, authed_client):
        c, *_ = authed_client
        r = c.get("/button-mapping")
        assert r.status_code == 200

    def test_map_controls_redirects_when_not_logged_in(self, client):
        c, *_ = client
        r = c.get("/map-controls")
        assert r.status_code in (301, 302)

    def test_map_controls_accessible_when_logged_in(self, authed_client):
        c, *_ = authed_client
        r = c.get("/map-controls")
        assert r.status_code == 200

    def test_save_redirects_to_dashboard(self, authed_client):
        c, db, fake_dn, _ = authed_client
        r = c.post("/save", data={"user_name": "tester"})
        # Should redirect to /main-dashboard
        assert r.status_code in (301, 302)
        assert b"main-dashboard" in r.data or "main-dashboard" in (r.headers.get("Location") or "")

    def test_save_empty_name_shows_error(self, client):
        c, *_ = client
        r = c.post("/save", data={"user_name": ""})
        assert r.status_code == 200
        assert b"Please enter" in r.data


# ─────────────────────────────────────────────────────────────────────────────
# Tests: /api/opencr-state and /api/led/*
# ─────────────────────────────────────────────────────────────────────────────

class TestLedAndOpencrAPI:
    def test_opencr_state_unavailable(self, client):
        c, db, fake_dn, _ = client
        fake_dn.LATEST_OPENCR_STATE = None
        r = c.get("/api/opencr-state")
        assert r.status_code == 200
        assert r.get_json()["available"] is False

    def test_opencr_state_available(self, client):
        c, db, fake_dn, _ = client
        fake_dn.LATEST_OPENCR_STATE = {"voltage": 12.0}
        r = c.get("/api/opencr-state")
        assert r.status_code == 200
        d = r.get_json()
        assert d["available"] is True
        assert d["voltage"] == 12.0

    def test_led_status_unavailable(self, client):
        c, db, fake_dn, _ = client
        fake_dn.LATEST_LED_STATE = None
        r = c.get("/api/led/status")
        assert r.status_code == 200
        assert r.get_json()["available"] is False

    def test_led_set_missing_corner_returns_400(self, authed_client):
        c, *_ = authed_client
        # Only 3 of 4 corners supplied
        payload = {
            "FL": {"mode": 1, "color": 0},
            "FR": {"mode": 1, "color": 0},
            "BL": {"mode": 1, "color": 0},
            # BR missing
        }
        r = c.post("/api/led/set",
                   data=json.dumps(payload),
                   content_type="application/json")
        assert r.status_code == 400

    def test_led_set_success(self, authed_client):
        c, db, fake_dn, ros_node = authed_client
        payload = {k: {"mode": 1, "color": 0} for k in ("FL", "FR", "BL", "BR")}
        r = c.post("/api/led/set",
                   data=json.dumps(payload),
                   content_type="application/json")
        assert r.status_code == 200
        assert r.get_json()["success"] is True


# ─────────────────────────────────────────────────────────────────────────────
# Tests: controller button list — live controller type detection
# ─────────────────────────────────────────────────────────────────────────────

class TestControllerButtonList:
    """
    Verify that _get_controller_buttons() and the pages that use it return
    the correct button set depending on the live controller type reported
    by LATEST_CONTROLLER_STATUS.
    """

    def _set_controller_type(self, fake_dn, ctype):
        fake_dn.LATEST_CONTROLLER_STATUS = {
            "connected": ctype != "Unknown",
            "controller_type": ctype,
        }

    # ── unit-level: _get_controller_buttons() ────────────────────────────────

    def test_get_controller_buttons_xbox360(self, authed_client):
        _, _, fake_dn, _ = authed_client
        self._set_controller_type(fake_dn, "Controller 360")
        import app as app_module
        buttons = app_module._get_controller_buttons()
        assert "XBOX" in buttons
        assert "HOME" not in buttons

    def test_get_controller_buttons_generic(self, authed_client):
        _, _, fake_dn, _ = authed_client
        self._set_controller_type(fake_dn, "Generic")
        import app as app_module
        buttons = app_module._get_controller_buttons()
        assert "HOME" in buttons
        assert "XBOX" not in buttons

    def test_get_controller_buttons_unknown_falls_back_to_generic(self, authed_client):
        _, _, fake_dn, _ = authed_client
        self._set_controller_type(fake_dn, "Unknown")
        import app as app_module
        buttons = app_module._get_controller_buttons()
        assert "HOME" in buttons

    def test_get_controller_buttons_offline_falls_back_to_generic(self, authed_client):
        _, _, fake_dn, _ = authed_client
        fake_dn.LATEST_CONTROLLER_STATUS = {"connected": False}  # no controller_type key
        import app as app_module
        buttons = app_module._get_controller_buttons()
        assert "HOME" in buttons

    # ── page-level: button list appears in rendered HTML ─────────────────────

    def test_map_controls_shows_xbox360_buttons(self, authed_client):
        c, _, fake_dn, _ = authed_client
        self._set_controller_type(fake_dn, "Controller 360")
        r = c.get("/map-controls")
        assert r.status_code == 200
        assert b'"XBOX"' in r.data
        assert b'"HOME"' not in r.data

    def test_map_controls_shows_generic_buttons(self, authed_client):
        c, _, fake_dn, _ = authed_client
        self._set_controller_type(fake_dn, "Generic")
        r = c.get("/map-controls")
        assert r.status_code == 200
        assert b'"HOME"' in r.data
        assert b'"XBOX"' not in r.data

    def test_button_mapping_shows_xbox360_buttons(self, authed_client):
        # button_mapping page defines actions, not button assignments —
        # the buttons list is passed but not rendered into JS on this page.
        # Just verify the page loads successfully for both controller types.
        c, _, fake_dn, _ = authed_client
        self._set_controller_type(fake_dn, "Controller 360")
        r = c.get("/button-mapping")
        assert r.status_code == 200

    def test_button_mapping_shows_generic_buttons(self, authed_client):
        c, _, fake_dn, _ = authed_client
        self._set_controller_type(fake_dn, "Generic")
        r = c.get("/button-mapping")
        assert r.status_code == 200

    def test_map_controls_offline_shows_generic_fallback(self, authed_client):
        c, _, fake_dn, _ = authed_client
        fake_dn.LATEST_CONTROLLER_STATUS = {"connected": False}
        r = c.get("/map-controls")
        assert r.status_code == 200
        assert b'"HOME"' in r.data
