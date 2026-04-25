"""
test_database.py — unit tests for the SQLite Database class.

Covers:
  * User management (get_or_create_user)
  * Actions: save, get_all, filter by type, delete, cascade behaviour
  * Mappings: save, get_all, get single, delete, action-name references
  * get_mappings_using_action
  * Recording schemes: save, get_all, delete
  * Schema validation (DatabaseSchemaError on corrupt DB)
  * Partial tuple fields (default values)
"""

import os
import sqlite3
import tempfile
import pytest

from database import (
    Database,
    DatabaseDirectoryError,
    DatabasePermissionError,
    DatabaseSchemaError,
    DatabaseConnectionError,
)
from database_interface import ACTION_BUTTON_ONCE, ACTION_BUTTON_HOLD, ACTION_JOYSTICK

# ─────────────────────────────────────────────────────────────────────────────
# Fixtures
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture
def db(tmp_path):
    """A fresh in-memory-style Database backed by a temp file."""
    return Database(str(tmp_path / "test.db"))


@pytest.fixture
def uid(db):
    """A pre-created user in the test database."""
    return db.get_or_create_user("alice")


@pytest.fixture
def action_name(db, uid):
    """A pre-saved button_once action for convenience."""
    name = "Move Forward"
    tuples = [("/cmd_vel", '{"linear":{"x":1.0}}', "geometry_msgs/msg/Twist")]
    db.save_action(uid, name, tuples, ACTION_BUTTON_ONCE)
    return name


# ─────────────────────────────────────────────────────────────────────────────
# User management
# ─────────────────────────────────────────────────────────────────────────────

class TestUsers:
    def test_create_new_user(self, db):
        uid = db.get_or_create_user("bob")
        assert isinstance(uid, int)
        assert uid > 0

    def test_create_returns_same_id(self, db):
        uid1 = db.get_or_create_user("charlie")
        uid2 = db.get_or_create_user("charlie")
        assert uid1 == uid2

    def test_different_users_get_different_ids(self, db):
        uid1 = db.get_or_create_user("dave")
        uid2 = db.get_or_create_user("eve")
        assert uid1 != uid2

    def test_uid_is_isolated_between_users(self, db):
        uid_a = db.get_or_create_user("user_a")
        uid_b = db.get_or_create_user("user_b")
        db.save_action(uid_a, "A Action", [("/t", "{}", "std_msgs/msg/String")])
        # user_b should see no actions
        assert db.get_all_actions(uid_b) == {}


# ─────────────────────────────────────────────────────────────────────────────
# Actions
# ─────────────────────────────────────────────────────────────────────────────

class TestActions:
    def test_save_and_retrieve_action(self, db, uid):
        tuples = [("/cmd_vel", '{"linear":{"x":1.0}}', "geometry_msgs/msg/Twist")]
        ok = db.save_action(uid, "Drive", tuples, ACTION_BUTTON_ONCE)
        assert ok is True
        actions = db.get_all_actions(uid)
        assert "Drive" in actions
        assert actions["Drive"]["type"] == ACTION_BUTTON_ONCE
        assert len(actions["Drive"]["tuples"]) == 1

    def test_save_multiple_tuples(self, db, uid):
        tuples = [
            ("/t1", "{}", "std_msgs/msg/String"),
            ("/t2", "{}", "std_msgs/msg/String"),
            ("/t3", "{}", "std_msgs/msg/String"),
        ]
        db.save_action(uid, "Multi", tuples)
        actions = db.get_all_actions(uid)
        assert len(actions["Multi"]["tuples"]) == 3

    def test_save_overwrites_existing_action(self, db, uid):
        db.save_action(uid, "Act", [("/old", "{}", "std_msgs/msg/String")])
        db.save_action(uid, "Act", [("/new", "{}", "std_msgs/msg/String")])
        actions = db.get_all_actions(uid)
        assert actions["Act"]["tuples"][0][0] == "/new"
        assert len(actions["Act"]["tuples"]) == 1

    def test_save_updates_action_type(self, db, uid):
        db.save_action(uid, "Act", [("/t", "{}", "std_msgs/msg/String")], ACTION_BUTTON_ONCE)
        db.save_action(uid, "Act", [("/t", "{}", "std_msgs/msg/String")], ACTION_BUTTON_HOLD)
        actions = db.get_all_actions(uid)
        assert actions["Act"]["type"] == ACTION_BUTTON_HOLD

    def test_filter_by_action_type(self, db, uid):
        db.save_action(uid, "BtnOnce", [("/t", "{}", "std_msgs/msg/String")], ACTION_BUTTON_ONCE)
        db.save_action(uid, "BtnHold", [("/t", "{}", "std_msgs/msg/String")], ACTION_BUTTON_HOLD)
        db.save_action(uid, "Joy",     [("/t", "{}", "std_msgs/msg/String")], ACTION_JOYSTICK)
        btn_only = db.get_all_actions(uid, ACTION_BUTTON_ONCE)
        assert "BtnOnce" in btn_only
        assert "BtnHold" not in btn_only
        assert "Joy"     not in btn_only

    def test_delete_action(self, db, uid, action_name):
        ok = db.delete_action(uid, action_name)
        assert ok is True
        assert action_name not in db.get_all_actions(uid)

    def test_delete_nonexistent_action_returns_true(self, db, uid):
        # Deleting something that doesn't exist should be a no-op, not an error
        assert db.delete_action(uid, "ghost") is True

    def test_tuple_field_defaults(self, db, uid):
        """Only topic + message supplied — all remaining fields must use defaults."""
        db.save_action(uid, "Minimal", [("/t", "{}")])
        actions = db.get_all_actions(uid)
        tup = actions["Minimal"]["tuples"][0]
        # tuple shape: topic, message, message_type, scale_x, scale_y, offset_x, offset_y, publish_type, service_name
        assert tup[2] == "std_msgs/msg/String"  # message_type default
        assert tup[3] == 1.0   # scale_x
        assert tup[4] == 1.0   # scale_y
        assert tup[5] == 0.0   # offset_x
        assert tup[6] == 0.0   # offset_y
        assert tup[7] == "topic"  # publish_type
        assert tup[8] == ""       # service_name

    def test_tuple_full_fields(self, db, uid):
        tup = ("/t", "{}", "geometry_msgs/msg/Twist", 2.0, 3.0, 0.1, 0.2, "service", "my_srv")
        db.save_action(uid, "Full", [tup])
        actions = db.get_all_actions(uid)
        stored = actions["Full"]["tuples"][0]
        assert stored[2] == "geometry_msgs/msg/Twist"
        assert stored[3] == 2.0
        assert stored[4] == 3.0
        assert stored[5] == 0.1
        assert stored[6] == 0.2
        assert stored[7] == "service"
        assert stored[8] == "my_srv"

    def test_action_id_present_in_result(self, db, uid):
        db.save_action(uid, "WithId", [("/t", "{}")])
        actions = db.get_all_actions(uid)
        assert "id" in actions["WithId"]
        assert isinstance(actions["WithId"]["id"], int)

    def test_empty_actions_returns_empty_dict(self, db, uid):
        assert db.get_all_actions(uid) == {}

    def test_actions_isolated_between_users(self, db):
        uid1 = db.get_or_create_user("u1")
        uid2 = db.get_or_create_user("u2")
        db.save_action(uid1, "Shared Name", [("/t", "{}")])
        assert "Shared Name" not in db.get_all_actions(uid2)


# ─────────────────────────────────────────────────────────────────────────────
# Mappings
# ─────────────────────────────────────────────────────────────────────────────

class TestMappings:
    def test_save_and_retrieve_mapping(self, db, uid, action_name):
        btn_entries = [("A", action_name, "once")]
        ok = db.save_mapping(uid, "Config1", btn_entries, [])
        assert ok is True
        mappings = db.get_all_mappings(uid)
        names = [m["name"] for m in mappings]
        assert "Config1" in names

    def test_mapping_contains_correct_button_entry(self, db, uid, action_name):
        db.save_mapping(uid, "M1", [("A", action_name, "hold")], [])
        m = db.get_mapping(uid, "M1")
        assert m is not None
        assert len(m["buttons"]) == 1
        assert m["buttons"][0]["button"] == "A"
        assert m["buttons"][0]["action"] == action_name
        assert m["buttons"][0]["trigger_mode"] == "hold"

    def test_mapping_contains_correct_joystick_entry(self, db, uid):
        db.save_action(uid, "Drive", [("/cmd_vel", "{}")], ACTION_JOYSTICK)
        db.save_mapping(uid, "M2", [], [("Left Stick", "Drive")])
        m = db.get_mapping(uid, "M2")
        assert len(m["joysticks"]) == 1
        assert m["joysticks"][0]["joystick"] == "Left Stick"
        assert m["joysticks"][0]["action"] == "Drive"

    def test_save_mapping_overwrites_entries(self, db, uid, action_name):
        db.save_mapping(uid, "M3", [("A", action_name, "once")], [])
        db.save_mapping(uid, "M3", [("B", action_name, "once")], [])
        m = db.get_mapping(uid, "M3")
        buttons = [b["button"] for b in m["buttons"]]
        assert "B" in buttons
        assert "A" not in buttons

    def test_get_mapping_returns_none_when_missing(self, db, uid):
        assert db.get_mapping(uid, "does_not_exist") is None

    def test_delete_mapping(self, db, uid, action_name):
        db.save_mapping(uid, "Del", [("A", action_name, "once")], [])
        ok = db.delete_mapping(uid, "Del")
        assert ok is True
        assert db.get_mapping(uid, "Del") is None

    def test_delete_nonexistent_mapping_is_ok(self, db, uid):
        assert db.delete_mapping(uid, "ghost") is True

    def test_get_all_mappings_empty(self, db, uid):
        assert db.get_all_mappings(uid) == []

    def test_save_mapping_silently_skips_unknown_action(self, db, uid):
        """An action name that doesn't exist should not crash save_mapping."""
        ok = db.save_mapping(uid, "Bad", [("X", "nonexistent_action", "once")], [])
        assert ok is True
        m = db.get_mapping(uid, "Bad")
        # The button entry was silently skipped
        assert m["buttons"] == []

    def test_mappings_isolated_between_users(self, db, action_name):
        uid1 = db.get_or_create_user("mu1")
        uid2 = db.get_or_create_user("mu2")
        db.save_action(uid1, action_name, [("/t", "{}")])
        db.save_mapping(uid1, "Shared", [("A", action_name, "once")], [])
        assert db.get_mapping(uid2, "Shared") is None


# ─────────────────────────────────────────────────────────────────────────────
# get_mappings_using_action
# ─────────────────────────────────────────────────────────────────────────────

class TestMappingsUsingAction:
    def test_returns_mapping_that_uses_action_via_button(self, db, uid, action_name):
        db.save_mapping(uid, "Config", [("A", action_name, "once")], [])
        result = db.get_mappings_using_action(uid, action_name)
        assert "Config" in result

    def test_returns_mapping_that_uses_action_via_joystick(self, db, uid):
        db.save_action(uid, "Joy Action", [("/t", "{}")], ACTION_JOYSTICK)
        db.save_mapping(uid, "JoyConfig", [], [("Left Stick", "Joy Action")])
        result = db.get_mappings_using_action(uid, "Joy Action")
        assert "JoyConfig" in result

    def test_does_not_return_unrelated_mappings(self, db, uid, action_name):
        db.save_action(uid, "Other", [("/t", "{}")])
        db.save_mapping(uid, "Other Config", [("B", "Other", "once")], [])
        result = db.get_mappings_using_action(uid, action_name)
        assert "Other Config" not in result

    def test_returns_empty_list_when_no_references(self, db, uid):
        result = db.get_mappings_using_action(uid, "nonexistent")
        assert result == []

    def test_delete_action_cascades_out_of_mapping(self, db, uid, action_name):
        db.save_mapping(uid, "Cfg", [("A", action_name, "once")], [])
        db.delete_action(uid, action_name)
        m = db.get_mapping(uid, "Cfg")
        assert m["buttons"] == []


# ─────────────────────────────────────────────────────────────────────────────
# Recording schemes
# ─────────────────────────────────────────────────────────────────────────────

class TestRecordingSchemes:
    def test_save_and_retrieve_scheme(self, db, uid):
        ok = db.save_recording_scheme(uid, "Full", ["/cmd_vel", "/joy"])
        assert ok is True
        schemes = db.get_all_recording_schemes(uid)
        assert "Full" in schemes
        assert "/cmd_vel" in schemes["Full"]
        assert "/joy" in schemes["Full"]

    def test_save_overwrites_scheme(self, db, uid):
        db.save_recording_scheme(uid, "S", ["/old"])
        db.save_recording_scheme(uid, "S", ["/new1", "/new2"])
        schemes = db.get_all_recording_schemes(uid)
        assert "/old" not in schemes["S"]
        assert "/new1" in schemes["S"]

    def test_delete_scheme(self, db, uid):
        db.save_recording_scheme(uid, "Del", ["/t"])
        ok = db.delete_recording_scheme(uid, "Del")
        assert ok is True
        assert "Del" not in db.get_all_recording_schemes(uid)

    def test_delete_nonexistent_scheme_is_ok(self, db, uid):
        assert db.delete_recording_scheme(uid, "ghost") is True

    def test_empty_schemes_returns_empty_dict(self, db, uid):
        assert db.get_all_recording_schemes(uid) == {}

    def test_schemes_isolated_between_users(self, db):
        uid1 = db.get_or_create_user("su1")
        uid2 = db.get_or_create_user("su2")
        db.save_recording_scheme(uid1, "Mine", ["/t"])
        assert "Mine" not in db.get_all_recording_schemes(uid2)


# ─────────────────────────────────────────────────────────────────────────────
# Schema validation
# ─────────────────────────────────────────────────────────────────────────────

class TestSchemaValidation:
    def test_raises_on_corrupt_schema(self, tmp_path):
        db_path = str(tmp_path / "corrupt.db")
        # Create a DB with the 'users' table missing the 'created_utc' column
        conn = sqlite3.connect(db_path)
        conn.execute("CREATE TABLE users (id INTEGER PRIMARY KEY, name TEXT UNIQUE NOT NULL)")
        conn.commit()
        conn.close()
        with pytest.raises(DatabaseSchemaError):
            Database(db_path)

    def test_fresh_db_initialises_successfully(self, tmp_path):
        db = Database(str(tmp_path / "fresh.db"))
        uid = db.get_or_create_user("test")
        assert isinstance(uid, int)

    def test_database_directory_error(self, tmp_path):
        """Path whose parent directory is a file (can't create sub-dir)."""
        blocker = tmp_path / "blocker"
        blocker.write_text("I am a file")
        with pytest.raises((DatabaseDirectoryError, DatabaseConnectionError, Exception)):
            Database(str(blocker / "sub" / "test.db"))
