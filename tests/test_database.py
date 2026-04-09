"""
test_database.py — Unit tests for the concrete Database(IDatabase) class.

These tests require only sqlite3 (stdlib) and pytest.
No ROS2, no Flask, no robot hardware needed.
"""

import os
import sys
import pytest

# conftest.py adds shared/ to sys.path
from database import Database


# ── Fixture ──────────────────────────────────────────────────────────────────

@pytest.fixture
def db(tmp_db_path):
    return Database(tmp_db_path)


# ── Actions ───────────────────────────────────────────────────────────────────

class TestSaveAndGetActions:

    def test_save_and_retrieve(self, db):
        tuples = [('/cmd_vel', '{"linear":{"x":0.5}}', 'geometry_msgs/msg/Twist',
                   1.0, 1.0, 0.0, 0.0, 'topic', '')]
        assert db.save_action('move_forward', tuples, 'button_once') is True

        actions = db.get_all_actions()
        assert 'move_forward' in actions
        assert actions['move_forward']['type'] == 'button_once'
        assert len(actions['move_forward']['tuples']) == 1

    def test_tuple_fields_preserved(self, db):
        tuples = [('/cmd_vel', '{"linear":{"x":0.5}}', 'geometry_msgs/msg/Twist',
                   2.0, 3.0, 0.1, 0.2, 'topic', '')]
        db.save_action('test_action', tuples, 'joystick')

        t = db.get_all_actions()['test_action']['tuples'][0]
        assert t[0] == '/cmd_vel'
        assert t[2] == 'geometry_msgs/msg/Twist'
        assert t[3] == 2.0    # scale_x
        assert t[4] == 3.0    # scale_y
        assert t[5] == pytest.approx(0.1)   # offset_x
        assert t[6] == pytest.approx(0.2)   # offset_y
        assert t[7] == 'topic'

    def test_multiple_tuples_in_order(self, db):
        tuples = [
            ('/topic_a', 'msg_a', 'std_msgs/msg/String', 1.0, 1.0, 0.0, 0.0, 'topic', ''),
            ('/topic_b', 'msg_b', 'std_msgs/msg/String', 1.0, 1.0, 0.0, 0.0, 'topic', ''),
        ]
        db.save_action('multi', tuples, 'button_once')
        result = db.get_all_actions()['multi']['tuples']
        assert len(result) == 2
        assert result[0][0] == '/topic_a'
        assert result[1][0] == '/topic_b'

    def test_overwrite_replaces_tuples_and_type(self, db):
        db.save_action('move', [('/cmd_vel', 'old', 'std_msgs/msg/String',
                                  1.0, 1.0, 0.0, 0.0, 'topic', '')], 'button_once')
        db.save_action('move', [('/cmd_vel', 'new', 'geometry_msgs/msg/Twist',
                                  2.0, 2.0, 0.0, 0.0, 'topic', '')], 'button_hold')

        actions = db.get_all_actions()
        assert len(actions) == 1                            # not duplicated
        assert actions['move']['type'] == 'button_hold'
        assert actions['move']['tuples'][0][1] == 'new'     # message replaced
        assert actions['move']['tuples'][0][3] == 2.0       # scale_x replaced

    def test_delete_action(self, db):
        db.save_action('to_delete', [], 'button_once')
        assert 'to_delete' in db.get_all_actions()
        db.delete_action('to_delete')
        assert 'to_delete' not in db.get_all_actions()

    def test_delete_nonexistent_returns_true(self, db):
        """Deleting something that does not exist should not raise."""
        assert db.delete_action('ghost') is True

    def test_filter_by_type(self, db):
        db.save_action('btn_action',  [], 'button_once')
        db.save_action('hold_action', [], 'button_hold')
        db.save_action('joy_action',  [], 'joystick')

        assert len(db.get_all_actions('button_once')) == 1
        assert len(db.get_all_actions('button_hold')) == 1
        assert len(db.get_all_actions('joystick'))    == 1
        assert len(db.get_all_actions())              == 3

    def test_service_tuple(self, db):
        tuples = [('', '{"fl_mode":3}', 'mecanumbot_msgs/srv/SetLedStatus',
                   1.0, 1.0, 0.0, 0.0, 'service', 'set_led_status')]
        db.save_action('set_led', tuples, 'button_once')
        t = db.get_all_actions()['set_led']['tuples'][0]
        assert t[7] == 'service'
        assert t[8] == 'set_led_status'


# ── Button Mappings ───────────────────────────────────────────────────────────

class TestButtonMappings:

    def test_save_and_get(self, db):
        db.save_action('grab', [], 'button_once')
        assert db.save_button_mapping('A', 'grab', 'once') is True

        mappings = db.get_all_button_mappings()
        assert 'A' in mappings
        assert mappings['A']['action'] == 'grab'
        assert mappings['A']['trigger_mode'] == 'once'

    def test_hold_trigger_mode(self, db):
        db.save_action('spin', [], 'button_hold')
        db.save_button_mapping('LB', 'spin', 'hold')
        assert db.get_all_button_mappings()['LB']['trigger_mode'] == 'hold'

    def test_upsert_replaces_existing(self, db):
        db.save_action('action_a', [], 'button_once')
        db.save_action('action_b', [], 'button_once')
        db.save_button_mapping('X', 'action_a', 'once')
        db.save_button_mapping('X', 'action_b', 'once')   # overwrite same button

        mappings = db.get_all_button_mappings()
        assert mappings['X']['action'] == 'action_b'
        assert len(mappings) == 1

    def test_save_fails_for_unknown_action(self, db):
        result = db.save_button_mapping('Y', 'nonexistent_action', 'once')
        assert result is False

    def test_delete_mapping(self, db):
        db.save_action('grab', [], 'button_once')
        db.save_button_mapping('A', 'grab', 'once')
        db.delete_button_mapping('A')
        assert 'A' not in db.get_all_button_mappings()

    def test_delete_action_cascades_to_button_mapping(self, db):
        db.save_action('grab', [], 'button_once')
        db.save_button_mapping('A', 'grab', 'once')
        db.delete_action('grab')
        assert 'A' not in db.get_all_button_mappings()


# ── Joystick Mappings ─────────────────────────────────────────────────────────

class TestJoystickMappings:

    def test_save_and_get(self, db):
        db.save_action('drive', [], 'joystick')
        assert db.save_joystick_mapping('Left Stick', 'drive') is True

        mappings = db.get_all_joystick_mappings()
        assert 'Left Stick' in mappings
        assert mappings['Left Stick'] == 'drive'

    def test_upsert_replaces_existing(self, db):
        db.save_action('drive', [], 'joystick')
        db.save_action('strafe', [], 'joystick')
        db.save_joystick_mapping('Left Stick', 'drive')
        db.save_joystick_mapping('Left Stick', 'strafe')   # overwrite

        mappings = db.get_all_joystick_mappings()
        assert mappings['Left Stick'] == 'strafe'
        assert len(mappings) == 1

    def test_save_fails_for_unknown_action(self, db):
        result = db.save_joystick_mapping('Right Stick', 'ghost_action')
        assert result is False

    def test_delete_mapping(self, db):
        db.save_action('drive', [], 'joystick')
        db.save_joystick_mapping('Left Stick', 'drive')
        db.delete_joystick_mapping('Left Stick')
        assert 'Left Stick' not in db.get_all_joystick_mappings()

    def test_delete_action_cascades_to_joystick_mapping(self, db):
        db.save_action('drive', [], 'joystick')
        db.save_joystick_mapping('Left Stick', 'drive')
        db.delete_action('drive')
        assert 'Left Stick' not in db.get_all_joystick_mappings()
