"""
test_database.py — Unit tests for the concrete Database(IDatabase) class.

These tests require only sqlite3 (stdlib) and pytest.
No ROS2, no Flask, no robot hardware needed.

All data is scoped per-user: each test obtains a user_id via
get_or_create_user() and passes it to every subsequent call.
"""

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
        uid = db.get_or_create_user('alice')
        tuples = [('/cmd_vel', '{"linear":{"x":0.5}}', 'geometry_msgs/msg/Twist',
                   1.0, 1.0, 0.0, 0.0, 'topic', '')]
        assert db.save_action(uid, 'move_forward', tuples, 'button_once') is True

        actions = db.get_all_actions(uid)
        assert 'move_forward' in actions
        assert actions['move_forward']['type'] == 'button_once'
        assert len(actions['move_forward']['tuples']) == 1

    def test_tuple_fields_preserved(self, db):
        uid = db.get_or_create_user('alice')
        tuples = [('/cmd_vel', '{"linear":{"x":0.5}}', 'geometry_msgs/msg/Twist',
                   2.0, 3.0, 0.1, 0.2, 'topic', '')]
        db.save_action(uid, 'test_action', tuples, 'joystick')

        t = db.get_all_actions(uid)['test_action']['tuples'][0]
        assert t[0] == '/cmd_vel'
        assert t[2] == 'geometry_msgs/msg/Twist'
        assert t[3] == 2.0    # scale_x
        assert t[4] == 3.0    # scale_y
        assert t[5] == pytest.approx(0.1)   # offset_x
        assert t[6] == pytest.approx(0.2)   # offset_y
        assert t[7] == 'topic'

    def test_multiple_tuples_in_order(self, db):
        uid = db.get_or_create_user('alice')
        tuples = [
            ('/topic_a', 'msg_a', 'std_msgs/msg/String', 1.0, 1.0, 0.0, 0.0, 'topic', ''),
            ('/topic_b', 'msg_b', 'std_msgs/msg/String', 1.0, 1.0, 0.0, 0.0, 'topic', ''),
        ]
        db.save_action(uid, 'multi', tuples, 'button_once')
        result = db.get_all_actions(uid)['multi']['tuples']
        assert len(result) == 2
        assert result[0][0] == '/topic_a'
        assert result[1][0] == '/topic_b'

    def test_overwrite_replaces_tuples_and_type(self, db):
        uid = db.get_or_create_user('alice')
        db.save_action(uid, 'move', [('/cmd_vel', 'old', 'std_msgs/msg/String',
                                      1.0, 1.0, 0.0, 0.0, 'topic', '')], 'button_once')
        db.save_action(uid, 'move', [('/cmd_vel', 'new', 'geometry_msgs/msg/Twist',
                                      2.0, 2.0, 0.0, 0.0, 'topic', '')], 'button_hold')

        actions = db.get_all_actions(uid)
        assert len(actions) == 1                            # not duplicated
        assert actions['move']['type'] == 'button_hold'
        assert actions['move']['tuples'][0][1] == 'new'     # message replaced
        assert actions['move']['tuples'][0][3] == 2.0       # scale_x replaced

    def test_delete_action(self, db):
        uid = db.get_or_create_user('alice')
        db.save_action(uid, 'to_delete', [], 'button_once')
        assert 'to_delete' in db.get_all_actions(uid)
        db.delete_action(uid, 'to_delete')
        assert 'to_delete' not in db.get_all_actions(uid)

    def test_delete_nonexistent_returns_true(self, db):
        """Deleting something that does not exist should not raise."""
        uid = db.get_or_create_user('alice')
        assert db.delete_action(uid, 'ghost') is True

    def test_filter_by_type(self, db):
        uid = db.get_or_create_user('alice')
        db.save_action(uid, 'btn_action',  [], 'button_once')
        db.save_action(uid, 'hold_action', [], 'button_hold')
        db.save_action(uid, 'joy_action',  [], 'joystick')

        assert len(db.get_all_actions(uid, 'button_once')) == 1
        assert len(db.get_all_actions(uid, 'button_hold')) == 1
        assert len(db.get_all_actions(uid, 'joystick'))    == 1
        assert len(db.get_all_actions(uid))                == 3

    def test_service_tuple(self, db):
        uid = db.get_or_create_user('alice')
        tuples = [('', '{"fl_mode":3}', 'mecanumbot_msgs/srv/SetLedStatus',
                   1.0, 1.0, 0.0, 0.0, 'service', 'set_led_status')]
        db.save_action(uid, 'set_led', tuples, 'button_once')
        t = db.get_all_actions(uid)['set_led']['tuples'][0]
        assert t[7] == 'service'
        assert t[8] == 'set_led_status'

    def test_actions_are_isolated_between_users(self, db):
        uid_a = db.get_or_create_user('alice')
        uid_b = db.get_or_create_user('bob')
        db.save_action(uid_a, 'alice_move', [], 'button_once')
        db.save_action(uid_b, 'bob_drive',  [], 'joystick')

        assert 'alice_move' in db.get_all_actions(uid_a)
        assert 'bob_drive'  not in db.get_all_actions(uid_a)
        assert 'bob_drive'  in db.get_all_actions(uid_b)
        assert 'alice_move' not in db.get_all_actions(uid_b)


# ── Mappings ──────────────────────────────────────────────────────────────────

class TestMappings:

    def test_save_and_get_all(self, db):
        uid = db.get_or_create_user('alice')
        # Actions must exist before they can be referenced in a mapping
        db.save_action(uid, 'grab',  [], 'button_once')
        db.save_action(uid, 'stop',  [], 'button_once')
        db.save_action(uid, 'drive', [], 'joystick')
        btn_entries = [('A', 'grab', 'once'), ('B', 'stop', 'once')]
        joy_entries = [('Left Stick', 'drive')]
        assert db.save_mapping(uid, 'Drive Config', btn_entries, joy_entries) is True

        mappings = db.get_all_mappings(uid)
        assert len(mappings) == 1
        m = mappings[0]
        assert m['name'] == 'Drive Config'
        assert len(m['buttons'])   == 2
        assert len(m['joysticks']) == 1

    def test_get_mapping_by_name(self, db):
        uid = db.get_or_create_user('alice')
        db.save_action(uid, 'grab', [], 'button_once')
        db.save_mapping(uid, 'Config A', [('A', 'grab', 'once')], [])
        m = db.get_mapping(uid, 'Config A')
        assert m is not None
        assert m['name'] == 'Config A'
        assert m['buttons'][0]['button'] == 'A'

    def test_get_mapping_not_found_returns_none(self, db):
        uid = db.get_or_create_user('alice')
        assert db.get_mapping(uid, 'nonexistent') is None

    def test_upsert_replaces_entries(self, db):
        uid = db.get_or_create_user('alice')
        db.save_action(uid, 'old_action', [], 'button_once')
        db.save_action(uid, 'new_action', [], 'button_hold')
        db.save_action(uid, 'drive',      [], 'joystick')
        db.save_mapping(uid, 'M', [('A', 'old_action', 'once')], [])
        db.save_mapping(uid, 'M', [('B', 'new_action', 'hold')], [('Left Stick', 'drive')])

        m = db.get_mapping(uid, 'M')
        assert len(m['buttons'])   == 1
        assert m['buttons'][0]['button'] == 'B'
        assert len(m['joysticks']) == 1

    def test_delete_mapping(self, db):
        uid = db.get_or_create_user('alice')
        db.save_mapping(uid, 'ToDelete', [], [])
        db.delete_mapping(uid, 'ToDelete')
        assert db.get_mapping(uid, 'ToDelete') is None

    def test_mappings_isolated_between_users(self, db):
        uid_a = db.get_or_create_user('alice')
        uid_b = db.get_or_create_user('bob')
        db.save_mapping(uid_a, 'Alice Config', [], [])
        assert len(db.get_all_mappings(uid_b)) == 0


# ── Recording schemes ─────────────────────────────────────────────────────────

class TestRecordingSchemes:

    def test_save_and_get(self, db):
        uid = db.get_or_create_user('alice')
        assert db.save_recording_scheme(uid, 'Full Suite',
                                        ['/cmd_vel', '/joy']) is True
        schemes = db.get_all_recording_schemes(uid)
        assert 'Full Suite' in schemes
        assert '/cmd_vel' in schemes['Full Suite']
        assert '/joy'     in schemes['Full Suite']

    def test_upsert_replaces_topics(self, db):
        uid = db.get_or_create_user('alice')
        db.save_recording_scheme(uid, 'Scheme', ['/cmd_vel'])
        db.save_recording_scheme(uid, 'Scheme', ['/joy', '/chatter'])
        topics = db.get_all_recording_schemes(uid)['Scheme']
        assert len(topics) == 2
        assert '/joy' in topics

    def test_delete_scheme(self, db):
        uid = db.get_or_create_user('alice')
        db.save_recording_scheme(uid, 'ToDelete', ['/cmd_vel'])
        db.delete_recording_scheme(uid, 'ToDelete')
        assert 'ToDelete' not in db.get_all_recording_schemes(uid)

    def test_schemes_isolated_between_users(self, db):
        uid_a = db.get_or_create_user('alice')
        uid_b = db.get_or_create_user('bob')
        db.save_recording_scheme(uid_a, 'Alice Scheme', ['/cmd_vel'])
        assert len(db.get_all_recording_schemes(uid_b)) == 0

