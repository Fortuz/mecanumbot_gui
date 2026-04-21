"""
test_flask_routes.py — Integration tests for the FlaskApp HTTP routes.

Uses Flask's built-in test client.  The ROS2 background thread is patched
out so these tests run entirely in-process with no robot / ROS2 context.
"""

import sys
import types
import json
import pytest
from unittest.mock import MagicMock, patch

# ── Stub heavy dependencies before importing app ──────────────────────────────

# Reuse any stubs already registered by test_docker_node (if run in same session)
# otherwise register fresh ones so the import doesn't explode.
_STUB_MODS = [
    'rclpy', 'rclpy.node', 'rclpy.executors',
    'std_msgs', 'std_msgs.msg',
    'sensor_msgs', 'sensor_msgs.msg',
    'geometry_msgs', 'geometry_msgs.msg',
    'nav_msgs', 'nav_msgs.msg',
    'rosidl_runtime_py',
    'mecanumbot_msgs', 'mecanumbot_msgs.msg', 'mecanumbot_msgs.srv',
]
for _m in _STUB_MODS:
    if _m not in sys.modules:
        _stub = types.ModuleType(_m)
        sys.modules[_m] = _stub

# Give msg stubs needed class references
sys.modules['sensor_msgs.msg'].Joy     = MagicMock
sys.modules['geometry_msgs.msg'].Twist = MagicMock

# Provide all class attributes that docker_node.py imports at module level
for _cls in ('GetRobotActions', 'SetLedStatus', 'GetLedStatus',
             'GetMappings', 'SaveMapping', 'DeleteRobotMapping',
             'ApplyMapping', 'SaveRobotAction', 'DeleteRobotAction',
             'GetRecordingSchemes', 'SaveRecordingScheme', 'DeleteRecordingScheme'):
    setattr(sys.modules['mecanumbot_msgs.srv'], _cls, MagicMock)
for _cls in ('ActionTuple', 'ActionDescriptor',
             'ControllerStatus', 'ButtonEvent', 'JoystickEvent', 'OpenCRState'):
    setattr(sys.modules['mecanumbot_msgs.msg'], _cls, MagicMock)

# ── Import the real app / docker_node ────────────────────────────────────────
import docker_node as dn
from mock_database import MockDatabase


# ── Fixtures ──────────────────────────────────────────────────────────────────

@pytest.fixture
def mock_node():
    """A fully mocked DockerNode instance."""
    node = MagicMock()
    node.save_action_to_robot.return_value = (True, '')
    node.delete_robot_action.return_value  = (True, '')
    node.record_new_action.return_value    = None
    node.get_robot_actions.return_value    = ({}, None)
    return node


@pytest.fixture
def client(mock_node):
    """Flask test client with ROS2 thread and docker_node patched out."""
    db = MockDatabase()

    # Patch _start_docker_node so no thread is started
    with patch.object(dn, '_start_docker_node'):
        from app import FlaskApp
        flask_app = FlaskApp(db)
        flask_app.app.config['TESTING'] = True

    # Inject mock node as the live instance
    dn._docker_node_instance = mock_node

    with flask_app.app.test_client() as c:
        yield c, db, mock_node

    # Cleanup
    dn._docker_node_instance = None
    dn.CURRENT_USER = {"name": None, "id": None}


def _login(c, db, user_name='TestUser'):
    """Helper: POST /save to set CURRENT_USER in docker_node."""
    c.post('/save', data={'user_name': user_name})


# ── /api/controller-status ────────────────────────────────────────────────────

class TestControllerStatusApi:

    def test_returns_json(self, client):
        c, db, _ = client
        dn.LATEST_CONTROLLER_STATUS = {
            'connected': True, 'controller_type': 'Xbox 360',
            'time_since_last_input': 0.0, 'timestamp': 1, 'last_updated': 'now'
        }
        resp = c.get('/api/controller-status')
        assert resp.status_code == 200
        data = resp.get_json()
        assert data['connected'] is True
        assert data['controller_type'] == 'Xbox 360'


# ── /api/actions (GET) ───────────────────────────────────────────────────────

class TestActionsApiSource:

    def test_host_source_default(self, client):
        c, db, _ = client
        _login(c, db)
        resp = c.get('/api/actions')
        assert resp.status_code == 200
        assert isinstance(resp.get_json(), list)

    def test_host_source_explicit(self, client):
        c, db, _ = client
        _login(c, db)
        resp = c.get('/api/actions?source=host')
        assert resp.status_code == 200
        assert isinstance(resp.get_json(), list)

    def test_robot_source_offline(self, client):
        c, db, node = client
        _login(c, db)
        node.get_robot_actions.return_value = (None, 'Robot not connected')
        resp = c.get('/api/actions?source=robot')
        assert resp.status_code == 502
        assert 'error' in resp.get_json()

    def test_robot_source_online(self, client):
        c, db, node = client
        _login(c, db)
        node.get_robot_actions.return_value = (
            {'drive': {'type': 'joystick', 'tuples': []}}, None)
        resp = c.get('/api/actions?source=robot')
        assert resp.status_code == 200
        data = resp.get_json()
        assert isinstance(data, list)
        assert data[0]['name'] == 'drive'

    def test_include_tuples_returns_tuple_data(self, client):
        c, db, _ = client
        _login(c, db)
        uid = dn.CURRENT_USER['id']
        tuples = [('/cmd_vel', '{"linear":{"x":1}}', 'geometry_msgs/msg/Twist',
                   1.0, 1.0, 0.0, 0.0, 'topic', '')]
        db.save_action(uid, 'test_act', tuples, 'button_once')

        resp = c.get('/api/actions?include_tuples=1')
        assert resp.status_code == 200
        data = resp.get_json()
        assert len(data) == 1
        assert data[0]['name'] == 'test_act'
        assert 'tuples' in data[0]
        assert len(data[0]['tuples']) == 1
        assert data[0]['tuples'][0][0] == '/cmd_vel'

    def test_without_include_tuples_no_tuple_data(self, client):
        c, db, _ = client
        _login(c, db)
        uid = dn.CURRENT_USER['id']
        tuples = [('/cmd_vel', '{}', 'geometry_msgs/msg/Twist',
                   1.0, 1.0, 0.0, 0.0, 'topic', '')]
        db.save_action(uid, 'test_act2', tuples, 'button_once')

        resp = c.get('/api/actions')
        assert resp.status_code == 200
        data = resp.get_json()
        assert len(data) >= 1
        # Without include_tuples, tuples should not be present
        for item in data:
            if item['name'] == 'test_act2':
                assert 'tuples' not in item
                break


# ── /api/robot-status ────────────────────────────────────────────────────────

class TestRobotStatusApi:

    def test_robot_offline(self, client):
        c, *_ = client
        dn.ROBOT_ACTIVE = False
        resp = c.get('/api/robot-status')
        assert resp.status_code == 200
        assert resp.get_json()['available'] is False

    def test_robot_online(self, client):
        c, *_ = client
        dn.ROBOT_ACTIVE = True
        resp = c.get('/api/robot-status')
        assert resp.get_json()['available'] is True


# ── /api/actions/save (POST JSON) ────────────────────────────────────────────

class TestSaveActionApi:

    def test_valid_action_saves_to_db(self, client):
        c, db, node = client
        _login(c, db)
        uid = dn.CURRENT_USER['id']

        resp = c.post('/api/actions/save',
                      data=json.dumps({
                          'name':        'my_action',
                          'action_type': 'button_once',
                          'tuples':      [['/cmd_vel', '{}', 'geometry_msgs/msg/Twist',
                                           1.0, 1.0, 0.0, 0.0, 'topic', '']],
                          'destination': 'host',
                      }),
                      content_type='application/json')

        assert resp.status_code == 200
        data = resp.get_json()
        assert data['success'] is True
        assert data['saved_host'] is True
        assert 'my_action' in db.get_all_actions(uid)

    def test_missing_name_returns_400(self, client):
        c, db, _ = client
        _login(c, db)
        resp = c.post('/api/actions/save',
                      data=json.dumps({
                          'name':        '',
                          'action_type': 'button_once',
                          'tuples':      [['/cmd_vel', '{}', 'geometry_msgs/msg/Twist',
                                           1.0, 1.0, 0.0, 0.0, 'topic', '']],
                      }),
                      content_type='application/json')
        assert resp.status_code == 400

    def test_missing_tuples_returns_400(self, client):
        c, db, _ = client
        _login(c, db)
        resp = c.post('/api/actions/save',
                      data=json.dumps({'name': 'no_tuples', 'tuples': []}),
                      content_type='application/json')
        assert resp.status_code == 400

    def test_not_logged_in_returns_401(self, client):
        c, db, _ = client
        # Do NOT call _login — CURRENT_USER.id is None
        resp = c.post('/api/actions/save',
                      data=json.dumps({
                          'name': 'act', 'action_type': 'button_once',
                          'tuples': [['/cmd_vel', '{}', 'std_msgs/msg/String',
                                      1.0, 1.0, 0.0, 0.0, 'topic', '']],
                      }),
                      content_type='application/json')
        assert resp.status_code == 401

    def test_save_to_robot_calls_node(self, client):
        c, db, node = client
        _login(c, db)
        resp = c.post('/api/actions/save',
                      data=json.dumps({
                          'name':        'robot_act',
                          'action_type': 'button_once',
                          'tuples':      [['/cmd_vel', '{}', 'geometry_msgs/msg/Twist',
                                           1.0, 1.0, 0.0, 0.0, 'topic', '']],
                          'destination': 'robot',
                      }),
                      content_type='application/json')
        assert resp.status_code == 200
        node.save_action_to_robot.assert_called_once()

    def test_conflict_returns_409_without_overwrite(self, client):
        c, db, _ = client
        _login(c, db)
        uid = dn.CURRENT_USER['id']
        # Create an existing action
        tuples = [('/cmd_vel', '{}', 'geometry_msgs/msg/Twist',
                   1.0, 1.0, 0.0, 0.0, 'topic', '')]
        db.save_action(uid, 'existing_action', tuples, 'button_once')

        # Try to save again without overwrite
        resp = c.post('/api/actions/save',
                      data=json.dumps({
                          'name':        'existing_action',
                          'action_type': 'button_once',
                          'tuples':      [['/cmd_vel', '{}', 'geometry_msgs/msg/Twist',
                                           1.0, 1.0, 0.0, 0.0, 'topic', '']],
                          'destination': 'host',
                      }),
                      content_type='application/json')
        assert resp.status_code == 409
        assert 'already exists' in resp.get_json()['error']

    def test_overwrite_allows_replacing_existing(self, client):
        c, db, _ = client
        _login(c, db)
        uid = dn.CURRENT_USER['id']
        # Create an existing action
        tuples = [('/cmd_vel', '{}', 'geometry_msgs/msg/Twist',
                   1.0, 1.0, 0.0, 0.0, 'topic', '')]
        db.save_action(uid, 'overwrite_action', tuples, 'button_once')

        # Save again with overwrite=True
        resp = c.post('/api/actions/save',
                      data=json.dumps({
                          'name':        'overwrite_action',
                          'action_type': 'button_hold',  # Changed type
                          'tuples':      [['/cmd_vel', '{}', 'geometry_msgs/msg/Twist',
                                           1.0, 1.0, 0.0, 0.0, 'topic', '']],
                          'destination': 'host',
                          'overwrite':   True,
                      }),
                      content_type='application/json')
        assert resp.status_code == 200
        assert resp.get_json()['success'] is True
        # Verify the action was updated
        actions = db.get_all_actions(uid)
        assert actions['overwrite_action']['type'] == 'button_hold'


# ── /api/actions/delete (POST JSON) ─────────────────────────────────────────

class TestDeleteActionApi:

    def test_delete_removes_from_db(self, client):
        c, db, _ = client
        _login(c, db)
        uid = dn.CURRENT_USER['id']
        db.save_action(uid, 'to_delete', [], 'button_once')

        resp = c.post('/api/actions/delete',
                      data=json.dumps({'name': 'to_delete', 'destination': 'host'}),
                      content_type='application/json')
        assert resp.status_code == 200
        assert resp.get_json()['success'] is True
        assert 'to_delete' not in db.get_all_actions(uid)

    def test_missing_name_returns_400(self, client):
        c, db, _ = client
        _login(c, db)
        resp = c.post('/api/actions/delete',
                      data=json.dumps({'name': ''}),
                      content_type='application/json')
        assert resp.status_code == 400

    def test_delete_nonexistent_does_not_crash(self, client):
        c, db, _ = client
        _login(c, db)
        resp = c.post('/api/actions/delete',
                      data=json.dumps({'name': 'ghost', 'destination': 'host'}),
                      content_type='application/json')
        assert resp.status_code == 200


# ── /api/mappings/save conflict checks ───────────────────────────────────────

class TestSaveMappingConflict:

    def test_mapping_conflict_returns_409(self, client):
        c, db, _ = client
        _login(c, db)
        uid = dn.CURRENT_USER['id']
        # Create an existing mapping
        db.save_mapping(uid, 'existing_map', [('A', 'action1', 'once')], [])

        # Try to save again without overwrite
        resp = c.post('/api/mappings/save',
                      data=json.dumps({
                          'name':        'existing_map',
                          'destination': 'host',
                          'buttons':     [{'button': 'B', 'action': 'action2', 'trigger_mode': 'once'}],
                          'joysticks':   [],
                      }),
                      content_type='application/json')
        assert resp.status_code == 409
        assert 'already exists' in resp.get_json()['error']

    def test_mapping_overwrite_succeeds(self, client):
        c, db, _ = client
        _login(c, db)
        uid = dn.CURRENT_USER['id']
        # Create an existing mapping
        db.save_mapping(uid, 'overwrite_map', [('A', 'action1', 'once')], [])

        # Save again with overwrite=True
        resp = c.post('/api/mappings/save',
                      data=json.dumps({
                          'name':        'overwrite_map',
                          'destination': 'host',
                          'buttons':     [{'button': 'B', 'action': 'action2', 'trigger_mode': 'once'}],
                          'joysticks':   [],
                          'overwrite':   True,
                      }),
                      content_type='application/json')
        assert resp.status_code == 200
        assert resp.get_json()['success'] is True
        # Verify the mapping was updated
        mapping = db.get_mapping(uid, 'overwrite_map')
        assert mapping is not None
        # Check that button B is now assigned
        assert any(b['button'] == 'B' for b in mapping['buttons'])


# ── /api/recording-schemes/save conflict checks ──────────────────────────────

class TestSaveRecordingSchemeConflict:

    def test_conflict_returns_409_if_exists_without_overwrite(self, client):
        c, db, _ = client
        _login(c, db)
        uid = dn.CURRENT_USER['id']
        # Pre-create scheme
        db.save_recording_scheme(uid, 'existing_scheme', ['/cmd_vel'])

        # Try to save again without overwrite
        resp = c.post('/api/recording-schemes/save',
                      data=json.dumps({
                          'name': 'existing_scheme',
                          'topics': ['/joy', '/chatter'],
                      }),
                      content_type='application/json')
        assert resp.status_code == 409
        assert 'already exists' in resp.get_json()['error']

    def test_overwrite_allows_replacing_existing(self, client):
        c, db, _ = client
        _login(c, db)
        uid = dn.CURRENT_USER['id']
        # Pre-create scheme
        db.save_recording_scheme(uid, 'overwrite_scheme', ['/cmd_vel'])

        # Save again with overwrite=True
        resp = c.post('/api/recording-schemes/save',
                      data=json.dumps({
                          'name': 'overwrite_scheme',
                          'topics': ['/joy', '/chatter'],
                          'overwrite': True,
                      }),
                      content_type='application/json')
        assert resp.status_code == 200
        assert resp.get_json()['success'] is True
        # Verify the scheme was updated
        schemes = db.get_all_recording_schemes(uid)
        assert 'overwrite_scheme' in schemes
        assert '/joy' in schemes['overwrite_scheme']


# ── /api/recording/status ────────────────────────────────────────────────────

class TestRecordingStatusApi:

    def test_returns_inactive_by_default(self, client):
        c, *_ = client
        dn.RECORDING_STATE = {
            'active': False, 'user_name': None,
            'topics': [], 'dir': None, 'started_at': None
        }
        resp = c.get('/api/recording/status')
        assert resp.status_code == 200
        assert resp.get_json()['active'] is False

    def test_returns_active_state(self, client):
        c, *_ = client
        dn.RECORDING_STATE = {
            'active': True, 'user_name': 'alice',
            'topics': ['/cmd_vel'], 'dir': '/tmp/rec', 'started_at': 'now'
        }
        resp = c.get('/api/recording/status')
        data = resp.get_json()
        assert data['active'] is True
        assert data['user_name'] == 'alice'

