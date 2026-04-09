"""
test_flask_routes.py — Integration tests for the FlaskApp HTTP routes.

Uses Flask's built-in test client.  The ROS2 background thread is patched
out so these tests run entirely in-process with no robot / ROS2 context.
"""

import sys
import os
import types
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
sys.modules['std_msgs.msg'].String   = MagicMock
sys.modules['sensor_msgs.msg'].Joy   = MagicMock
sys.modules['geometry_msgs.msg'].Twist = MagicMock

# ── Import the real app / docker_node ────────────────────────────────────────
import docker_node as dn
from mock_database import MockDatabase


# ── Fixtures ──────────────────────────────────────────────────────────────────

@pytest.fixture
def mock_node():
    """A fully mocked DockerNode instance."""
    node = MagicMock()
    node.register_action_with_robot.return_value = (True, '')
    node.record_new_action.return_value = None
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


# ── /api/sync-status ─────────────────────────────────────────────────────────

class TestSyncStatusApi:

    def test_not_attempted(self, client):
        c, *_ = client
        dn.LATEST_SYNC_RESULT = {
            'attempted': False, 'success': None,
            'accepted_count': 0, 'conflicts': [], 'timestamp': None
        }
        resp = c.get('/api/sync-status')
        assert resp.status_code == 200
        assert resp.get_json()['attempted'] is False

    def test_success_with_accepted_count(self, client):
        c, *_ = client
        dn.LATEST_SYNC_RESULT = {
            'attempted': True, 'success': True,
            'accepted_count': 3, 'conflicts': [], 'timestamp': '2026-01-01'
        }
        resp = c.get('/api/sync-status')
        assert resp.get_json()['accepted_count'] == 3


# ── /button-mapping/create (POST) ────────────────────────────────────────────

class TestCreateActionRoute:

    def test_valid_action_saves_to_db(self, client):
        c, db, node = client
        resp = c.post('/button-mapping/create', data={
            'user_name':    'TestUser',
            'mapping_name': 'my_action',
            'action_type':  'button_once',
            'topic_0':      '/cmd_vel',
            'message_0':    '{"linear":{"x":0.5}}',
            'message_type_0': 'geometry_msgs/msg/Twist',
            'publish_type_0': 'topic',
            'service_name_0': '',
            'offset_x_0':   '0.0',
            'offset_y_0':   '0.0',
        }, follow_redirects=False)

        # Should redirect on success
        assert resp.status_code in (302, 200)
        assert 'my_action' in db._actions

    def test_robot_rejection_shows_warning(self, client):
        c, db, node = client
        # Robot rejects the action
        node.register_action_with_robot.return_value = (False, 'Robot did not respond.')

        resp = c.post('/button-mapping/create', data={
            'user_name':    'TestUser',
            'mapping_name': 'offline_action',
            'action_type':  'button_once',
            'topic_0':      '/cmd_vel',
            'message_0':    '{"linear":{"x":1.0}}',
            'message_type_0': 'geometry_msgs/msg/Twist',
            'publish_type_0': 'topic',
            'service_name_0': '',
            'offset_x_0':   '0.0',
            'offset_y_0':   '0.0',
        }, follow_redirects=False)

        # Action is still saved locally
        assert 'offline_action' in db._actions
        # Page re-renders with a warning (200, not redirect)
        assert resp.status_code == 200
        assert b'robot_warning' in resp.data or b'Saved locally' in resp.data or \
               b'Robot did not respond' in resp.data

    def test_missing_name_does_not_save(self, client):
        c, db, _ = client
        c.post('/button-mapping/create', data={
            'user_name':    'TestUser',
            'mapping_name': '',
            'action_type':  'button_once',
        })
        assert len(db._actions) == 0

    def test_missing_tuples_redirects_without_save(self, client):
        c, db, _ = client
        resp = c.post('/button-mapping/create', data={
            'user_name':    'TestUser',
            'mapping_name': 'no_tuples',
            'action_type':  'button_once',
            # No topic/message rows
        }, follow_redirects=False)
        assert 'no_tuples' not in db._actions


# ── /button-mapping/delete (POST) ────────────────────────────────────────────

class TestDeleteActionRoute:

    def test_delete_removes_from_db(self, client):
        c, db, _ = client
        db.save_action('to_delete', [], 'button_once')
        c.post('/button-mapping/delete', data={
            'user_name':   'TestUser',
            'action_name': 'to_delete',
        })
        assert 'to_delete' not in db._actions

    def test_delete_nonexistent_does_not_crash(self, client):
        c, *_ = client
        resp = c.post('/button-mapping/delete', data={
            'user_name':   'TestUser',
            'action_name': 'ghost',
        })
        assert resp.status_code in (302, 200)


# ── /api/recording-state ─────────────────────────────────────────────────────

class TestRecordingStateApi:

    def test_returns_inactive_by_default(self, client):
        c, *_ = client
        dn.RECORDING_STATE = {
            'active': False, 'user_name': None,
            'topics': [], 'dir': None, 'started_at': None
        }
        resp = c.get('/api/recording-state')
        assert resp.status_code == 200
        assert resp.get_json()['active'] is False
