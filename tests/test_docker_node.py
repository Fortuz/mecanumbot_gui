"""
test_docker_node.py — Unit tests for DockerNode methods that contain
pure business logic (status callbacks, recording helpers, sync result
building, robot-watchdog).

Strategy: DockerNode inherits from rclpy.Node.  Rather than spinning a
real ROS2 context we build a *minimal stub* of the class using
__new__ + manual attribute injection, bypassing __init__ entirely.
All ROS2 calls (publish, create_subscription, get_logger …) are replaced
with MagicMock objects so the test never touches a real ROS2 graph.
"""

import sys
import time
from unittest.mock import MagicMock, patch

# conftest.py already added host/ and shared/ to sys.path.
# We still need to stub heavy ROS2 / mecanumbot_msgs imports before
# docker_node is loaded so that the module-level try/except blocks don't fail.

# ── Stub rclpy and rclpy.node before importing docker_node ───────────────────
import types

_rclpy_stub = types.ModuleType('rclpy')
_rclpy_stub.init = MagicMock()
_rclpy_stub.shutdown = MagicMock()
_node_stub = types.ModuleType('rclpy.node')

class _FakeNode:
    def __init__(self, name):  pass
    def create_subscription(self, *a, **kw): return MagicMock()
    def create_timer(self, *a, **kw):        return MagicMock()
    def create_client(self, *a, **kw):       return MagicMock()
    def get_logger(self):                    return MagicMock()
    def destroy_subscription(self, sub):     pass

_node_stub.Node = _FakeNode
_rclpy_stub.node = _node_stub

_exec_stub = types.ModuleType('rclpy.executors')
class _FakeExecutor:
    def add_node(self, n): pass
    def spin(self): pass
_exec_stub.MultiThreadedExecutor = _FakeExecutor
_rclpy_stub.executors = _exec_stub

sys.modules.setdefault('rclpy',           _rclpy_stub)
sys.modules.setdefault('rclpy.node',      _node_stub)
sys.modules.setdefault('rclpy.executors', _exec_stub)

# Stub sensor / geometry messages
for _mod in ('std_msgs', 'std_msgs.msg',
             'sensor_msgs', 'sensor_msgs.msg',
             'geometry_msgs', 'geometry_msgs.msg',
             'nav_msgs', 'nav_msgs.msg',
             'rosidl_runtime_py'):
    sys.modules.setdefault(_mod, types.ModuleType(_mod))

# Give the msg stubs the class attributes docker_node.py will reference
sys.modules['sensor_msgs.msg'].Joy = MagicMock
sys.modules['geometry_msgs.msg'].Twist = MagicMock

# Stub mecanumbot_msgs so SYNC_SRV_AVAILABLE stays False (simpler path)
for _mod in ('mecanumbot_msgs', 'mecanumbot_msgs.msg', 'mecanumbot_msgs.srv'):
    sys.modules.setdefault(_mod, types.ModuleType(_mod))

# Provide all class attributes that docker_node.py imports at module level
# Each service class needs a Request inner class
for _cls in ('GetRobotActions', 'SetLedStatus', 'GetLedStatus',
             'GetMappings', 'SaveMapping', 'DeleteRobotMapping',
             'ApplyMapping', 'SaveRobotAction', 'DeleteRobotAction',
             'GetRecordingSchemes', 'SaveRecordingScheme', 'DeleteRecordingScheme'):
    mock_srv = MagicMock()
    mock_srv.Request = MagicMock
    setattr(sys.modules['mecanumbot_msgs.srv'], _cls, mock_srv)
for _cls in ('ActionTuple', 'ActionDescriptor',
             'ControllerStatus', 'ButtonEvent', 'JoystickEvent', 'OpenCRState'):
    setattr(sys.modules['mecanumbot_msgs.msg'], _cls, MagicMock)

# ── Now import the real module ────────────────────────────────────────────────
import docker_node as dn
from mock_database import MockDatabase


# ── Helper: build a DockerNode stub without __init__ ─────────────────────────

def _make_node(db=None):
    """
    Create a DockerNode instance without touching ROS2 or the filesystem.
    Only the attributes needed by the tested methods are set.
    """
    node = dn.DockerNode.__new__(dn.DockerNode)
    node._db               = db or MockDatabase()
    node._rec_file         = None
    node._rec_led_file     = None
    node._rec_files        = {}
    node._rec_subs         = {}
    node._last_opencr_time = None
    node._robot_gone_timeout = 8.0
    node._get_robot_actions_client = MagicMock()
    node._logger           = MagicMock()
    node.get_logger        = lambda: node._logger
    node.destroy_subscription = MagicMock()
    return node


# ── Status callbacks ──────────────────────────────────────────────────────────

class TestStatusCallback:

    def test_typed_callback_updates_global(self):
        node = _make_node()
        msg = MagicMock()
        msg.connected             = True
        msg.controller_type       = 'Xbox 360'
        msg.time_since_last_input = 1.5
        msg.timestamp             = 12345

        with patch.object(node, '_write_log'):
            node._status_callback(msg)

        assert dn.LATEST_CONTROLLER_STATUS['connected']       is True
        assert dn.LATEST_CONTROLLER_STATUS['controller_type'] == 'Xbox 360'

    def test_typed_callback_stores_disconnected(self):
        node = _make_node()
        msg = MagicMock()
        msg.connected             = False
        msg.controller_type       = 'Unknown'
        msg.time_since_last_input = 0.0
        msg.timestamp             = 0

        with patch.object(node, '_write_log'):
            node._status_callback(msg)

        assert dn.LATEST_CONTROLLER_STATUS['connected'] is False



# ── Robot watchdog ────────────────────────────────────────────────────────────

class TestRobotWatchdog:

    def setup_method(self):
        dn.ROBOT_ACTIVE = False

    def test_no_opencr_time_does_nothing(self):
        node = _make_node()
        node._last_opencr_time = None
        node._robot_watchdog()
        # Should not raise or change ROBOT_ACTIVE

    def test_recent_heartbeat_does_not_mark_offline(self):
        node = _make_node()
        node._last_opencr_time = time.monotonic()
        dn.ROBOT_ACTIVE = True
        node._robot_watchdog()
        assert dn.ROBOT_ACTIVE is True

    def test_stale_heartbeat_marks_offline(self):
        node = _make_node()
        node._last_opencr_time = time.monotonic() - 20.0   # 20 s ago
        dn.ROBOT_ACTIVE = True
        node._robot_watchdog()
        assert dn.ROBOT_ACTIVE is False


# ── GetRobotActions tests ─────────────────────────────────────────────────────

class TestGetRobotActions:

    def test_robot_offline_returns_none(self):
        """When robot is offline, get_robot_actions returns (None, error)."""
        node = _make_node()
        dn.ROBOT_ACTIVE = False
        actions, err = node.get_robot_actions('testuser')
        assert actions is None
        assert 'not connected' in err.lower()

    def test_success_parses_response(self):
        """When robot responds successfully, actions are parsed from JSON."""
        node = _make_node()
        dn.ROBOT_ACTIVE = True

        resp              = MagicMock()
        resp.success      = True
        resp.action_names = ['move']
        resp.action_types = ['joystick']
        resp.actions_json = ['{"name":"move","type":"joystick","tuples":[]}']

        future = MagicMock()
        future.done.return_value = True
        future.result.return_value = resp
        node._get_robot_actions_client = MagicMock()
        node._get_robot_actions_client.call_async.return_value = future

        actions, err = node.get_robot_actions('testuser')
        dn.ROBOT_ACTIVE = False

        assert err is None
        assert 'move' in actions
        assert actions['move']['type'] == 'joystick'


# ── Recording helpers ─────────────────────────────────────────────────────────

class TestRecordNewAction:

    def test_does_nothing_when_no_rec_file(self):
        node = _make_node()
        node._rec_file = None
        # Must not raise
        node.record_new_action('test', 'button_once', [])

    def test_writes_action_header(self):
        node = _make_node()
        fake_file = MagicMock()
        node._rec_file = fake_file

        node.record_new_action('my_action', 'button_once',
                               [('/cmd_vel', 'msg', 'std_msgs/msg/String',
                                 1.0, 1.0, 0.0, 0.0, 'topic', '')])

        written = ''.join(call.args[0] for call in fake_file.write.call_args_list)
        assert 'NEW ACTION DEFINED' in written
        assert 'my_action' in written
        assert 'button_once' in written

    def test_writes_service_tuple(self):
        node = _make_node()
        fake_file = MagicMock()
        node._rec_file = fake_file

        node.record_new_action('led', 'button_once',
                               [('', '{}', 'srv_type', 1.0, 1.0, 0.0, 0.0,
                                 'service', 'set_led_status')])

        written = ''.join(call.args[0] for call in fake_file.write.call_args_list)
        assert 'service=set_led_status' in written


class TestRecordLedCommand:

    def test_does_nothing_when_no_rec_file(self):
        node = _make_node()
        node._rec_file = None
        node.record_led_command(
            {'FL': {'color': 1, 'mode': 4}, 'FR': {'color': 1, 'mode': 4},
             'BL': {'color': 1, 'mode': 4}, 'BR': {'color': 1, 'mode': 4}},
            True, 'OK')

    def test_writes_all_corners(self):
        node = _make_node()
        fake_file = MagicMock()
        node._rec_file = fake_file

        node.record_led_command(
            {'FL': {'color': 2, 'mode': 4},
             'FR': {'color': 3, 'mode': 3},
             'BL': {'color': 1, 'mode': 1},
             'BR': {'color': 0, 'mode': 2}},
            True, 'OK')

        written = ''.join(call.args[0] for call in fake_file.write.call_args_list)
        assert 'FL' in written
        assert 'FR' in written
        assert 'BL' in written
        assert 'BR' in written
        assert 'GREEN' in written    # color 2
        assert 'SOLID' in written    # mode 4


# ── _make_rec_callback ────────────────────────────────────────────────────────

class TestMakeRecCallback:

    def test_callback_writes_msg_data(self):
        node = _make_node()
        fake_file = MagicMock()
        node._rec_files['/cmd_vel'] = fake_file

        cb = node._make_rec_callback('/cmd_vel')
        msg = MagicMock()
        msg.data = 'hello'

        with patch.object(dn, '_MSG_TO_DICT_AVAILABLE', False):
            cb(msg)

        written = ''.join(call.args[0] for call in fake_file.write.call_args_list)
        assert 'hello' in written

    def test_callback_skips_when_no_file(self):
        node = _make_node()
        # _rec_files is empty — callback should silently return
        cb = node._make_rec_callback('/cmd_vel')
        cb(MagicMock())   # must not raise


# ── Stop recording ────────────────────────────────────────────────────────────

class TestStopRecording:

    def test_clears_all_state(self):
        node = _make_node()
        fake_session = MagicMock()
        fake_topic   = MagicMock()
        fake_led     = MagicMock()

        node._rec_file     = fake_session
        node._rec_files    = {'/cmd_vel': fake_topic}
        node._rec_led_file = fake_led
        node._rec_subs     = {'/cmd_vel': MagicMock()}

        dn.RECORDING_STATE = {
            'active': True, 'user_name': 'alice',
            'topics': ['/cmd_vel'], 'dir': '/tmp/x', 'started_at': 'now'
        }

        node.stop_recording()

        assert node._rec_file     is None
        assert node._rec_led_file is None
        assert node._rec_files    == {}
        assert node._rec_subs     == {}
        assert dn.RECORDING_STATE['active'] is False
        assert dn.RECORDING_STATE['user_name'] == 'alice'   # preserved
        fake_session.close.assert_called_once()
        fake_topic.close.assert_called_once()
        fake_led.close.assert_called_once()
