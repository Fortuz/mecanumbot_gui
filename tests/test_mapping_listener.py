"""
test_mapping_listener.py — unit tests for the MappingListener ROS2 node.

Tests all service callbacks and controller event handlers with a mocked
ROS environment and database.
"""

import sys
import json
from unittest.mock import MagicMock, Mock, patch
import pytest

# ══════════════════════════════════════════════════════════════════════════════
# Mock ROS modules before importing mapping_listener
# ══════════════════════════════════════════════════════════════════════════════

class MockNode:
    """Minimal mock for rclpy.node.Node."""
    def __init__(self, name):
        self.name = name
        self._logger = MagicMock()
        self._clock = MagicMock()
        self._clock.now.return_value = MagicMock()
        self._clock.now.return_value.to_msg.return_value.sec = 1234567890

    def get_logger(self):
        return self._logger
    
    def get_clock(self):
        return self._clock
    
    def create_subscription(self, msg_type, topic, callback, qos, **kwargs):
        return MagicMock()
    
    def create_publisher(self, msg_type, topic, qos, **kwargs):
        pub = MagicMock()
        return pub
    
    def create_service(self, srv_type, name, callback, **kwargs):
        return MagicMock()
    
    def create_client(self, srv_type, name, **kwargs):
        client = MagicMock()
        client.service_is_ready.return_value = False
        return client
    
    def create_timer(self, period, callback, **kwargs):
        return MagicMock()
    
    def destroy_node(self):
        pass


# Mock all required ROS modules
sys.modules['rclpy'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock(Node=MockNode)
sys.modules['rclpy.executors'] = MagicMock()
sys.modules['rclpy.callback_groups'] = MagicMock()
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()
sys.modules['geometry_msgs'] = MagicMock()
sys.modules['geometry_msgs.msg'] = MagicMock()

# Mock mecanumbot_msgs
mock_msg = MagicMock()
mock_srv = MagicMock()
sys.modules['mecanumbot_msgs'] = MagicMock()
sys.modules['mecanumbot_msgs.msg'] = mock_msg
sys.modules['mecanumbot_msgs.srv'] = mock_srv

# Now import the node module
from button_mapping_ros.mapping_listener import MappingListener

# ══════════════════════════════════════════════════════════════════════════════
# Fixtures
# ══════════════════════════════════════════════════════════════════════════════

@pytest.fixture
def mock_db():
    """A mock IDatabase instance."""
    db = MagicMock()
    db.get_or_create_user.return_value = 1  # uid=1
    db.get_all_actions.return_value = {}
    db.get_all_mappings.return_value = []
    db.get_mapping.return_value = None
    db.save_mapping.return_value = True
    db.save_action.return_value = True
    db.delete_mapping.return_value = True
    db.delete_action.return_value = True
    db.get_mappings_using_action.return_value = []
    db.get_all_recording_schemes.return_value = {}
    db.save_recording_scheme.return_value = True
    db.delete_recording_scheme.return_value = True
    return db


@pytest.fixture
def node(mock_db):
    """A MappingListener node with mocked database."""
    return MappingListener(mock_db)


# ══════════════════════════════════════════════════════════════════════════════
# Test: GetRobotActions service
# ══════════════════════════════════════════════════════════════════════════════

class TestGetRobotActions:
    def test_returns_actions_correctly(self, node, mock_db):
        mock_db.get_all_actions.return_value = {
            "Drive": {
                "type": "button_once",
                "tuples": [("/cmd_vel", '{"linear":{"x":1.0}}', "geometry_msgs/msg/Twist")],
            }
        }
        request = MagicMock(user_name="alice")
        response = MagicMock()
        result = node._get_robot_actions_callback(request, response)
        
        assert result.success is True
        assert "Drive" in result.action_names
        assert "button_once" in result.action_types
        assert len(result.actions_json) == 1
        action_obj = json.loads(result.actions_json[0])
        assert action_obj["name"] == "Drive"

    def test_database_error_returns_failure(self, node, mock_db):
        mock_db.get_all_actions.side_effect = Exception("DB error")
        request = MagicMock(user_name="bob")
        response = MagicMock()
        result = node._get_robot_actions_callback(request, response)
        
        assert result.success is False
        assert "DB error" in result.message


# ══════════════════════════════════════════════════════════════════════════════
# Test: GetMappingNames service
# ══════════════════════════════════════════════════════════════════════════════

class TestGetMappingNames:
    def test_returns_mapping_names(self, node, mock_db):
        mock_db.get_all_mappings.return_value = [
            {"name": "Config1"},
            {"name": "Config2"},
        ]
        request = MagicMock(user_name="charlie")
        response = MagicMock()
        result = node._get_mapping_names_callback(request, response)
        
        assert result.success is True
        assert "Config1" in result.mapping_names
        assert "Config2" in result.mapping_names

    def test_database_error_handled(self, node, mock_db):
        mock_db.get_all_mappings.side_effect = Exception("DB fail")
        request = MagicMock(user_name="dave")
        response = MagicMock()
        result = node._get_mapping_names_callback(request, response)
        
        assert result.success is False
        assert "DB fail" in result.message


# ══════════════════════════════════════════════════════════════════════════════
# Test: GetMappingDetails service
# ══════════════════════════════════════════════════════════════════════════════

class TestGetMappingDetails:
    def test_found_returns_json(self, node, mock_db):
        mock_db.get_mapping.return_value = {
            "name": "Test",
            "buttons": [{"button": "A", "action": "Drive", "trigger_mode": "once"}],
            "joysticks": [],
        }
        request = MagicMock(user_name="eve", mapping_name="Test")
        response = MagicMock()
        result = node._get_mapping_details_callback(request, response)
        
        assert result.success is True
        data = json.loads(result.mapping_json)
        assert data["name"] == "Test"

    def test_not_found_returns_error(self, node, mock_db):
        mock_db.get_mapping.return_value = None
        request = MagicMock(user_name="frank", mapping_name="Missing")
        response = MagicMock()
        result = node._get_mapping_details_callback(request, response)
        
        assert result.success is False
        assert "not found" in result.message

    def test_database_error(self, node, mock_db):
        mock_db.get_mapping.side_effect = Exception("DB crash")
        request = MagicMock(user_name="grace", mapping_name="X")
        response = MagicMock()
        result = node._get_mapping_details_callback(request, response)
        
        assert result.success is False
        assert "DB crash" in result.message


# ══════════════════════════════════════════════════════════════════════════════
# Test: SaveMapping service
# ══════════════════════════════════════════════════════════════════════════════

class TestSaveMapping:
    def test_conflict_blocks_overwrite(self, node, mock_db):
        mock_db.get_mapping.return_value = {"name": "Existing"}
        request = MagicMock(
            user_name="heidi",
            mapping_name="Existing",
            overwrite=False,
            button_names=["A"],
            button_action_names=["Drive"],
            button_trigger_modes=["once"],
            joystick_names=[],
            joystick_action_names=[],
            actions=[],
        )
        response = MagicMock()
        result = node._save_mapping_callback(request, response)
        
        assert result.success is False
        assert "already exists" in result.message

    def test_overwrite_true_bypasses_conflict(self, node, mock_db):
        mock_db.get_mapping.return_value = {"name": "Old"}
        request = MagicMock(
            user_name="ivan",
            mapping_name="Old",
            overwrite=True,
            button_names=["B"],
            button_action_names=["Jump"],
            button_trigger_modes=["once"],
            joystick_names=[],
            joystick_action_names=[],
            actions=[],
        )
        response = MagicMock()
        result = node._save_mapping_callback(request, response)
        
        assert result.success is True
        mock_db.save_mapping.assert_called_once()

    def test_bundled_actions_saved(self, node, mock_db):
        action_desc = MagicMock()
        action_desc.name = "BundledAction"
        action_desc.action_type = "button_once"
        tuple_mock = MagicMock()
        tuple_mock.topic = "/test"
        tuple_mock.message = "{}"
        tuple_mock.message_type = "std_msgs/msg/String"
        tuple_mock.scale_x = 1.0
        tuple_mock.scale_y = 1.0
        tuple_mock.offset_x = 0.0
        tuple_mock.offset_y = 0.0
        tuple_mock.publish_type = "topic"
        tuple_mock.service_name = ""
        action_desc.tuples = [tuple_mock]
        
        request = MagicMock(
            user_name="judy",
            mapping_name="NewMapping",
            overwrite=False,
            button_names=["A"],
            button_action_names=["BundledAction"],
            button_trigger_modes=["once"],
            joystick_names=[],
            joystick_action_names=[],
            actions=[action_desc],
        )
        mock_db.get_mapping.return_value = None
        response = MagicMock()
        result = node._save_mapping_callback(request, response)
        
        assert result.success is True
        mock_db.save_action.assert_called_once()


# ══════════════════════════════════════════════════════════════════════════════
# Test: DeleteRobotMapping service
# ══════════════════════════════════════════════════════════════════════════════

class TestDeleteMapping:
    def test_successful_delete(self, node, mock_db):
        request = MagicMock(user_name="karl", mapping_name="ToDelete")
        response = MagicMock()
        result = node._delete_mapping_callback(request, response)
        
        assert result.success is True
        mock_db.delete_mapping.assert_called_once()

    def test_database_error_handled(self, node, mock_db):
        mock_db.delete_mapping.side_effect = Exception("Delete fail")
        request = MagicMock(user_name="lisa", mapping_name="X")
        response = MagicMock()
        result = node._delete_mapping_callback(request, response)
        
        assert result.success is False
        assert "Delete fail" in result.message


# ══════════════════════════════════════════════════════════════════════════════
# Test: ApplyMapping service
# ══════════════════════════════════════════════════════════════════════════════

class TestApplyMapping:
    def test_from_robot_db_loads_from_db(self, node, mock_db):
        mock_db.get_mapping.return_value = {
            "buttons": [{"button": "A", "action": "Drive", "trigger_mode": "once"}],
            "joysticks": [],
        }
        mock_db.get_all_actions.return_value = {
            "Drive": {
                "type": "button_once",
                "tuples": [("/cmd_vel", '{}', "std_msgs/msg/String")],
            }
        }
        request = MagicMock(
            user_name="mike",
            mapping_name="RobotConfig",
            from_robot_db=True,
            button_names=[],
            button_action_names=[],
            button_trigger_modes=[],
            joystick_names=[],
            joystick_action_names=[],
            actions=[],
        )
        response = MagicMock()
        result = node._apply_mapping_callback(request, response)
        
        assert result.success is True
        assert "Drive" in node.actions
        assert "A" in node.button_mappings

    def test_from_robot_db_false_uses_inline(self, node, mock_db):
        action_desc = MagicMock()
        action_desc.name = "InlineAction"
        action_desc.action_type = "button_once"
        tuple_mock = MagicMock()
        tuple_mock.topic = "/inline"
        tuple_mock.message = "{}"
        tuple_mock.message_type = "std_msgs/msg/String"
        tuple_mock.scale_x = 1.0
        tuple_mock.scale_y = 1.0
        tuple_mock.offset_x = 0.0
        tuple_mock.offset_y = 0.0
        tuple_mock.publish_type = "topic"
        tuple_mock.service_name = ""
        action_desc.tuples = [tuple_mock]
        
        request = MagicMock(
            user_name="nancy",
            mapping_name="HostConfig",
            from_robot_db=False,
            button_names=["B"],
            button_action_names=["InlineAction"],
            button_trigger_modes=["hold"],
            joystick_names=[],
            joystick_action_names=[],
            actions=[action_desc],
        )
        response = MagicMock()
        result = node._apply_mapping_callback(request, response)
        
        assert result.success is True
        assert "InlineAction" in node.actions
        assert node.button_mappings["B"]["trigger_mode"] == "hold"

    def test_mapping_not_found_returns_error(self, node, mock_db):
        mock_db.get_mapping.return_value = None
        request = MagicMock(
            user_name="oscar",
            mapping_name="Ghost",
            from_robot_db=True,
            button_names=[],
            button_action_names=[],
            button_trigger_modes=[],
            joystick_names=[],
            joystick_action_names=[],
            actions=[],
        )
        response = MagicMock()
        result = node._apply_mapping_callback(request, response)
        
        assert result.success is False
        assert "not found" in result.message

    def test_state_fully_replaced_on_each_call(self, node, mock_db):
        # First apply
        node.actions = {"Old": {}}
        node.button_mappings = {"X": {}}
        
        mock_db.get_mapping.return_value = {
            "buttons": [],
            "joysticks": [],
        }
        request = MagicMock(
            user_name="paul",
            mapping_name="NewEmpty",
            from_robot_db=True,
            button_names=[],
            button_action_names=[],
            button_trigger_modes=[],
            joystick_names=[],
            joystick_action_names=[],
            actions=[],
        )
        response = MagicMock()
        node._apply_mapping_callback(request, response)
        
        # Old state should be cleared
        assert "Old" not in node.actions
        assert "X" not in node.button_mappings


# ══════════════════════════════════════════════════════════════════════════════
# Test: SaveRobotAction service
# ══════════════════════════════════════════════════════════════════════════════

class TestSaveRobotAction:
    def test_conflict_blocks_save(self, node, mock_db):
        mock_db.get_all_actions.return_value = {"Existing": {}}
        action_desc = MagicMock()
        action_desc.name = "Existing"
        action_desc.action_type = "button_once"
        action_desc.tuples = []
        request = MagicMock(
            user_name="quinn",
            action=action_desc,
            overwrite=False,
        )
        response = MagicMock()
        result = node._save_robot_action_callback(request, response)
        
        assert result.success is False
        assert "already exists" in result.message

    def test_overwrite_true_saves(self, node, mock_db):
        mock_db.get_all_actions.return_value = {"Old": {}}
        action_desc = MagicMock()
        action_desc.name = "Old"
        action_desc.action_type = "button_hold"
        tuple_mock = MagicMock()
        tuple_mock.topic = "/new"
        tuple_mock.message = "{}"
        tuple_mock.message_type = "std_msgs/msg/String"
        tuple_mock.scale_x = 1.0
        tuple_mock.scale_y = 1.0
        tuple_mock.offset_x = 0.0
        tuple_mock.offset_y = 0.0
        tuple_mock.publish_type = "topic"
        tuple_mock.service_name = ""
        action_desc.tuples = [tuple_mock]
        request = MagicMock(
            user_name="rachel",
            action=action_desc,
            overwrite=True,
        )
        response = MagicMock()
        result = node._save_robot_action_callback(request, response)
        
        assert result.success is True
        mock_db.save_action.assert_called_once()

    def test_database_error(self, node, mock_db):
        mock_db.save_action.side_effect = Exception("Save error")
        action_desc = MagicMock()
        action_desc.name = "Test"
        action_desc.action_type = "button_once"
        action_desc.tuples = []
        request = MagicMock(
            user_name="sam",
            action=action_desc,
            overwrite=True,
        )
        response = MagicMock()
        result = node._save_robot_action_callback(request, response)
        
        assert result.success is False


# ══════════════════════════════════════════════════════════════════════════════
# Test: DeleteRobotAction service
# ══════════════════════════════════════════════════════════════════════════════

class TestDeleteRobotAction:
    def test_removes_from_db_and_memory(self, node, mock_db):
        node.actions["ToDelete"] = {"type": "button_once", "tuples": []}
        request = MagicMock(user_name="tina", action_name="ToDelete")
        response = MagicMock()
        result = node._delete_robot_action_callback(request, response)
        
        assert result.success is True
        assert "ToDelete" not in node.actions
        mock_db.delete_action.assert_called_once()

    def test_database_error_handled(self, node, mock_db):
        mock_db.delete_action.side_effect = Exception("Delete failed")
        request = MagicMock(user_name="uma", action_name="X")
        response = MagicMock()
        result = node._delete_robot_action_callback(request, response)
        
        assert result.success is False


# ══════════════════════════════════════════════════════════════════════════════
# Test: GetActionUsages service
# ══════════════════════════════════════════════════════════════════════════════

class TestGetActionUsages:
    def test_returns_mapping_names(self, node, mock_db):
        mock_db.get_mappings_using_action.return_value = ["Config1", "Config2"]
        request = MagicMock(user_name="victor", action_name="SharedAction")
        response = MagicMock()
        result = node._get_action_usages_callback(request, response)
        
        assert result.success is True
        assert "Config1" in result.mapping_names
        assert "Config2" in result.mapping_names

    def test_empty_list_when_no_usages(self, node, mock_db):
        mock_db.get_mappings_using_action.return_value = []
        request = MagicMock(user_name="wendy", action_name="Unused")
        response = MagicMock()
        result = node._get_action_usages_callback(request, response)
        
        assert result.success is True
        assert result.mapping_names == []

    def test_error_handled(self, node, mock_db):
        mock_db.get_mappings_using_action.side_effect = Exception("Error")
        request = MagicMock(user_name="xander", action_name="Test")
        response = MagicMock()
        result = node._get_action_usages_callback(request, response)
        
        assert result.success is False


# ══════════════════════════════════════════════════════════════════════════════
# Test: Recording scheme services
# ══════════════════════════════════════════════════════════════════════════════

class TestRecordingSchemeCallbacks:
    def test_get_schemes_success(self, node, mock_db):
        mock_db.get_all_recording_schemes.return_value = {
            "Full": ["/cmd_vel", "/joy"],
        }
        request = MagicMock(user_name="yara")
        response = MagicMock()
        result = node._get_recording_schemes_callback(request, response)
        
        assert result.success is True
        assert "Full" in result.scheme_names

    def test_save_scheme_conflict(self, node, mock_db):
        mock_db.get_all_recording_schemes.return_value = {"Existing": []}
        request = MagicMock(
            user_name="zach",
            scheme_name="Existing",
            topics=["/new"],
            overwrite=False,
        )
        response = MagicMock()
        result = node._save_recording_scheme_callback(request, response)
        
        assert result.success is False
        assert "already exists" in result.message

    def test_save_scheme_overwrite(self, node, mock_db):
        mock_db.get_all_recording_schemes.return_value = {"Old": []}
        request = MagicMock(
            user_name="anna",
            scheme_name="Old",
            topics=["/updated"],
            overwrite=True,
        )
        response = MagicMock()
        result = node._save_recording_scheme_callback(request, response)
        
        assert result.success is True
        mock_db.save_recording_scheme.assert_called_once()

    def test_delete_scheme(self, node, mock_db):
        request = MagicMock(user_name="ben", scheme_name="ToDelete")
        response = MagicMock()
        result = node._delete_recording_scheme_callback(request, response)
        
        assert result.success is True
        mock_db.delete_recording_scheme.assert_called_once()


# ══════════════════════════════════════════════════════════════════════════════
# Test: Message parsing
# ══════════════════════════════════════════════════════════════════════════════

class TestParseMessageData:
    def test_parse_string_message(self, node):
        msg = node.parse_message_data('{"data":"hello"}', 'std_msgs/msg/String')
        assert msg is not None
        assert msg.data == "hello"

    def test_parse_bool_message(self, node):
        msg = node.parse_message_data('{"data":true}', 'std_msgs/msg/Bool')
        assert msg is not None
        assert msg.data is True

    def test_parse_twist_message(self, node):
        msg = node.parse_message_data(
            '{"linear":{"x":1.5,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0.5}}',
            'geometry_msgs/msg/Twist'
        )
        assert msg is not None
        assert msg.linear.x == 1.5
        assert msg.angular.z == 0.5

    def test_bad_json_returns_none(self, node):
        msg = node.parse_message_data('not json', 'std_msgs/msg/String')
        assert msg is None

    def test_unknown_type_returns_none(self, node):
        msg = node.parse_message_data('{}', 'unknown/msg/Type')
        assert msg is None


# ══════════════════════════════════════════════════════════════════════════════
# Test: Button callbacks
# ══════════════════════════════════════════════════════════════════════════════

class TestButtonCallback:
    def test_unmapped_button_no_publish(self, node):
        event = MagicMock()
        event.button_name = "Unmapped"
        event.event = "PRESSED"
        node.xbox_button_callback(event)
        # Should not crash; no action taken

    def test_once_pressed_publishes(self, node):
        node.actions = {"TestAction": {"type": "button_once", "tuples": []}}
        node.button_mappings = {"A": {"action": "TestAction", "trigger_mode": "once"}}
        
        with patch.object(node, 'publish_action') as mock_publish:
            event = MagicMock()
            event.button_name = "A"
            event.event = "PRESSED"
            node.xbox_button_callback(event)
            mock_publish.assert_called_once_with("TestAction", "A")

    def test_once_hold_no_publish(self, node):
        node.actions = {"TestAction": {"type": "button_once", "tuples": []}}
        node.button_mappings = {"A": {"action": "TestAction", "trigger_mode": "once"}}
        
        with patch.object(node, 'publish_action') as mock_publish:
            event = MagicMock()
            event.button_name = "A"
            event.event = "HOLD"
            node.xbox_button_callback(event)
            mock_publish.assert_not_called()

    def test_hold_pressed_publishes(self, node):
        node.actions = {"HoldAction": {"type": "button_hold", "tuples": []}}
        node.button_mappings = {"B": {"action": "HoldAction", "trigger_mode": "hold"}}
        
        with patch.object(node, 'publish_action') as mock_publish:
            event = MagicMock()
            event.button_name = "B"
            event.event = "PRESSED"
            node.xbox_button_callback(event)
            mock_publish.assert_called_once_with("HoldAction", "B")

    def test_hold_hold_publishes(self, node):
        node.actions = {"HoldAction": {"type": "button_hold", "tuples": []}}
        node.button_mappings = {"B": {"action": "HoldAction", "trigger_mode": "hold"}}
        
        with patch.object(node, 'publish_action') as mock_publish:
            event = MagicMock()
            event.button_name = "B"
            event.event = "HOLD"
            node.xbox_button_callback(event)
            mock_publish.assert_called_once()

    def test_action_missing_warns(self, node):
        node.button_mappings = {"X": {"action": "MissingAction", "trigger_mode": "once"}}
        event = MagicMock()
        event.button_name = "X"
        event.event = "PRESSED"
        # Should log warning but not crash
        node.xbox_button_callback(event)


# ══════════════════════════════════════════════════════════════════════════════
# Test: Joystick callbacks
# ══════════════════════════════════════════════════════════════════════════════

class TestJoystickCallback:
    def test_unmapped_joystick_no_publish(self, node):
        event = MagicMock()
        event.joystick_name = "Unmapped Stick"
        event.x = 0.5
        event.y = 0.3
        node.xbox_joystick_callback(event)
        # Should not crash

    def test_mapped_joystick_publishes(self, node):
        node.actions = {"Drive": {"type": "joystick", "tuples": []}}
        node.joystick_mappings = {"Left Stick": "Drive"}
        
        with patch.object(node, 'publish_joystick_action') as mock_publish:
            event = MagicMock()
            event.joystick_name = "Left Stick"
            event.x = 0.8
            event.y = -0.3
            node.xbox_joystick_callback(event)
            mock_publish.assert_called_once_with("Drive", "Left Stick", 0.8, -0.3)


# ══════════════════════════════════════════════════════════════════════════════
# Test: Joystick action publishing
# ══════════════════════════════════════════════════════════════════════════════

class TestPublishJoystickAction:
    def test_scale_offset_applied(self, node):
        node.actions = {
            "ScaledAction": {
                "type": "joystick",
                "tuples": [
                    ("/test", '{"x":"X","y":"Y"}', "std_msgs/msg/String", 2.0, 3.0, 0.1, 0.2, "topic", ""),
                ],
            }
        }
        
        with patch.object(node, '_publish_single') as mock_pub:
            node.publish_joystick_action("ScaledAction", "Left Stick", 1.0, 1.0)
            # x = 1.0 * 2.0 + 0.1 = 2.1, y = 1.0 * 3.0 + 0.2 = 3.2
            call_args = mock_pub.call_args[0]
            message_str = call_args[1]
            assert "2.1" in message_str
            assert "3.2" in message_str

    def test_service_publish_type_skipped(self, node):
        node.actions = {
            "ServiceAction": {
                "type": "joystick",
                "tuples": [
                    ("/test", '{}', "std_msgs/msg/String", 1.0, 1.0, 0.0, 0.0, "service", "my_service"),
                ],
            }
        }
        
        with patch.object(node, '_publish_single') as mock_pub:
            node.publish_joystick_action("ServiceAction", "Left Stick", 0.5, 0.5)
            # Service tuples should be skipped for joystick
            mock_pub.assert_not_called()
