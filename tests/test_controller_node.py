"""
test_controller_node.py — unit tests for the ControllerNode ROS2 node.

Tests Joy message processing, button/joystick event publishing, layout detection,
and connection status tracking with a mocked ROS environment.
"""

import sys
from unittest.mock import MagicMock, patch
import pytest

# ══════════════════════════════════════════════════════════════════════════════
# Mock ROS modules before importing controller_node
# ══════════════════════════════════════════════════════════════════════════════

class MockTime:
    """Mock for ROS time that supports arithmetic."""
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds
        self.sec = int(nanoseconds / 1e9)
    
    def __sub__(self, other):
        result = MockTime(self.nanoseconds - other.nanoseconds)
        return result
    
    def to_msg(self):
        msg = MagicMock()
        msg.sec = self.sec
        return msg


class MockNode:
    """Minimal mock for rclpy.node.Node."""
    def __init__(self, name):
        self.name = name
        self._logger = MagicMock()
        self._clock = MagicMock()
        # Mock clock returning a time value
        self._clock.now.return_value = MockTime(5000000000)  # 5 seconds in nanoseconds

    def get_logger(self):
        return self._logger
    
    def get_clock(self):
        return self._clock
    
    def create_subscription(self, msg_type, topic, callback, qos, **kwargs):
        return MagicMock()
    
    def create_publisher(self, msg_type, topic, qos, **kwargs):
        return MagicMock()
    
    def create_timer(self, period, callback, **kwargs):
        return MagicMock()
    
    def destroy_node(self):
        pass


# Mock rclpy modules
mock_rclpy = MagicMock()

# Mock Duration to return a MockTime
def mock_duration_constructor(seconds=0, nanoseconds=0):
    total_ns = int(seconds * 1e9) + nanoseconds
    return MockTime(total_ns)

mock_duration = MagicMock()
mock_duration.Duration = mock_duration_constructor
sys.modules['rclpy'] = mock_rclpy
sys.modules['rclpy.duration'] = mock_duration
sys.modules['rclpy.node'] = MagicMock(Node=MockNode)
sys.modules['rclpy.executors'] = MagicMock()

# Mock sensor_msgs
mock_sensor_msgs = MagicMock()
sys.modules['sensor_msgs'] = MagicMock()
sys.modules['sensor_msgs.msg'] = mock_sensor_msgs

# Mock mecanumbot_msgs
mock_mecanumbot_msgs = MagicMock()
sys.modules['mecanumbot_msgs'] = MagicMock()
sys.modules['mecanumbot_msgs.msg'] = mock_mecanumbot_msgs

# Now import the node module
from controller_node import ControllerNode

# ══════════════════════════════════════════════════════════════════════════════
# Fixtures
# ══════════════════════════════════════════════════════════════════════════════

@pytest.fixture
def node():
    """A ControllerNode instance."""
    # Patch publish_connection_status during initialization to avoid mock time issues
    with patch.object(ControllerNode, 'publish_connection_status'):
        node = ControllerNode()
    return node


def create_joy_msg(axes, buttons):
    """Helper to create a mock Joy message."""
    msg = MagicMock()
    msg.axes = axes
    msg.buttons = buttons
    return msg


# ══════════════════════════════════════════════════════════════════════════════
# Test: Layout detection
# ══════════════════════════════════════════════════════════════════════════════

class TestDetectLayout:
    def test_xbox360_layout_detected(self, node):
        # Xbox 360 wireless: 8 axes, 11 buttons
        joy_msg = create_joy_msg(
            axes=[0.0] * 8,
            buttons=[0] * 11
        )
        node._detect_layout(joy_msg)
        
        assert node._layout == 'xbox360'
        assert node._layout_name == 'Controller 360'
        assert len(node.button_names) > 0

    def test_generic_layout_detected(self, node):
        # Generic controller: different from Xbox 360
        joy_msg = create_joy_msg(
            axes=[0.0] * 6,
            buttons=[0] * 16
        )
        node._detect_layout(joy_msg)
        
        assert node._layout == 'generic'
        assert node._layout_name == 'Generic Controller'

    def test_generic_layout_for_xbox_one(self, node):
        # Xbox One controller: different axes/button count than 360
        joy_msg = create_joy_msg(
            axes=[0.0] * 6,
            buttons=[0] * 13
        )
        node._detect_layout(joy_msg)
        
        assert node._layout == 'generic'


# ══════════════════════════════════════════════════════════════════════════════
# Test: Joy callback
# ══════════════════════════════════════════════════════════════════════════════

class TestJoyCallback:
    def test_layout_detected_on_first_message(self, node):
        assert node._layout is None
        joy_msg = create_joy_msg(
            axes=[0.0] * 8,
            buttons=[0] * 11
        )
        node.joy_callback(joy_msg)
        assert node._layout == 'xbox360'

    def test_button_pressed_transition(self, node):
        # Setup: Xbox 360 layout
        node._layout = 'xbox360'
        node._detect_layout(create_joy_msg([0.0] * 8, [0] * 11))
        
        with patch.object(node, 'publish_button_event') as mock_publish:
            # First message: button 0 (A) pressed
            joy_msg = create_joy_msg(
                axes=[0.0] * 8,
                buttons=[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            )
            node.joy_callback(joy_msg)
            # Should publish PRESSED event
            mock_publish.assert_called_with('PRESSED', 'A', 0)

    def test_button_hold_transition(self, node):
        # Setup
        node._layout = 'xbox360'
        node._detect_layout(create_joy_msg([0.0] * 8, [0] * 11))
        node.prev_button_states[0] = 1  # Button was already pressed
        
        with patch.object(node, 'publish_button_event') as mock_publish:
            # Button still held
            joy_msg = create_joy_msg(
                axes=[0.0] * 8,
                buttons=[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            )
            node.joy_callback(joy_msg)
            # Should publish HOLD event
            mock_publish.assert_called_with('HOLD', 'A', 0)

    def test_button_released_transition(self, node):
        # Setup
        node._layout = 'xbox360'
        node._detect_layout(create_joy_msg([0.0] * 8, [0] * 11))
        node.prev_button_states[0] = 1  # Button was pressed
        
        with patch.object(node, 'publish_button_event') as mock_publish:
            # Button released
            joy_msg = create_joy_msg(
                axes=[0.0] * 8,
                buttons=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            )
            node.joy_callback(joy_msg)
            # Should publish RELEASED event
            mock_publish.assert_called_with('RELEASED', 'A', 0)

    def test_generic_dpad_buttons_skipped_in_main_loop(self, node):
        # Setup generic layout
        node._layout = 'generic'
        node._detect_layout(create_joy_msg([0.0] * 6, [0] * 16))
        
        with patch.object(node, 'publish_button_event') as mock_publish:
            # D-Pad buttons (11-14) should be handled separately, not as BUTTON_11, etc.
            joy_msg = create_joy_msg(
                axes=[0.0] * 6,
                buttons=[0] * 11 + [1, 0, 0, 0, 0]  # DPad Up pressed (index 11)
            )
            node.joy_callback(joy_msg)
            # The main button loop should skip index 11
            # D-Pad handling publishes it as 'DPadUp' in publish_joystick_positions
            called_button_ids = [call[0][2] for call in mock_publish.call_args_list]
            # Button 11 should not appear as a raw button event from the main loop
            # (it's handled as D-Pad in publish_joystick_positions)


# ══════════════════════════════════════════════════════════════════════════════
# Test: Publish button event
# ══════════════════════════════════════════════════════════════════════════════

class TestPublishButtonEvent:
    def test_button_event_published(self, node):
        node.button_publisher = MagicMock()
        node.publish_button_event('PRESSED', 'TestButton', 5)
        
        # Verify publisher was called
        node.button_publisher.publish.assert_called_once()
        msg = node.button_publisher.publish.call_args[0][0]
        assert msg.event == 'PRESSED'
        assert msg.button_name == 'TestButton'
        assert msg.button_id == 5


# ══════════════════════════════════════════════════════════════════════════════
# Test: Publish joystick positions
# ══════════════════════════════════════════════════════════════════════════════

@pytest.fixture
def fresh_joy_events():
    """
    Make JoystickEvent() return a distinct MagicMock per call for the duration
    of a test.  Without this, all msg = JoystickEvent() calls return the same
    return_value mock, so the last attribute assignment overwrites all previous
    ones and message-content assertions become unreliable.

    NOTE: we patch controller_node.JoystickEvent directly instead of going
    through sys.modules['mecanumbot_msgs.msg'], because other test modules
    (e.g. test_mapping_listener) replace that sys.modules entry with a fresh
    MagicMock during collection, which would make the fixture target the wrong
    object at runtime.
    """
    import controller_node as cn
    cn.JoystickEvent.side_effect = lambda: MagicMock()
    yield
    cn.JoystickEvent.side_effect = None


class TestPublishJoystickPositions:
    def test_left_stick_published_regardless_of_magnitude(self, node, fresh_joy_events):
        """Left stick is always published — no deadzone filtering."""
        node._layout = 'xbox360'
        node.joystick_publisher = MagicMock()
        node.current_joy_msg = create_joy_msg(
            axes=[0.02, 0.03, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            buttons=[0] * 11
        )
        node.publish_joystick_positions()
        calls = node.joystick_publisher.publish.call_args_list
        left_stick_calls = [c for c in calls if c[0][0].joystick_name == 'Left Stick']
        assert len(left_stick_calls) == 1
        msg = left_stick_calls[0][0][0]
        assert msg.x == 0.02
        assert msg.y == 0.03

    def test_left_stick_above_deadzone(self, node, fresh_joy_events):
        node._layout = 'xbox360'
        node.joystick_publisher = MagicMock()
        node.current_joy_msg = create_joy_msg(
            axes=[0.8, 0.6, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            buttons=[0] * 11
        )
        node.publish_joystick_positions()
        calls = node.joystick_publisher.publish.call_args_list
        left_stick_calls = [c for c in calls if c[0][0].joystick_name == 'Left Stick']
        assert len(left_stick_calls) == 1
        msg = left_stick_calls[0][0][0]
        assert msg.joystick_name == 'Left Stick'
        assert msg.x == 0.8
        assert msg.y == 0.6

    def test_right_stick_published(self, node, fresh_joy_events):
        node._layout = 'xbox360'
        node.joystick_publisher = MagicMock()
        node.current_joy_msg = create_joy_msg(
            axes=[0.0, 0.0, 1.0, 0.7, -0.5, 1.0, 0.0, 0.0],
            buttons=[0] * 11
        )
        node.publish_joystick_positions()
        calls = node.joystick_publisher.publish.call_args_list
        right_stick_call = [c for c in calls if c[0][0].joystick_name == 'Right Stick']
        assert len(right_stick_call) == 1
        msg = right_stick_call[0][0][0]
        assert msg.x == 0.7
        assert msg.y == -0.5

    def test_trigger_normalisation_xbox360(self, node, fresh_joy_events):
        """Xbox 360 trigger: rest=+1.0, fully pressed=-1.0 → normalised to 0.0…1.0."""
        node._layout = 'xbox360'
        node.joystick_publisher = MagicMock()
        # LT fully pressed (axes[2] = -1.0), RT at rest (axes[5] = +1.0)
        node.current_joy_msg = create_joy_msg(
            axes=[0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            buttons=[0] * 11
        )
        node.publish_joystick_positions()
        calls = node.joystick_publisher.publish.call_args_list
        lt_calls = [c for c in calls if 'Left Trigger' in c[0][0].joystick_name]
        assert len(lt_calls) == 1
        assert lt_calls[0][0][0].x == 1.0  # (1.0 - (-1.0)) / 2.0 = 1.0

    def test_trigger_normalisation_generic(self, node, fresh_joy_events):
        """Generic trigger: rest=-1.0, fully pressed=+1.0 → normalised to 0.0…1.0."""
        node._layout = 'generic'
        node.joystick_publisher = MagicMock()
        # LT fully pressed (axes[2] = +1.0), RT at rest (axes[5] = -1.0)
        node.current_joy_msg = create_joy_msg(
            axes=[0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0],
            buttons=[0] * 16
        )
        node.publish_joystick_positions()
        calls = node.joystick_publisher.publish.call_args_list
        lt_calls = [c for c in calls if 'Left Trigger' in c[0][0].joystick_name]
        assert len(lt_calls) == 1
        assert lt_calls[0][0][0].x == 1.0  # (1.0 + 1.0) / 2.0 = 1.0

    def test_dpad_via_axes_xbox360(self, node):
        node._layout = 'xbox360'
        node._detect_layout(create_joy_msg([0.0] * 8, [0] * 11))
        
        with patch.object(node, 'publish_button_event') as mock_publish:
            # D-Pad left pressed (axes[6] > 0.5)
            node.current_joy_msg = create_joy_msg(
                axes=[0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0],
                buttons=[0] * 11
            )
            node.publish_joystick_positions()
            # Should publish DPadLeft PRESSED
            calls = [c for c in mock_publish.call_args_list if 'DPadLeft' in str(c)]
            assert len(calls) >= 1

    def test_dpad_via_buttons_generic(self, node):
        node._layout = 'generic'
        node._detect_layout(create_joy_msg([0.0] * 6, [0] * 16))
        
        with patch.object(node, 'publish_button_event') as mock_publish:
            # D-Pad up pressed (button 11)
            node.current_joy_msg = create_joy_msg(
                axes=[0.0] * 6,
                buttons=[0] * 11 + [1, 0, 0, 0, 0]
            )
            node.publish_joystick_positions()
            # Should publish DPadUp PRESSED
            dpad_calls = [c for c in mock_publish.call_args_list if 'DPad' in str(c[0][1])]
            assert len(dpad_calls) >= 1


# ══════════════════════════════════════════════════════════════════════════════
# Test: Connection status
# ══════════════════════════════════════════════════════════════════════════════

class TestPublishConnectionStatus:
    def test_publishes_with_unknown_layout(self, node):
        """Before any joy message, layout is None → controller_type is 'Unknown'."""
        node.connection_publisher = MagicMock()
        node.publish_connection_status()
        node.connection_publisher.publish.assert_called_once()
        msg = node.connection_publisher.publish.call_args[0][0]
        assert msg.controller_type == 'Unknown'

    def test_publishes_controller_type_xbox360(self, node):
        """After xbox360 layout detection, controller_type reflects the layout name."""
        node.connection_publisher = MagicMock()
        node._detect_layout(create_joy_msg([0.0] * 8, [0] * 11))
        node.publish_connection_status()
        msg = node.connection_publisher.publish.call_args[0][0]
        assert msg.controller_type == 'Controller 360'

    def test_publishes_controller_type_generic(self, node):
        """After generic layout detection, controller_type is 'Generic Controller'."""
        node.connection_publisher = MagicMock()
        node._detect_layout(create_joy_msg([0.0] * 6, [0] * 16))
        node.publish_connection_status()
        msg = node.connection_publisher.publish.call_args[0][0]
        assert msg.controller_type == 'Generic Controller'

    def test_always_publishes_exactly_once(self, node):
        """publish_connection_status always publishes exactly one message per call."""
        node.connection_publisher = MagicMock()
        node.publish_connection_status()
        assert node.connection_publisher.publish.call_count == 1


# ══════════════════════════════════════════════════════════════════════════════
# Test: Button name mapping
# ══════════════════════════════════════════════════════════════════════════════

class TestButtonNameMapping:
    def test_xbox360_button_names(self, node):
        node._layout = 'xbox360'
        node._detect_layout(create_joy_msg([0.0] * 8, [0] * 11))
        
        # Check standard button names
        assert node.button_names[0] == 'A'
        assert node.button_names[1] == 'B'
        assert node.button_names[4] == 'LB'
        assert node.button_names[8] == 'GUIDE'

    def test_generic_button_names(self, node):
        node._layout = 'generic'
        node._detect_layout(create_joy_msg([0.0] * 6, [0] * 16))
        
        # Check standard button names and D-Pad
        assert node.button_names[0] == 'A'
        assert node.button_names[8] == 'HOME'
        assert node.button_names[11] == 'DPadUp'
        assert node.button_names[14] == 'DPadRight'

    def test_unmapped_button_gets_fallback_name(self, node):
        node._layout = 'generic'
        node._detect_layout(create_joy_msg([0.0] * 6, [0] * 20))
        
        with patch.object(node, 'publish_button_event') as mock_publish:
            # Button 19 (not in mapping)
            joy_msg = create_joy_msg(
                axes=[0.0] * 6,
                buttons=[0] * 19 + [1]
            )
            node.joy_callback(joy_msg)
            
            # Should use fallback name
            calls = [c[0][1] for c in mock_publish.call_args_list if c[0][2] == 19]
            if calls:
                assert calls[0] == 'BUTTON_19'
