#!/usr/bin/env python3
"""
Controller Button Publisher Node

This ROS 2 node listens to controller input via the joy package and
publishes button press events to a custom topic.

It auto-detects the controller layout from the first /joy message:

  Xbox 360 wireless (xpad driver) — 8 axes, 11 buttons
    - Axes 0-1: Left Stick (X/Y)
    - Axes 2:   Left Trigger  (LT)  rest=+1, fully pressed=-1
    - Axes 3-4: Right Stick (X/Y)
    - Axes 5:   Right Trigger (RT)  rest=+1, fully pressed=-1
    - Axes 6-7: D-Pad (left/right, down/up) as ±1 floats
    - Buttons 0-10: A B X Y LB RB BACK START GUIDE LS RS

  Generic / Xbox One / PS4 / other — anything else
    - Axes 0-1: Left Stick (X/Y)
    - Axes 2:   Left Trigger (LT)   rest=-1, fully pressed=+1
    - Axes 3-4: Right Stick (X/Y)
    - Axes 5:   Right Trigger (RT)  rest=-1, fully pressed=+1
    - D-Pad reported as BUTTONS (indices defined by GENERIC_DPAD_* constants)
    - Buttons 0-3: A/Cross  B/Circle  X/Square  Y/Triangle

Requirements:
- ros-humble-joy package
- Any HID-compatible gamepad

"""

import rclpy
import rclpy.duration
from rclpy.node import Node
from sensor_msgs.msg import Joy
from mecanumbot_msgs.msg import ButtonEvent, JoystickEvent, ControllerStatus


# ── Layout fingerprint ────────────────────────────────────────────────────────
# Xbox 360 wireless via xpad produces exactly these counts.
_XBOX360_NUM_AXES    = 8
_XBOX360_NUM_BUTTONS = 11

# ── Generic D-Pad button indices ──────────────────────────────────────────────
# Most generic USB gamepads, Xbox One (xpad), and DS4 (via ds4drv) report
# D-Pad as buttons starting around index 11.  Adjust if your controller
# differs; the layout is printed to the log on first connection.
GENERIC_DPAD_UP    = 11
GENERIC_DPAD_DOWN  = 12
GENERIC_DPAD_LEFT  = 13
GENERIC_DPAD_RIGHT = 14


class ControllerNode(Node):
    """
    ROS 2 Node that processes controller input and publishes button /
    joystick events.  Supports Xbox 360 wireless (xpad layout) and
    generic HID gamepads automatically.
    """

    # ── Per-layout button name tables ─────────────────────────────────────────
    _BUTTON_NAMES_XBOX360 = {
        0: 'A', 1: 'B', 2: 'X', 3: 'Y',
        4: 'LB', 5: 'RB',
        6: 'BACK', 7: 'START', 8: 'GUIDE',
        9: 'LS', 10: 'RS',
    }

    _BUTTON_NAMES_GENERIC = {
        0: 'A', 1: 'B', 2: 'X', 3: 'Y',
        4: 'LB', 5: 'RB',
        6: 'BACK', 7: 'START', 8: 'HOME',
        9: 'LS', 10: 'RS',
        GENERIC_DPAD_UP:    'DPadUp',
        GENERIC_DPAD_DOWN:  'DPadDown',
        GENERIC_DPAD_LEFT:  'DPadLeft',
        GENERIC_DPAD_RIGHT: 'DPadRight',
    }

    def __init__(self):
        super().__init__('controller_node')

        # Detected layout: None until first /joy message, then 'xbox360' or 'generic'
        self._layout      = None
        self._layout_name = 'Unknown'
        self.button_names = {}

        # Track previous D-Pad axis states (xbox360 layout)
        self.prev_dpad_states = {'DPadLeft': False, 'DPadRight': False,
                                 'DPadUp': False, 'DPadDown': False}

        # Store previous button states to detect press/release events
        self.prev_button_states = {}

        # Create subscription to joy topic
        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Create publisher for button events
        self.button_publisher = self.create_publisher(
            ButtonEvent,
            '/controller/button_events',
            10
        )
        
        # Create publisher for joystick events
        self.joystick_publisher = self.create_publisher(
            JoystickEvent,
            '/controller/joystick_events',
            10
        )
        
        # Create publisher for connection status
        self.connection_publisher = self.create_publisher(
            ControllerStatus,
            '/controller/connection_status',
            10
        )
        
        # Timer for publishing connection status periodically
        self.connection_timer = self.create_timer(2.0, self.publish_connection_status)  # Every 2 seconds
        
        self.current_joy_msg = None
        self.controller_connected = False
        self._ever_connected = False   # distinguishes first connect from reconnect
        # Initialise far enough in the past so the status timer immediately
        # reports "disconnected" on startup rather than showing a false
        # 3-second connected window before the first /joy message arrives.
        self.last_joy_time = self.get_clock().now() - rclpy.duration.Duration(seconds=10)
        
        self.get_logger().info('Controller Node initialized')
        self.get_logger().info('Waiting for controller input on /joy topic...')
        self.get_logger().info('Make sure to run: ros2 run joy joy_node')
        
        # Publish initial connection status on startup
        self.publish_connection_status()

    def _detect_layout(self, msg):
        """Called once on the first /joy message to determine controller layout."""
        n_axes    = len(msg.axes)
        n_buttons = len(msg.buttons)

        if n_axes == _XBOX360_NUM_AXES and n_buttons == _XBOX360_NUM_BUTTONS:
            self._layout      = 'xbox360'
            self._layout_name = 'Controller 360'
            self.button_names = self._BUTTON_NAMES_XBOX360
            self.get_logger().info(
                f'Controller layout detected: 360-style (xpad) '
                f'({n_axes} axes, {n_buttons} buttons) — D-Pad via axes[6]/axes[7]')
        else:
            self._layout      = 'generic'
            self._layout_name = 'Generic Controller'
            self.button_names = self._BUTTON_NAMES_GENERIC
            self.get_logger().info(
                f'Controller layout detected: Generic '
                f'({n_axes} axes, {n_buttons} buttons) — D-Pad via buttons '
                f'{GENERIC_DPAD_UP}/{GENERIC_DPAD_DOWN}/'
                f'{GENERIC_DPAD_LEFT}/{GENERIC_DPAD_RIGHT}')
        self.get_logger().info(
            f'Button table: {self.button_names}')

    def joy_callback(self, msg):
        """
        Callback function for joy messages from the controller.
        """
        # ── Auto-detect layout on first message ───────────────────────────────
        if self._layout is None:
            self._detect_layout(msg)

        self.current_joy_msg = msg
        self.last_joy_time = self.get_clock().now()

        # ── Generic D-Pad button indices to skip in the main loop ─────────────
        # (they are handled as named D-Pad events, not raw BUTTON_N events)
        generic_dpad_indices = (
            {GENERIC_DPAD_UP, GENERIC_DPAD_DOWN, GENERIC_DPAD_LEFT, GENERIC_DPAD_RIGHT}
            if self._layout == 'generic' else set()
        )

        # Process button events (detect button presses)
        for i, button_state in enumerate(msg.buttons):
            if i in generic_dpad_indices:
                continue   # handled in publish_joystick_positions()
            button_name = self.button_names.get(i, f'BUTTON_{i}')
            prev_state = self.prev_button_states.get(i, 0)
            
            # Detect button press (transition from 0 to 1)
            if button_state == 1 and prev_state == 0:
                self.publish_button_event('PRESSED', button_name, i)
            # Detect button hold (still pressed)
            elif button_state == 1 and prev_state == 1:
                self.publish_button_event('HOLD', button_name, i)
            # Detect button release (transition from 1 to 0)
            elif button_state == 0 and prev_state == 1:
                self.publish_button_event('RELEASED', button_name, i)
            
            self.prev_button_states[i] = button_state
        
        # Publish joystick positions (Left and Right Stick)
        self.publish_joystick_positions()

    def publish_button_event(self, event_type, button_name, button_id):
        """
        Publish a button event message.
        
        Args:
            event_type (str): 'PRESSED', 'HOLD', or 'RELEASED'
            button_name (str): Human-readable button name
            button_id (int): Button index (-1 for D-Pad)
        """
        msg = ButtonEvent()
        msg.event       = event_type
        msg.button_name = button_name
        msg.button_id   = button_id
        msg.timestamp   = self.get_clock().now().to_msg().sec
        self.button_publisher.publish(msg)
        
        self.get_logger().info(f'Button {event_type}: {button_name} (ID: {button_id})')

    def publish_connection_status(self):
        """
        Publish controller connection status to ROS2 topic.
        Checks if we've received joy messages recently (within 3 seconds).
        """
        current_time = self.get_clock().now()
        time_since_last_msg = (current_time - self.last_joy_time).nanoseconds / 1e9
        
        # Consider disconnected if no message in 3 seconds
        was_connected = self.controller_connected
        self.controller_connected = time_since_last_msg < 3.0
        
        # Log status transitions
        if not was_connected and self.controller_connected:
            if not self._ever_connected:
                self.get_logger().info('Controller connected!')
                self._ever_connected = True
            else:
                self.get_logger().info('Controller reconnected!')
        elif was_connected and not self.controller_connected:
            self.get_logger().warn('Controller disconnected!')
        
        msg = ControllerStatus()
        msg.connected              = self.controller_connected
        msg.controller_type        = self._layout_name
        msg.time_since_last_input  = round(time_since_last_msg, 2)
        msg.timestamp              = current_time.to_msg().sec
        self.connection_publisher.publish(msg)
    
    def publish_joystick_positions(self):
        """
        Publish joystick positions as continuous events for joystick actions.
        Handles both Xbox 360 and generic controller layouts.
        """
        if self.current_joy_msg is None:
            return

        axes    = self.current_joy_msg.axes
        buttons = self.current_joy_msg.buttons

        if len(axes) < 6:
            return

        left_x  = round(axes[0], 3)
        left_y  = round(axes[1], 3)
        right_x = round(axes[3], 3)
        right_y = round(axes[4], 3)

        # ── Trigger normalisation ─────────────────────────────────────────────
        # Xbox 360 wireless (xpad): rest = +1.0, fully pressed = -1.0
        # Generic / Xbox One:       rest = -1.0 (or 0 before first touch),
        #                           fully pressed = +1.0
        if self._layout == 'xbox360':
            lt_val = round((1.0 - axes[2]) / 2.0, 3)   # 0.0 … 1.0
            rt_val = round((1.0 - axes[5]) / 2.0, 3)   # 0.0 … 1.0
        else:
            lt_val = round((axes[2] + 1.0) / 2.0, 3)   # 0.0 … 1.0
            rt_val = round((axes[5] + 1.0) / 2.0, 3)   # 0.0 … 1.0

        # Publish Left Stick event if not centered (deadzone 0.05)
        if abs(left_x) > 0.05 or abs(left_y) > 0.05:
            msg = JoystickEvent()
            msg.joystick_name = 'Left Stick'
            msg.x             = left_x
            msg.y             = left_y
            msg.timestamp     = self.get_clock().now().to_msg().sec
            self.joystick_publisher.publish(msg)

        # Publish Right Stick event if not centered (deadzone 0.05)
        if abs(right_x) > 0.05 or abs(right_y) > 0.05:
            msg = JoystickEvent()
            msg.joystick_name = 'Right Stick'
            msg.x             = right_x
            msg.y             = right_y
            msg.timestamp     = self.get_clock().now().to_msg().sec
            self.joystick_publisher.publish(msg)

        # Publish Left Trigger event if pressed (deadzone 0.05)
        if lt_val > 0.05:
            msg = JoystickEvent()
            msg.joystick_name = 'Left Trigger (LT)'
            msg.x             = lt_val
            msg.y             = 0.0
            msg.timestamp     = self.get_clock().now().to_msg().sec
            self.joystick_publisher.publish(msg)

        # Publish Right Trigger event if pressed (deadzone 0.05)
        if rt_val > 0.05:
            msg = JoystickEvent()
            msg.joystick_name = 'Right Trigger (RT)'
            msg.x             = rt_val
            msg.y             = 0.0
            msg.timestamp     = self.get_clock().now().to_msg().sec
            self.joystick_publisher.publish(msg)

        # ── D-Pad ─────────────────────────────────────────────────────────────
        if self._layout == 'xbox360':
            # Xbox 360 wireless: D-Pad reported as axes[6] / axes[7]
            if len(axes) >= 8:
                dpad_map = {
                    'DPadLeft':  axes[6] > 0.5,
                    'DPadRight': axes[6] < -0.5,
                    'DPadUp':    axes[7] > 0.5,
                    'DPadDown':  axes[7] < -0.5,
                }
                for btn_name, is_pressed in dpad_map.items():
                    was_pressed = self.prev_dpad_states[btn_name]
                    if is_pressed and not was_pressed:
                        self.publish_button_event('PRESSED', btn_name, -1)
                    elif is_pressed and was_pressed:
                        self.publish_button_event('HOLD', btn_name, -1)
                    elif not is_pressed and was_pressed:
                        self.publish_button_event('RELEASED', btn_name, -1)
                    self.prev_dpad_states[btn_name] = is_pressed
        else:
            # Generic: D-Pad reported as buttons
            dpad_btn_map = {
                'DPadUp':    GENERIC_DPAD_UP,
                'DPadDown':  GENERIC_DPAD_DOWN,
                'DPadLeft':  GENERIC_DPAD_LEFT,
                'DPadRight': GENERIC_DPAD_RIGHT,
            }
            for btn_name, idx in dpad_btn_map.items():
                if idx >= len(buttons):
                    continue
                is_pressed  = buttons[idx] == 1
                was_pressed = self.prev_dpad_states[btn_name]
                if is_pressed and not was_pressed:
                    self.publish_button_event('PRESSED', btn_name, idx)
                elif is_pressed and was_pressed:
                    self.publish_button_event('HOLD', btn_name, idx)
                elif not is_pressed and was_pressed:
                    self.publish_button_event('RELEASED', btn_name, idx)
                self.prev_dpad_states[btn_name] = is_pressed


def main(args=None):
    """
    Main function to initialize and run the controller node.
    """
    rclpy.init(args=args)

    try:
        controller_node = ControllerNode()
        rclpy.spin(controller_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if 'controller_node' in locals():
            controller_node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()