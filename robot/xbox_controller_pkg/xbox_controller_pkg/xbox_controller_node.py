#!/usr/bin/env python3
"""
Xbox 360 Controller Button Publisher Node

This ROS 2 node listens to Xbox 360 controller input via the joy package
and publishes button press events to a custom topic.

Requirements:
- ros-humble-joy package
- Xbox 360 controller connected via wireless adapter

"""

import rclpy
import rclpy.duration
from rclpy.node import Node
from sensor_msgs.msg import Joy
from mecanumbot_msgs.msg import ButtonEvent, JoystickEvent, ControllerStatus


class Xbox360ControllerNode(Node):
    """
    ROS 2 Node that processes Xbox 360 controller input and publishes button events.
    """
    
    def __init__(self):
        super().__init__('xbox360_controller_node')
        
        # Xbox 360 controller button mapping (standard layout)
        # NOTE: The D-Pad is reported as axes (not buttons) by the ROS2 joy
        # node for the Xbox 360 wireless adapter.  It is handled separately
        # in publish_joystick_positions() via axes[6] / axes[7].
        self.button_names = {
            0: 'A',
            1: 'B',
            2: 'X',
            3: 'Y',
            4: 'LB',  # Left Bumper
            5: 'RB',  # Right Bumper
            6: 'BACK',
            7: 'START',
            8: 'XBOX',  # Xbox button (center)
            9: 'LS',    # Left Stick button
            10: 'RS'    # Right Stick button
        }

        # Track previous D-Pad axis states to emit PRESSED / RELEASED events
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
            '/xbox_controller/button_events',
            10
        )
        
        # Create publisher for joystick events
        self.joystick_publisher = self.create_publisher(
            JoystickEvent,
            '/xbox_controller/joystick_events',
            10
        )
        
        # Create publisher for connection status
        self.connection_publisher = self.create_publisher(
            ControllerStatus,
            '/xbox_controller/connection_status',
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
        
        self.get_logger().info('Xbox 360 Controller Node initialized')
        self.get_logger().info('Waiting for controller input on /joy topic...')
        self.get_logger().info('Make sure to run: ros2 run joy joy_node')
        
        # Publish initial connection status on startup
        self.publish_connection_status()

    def joy_callback(self, msg):
        """
        Callback function for joy messages from the controller.
        
        Args:
            msg (Joy): Joy message containing button and axis states
        """
        self.current_joy_msg = msg
        self.last_joy_time = self.get_clock().now()

        # Process button events (detect button presses)
        for i, button_state in enumerate(msg.buttons):
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
        msg.controller_type        = 'Xbox 360'
        msg.time_since_last_input  = round(time_since_last_msg, 2)
        msg.timestamp              = current_time.to_msg().sec
        self.connection_publisher.publish(msg)
    
    def publish_joystick_positions(self):
        """
        Publish joystick positions as continuous events for joystick actions.
        """
        if self.current_joy_msg is None:
            return
        
        axes = self.current_joy_msg.axes
        
        # Check if we have enough axes for both sticks and triggers (indices 0-5)
        if len(axes) < 6:
            return
        
        left_x = round(axes[0], 3)
        left_y = round(axes[1], 3)
        right_x = round(axes[3], 3)
        right_y = round(axes[4], 3)

        # Triggers: axes[2] = LT, axes[5] = RT
        # At rest = +1.0; fully pressed = -1.0.
        # Normalise to 0.0 (rest) … 1.0 (fully pressed) and publish as X-only.
        lt_raw = axes[2]
        rt_raw = axes[5]
        lt_val = round((1.0 - lt_raw) / 2.0, 3)   # 0.0 … 1.0
        rt_val = round((1.0 - rt_raw) / 2.0, 3)   # 0.0 … 1.0
        
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

        # D-Pad: reported as axes[6] (left=-1/right=+1) and axes[7] (down=-1/up=+1)
        # Convert to PRESSED / RELEASED button events so button mappings work.
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


def main(args=None):
    """
    Main function to initialize and run the Xbox 360 controller node.
    """
    rclpy.init(args=args)

    try:
        xbox_controller_node = Xbox360ControllerNode()
        rclpy.spin(xbox_controller_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if 'xbox_controller_node' in locals():
            xbox_controller_node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()