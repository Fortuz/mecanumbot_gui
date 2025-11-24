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
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import json
import os


class Xbox360ControllerNode(Node):
    """
    ROS 2 Node that processes Xbox 360 controller input and publishes button events.
    """
    
    def __init__(self):
        super().__init__('xbox360_controller_node')
        
        # Xbox 360 controller button mapping (standard layout)
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
        
        # Axis mapping for triggers and sticks
        self.axis_names = {
            0: 'LEFT_STICK_X',
            1: 'LEFT_STICK_Y', 
            2: 'LEFT_TRIGGER',
            3: 'RIGHT_STICK_X',
            4: 'RIGHT_STICK_Y',
            5: 'RIGHT_TRIGGER',
            6: 'DPAD_X',
            7: 'DPAD_Y'
        }
        
        # Store previous button states to detect press events
        self.prev_button_states = {}
        self.prev_axis_states = {}
        
        # Create subscription to joy topic
        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Create publisher for button events
        self.button_publisher = self.create_publisher(
            String,
            '/xbox_controller/button_events',
            10
        )
        
        # Create publisher for joystick events
        self.joystick_publisher = self.create_publisher(
            String,
            '/xbox_controller/joystick_events',
            10
        )
        
        # Create publisher for connection status
        self.connection_publisher = self.create_publisher(
            String,
            '/xbox_controller/connection_status',
            10
        )
        
        # Timer for publishing joystick positions periodically
        self.joystick_timer = self.create_timer(0.1, self.publish_joystick_positions)  # 10Hz
        
        # Timer for publishing connection status periodically
        self.connection_timer = self.create_timer(2.0, self.publish_connection_status)  # Every 2 seconds
        
        self.current_joy_msg = None
        self.controller_connected = False
        self.last_joy_time = self.get_clock().now()
        
        # Path for status file (shared with Docker)
        self.status_file = os.path.expanduser("~/Documents/controller_status.json")
        
        self.get_logger().info('Xbox 360 Controller Node initialized')
        self.get_logger().info('Waiting for controller input on /joy topic...')
        self.get_logger().info('Make sure to run: ros2 run joy joy_node')

    def joy_callback(self, msg):
        """
        Callback function for joy messages from the controller.
        
        Args:
            msg (Joy): Joy message containing button and axis states
        """
        self.current_joy_msg = msg
        self.last_joy_time = self.get_clock().now()
        
        # Mark controller as connected if we receive data
        if not self.controller_connected:
            self.controller_connected = True
            self.get_logger().info('Controller connected!')
        
        # Process button events (detect button presses)
        for i, button_state in enumerate(msg.buttons):
            button_name = self.button_names.get(i, f'BUTTON_{i}')
            prev_state = self.prev_button_states.get(i, 0)
            
            # Detect button press (transition from 0 to 1)
            if button_state == 1 and prev_state == 0:
                self.publish_button_event('PRESSED', button_name, i)
            # Detect button release (transition from 1 to 0)
            elif button_state == 0 and prev_state == 1:
                self.publish_button_event('RELEASED', button_name, i)
            
            self.prev_button_states[i] = button_state
        
        # Process axis events (detect significant changes)
        for i, axis_value in enumerate(msg.axes):
            axis_name = self.axis_names.get(i, f'AXIS_{i}')
            prev_value = self.prev_axis_states.get(i, 0.0)
            
            # Detect significant axis change (threshold to avoid noise)
            if abs(axis_value - prev_value) > 0.1:
                self.publish_axis_event(axis_name, i, axis_value)
            
            self.prev_axis_states[i] = axis_value

    def publish_button_event(self, event_type, button_name, button_id):
        """
        Publish a button event message.
        
        Args:
            event_type (str): 'PRESSED' or 'RELEASED'
            button_name (str): Human-readable button name
            button_id (int): Button index
        """
        event_data = {
            'type': 'BUTTON',
            'event': event_type,
            'button_name': button_name,
            'button_id': button_id,
            'timestamp': self.get_clock().now().to_msg()._sec
        }
        
        msg = String()
        msg.data = json.dumps(event_data)
        self.button_publisher.publish(msg)
        
        self.get_logger().info(f'Button {event_type}: {button_name} (ID: {button_id})')

    def publish_axis_event(self, axis_name, axis_id, value):
        """
        Publish an axis event message.
        
        Args:
            axis_name (str): Human-readable axis name
            axis_id (int): Axis index
            value (float): Axis value
        """
        event_data = {
            'type': 'AXIS',
            'event': 'CHANGED',
            'axis_name': axis_name,
            'axis_id': axis_id,
            'value': round(value, 3),
            'timestamp': self.get_clock().now().to_msg()._sec
        }

        self.get_logger().info(f'Axis CHANGED: {axis_name} (ID: {axis_id})')

        msg = String()
        msg.data = json.dumps(event_data)

        self.joystick_publisher.publish(msg)
    def publish_connection_status(self):
        """
        Publish controller connection status.
        Checks if we've received joy messages recently (within 3 seconds).
        Also writes status to a file for Docker app to read.
        """
        current_time = self.get_clock().now()
        time_since_last_msg = (current_time - self.last_joy_time).nanoseconds / 1e9
        
        # Consider disconnected if no message in 3 seconds
        was_connected = self.controller_connected
        self.controller_connected = time_since_last_msg < 3.0
        
        # Log status changes
        if was_connected and not self.controller_connected:
            self.get_logger().warn('Controller disconnected!')
        elif not was_connected and self.controller_connected:
            self.get_logger().info('Controller reconnected!')
        
        status_data = {
            'connected': self.controller_connected,
            'controller_type': 'Xbox 360',
            'time_since_last_input': round(time_since_last_msg, 2),
            'timestamp': current_time.to_msg()._sec
        }
        
        # Publish to ROS2 topic
        msg = String()
        msg.data = json.dumps(status_data)
        self.connection_publisher.publish(msg)
        
        # Write to file for Docker app
        try:
            with open(self.status_file, 'w') as f:
                json.dump(status_data, f)
        except Exception as e:
            self.get_logger().error(f'Failed to write status file: {e}')
    
    def publish_joystick_positions(self):
        """
        Publish joystick positions as continuous events for joystick actions.
        """
        if self.current_joy_msg is None:
            return
        
        axes = self.current_joy_msg.axes
        
        # Check if we have enough axes for both sticks
        if len(axes) < 5:
            return
        
        left_x = round(axes[0], 3)
        left_y = round(axes[1], 3)
        right_x = round(axes[3], 3)
        right_y = round(axes[4], 3)
        
        # Publish Left Stick event if not centered (deadzone 0.05)
        if abs(left_x) > 0.05 or abs(left_y) > 0.05:
            left_event = {
                'type': 'JOYSTICK',
                'joystick_name': 'Left Stick',
                'x': left_x,
                'y': left_y,
                'timestamp': self.get_clock().now().to_msg()._sec
            }
            msg = String()
            msg.data = json.dumps(left_event)
            self.joystick_publisher.publish(msg)
        
        # Publish Right Stick event if not centered (deadzone 0.05)
        if abs(right_x) > 0.05 or abs(right_y) > 0.05:
            right_event = {
                'type': 'JOYSTICK',
                'joystick_name': 'Right Stick',
                'x': right_x,
                'y': right_y,
                'timestamp': self.get_clock().now().to_msg()._sec
            }
            msg = String()
            msg.data = json.dumps(right_event)
            self.joystick_publisher.publish(msg)


def main(args=None):
    """
    Main function to initialize and run the Xbox 360 controller node.
    """
    rclpy.init(args=args)
    
    try:
        xbox_controller_node = Xbox360ControllerNode()
        rclpy.spin(xbox_controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'xbox_controller_node' in locals():
            xbox_controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()