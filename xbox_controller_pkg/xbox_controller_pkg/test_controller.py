#!/usr/bin/env python3
"""
Test script for Xbox 360 Controller Node

This script listens to the controller events and prints them to the console.
Use this to test if the xbox_controller_node is working correctly.

Usage:
    ros2 run xbox_controller_pkg test_controller

Author: Generated for mecanumbot project
Date: November 9, 2025
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class ControllerTestNode(Node):
    """Test node to monitor controller events."""
    
    def __init__(self):
        super().__init__('controller_test_node')
        
        # Subscribe to button events
        self.button_subscription = self.create_subscription(
            String,
            '/xbox_controller/button_events',
            self.button_event_callback,
            10
        )
        
        # Subscribe to controller state
        self.state_subscription = self.create_subscription(
            String,
            '/xbox_controller/state', 
            self.state_callback,
            10
        )
        
        self.get_logger().info('Controller Test Node started')
        self.get_logger().info('Listening for controller events...')
        print("\n" + "="*60)
        print("Xbox 360 Controller Test - Press buttons to see events")
        print("="*60)

    def button_event_callback(self, msg):
        """Handle button and axis events."""
        try:
            data = json.loads(msg.data)
            
            if data['type'] == 'BUTTON':
                if data['event'] == 'PRESSED':
                    print(f"🎮 BUTTON PRESSED:  {data['button_name']} (ID: {data['button_id']})")
                elif data['event'] == 'RELEASED':
                    print(f"🎮 BUTTON RELEASED: {data['button_name']} (ID: {data['button_id']})")
                    
            elif data['type'] == 'AXIS':
                axis_name = data['axis_name']
                value = data['value']
                
                # Format axis values nicely
                if 'TRIGGER' in axis_name:
                    # Triggers go from -1 (not pressed) to 1 (fully pressed)
                    trigger_percent = int(((value + 1) / 2) * 100)
                    print(f"🎯 {axis_name}: {trigger_percent}% ({value:.3f})")
                elif 'STICK' in axis_name:
                    # Sticks go from -1 to 1
                    print(f"🕹️  {axis_name}: {value:.3f}")
                elif 'DPAD' in axis_name:
                    # D-pad values
                    if axis_name == 'DPAD_X':
                        direction = 'LEFT' if value < 0 else 'RIGHT' if value > 0 else 'CENTER'
                        print(f"⬅️➡️ D-PAD X: {direction} ({value:.3f})")
                    elif axis_name == 'DPAD_Y':
                        direction = 'UP' if value > 0 else 'DOWN' if value < 0 else 'CENTER'  
                        print(f"⬆️⬇️ D-PAD Y: {direction} ({value:.3f})")
                        
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to parse JSON: {msg.data}")

    def state_callback(self, msg):
        """Handle periodic state updates (optional, can be used for debugging)."""
        # Uncomment the following lines if you want to see periodic state updates
        # try:
        #     data = json.loads(msg.data)
        #     if data['type'] == 'STATE':
        #         pressed_buttons = [name for name, pressed in data['buttons'].items() if pressed]
        #         if pressed_buttons:
        #             print(f"📊 Currently pressed: {', '.join(pressed_buttons)}")
        # except json.JSONDecodeError:
        #     pass
        pass


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    try:
        test_node = ControllerTestNode()
        print("\nPress Ctrl+C to stop the test\n")
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        print("\n" + "=" * 60)
        print("Controller test stopped")
        print("=" * 60)
    finally:
        if 'test_node' in locals():
            test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()