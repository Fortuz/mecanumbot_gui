#!/usr/bin/env python3
"""
Button Mapping Listener Node

This ROS2 node:
1. Subscribes to mapping configuration topics from the GUI
2. Subscribes to Xbox controller events
3. Maps controller inputs to configured actions and publishes them
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sqlite3
import os


class MappingListener(Node):
    """
    ROS2 Node that listens for button and joystick mapping updates,
    subscribes to controller events, and publishes mapped actions.
    """
    
    def __init__(self):
        super().__init__('mapping_listener')
        
        # Database path
        self.db_path = os.path.expanduser("~/Documents/actions.db")
        
        # Dictionaries to store mappings
        self.button_mappings = {}  # button_name -> action_name
        self.joystick_mappings = {}  # joystick_name -> action_name
        self.actions = {}  # action_name -> {type, tuples}
        
        # Load existing mappings from database on startup
        self.load_mappings_from_database()
        
        # Create subscribers for Xbox controller events
        self.xbox_button_sub = self.create_subscription(
            String,
            '/xbox_controller/button_events',
            self.xbox_button_callback,
            10
        )
        
        self.xbox_joystick_sub = self.create_subscription(
            String,
            '/xbox_controller/joystick_events',
            self.xbox_joystick_callback,
            10
        )
        
        # Create publishers for mapped actions (will be dynamically created)
        self.action_publishers = {}  # topic -> publisher
        
        # Timer to periodically publish current state (every 5 seconds)
        self.create_timer(5.0, self.publish_state)
        
        # Timer to reload mappings from database (every 10 seconds)
        self.create_timer(5.0, self.load_mappings_from_database)
        
        self.get_logger().info('Mapping Listener Node started')
        self.get_logger().info('Subscribed to controller event topics:')
        self.get_logger().info('  - /xbox_controller/button_events')
        self.get_logger().info('  - /xbox_controller/joystick_events')
        self.get_logger().info('Mappings auto-reload from database every 10 seconds')
        
    def load_mappings_from_database(self):
        """
        Load existing button mappings, joystick mappings, and actions from database on startup.
        This ensures the node starts with the current configuration.
        """
        if not os.path.exists(self.db_path):
            self.get_logger().warn(f'Database not found at {self.db_path}, starting with empty mappings')
            return
        
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            # Load actions first (schema: id, name, action_type, created_utc)
            cursor.execute('SELECT id, name, action_type FROM actions')
            actions_rows = cursor.fetchall()
            for action_id, action_name, action_type in actions_rows:
                # Load tuples (topic-message pairs) for this action from action_tuples table
                cursor.execute('''
                    SELECT topic, message 
                    FROM action_tuples 
                    WHERE action_id = ? 
                    ORDER BY tuple_order
                ''', (action_id,))
                tuples = cursor.fetchall()  # Returns list of (topic, message) tuples
                
                self.actions[action_name] = {
                    'type': action_type,
                    'tuples': tuples
                }
            
            # Load button mappings (schema: id, button_name, action_id, created_utc)
            # Need to join with actions table to get action name
            cursor.execute('''
                SELECT bm.button_name, a.name 
                FROM button_mappings bm 
                JOIN actions a ON bm.action_id = a.id
            ''')
            button_rows = cursor.fetchall()
            for button_name, action_name in button_rows:
                self.button_mappings[button_name] = action_name
            
            # Load joystick mappings (schema: id, joystick_name, action_id, created_utc)
            cursor.execute('''
                SELECT jm.joystick_name, a.name 
                FROM joystick_mappings jm 
                JOIN actions a ON jm.action_id = a.id
            ''')
            joystick_rows = cursor.fetchall()
            for joystick_name, action_name in joystick_rows:
                self.joystick_mappings[joystick_name] = action_name
            
            conn.close()
            
            self.get_logger().info(f'Loaded from database: {len(self.actions)} actions, '
                                  f'{len(self.button_mappings)} button mappings, '
                                  f'{len(self.joystick_mappings)} joystick mappings')
            
        except sqlite3.Error as e:
            self.get_logger().error(f'Database error while loading mappings: {e}')
        except Exception as e:
            self.get_logger().error(f'Error loading mappings from database: {e}')
    
    def xbox_button_callback(self, msg):
        """
        Callback for Xbox controller button events.
        Looks up the button in mappings and publishes the mapped action.
        """
        try:
            data = json.loads(msg.data)
            
            # Only process button press events (not releases)
            if data.get('event') != 'PRESSED':
                return
            
            button_name = data.get('button_name')
            
            # Check if this button has a mapping
            if button_name not in self.button_mappings:
                self.get_logger().debug(f'Button {button_name} pressed but not mapped')
                return
            
            action_name = self.button_mappings[button_name]
            
            # Check if the action exists
            if action_name not in self.actions:
                self.get_logger().warn(f'Action {action_name} not configured')
                return
            
            # Publish the action
            self.publish_action(action_name, button_name)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse button event: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing button event: {e}')
    
    def xbox_joystick_callback(self, msg):
        """
        Callback for Xbox controller joystick events.
        Looks up the joystick in mappings and publishes the mapped action with X, Y values.
        """
        try:
            data = json.loads(msg.data)
            joystick_name = data.get('joystick_name')
            x_value = data.get('x', 0.0)
            y_value = data.get('y', 0.0)
            
            # Check if this joystick has a mapping
            if joystick_name not in self.joystick_mappings:
                return
            
            action_name = self.joystick_mappings[joystick_name]
            
            # Check if the action exists
            if action_name not in self.actions:
                self.get_logger().warn(f'Action {action_name} not configured')
                return
            
            # Publish the action with X, Y substitution
            self.publish_joystick_action(action_name, joystick_name, x_value, y_value)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse joystick event: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing joystick event: {e}')
    
    def publish_action(self, action_name, trigger_control):
        """
        Publish all topic-message pairs for a button action.
        
        Args:
            action_name: Name of the action to publish
            trigger_control: Name of the button/control that triggered it
        """
        action_data = self.actions[action_name]
        tuples = action_data.get('tuples', [])
        
        self.get_logger().info(f'[DEBUG] Would publish action "{action_name}" (triggered by {trigger_control})')
        
        for topic, message in tuples:
            # COMMENTED OUT FOR TESTING - Print instead of publish
            # Get or create publisher for this topic
            # if topic not in self.action_publishers:
            #     self.action_publishers[topic] = self.create_publisher(String, topic, 10)
            #     self.get_logger().info(f'Created publisher for topic: {topic}')
            
            # Publish the message
            # msg = String()
            # msg.data = message
            # self.action_publishers[topic].publish(msg)
            
            # DEBUG: Print what would be published
            self.get_logger().info(f'  [DEBUG] Would publish to topic "{topic}": {message}')
    
    def publish_joystick_action(self, action_name, joystick_name, x_value, y_value):
        """
        Publish all topic-message pairs for a joystick action with X, Y substitution.
        
        Args:
            action_name: Name of the action to publish
            joystick_name: Name of the joystick that triggered it
            x_value: X-axis value (-1.0 to 1.0)
            y_value: Y-axis value (-1.0 to 1.0)
        """
        action_data = self.actions[action_name]
        tuples = action_data.get('tuples', [])
        
        self.get_logger().info(f'[DEBUG] Would publish joystick action "{action_name}" (triggered by {joystick_name}, X={x_value:.2f}, Y={y_value:.2f})')
        
        for topic, message_template in tuples:
            # COMMENTED OUT FOR TESTING - Print instead of publish
            # Get or create publisher for this topic
            # if topic not in self.action_publishers:
            #     self.action_publishers[topic] = self.create_publisher(String, topic, 10)
            #     self.get_logger().info(f'Created publisher for topic: {topic}')
            
            # Replace X and Y placeholders in message
            message = message_template.replace('X', str(x_value)).replace('Y', str(y_value))
            
            # Publish the message
            # msg = String()
            # msg.data = message
            # self.action_publishers[topic].publish(msg)
            
            # DEBUG: Print what would be published
            self.get_logger().info(f'  [DEBUG] Would publish to topic "{topic}": {message}')
    
    def publish_state(self):
        """Periodically log current mapping state for debugging"""
        self.get_logger().info(
            f'State: {len(self.button_mappings)} button mappings, '
            f'{len(self.joystick_mappings)} joystick mappings, '
            f'{len(self.actions)} actions',
            throttle_duration_sec=30.0  # Only log every 30 seconds
        )


def main(args=None):
    rclpy.init(args=args)
    
    mapping_listener = MappingListener()
    
    try:
        rclpy.spin(mapping_listener)
    except KeyboardInterrupt:
        pass
    finally:
        mapping_listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
