#!/usr/bin/env python3
"""
Launch file for Xbox 360 Controller Node

This launch file starts both the joy_node (for controller input) 
and the xbox_controller_node (for processing and publishing events).

Usage:
    ros2 launch xbox_controller_pkg xbox_controller_launch.py

Author: Generated for mecanumbot project  
Date: November 9, 2025
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generate launch description for Xbox 360 controller setup.
    """
    
    # Declare launch arguments
    joy_device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/input/js0',
        description='Path to the joystick device'
    )
    
    joy_autorepeat_rate_arg = DeclareLaunchArgument(
        'autorepeat_rate',
        default_value='20.0',
        description='Rate at which to republish the last command (Hz)'
    )
    
    # Joy node for reading controller input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device': LaunchConfiguration('device'),
            'autorepeat_rate': LaunchConfiguration('autorepeat_rate'),
            'coalesce_interval': 0.01
        }],
        output='screen'
    )
    
    # Xbox controller processing node
    xbox_controller_node = Node(
        package='xbox_controller_pkg',
        executable='xbox_controller_node',
        name='xbox_controller_node',
        output='screen',
        parameters=[],
        remappings=[]
    )
    
    return LaunchDescription([
        joy_device_arg,
        joy_autorepeat_rate_arg,
        joy_node,
        xbox_controller_node
    ])