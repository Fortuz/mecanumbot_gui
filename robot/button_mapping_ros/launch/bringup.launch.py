from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Must match ROS_DOMAIN_ID on the host side
        SetEnvironmentVariable('ROS_DOMAIN_ID',      '19'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        # Use CycloneDDS to avoid FastRTPS CDR deserialization bugs with
        # multiple concurrent service servers/clients on the same node.
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),

        # joy driver — reads /dev/input/js* and publishes sensor_msgs/msg/Joy
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),

        # Translates raw Joy messages → controller-typed ROS2 topics
        Node(
            package='xbox_controller_pkg',
            executable='controller_node',
            name='controller_node',
            output='screen',
        ),

        # Listens to controller topics and executes button/joystick mappings
        Node(
            package='button_mapping_ros',
            executable='mapping_listener',
            name='mapping_listener',
            output='screen',
        ),
    ])
