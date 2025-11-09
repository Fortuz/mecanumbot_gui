#!/usr/bin/env python3

import pytest
import rclpy
from xbox_controller_pkg.xbox_controller_node import Xbox360ControllerNode


def test_node_initialization():
    """Test that the Xbox controller node initializes correctly."""
    rclpy.init()
    try:
        node = Xbox360ControllerNode()
        assert node.get_name() == 'xbox360_controller_node'
        assert hasattr(node, 'button_names')
        assert hasattr(node, 'axis_names')
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_button_mapping():
    """Test button name mapping."""
    rclpy.init()
    try:
        node = Xbox360ControllerNode()
        
        # Test known button mappings
        assert node.button_names[0] == 'A'
        assert node.button_names[1] == 'B'
        assert node.button_names[2] == 'X'
        assert node.button_names[3] == 'Y'
        
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_axis_mapping():
    """Test axis name mapping.""" 
    rclpy.init()
    try:
        node = Xbox360ControllerNode()
        
        # Test known axis mappings
        assert node.axis_names[0] == 'LEFT_STICK_X'
        assert node.axis_names[1] == 'LEFT_STICK_Y'
        assert node.axis_names[2] == 'LEFT_TRIGGER'
        
        node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    pytest.main([__file__])