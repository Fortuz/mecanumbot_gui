"""
conftest.py — pytest configuration for the mecanumbot_gui test suite.

Adds the shared/, host/, and robot package directories to sys.path so that
  import database
  import database_interface
  import app
  import docker_node
  import mapping_listener
  import controller_node
all resolve correctly from any working directory.
"""
import sys
import os

_ROOT   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_HOST   = os.path.join(_ROOT, 'host')
_SHARED = os.path.join(_ROOT, 'shared')
_BUTTON_MAPPING_ROS = os.path.join(_ROOT, 'robot', 'button_mapping_ros')
_CONTROLLER_PKG = os.path.join(_ROOT, 'robot', 'controller_pkg', 'controller_pkg')

for _p in (_SHARED, _HOST, _BUTTON_MAPPING_ROS, _CONTROLLER_PKG):
    if _p not in sys.path:
        sys.path.insert(0, _p)
