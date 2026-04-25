"""
conftest.py — pytest configuration for the mecanumbot_gui test suite.

Adds the shared/ and host/ directories to sys.path so that
  import database
  import database_interface
  import app
  import docker_node
all resolve correctly from any working directory.
"""
import sys
import os

_ROOT   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_HOST   = os.path.join(_ROOT, 'host')
_SHARED = os.path.join(_ROOT, 'shared')

for _p in (_SHARED, _HOST):
    if _p not in sys.path:
        sys.path.insert(0, _p)
