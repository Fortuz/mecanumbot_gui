"""
conftest.py — shared pytest fixtures for all test modules.
"""

import os
import sys
import pytest

# ── Add source directories to sys.path so imports work without installation ──
ROOT = os.path.dirname(os.path.dirname(__file__))   # mecanumbot_gui-button_mapping/
sys.path.insert(0, os.path.join(ROOT, 'shared'))
sys.path.insert(0, os.path.join(ROOT, 'host'))
sys.path.insert(0, os.path.join(ROOT, 'robot'))

# ── Shared DB fixture ────────────────────────────────────────────────────────

@pytest.fixture
def tmp_db_path(tmp_path):
    """Return a path to a temp directory for a fresh SQLite database."""
    return str(tmp_path / 'test.db')
