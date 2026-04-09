"""
mock_database.py — A fully in-memory implementation of IDatabase for testing.

Import this in any test module that needs a database without touching SQLite.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'shared'))

from database_interface import IDatabase


class MockDatabase(IDatabase):
    """In-memory IDatabase — zero I/O, fast, hermetic."""

    def __init__(self, actions=None, button_mappings=None, joystick_mappings=None):
        self._actions         = dict(actions         or {})
        self._button_mappings = dict(button_mappings or {})
        self._joy_mappings    = dict(joystick_mappings or {})

    # ── Actions ───────────────────────────────────────────────────────────────

    def save_action(self, name: str, tuples: list, action_type: str = 'button_once') -> bool:
        self._actions[name] = {'type': action_type, 'tuples': list(tuples)}
        return True

    def get_all_actions(self, action_type: str = None) -> dict:
        if action_type is None:
            return dict(self._actions)
        return {k: v for k, v in self._actions.items() if v['type'] == action_type}

    def delete_action(self, name: str) -> bool:
        self._actions.pop(name, None)
        # cascade
        to_del_btn = [b for b, v in self._button_mappings.items() if v.get('action') == name]
        for b in to_del_btn:
            del self._button_mappings[b]
        to_del_joy = [j for j, a in self._joy_mappings.items() if a == name]
        for j in to_del_joy:
            del self._joy_mappings[j]
        return True

    # ── Button Mappings ───────────────────────────────────────────────────────

    def save_button_mapping(self, button_name: str, action_name: str,
                             trigger_mode: str = 'once') -> bool:
        if action_name not in self._actions:
            return False
        self._button_mappings[button_name] = {
            'action': action_name, 'trigger_mode': trigger_mode
        }
        return True

    def get_all_button_mappings(self) -> dict:
        return dict(self._button_mappings)

    def delete_button_mapping(self, button_name: str) -> bool:
        self._button_mappings.pop(button_name, None)
        return True

    # ── Joystick Mappings ─────────────────────────────────────────────────────

    def save_joystick_mapping(self, joystick_name: str, action_name: str) -> bool:
        if action_name not in self._actions:
            return False
        self._joy_mappings[joystick_name] = action_name
        return True

    def get_all_joystick_mappings(self) -> dict:
        return dict(self._joy_mappings)

    def delete_joystick_mapping(self, joystick_name: str) -> bool:
        self._joy_mappings.pop(joystick_name, None)
        return True
