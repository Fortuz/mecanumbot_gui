"""
mock_database.py — A fully in-memory implementation of IDatabase for testing.

Import this in any test module that needs a database without touching SQLite.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'shared'))

from typing import Optional
from database_interface import IDatabase


class MockDatabase(IDatabase):
    """In-memory IDatabase — zero I/O, fast, hermetic.

    Internal layout
    ---------------
    _users    : {username: user_id}
    _actions  : {user_id: {name: {'id': int, 'type': str, 'tuples': list}}}
    _mappings : {user_id: {name: {'buttons': [...], 'joysticks': [...]}}}
    _schemes  : {user_id: {name: [topics]}}
    """

    def __init__(self):
        self._users    = {}   # str -> int
        self._next_uid = 1
        self._actions  = {}   # uid -> {name -> {...}}
        self._next_aid = 1
        self._mappings = {}   # uid -> {name -> {buttons, joysticks}}
        self._schemes  = {}   # uid -> {name -> [topics]}

    # ── User management ───────────────────────────────────────────────────────

    def get_or_create_user(self, username: str) -> int:
        if username not in self._users:
            self._users[username] = self._next_uid
            self._next_uid += 1
        return self._users[username]

    # ── Actions ───────────────────────────────────────────────────────────────

    def save_action(self, user_id: int, name: str, tuples: list,
                    action_type: str = 'button_once') -> bool:
        if user_id is None:
            return False
        bucket = self._actions.setdefault(user_id, {})
        aid = bucket[name]['id'] if name in bucket else self._next_aid
        if name not in bucket:
            self._next_aid += 1
        bucket[name] = {'id': aid, 'type': action_type, 'tuples': list(tuples)}
        return True

    def get_all_actions(self, user_id: int,
                        action_type: Optional[str] = None) -> dict:
        if user_id is None:
            return {}
        bucket = self._actions.get(user_id, {})
        if action_type is None:
            return dict(bucket)
        return {k: v for k, v in bucket.items() if v['type'] == action_type}

    def delete_action(self, user_id: int, name: str) -> bool:
        if user_id is None:
            return False
        bucket = self._actions.get(user_id, {})
        bucket.pop(name, None)
        return True

    # ── Mappings ──────────────────────────────────────────────────────────────

    def save_mapping(self, user_id: int, name: str,
                     button_entries: list, joystick_entries: list) -> bool:
        if user_id is None:
            return False
        bucket = self._mappings.setdefault(user_id, {})
        bucket[name] = {
            'buttons': [
                {'button': b, 'action': a, 'trigger_mode': m}
                for b, a, m in button_entries
            ],
            'joysticks': [
                {'joystick': j, 'action': a}
                for j, a in joystick_entries
            ],
        }
        return True

    def get_all_mappings(self, user_id: int) -> list:
        if user_id is None:
            return []
        bucket = self._mappings.get(user_id, {})
        return [{'name': n, **v} for n, v in bucket.items()]

    def get_mapping(self, user_id: int, name: str) -> Optional[dict]:
        if user_id is None:
            return None
        bucket = self._mappings.get(user_id, {})
        if name not in bucket:
            return None
        return {'name': name, **bucket[name]}

    def delete_mapping(self, user_id: int, name: str) -> bool:
        if user_id is None:
            return False
        bucket = self._mappings.get(user_id, {})
        bucket.pop(name, None)
        return True

    # ── Recording schemes ─────────────────────────────────────────────────────

    def save_recording_scheme(self, user_id: int, name: str,
                              topics: list) -> bool:
        if user_id is None:
            return False
        self._schemes.setdefault(user_id, {})[name] = list(topics)
        return True

    def get_all_recording_schemes(self, user_id: int) -> dict:
        if user_id is None:
            return {}
        return dict(self._schemes.get(user_id, {}))

    def delete_recording_scheme(self, user_id: int, name: str) -> bool:
        if user_id is None:
            return False
        self._schemes.get(user_id, {}).pop(name, None)
        return True
