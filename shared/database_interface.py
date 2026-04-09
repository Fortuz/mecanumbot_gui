"""
database_interface.py — Abstract base class for the MecanumBot actions database.

Both the real SQLite-backed Database and any test MockDatabase must implement
this interface.  MappingListener (robot side) and DockerNode (host side) depend
only on IDatabase, never on the concrete class.

Return-type conventions (same as Database):
  get_all_actions()         → dict[str, {"type": str, "tuples": list[tuple]}]
  get_all_button_mappings() → dict[str, {"action": str, "trigger_mode": str}]
  get_all_joystick_mappings() → dict[str, str]   # joystick_name → action_name
  save_*  / delete_*        → bool  (True on success)
"""

from abc import ABC, abstractmethod


class IDatabase(ABC):

    # ── Actions ───────────────────────────────────────────────────────────────

    @abstractmethod
    def save_action(self, name: str, tuples: list, action_type: str = "button_once") -> bool:
        """Insert or overwrite an action and its publish steps."""

    @abstractmethod
    def get_all_actions(self, action_type: str = None) -> dict:
        """
        Return all actions, optionally filtered by action_type.

        Shape:
          {
            "action_name": {
              "type":   str,
              "tuples": [(topic, message, message_type, scale_x, scale_y,
                          offset_x, offset_y, publish_type, service_name), ...]
            },
            ...
          }
        """

    @abstractmethod
    def delete_action(self, name: str) -> bool:
        """Delete an action and its tuples (cascades to mappings)."""

    # ── Button mappings ───────────────────────────────────────────────────────

    @abstractmethod
    def save_button_mapping(self, button_name: str, action_name: str,
                            trigger_mode: str = "once") -> bool:
        """Upsert a button → action mapping."""

    @abstractmethod
    def get_all_button_mappings(self) -> dict:
        """
        Return all button mappings.

        Shape:
          {
            "button_name": {"action": str, "trigger_mode": str},
            ...
          }
        """

    @abstractmethod
    def delete_button_mapping(self, button_name: str) -> bool:
        """Delete a button mapping."""

    # ── Joystick mappings ─────────────────────────────────────────────────────

    @abstractmethod
    def save_joystick_mapping(self, joystick_name: str, action_name: str) -> bool:
        """Upsert a joystick → action mapping."""

    @abstractmethod
    def get_all_joystick_mappings(self) -> dict:
        """
        Return all joystick mappings.

        Shape:
          {"joystick_name": "action_name", ...}
        """

    @abstractmethod
    def delete_joystick_mapping(self, joystick_name: str) -> bool:
        """Delete a joystick mapping."""
