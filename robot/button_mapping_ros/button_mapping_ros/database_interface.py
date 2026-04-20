"""
IDatabase  —  Abstract interface for the MecanumBot actions/mappings database.

Every concrete backend (SQLite, in-memory mock, …) must subclass this and
implement all abstract methods.

Schema overview
---------------
  users              — registered human users (host-side concept)
  actions            — publish-step sequences, scoped per user
  action_tuples      — individual publish steps belonging to an action
  mappings           — named controller configurations, scoped per user
  mapping_buttons    — button→action assignments belonging to a mapping
  mapping_joysticks  — joystick→action assignments belonging to a mapping
  recording_schemes  — named sets of topics to record, scoped per user
  recording_scheme_topics — individual topics belonging to a scheme

Return shapes are documented on each method.
"""

from abc import ABC, abstractmethod
from typing import Optional

# ── Action type constants (stored verbatim in the DB) ─────────────────────────
ACTION_BUTTON_ONCE = 'button_once'
ACTION_BUTTON_HOLD = 'button_hold'
ACTION_JOYSTICK    = 'joystick'


class IDatabase(ABC):

    # ── User management ───────────────────────────────────────────────────────

    @abstractmethod
    def get_or_create_user(self, username: str) -> int:
        """
        Return the integer id for *username*, creating the user row if it does
        not exist yet.

        Args:
            username: human-readable name supplied on the landing page
        Returns:
            user_id (int)
        """

    # ── Actions ───────────────────────────────────────────────────────────────

    @abstractmethod
    def save_action(self, user_id: int, name: str, tuples: list,
                    action_type: str = ACTION_BUTTON_ONCE) -> bool:
        """
        Insert or update an action and its publish steps.

        Args:
            user_id:     owner of this action
            name:        display name, e.g. "Move Forward"
            tuples:      list of tuples; each tuple contains positional
                         fields in this order:
                           (topic, message, message_type,
                            scale_x, scale_y, offset_x, offset_y,
                            publish_type, service_name)
                         All fields beyond `message` are optional and fall
                         back to their defaults.
            action_type: ACTION_BUTTON_ONCE | ACTION_BUTTON_HOLD | ACTION_JOYSTICK
        Returns:
            True on success, False on error
        """

    @abstractmethod
    def get_all_actions(self, user_id: int,
                        action_type: Optional[str] = None) -> dict:
        """
        Return all actions for *user_id* as a dict, optionally filtered.

        Returns:
            {
              "Move Forward": {
                  "id": 1,
                  "type": "joystick",
                  "tuples": [
                      (topic, message, message_type,
                       scale_x, scale_y, offset_x, offset_y,
                       publish_type, service_name),
                      ...
                  ]
              },
              ...
            }
        """

    @abstractmethod
    def delete_action(self, user_id: int, name: str) -> bool:
        """
        Delete an action by name for *user_id*.
        Cascades to action_tuples automatically.
        Returns True on success, False on error.
        """

    # ── Mappings ──────────────────────────────────────────────────────────────

    @abstractmethod
    def save_mapping(self, user_id: int, name: str,
                     button_entries: list,
                     joystick_entries: list) -> bool:
        """
        Insert or replace a named controller mapping.

        Args:
            user_id:          owner of this mapping
            name:             display name, e.g. "Drive Config"
            button_entries:   list of (button_name, action_name, trigger_mode)
            joystick_entries: list of (joystick_name, action_name)
        Returns:
            True on success, False on error
        """

    @abstractmethod
    def get_all_mappings(self, user_id: int) -> list:
        """
        Return all mappings for *user_id*.

        Returns:
            [
              {
                "name": "Drive Config",
                "buttons":   [{"button":   "A",          "action": "Honk",  "trigger_mode": "once"}, ...],
                "joysticks": [{"joystick": "Left Stick",  "action": "Drive"}, ...]
              },
              ...
            ]
        """

    @abstractmethod
    def get_mapping(self, user_id: int, name: str) -> Optional[dict]:
        """
        Return one named mapping for *user_id*, or None if not found.
        Same shape as a single element of get_all_mappings().
        """

    @abstractmethod
    def delete_mapping(self, user_id: int, name: str) -> bool:
        """
        Delete a named mapping for *user_id*.
        Cascades to mapping_buttons and mapping_joysticks automatically.
        Returns True on success, False on error.
        """

    # ── Recording schemes ─────────────────────────────────────────────────────

    @abstractmethod
    def save_recording_scheme(self, user_id: int, name: str,
                              topics: list) -> bool:
        """
        Insert or replace a named recording scheme.

        Args:
            user_id: owner of this scheme
            name:    display name, e.g. "Full sensor suite"
            topics:  list of topic strings, e.g. ["/cmd_vel", "/joy"]
        Returns:
            True on success, False on error
        """

    @abstractmethod
    def get_all_recording_schemes(self, user_id: int) -> dict:
        """
        Return all recording schemes for *user_id*.

        Returns:
            {
              "Full sensor suite": ["/cmd_vel", "/joy", ...],
              ...
            }
        """

    @abstractmethod
    def delete_recording_scheme(self, user_id: int, name: str) -> bool:
        """
        Delete a named recording scheme for *user_id*.
        Returns True on success, False on error.
        """
