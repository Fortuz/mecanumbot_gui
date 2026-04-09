"""
database.py — MecanumBot Actions Database

Schema overview:
─────────────────────────────────────────────────────────────────────────────

  actions                    — one row per named action (e.g. "Move Forward")
  ├── id              PK
  ├── name            unique display name
  ├── action_type     "button_once" | "button_hold" | "joystick"
  └── created_utc     ISO timestamp

  action_tuples              — one row per ROS2 publish/service step inside an action.
  │                            An action can target multiple topics/services in order.
  ├── id              PK
  ├── action_id       FK → actions.id  (CASCADE DELETE)
  ├── topic           ROS2 topic name  e.g. "/cmd_vel"  (unused for service rows)
  ├── message         JSON string      e.g. '{"linear":{"x":0.5}}'
  ├── message_type    ROS2 type string e.g. "geometry_msgs/msg/Twist"
  ├── tuple_order     execution order (0-based)
  ├── scale_x         axis X multiplier (default 1.0; hardcoded 1.0 from UI)
  ├── scale_y         axis Y multiplier (default 1.0; hardcoded 1.0 from UI)
  ├── offset_x        value added after scaling (default 0.0; user-configurable)
  ├── offset_y        value added after scaling (default 0.0; user-configurable)
  ├── publish_type    "topic" (default) | "service"
  └── service_name    ROS2 service name e.g. "set_led_status" (empty for topic rows)

  button_mappings            — maps one controller button → one action
  ├── id              PK
  ├── button_name     unique  e.g. "A", "LB", "DPadUp"
  ├── action_id       FK → actions.id  (CASCADE DELETE)
  ├── trigger_mode    "once" (fire on press) | "hold" (fire continuously at 10Hz)
  └── created_utc     ISO timestamp

  joystick_mappings          — maps one joystick axis → one action
  ├── id              PK
  ├── joystick_name   unique  e.g. "Left Stick", "Right Trigger (RT)"
  ├── action_id       FK → actions.id  (CASCADE DELETE)
  └── created_utc     ISO timestamp

─────────────────────────────────────────────────────────────────────────────

Relationships:
  actions  1──* action_tuples       (one action has 1 or more publish steps)
  actions  1──? button_mappings     (one action may be bound to one button)
  actions  1──? joystick_mappings   (one action may be bound to one joystick)

Example data:
  actions:          id=1, name="Move Forward", action_type="joystick"
  action_tuples:    action_id=1, topic="/cmd_vel", message='{"linear":{"x":"X"}}',
                    message_type="geometry_msgs/msg/Twist", tuple_order=0
  joystick_mappings: joystick_name="Left Stick", action_id=1

  actions:          id=2, name="Grab", action_type="button_once"
  action_tuples:    action_id=2, topic="/cmd_accessory_pos",
                    message='{"gl_pos":8.5,"gr_pos":8.5,"n_pos":5.3}',
                    message_type="mecanumbot_msgs/msg/AccessMotorCmd", tuple_order=0
  button_mappings:  button_name="A", action_id=2, trigger_mode="once"

─────────────────────────────────────────────────────────────────────────────
"""

import sqlite3
import os
import threading
from datetime import datetime

from database_interface import IDatabase


# ── Action type constants ─────────────────────────────────────────────────────

ACTION_BUTTON_ONCE = "button_once"   # fires once on button press
ACTION_BUTTON_HOLD = "button_hold"   # fires continuously while button is held
ACTION_JOYSTICK    = "joystick"      # fired by analog axis movement

# Tuple publish_type constants
PUBLISH_TOPIC   = "topic"    # publish to a ROS2 topic
PUBLISH_SERVICE = "service"  # call a ROS2 service

# ── Expected schema ───────────────────────────────────────────────────────────
# If the on-disk database is missing any of these columns the application will
# refuse to start rather than silently returning wrong data.  Delete the DB
# file and restart to get a fresh one.

_EXPECTED_COLUMNS = {
    'action_tuples': {
        'topic', 'message', 'message_type', 'tuple_order',
        'scale_x', 'scale_y', 'offset_x', 'offset_y',
        'publish_type', 'service_name',
    },
    'actions': {
        'id', 'name', 'action_type', 'created_utc',
    },
    'button_mappings': {
        'id', 'button_name', 'action_id', 'trigger_mode', 'created_utc',
    },
    'joystick_mappings': {
        'id', 'joystick_name', 'action_id', 'created_utc',
    },
}


# ── Database class ────────────────────────────────────────────────────────────

class Database(IDatabase):
    """
    Encapsulates all SQLite access for the MecanumBot actions database.

    A single instance is created by app.py and passed to DockerNode as a
    constructor argument, making the dependency explicit and testable.

    Thread safety: each method opens and closes its own connection so
    concurrent Flask threads and the ROS2 node thread can call methods
    without interfering with each other.
    """

    def __init__(self, db_path: str):
        self._db_path    = db_path
        self._db_dir     = os.path.dirname(db_path)
        self._init_lock  = threading.Lock()
        self._initialized = False
        self._init_db()

    # ── Schema ────────────────────────────────────────────────────────────────

    def _init_db(self):
        """
        Create all tables if they do not exist (fresh database).
        If the database already exists, validate that every expected column is
        present.  Raises RuntimeError if the schema is outdated or corrupt —
        delete the DB file and restart to recover.
        Skips all work after the first successful call in this process.
        """
        if self._initialized:
            return
        with self._init_lock:
            if self._initialized:   # re-check after acquiring lock
                return
            os.makedirs(self._db_dir, exist_ok=True)

            with sqlite3.connect(self._db_path) as conn:
                c = conn.cursor()

                # ── actions ───────────────────────────────────────────────────
                c.execute("""
                    CREATE TABLE IF NOT EXISTS actions (
                        id          INTEGER PRIMARY KEY AUTOINCREMENT,
                        name        TEXT    UNIQUE NOT NULL,
                        action_type TEXT    NOT NULL DEFAULT 'button',
                        created_utc TEXT    NOT NULL
                    )
                """)

                # ── action_tuples ─────────────────────────────────────────────
                c.execute("""
                    CREATE TABLE IF NOT EXISTS action_tuples (
                        id           INTEGER PRIMARY KEY AUTOINCREMENT,
                        action_id    INTEGER NOT NULL,
                        topic        TEXT    NOT NULL,
                        message      TEXT    NOT NULL,
                        message_type TEXT    NOT NULL DEFAULT 'std_msgs/msg/String',
                        tuple_order  INTEGER NOT NULL,
                        scale_x      REAL    NOT NULL DEFAULT 1.0,
                        scale_y      REAL    NOT NULL DEFAULT 1.0,
                        offset_x     REAL    NOT NULL DEFAULT 0.0,
                        offset_y     REAL    NOT NULL DEFAULT 0.0,
                        publish_type TEXT    NOT NULL DEFAULT 'topic',
                        service_name TEXT    NOT NULL DEFAULT '',
                        FOREIGN KEY (action_id) REFERENCES actions(id) ON DELETE CASCADE
                    )
                """)

                # ── button_mappings ───────────────────────────────────────────
                c.execute("""
                    CREATE TABLE IF NOT EXISTS button_mappings (
                        id           INTEGER PRIMARY KEY AUTOINCREMENT,
                        button_name  TEXT    UNIQUE NOT NULL,
                        action_id    INTEGER NOT NULL,
                        trigger_mode TEXT    NOT NULL DEFAULT 'once',
                        created_utc  TEXT    NOT NULL,
                        FOREIGN KEY (action_id) REFERENCES actions(id) ON DELETE CASCADE
                    )
                """)

                # ── joystick_mappings ─────────────────────────────────────────
                c.execute("""
                    CREATE TABLE IF NOT EXISTS joystick_mappings (
                        id            INTEGER PRIMARY KEY AUTOINCREMENT,
                        joystick_name TEXT    UNIQUE NOT NULL,
                        action_id     INTEGER NOT NULL,
                        created_utc   TEXT    NOT NULL,
                        FOREIGN KEY (action_id) REFERENCES actions(id) ON DELETE CASCADE
                    )
                """)

                conn.commit()

                # ── Schema validation ─────────────────────────────────────────
                # Runs on every startup (fresh or existing DB).  Raises immediately
                # if any expected column is missing instead of crashing later with a
                # confusing SQL error.
                missing = {}
                for table, expected_cols in _EXPECTED_COLUMNS.items():
                    c.execute(f"PRAGMA table_info({table})")
                    actual_cols = {row[1] for row in c.fetchall()}
                    absent = expected_cols - actual_cols
                    if absent:
                        missing[table] = sorted(absent)

                if missing:
                    details = '; '.join(
                        f"{tbl}: missing {cols}" for tbl, cols in missing.items()
                    )
                    raise RuntimeError(
                        f"[DB] Schema mismatch — the database at '{self._db_path}' is outdated or corrupt.\n"
                        f"  {details}\n"
                        f"  Delete '{self._db_path}' and restart the application to create a fresh database."
                    )

            self._initialized = True

    # ── Actions ───────────────────────────────────────────────────────────────

    def save_action(self, name, tuples, action_type=ACTION_BUTTON_ONCE):
        """
        Insert or update an action and its publish steps (tuples).

        Args:
            name:        Display name, e.g. "Move Forward"
            tuples:      List of (topic, message) or (topic, message, message_type)
            action_type: ACTION_BUTTON_ONCE | ACTION_BUTTON_HOLD | ACTION_JOYSTICK
        Returns:
            True on success, False on error
        """
        now = datetime.utcnow().isoformat()

        try:
            with sqlite3.connect(self._db_path) as conn:
                c = conn.cursor()

                # Check if action already exists
                c.execute("SELECT id FROM actions WHERE name = ?", (name,))
                row = c.fetchone()

                if row:
                    # Update: wipe old tuples, keep the same id and created_utc
                    # (created_utc intentionally preserved — it records the original creation time)
                    action_id = row[0]
                    c.execute("DELETE FROM action_tuples WHERE action_id = ?", (action_id,))
                    c.execute("UPDATE actions SET action_type = ? WHERE id = ?", (action_type, action_id))
                else:
                    # Insert new action
                    c.execute(
                        "INSERT INTO actions (name, action_type, created_utc) VALUES (?, ?, ?)",
                        (name, action_type, now)
                    )
                    action_id = c.lastrowid

                # Insert publish steps in order
                for order, tup in enumerate(tuples):
                    topic        = tup[0]
                    message      = tup[1]
                    message_type = tup[2] if len(tup) >= 3 else "std_msgs/msg/String"
                    scale_x      = float(tup[3]) if len(tup) >= 4 else 1.0
                    scale_y      = float(tup[4]) if len(tup) >= 5 else 1.0
                    offset_x     = float(tup[5]) if len(tup) >= 6 else 0.0
                    offset_y     = float(tup[6]) if len(tup) >= 7 else 0.0
                    publish_type = tup[7] if len(tup) >= 8 else "topic"
                    service_name = tup[8] if len(tup) >= 9 else ""
                    c.execute(
                        """INSERT INTO action_tuples
                              (action_id, topic, message, message_type, tuple_order,
                               scale_x, scale_y, offset_x, offset_y,
                               publish_type, service_name)
                           VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                        (action_id, topic, message, message_type, order,
                         scale_x, scale_y, offset_x, offset_y,
                         publish_type, service_name)
                    )

                conn.commit()
            return True
        except Exception as e:
            print(f"[DB] Error saving action '{name}': {e}")
            return False

    def get_all_actions(self, action_type=None):
        """
        Return all actions as a dict, optionally filtered by type.

        Returns:
            {
              "Move Forward": {
                  "id": 1,
                  "type": "joystick",
                  "tuples": [("/cmd_vel", '{"linear":{"x":"X"}}', "geometry_msgs/msg/Twist")]
              },
              ...
            }
        """
        actions = {}

        try:
            with sqlite3.connect(self._db_path) as conn:
                c = conn.cursor()

                if action_type:
                    c.execute(
                        "SELECT id, name, action_type FROM actions WHERE action_type = ? ORDER BY name",
                        (action_type,)
                    )
                else:
                    c.execute("SELECT id, name, action_type FROM actions ORDER BY name")

                for action_id, action_name, act_type in c.fetchall():
                    c.execute(
                        """SELECT topic, message, message_type, scale_x, scale_y, offset_x, offset_y,
                                  publish_type, service_name
                             FROM action_tuples
                            WHERE action_id = ?
                            ORDER BY tuple_order""",
                        (action_id,)
                    )
                    actions[action_name] = {
                        "id":     action_id,
                        "type":   act_type,
                        "tuples": c.fetchall()
                    }
        except Exception as e:
            print(f"[DB] Error loading actions: {e}")

        return actions

    def delete_action(self, name):
        """
        Delete an action by name.
        Cascades to action_tuples, button_mappings, and joystick_mappings automatically.
        Returns True on success, False on error.
        """
        try:
            with sqlite3.connect(self._db_path) as conn:
                conn.execute("PRAGMA foreign_keys = ON")
                conn.execute("DELETE FROM actions WHERE name = ?", (name,))
                conn.commit()
            return True
        except Exception as e:
            print(f"[DB] Error deleting action '{name}': {e}")
            return False

    # ── Button Mappings ───────────────────────────────────────────────────────

    def save_button_mapping(self, button_name, action_name, trigger_mode="once"):
        """
        Bind a controller button to an action.

        Args:
            button_name:  e.g. "A", "LB", "DPadUp"
            action_name:  must already exist in the actions table
            trigger_mode: "once" | "hold"
        Returns:
            True on success, False if action not found or on error
        """
        now = datetime.utcnow().isoformat()

        try:
            with sqlite3.connect(self._db_path) as conn:
                c = conn.cursor()

                c.execute("SELECT id FROM actions WHERE name = ?", (action_name,))
                row = c.fetchone()
                if not row:
                    print(f"[DB] Action '{action_name}' not found — cannot map button")
                    return False

                action_id = row[0]
                c.execute(
                    """INSERT OR REPLACE INTO button_mappings
                          (button_name, action_id, trigger_mode, created_utc)
                       VALUES (?, ?, ?, ?)""",
                    (button_name, action_id, trigger_mode, now)
                )
                conn.commit()
            return True
        except Exception as e:
            print(f"[DB] Error saving button mapping '{button_name}': {e}")
            return False

    def get_all_button_mappings(self):
        """
        Return all button mappings as a dict.

        Returns:
            { "A": {"action": "Grab", "trigger_mode": "once"}, ... }
        """
        mappings = {}

        try:
            with sqlite3.connect(self._db_path) as conn:
                c = conn.cursor()
                c.execute(
                    """SELECT bm.button_name, a.name, bm.trigger_mode
                         FROM button_mappings bm
                         JOIN actions a ON bm.action_id = a.id
                        ORDER BY bm.button_name"""
                )
                for button_name, action_name, trigger_mode in c.fetchall():
                    mappings[button_name] = {
                        "action":       action_name,
                        "trigger_mode": trigger_mode or "once"
                    }
        except Exception as e:
            print(f"[DB] Error loading button mappings: {e}")

        return mappings

    def delete_button_mapping(self, button_name):
        """Remove a button binding. Returns True on success, False on error."""
        try:
            with sqlite3.connect(self._db_path) as conn:
                conn.execute("DELETE FROM button_mappings WHERE button_name = ?", (button_name,))
                conn.commit()
            return True
        except Exception as e:
            print(f"[DB] Error deleting button mapping '{button_name}': {e}")
            return False

    # ── Joystick Mappings ─────────────────────────────────────────────────────

    def save_joystick_mapping(self, joystick_name, action_name):
        """
        Bind a joystick axis to an action.

        Args:
            joystick_name: e.g. "Left Stick", "Right Trigger (RT)"
            action_name:   must already exist in the actions table
        Returns:
            True on success, False if action not found or on error
        """
        now = datetime.utcnow().isoformat()

        try:
            with sqlite3.connect(self._db_path) as conn:
                c = conn.cursor()

                c.execute("SELECT id FROM actions WHERE name = ?", (action_name,))
                row = c.fetchone()
                if not row:
                    print(f"[DB] Action '{action_name}' not found — cannot map joystick")
                    return False

                action_id = row[0]
                c.execute(
                    """INSERT OR REPLACE INTO joystick_mappings
                          (joystick_name, action_id, created_utc)
                       VALUES (?, ?, ?)""",
                    (joystick_name, action_id, now)
                )
                conn.commit()
            return True
        except Exception as e:
            print(f"[DB] Error saving joystick mapping '{joystick_name}': {e}")
            return False

    def get_all_joystick_mappings(self):
        """
        Return all joystick mappings as a dict.

        Returns:
            { "Left Stick": "Move Forward", "Right Trigger (RT)": "Grab", ... }
        """
        mappings = {}

        try:
            with sqlite3.connect(self._db_path) as conn:
                c = conn.cursor()
                c.execute(
                    """SELECT jm.joystick_name, a.name
                         FROM joystick_mappings jm
                         JOIN actions a ON jm.action_id = a.id
                        ORDER BY jm.joystick_name"""
                )
                for joystick_name, action_name in c.fetchall():
                    mappings[joystick_name] = action_name
        except Exception as e:
            print(f"[DB] Error loading joystick mappings: {e}")

        return mappings

    def delete_joystick_mapping(self, joystick_name):
        """Remove a joystick binding. Returns True on success, False on error."""
        try:
            with sqlite3.connect(self._db_path) as conn:
                conn.execute("DELETE FROM joystick_mappings WHERE joystick_name = ?", (joystick_name,))
                conn.commit()
            return True
        except Exception as e:
            print(f"[DB] Error deleting joystick mapping '{joystick_name}': {e}")
            return False
