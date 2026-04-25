"""
Database  —  SQLite implementation of IDatabase.

Schema
------
  users
      id          INTEGER PK AUTOINCREMENT
      name        TEXT UNIQUE NOT NULL
      created_utc TEXT NOT NULL

  actions
      id          INTEGER PK AUTOINCREMENT
      user_id     INTEGER FK → users.id  ON DELETE CASCADE
      name        TEXT NOT NULL
      action_type TEXT NOT NULL DEFAULT 'button_once'
      created_utc TEXT NOT NULL
      UNIQUE(user_id, name)

  action_tuples
      id           INTEGER PK AUTOINCREMENT
      action_id    INTEGER FK → actions.id  ON DELETE CASCADE
      topic        TEXT NOT NULL
      message      TEXT NOT NULL
      message_type TEXT NOT NULL DEFAULT 'std_msgs/msg/String'
      tuple_order  INTEGER NOT NULL
      scale_x      REAL NOT NULL DEFAULT 1.0
      scale_y      REAL NOT NULL DEFAULT 1.0
      offset_x     REAL NOT NULL DEFAULT 0.0
      offset_y     REAL NOT NULL DEFAULT 0.0
      publish_type TEXT NOT NULL DEFAULT 'topic'
      service_name TEXT NOT NULL DEFAULT ''

  mappings
      id          INTEGER PK AUTOINCREMENT
      user_id     INTEGER FK → users.id  ON DELETE CASCADE
      name        TEXT NOT NULL
      created_utc TEXT NOT NULL
      UNIQUE(user_id, name)

  mapping_buttons
      id           INTEGER PK AUTOINCREMENT
      mapping_id   INTEGER FK → mappings.id  ON DELETE CASCADE
      button_name  TEXT NOT NULL
      action_id    INTEGER FK → actions.id   ON DELETE CASCADE
      trigger_mode TEXT NOT NULL DEFAULT 'once'
      UNIQUE(mapping_id, button_name)

  mapping_joysticks
      id            INTEGER PK AUTOINCREMENT
      mapping_id    INTEGER FK → mappings.id  ON DELETE CASCADE
      joystick_name TEXT NOT NULL
      action_id     INTEGER FK → actions.id   ON DELETE CASCADE
      UNIQUE(mapping_id, joystick_name)

  recording_schemes
      id          INTEGER PK AUTOINCREMENT
      user_id     INTEGER FK → users.id  ON DELETE CASCADE
      name        TEXT NOT NULL
      created_utc TEXT NOT NULL
      UNIQUE(user_id, name)

  recording_scheme_topics
      id        INTEGER PK AUTOINCREMENT
      scheme_id INTEGER FK → recording_schemes.id  ON DELETE CASCADE
      topic     TEXT NOT NULL

Notes
-----
* Thread safety: every method opens and closes its own connection so
  concurrent Flask threads and the ROS2 node thread can call freely.
* The robot-side mapping_listener.py uses this class with username="robot"
  to get a predictable, single-user view of the schema.
* If the database file is missing it is created from scratch.  If it exists
  but has an unexpected schema the application raises DatabaseSchemaError.
"""

import os
import sqlite3
import threading
from datetime import datetime
from typing import Optional

from database_interface import (
    IDatabase,
    ACTION_BUTTON_ONCE,
    ACTION_BUTTON_HOLD,
    ACTION_JOYSTICK,
)

# ── Custom exceptions ─────────────────────────────────────────────────────────

class DatabaseDirectoryError(RuntimeError):
    """The database directory does not exist and could not be created."""

class DatabasePermissionError(RuntimeError):
    """The database directory or file is not writable."""

class DatabaseSchemaError(RuntimeError):
    """The on-disk schema is outdated or corrupt."""

class DatabaseConnectionError(RuntimeError):
    """SQLite could not open (or create) the database file."""


# ── Expected columns — used for schema validation ─────────────────────────────
# Maps table name → set of column names that MUST be present.
# Add new required columns here when the schema evolves.
_EXPECTED_COLUMNS = {
    'users': {'id', 'name', 'created_utc'},
    'actions': {'id', 'user_id', 'name', 'action_type', 'created_utc'},
    'action_tuples': {
        'id', 'action_id', 'topic', 'message', 'message_type', 'tuple_order',
        'scale_x', 'scale_y', 'offset_x', 'offset_y', 'publish_type', 'service_name',
    },
    'mappings': {'id', 'user_id', 'name', 'created_utc'},
    'mapping_buttons': {
        'id', 'mapping_id', 'button_name', 'action_id', 'trigger_mode',
    },
    'mapping_joysticks': {
        'id', 'mapping_id', 'joystick_name', 'action_id',
    },
    'recording_schemes': {'id', 'user_id', 'name', 'created_utc'},
    'recording_scheme_topics': {'id', 'scheme_id', 'topic'},
}


# ── Database class ────────────────────────────────────────────────────────────

class Database(IDatabase):
    """
    Encapsulates all SQLite access for the MecanumBot actions/mappings database.
    """

    def __init__(self, db_path: str):
        self._db_path    = db_path
        self._db_dir     = os.path.dirname(db_path)
        self._init_lock  = threading.Lock()
        self._initialized = False
        self._init_db()

    # ── Schema ────────────────────────────────────────────────────────────────

    def _init_db(self):
        """Create all tables (fresh DB) or validate the existing schema."""
        if self._initialized:
            return
        with self._init_lock:
            if self._initialized:
                return

            # ── Directory checks ──────────────────────────────────────────────
            if not os.path.isdir(self._db_dir):
                try:
                    os.makedirs(self._db_dir, exist_ok=True)
                except OSError as e:
                    raise DatabaseDirectoryError(
                        f"Database directory does not exist and could not be created: "
                        f"'{self._db_dir}'\n"
                        f"  OS error: {e}\n"
                        f"  This usually means the Docker volume is not mounted.\n"
                        f"  Start the container with: ./start_docker.sh"
                    ) from e
            elif not os.access(self._db_dir, os.W_OK):
                raise DatabasePermissionError(
                    f"Database directory is not writable: '{self._db_dir}'\n"
                    f"  Fix permissions on the host machine: chmod 755 ~/Documents"
                )

            # ── Open connection ───────────────────────────────────────────────
            try:
                conn_test = sqlite3.connect(self._db_path)
                conn_test.close()
            except sqlite3.DatabaseError as e:
                raise DatabaseConnectionError(
                    f"SQLite cannot open the database file: '{self._db_path}'\n"
                    f"  SQLite error: {e}\n"
                    f"  The file may be corrupt. Delete it and restart:\n"
                    f"      rm {self._db_path}"
                ) from e

            with sqlite3.connect(self._db_path) as conn:
                conn.execute("PRAGMA foreign_keys = ON")
                c = conn.cursor()

                # ── users ─────────────────────────────────────────────────────
                c.execute("""
                    CREATE TABLE IF NOT EXISTS users (
                        id          INTEGER PRIMARY KEY AUTOINCREMENT,
                        name        TEXT    UNIQUE NOT NULL,
                        created_utc TEXT    NOT NULL
                    )
                """)

                # ── actions ───────────────────────────────────────────────────
                c.execute("""
                    CREATE TABLE IF NOT EXISTS actions (
                        id          INTEGER PRIMARY KEY AUTOINCREMENT,
                        user_id     INTEGER NOT NULL,
                        name        TEXT    NOT NULL,
                        action_type TEXT    NOT NULL DEFAULT 'button_once',
                        created_utc TEXT    NOT NULL,
                        FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE,
                        UNIQUE(user_id, name)
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

                # ── mappings ──────────────────────────────────────────────────
                c.execute("""
                    CREATE TABLE IF NOT EXISTS mappings (
                        id          INTEGER PRIMARY KEY AUTOINCREMENT,
                        user_id     INTEGER NOT NULL,
                        name        TEXT    NOT NULL,
                        created_utc TEXT    NOT NULL,
                        FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE,
                        UNIQUE(user_id, name)
                    )
                """)

                # ── mapping_buttons ───────────────────────────────────────────
                c.execute("""
                    CREATE TABLE IF NOT EXISTS mapping_buttons (
                        id           INTEGER PRIMARY KEY AUTOINCREMENT,
                        mapping_id   INTEGER NOT NULL,
                        button_name  TEXT    NOT NULL,
                        action_id    INTEGER NOT NULL,
                        trigger_mode TEXT    NOT NULL DEFAULT 'once',
                        FOREIGN KEY (mapping_id) REFERENCES mappings(id) ON DELETE CASCADE,
                        FOREIGN KEY (action_id)  REFERENCES actions(id)  ON DELETE CASCADE,
                        UNIQUE(mapping_id, button_name)
                    )
                """)

                # ── mapping_joysticks ─────────────────────────────────────────
                c.execute("""
                    CREATE TABLE IF NOT EXISTS mapping_joysticks (
                        id            INTEGER PRIMARY KEY AUTOINCREMENT,
                        mapping_id    INTEGER NOT NULL,
                        joystick_name TEXT    NOT NULL,
                        action_id     INTEGER NOT NULL,
                        FOREIGN KEY (mapping_id) REFERENCES mappings(id) ON DELETE CASCADE,
                        FOREIGN KEY (action_id)  REFERENCES actions(id)  ON DELETE CASCADE,
                        UNIQUE(mapping_id, joystick_name)
                    )
                """)

                # ── recording_schemes ─────────────────────────────────────────
                c.execute("""
                    CREATE TABLE IF NOT EXISTS recording_schemes (
                        id          INTEGER PRIMARY KEY AUTOINCREMENT,
                        user_id     INTEGER NOT NULL,
                        name        TEXT    NOT NULL,
                        created_utc TEXT    NOT NULL,
                        FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE,
                        UNIQUE(user_id, name)
                    )
                """)

                # ── recording_scheme_topics ───────────────────────────────────
                c.execute("""
                    CREATE TABLE IF NOT EXISTS recording_scheme_topics (
                        id        INTEGER PRIMARY KEY AUTOINCREMENT,
                        scheme_id INTEGER NOT NULL,
                        topic     TEXT    NOT NULL,
                        FOREIGN KEY (scheme_id) REFERENCES recording_schemes(id) ON DELETE CASCADE
                    )
                """)

                conn.commit()

                # ── Schema validation ─────────────────────────────────────────
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
                    raise DatabaseSchemaError(
                        f"Database schema is outdated or corrupt: '{self._db_path}'\n"
                        f"  {details}\n"
                        f"  Delete the file and restart to create a fresh database:\n"
                        f"      rm {self._db_path}"
                    )

            self._initialized = True

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _connect(self):
        conn = sqlite3.connect(self._db_path)
        conn.execute("PRAGMA foreign_keys = ON")
        return conn

    # ── User management ───────────────────────────────────────────────────────

    def get_or_create_user(self, username: str) -> int:
        now = datetime.utcnow().isoformat()
        try:
            with self._connect() as conn:
                c = conn.cursor()
                c.execute("SELECT id FROM users WHERE name = ?", (username,))
                row = c.fetchone()
                if row:
                    return row[0]
                c.execute(
                    "INSERT INTO users (name, created_utc) VALUES (?, ?)",
                    (username, now)
                )
                conn.commit()
                return c.lastrowid
        except Exception as e:
            print(f"[DB] Error in get_or_create_user('{username}'): {e}")
            raise

    # ── Actions ───────────────────────────────────────────────────────────────

    def save_action(self, user_id: int, name: str, tuples: list,
                    action_type: str = ACTION_BUTTON_ONCE) -> bool:
        now = datetime.utcnow().isoformat()
        try:
            with self._connect() as conn:
                c = conn.cursor()

                c.execute(
                    "SELECT id FROM actions WHERE user_id = ? AND name = ?",
                    (user_id, name)
                )
                row = c.fetchone()

                if row:
                    action_id = row[0]
                    c.execute(
                        "DELETE FROM action_tuples WHERE action_id = ?",
                        (action_id,)
                    )
                    c.execute(
                        "UPDATE actions SET action_type = ? WHERE id = ?",
                        (action_type, action_id)
                    )
                else:
                    c.execute(
                        "INSERT INTO actions (user_id, name, action_type, created_utc) VALUES (?, ?, ?, ?)",
                        (user_id, name, action_type, now)
                    )
                    action_id = c.lastrowid

                for order, tup in enumerate(tuples):
                    topic        = tup[0]
                    message      = tup[1]
                    message_type = tup[2] if len(tup) >= 3 else 'std_msgs/msg/String'
                    scale_x      = float(tup[3]) if len(tup) >= 4 else 1.0
                    scale_y      = float(tup[4]) if len(tup) >= 5 else 1.0
                    offset_x     = float(tup[5]) if len(tup) >= 6 else 0.0
                    offset_y     = float(tup[6]) if len(tup) >= 7 else 0.0
                    publish_type = tup[7] if len(tup) >= 8 else 'topic'
                    service_name = tup[8] if len(tup) >= 9 else ''
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
            print(f"[DB] Error saving action '{name}' for user {user_id}: {e}")
            return False

    def get_all_actions(self, user_id: int,
                        action_type: Optional[str] = None) -> dict:
        actions = {}
        try:
            with self._connect() as conn:
                c = conn.cursor()
                if action_type:
                    c.execute(
                        "SELECT id, name, action_type FROM actions "
                        "WHERE user_id = ? AND action_type = ? ORDER BY name",
                        (user_id, action_type)
                    )
                else:
                    c.execute(
                        "SELECT id, name, action_type FROM actions "
                        "WHERE user_id = ? ORDER BY name",
                        (user_id,)
                    )
                for action_id, action_name, act_type in c.fetchall():
                    c.execute(
                        """SELECT topic, message, message_type,
                                  scale_x, scale_y, offset_x, offset_y,
                                  publish_type, service_name
                             FROM action_tuples
                            WHERE action_id = ?
                            ORDER BY tuple_order""",
                        (action_id,)
                    )
                    actions[action_name] = {
                        'id':     action_id,
                        'type':   act_type,
                        'tuples': c.fetchall(),
                    }
        except Exception as e:
            print(f"[DB] Error loading actions for user {user_id}: {e}")
        return actions

    def get_mappings_using_action(self, user_id: int, action_name: str) -> list:
        """Return names of all mappings that reference the given action name."""
        try:
            with self._connect() as conn:
                c = conn.cursor()
                c.execute(
                    """SELECT DISTINCT m.name
                         FROM mappings m
                         JOIN actions a ON a.user_id = m.user_id AND a.name = ?
                    LEFT JOIN mapping_buttons   mb ON mb.mapping_id = m.id AND mb.action_id = a.id
                    LEFT JOIN mapping_joysticks mj ON mj.mapping_id = m.id AND mj.action_id = a.id
                        WHERE m.user_id = ?
                          AND (mb.id IS NOT NULL OR mj.id IS NOT NULL)
                        ORDER BY m.name""",
                    (action_name, user_id)
                )
                return [row[0] for row in c.fetchall()]
        except Exception as e:
            print(f"[DB] Error getting mappings using action '{action_name}': {e}")
            return []

    def delete_action(self, user_id: int, name: str) -> bool:
        try:
            with self._connect() as conn:
                c = conn.cursor()
                # Resolve the action id for the given user/name. If it doesn't
                # exist there's nothing to do.
                c.execute(
                    "SELECT id FROM actions WHERE user_id = ? AND name = ?",
                    (user_id, name)
                )
                row = c.fetchone()
                if not row:
                    return True
                action_id = row[0]

                # Explicitly delete any mapping references that point to this
                # action. The schema already defines ON DELETE CASCADE but
                # some SQLite connections may not have foreign_keys enabled in
                # all contexts, so be defensive and remove dependent rows here.
                c.execute("DELETE FROM mapping_buttons   WHERE action_id = ?", (action_id,))
                c.execute("DELETE FROM mapping_joysticks WHERE action_id = ?", (action_id,))

                # Remove stored tuples for the action as well.
                c.execute("DELETE FROM action_tuples WHERE action_id = ?", (action_id,))

                # Finally remove the action itself.
                c.execute("DELETE FROM actions WHERE id = ?", (action_id,))
                conn.commit()
            return True
        except Exception as e:
            print(f"[DB] Error deleting action '{name}' for user {user_id}: {e}")
            return False

    # ── Mappings ──────────────────────────────────────────────────────────────

    def save_mapping(self, user_id: int, name: str,
                     button_entries: list,
                     joystick_entries: list) -> bool:
        """
        Insert or replace a named controller mapping.

        button_entries:   [(button_name, action_name, trigger_mode), ...]
        joystick_entries: [(joystick_name, action_name), ...]
        """
        now = datetime.utcnow().isoformat()
        try:
            with self._connect() as conn:
                c = conn.cursor()

                # Upsert the mapping row
                c.execute(
                    "SELECT id FROM mappings WHERE user_id = ? AND name = ?",
                    (user_id, name)
                )
                row = c.fetchone()
                if row:
                    mapping_id = row[0]
                    c.execute("DELETE FROM mapping_buttons   WHERE mapping_id = ?", (mapping_id,))
                    c.execute("DELETE FROM mapping_joysticks WHERE mapping_id = ?", (mapping_id,))
                else:
                    c.execute(
                        "INSERT INTO mappings (user_id, name, created_utc) VALUES (?, ?, ?)",
                        (user_id, name, now)
                    )
                    mapping_id = c.lastrowid

                # Insert button entries
                for btn, action_name, trigger_mode in button_entries:
                    c.execute(
                        "SELECT id FROM actions WHERE user_id = ? AND name = ?",
                        (user_id, action_name)
                    )
                    arow = c.fetchone()
                    if not arow:
                        print(f"[DB] save_mapping: action '{action_name}' not found for user {user_id}")
                        continue
                    c.execute(
                        """INSERT OR REPLACE INTO mapping_buttons
                              (mapping_id, button_name, action_id, trigger_mode)
                           VALUES (?, ?, ?, ?)""",
                        (mapping_id, btn, arow[0], trigger_mode)
                    )

                # Insert joystick entries
                for joy, action_name in joystick_entries:
                    c.execute(
                        "SELECT id FROM actions WHERE user_id = ? AND name = ?",
                        (user_id, action_name)
                    )
                    arow = c.fetchone()
                    if not arow:
                        print(f"[DB] save_mapping: action '{action_name}' not found for user {user_id}")
                        continue
                    c.execute(
                        """INSERT OR REPLACE INTO mapping_joysticks
                              (mapping_id, joystick_name, action_id)
                           VALUES (?, ?, ?)""",
                        (mapping_id, joy, arow[0])
                    )

                conn.commit()
            return True
        except Exception as e:
            print(f"[DB] Error saving mapping '{name}' for user {user_id}: {e}")
            return False

    def get_all_mappings(self, user_id: int) -> list:
        result = []
        try:
            with self._connect() as conn:
                c = conn.cursor()
                c.execute(
                    "SELECT id, name FROM mappings WHERE user_id = ? ORDER BY name",
                    (user_id,)
                )
                for mapping_id, mapping_name in c.fetchall():
                    entry = self._load_mapping_entries(c, mapping_id, mapping_name)
                    result.append(entry)
        except Exception as e:
            print(f"[DB] Error loading mappings for user {user_id}: {e}")
        return result

    def get_mapping(self, user_id: int, name: str) -> Optional[dict]:
        try:
            with self._connect() as conn:
                c = conn.cursor()
                c.execute(
                    "SELECT id FROM mappings WHERE user_id = ? AND name = ?",
                    (user_id, name)
                )
                row = c.fetchone()
                if not row:
                    return None
                return self._load_mapping_entries(c, row[0], name)
        except Exception as e:
            print(f"[DB] Error loading mapping '{name}' for user {user_id}: {e}")
            return None

    def _load_mapping_entries(self, c, mapping_id: int, mapping_name: str) -> dict:
        """Helper: load button/joystick entries for a mapping row."""
        c.execute(
            """SELECT mb.button_name, a.name, mb.trigger_mode
                 FROM mapping_buttons mb
                 JOIN actions a ON mb.action_id = a.id
                WHERE mb.mapping_id = ?
                ORDER BY mb.button_name""",
            (mapping_id,)
        )
        buttons = [
            {'button': btn, 'action': act, 'trigger_mode': mode}
            for btn, act, mode in c.fetchall()
        ]
        c.execute(
            """SELECT mj.joystick_name, a.name
                 FROM mapping_joysticks mj
                 JOIN actions a ON mj.action_id = a.id
                WHERE mj.mapping_id = ?
                ORDER BY mj.joystick_name""",
            (mapping_id,)
        )
        joysticks = [
            {'joystick': joy, 'action': act}
            for joy, act in c.fetchall()
        ]
        return {'name': mapping_name, 'buttons': buttons, 'joysticks': joysticks}

    def delete_mapping(self, user_id: int, name: str) -> bool:
        try:
            with self._connect() as conn:
                conn.execute(
                    "DELETE FROM mappings WHERE user_id = ? AND name = ?",
                    (user_id, name)
                )
                conn.commit()
            return True
        except Exception as e:
            print(f"[DB] Error deleting mapping '{name}' for user {user_id}: {e}")
            return False

    # ── Recording schemes ─────────────────────────────────────────────────────

    def save_recording_scheme(self, user_id: int, name: str,
                              topics: list) -> bool:
        now = datetime.utcnow().isoformat()
        try:
            with self._connect() as conn:
                c = conn.cursor()
                c.execute(
                    "SELECT id FROM recording_schemes WHERE user_id = ? AND name = ?",
                    (user_id, name)
                )
                row = c.fetchone()
                if row:
                    scheme_id = row[0]
                    c.execute(
                        "DELETE FROM recording_scheme_topics WHERE scheme_id = ?",
                        (scheme_id,)
                    )
                else:
                    c.execute(
                        "INSERT INTO recording_schemes (user_id, name, created_utc) VALUES (?, ?, ?)",
                        (user_id, name, now)
                    )
                    scheme_id = c.lastrowid
                for topic in topics:
                    c.execute(
                        "INSERT INTO recording_scheme_topics (scheme_id, topic) VALUES (?, ?)",
                        (scheme_id, topic)
                    )
                conn.commit()
            return True
        except Exception as e:
            print(f"[DB] Error saving recording scheme '{name}' for user {user_id}: {e}")
            return False

    def get_all_recording_schemes(self, user_id: int) -> dict:
        schemes = {}
        try:
            with self._connect() as conn:
                c = conn.cursor()
                c.execute(
                    "SELECT id, name FROM recording_schemes WHERE user_id = ? ORDER BY name",
                    (user_id,)
                )
                for scheme_id, scheme_name in c.fetchall():
                    c.execute(
                        "SELECT topic FROM recording_scheme_topics WHERE scheme_id = ? ORDER BY id",
                        (scheme_id,)
                    )
                    schemes[scheme_name] = [row[0] for row in c.fetchall()]
        except Exception as e:
            print(f"[DB] Error loading recording schemes for user {user_id}: {e}")
        return schemes

    def delete_recording_scheme(self, user_id: int, name: str) -> bool:
        try:
            with self._connect() as conn:
                conn.execute(
                    "DELETE FROM recording_schemes WHERE user_id = ? AND name = ?",
                    (user_id, name)
                )
                conn.commit()
            return True
        except Exception as e:
            print(f"[DB] Error deleting recording scheme '{name}' for user {user_id}: {e}")
            return False
