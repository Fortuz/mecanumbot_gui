from flask import Flask, request, render_template_string, redirect, url_for
import os
import csv
import sqlite3
from datetime import datetime

app = Flask(__name__)

# -------- Storage config --------
BASE_DIR = "/host_docs"  # bind-mounted host Documents
USE_SQLITE = False       # set True to use SQLite

# CSV files
ACTIONS_CSV = os.path.join(BASE_DIR, "actions.csv")
MAPPINGS_CSV = os.path.join(BASE_DIR, "mappings.csv")

# SQLite paths
SQLITE_PATH = os.path.join(BASE_DIR, "data")
SQLITE_DB = os.path.join(SQLITE_PATH, "app.db")

# Controller buttons to map (Xbox 360 set, incl. sticks + directions)
CONTROLLER_BUTTONS = [
    "A", "B", "X", "Y",
    "LB", "RB", "LT", "RT",
    "Start", "Back",
    "LS_Click", "RS_Click",           # stick clicks
    "LStickUp", "LStickDown", "LStickLeft", "LStickRight",
    "RStickUp", "RStickDown", "RStickLeft", "RStickRight",
    "DPadUp", "DPadDown", "DPadLeft", "DPadRight"
]

# -------- HTML templates --------

INDEX_HTML = """
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Submit Names</title>
  <style>
    body { font-family: system-ui, sans-serif; margin: 2rem; max-width: 720px; }
    form { display: grid; gap: .75rem; }
    label { font-weight: 600; }
    input[type=text] { padding: .5rem; border: 1px solid #cbd5e1; border-radius: .5rem; }
    button { padding: .6rem 1rem; border-radius: .5rem; border: 1px solid #cbd5e1; cursor: pointer; }
    .err { color: #7f1d1d; background: #fee2e2; padding: .5rem .75rem; border-radius: .5rem; }
    .hint { color: #475569; }
  </style>
</head>
<body>
  <h1>Save user & volunteer</h1>
  {% if msg %}<div class="err">{{ msg }}</div>{% endif %}
  <form method="post" action="{{ url_for('save') }}">
    <div>
      <label for="user_name">User name</label>
      <input type="text" id="user_name" name="user_name" placeholder="e.g. Alex Smith">
    </div>
    <div>
      <label for="volunteer_name">Volunteer name</label>
      <input type="text" id="volunteer_name" name="volunteer_name" placeholder="e.g. Jamie Doe">
    </div>
    <button type="submit">Save & Configure Controller</button>
  </form>
  <p class="hint">Storage: {{ storage_hint }}</p>
</body>
</html>
"""

CONFIGURE_HTML = """
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Configure Controller</title>
  <style>
    * { box-sizing: border-box; }
    body { margin: 0; font-family: system-ui, sans-serif; }
    .wrap { display: grid; grid-template-columns: 1fr 1fr; min-height: 100vh; }
    .left { background: #f1f5f9; display: flex; align-items: center; justify-content: center; padding: 2rem; } /* brighter */
    .left img { max-width: 100%; height: auto; border-radius: 1rem; box-shadow: 0 8px 24px rgba(0,0,0,.12); }
    .right { padding: 2rem; }
    h1 { margin-top: 0; }
    form { display: grid; gap: 1rem; }
    .row { display: grid; grid-template-columns: 160px 1fr; align-items: center; gap: .75rem; }
    select { padding: .5rem; border: 1px solid #cbd5e1; border-radius: .5rem; width: 100%; }
    button { justify-self: start; padding: .6rem 1rem; border-radius: .5rem; border: 1px solid #cbd5e1; cursor: pointer; }
    .note { color: #475569; margin-bottom: 1rem; }
    .pill { display:inline-block; background:#e2e8f0; padding:.2rem .5rem; border-radius:999px; margin-right:.25rem; font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="left">
      <img src="{{ url_for('static', filename='xbox-controller.svg.png') }}" alt="Xbox 360 Controller">
    </div>
    <div class="right">
      <h1>Map buttons to actions</h1>
      <p class="note">
        Choose an <strong>action</strong> for each controller input. Actions are loaded from
        {{ 'SQLite' if use_sqlite else 'CSV' }} ({{ actions_source }}).
      </p>

      {% if actions|length == 0 %}
        <p><em>No actions found. Add some to the actions store first.</em></p>
      {% else %}
      <form method="post" action="{{ url_for('save_mapping') }}">
        {% for btn in buttons %}
          <div class="row">
            <div><span class="pill">{{ btn }}</span></div>
            <div>
              <select name="btn_{{ btn }}">
                <option value="">-- Select action --</option>
                {% for a in actions %}
                  <option value="{{ a }}">{{ a }}</option>
                {% endfor %}
              </select>
            </div>
          </div>
        {% endfor %}
        <button type="submit">Save mappings</button>
      </form>
      {% endif %}
    </div>
  </div>
</body>
</html>
"""

SUCCESS_HTML = """
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Saved!</title>
  <style>
    body { font-family: system-ui, sans-serif; margin: 2rem; text-align:center; }
    a { display:inline-block; margin-top: 1rem; }
  </style>
</head>
<body>
  <h1>Mappings saved!</h1>
  <div><a href="{{ url_for('configure') }}">Back to controller</a></div>
  <div><a href="{{ url_for('index') }}">Back to names form</a></div>
</body>
</html>
"""

# -------- helpers --------

def ensure_dirs():
    os.makedirs(BASE_DIR, exist_ok=True)
    if USE_SQLITE:
        os.makedirs(SQLITE_PATH, exist_ok=True)

def init_sqlite():
    ensure_dirs()
    with sqlite3.connect(SQLITE_DB) as conn:
        c = conn.cursor()
        c.execute("""CREATE TABLE IF NOT EXISTS actions (
            action_name TEXT PRIMARY KEY
        )""")
        c.execute("""CREATE TABLE IF NOT EXISTS mappings (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            created_utc TEXT NOT NULL,
            button TEXT NOT NULL,
            action_name TEXT NOT NULL
        )""")
        conn.commit()

def seed_actions_if_empty_sqlite():
    init_sqlite()
    defaults = ["Jump", "Shoot", "Run", "Reload", "Crouch", "Interact", "Pause", "Sprint", "Melee", "Use"]
    with sqlite3.connect(SQLITE_DB) as conn:
        c = conn.cursor()
        c.execute("SELECT COUNT(*) FROM actions")
        cnt = c.fetchone()[0]
        if cnt == 0:
            c.executemany("INSERT OR IGNORE INTO actions(action_name) VALUES (?)",
                          [(a,) for a in defaults])
            conn.commit()

def read_actions_sqlite():
    seed_actions_if_empty_sqlite()
    with sqlite3.connect(SQLITE_DB) as conn:
        c = conn.cursor()
        c.execute("SELECT action_name FROM actions ORDER BY action_name COLLATE NOCASE")
        return [row[0] for row in c.fetchall()]

def save_mappings_sqlite(mapping_dict):
    init_sqlite()
    now = datetime.utcnow().isoformat()
    rows = [(now, btn, act) for btn, act in mapping_dict.items() if act]
    if not rows:
        return
    with sqlite3.connect(SQLITE_DB) as conn:
        c = conn.cursor()
        c.executemany(
            "INSERT INTO mappings (created_utc, button, action_name) VALUES (?, ?, ?)", rows
        )
        conn.commit()

def seed_actions_if_empty_csv():
    ensure_dirs()
    if not os.path.exists(ACTIONS_CSV):
        defaults = ["Jump", "Shoot", "Run", "Reload", "Crouch", "Interact", "Pause", "Sprint", "Melee", "Use"]
        with open(ACTIONS_CSV, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            for a in defaults:
                w.writerow([a])

def read_actions_csv():
    seed_actions_if_empty_csv()
    actions = []
    with open(ACTIONS_CSV, "r", newline="", encoding="utf-8") as f:
        r = csv.reader(f)
        for row in r:
            if row and row[0].strip():
                actions.append(row[0].strip())
    # dedupe & sort
    actions = sorted(list(dict.fromkeys(actions)), key=lambda x: x.lower())
    return actions

def save_mappings_csv(mapping_dict):
    ensure_dirs()
    new_file = not os.path.exists(MAPPINGS_CSV)
    with open(MAPPINGS_CSV, "a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if new_file:
            w.writerow(["created_utc", "button", "action_name"])
        now = datetime.utcnow().isoformat()
        for btn, act in mapping_dict.items():
            if act:
                w.writerow([now, btn, act])

def storage_hint_text():
    return (f"SQLite: {SQLITE_DB}" if USE_SQLITE else f"CSV: {ACTIONS_CSV} (actions), {MAPPINGS_CSV} (mappings)")

# -------- routes --------

@app.get("/")
def index():
    return render_template_string(
        INDEX_HTML,
        msg="",
        storage_hint=storage_hint_text()
    )

@app.post("/save")
def save():
    # Save the two names (demo: append to a simple CSV)
    user_name = (request.form.get("user_name") or "").strip()
    volunteer_name = (request.form.get("volunteer_name") or "").strip()
    if not user_name or not volunteer_name:
        return render_template_string(INDEX_HTML, msg="Please fill in both fields.", storage_hint=storage_hint_text())

    # Minimal persistence of names as log (CSV)
    names_csv = os.path.join(BASE_DIR, "submissions.csv")
    os.makedirs(BASE_DIR, exist_ok=True)
    new_file = not os.path.exists(names_csv)
    with open(names_csv, "a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if new_file:
            w.writerow(["created_utc", "user_name", "volunteer_name"])
        w.writerow([datetime.utcnow().isoformat(), user_name, volunteer_name])

    # Go to the controller configuration screen
    return redirect(url_for("configure"))

@app.get("/configure")
def configure():
    # Load actions from storage
    if USE_SQLITE:
        actions = read_actions_sqlite()
    else:
        actions = read_actions_csv()
    return render_template_string(
        CONFIGURE_HTML,
        actions=actions,
        buttons=CONTROLLER_BUTTONS,
        use_sqlite=USE_SQLITE,
        actions_source=(SQLITE_DB if USE_SQLITE else ACTIONS_CSV)
    )

@app.post("/map")
def save_mapping():
    # Build dict: button -> action (skip empty)
    mapping = {}
    for btn in CONTROLLER_BUTTONS:
        val = (request.form.get(f"btn_{btn}") or "").strip()
        mapping[btn] = val

    # Save to storage
    if USE_SQLITE:
        save_mappings_sqlite(mapping)
    else:
        save_mappings_csv(mapping)

    return render_template_string(SUCCESS_HTML)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080)
