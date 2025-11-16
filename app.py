from flask import Flask, request, render_template, redirect, url_for
import os
import sqlite3
import csv
from datetime import datetime

app = Flask(__name__)

# -------- Storage config --------
BASE_DIR = "/host_docs"  # bind-mounted host Documents

# Actions database
ACTIONS_DB = os.path.join(BASE_DIR, "actions.db")

# Action types
ACTION_TYPE_BUTTON = "button"
ACTION_TYPE_JOYSTICK = "joystick"

# Default Xbox 360 controller buttons
XBOX360_BUTTONS = [
    "A", "B", "X", "Y",
    "LB", "RB", "LT", "RT",
    "Start", "Back",
    "LS_Click", "RS_Click",           # stick clicks
    "LStickUp", "LStickDown", "LStickLeft", "LStickRight",
    "RStickUp", "RStickDown", "RStickLeft", "RStickRight",
    "DPadUp", "DPadDown", "DPadLeft", "DPadRight"
]

# PS4 controller buttons (DualShock 4)
PS4_BUTTONS = [
    "Cross", "Circle", "Square", "Triangle",
    "L1", "R1", "L2", "R2",
    "Share", "Options", "PS",
    "L3", "R3",                       # stick clicks
    "LStickUp", "LStickDown", "LStickLeft", "LStickRight",
    "RStickUp", "RStickDown", "RStickLeft", "RStickRight",
    "DPadUp", "DPadDown", "DPadLeft", "DPadRight",
    "Touchpad_Click", "Touchpad_Touch"  # PS4 specific features
]

# Global variable to store detected controller buttons
CONTROLLER_BUTTONS = []
CONTROLLER_TYPE = ""

# Joystick options (for joystick action mapping)
CONTROLLER_JOYSTICKS = ["Left Stick", "Right Stick"]

def detect_controller():
    """Detect plugged-in controller and return its type and available buttons."""
    global CONTROLLER_BUTTONS, CONTROLLER_TYPE
    
    # Use Xbox 360 layout as default for ROS2 environment
    print("Using Xbox 360 controller layout (standard for ROS2 joy node)")
    CONTROLLER_TYPE = "Xbox 360"
    CONTROLLER_BUTTONS = XBOX360_BUTTONS.copy()
    return True

# Detect controller on startup
print("Detecting controller...")
controller_detected = detect_controller()
if not controller_detected:
    print("Warning: No controller detected or error occurred. Using Xbox 360 layout as fallback.")
    CONTROLLER_TYPE = "Xbox 360 (Fallback)"
    CONTROLLER_BUTTONS = XBOX360_BUTTONS.copy()

# -------- Actions Database Functions --------

def ensure_dirs():
    """Ensure base directory exists"""
    os.makedirs(BASE_DIR, exist_ok=True)

def init_actions_db():
    """Initialize the actions database"""
    ensure_dirs()
    with sqlite3.connect(ACTIONS_DB) as conn:
        c = conn.cursor()
        # Table for actions
        c.execute("""CREATE TABLE IF NOT EXISTS actions (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT UNIQUE NOT NULL,
            action_type TEXT NOT NULL DEFAULT 'button',
            created_utc TEXT NOT NULL
        )""")
        
        # Migration: Add action_type column if it doesn't exist
        try:
            c.execute("SELECT action_type FROM actions LIMIT 1")
        except sqlite3.OperationalError:
            # Column doesn't exist, add it
            print("Migrating database: Adding action_type column...")
            c.execute("ALTER TABLE actions ADD COLUMN action_type TEXT NOT NULL DEFAULT 'button'")
            conn.commit()
        
        # Table for action tuples
        c.execute("""CREATE TABLE IF NOT EXISTS action_tuples (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            action_id INTEGER NOT NULL,
            topic TEXT NOT NULL,
            message TEXT NOT NULL,
            tuple_order INTEGER NOT NULL,
            FOREIGN KEY (action_id) REFERENCES actions(id) ON DELETE CASCADE
        )""")
        # Table for button mappings
        c.execute("""CREATE TABLE IF NOT EXISTS button_mappings (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            button_name TEXT UNIQUE NOT NULL,
            action_id INTEGER NOT NULL,
            created_utc TEXT NOT NULL,
            FOREIGN KEY (action_id) REFERENCES actions(id) ON DELETE CASCADE
        )""")
        # Table for joystick mappings
        c.execute("""CREATE TABLE IF NOT EXISTS joystick_mappings (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            joystick_name TEXT UNIQUE NOT NULL,
            action_id INTEGER NOT NULL,
            created_utc TEXT NOT NULL,
            FOREIGN KEY (action_id) REFERENCES actions(id) ON DELETE CASCADE
        )""")
        conn.commit()

def save_action_to_db(action_name, tuples, action_type=ACTION_TYPE_BUTTON):
    """Save an action with its tuples to the database"""
    init_actions_db()
    now = datetime.utcnow().isoformat()
    
    try:
        with sqlite3.connect(ACTIONS_DB) as conn:
            c = conn.cursor()
            
            # Check if action already exists
            c.execute("SELECT id FROM actions WHERE name = ?", (action_name,))
            existing = c.fetchone()
            
            if existing:
                # Update existing action - delete old tuples first
                action_id = existing[0]
                c.execute("DELETE FROM action_tuples WHERE action_id = ?", (action_id,))
                c.execute("UPDATE actions SET action_type = ? WHERE id = ?", (action_type, action_id))
            else:
                # Insert new action
                c.execute("INSERT INTO actions (name, action_type, created_utc) VALUES (?, ?, ?)", 
                         (action_name, action_type, now))
                action_id = c.lastrowid
            
            # Insert tuples
            for idx, (topic, message) in enumerate(tuples):
                c.execute("""INSERT INTO action_tuples 
                            (action_id, topic, message, tuple_order) 
                            VALUES (?, ?, ?, ?)""",
                         (action_id, topic, message, idx))
            
            conn.commit()
            return True
    except Exception as e:
        print(f"Error saving action to database: {e}")
        return False

def get_all_actions_from_db(action_type=None):
    """Get all actions from the database, optionally filtered by type"""
    init_actions_db()
    actions = {}
    
    try:
        with sqlite3.connect(ACTIONS_DB) as conn:
            c = conn.cursor()
            
            # Get all actions, optionally filtered by type
            if action_type:
                c.execute("SELECT id, name, action_type FROM actions WHERE action_type = ? ORDER BY name", (action_type,))
            else:
                c.execute("SELECT id, name, action_type FROM actions ORDER BY name")
            action_rows = c.fetchall()
            
            for action_id, action_name, act_type in action_rows:
                # Get tuples for this action
                c.execute("""SELECT topic, message FROM action_tuples 
                            WHERE action_id = ? ORDER BY tuple_order""", 
                         (action_id,))
                tuples = c.fetchall()
                
                actions[action_name] = {
                    "id": action_id,
                    "type": act_type,
                    "tuples": tuples
                }
        
        return actions
    except Exception as e:
        print(f"Error loading actions from database: {e}")
        return {}

def delete_action_from_db(action_name):
    """Delete an action from the database"""
    init_actions_db()
    
    try:
        with sqlite3.connect(ACTIONS_DB) as conn:
            c = conn.cursor()
            c.execute("DELETE FROM actions WHERE name = ?", (action_name,))
            conn.commit()
            return True
    except Exception as e:
        print(f"Error deleting action: {e}")
        return False

def save_button_mapping_to_db(button_name, action_name):
    """Save a button to action mapping to the database"""
    init_actions_db()
    now = datetime.utcnow().isoformat()
    
    try:
        with sqlite3.connect(ACTIONS_DB) as conn:
            c = conn.cursor()
            
            # Get action_id
            c.execute("SELECT id FROM actions WHERE name = ?", (action_name,))
            result = c.fetchone()
            if not result:
                return False
            
            action_id = result[0]
            
            # Insert or update button mapping
            c.execute("""INSERT OR REPLACE INTO button_mappings 
                        (button_name, action_id, created_utc) 
                        VALUES (?, ?, ?)""",
                     (button_name, action_id, now))
            
            conn.commit()
            return True
    except Exception as e:
        print(f"Error saving button mapping: {e}")
        return False

def get_all_button_mappings_from_db():
    """Get all button to action mappings from the database"""
    init_actions_db()
    mappings = {}
    
    try:
        with sqlite3.connect(ACTIONS_DB) as conn:
            c = conn.cursor()
            
            c.execute("""SELECT bm.button_name, a.name 
                        FROM button_mappings bm
                        JOIN actions a ON bm.action_id = a.id
                        ORDER BY bm.button_name""")
            
            for button_name, action_name in c.fetchall():
                mappings[button_name] = action_name
        
        return mappings
    except Exception as e:
        print(f"Error loading button mappings: {e}")
        return {}

def delete_button_mapping_from_db(button_name):
    """Delete a button mapping from the database"""
    init_actions_db()
    
    try:
        with sqlite3.connect(ACTIONS_DB) as conn:
            c = conn.cursor()
            c.execute("DELETE FROM button_mappings WHERE button_name = ?", (button_name,))
            conn.commit()
            return True
    except Exception as e:
        print(f"Error deleting button mapping: {e}")
        return False

def save_joystick_mapping_to_db(joystick_name, action_name):
    """Save a joystick to action mapping to the database"""
    init_actions_db()
    now = datetime.utcnow().isoformat()
    
    try:
        with sqlite3.connect(ACTIONS_DB) as conn:
            c = conn.cursor()
            
            # Get action_id
            c.execute("SELECT id FROM actions WHERE name = ?", (action_name,))
            result = c.fetchone()
            if not result:
                return False
            
            action_id = result[0]
            
            # Insert or update joystick mapping
            c.execute("""INSERT OR REPLACE INTO joystick_mappings 
                        (joystick_name, action_id, created_utc) 
                        VALUES (?, ?, ?)""",
                     (joystick_name, action_id, now))
            
            conn.commit()
            return True
    except Exception as e:
        print(f"Error saving joystick mapping: {e}")
        return False

def get_all_joystick_mappings_from_db():
    """Get all joystick to action mappings from the database"""
    init_actions_db()
    mappings = {}
    
    try:
        with sqlite3.connect(ACTIONS_DB) as conn:
            c = conn.cursor()
            
            c.execute("""SELECT jm.joystick_name, a.name 
                        FROM joystick_mappings jm
                        JOIN actions a ON jm.action_id = a.id
                        ORDER BY jm.joystick_name""")
            
            for joystick_name, action_name in c.fetchall():
                mappings[joystick_name] = action_name
        
        return mappings
    except Exception as e:
        print(f"Error loading joystick mappings: {e}")
        return {}

def delete_joystick_mapping_from_db(joystick_name):
    """Delete a joystick mapping from the database"""
    init_actions_db()
    
    try:
        with sqlite3.connect(ACTIONS_DB) as conn:
            c = conn.cursor()
            c.execute("DELETE FROM joystick_mappings WHERE joystick_name = ?", (joystick_name,))
            conn.commit()
            return True
    except Exception as e:
        print(f"Error deleting joystick mapping: {e}")
        return False

def get_controller_image():
    """Get the appropriate controller image filename based on detected controller type"""
    if "PS4" in CONTROLLER_TYPE or "DualShock 4" in CONTROLLER_TYPE or "ps4" in CONTROLLER_TYPE.lower():
        return "ps4_controller.jpeg"
    elif "Xbox 360" in CONTROLLER_TYPE or "xbox 360" in CONTROLLER_TYPE.lower():
        return "xbox-controller.svg.png"
    else:
        return "xbox-controller.svg.png"  # Default fallback

# -------- routes --------

@app.get("/")
def index():
    return render_template(
        "index.html",
        msg="",
        controller_type=CONTROLLER_TYPE,
        controller_detected=controller_detected
    )

@app.post("/save")
def save():
    # Save the user name with timestamp
    user_name = (request.form.get("user_name") or "").strip()
    volunteer_name = (request.form.get("volunteer_name") or "").strip()
    
    if not user_name:
        return render_template("index.html", 
                                    msg="Please enter a user name.", 
                                    controller_type=CONTROLLER_TYPE,
                                    controller_detected=controller_detected)

    # Save name and current time to CSV
    names_csv = os.path.join(BASE_DIR, "submissions.csv")
    os.makedirs(BASE_DIR, exist_ok=True)
    new_file = not os.path.exists(names_csv)
    with open(names_csv, "a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if new_file:
            w.writerow(["timestamp", "user_name"])
        w.writerow([datetime.utcnow().isoformat(), user_name])

    # Redirect back to home
    return redirect(url_for("index"))

@app.get("/configure")
def configure():
    # Return the HTML template with the dynamic button list
    return render_template(
        "configure.html",
        actions=[],
        buttons=CONTROLLER_BUTTONS,
        controller_type=CONTROLLER_TYPE
    )

@app.post("/redetect-controller")
def redetect_controller():
    """Re-detect the controller and redirect back to configure page"""
    global CONTROLLER_BUTTONS, CONTROLLER_TYPE
    print("Re-detecting controller...")
    
    detect_controller()
    return redirect(url_for("configure"))

@app.post("/map")
def save_mapping():
    # Legacy route - no longer used with new action-based system
    return render_template("success.html")

@app.get("/button-mapping")
def button_mapping():
    """Show action configuration page"""
    # Load existing actions from database
    actions = get_all_actions_from_db()
    return render_template(
        "button_mapping.html",
        buttons=CONTROLLER_BUTTONS,
        controller_type=CONTROLLER_TYPE,
        mappings=actions
    )

@app.post("/button-mapping/create")
def create_button_mapping():
    """Create a new action"""
    action_name = request.form.get("mapping_name", "").strip()
    tuple_list_raw = request.form.get("tuple_list", "").strip()
    action_type = request.form.get("action_type", ACTION_TYPE_BUTTON).strip()
    
    if not action_name or not tuple_list_raw:
        return redirect(url_for("button_mapping"))
    
    # Parse tuple list (one tuple per line: topic, message)
    tuples = []
    for line in tuple_list_raw.split('\n'):
        line = line.strip()
        if not line:
            continue
        
        # Split by first comma to separate topic and message
        if ',' in line:
            parts = line.split(',', 1)
            topic = parts[0].strip()
            message = parts[1].strip() if len(parts) > 1 else ""
            tuples.append((topic, message))
    
    if not tuples:
        return redirect(url_for("button_mapping"))
    
    # Save action to database with type
    save_action_to_db(action_name, tuples, action_type)
    
    return redirect(url_for("button_mapping"))

@app.post("/button-mapping/delete")
def delete_button_mapping():
    """Delete an action"""
    action_name = request.form.get("action_name", "").strip()
    
    if action_name:
        delete_action_from_db(action_name)
    
    return redirect(url_for("button_mapping"))

@app.get("/map-buttons")
def map_buttons():
    """Show button to action mapping page"""
    # Only get button-type actions (not joystick actions)
    actions = get_all_actions_from_db(action_type=ACTION_TYPE_BUTTON)
    button_mappings = get_all_button_mappings_from_db()
    
    return render_template(
        "map_buttons.html",
        buttons=CONTROLLER_BUTTONS,
        controller_type=CONTROLLER_TYPE,
        actions=actions,
        button_mappings=button_mappings
    )

@app.post("/map-buttons/save")
def save_button_action_mapping():
    """Save a button to action mapping"""
    button_name = request.form.get("button_name", "").strip()
    action_name = request.form.get("action_name", "").strip()
    
    if button_name and action_name:
        save_button_mapping_to_db(button_name, action_name)
        
        # Publish updated mappings to ROS2
        publish_button_action_mappings()
    
    return redirect(url_for("map_buttons"))

@app.get("/map-joysticks")
def map_joysticks():
    """Show joystick to action mapping page"""
    # Only get joystick-type actions
    actions = get_all_actions_from_db(action_type=ACTION_TYPE_JOYSTICK)
    joystick_mappings = get_all_joystick_mappings_from_db()
    
    return render_template(
        "map_joysticks.html",
        joysticks=CONTROLLER_JOYSTICKS,
        controller_type=CONTROLLER_TYPE,
        actions=actions,
        joystick_mappings=joystick_mappings
    )

@app.post("/map-joysticks/save")
def save_joystick_action_mapping():
    """Save a joystick to action mapping"""
    joystick_name = request.form.get("joystick_name", "").strip()
    action_name = request.form.get("action_name", "").strip()
    
    if joystick_name and action_name:
        save_joystick_mapping_to_db(joystick_name, action_name)
        
        # Publish updated mappings to ROS2
        publish_joystick_action_mappings()
    
    return redirect(url_for("map_joysticks"))

@app.post("/map-joysticks/delete")
def delete_joystick_action_mapping():
    """Delete a joystick to action mapping"""
    joystick_name = request.form.get("joystick_name", "").strip()
    
    if joystick_name:
        delete_joystick_mapping_from_db(joystick_name)
        
        # Publish updated mappings to ROS2
        publish_joystick_action_mappings()
    
    return redirect(url_for("map_joysticks"))

def publish_joystick_action_mappings():
    """Publish all joystick->action mappings to ROS2"""
    # Get all joystick mappings
    joystick_mappings = get_all_joystick_mappings_from_db()
    actions = get_all_actions_from_db()
    
    # Build complete mapping data for ROS2
    ros2_data = {}
    for joystick_name, action_name in joystick_mappings.items():
        if action_name in actions:
            ros2_data[joystick_name] = {
                "action_name": action_name,
                "tuples": actions[action_name]["tuples"]
            }
    
    print(f"Joystick mappings ready to publish: {len(ros2_data)} joysticks")
    # Note: ROS2 publishing handled by external ROS2 nodes
    return ros2_data

@app.post("/map-buttons/delete")
def delete_button_action_mapping():
    """Delete a button to action mapping"""
    button_name = request.form.get("button_name", "").strip()
    
    if button_name:
        delete_button_mapping_from_db(button_name)
        
        # Publish updated mappings to ROS2
        publish_button_action_mappings()
    
    return redirect(url_for("map_buttons"))

def publish_button_action_mappings():
    """Publish all button->action mappings to ROS2"""
    # Get all button mappings
    button_mappings = get_all_button_mappings_from_db()
    actions = get_all_actions_from_db()
    
    # Build complete mapping data for ROS2
    ros2_data = {}
    for button_name, action_name in button_mappings.items():
        if action_name in actions:
            ros2_data[button_name] = {
                "action_name": action_name,
                "tuples": actions[action_name]["tuples"]
            }
    
    print(f"Button mappings ready to publish: {len(ros2_data)} buttons")
    # Note: ROS2 publishing handled by external ROS2 nodes
    #kell egy topic
    return ros2_data

# Button and joystick mappings are stored in database and accessed by ROS2 nodes directly

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080)
