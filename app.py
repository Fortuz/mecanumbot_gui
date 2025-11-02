from flask import Flask, request, render_template, redirect, url_for
import os
import csv
import sqlite3
from datetime import datetime
import pygame
import sys
import requests
import json

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

def detect_controller():
    """Detect plugged-in controller and return its type and available buttons."""
    global CONTROLLER_BUTTONS, CONTROLLER_TYPE
    
    # Check if running in Docker
    in_docker = os.path.exists('/.dockerenv') or os.environ.get('container') == 'docker'
    
    # Try to get controller info from bridge service first (for Docker)
    if in_docker:
        try:
            # Try to connect to host bridge service
            # Docker's host.docker.internal or gateway IP
            bridge_urls = [
                "http://host.docker.internal:8899/controller-info",
                "http://172.17.0.1:8899/controller-info",  # Default Docker gateway
                "http://192.168.1.1:8899/controller-info",  # Common router IP
                "http://10.0.0.1:8899/controller-info"      # Another common gateway
            ]
            
            for url in bridge_urls:
                try:
                    print(f"Trying to connect to bridge service at {url}...")
                    response = requests.get(url, timeout=2)
                    if response.status_code == 200:
                        controller_data = response.json()
                        
                        CONTROLLER_TYPE = controller_data.get("controller_type", "Unknown") + " (via Bridge)"
                        CONTROLLER_BUTTONS = controller_data.get("controller_buttons", [])
                        detected = controller_data.get("controller_detected", False)
                        
                        print(f"✓ Connected to bridge service!")
                        print(f"Controller: {controller_data.get('controller_type', 'Unknown')}")
                        print(f"Buttons: {len(CONTROLLER_BUTTONS)}")
                        
                        return detected
                        
                except requests.exceptions.RequestException:
                    continue
            
            print("❌ Could not connect to bridge service!")
            print("Make sure to run the controller_bridge.py script on the host system first.")
            print("Expected bridge service at: http://host.docker.internal:8899")
            
        except Exception as e:
            print(f"Bridge service error: {e}")
    
    # Fallback to direct detection (for non-Docker or when bridge fails)
    try:
        # Initialize pygame for controller detection
        pygame.init()
        pygame.joystick.init()
        
        # Check if any controllers are connected
        joystick_count = pygame.joystick.get_count()
        
        if joystick_count == 0:
            if in_docker:
                print("No controllers detected! (Running in Docker - try using bridge service)")
                print("1. Run: python controller_bridge.py (on host)")
                print("2. Restart this container")
            else:
                print("No controllers detected!")
            CONTROLLER_TYPE = "None" + (" (Docker)" if in_docker else "")
            CONTROLLER_BUTTONS = []
            return False
        
        # Get the first controller
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        
        controller_name = joystick.get_name().lower()
        print(f"Controller detected: {joystick.get_name()}" + (" (in Docker)" if in_docker else ""))
        
        # Check if it's an Xbox 360 controller
        if "xbox 360" in controller_name or "xbox360" in controller_name or "360" in controller_name:
            CONTROLLER_TYPE = "Xbox 360" + (" (Docker)" if in_docker else "")
            CONTROLLER_BUTTONS = XBOX360_BUTTONS.copy()
            print("Using Xbox 360 button layout")
        # Check if it's a PS4 controller
        elif "ps4" in controller_name or "dualshock 4" in controller_name or "wireless controller" in controller_name:
            CONTROLLER_TYPE = "PS4 Controller" + (" (Docker)" if in_docker else "")
            CONTROLLER_BUTTONS = PS4_BUTTONS.copy()
            print("Using PS4 controller button layout")
        else:
            # For other controllers, detect available buttons dynamically
            CONTROLLER_TYPE = joystick.get_name() + (" (Docker)" if in_docker else "")
            CONTROLLER_BUTTONS = []
            
            # Get number of buttons
            num_buttons = joystick.get_numbuttons()
            for i in range(num_buttons):
                CONTROLLER_BUTTONS.append(f"Button_{i}")
            
            # Get number of axes (analog sticks, triggers)
            num_axes = joystick.get_numaxes()
            for i in range(num_axes):
                CONTROLLER_BUTTONS.append(f"Axis_{i}_Positive")
                CONTROLLER_BUTTONS.append(f"Axis_{i}_Negative")
            
            # Get number of hats (d-pads)
            num_hats = joystick.get_numhats()
            for i in range(num_hats):
                CONTROLLER_BUTTONS.append(f"Hat_{i}_Up")
                CONTROLLER_BUTTONS.append(f"Hat_{i}_Down")
                CONTROLLER_BUTTONS.append(f"Hat_{i}_Left")
                CONTROLLER_BUTTONS.append(f"Hat_{i}_Right")
            
            print(f"Detected {num_buttons} buttons, {num_axes} axes, {num_hats} hats")
            print(f"Using dynamic button layout: {CONTROLLER_BUTTONS}")
        
        return True
        
    except Exception as e:
        error_msg = f"Error detecting controller: {e}"
        if in_docker:
            error_msg += " (Docker environment - consider using bridge service)"
        print(error_msg)
        CONTROLLER_TYPE = "Error" + (" (Docker)" if in_docker else "")
        CONTROLLER_BUTTONS = []
        return False
    finally:
        # Clean up pygame
        try:
            pygame.joystick.quit()
            pygame.quit()
        except:
            pass

# Detect controller on startup
print("Detecting controller...")
controller_detected = detect_controller()
if not controller_detected:
    print("Warning: No controller detected or error occurred. Using Xbox 360 layout as fallback.")
    CONTROLLER_TYPE = "Xbox 360 (Fallback)"
    CONTROLLER_BUTTONS = XBOX360_BUTTONS.copy()

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
        controller_detected=controller_detected,
        storage_hint=storage_hint_text()
    )

@app.post("/save")
def save():
    # Save the two names (demo: append to a simple CSV)
    user_name = (request.form.get("user_name") or "").strip()
    volunteer_name = (request.form.get("volunteer_name") or "").strip()
    if not user_name or not volunteer_name:
        return render_template("index.html", 
                                    msg="Please fill in both fields.", 
                                    controller_type=CONTROLLER_TYPE,
                                    controller_detected=controller_detected,
                                    storage_hint=storage_hint_text())

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
    return render_template(
        "configure.html",
        actions=actions,
        buttons=CONTROLLER_BUTTONS,
        controller_type=CONTROLLER_TYPE,
        controller_image=get_controller_image(),
        use_sqlite=USE_SQLITE,
        actions_source=(SQLITE_DB if USE_SQLITE else ACTIONS_CSV)
    )

@app.post("/redetect-controller")
def redetect_controller():
    """Re-detect the controller and redirect back to configure page"""
    global CONTROLLER_BUTTONS, CONTROLLER_TYPE
    print("Re-detecting controller...")
    
    # If in Docker, try to trigger bridge service re-detection
    in_docker = os.path.exists('/.dockerenv') or os.environ.get('container') == 'docker'
    if in_docker:
        try:
            bridge_urls = [
                "http://host.docker.internal:8899/detect",
                "http://172.17.0.1:8899/detect",
                "http://192.168.1.1:8899/detect",
                "http://10.0.0.1:8899/detect"
            ]
            
            for url in bridge_urls:
                try:
                    response = requests.get(url, timeout=2)
                    if response.status_code == 200:
                        print("Triggered bridge service re-detection")
                        break
                except requests.exceptions.RequestException:
                    continue
        except Exception as e:
            print(f"Could not trigger bridge re-detection: {e}")
    
    detect_controller()
    return redirect(url_for("configure"))

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

    return render_template("success.html")

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080)
