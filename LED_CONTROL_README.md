# LED Control System - Technical Documentation

Complete guide to the MecanumBot RGB LED control system, covering hardware architecture, software implementation, service interfaces, and action creation.

---

## 📋 Table of Contents

- [System Overview](#system-overview)
- [Hardware Architecture](#hardware-architecture)
- [Software Architecture](#software-architecture)
- [ROS2 Service Interface](#ros2-service-interface)
- [LED Action Creation](#led-action-creation)
- [Dashboard Control](#dashboard-control)
- [LED Modes](#led-modes)
- [Color Palette](#color-palette)
- [Implementation Details](#implementation-details)
- [Message Flow](#message-flow)
- [Troubleshooting](#troubleshooting)
- [Advanced Usage](#advanced-usage)

---

## 🎯 System Overview

The MecanumBot features **4 RGB LED strips** mounted at each corner of the robot chassis, providing visual feedback and aesthetic lighting. Each corner can be independently controlled with different colors and animation modes.

### Key Features

- **4 Independent Corners:** Front-Left (FL), Front-Right (FR), Back-Left (BL), Back-Right (BR)
- **8 Colors:** Black, White, Green, Red, Blue, Cyan, Pink, Yellow
- **4 Animation Modes:** Wave Right, Wave Left, Pulse, Solid
- **ROS2 Integration:** Services for get/set operations
- **Web GUI Control:** Real-time status monitoring and direct control
- **Button Mapping:** Create LED actions triggered by controller buttons

---

## 🔌 Hardware Architecture

### Physical Layout

```
         FRONT
    FL ────┬──── FR
           │
           │   Robot Chassis
    BL ────┴──── BR
         BACK
```

### Hardware Specifications

| Component | Specification |
|-----------|---------------|
| LED Type | WS2812B RGB (individually addressable) |
| LEDs per Strip | 8-16 per corner |
| Controller | Arduino Nano (LED Control Board) |
| Communication | Serial UART (9600 baud) |
| Voltage | 5V DC |
| Current | ~60mA per LED at full white |
| Total Power | ~15W at full brightness |

### Connection Topology

```
┌─────────────────────┐
│   Raspberry Pi 4    │
│   (ROS2 Node)       │
└──────────┬──────────┘
           │ ROS2 Service
           │ (SetLedStatus/GetLedStatus)
           ▼
┌─────────────────────┐
│  LED Control Node   │
│  (led_service)      │
└──────────┬──────────┘
           │ Serial UART
           │ (Protocol)
           ▼
┌─────────────────────┐
│  Arduino Nano       │
│  (LED Controller)   │
└──────────┬──────────┘
           │ Data Lines (4x)
           ▼
┌─────────────────────┐
│  WS2812B LED Strips │
│  FL, FR, BL, BR     │
└─────────────────────┘
```

---

## 💻 Software Architecture

### Component Stack

```
┌────────────────────────────────────────────────────┐
│                Web Browser (User)                   │
│           http://localhost:8080                     │
└───────────────────┬────────────────────────────────┘
                    │ HTTP/JSON
                    ▼
┌────────────────────────────────────────────────────┐
│         Docker Container (Host)                     │
│  ┌──────────────────────────────────────────────┐ │
│  │  Flask Application (app.py)                  │ │
│  │  - /api/led/status  (GET)                    │ │
│  │  - /api/led/get     (POST)                   │ │
│  │  - /api/led/set     (POST)                   │ │
│  └──────────────┬───────────────────────────────┘ │
│                 │                                   │
│  ┌──────────────▼───────────────────────────────┐ │
│  │  DockerNode (ROS2)                           │ │
│  │  - LED service clients                       │ │
│  │  - get_led_status()  → calls service         │ │
│  │  - set_led_status()  → calls service         │ │
│  └──────────────┬───────────────────────────────┘ │
└─────────────────┼───────────────────────────────────┘
                  │ ROS2 Services (DDS)
                  │ - /get_led_status
                  │ - /set_led_status
                  ▼
┌────────────────────────────────────────────────────┐
│         Robot (Raspberry Pi)                        │
│  ┌──────────────────────────────────────────────┐ │
│  │  MappingListener (mapping_listener.py)       │ │
│  │  - Executes LED actions from buttons         │ │
│  │  - Calls set_led_status service              │ │
│  └──────────────┬───────────────────────────────┘ │
│                 │                                   │
│  ┌──────────────▼───────────────────────────────┐ │
│  │  LED Service Node (led_service)              │ │
│  │  - Service servers:                          │ │
│  │    * /get_led_status                         │ │
│  │    * /set_led_status                         │ │
│  │  - Serial communication to Arduino           │ │
│  └──────────────┬───────────────────────────────┘ │
└─────────────────┼───────────────────────────────────┘
                  │ UART (9600 baud)
                  ▼
         ┌────────────────────┐
         │  Arduino Nano      │
         │  - Protocol parser │
         │  - LED driver      │
         └────────────────────┘
```

### Code Components

| Component | File | Responsibility |
|-----------|------|----------------|
| Flask API | `host/app.py` | HTTP endpoints for LED control |
| Docker Node | `host/app.py` (DockerNode class) | ROS2 service client wrapper |
| Robot Executor | `robot/button_mapping_ros/mapping_listener.py` | Execute LED actions from controller |
| Service Definitions | `shared/mecanumbot_msgs/srv/` | SetLedStatus.srv, GetLedStatus.srv |
| Frontend | `host/templates/main_dashboard.html` | LED status panel + polling |
| Button Mapping | `host/templates/button_mapping.html` | LED action creation UI |

---

## 🔧 ROS2 Service Interface

### Service Definitions

#### `mecanumbot_msgs/srv/GetLedStatus.srv`

```
# Request (empty)
---
# Response
uint8 fl_mode      # Front-Left mode (1-4)
uint8 fl_color     # Front-Left color (0-7)
uint8 fr_mode      # Front-Right mode (1-4)
uint8 fr_color     # Front-Right color (0-7)
uint8 bl_mode      # Back-Left mode (1-4)
uint8 bl_color     # Back-Left color (0-7)
uint8 br_mode      # Back-Right mode (1-4)
uint8 br_color     # Back-Right color (0-7)
```

#### `mecanumbot_msgs/srv/SetLedStatus.srv`

```
# Request
uint8 fl_mode      # Front-Left mode (1-4)
uint8 fl_color     # Front-Left color (0-7)
uint8 fr_mode      # Front-Right mode (1-4)
uint8 fr_color     # Front-Right color (0-7)
uint8 bl_mode      # Back-Left mode (1-4)
uint8 bl_color     # Back-Left color (0-7)
uint8 br_mode      # Back-Right mode (1-4)
uint8 br_color     # Back-Right color (0-7)
---
# Response
bool success       # True if command accepted
```

### Service Names

**Important:** Services are registered with **bare names** (no namespace):

```python
# Correct
self.create_client(SetLedStatus, 'set_led_status')
self.create_client(GetLedStatus, 'get_led_status')

# Incorrect (would fail)
self.create_client(SetLedStatus, '/mecanumbot/set_led_status')
```

### Usage Examples

#### Command Line

```bash
# Get current LED state
ros2 service call /get_led_status mecanumbot_msgs/srv/GetLedStatus

# Set all corners to green pulse
ros2 service call /set_led_status mecanumbot_msgs/srv/SetLedStatus \
  "{fl_mode: 3, fl_color: 2, \
    fr_mode: 3, fr_color: 2, \
    bl_mode: 3, bl_color: 2, \
    br_mode: 3, br_color: 2}"
```

#### Python (ROS2 Node)

```python
from mecanumbot_msgs.srv import SetLedStatus, GetLedStatus

# Create client
led_client = self.create_client(SetLedStatus, 'set_led_status')

# Build request
req = SetLedStatus.Request()
req.fl_mode = 4   # Solid
req.fl_color = 2  # Green
req.fr_mode = 4
req.fr_color = 2
req.bl_mode = 4
req.bl_color = 2
req.br_mode = 4
req.br_color = 2

# Call service
future = led_client.call_async(req)
rclpy.spin_until_future_complete(self, future)
result = future.result()

if result.success:
    print("LED command accepted")
else:
    print("LED command failed")
```

---

## 🎨 LED Action Creation

### Via Web Interface

1. **Navigate to "Create Actions"** button in main dashboard
2. **Fill action form:**
   - **Action Name:** `Party Lights`
   - **Action Type:** Select `LED Lights` from dropdown
3. **Configure each corner** using the LED configuration table:

   | Corner | Color | Mode |
   |--------|-------|------|
   | Front-Left | Green | Pulse |
   | Front-Right | Red | Wave → |
   | Back-Left | Blue | Wave ← |
   | Back-Right | Yellow | Solid |

4. **Click "Create Action"**
5. **Map to button:** Navigate to "Map Controls" and assign to button (e.g., "Y")

### JSON Format (Internal)

When saved to database, LED actions use this format:

```json
{
  "topic": "__led__",
  "message": "{\"FL\":{\"mode\":3,\"color\":2},\"FR\":{\"mode\":1,\"color\":3},\"BL\":{\"mode\":2,\"color\":4},\"BR\":{\"mode\":4,\"color\":7}}",
  "message_type": "mecanumbot_msgs/srv/SetLedStatus"
}
```

**Key Points:**
- Topic is special marker `__led__`
- Message is JSON containing 4 corner configurations
- Type is `SetLedStatus` service (not a topic publish)

### Execution Flow

```
1. User presses button "Y"
   ↓
2. Xbox controller publishes button event
   ↓
3. MappingListener receives event
   ↓
4. Looks up button mapping: Y → "Party Lights"
   ↓
5. Loads action from memory
   ↓
6. Detects topic == "__led__"
   ↓
7. Parses JSON config
   ↓
8. Builds SetLedStatus.Request
   ↓
9. Calls service asynchronously
   ↓
10. LED service forwards to Arduino
    ↓
11. Arduino updates LED strips
```

---

## 🖥️ Dashboard Control

### LED Status Panel

Located in main dashboard (`main_dashboard.html`), displays current LED state:

```
┌─────────────────────────────────────────┐
│ 🟢 LED Status  (Updated: 14:32:15)      │
├─────────────────────────────────────────┤
│  Front-Left    ●  GREEN    / Pulse      │
│  Front-Right   ●  RED      / Wave →     │
│  Back-Left     ●  BLUE     / Wave ←     │
│  Back-Right    ●  YELLOW   / Solid      │
└─────────────────────────────────────────┘
```

### Polling Mechanism

```javascript
// JavaScript in main_dashboard.html
let _ledPolling = false;   // Guard prevents concurrent requests

function updateLedStatus() {
    if (_ledPolling) return;   // Skip if already in-flight
    _ledPolling = true;

    fetch('/api/led/get', { method: 'POST' })
        .then(r => r.json())
        .then(data => {
            _ledPolling = false;
            if (data.success) {
                // Update UI with data.corners
                displayLedState(data.corners);
            } else {
                // Show offline state
                displayLedOffline();
            }
        })
        .catch(() => {
            _ledPolling = false;
            displayLedOffline();
        });
}

// Poll every 5 seconds
setInterval(updateLedStatus, 5000);
```

### API Workflow

1. **Frontend** sends `POST /api/led/get`
2. **Flask** calls `DockerNode.get_led_status()`
3. **DockerNode** calls ROS2 service `/get_led_status`
4. **LED service** responds with current state
5. **Flask** caches state in `LATEST_LED_STATE` global
6. **Flask** returns JSON to frontend
7. **Frontend** updates dashboard panel

---

## 🌈 LED Modes

### Mode 1: Wave Right (→)

**Behavior:** Color wave animation moving from left to right across strip

**Characteristics:**
- Wave speed: ~50 pixels/second
- Smooth gradient transition
- Continuous loop
- Uses palette interpolation

**Best For:** Dynamic, flowing effects

**Code Reference:**
```cpp
// Arduino implementation
void colorWaveFix(CRGB* ledarray, uint16_t numleds, 
                  const CRGBPalette16& palette, int direction)
{
    // direction = +1 for right, -1 for left
    // Uses sinusoidal brightness modulation
    // Hue shifts based on position and time
}
```

### Mode 2: Wave Left (←)

**Behavior:** Same as Mode 1, but wave moves right to left

**Characteristics:**
- Opposite direction of Mode 1
- Same speed and smoothness
- Mirror effect of Wave Right

**Best For:** Complementary to Mode 1, creates circular effect when used on opposite corners

### Mode 3: Pulse

**Behavior:** Entire strip fades in and out synchronously

**Characteristics:**
- Pulse period: ~2 seconds (full cycle)
- Smooth sine-wave brightness curve
- All LEDs in strip pulse together
- Color remains constant

**Best For:** Breathing effects, status indication, calm ambiance

**Code Reference:**
```cpp
// Brightness varies: min → max → min
brightness = (sin(time * frequency) + 1.0) * 0.5 * max_brightness
```

### Mode 4: Solid

**Behavior:** All LEDs set to constant color and brightness

**Characteristics:**
- No animation
- Immediate response
- Lowest power consumption
- Maximum brightness

**Best For:** Static lighting, color coding, maximum visibility

---

## 🎨 Color Palette

### Color Index Mapping

| Index | Color  | RGB Values | Hex Code | Use Case |
|-------|--------|------------|----------|----------|
| 0 | Black  | (26,26,26)    | #1a1a1a  | Off/Dark |
| 1 | White  | (240,240,240) | #f0f0f0  | Maximum light |
| 2 | Green  | (34,197,94)   | #22c55e  | Success, go, active |
| 3 | Red    | (239,68,68)   | #ef4444  | Error, stop, alert |
| 4 | Blue   | (59,130,246)  | #3b82f6  | Info, water, calm |
| 5 | Cyan   | (6,182,212)   | #06b6d4  | Highlight, tech |
| 6 | Pink   | (236,72,153)  | #ec4899  | Fun, party, creative |
| 7 | Yellow | (234,179,8)   | #eab308  | Warning, attention |

### Color Selection Logic

```javascript
// Frontend color picker (button_mapping.html)
const LED_COLORS = [
    { name: 'Black',  hex: '#1a1a1a' },
    { name: 'White',  hex: '#f0f0f0' },
    { name: 'Green',  hex: '#22c55e' },
    { name: 'Red',    hex: '#ef4444' },
    { name: 'Blue',   hex: '#3b82f6' },
    { name: 'Cyan',   hex: '#06b6d4' },
    { name: 'Pink',   hex: '#ec4899' },
    { name: 'Yellow', hex: '#eab308' }
];

// Rendered as dropdown for each corner
<select class="led-color-select">
    <option value="0">Black</option>
    <option value="1">White</option>
    <option value="2">Green</option>
    <!-- ... -->
</select>
```

---

## ⚙️ Implementation Details

### Host-Side (Docker)

#### DockerNode LED Methods

**File:** `host/app.py`

```python
class DockerNode(Node):
    def __init__(self):
        # Create LED service clients (bare names)
        if LED_SRV_AVAILABLE:
            self._led_set_client = self.create_client(
                SetLedStatus, 'set_led_status')
            self._led_get_client = self.create_client(
                GetLedStatus, 'get_led_status')

    def get_led_status(self):
        """Query robot for current LED state."""
        if not LED_SRV_AVAILABLE or self._led_get_client is None:
            return None, 'LED service not available'
        
        req = GetLedStatus.Request()
        future = self._led_get_client.call_async(req)
        
        # Poll with timeout (8 seconds)
        elapsed, timeout, poll = 0.0, 8.0, 0.05
        while not future.done():
            if elapsed >= timeout:
                return None, 'get_led_status timed out'
            time.sleep(poll)
            elapsed += poll
        
        result = future.result()
        if result is None:
            return None, 'Service returned None'
        
        # Convert to dict format
        state = {
            'FL': {'mode': result.fl_mode, 'color': result.fl_color},
            'FR': {'mode': result.fr_mode, 'color': result.fr_color},
            'BL': {'mode': result.bl_mode, 'color': result.bl_color},
            'BR': {'mode': result.br_mode, 'color': result.br_color},
        }
        return state, ''

    def set_led_status(self, values):
        """Send LED configuration to robot."""
        if not LED_SRV_AVAILABLE or self._led_set_client is None:
            return False, 'LED service not available'
        
        req = SetLedStatus.Request()
        # Assign using named fields (order-independent)
        req.fl_mode  = int(values['FL']['mode'])
        req.fl_color = int(values['FL']['color'])
        req.fr_mode  = int(values['FR']['mode'])
        req.fr_color = int(values['FR']['color'])
        req.bl_mode  = int(values['BL']['mode'])
        req.bl_color = int(values['BL']['color'])
        req.br_mode  = int(values['BR']['mode'])
        req.br_color = int(values['BR']['color'])
        
        future = self._led_set_client.call_async(req)
        
        # Poll with timeout
        elapsed, timeout, poll = 0.0, 8.0, 0.05
        while not future.done():
            if elapsed >= timeout:
                return False, 'set_led_status timed out'
            time.sleep(poll)
            elapsed += poll
        
        result = future.result()
        if result is None:
            return False, 'Service returned None'
        
        return result.success, 'OK' if result.success else 'Failed'
```

**Key Implementation Notes:**
- ✅ Uses bare service names (`'set_led_status'`, not `/mecanumbot/set_led_status`)
- ✅ Named field assignment (order-independent, robust)
- ✅ Proper timeout handling (8 seconds)
- ✅ Converts between service format and JSON format

### Robot-Side (Mapping Listener)

#### LED Action Execution

**File:** `robot/button_mapping_ros/button_mapping_ros/mapping_listener.py`

```python
class MappingListener(Node):
    def __init__(self):
        # Create LED service client
        if MECANUMBOT_SRV_AVAILABLE:
            self._led_set_client = self.create_client(
                SetLedStatus, 'set_led_status')

    def publish_action(self, action_name, trigger_control):
        """Execute action (called when button pressed)."""
        action_data = self.actions.get(action_name)
        for tup in action_data.get('tuples', []):
            topic        = tup[0]
            message_str  = tup[1]
            message_type = tup[2] if len(tup) >= 3 else 'std_msgs/msg/String'

            # Special handling for LED actions
            if topic == '__led__':
                self._execute_led_action(message_str, action_name)
            else:
                self._publish_single(topic, message_str, message_type)

    def _execute_led_action(self, cfg_json: str, action_name: str):
        """Parse LED JSON and call SetLedStatus service."""
        if self._led_set_client is None:
            self.get_logger().warn(
                f'[LED] Cannot execute "{action_name}" — client not available')
            return
        
        if not self._led_set_client.service_is_ready():
            self.get_logger().warn(
                f'[LED] Cannot execute "{action_name}" — service not ready')
            return
        
        # Parse JSON config
        try:
            cfg = json.loads(cfg_json)
        except (json.JSONDecodeError, TypeError) as e:
            self.get_logger().error(f'[LED] Failed to parse JSON: {e}')
            return

        # Build service request
        req = SetLedStatus.Request()
        corners = {
            'FL': ('fl_color', 'fl_mode'),
            'FR': ('fr_color', 'fr_mode'),
            'BL': ('bl_color', 'bl_mode'),
            'BR': ('br_color', 'br_mode'),
        }
        for corner, (color_field, mode_field) in corners.items():
            corner_data = cfg.get(corner, {})
            # Use 'or' guards for safety (default mode=4, color=0)
            setattr(req, color_field, int(corner_data.get('color') or 0))
            setattr(req, mode_field,  int(corner_data.get('mode')  or 4))

        # Call service asynchronously
        future = self._led_set_client.call_async(req)
        future.add_done_callback(
            lambda f: self._on_led_response(f, action_name))
        
        self.get_logger().info(
            f'  -> LED action "{action_name}" sent to set_led_status')

    def _on_led_response(self, future, action_name: str):
        """Handle LED service response (callback)."""
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f'  [LED] "{action_name}" applied OK')
            else:
                self.get_logger().warn(
                    f'  [LED] "{action_name}" service returned failure')
        except Exception as e:
            self.get_logger().error(f'  [LED] "{action_name}" call error: {e}')
```

**Key Implementation Notes:**
- ✅ Detects LED actions via `topic == '__led__'`
- ✅ Parses JSON safely with error handling
- ✅ Uses `or` guards: `corner_data.get('color') or 0` (prevents None/null)
- ✅ Named field assignment with `setattr()`
- ✅ Async service call with callback for logging
- ✅ Checks service readiness before calling

---

## 📊 Message Flow

### Complete Flow: Button Press → LED Update

```
┌──────────────┐
│ User presses │
│  button "A"  │
└──────┬───────┘
       │ Physical input
       ▼
┌────────────────────────┐
│  Xbox Controller       │
│  (USB dongle)          │
└──────┬─────────────────┘
       │ USB HID
       ▼
┌────────────────────────┐
│  joy_node (ROS2)       │
│  Publishes /joy topic  │
└──────┬─────────────────┘
       │ ROS2 Topic: /joy
       ▼
┌────────────────────────┐
│  xbox_controller_node  │
│  Translates to events  │
└──────┬─────────────────┘
       │ ROS2 Topic: /xbox_controller/button_events
       │ Message: {"button": "A", "event": "PRESSED", ...}
       ▼
┌──────────────────────────────────┐
│  MappingListener.xbox_button_    │
│  callback()                       │
│  - Parses JSON                    │
│  - Looks up mapping: A → "Lights" │
└──────┬───────────────────────────┘
       │
       ▼
┌──────────────────────────────────┐
│  MappingListener.publish_action() │
│  - Loads action "Lights"          │
│  - Sees topic == "__led__"        │
└──────┬───────────────────────────┘
       │
       ▼
┌──────────────────────────────────┐
│  MappingListener._execute_led_   │
│  action()                         │
│  - Parses JSON config             │
│  - Builds SetLedStatus.Request    │
└──────┬───────────────────────────┘
       │ ROS2 Service: /set_led_status
       │ Request: {fl_mode: 3, fl_color: 2, ...}
       ▼
┌──────────────────────────────────┐
│  LED Service Node                │
│  - Receives request               │
│  - Validates parameters           │
│  - Encodes serial protocol        │
└──────┬───────────────────────────┘
       │ UART (9600 baud)
       │ Protocol: [START][CMD][DATA...][CHECKSUM][END]
       ▼
┌──────────────────────────────────┐
│  Arduino Nano                    │
│  - Parses protocol                │
│  - Updates LED strip buffers      │
│  - Calls FastLED.show()           │
└──────┬───────────────────────────┘
       │ WS2812B Data Signal
       ▼
┌──────────────────────────────────┐
│  LED Strips (4 corners)          │
│  - Display new colors/modes       │
└──────────────────────────────────┘
```

**Timing:**
- Button press to LED update: **~100-200ms**
- Network latency (ROS2): ~10-30ms
- Service call overhead: ~20-50ms
- Serial communication: ~30-60ms
- LED update (FastLED): ~5-10ms

---

## 🐛 Troubleshooting

### Problem: LED Status Shows "Offline"

**Symptoms:**
- Dashboard LED panel shows "offline" or "No signal"
- Red status dot instead of green

**Diagnosis:**
```bash
# Check if LED service exists
ros2 service list | grep led_status

# Expected output:
# /get_led_status
# /set_led_status

# If not present, LED service node is not running
```

**Solutions:**

1. **Start LED Service Node:**
   ```bash
   # On robot
   ros2 run mecanumbot_led led_service
   ```

2. **Check Service Type:**
   ```bash
   ros2 service type /get_led_status
   # Should output: mecanumbot_msgs/srv/GetLedStatus
   ```

3. **Test Service Manually:**
   ```bash
   ros2 service call /get_led_status mecanumbot_msgs/srv/GetLedStatus
   ```

4. **Check Arduino Connection:**
   ```bash
   # Verify serial port
   ls /dev/ttyUSB* /dev/ttyACM*
   # Should see /dev/ttyUSB0 or /dev/ttyACM0
   
   # Check permissions
   sudo chmod 666 /dev/ttyUSB0
   ```

### Problem: LED Action Doesn't Execute

**Symptoms:**
- Button press doesn't change LEDs
- No error in logs
- Other actions work fine

**Diagnosis:**
```bash
# On robot, check mapping listener logs
ros2 run button_mapping_ros mapping_listener

# Press the button and look for:
# [INFO] [LED] Cannot execute "MyAction" — service not ready
# OR
# [INFO] -> LED action "MyAction" sent to set_led_status
```

**Solutions:**

1. **Verify Action Type:**
   - Go to "Create Actions" page
   - Check action type is "LED Lights" (not "Button Once")

2. **Check JSON Format:**
   ```bash
   # In host database
   sqlite3 ~/Documents/actions.db
   SELECT * FROM action_tuples WHERE action_id = <your_action_id>;
   # topic should be "__led__"
   # message should be valid JSON with FL/FR/BL/BR
   ```

3. **Re-sync Actions:**
   - Dashboard → "Sync Actions" button
   - Wait for sync complete message

4. **Check Service Ready:**
   ```bash
   ros2 service list
   # Verify /set_led_status exists
   ```

### Problem: Some Corners Don't Light Up

**Symptoms:**
- 1-3 corners work, others stay dark
- Specific corner always fails

**Diagnosis:**

1. **Test Hardware:**
   ```bash
   # Set all corners to white solid
   ros2 service call /set_led_status mecanumbot_msgs/srv/SetLedStatus \
     "{fl_mode: 4, fl_color: 1, \
       fr_mode: 4, fr_color: 1, \
       bl_mode: 4, bl_color: 1, \
       br_mode: 4, br_color: 1}"
   ```

2. **Check Wiring:**
   - Verify LED strip power connections
   - Check data line integrity
   - Test with multimeter (5V present)

3. **Arduino Serial Debug:**
   ```bash
   # Connect to Arduino serial monitor
   screen /dev/ttyUSB0 9600
   # Or use Arduino IDE Serial Monitor
   # Look for error messages
   ```

**Solutions:**

1. **Power Issue:**
   - Check 5V supply can handle all LEDs (~1A per strip)
   - Add capacitor (1000µF) across power supply
   - Reduce brightness in code

2. **Data Signal Issue:**
   - Check data pin connection
   - Add 330Ω resistor in series with data line
   - Ensure common ground between Arduino and LED strips

3. **Code Issue (Arduino):**
   ```cpp
   // Verify LED pin definitions
   #define LED_PIN_FL  6
   #define LED_PIN_FR  5
   #define LED_PIN_BL  4
   #define LED_PIN_BR  3
   
   // Check FastLED initialization
   FastLED.addLeds<WS2812B, LED_PIN_FL, GRB>(leds_fl, NUM_LEDS);
   ```

### Problem: Colors Wrong or Swapped

**Symptoms:**
- Request green, get red
- Colors don't match palette

**Diagnosis:**
- Check color index in action definition
- Verify Arduino color palette matches host

**Solution:**

Ensure color indices match between systems:

**Host (Python):**
```python
LED_COLOR_INFO = [
    { name: 'BLACK',  hex: '#1a1a1a' },  # 0
    { name: 'WHITE',  hex: '#f0f0f0' },  # 1
    { name: 'GREEN',  hex: '#22c55e' },  # 2
    { name: 'RED',    hex: '#ef4444' },  # 3
    # ...
]
```

**Arduino (C++):**
```cpp
const CRGB COLORS[8] = {
    CRGB(26,26,26),      // 0: Black
    CRGB(240,240,240),   // 1: White
    CRGB(34,197,94),     // 2: Green
    CRGB(239,68,68),     // 3: Red
    // ...
};
```

### Problem: Animation Stuttering

**Symptoms:**
- Wave mode not smooth
- Pulse mode jerky
- Random freezes

**Diagnosis:**
```bash
# Check CPU usage on robot
htop
# Look for high CPU processes

# Check ROS2 topic rates
ros2 topic hz /set_led_status
```

**Solutions:**

1. **Reduce Update Rate:**
   - LED actions shouldn't be called faster than 10 Hz
   - Use "Button Once" instead of "Button Hold" for LED actions

2. **Arduino Optimization:**
   ```cpp
   // Use FastLED's built-in frame rate limiter
   FastLED.setMaxRefreshRate(60);  // 60 fps cap
   ```

3. **Reduce Animation Complexity:**
   ```cpp
   // Simplify wave calculation
   // Use lookup tables instead of sin/cos
   ```

---

## 🚀 Advanced Usage

### Creating Complex LED Patterns

#### Example: Police Lights

```json
{
  "FL": {"mode": 3, "color": 4},   // Blue pulse
  "FR": {"mode": 3, "color": 3},   // Red pulse
  "BL": {"mode": 3, "color": 4},   // Blue pulse  
  "BR": {"mode": 3, "color": 3}    // Red pulse
}
```

#### Example: Rainbow Chase

Use Wave mode with different colors per corner:

```json
{
  "FL": {"mode": 1, "color": 2},   // Green wave →
  "FR": {"mode": 1, "color": 7},   // Yellow wave →
  "BL": {"mode": 1, "color": 4},   // Blue wave →
  "BR": {"mode": 1, "color": 6}    // Pink wave →
}
```

### Combining with Movement Actions

Create actions that combine LED changes with movement:

1. **Create LED action:** "Forward Lights" (green solid)
2. **Create movement action:** "Drive Forward" (/cmd_vel)
3. **Button mapping strategy:**
   - Hold "A": Trigger movement (hold mode)
   - Press "A": Trigger LED (once mode)
   - **Problem:** Can only have one action per button

**Workaround:** Use sequential actions
- Create single action with 2 tuples:
  1. Tuple 1: `/cmd_vel` (movement)
  2. Tuple 2: `__led__` (lights)

### Custom Color Palette

To add new colors, modify both systems:

**Arduino:**
```cpp
// In LED controller code
const CRGB COLORS[9] = {  // Extend array
    // ... existing colors ...
    CRGB(255, 128, 0)     // 8: Orange
};
```

**Host:**
```javascript
// In button_mapping.html
const LED_COLORS = [
    // ... existing colors ...
    { name: 'Orange', hex: '#ff8000' }  // Index 8
];
```

**Dashboard:**
```javascript
// In main_dashboard.html
const LED_COLOR_INFO = [
    // ... existing colors ...
    { name: 'ORANGE', hex: '#ff8000' }
];
```

### Performance Tuning

#### Optimize Dashboard Polling

```javascript
// Reduce polling frequency for battery savings
setInterval(updateLedStatus, 10000);  // 10 seconds instead of 5
```

#### Batch LED Updates

Instead of calling service multiple times, update all corners at once:

```python
# Good: Single service call
req.fl_mode = 4; req.fl_color = 2
req.fr_mode = 4; req.fr_color = 2
req.bl_mode = 4; req.bl_color = 2
req.br_mode = 4; req.br_color = 2
led_client.call_async(req)

# Bad: Four separate calls (slower, more overhead)
# Don't do this!
```

---

## 📚 References

### ROS2 Documentation
- [ROS2 Services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
- [Custom Interfaces](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)

### Hardware Documentation
- [WS2812B Datasheet](https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf)
- [FastLED Library](https://fastled.io/)

### Project Files
- Service Definitions: `shared/mecanumbot_msgs/srv/`
- Host Code: `host/app.py`
- Robot Code: `robot/button_mapping_ros/button_mapping_ros/mapping_listener.py`
- Frontend: `host/templates/main_dashboard.html`, `host/templates/button_mapping.html`

---

**Last Updated:** 2026. április 5.  
**Version:** 1.0.0  
**Author:** Adorján
