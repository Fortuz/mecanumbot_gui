# MecanumBot GUI - Controller Mapping System

A comprehensive Flask web application for configuring Xbox 360  mappings and publishing them to ROS2 topics. This system bridges web-based configuration with ROS2 robot control through an intuitive dark-themed interface.

**Platform Support:** Linux (Ubuntu) and Windows with automated startup scripts.

## 🎮 System Overview

This application provides a complete pipeline for controller-based robot control:

1. **Web GUI** (Flask in Docker) - Configure actions and map controller inputs
2. **ROS2 Integration** - Publish configurations to ROS2 topics
3. **joy_node** - Reads PS4/Xbox controller hardware
4. **xbox_controller_node** - Converts joy messages to Xbox-compatible events
5. **mapping_listener** - Bridges controller events to robot action topics
6. **Robot Control** - The robot nodes receive action commands

## 🚀 Quick Start

### Prerequisites
- **Install Requirements** 
- **Docker** installed and running
- **ROS2 Humble** installed and sourced
- **Xbox 360 controller** connected via USB 
- **Rules for Xbox 360 controller** are placed 


### Automated Startup (Recommended)

#### Linux/Ubuntu:
```bash
./start_system.sh
```

#### Windows (PowerShell):
```powershell
.\start_system.ps1
```

The startup script will automatically:
1. Check if controller is connected(needs .rules fo Linux)
2. Start joy_node (reads controller hardware)
3. Start xbox_controller_node (publishes controller events)
4. Start mapping_listener (publishes actions based on what button was pressed)
5. Start Docker GUI (web interface on port 8080)

Press `Ctrl+C` to stop all processes cleanly.

#### Access the GUI:
Open browser to **http://localhost:8080**



## Controller Permissions (Linux Only)

### udev Rules for Controller Access

To allow non-root users to access the Xbox 360, you need to install the provided udev rules file.

#### Installation Steps:

1. **Copy the rules file to the udev rules directory:**
   ```bash
   sudo cp controller_rules/99-xbox360-wireless.rules /etc/udev/rules.d/
   ```

2. **Set appropriate permissions:**
   ```bash
   sudo chmod 644 /etc/udev/rules.d/99-xbox360-wireless.rules
   ```

3. **Reload udev rules:**
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

4. **Add your user to the plugdev group (if not already a member):**
   ```bash
   sudo usermod -a -G plugdev $USER
   ```

5. **Log out and log back in** for group changes to take effect.

#### Verification:

After installation, verify the controller is accessible:

```bash
# Check if controller is detected
ls -la /dev/input/js0

# Should show permissions like: crw-rw-rw- (readable/writable by all)
```


**Note:** The provided rules file works for Xbox 360 Wireless Adapter. If you're using a different controller, you may need to adjust the vendor/product IDs in the rules file.

## �🎯 Features


### Action Types
1. **Button Actions**: Triggered by single button press
   - Example: A button → Forward motion
   
2. **Joystick Actions**: Continuous values with X, Y placeholders
   - Example: Left Stick → Movement with X/Y velocity

### ROS2 Integration

- **Controller Events**: Xbox node publishes to `/xbox_controller/*` topics
- **Action Publishing**: Mapping node publishes to user-defined topics
- **Message Format**: All use `std_msgs/String` with JSON payloads

## 📁 Project Structure

```
mecanumbot_gui/
├── app.py                      # Flask application with ROS2 publishing
├── Dockerfile                  # Container configuration
├── requirements.txt            # Python dependencies
├── templates/                  # Jinja2 templates
│   ├── landing.html           # Entry page (user name)
│   ├── dashboard.html         # Main interface with tabs
│   ├── map_controls.html      # Unified mapping page
│   ├── button_mapping.html    # Action configuration
│   └── success.html           # Confirmation page
├── static/                     # CSS and assets
├── xbox_controller_pkg/        # ROS2 controller input package
└── button_mapping_ros/         # ROS2 mapping bridge package
```
### Runtime Execution

When you press the A button:
1. joy_node detects button press
2. xbox_controller_node publishes to `/xbox_controller/button_events`
3. mapping_listener receives event, looks up "A" in database→ "Forward"
4. mapping_listener publishes to `/cmd_vel` and `/robot/status` for example


