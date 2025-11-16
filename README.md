
How to start:
  start docker desktop.
  
  docker build -t mecanumbot-gui .

  docker run -d --name mecanumbot-gui -p 8080:8080 -v "$HOME\Documents:/host_docs" mecanumbot-gui

  runs on http://localhost:8080
How stop it:
  docker stop mecanumbot-gui
  or
  docker stop mecanumbot-gui; docker rm mecanumbot-gui; docker run -d --name mecanumbot-gui -p 8080:8080 -v "$HOME\Documents:/host_docs" mecanumbot-gui
restart:
  docker start mecanumbot-gui












# Controller Configuration App

A Flask web application that automatically detects different types of game controllers and provides appropriate button mapping interfaces.

## 🚀 Quick Start

### Prerequisites
- Python 3.8+ installed
- Docker Desktop (for containerized deployment)
- Game controller connected to PC

### Method 1: Easy Setup (Recommended)

#### For Windows
```powershell
# 1. Navigate to project directory
cd "C:\Users\YourName\Desktop\docker_teszt"

# 2. Run the automated setup script
.\run_with_bridge.ps1
```

#### For Linux/Unix/macOS
```bash
# 1. Navigate to project directory
cd ~/Desktop/docker_teszt

# 2. Make script executable and run
chmod +x run_with_bridge.sh
./run_with_bridge.sh
```

This script will:
- Start the controller bridge service on your host system
- Build the Docker image
- Run the Docker container with proper networking
- Set up the complete system automatically
- **Linux**: Automatically handles cleanup when you press Ctrl+C

### Method 2: Manual Setup

#### Step 1: Install Dependencies
```powershell
# Install Python packages
pip install flask pygame requests
```

#### Step 2: Start Controller Bridge Service
```powershell
# Start the bridge service (keep this terminal open)
python controller_bridge.py
```

You should see output like:
```
Controller Bridge Service
=========================
✓ Controller: PS4 Controller  # (or Xbox 360, etc.)
✓ Buttons available: 27
Controller Bridge API running on http://localhost:8899
```

#### Step 3: Build and Run Docker Container
```powershell
# In a new terminal, build Docker image
docker build -t controller-app .

# Run the container with bridge communication
docker run -p 8080:8080 --add-host=host.docker.internal:host-gateway controller-app
```

#### Step 4: Access the Application
Open your web browser and go to: **http://localhost:8080**

### Method 3: Direct Python Execution (Development)
```powershell
# For development/testing without Docker
python app.py
```

Access at: **http://localhost:8080**

## ✅ System Requirements

### Hardware
- **Windows 10/11** or **Linux** (Ubuntu 20.04+, other distributions should work)
- **Game Controller**: Xbox 360, PS4 (DualShock 4), or any HID-compatible controller
- **USB Port** or **Bluetooth** for controller connection

### Software
- **Python 3.8+** with pip
- **Docker Desktop** (for containerized deployment)
- **Web Browser** (Chrome, Firefox, Edge, etc.)

### Network
- **Port 8080**: Flask web application
- **Port 8899**: Controller bridge service (host-to-Docker communication)

## 🎮 Controller Support

The application automatically detects and configures different controller types:

### Fully Supported Controllers
- **Xbox 360 Controller**: Shows familiar Xbox button names (A, B, X, Y, LB, RB, LT, RT, Start, Back, sticks, D-pad)
- **PS4 Controller (DualShock 4)**: Shows PlayStation button names (Cross, Circle, Square, Triangle, L1, R1, L2, R2, Share, Options, PS, L3, R3, sticks, D-pad, Touchpad)

### Other Controllers
- **Dynamic Detection**: Any HID-compatible controller gets generic button names (Button_0, Button_1, Axis_0_Positive, etc.)
- **Full Mapping**: All detected buttons, axes, and hats are available for action mapping

### Automatic Controller Detection
When the app starts, it automatically detects what type of controller is plugged into the PC:

- **Xbox 360 Controller**: Uses predefined button layout (A, B, X, Y, LB, RB, LT, RT, Start, Back, stick clicks, analog stick directions, D-pad directions)
- **PS4 Controller (DualShock 4)**: Uses predefined PS4 button layout (Cross, Circle, Square, Triangle, L1, R1, L2, R2, Share, Options, PS, L3, R3, stick directions, D-pad directions, Touchpad)
- **Other Controllers**: Dynamically detects all available buttons, axes, and hats/D-pads and creates mappable inputs

### Dynamic Button Mapping
- If Xbox 360 controller detected: Shows familiar Xbox button names
- If PS4 controller detected: Shows PlayStation button names (Cross, Circle, Square, Triangle, etc.)
- If other controller detected: Shows generic button names (Button_0, Button_1, Axis_0_Positive, etc.)
- Re-detection available: Users can re-detect controllers if they plug in a different one

### Controller Status Display
- Main page shows current controller status
- Configuration page shows detected controller type and number of available inputs
- Dynamic controller images: Shows Xbox 360 or PS4 controller image based on detection
- Fallback to Xbox 360 layout if no controller detected

## 📖 Usage Guide

### Basic Workflow
1. **Connect Controller**: Plug in Xbox 360, PS4, or other controller
2. **Start Bridge Service**: Run `python controller_bridge.py` (keep running)
3. **Start Application**: Run `.\run_with_bridge.ps1` or manually start Docker
4. **Open Web Interface**: Navigate to http://localhost:8080
5. **Enter User Info**: Fill in user and volunteer names
6. **Configure Mappings**: Map controller buttons to game actions
7. **Save Settings**: Mappings are saved to CSV or SQLite

### Button Mapping Process
1. **Detection**: App shows detected controller type and available buttons
2. **Action Selection**: For each button, select an action from dropdown
3. **Visual Feedback**: See controller image (Xbox or PS4) and button names
4. **Save Mappings**: Click "Save mappings" to store configuration
5. **Re-detection**: Use "Re-detect Controller" if you change controllers

## Usage

1. **Connect your controller** to the PC before starting the app
2. **Start the application**: `python app.py`
3. **Check controller detection** on the main page
4. **Configure button mappings** for your specific controller
5. **Re-detect** if you switch controllers

## Files

- `app.py` - Main Flask application with controller detection
- `test_controller.py` - Test script to verify controller detection
- `requirements.txt` - Python dependencies (Flask, pygame)
- `Dockerfile` - Container setup with SDL2 libraries for controller support

## Installation

### Local Development
```bash
pip install -r requirements.txt
python app.py
```

### Docker with Controller Bridge (Recommended for Docker users)

**✅ This solution enables full controller detection in Docker!**

The bridge service runs on your host system and communicates controller information to the Docker container via HTTP API.

#### Automatic Setup (Windows)
```powershell
# Run everything automatically
.\run_with_bridge.ps1
```

#### Automatic Setup (Linux/Unix)
```bash
# Make executable and run everything automatically
chmod +x run_with_bridge.sh
./run_with_bridge.sh
```

#### Manual Setup (Windows)
```powershell
# 1. Start the bridge service on host (keep this running)
python controller_bridge.py

# 2. In another terminal, build and run Docker
docker build -t controller-app .
docker run -p 8080:8080 --add-host=host.docker.internal:host-gateway controller-app
```

#### Manual Setup (Linux/Unix)
```bash
# 1. Start the bridge service on host (keep this running)
python3 controller_bridge.py

# 2. In another terminal, build and run Docker
docker build -t controller-app .
docker run -p 8080:8080 --network="host" controller-app
```
```

#### How it works:
1. **Bridge Service** (`controller_bridge.py`) runs on host system
2. **Detects controllers** using pygame with full hardware access
3. **Exposes HTTP API** on `http://localhost:8899`
4. **Docker container** requests controller info from bridge service
5. **Automatic fallback** to direct detection if bridge unavailable

#### Bridge API Endpoints:
- `http://localhost:8899/controller-info` - Get current controller status
- `http://localhost:8899/detect` - Force controller re-detection  
- `http://localhost:8899/status` - Service health check

#### Testing the Bridge:
```powershell
# Test bridge service is working
python test_bridge.py
```

## 🏭 Production Deployment

### For Production Use
- **Recommended**: Run Flask app directly on host system for full controller access
- **Docker**: Use bridge service for containerized deployments
- **Security**: Configure proper firewall rules for ports 8080 and 8899
- **SSL**: Add HTTPS support for production web interface
- **Database**: Consider switching to SQLite for better data management

### Environment Variables
```powershell
# Optional configuration
$env:USE_SQLITE = "True"           # Use SQLite instead of CSV
$env:BASE_DIR = "C:\ProgramData"   # Custom data directory
```

## 📝 Development


### Code Organization
- **`app.py`**: Main Flask web application with route handling
- **`controller_bridge.py`**: Host-side service for hardware controller access
- **HTML Templates**: Embedded in app.py for single-file deployment
- **Static Assets**: Controller images in `static/` folder
- **Configuration**: CSV/SQLite storage options

**Solutions:**
- Ensure controller is connected and drivers are installed
- Try disconnecting and reconnecting the controller
- Use the "Re-detect Controller" button in the web interface
- Restart the bridge service: Stop with Ctrl+C, then run `python controller_bridge.py` again
- **Linux**: Ensure user has permission to access input devices:
  ```bash
  # Add user to input group (may require logout/login)
  sudo usermod -a -G input $USER
  
  # Alternative: Run with elevated permissions (not recommended for production)
  sudo python3 controller_bridge.py
  ```

### Bridge Service Issues

#### Windows
```powershell
# Test if bridge service is running
python test_bridge.py

# Check if bridge service port is open
netstat -an | findstr :8899

# Manual bridge service check (if running)
curl http://localhost:8899/status
```

#### Linux/Unix
```bash
# Test if bridge service is running
python3 test_bridge.py

# Check if bridge service port is open
netstat -an | grep :8899
# OR use ss command
ss -tlnp | grep :8899

# Manual bridge service check (if running)
curl http://localhost:8899/status

# Check bridge service logs (if using automated script)
tail -f bridge_service.log

# Stop bridge service manually (if needed)
pkill -f controller_bridge.py
```

**Solutions:**
- **Bridge not found**: Ensure `python controller_bridge.py` is running first
- **Connection refused**: Check Windows Firewall settings, allow Python through firewall
- **Port conflict**: Make sure no other service is using port 8899

### Docker Communication Issues
**Symptoms:** Docker container shows "Could not connect to bridge service"

**Solutions:**
- Ensure bridge service is running BEFORE starting Docker container
- Use the correct Docker run command: `docker run -p 8080:8080 --add-host=host.docker.internal:host-gateway controller-app`
- Check Docker Desktop is running and Linux containers are enabled
- Try restarting Docker Desktop

### Web Interface Issues
**Symptoms:** Shows generic "Button_0" instead of proper button names

**Solutions:**
- Stop all Python processes: `Get-Process python | Stop-Process -Force`
- Restart bridge service: `python controller_bridge.py`
- Refresh the web page or click "Re-detect Controller"
- Ensure controller is connected before starting bridge service

### Performance Issues
**Solutions:**
- **High CPU**: Normal during controller detection, should settle after startup
- **Memory Usage**: Restart services if memory usage grows over time
- **Slow Response**: Check if antivirus is scanning Python processes

## 📁 Project Structure

```
docker_teszt/
├── app.py                          # Main Flask application
├── controller_bridge.py            # Host-side controller detection service  
├── requirements.txt               # Python dependencies
├── Dockerfile                     # Container configuration
├── run_with_bridge.ps1           # Automated Windows setup script
├── run_with_bridge.sh            # Automated Linux/Unix setup script
├── templates/                     # HTML templates for web interface
│   ├── index.html                # Main form page
│   ├── configure.html            # Controller configuration page
│   └── success.html              # Success confirmation page
├── static/
│   ├── xbox-controller.svg.png   # Xbox 360 controller image
│   └── ps4_controller.jpeg       # PS4 controller image
├── test_bridge.py                # Bridge service testing script
├── test_controller.py            # Controller detection testing script
├── test_ps4_detection.py         # PS4 controller specific tests
├── test_ps4_button_logic.py      # PS4 button mapping validation
└── README.md                      # This file
```

## 🔄 How It Works

### Architecture Overview
```
┌─────────────────────────┐    HTTP API    ┌─────────────────────────┐
│    Docker Container     │ ◄────────────► │     Host System         │
│                         │                │                         │
│  ┌─────────────────┐   │                │  ┌─────────────────┐    │
│  │     app.py      │   │                │  │ controller_     │    │
│  │  (Flask Web App)│   │                │  │ bridge.py       │    │
│  └─────────────────┘   │                │  └─────────────────┘    │
│                         │                │         │               │
└─────────────────────────┘                │         ▼               │
                                           │  ┌─────────────────┐    │
                                           │  │ 🎮 Controller   │    │
                                           │  │    Hardware     │    │
                                           │  └─────────────────┘    │
                                           └─────────────────────────┘
```

### Communication Flow
1. **Controller Detection**: Bridge service detects connected controller using pygame
2. **Type Recognition**: Identifies Xbox 360, PS4, or uses dynamic detection
3. **Button Mapping**: Creates appropriate button layout (Xbox names, PS4 names, or generic)
4. **API Exposure**: Bridge exposes controller info via HTTP API on port 8899
5. **Docker Communication**: Docker container requests controller info from bridge
6. **Web Interface**: Flask app displays controller-specific interface with proper button names
7. **Action Mapping**: Users map controller buttons to game actions
8. **Data Persistence**: Mappings saved to CSV or SQLite

## 🎯 ROS2 Button Mapping System

The application includes a comprehensive ROS2 button mapping system that allows you to map controller buttons to custom ROS2 topics with configurable parameters.

### Features
- **Web-based Configuration**: Configure button mappings through an intuitive web interface
- **Dynamic Topic Publishing**: Map any button to any ROS2 topic with custom parameters
- **Real-time Updates**: Changes take effect immediately
- **JSON Parameters**: Full flexibility with JSON-formatted parameters
- **Visual Validation**: JSON syntax validation in the browser

### Components

#### 1. Web Interface
- **URL**: `http://localhost:8080/button-mapping`
- Configure each button with:
  - Target ROS2 topic (e.g., `/robot/move`)
  - Parameters as JSON (e.g., `{"direction": "forward", "speed": 1.0}`)
- Visual interface with controller-specific button names
- Real-time JSON validation

#### 2. Button Mapper Package (ROS2)
The `button_mapper_pkg` is a ROS2 package that:
- Subscribes to button mapping configuration (`/button_mapper/config`)
- Subscribes to Xbox controller button events (`/xbox_controller/button_events`)
- Publishes to user-configured topics when buttons are pressed

#### 3. Controller Bridge
The bridge service handles:
- Controller detection on host system
- Communication between Docker and ROS2
- Publishing button mapping configuration to ROS2 topics

### Quick Start with Button Mapping

#### Step 1: Install the Button Mapper Package
```bash
cd button_mapper_pkg
./install.sh
source ../install/setup.bash
```

#### Step 2: Start All Services
```bash
# Use the quick start script
./quick_start.sh

# Or manually start each component:

# Terminal 1: Controller Bridge (host)
python3 controller_bridge.py

# Terminal 2: Joy Node (ROS2)
ros2 run joy joy_node

# Terminal 3: Xbox Controller Node (ROS2)
ros2 run xbox_controller_pkg xbox_controller_node

# Terminal 4: Button Mapper Node (ROS2)
ros2 run button_mapper_pkg button_mapper_node
```

#### Step 3: Configure Mappings
1. Open web browser: `http://localhost:8080/button-mapping`
2. Configure buttons with topics and parameters
3. Click "Save Mappings"

#### Step 4: Test the Mappings
Press buttons on your controller and observe the published messages:
```bash
# Monitor a specific topic
ros2 topic echo /robot/move

# List all active topics
ros2 topic list
```

### Example Button Mappings

The system includes example configurations in `example_mappings.py`:

```python
# View example mappings
python example_mappings.py print

# Save examples to file
python example_mappings.py save

# Publish examples directly to ROS2
python example_mappings.py publish
```

Example mappings include:
- **Movement**: Forward, backward, strafe left/right
- **Rotation**: Turn left/right
- **System**: Start, stop commands
- **LED Control**: Patterns, colors, brightness
- **Camera**: Pan and tilt
- **Arm Control**: Joint movements
- **Gripper**: Open/close

### ROS2 Topics

#### Subscribed Topics
- `/button_mapper/config` (std_msgs/String): Button mapping configuration
- `/xbox_controller/button_events` (std_msgs/String): Button press events

#### Published Topics
Dynamic topics based on your configuration. Examples:
- `/robot/move` - Movement commands
- `/robot/rotation` - Rotation commands
- `/robot/led` - LED control
- `/robot/camera` - Camera control
- `/robot/arm` - Arm control
- `/robot/gripper` - Gripper control

### Documentation

For detailed information about the button mapping system, see:
- **[BUTTON_MAPPING_GUIDE.md](BUTTON_MAPPING_GUIDE.md)** - Complete system documentation
- **[button_mapper_pkg/README.md](button_mapper_pkg/README.md)** - ROS2 package details

### Troubleshooting

**Configuration not updating?**
- Check that `controller_bridge.py` is running
- Verify ROS2 environment is sourced
- Monitor topic: `ros2 topic echo /button_mapper/config`

**Button events not received?**
- Ensure `xbox_controller_node` is running
- Check `joy_node` is publishing to `/joy`
- Monitor: `ros2 topic echo /xbox_controller/button_events`

**Messages not publishing?**
- Check `button_mapper_node` logs for errors
- Verify button names match configuration
- Use `ros2 topic list` to see all topics

# mecanumbot_gui
General GUI to observe and control the Mecanumbot

## Bringup

```
docker build -t text-saver .
docker run --rm -p 8080:8080 -v "$HOME\Documents:/host_docs" text-saver
```
