# Xbox Controller Package

A ROS 2 package for Xbox 360 controller integration with button event publishing and controller state monitoring.

## Package Overview

This package provides a complete ROS 2 integration for Xbox 360 wireless controllers, allowing you to capture button press events, joystick movements, and trigger inputs for robot control applications.

## Features

- ✅ **Real-time button detection** - Press/release events for all buttons
- 🕹️ **Analog stick monitoring** - Left/right stick position tracking  
- 🎯 **Trigger support** - Left/right trigger pressure detection
- ⬆️ **D-pad input** - Directional pad state monitoring
- 📊 **State publishing** - Complete controller state at 10Hz
- 🔧 **Configurable mapping** - Customizable button and axis names
- 🚀 **Easy integration** - Simple JSON message format
- 📋 **Hardware setup** - Included udev rules for Linux

## Package Structure

```
xbox_controller_pkg/
├── package.xml                          # Package manifest
├── setup.py                            # Python package setup
├── setup.cfg                           # Setup configuration
├── resource/                           
│   └── xbox_controller_pkg             # Package marker
├── xbox_controller_pkg/                # Python source code
│   ├── __init__.py                     # Package initialization
│   ├── xbox_controller_node.py         # Main controller node
│   ├── test_controller.py              # Test/verification node
│   └── controller_diagnostics.py       # Diagnostic utilities
├── launch/                             # Launch files
│   └── xbox_controller_launch.py       # Main launch file
├── config/                             # Configuration files
│   └── xbox_controller_config.yaml     # Controller settings
├── udev/                               # Hardware setup
│   └── 99-xbox360-wireless.rules       # Linux udev rules
└── test/                               # Unit tests
```

## Hardware Requirements

- Xbox 360 Wireless Controller
- Xbox 360 Wireless Adapter (USB ID: 045e:0719)

## Software Dependencies

- ROS 2 (Humble or later)
- `ros-humble-joy` package
- Python 3.8+

## Installation

### 1. Install ROS 2 Dependencies

```bash
sudo apt update
sudo apt install ros-humble-joy
```

### 2. Build the Package

```bash
# Navigate to your ROS 2 workspace
cd ~/ros2_ws/src

# Copy the package (or clone from repository)
cp -r /path/to/xbox_controller_pkg .

# Build the package
cd ~/ros2_ws
colcon build --packages-select xbox_controller_pkg

# Source the workspace
source install/setup.bash
```

### 3. Hardware Setup (Linux)

```bash
# Copy udev rules for controller permissions
sudo cp ~/ros2_ws/src/xbox_controller_pkg/udev/99-xbox360-wireless.rules /etc/udev/rules.d/

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add user to plugdev group
sudo usermod -a -G plugdev $USER

# Log out and back in for group changes to take effect
```

### 4. Verify Installation

```bash
# Check if controller is detected
lsusb | grep 045e:0719

# Should show: Bus 001 Device 003: ID 045e:0719 Microsoft Corp. Xbox 360 Wireless Adapter

# Check joystick device
ls /dev/input/js*

# Should show: /dev/input/js0 (or similar)
```

## Usage

### Quick Start

```bash
# Source ROS 2 workspace
source ~/ros2_ws/install/setup.bash

# Launch the controller system
ros2 launch xbox_controller_pkg xbox_controller_launch.py

# In another terminal, test the controller
ros2 run xbox_controller_pkg test_controller
```

### Individual Nodes

**Controller Node:**
```bash
ros2 run xbox_controller_pkg xbox_controller_node
```

**Test Node:**
```bash
ros2 run xbox_controller_pkg test_controller
```

**Diagnostics:**
```bash
ros2 run xbox_controller_pkg controller_diagnostics
```

## Topics

### Published Topics

#### `/xbox_controller/button_events`
- **Type:** `std_msgs/String` (JSON)
- **Description:** Button press/release events and axis changes
- **Rate:** Event-driven

**Button Event Example:**
```json
{
  "type": "BUTTON",
  "event": "PRESSED",
  "button_name": "A",
  "button_id": 0,
  "timestamp": 1699564800
}
```

**Axis Event Example:**
```json
{
  "type": "AXIS",
  "event": "CHANGED", 
  "axis_name": "LEFT_STICK_X",
  "axis_id": 0,
  "value": 0.745,
  "timestamp": 1699564800
}
```

#### `/xbox_controller/state`
- **Type:** `std_msgs/String` (JSON)
- **Description:** Complete controller state
- **Rate:** 10 Hz

**State Example:**
```json
{
  "type": "STATE",
  "buttons": {
    "A": false, "B": true, "X": false, "Y": false,
    "LB": false, "RB": false, "BACK": false, "START": false,
    "XBOX": false, "LS": false, "RS": false
  },
  "axes": {
    "LEFT_STICK_X": 0.123, "LEFT_STICK_Y": -0.456,
    "LEFT_TRIGGER": -1.000, "RIGHT_STICK_X": 0.000,
    "RIGHT_STICK_Y": 0.000, "RIGHT_TRIGGER": -1.000,
    "DPAD_X": 0.000, "DPAD_Y": 0.000
  },
  "timestamp": 1699564800
}
```

### Subscribed Topics

#### `/joy`
- **Type:** `sensor_msgs/Joy`
- **Description:** Raw joystick input from joy_node
- **Source:** `ros2 run joy joy_node`

## Controller Mapping

### Buttons
| Button ID | Name | Description |
|-----------|------|-------------|
| 0 | A | Bottom face button |
| 1 | B | Right face button |
| 2 | X | Left face button |
| 3 | Y | Top face button |
| 4 | LB | Left bumper |
| 5 | RB | Right bumper |
| 6 | BACK | Back button |
| 7 | START | Start button |
| 8 | XBOX | Xbox center button |
| 9 | LS | Left stick button |
| 10 | RS | Right stick button |

### Axes
| Axis ID | Name | Range | Description |
|---------|------|-------|-------------|
| 0 | LEFT_STICK_X | -1.0 to 1.0 | Left stick horizontal |
| 1 | LEFT_STICK_Y | -1.0 to 1.0 | Left stick vertical |
| 2 | LEFT_TRIGGER | -1.0 to 1.0 | Left trigger pressure |
| 3 | RIGHT_STICK_X | -1.0 to 1.0 | Right stick horizontal |
| 4 | RIGHT_STICK_Y | -1.0 to 1.0 | Right stick vertical |
| 5 | RIGHT_TRIGGER | -1.0 to 1.0 | Right trigger pressure |
| 6 | DPAD_X | -1.0 to 1.0 | D-pad horizontal |
| 7 | DPAD_Y | -1.0 to 1.0 | D-pad vertical |

## Integration Examples

### Basic Button Handling

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class MyRobotController(Node):
    def __init__(self):
        super().__init__('my_robot_controller')
        
        self.subscription = self.create_subscription(
            String,
            '/xbox_controller/button_events',
            self.controller_callback,
            10
        )
    
    def controller_callback(self, msg):
        data = json.loads(msg.data)
        
        if data['type'] == 'BUTTON' and data['event'] == 'PRESSED':
            button = data['button_name']
            
            if button == 'A':
                self.get_logger().info('Button A pressed - Moving forward!')
            elif button == 'B':
                self.get_logger().info('Button B pressed - Moving backward!')
            elif button == 'X':
                self.get_logger().info('Button X pressed - Turning left!')
            elif button == 'Y':
                self.get_logger().info('Button Y pressed - Turning right!')
```

### Mecanum Drive Control

```python
class MecanumDriveController(Node):
    def __init__(self):
        super().__init__('mecanum_drive_controller')
        
        self.subscription = self.create_subscription(
            String,
            '/xbox_controller/state',
            self.state_callback,
            10
        )
    
    def state_callback(self, msg):
        data = json.loads(msg.data)
        
        if data['type'] == 'STATE':
            # Get joystick values
            left_x = data['axes']['LEFT_STICK_X']      # Strafe
            left_y = data['axes']['LEFT_STICK_Y']      # Forward/Back
            right_x = data['axes']['RIGHT_STICK_X']    # Rotate
            
            # Calculate mecanum wheel speeds
            front_left = left_y + left_x + right_x
            front_right = left_y - left_x - right_x
            rear_left = left_y - left_x + right_x
            rear_right = left_y + left_x - right_x
            
            # Send to motor controllers
            self.send_wheel_speeds(front_left, front_right, rear_left, rear_right)
```

## Configuration

Edit `config/xbox_controller_config.yaml` to customize:

```yaml
xbox_controller:
  ros__parameters:
    # Adjust sensitivity
    axis_threshold: 0.1
    
    # Change publishing rate
    state_publish_rate: 10.0
    
    # Customize button names
    button_names:
      0: 'JUMP'     # Rename A button
      1: 'SHOOT'    # Rename B button
```

## Troubleshooting

### Controller Not Detected

1. **Check USB connection:**
   ```bash
   ros2 run xbox_controller_pkg controller_diagnostics
   ```

2. **Verify permissions:**
   ```bash
   ls -la /dev/input/js0
   groups $USER  # Should include 'plugdev'
   ```

3. **Test raw input:**
   ```bash
   ros2 topic echo /joy
   ```

### Common Issues

- **Permission denied:** Install udev rules and add user to `plugdev` group
- **No joystick device:** Check if adapter is creating `/dev/input/js*` devices
- **Button mapping wrong:** Different controllers may have different mappings

## Development

### Adding Custom Button Mappings

1. Edit the `button_names` dictionary in `xbox_controller_node.py`
2. Update configuration in `config/xbox_controller_config.yaml`
3. Rebuild the package

### Creating Custom Nodes

Use the existing nodes as templates for creating application-specific controllers.

## License

This package is licensed under MIT License.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Submit a pull request

## Support

For issues and questions:
- Check the troubleshooting section
- Run diagnostics: `ros2 run xbox_controller_pkg controller_diagnostics`
- Create an issue on the repository

---

**Package Version:** 1.0.0  
**ROS 2 Version:** Humble+  
**Maintainer:** Your Name <your.email@example.com>