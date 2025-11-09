#!/bin/bash
# Xbox Controller Package Installation Script
# This script helps set up the Xbox 360 controller ROS 2 package

set -e

echo "================================================="
echo "Xbox Controller Package Installation Script"
echo "================================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running on Linux
if [[ "$OSTYPE" != "linux-gnu"* ]]; then
    print_error "This script is designed for Linux systems only."
    exit 1
fi

# Check if ROS 2 is installed
if ! command -v ros2 &> /dev/null; then
    print_error "ROS 2 is not installed or not in PATH."
    print_error "Please install ROS 2 first: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

print_status "ROS 2 installation found."

# Check for joy package
print_status "Checking for ros-humble-joy package..."
if ! dpkg -l | grep -q ros-humble-joy; then
    print_warning "ros-humble-joy not found. Installing..."
    sudo apt update
    sudo apt install -y ros-humble-joy
else
    print_status "ros-humble-joy is already installed."
fi

# Install udev rules
print_status "Installing udev rules..."
if [ -f "udev/99-xbox360-wireless.rules" ]; then
    sudo cp udev/99-xbox360-wireless.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    print_status "Udev rules installed successfully."
else
    print_warning "Udev rules file not found. Make sure you're running this from the package directory."
fi

# Add user to plugdev group
print_status "Adding user to plugdev group..."
sudo usermod -a -G plugdev $USER
print_status "User added to plugdev group."

# Check for Xbox adapter
print_status "Checking for Xbox 360 Wireless Adapter..."
if lsusb | grep -q "045e:0719"; then
    print_status "Xbox 360 Wireless Adapter detected!"
else
    print_warning "Xbox 360 Wireless Adapter not detected. Make sure it's plugged in."
fi

# Check for joystick devices
print_status "Checking for joystick devices..."
if ls /dev/input/js* &> /dev/null; then
    print_status "Joystick devices found: $(ls /dev/input/js*)"
else
    print_warning "No joystick devices found. The controller may not be paired."
fi

# Build instructions
echo ""
echo "================================================="
echo "Next Steps:"
echo "================================================="
echo "1. Log out and log back in (for group changes to take effect)"
echo "2. Make sure your Xbox 360 controller is paired with the wireless adapter"
echo "3. Build the ROS 2 package:"
echo "   cd ~/ros2_ws"
echo "   colcon build --packages-select xbox_controller_pkg"
echo "   source install/setup.bash"
echo "4. Test the installation:"
echo "   ros2 launch xbox_controller_pkg xbox_controller_launch.py"
echo "5. In another terminal:"
echo "   ros2 run xbox_controller_pkg test_controller"
echo ""
echo "For troubleshooting, run:"
echo "   ros2 run xbox_controller_pkg controller_diagnostics"
echo ""
print_status "Installation script completed!"