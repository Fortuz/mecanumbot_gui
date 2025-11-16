#!/bin/bash

# Quick Start Script for Button Mapping System
# This script helps launch all necessary components

echo "=========================================="
echo "Button Mapping System - Quick Start"
echo "=========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}✓${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

# Check ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    print_error "ROS2 environment not sourced!"
    echo "Please run: source /opt/ros/humble/setup.bash"
    exit 1
fi

print_status "ROS2 Distribution: $ROS_DISTRO"

# Check if button_mapper_pkg is built
if [ ! -d "install/button_mapper_pkg" ]; then
    print_warning "button_mapper_pkg not built yet"
    echo "Building button_mapper_pkg..."
    cd button_mapper_pkg
    ./install.sh
    cd ..
    source install/setup.bash
fi

print_status "button_mapper_pkg is ready"

echo ""
echo "Choose an option:"
echo "  1) Start Controller Bridge (run on host)"
echo "  2) Start Button Mapper Node (ROS2)"
echo "  3) Start Xbox Controller Node (ROS2)"
echo "  4) Start Joy Node (ROS2 controller driver)"
echo "  5) Start ALL ROS2 nodes (in background)"
echo "  6) View topic list"
echo "  7) Monitor button mappings config"
echo "  8) Monitor button events"
echo "  9) Exit"
echo ""

read -p "Enter choice [1-9]: " choice

case $choice in
    1)
        print_status "Starting Controller Bridge..."
        python3 controller_bridge.py
        ;;
    2)
        print_status "Starting Button Mapper Node..."
        ros2 run button_mapper_pkg button_mapper_node
        ;;
    3)
        print_status "Starting Xbox Controller Node..."
        ros2 run xbox_controller_pkg xbox_controller_node
        ;;
    4)
        print_status "Starting Joy Node..."
        ros2 run joy joy_node
        ;;
    5)
        print_status "Starting all ROS2 nodes..."
        ros2 run joy joy_node &
        sleep 2
        ros2 run xbox_controller_pkg xbox_controller_node &
        sleep 2
        ros2 run button_mapper_pkg button_mapper_node &
        print_status "All nodes started in background"
        echo "To stop: pkill -f 'ros2 run'"
        ;;
    6)
        print_status "Available ROS2 topics:"
        ros2 topic list
        ;;
    7)
        print_status "Monitoring /button_mapper/config..."
        ros2 topic echo /button_mapper/config
        ;;
    8)
        print_status "Monitoring /xbox_controller/button_events..."
        ros2 topic echo /xbox_controller/button_events
        ;;
    9)
        print_status "Exiting..."
        exit 0
        ;;
    *)
        print_error "Invalid choice"
        exit 1
        ;;
esac
