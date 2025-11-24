#!/bin/bash
################################################################################
# Mecanumbot GUI System Startup Script
# 
# This script starts all components of the mecanumbot GUI system in order:
# 1. joy_node - Reads PS4/Xbox controller hardware
# 2. xbox_controller_node - Publishes controller events to ROS2 topics
# 3. mapping_listener - Maps controller inputs to robot actions
# 4. Docker GUI - Web interface for configuration
#
# Usage: ./start_system.sh
################################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Mecanumbot GUI System Startup${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Function to check if a process is running
check_process() {
    if pgrep -f "$1" > /dev/null; then
        return 0
    else
        return 1
    fi
}

# Function to wait for a process to start
wait_for_process() {
    local process_name=$1
    local max_wait=10
    local count=0
    
    while [ $count -lt $max_wait ]; do
        if check_process "$process_name"; then
            return 0
        fi
        sleep 0.5
        count=$((count + 1))
    done
    return 1
}

# Clean up function
cleanup() {
    echo -e "\n${YELLOW}Shutting down all processes...${NC}"
    pkill -f "joy_node" 2>/dev/null || true
    pkill -f "xbox_controller_node" 2>/dev/null || true
    pkill -f "mapping_listener" 2>/dev/null || true
    sudo docker stop mecanumbot-gui 2>/dev/null || true
    echo -e "${GREEN}All processes stopped${NC}"
    exit 0
}

# Set trap for cleanup on script exit
trap cleanup SIGINT SIGTERM

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}Sourcing ROS2 Humble...${NC}"
    source /opt/ros/humble/setup.bash
fi

# Check if controller is connected
if [ ! -e /dev/input/js0 ]; then
    echo -e "${RED}ERROR: No controller detected at /dev/input/js0${NC}"
    echo -e "${YELLOW}Please connect your PS4 or Xbox controller and try again${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Controller detected at /dev/input/js0${NC}"
echo ""

################################################################################
# Step 1: Start joy_node
################################################################################
echo -e "${BLUE}[1/4] Starting joy_node...${NC}"

if check_process "joy_node"; then
    echo -e "${YELLOW}  joy_node already running, killing old instance...${NC}"
    pkill -f "joy_node"
    sleep 1
fi

# Start joy_node in background
ros2 run joy joy_node > /tmp/joy_node.log 2>&1 &
JOY_PID=$!

if wait_for_process "joy_node"; then
    echo -e "${GREEN}  ✓ joy_node started (PID: $JOY_PID)${NC}"
else
    echo -e "${RED}  ✗ Failed to start joy_node${NC}"
    exit 1
fi
sleep 1

################################################################################
# Step 2: Start xbox_controller_node
################################################################################
echo -e "${BLUE}[2/4] Starting xbox_controller_node...${NC}"

if check_process "xbox_controller_node"; then
    echo -e "${YELLOW}  xbox_controller_node already running, killing old instance...${NC}"
    pkill -f "xbox_controller_node"
    sleep 1
fi

# Start xbox_controller_node in background
cd "$SCRIPT_DIR/xbox_controller_pkg"
python3 -m xbox_controller_pkg.xbox_controller_node > /tmp/xbox_controller_node.log 2>&1 &
XBOX_PID=$!

if wait_for_process "xbox_controller_node"; then
    echo -e "${GREEN}  ✓ xbox_controller_node started (PID: $XBOX_PID)${NC}"
else
    echo -e "${RED}  ✗ Failed to start xbox_controller_node${NC}"
    exit 1
fi
sleep 2

# Verify controller connection
if [ -f "$HOME/Documents/controller_status.json" ]; then
    CONNECTED=$(cat "$HOME/Documents/controller_status.json" | grep -o '"connected":[^,}]*' | cut -d':' -f2 | tr -d ' ')
    if [ "$CONNECTED" = "true" ]; then
        echo -e "${GREEN}  ✓ Controller connected and active${NC}"
    else
        echo -e "${YELLOW}  ⚠ Controller status file exists but shows disconnected${NC}"
    fi
else
    echo -e "${YELLOW}  ⚠ Waiting for controller status file...${NC}"
fi

################################################################################
# Step 3: Start mapping_listener
################################################################################
echo -e "${BLUE}[3/4] Starting mapping_listener...${NC}"

if check_process "mapping_listener"; then
    echo -e "${YELLOW}  mapping_listener already running, killing old instance...${NC}"
    pkill -f "mapping_listener"
    sleep 1
fi

# Start mapping_listener in background
cd "$SCRIPT_DIR/button_mapping_ros"
export PYTHONPATH="$SCRIPT_DIR/button_mapping_ros:$PYTHONPATH"
python3 -m button_mapping_ros.mapping_listener > /tmp/mapping_listener.log 2>&1 &
MAPPING_PID=$!

if wait_for_process "mapping_listener"; then
    echo -e "${GREEN}  ✓ mapping_listener started (PID: $MAPPING_PID)${NC}"
else
    echo -e "${RED}  ✗ Failed to start mapping_listener${NC}"
    exit 1
fi
sleep 1

################################################################################
# Step 4: Start Docker GUI
################################################################################
echo -e "${BLUE}[4/4] Starting Docker GUI...${NC}"

# Check if Docker container is already running
if sudo docker ps | grep -q "mecanumbot-gui"; then
    echo -e "${YELLOW}  Docker container already running, stopping old instance...${NC}"
    sudo docker stop mecanumbot-gui > /dev/null 2>&1
    sudo docker rm mecanumbot-gui > /dev/null 2>&1
fi

# Remove container if it exists but is stopped
if sudo docker ps -a | grep -q "mecanumbot-gui"; then
    sudo docker rm mecanumbot-gui > /dev/null 2>&1
fi

# Ensure Documents directory exists and has correct permissions
mkdir -p "$HOME/Documents"
sudo chown -R $USER:$USER "$HOME/Documents"

# Start Docker container
cd "$SCRIPT_DIR"
sudo docker run -d \
    --name mecanumbot-gui \
    -p 8080:8080 \
    -v "$HOME/Documents:/host_docs" \
    --network host \
    mecanumbot-gui > /dev/null 2>&1

if sudo docker ps | grep -q "mecanumbot-gui"; then
    echo -e "${GREEN}  ✓ Docker GUI started${NC}"
    sleep 2
    
    # Get IP addresses
    LOCAL_IP="127.0.0.1"
    NETWORK_IP=$(hostname -I | awk '{print $1}')
    
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}✓ All systems started successfully!${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo -e "${BLUE}Web Interface:${NC}"
    echo -e "  Local:   http://${LOCAL_IP}:8080"
    echo -e "  Network: http://${NETWORK_IP}:8080"
    echo ""
    echo -e "${BLUE}Process IDs:${NC}"
    echo -e "  joy_node:            ${JOY_PID}"
    echo -e "  xbox_controller:     ${XBOX_PID}"
    echo -e "  mapping_listener:    ${MAPPING_PID}"
    echo ""
    echo -e "${BLUE}Log Files:${NC}"
    echo -e "  joy_node:            /tmp/joy_node.log"
    echo -e "  xbox_controller:     /tmp/xbox_controller_node.log"
    echo -e "  mapping_listener:    /tmp/mapping_listener.log"
    echo -e "  docker:              sudo docker logs mecanumbot-gui"
    echo ""
    echo -e "${YELLOW}Press Ctrl+C to stop all processes${NC}"
    echo ""
    
    # Wait indefinitely (until Ctrl+C)
    while true; do
        sleep 1
    done
else
    echo -e "${RED}  ✗ Failed to start Docker GUI${NC}"
    echo -e "${YELLOW}  Check Docker logs: sudo docker logs mecanumbot-gui${NC}"
    exit 1
fi
