#!/bin/bash

echo "=========================================="
echo "Starting ROS2 Nodes on MecanumBot (robot)"
echo "=========================================="
echo ""

# ── Cleanup on Ctrl+C or termination ──────────────────────────
cleanup() {
    echo ""
    echo "Shutting down ROS2 nodes..."
    [ -n "$MAPPING_PID" ] && kill $MAPPING_PID 2>/dev/null
    [ -n "$XBOX_PID"    ] && kill $XBOX_PID    2>/dev/null
    [ -n "$JOY_PID"     ] && kill $JOY_PID     2>/dev/null
    wait 2>/dev/null
    echo "✓ All nodes stopped."
    exit 0
}
trap cleanup INT TERM

# Set ROS Domain ID (must match the host)
export ROS_DOMAIN_ID=19
export ROS_LOCALHOST_ONLY=0

# Directory containing this script (robot/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# The colcon workspace root — one level up from robot/
# Run 'colcon build' from PROJECT_ROOT before starting this script.
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Script directory : $SCRIPT_DIR"
echo "Workspace root   : $PROJECT_ROOT"
echo "ROS_DOMAIN_ID    : $ROS_DOMAIN_ID"
echo ""

# Source ROS2 workspace
echo "Sourcing ROS2 workspace..."

# 1. ROS2 base
source /opt/ros/humble/setup.bash

# 2. mecanumbot-main workspace — provides mecanumbot_msgs on the robot side
MECANUMBOT_MAIN="$HOME/mecanumbot-main/install/setup.bash"
if [ -f "$MECANUMBOT_MAIN" ]; then
    source "$MECANUMBOT_MAIN"
    echo "✓ mecanumbot-main workspace sourced"
else
    echo "⚠ mecanumbot-main not found at $MECANUMBOT_MAIN — some robot services may be unavailable"
fi

# 3. Local colcon workspace (xbox_controller_pkg, button_mapping_ros, mecanumbot_msgs)
#    Packages live under robot/ and shared/ — colcon must be told where to look.
if [ ! -f "$PROJECT_ROOT/install/setup.bash" ]; then
    echo "⚠ install/setup.bash not found — running colcon build first..."
    (cd "$PROJECT_ROOT" && colcon build --symlink-install \
        --paths \
            "$PROJECT_ROOT/robot/xbox_controller_pkg" \
            "$PROJECT_ROOT/robot/button_mapping_ros")
    if [ $? -ne 0 ]; then
        echo "✗ colcon build failed!"
        exit 1
    fi
    echo "✓ colcon build completed"
fi
source "$PROJECT_ROOT/install/setup.bash"
echo "✓ Local workspace sourced successfully"
echo ""

# Kill any stale nodes from a previous run
echo "Stopping any existing ROS2 nodes..."
pkill -f joy_node             2>/dev/null
pkill -f xbox_controller_node 2>/dev/null
pkill -f mapping_listener     2>/dev/null
sleep 1
echo ""

# Start joy_node
echo "Starting joy_node..."
ros2 run joy joy_node &
JOY_PID=$!
sleep 2

if ! ps -p $JOY_PID > /dev/null; then
    echo "✗ Failed to start joy_node"
    cleanup
fi
echo "✓ joy_node started (PID: $JOY_PID)"
echo ""

# Start xbox_controller_node
echo "Starting xbox_controller_node..."
ros2 run xbox_controller_pkg xbox_controller_node &
XBOX_PID=$!
sleep 2

if ! ps -p $XBOX_PID > /dev/null; then
    echo "✗ Failed to start xbox_controller_node"
    cleanup
fi
echo "✓ xbox_controller_node started (PID: $XBOX_PID)"
echo ""

# Start mapping_listener
echo "Starting mapping_listener..."
ros2 run button_mapping_ros mapping_listener &
MAPPING_PID=$!
sleep 2

if ! ps -p $MAPPING_PID > /dev/null; then
    echo "✗ Failed to start mapping_listener"
    cleanup
fi
echo "✓ mapping_listener started (PID: $MAPPING_PID)"
echo ""

echo "=========================================="
echo "All ROS2 nodes running."
echo "  joy_node:           PID $JOY_PID"
echo "  xbox_controller:    PID $XBOX_PID"
echo "  mapping_listener:   PID $MAPPING_PID"
echo "=========================================="
echo ""
echo "Press Ctrl+C to stop all nodes."
echo ""

# Wait for all background jobs — Ctrl+C triggers the trap
wait
