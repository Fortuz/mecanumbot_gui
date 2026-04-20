#!/bin/bash

echo "=========================================="
echo "Starting Fake Robot Node (for GUI testing)"
echo "=========================================="
echo ""

# ── Cleanup on Ctrl+C or termination ──────────────────────────
cleanup() {
    echo ""
    echo "Shutting down fake robot node..."
    [ -n "$FAKE_PID" ] && kill $FAKE_PID 2>/dev/null
    wait 2>/dev/null
    echo "✓ Fake robot node stopped."
    exit 0
}
trap cleanup INT TERM

# Must match the host Docker container (start_docker.sh)
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export ROS_SUPER_CLIENT=true

# Directory containing this script (fake_robot/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Colcon workspace root — one level up from fake_robot/
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Script directory : $SCRIPT_DIR"
echo "Workspace root   : $PROJECT_ROOT"
echo "ROS_DOMAIN_ID    : $ROS_DOMAIN_ID"
echo ""

# ── Source ROS2 base ──────────────────────────────────────────
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "✗ ROS2 Humble not found at /opt/ros/humble"
    echo "  Install it first: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi
source /opt/ros/humble/setup.bash
echo "✓ ROS2 Humble sourced"

# ── Source colcon workspace (provides mecanumbot_msgs) ────────
INSTALL_SETUP="$PROJECT_ROOT/install/setup.bash"
if [ ! -f "$INSTALL_SETUP" ]; then
    echo "✗ Colcon workspace not built — install/setup.bash not found"
    echo ""
    echo "  Build the workspace first:"
    echo "      cd $PROJECT_ROOT"
    echo "      source /opt/ros/humble/setup.bash"
    echo "      colcon build --packages-select mecanumbot_msgs button_mapping_ros xbox_controller_pkg"
    echo ""
    exit 1
fi
source "$INSTALL_SETUP"
echo "✓ Colcon workspace sourced"
echo ""

# ── Check mecanumbot_msgs is available ────────────────────────
python3 -c "from mecanumbot_msgs.msg import OpenCRState" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "✗ mecanumbot_msgs not available after sourcing workspace"
    echo "  Rebuild with:"
    echo "      colcon build --packages-select mecanumbot_msgs"
    exit 1
fi
echo "✓ mecanumbot_msgs available"
echo ""

# ── Launch fake robot node ────────────────────────────────────
echo "Starting fake_robot_node.py ..."
echo ""
echo "  Serving services:"
echo "    /mecanumbot/get_robot_actions"
echo "    /mecanumbot/get_mappings  save_mapping  delete_mapping  apply_mapping"
echo "    /mecanumbot/save_action  delete_action"
echo "    set_led_status  get_led_status"
echo ""
echo "  Publishing topics at 10 Hz:"
echo "    /opencr_state  /xbox_controller/connection_status"
echo "    /xbox_controller/button_events  /xbox_controller/joystick_events"
echo "    /cmd_vel  /joy  /odom  /imu  /joint_states  /battery_state"
echo ""
echo "Press Ctrl+C to stop."
echo ""

python3 "$SCRIPT_DIR/fake_robot_node.py" &
FAKE_PID=$!

wait $FAKE_PID
