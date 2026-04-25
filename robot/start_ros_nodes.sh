#!/bin/bash

echo "=========================================="
echo "Starting MecanumBot ROS2 Nodes"
echo "=========================================="
echo ""

# ── Resolve paths relative to this script (call-location independent) ────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Workspace root : $PROJECT_ROOT"

# ── 1. ROS2 base ──────────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash

# ── 2. mecanumbot-main (provides mecanumbot_msgs on the robot side) ───────────
MECANUMBOT_MAIN="$HOME/mecanumbot-main/install/setup.bash"
if [ -f "$MECANUMBOT_MAIN" ]; then
    source "$MECANUMBOT_MAIN"
    echo "✓ mecanumbot-main workspace sourced"
else
    echo "⚠ mecanumbot-main not found at $MECANUMBOT_MAIN — some robot services may be unavailable"
fi

# ── 3. Local colcon workspace ─────────────────────────────────────────────────
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
echo "✓ Local workspace sourced"
echo ""

# ── 4. Launch all nodes via the ROS2 launch file ──────────────────────────────
# bringup.launch.py starts joy_node, xbox_controller_node, and mapping_listener.
# Ctrl+C here cleanly stops all of them — no manual PID management needed.
echo "Launching nodes (Ctrl+C to stop all)..."
ros2 launch button_mapping_ros bringup.launch.py
