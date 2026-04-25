#!/bin/bash

echo "=========================================="
echo "Starting MecanumBot ROS2 Nodes"
echo "=========================================="
echo ""

# ── Resolve paths relative to this script (call-location independent) ────────
# Layout:  A / mecanumbot_gui-button_mapping / robot / start_ros_nodes.sh
#                                              ^^^^^^ SCRIPT_DIR
#                          ^^^^^^^^^^^^^^^^^^^^^^^^^^^  PROJECT_DIR (B)
#          ^               WORKSPACE_ROOT (A) — colcon is called from here
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
WORKSPACE_ROOT="$(dirname "$PROJECT_DIR")"

echo "Workspace root : $WORKSPACE_ROOT"

# ── 1. ROS2 base ──────────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash

# ── 2. Colcon workspace (contains mecanumbot_msgs + mecanumbot_gui packages) ──
if [ ! -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    echo "⚠ install/setup.bash not found — running colcon build first..."
    (cd "$WORKSPACE_ROOT" && colcon build --symlink-install)
    if [ $? -ne 0 ]; then
        echo "✗ colcon build failed!"
        exit 1
    fi
    echo "✓ colcon build completed"
fi
source "$WORKSPACE_ROOT/install/setup.bash"
echo "✓ Workspace sourced"
echo ""

# ── 3. Launch all nodes via the ROS2 launch file ──────────────────────────────
# bringup.launch.py starts joy_node, xbox_controller_node, and mapping_listener.
# Ctrl+C here cleanly stops all of them — no manual PID management needed.
echo "Launching nodes (Ctrl+C to stop all)..."
ros2 launch button_mapping_ros bringup.launch.py
