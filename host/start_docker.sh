#!/bin/bash

echo "=========================================="
echo "Starting MecanumBot Docker Container"
echo "=========================================="
echo ""

# ── Cleanup on Ctrl+C or termination ──────────────────────────
cleanup() {
    echo ""
    echo "Shutting down mecanumbot-gui container..."
    sudo docker stop mecanumbot-gui 2>/dev/null
    sudo docker rm   mecanumbot-gui 2>/dev/null
    echo "✓ Container stopped."
    exit 0
}
trap cleanup INT TERM

# Set ROS Domain ID
export ROS_DOMAIN_ID=30

# Stop and remove existing container if running
echo "Stopping existing container (if any)..."
sudo docker stop mecanumbot-gui 2>/dev/null
sudo docker rm   mecanumbot-gui 2>/dev/null
echo ""

# Always rebuild the image so code changes are picked up
echo "Building Docker image..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Build context is the project root (parent of host/) so the Dockerfile can
# reach both host/ and shared/ directories with its COPY instructions.
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
sudo docker build -t mecanumbot-gui -f "$SCRIPT_DIR/Dockerfile" "$PROJECT_ROOT"
if [ $? -ne 0 ]; then
    echo "✗ Docker build failed!"
    exit 1
fi
echo "✓ Image built successfully."
echo ""

# Ensure the host Documents directory exists and is owned by the current user
# before Docker mounts it (Docker auto-creates missing mount dirs as root).
mkdir -p ~/Documents

# Start Docker container (foreground-attached via logs, not -d)
echo "Starting mecanumbot-gui container..."
sudo docker run -d \
    --name mecanumbot-gui \
    --network host \
    --ipc=host \
    --pid=host \
    --privileged \
    --user "$(id -u):$(id -g)" \
    -e ROS_DOMAIN_ID=30 \
    -e ROS_LOCALHOST_ONLY=0 \
    -v ~/Documents:/host_docs \
    mecanumbot-gui

if [ $? -ne 0 ]; then
    echo "✗ Failed to start Docker container!"
    exit 1
fi

echo "✓ Docker container started successfully!"
echo ""
echo "=========================================="
echo "Flask GUI: http://localhost:8080"
echo "Log file:  ~/Documents/controller_status.log"
echo "Recordings:~/Documents/recordings/"
echo "=========================================="
echo ""
echo "Press Ctrl+C to stop the container."
echo ""

# Stream container logs in the foreground.
# docker logs -f exits on its own when the container stops,
# or is killed by the trap before that.
sudo docker logs -f mecanumbot-gui
stty sane 2>/dev/null
cleanup
