#!/bin/bash
# Bash script to run the complete controller bridge solution on Linux/Unix

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${GREEN}Controller Bridge Solution - Linux Setup${NC}"
echo -e "${GREEN}=========================================${NC}"

# Check if controller_bridge.py exists
if [ ! -f "controller_bridge.py" ]; then
    echo -e "${RED}Error: controller_bridge.py not found!${NC}"
    echo -e "${RED}Make sure you're running this from the correct directory.${NC}"
    exit 1
fi

# Check if Python is available
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}Error: python3 not found!${NC}"
    echo -e "${RED}Please install Python 3.8+ first.${NC}"
    exit 1
fi

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo -e "${RED}Error: docker not found!${NC}"
    echo -e "${RED}Please install Docker first.${NC}"
    exit 1
fi

echo -e "${YELLOW}Step 1: Starting Controller Bridge Service on host...${NC}"
echo -e "${YELLOW}This will run in the background. Check the logs if needed!${NC}"

# Start the bridge service in the background
python3 controller_bridge.py > bridge_service.log 2>&1 &
BRIDGE_PID=$!

# Store the PID for cleanup
echo $BRIDGE_PID > bridge_service.pid

# Wait a moment for the service to start
echo -e "${YELLOW}Waiting for bridge service to start...${NC}"
sleep 3

# Test if bridge service is running
if curl -s "http://localhost:8899/status" > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Bridge service is running!${NC}"
else
    echo -e "${YELLOW}⚠ Bridge service may not be ready yet. Continue anyway...${NC}"
fi

echo -e "${YELLOW}\nStep 2: Building Docker image...${NC}"
docker build -t controller-app .

if [ $? -ne 0 ]; then
    echo -e "${RED}Error building Docker image!${NC}"
    # Clean up bridge service
    if [ -f bridge_service.pid ]; then
        kill $(cat bridge_service.pid) 2>/dev/null
        rm bridge_service.pid
    fi
    exit 1
fi

echo -e "${YELLOW}\nStep 3: Starting Docker container...${NC}"
echo -e "${CYAN}The app will be available at http://localhost:8080${NC}"
echo -e "${CYAN}Controllers will be detected via the bridge service!${NC}"

# Function to cleanup on exit
cleanup() {
    echo -e "\n${YELLOW}Cleaning up...${NC}"
    if [ -f bridge_service.pid ]; then
        echo -e "${YELLOW}Stopping bridge service...${NC}"
        kill $(cat bridge_service.pid) 2>/dev/null
        rm bridge_service.pid
        echo -e "${GREEN}✓ Bridge service stopped${NC}"
    fi
    exit 0
}

# Set up signal handlers for cleanup
trap cleanup SIGINT SIGTERM

# Run Docker with host networking for bridge communication
# On Linux, we can use --network="host" for easier communication
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo -e "${YELLOW}Using Linux host networking...${NC}"
    docker run --rm -p 8080:8080 --network="host" controller-app
else
    echo -e "${YELLOW}Using standard Docker networking...${NC}"
    docker run --rm -p 8080:8080 --add-host=host.docker.internal:host-gateway controller-app
fi

# Cleanup when Docker container stops
cleanup