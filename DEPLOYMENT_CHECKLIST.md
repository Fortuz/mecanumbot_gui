# Button Mapping System - Deployment Checklist

## Pre-Deployment Checklist

### ✅ Prerequisites
- [ ] ROS2 Humble installed
- [ ] Python 3.8+ installed
- [ ] Docker installed (if using Docker)
- [ ] Xbox controller or compatible gamepad
- [ ] ros-humble-joy package installed: `sudo apt install ros-humble-joy`

### ✅ File Verification
Check that all files are present:

#### Web Interface Files
- [ ] `templates/button_mapping.html`
- [ ] `templates/success.html` (updated)
- [ ] `templates/index.html` (updated)
- [ ] `app.py` (updated with button mapping routes)

#### Controller Bridge
- [ ] `controller_bridge.py` (updated with POST endpoint)

#### ROS2 Package
- [ ] `button_mapper_pkg/package.xml`
- [ ] `button_mapper_pkg/setup.py`
- [ ] `button_mapper_pkg/setup.cfg`
- [ ] `button_mapper_pkg/button_mapper_pkg/__init__.py`
- [ ] `button_mapper_pkg/button_mapper_pkg/button_mapper_node.py`
- [ ] `button_mapper_pkg/launch/button_mapper_launch.py`
- [ ] `button_mapper_pkg/test/test_button_mapper_node.py`
- [ ] `button_mapper_pkg/resource/button_mapper_pkg`
- [ ] `button_mapper_pkg/install.sh`
- [ ] `button_mapper_pkg/README.md`

#### Documentation
- [ ] `BUTTON_MAPPING_GUIDE.md`
- [ ] `IMPLEMENTATION_SUMMARY.md`
- [ ] `SYSTEM_DIAGRAMS.md`
- [ ] `README.md` (updated)

#### Utilities
- [ ] `example_mappings.py`
- [ ] `quick_start.sh`

## Installation Steps

### Step 1: Build Button Mapper Package
```bash
cd button_mapper_pkg
chmod +x install.sh
./install.sh
cd ..
source install/setup.bash
```

**Verification:**
```bash
ros2 pkg list | grep button_mapper_pkg
# Should output: button_mapper_pkg
```

### Step 2: Test ROS2 Node
```bash
# In terminal 1
ros2 run button_mapper_pkg button_mapper_node

# In terminal 2
ros2 topic list | grep button_mapper
# Should show: /button_mapper/config
```

**Expected Output:**
```
[INFO] [button_mapper_node]: Button Mapper Node initialized
[INFO] [button_mapper_node]: Waiting for button mapping configuration...
```

### Step 3: Start Controller Bridge
```bash
python3 controller_bridge.py
```

**Expected Output:**
```
Controller Bridge Service
=========================
✓ Controller: Xbox 360
✓ Buttons available: 23
Controller Bridge API running on http://localhost:8899
```

**Verification:**
```bash
curl http://localhost:8899/status
# Should return: {"service": "Controller Bridge", "status": "running", ...}
```

### Step 4: Test Web Interface
```bash
# Start Flask app (or Docker container)
python3 app.py
# OR
docker-compose up
```

**Open browser:** `http://localhost:8080/button-mapping`

**Verification:**
- [ ] Page loads without errors
- [ ] Controller buttons are displayed
- [ ] Input fields for topic and parameters are present
- [ ] JSON validation works on blur

## Testing Checklist

### Test 1: Configuration Flow
- [ ] Open `http://localhost:8080/button-mapping`
- [ ] Configure button "A":
  - Topic: `/test/topic`
  - Params: `{"test": "value"}`
- [ ] Click "Save Mappings"
- [ ] Verify success message appears
- [ ] Check file created: `~/Documents/button_mappings.json`

### Test 2: ROS2 Configuration Publishing
```bash
# Terminal 1: Monitor config topic
ros2 topic echo /button_mapper/config

# Terminal 2: Configure via web interface
# Save button mappings

# Verify: Terminal 1 shows the configuration JSON
```

### Test 3: Button Event Publishing
```bash
# Start all nodes
ros2 run joy joy_node &
ros2 run xbox_controller_pkg xbox_controller_node &
ros2 run button_mapper_pkg button_mapper_node &

# Monitor button events
ros2 topic echo /xbox_controller/button_events

# Press buttons on controller
# Verify: Events appear in terminal
```

### Test 4: End-to-End Test
```bash
# Terminal 1: Monitor custom topic
ros2 topic echo /robot/move

# Terminal 2: Ensure all nodes running
ros2 node list
# Should show: /joy_node, /xbox360_controller_node, /button_mapper_node

# Terminal 3: Configure button A -> /robot/move {"direction": "forward"}
# Via web interface: http://localhost:8080/button-mapping

# Press button A on controller
# Verify: Terminal 1 shows: data: '{"direction": "forward"}'
```

### Test 5: Example Mappings
```bash
# Test example mappings script
python3 example_mappings.py print
# Should display 22+ button mappings

python3 example_mappings.py save test_mappings.json
# Should create test_mappings.json

python3 example_mappings.py publish
# Should publish to /button_mapper/config
```

## Troubleshooting Guide

### Issue: Button Mapper Node Not Receiving Config

**Symptoms:**
- Web interface saves successfully
- No configuration received by button_mapper_node

**Checks:**
1. [ ] Controller bridge running: `ps aux | grep controller_bridge`
2. [ ] ROS2 environment sourced: `echo $ROS_DISTRO`
3. [ ] Topic exists: `ros2 topic list | grep button_mapper`
4. [ ] Monitor topic: `ros2 topic echo /button_mapper/config`

**Solutions:**
- Restart controller_bridge.py
- Check bridge logs for errors
- Verify ROS2 installation: `ros2 --version`

### Issue: Button Events Not Received

**Symptoms:**
- Controller connected
- No events on `/xbox_controller/button_events`

**Checks:**
1. [ ] Joy node running: `ros2 node list | grep joy`
2. [ ] Joy topic publishing: `ros2 topic echo /joy`
3. [ ] Controller visible: `ls /dev/input/js*`
4. [ ] Xbox controller node running: `ros2 node list | grep xbox`

**Solutions:**
- Start joy node: `ros2 run joy joy_node`
- Check controller permissions: `sudo chmod 666 /dev/input/js0`
- Restart xbox_controller_node

### Issue: Mapped Topics Not Publishing

**Symptoms:**
- Button events received
- No messages on mapped topics (e.g., `/robot/move`)

**Checks:**
1. [ ] Button mapper node logs: Check for errors
2. [ ] Configuration loaded: Node logs should show "Received new button mapping configuration"
3. [ ] Button names match: Compare web config with event data
4. [ ] Topics created: `ros2 topic list | grep robot`

**Solutions:**
- Verify button names in configuration
- Check node logs: `ros2 run button_mapper_pkg button_mapper_node`
- Re-save configuration from web interface

### Issue: JSON Validation Errors

**Symptoms:**
- Red border on parameter field
- Alert message about invalid JSON

**Solutions:**
- Use proper JSON format: `{"key": "value"}`
- Use double quotes, not single quotes
- Ensure all braces and brackets are closed
- Example valid JSON: `{"direction": "forward", "speed": 1.0}`

### Issue: Docker Can't Connect to Bridge

**Symptoms:**
- Flask app shows "Could not connect to bridge service"

**Checks:**
1. [ ] Bridge running on host: `curl http://localhost:8899/status`
2. [ ] Docker network configuration: `--add-host=host.docker.internal:host-gateway`
3. [ ] Port 8899 accessible

**Solutions:**
- Verify bridge URL in Docker: Try `http://172.17.0.1:8899`
- Check firewall settings
- Restart both bridge and Docker container

## Performance Checklist

### Latency Test
- [ ] Button press to event published: < 50ms
- [ ] Event received to topic published: < 10ms
- [ ] Total latency: < 100ms

### Resource Usage
- [ ] button_mapper_node CPU: < 5%
- [ ] button_mapper_node RAM: < 50MB
- [ ] No memory leaks after extended use

### Reliability
- [ ] Handles rapid button presses
- [ ] No dropped events
- [ ] Recovers from configuration errors
- [ ] Handles node restarts gracefully

## Production Deployment

### Security Considerations
- [ ] Change default ports if needed
- [ ] Implement authentication for web interface
- [ ] Validate all JSON inputs on server side
- [ ] Rate limit configuration updates
- [ ] Sanitize user inputs

### Monitoring
- [ ] Set up ROS2 diagnostics
- [ ] Monitor topic health
- [ ] Log configuration changes
- [ ] Track button press statistics
- [ ] Alert on node failures

### Backup
- [ ] Backup button_mappings.json regularly
- [ ] Version control for configurations
- [ ] Document custom mappings
- [ ] Test restore procedures

## Documentation Checklist

- [ ] All users understand the data flow
- [ ] Configuration examples provided
- [ ] Troubleshooting guide accessible
- [ ] Architecture diagrams available
- [ ] Code is commented
- [ ] README is up to date

## Final Verification

Run all tests in sequence:
```bash
# 1. Start all services
./quick_start.sh
# Choose option 5 (Start ALL ROS2 nodes)

# 2. Open web interface
xdg-open http://localhost:8080/button-mapping

# 3. Configure a button
# Button A -> /robot/move -> {"direction": "forward", "speed": 1.0}

# 4. Save and test
# Press button A on controller

# 5. Verify message received
ros2 topic echo /robot/move

# Expected: data: '{"direction": "forward", "speed": 1.0}'
```

**Success Criteria:**
- ✅ All nodes running
- ✅ Configuration saved
- ✅ Button press detected
- ✅ Message published to topic
- ✅ No errors in logs

## Sign-Off

- [ ] All files created and in correct locations
- [ ] All nodes can be started successfully
- [ ] Web interface is accessible and functional
- [ ] Configuration flow works end-to-end
- [ ] Button events are published correctly
- [ ] Mapped topics receive messages
- [ ] Documentation is complete and accurate
- [ ] Tests pass successfully
- [ ] System is ready for use

---

**Deployment Date:** _________________

**Deployed By:** _________________

**Notes:**
_____________________________________________
_____________________________________________
_____________________________________________
