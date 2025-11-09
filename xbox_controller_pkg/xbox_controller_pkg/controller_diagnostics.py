#!/usr/bin/env python3
"""
Xbox 360 Controller Diagnostic Script

This script shows the complete data flow from USB device to button detection.

Usage: ros2 run xbox_controller_pkg controller_diagnostics

Author: Generated for mecanumbot project
Date: November 9, 2025
"""

import subprocess
import os
import time


def check_usb_device():
    """Check if Xbox 360 Wireless Adapter is connected."""
    print("🔍 Checking USB devices...")
    try:
        result = subprocess.run(['lsusb'], capture_output=True, text=True)
        lines = result.stdout.split('\n')
        
        xbox_devices = [line for line in lines if '045e:0719' in line]
        
        if xbox_devices:
            print("✅ Xbox 360 Wireless Adapter found:")
            for device in xbox_devices:
                print(f"   {device}")
                # Extract bus and device numbers
                parts = device.split()
                if len(parts) >= 4:
                    bus = parts[1]
                    dev = parts[3].rstrip(':')
                    print(f"   📍 Location: Bus {bus} Device {dev}")
        else:
            print("❌ Xbox 360 Wireless Adapter (045e:0719) not found!")
            print("   Make sure it's plugged in and recognized by the system.")
            
    except FileNotFoundError:
        print("❌ lsusb command not found (not on Linux?)")
    except Exception as e:
        print(f"❌ Error checking USB devices: {e}")


def check_joystick_devices():
    """Check for joystick device files."""
    print("\n🎮 Checking joystick devices...")
    
    js_devices = []
    for i in range(10):  # Check js0 to js9
        device_path = f'/dev/input/js{i}'
        if os.path.exists(device_path):
            js_devices.append(device_path)
    
    if js_devices:
        print("✅ Joystick devices found:")
        for device in js_devices:
            print(f"   {device}")
            
            # Check permissions
            try:
                stat_info = os.stat(device)
                mode = oct(stat_info.st_mode)[-3:]
                print(f"   📋 Permissions: {mode}")
                
                # Check if readable
                readable = os.access(device, os.R_OK)
                print(f"   📖 Readable: {'✅ Yes' if readable else '❌ No'}")
                
            except Exception as e:
                print(f"   ❌ Error checking {device}: {e}")
    else:
        print("❌ No joystick devices found in /dev/input/")
        print("   The Xbox adapter might not be creating a joystick device.")


def check_udev_rules():
    """Check if udev rules are installed."""
    print("\n📋 Checking udev rules...")
    
    rules_files = [
        '/etc/udev/rules.d/99-xbox360-wireless.rules',
        '/lib/udev/rules.d/99-xbox360-wireless.rules'
    ]
    
    found_rules = False
    for rules_file in rules_files:
        if os.path.exists(rules_file):
            print(f"✅ Found udev rules: {rules_file}")
            found_rules = True
            
            try:
                with open(rules_file, 'r') as f:
                    content = f.read()
                    if '045e:0719' in content:
                        print("   📋 Rules contain Xbox 360 Wireless Adapter (045e:0719)")
                    else:
                        print("   ⚠️  Rules don't seem to contain Xbox 360 Wireless Adapter")
            except Exception as e:
                print(f"   ❌ Error reading rules file: {e}")
    
    if not found_rules:
        print("❌ No Xbox 360 udev rules found")
        print("   Install the 99-xbox360-wireless.rules file to /etc/udev/rules.d/")


def test_jstest():
    """Test if jstest can read the controller.""" 
    print("\n🧪 Testing controller input with jstest...")
    
    js_devices = []
    for i in range(10):
        device_path = f'/dev/input/js{i}'
        if os.path.exists(device_path):
            js_devices.append(device_path)
    
    if not js_devices:
        print("❌ No joystick devices to test")
        return
        
    device = js_devices[0]  # Test first device
    print(f"   Testing {device}...")
    
    try:
        # Run jstest for 3 seconds to see if it can read the device
        print("   Press any button on the controller in the next 3 seconds...")
        result = subprocess.run(['timeout', '3', 'jstest', device], 
                              capture_output=True, text=True)
        
        if result.returncode == 0 or result.returncode == 124:  # 124 = timeout
            print("✅ jstest can read the device")
            if result.stdout:
                lines = result.stdout.split('\n')[:5]  # First few lines
                for line in lines:
                    if line.strip():
                        print(f"   📊 {line}")
        else:
            print(f"❌ jstest failed: {result.stderr}")
            
    except FileNotFoundError:
        print("❌ jstest not found. Install with: sudo apt install joystick")
    except Exception as e:
        print(f"❌ Error running jstest: {e}")


def show_button_mapping():
    """Show the Xbox 360 controller button mapping."""
    print("\n🎮 Xbox 360 Controller Button Mapping:")
    print("   Buttons (in Joy message buttons array):")
    print("   [0] = A button")
    print("   [1] = B button") 
    print("   [2] = X button")
    print("   [3] = Y button")
    print("   [4] = LB (Left Bumper)")
    print("   [5] = RB (Right Bumper)")
    print("   [6] = BACK button")
    print("   [7] = START button")
    print("   [8] = XBOX button (center)")
    print("   [9] = LS (Left Stick button)")
    print("   [10] = RS (Right Stick button)")
    
    print("\n   Axes (in Joy message axes array):")
    print("   [0] = Left Stick X (-1=left, +1=right)")
    print("   [1] = Left Stick Y (-1=down, +1=up)")
    print("   [2] = Left Trigger (-1=released, +1=pressed)")
    print("   [3] = Right Stick X (-1=left, +1=right)")
    print("   [4] = Right Stick Y (-1=down, +1=up)")
    print("   [5] = Right Trigger (-1=released, +1=pressed)")
    print("   [6] = D-Pad X (-1=left, +1=right)")
    print("   [7] = D-Pad Y (-1=down, +1=up)")


def main():
    """Run all diagnostic tests."""
    print("=" * 70)
    print("Xbox 360 Controller Diagnostic Report")
    print("=" * 70)
    
    check_usb_device()
    check_joystick_devices() 
    check_udev_rules()
    test_jstest()
    show_button_mapping()
    
    print("\n" + "=" * 70)
    print("Diagnostic complete!")
    print("\nData Flow Summary:")
    print("Controller → Xbox Adapter (045e:0719) → /dev/input/js0 → joy_node → Joy messages")
    print("=" * 70)


if __name__ == '__main__':
    main()