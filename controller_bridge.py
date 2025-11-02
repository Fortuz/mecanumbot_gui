#!/usr/bin/env python3
"""
Controller Bridge Service
Runs on the host system to detect controllers and expose the information
to Docker containers via HTTP API.

Place this file in your Documents folder and run it before starting the Docker container.
"""

import pygame
import json
import time
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from datetime import datetime
import sys
import os

class ControllerInfo:
    def __init__(self):
        self.controller_type = "None"
        self.controller_buttons = []
        self.last_updated = None
        self.controller_detected = False
        
    def detect_controller(self):
        """Detect controller and update info"""
        try:
            pygame.init()
            pygame.joystick.init()
            
            joystick_count = pygame.joystick.get_count()
            
            if joystick_count == 0:
                print(f"[{datetime.now().strftime('%H:%M:%S')}] No controllers detected")
                self.controller_type = "None"
                self.controller_buttons = []
                self.controller_detected = False
                self.last_updated = datetime.now().isoformat()
                return False
            
            # Get the first controller
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            
            controller_name = joystick.get_name()
            print(f"[{datetime.now().strftime('%H:%M:%S')}] Controller detected: {controller_name}")
            
            # Check if it's an Xbox 360 controller
            if "xbox 360" in controller_name.lower() or "xbox360" in controller_name.lower() or "360" in controller_name.lower():
                self.controller_type = "Xbox 360"
                self.controller_buttons = [
                    "A", "B", "X", "Y",
                    "LB", "RB", "LT", "RT",
                    "Start", "Back",
                    "LS_Click", "RS_Click",
                    "LStickUp", "LStickDown", "LStickLeft", "LStickRight",
                    "RStickUp", "RStickDown", "RStickLeft", "RStickRight",
                    "DPadUp", "DPadDown", "DPadLeft", "DPadRight"
                ]
                print("Using Xbox 360 button layout")
            # Check if it's a PS4 controller
            elif "ps4" in controller_name.lower() or "dualshock 4" in controller_name.lower() or "wireless controller" in controller_name.lower():
                self.controller_type = "PS4 Controller"
                self.controller_buttons = [
                    "Cross", "Circle", "Square", "Triangle",
                    "L1", "R1", "L2", "R2",
                    "Share", "Options", "PS",
                    "L3", "R3",
                    "LStickUp", "LStickDown", "LStickLeft", "LStickRight",
                    "RStickUp", "RStickDown", "RStickLeft", "RStickRight",
                    "DPadUp", "DPadDown", "DPadLeft", "DPadRight",
                    "Touchpad_Click", "Touchpad_Touch"
                ]
                print("Using PS4 controller button layout")
            else:
                # Dynamic detection for other controllers
                self.controller_type = controller_name
                self.controller_buttons = []
                
                num_buttons = joystick.get_numbuttons()
                for i in range(num_buttons):
                    self.controller_buttons.append(f"Button_{i}")
                
                num_axes = joystick.get_numaxes()
                for i in range(num_axes):
                    self.controller_buttons.append(f"Axis_{i}_Positive")
                    self.controller_buttons.append(f"Axis_{i}_Negative")
                
                num_hats = joystick.get_numhats()
                for i in range(num_hats):
                    self.controller_buttons.append(f"Hat_{i}_Up")
                    self.controller_buttons.append(f"Hat_{i}_Down")
                    self.controller_buttons.append(f"Hat_{i}_Left")
                    self.controller_buttons.append(f"Hat_{i}_Right")
                
                print(f"Dynamic layout: {num_buttons} buttons, {num_axes} axes, {num_hats} hats")
            
            self.controller_detected = True
            self.last_updated = datetime.now().isoformat()
            return True
            
        except Exception as e:
            print(f"Error detecting controller: {e}")
            self.controller_type = "Error"
            self.controller_buttons = []
            self.controller_detected = False
            self.last_updated = datetime.now().isoformat()
            return False
        finally:
            try:
                pygame.joystick.quit()
                pygame.quit()
            except:
                pass
    
    def to_dict(self):
        """Convert to dictionary for JSON response"""
        return {
            "controller_type": self.controller_type,
            "controller_buttons": self.controller_buttons,
            "controller_detected": self.controller_detected,
            "last_updated": self.last_updated,
            "button_count": len(self.controller_buttons)
        }

# Global controller info instance
controller_info = ControllerInfo()

class ControllerHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        """Handle GET requests"""
        if self.path == '/controller-info':
            # Return current controller info
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')  # Allow CORS
            self.end_headers()
            
            response = controller_info.to_dict()
            self.wfile.write(json.dumps(response, indent=2).encode())
            
        elif self.path == '/detect':
            # Force re-detection
            print("Forced re-detection requested...")
            controller_info.detect_controller()
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            
            response = controller_info.to_dict()
            self.wfile.write(json.dumps(response, indent=2).encode())
            
        elif self.path == '/status':
            # Simple status check
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            
            status = {
                "service": "Controller Bridge",
                "status": "running",
                "timestamp": datetime.now().isoformat()
            }
            self.wfile.write(json.dumps(status, indent=2).encode())
            
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b'Not Found')
    
    def log_message(self, format, *args):
        """Override to customize logging"""
        print(f"[{datetime.now().strftime('%H:%M:%S')}] {format % args}")

def auto_detect_loop():
    """Periodically detect controllers"""
    while True:
        controller_info.detect_controller()
        time.sleep(5)  # Check every 5 seconds

def main():
    print("=" * 50)
    print("Controller Bridge Service")
    print("=" * 50)
    print("This service detects controllers on the host system")
    print("and exposes the information to Docker containers.")
    print()
    
    # Initial detection
    print("Performing initial controller detection...")
    controller_info.detect_controller()
    
    print(f"✓ Controller: {controller_info.controller_type}")
    print(f"✓ Buttons available: {len(controller_info.controller_buttons)}")
    
    # Start HTTP server (NO auto-detection to avoid race conditions)
    PORT = 8899
    server = HTTPServer(('0.0.0.0', PORT), ControllerHandler)
    
    print(f"Controller Bridge API running on http://localhost:{PORT}")
    print("Available endpoints:")
    print(f"  http://localhost:{PORT}/controller-info - Get controller info")
    print(f"  http://localhost:{PORT}/detect - Force re-detection")
    print(f"  http://localhost:{PORT}/status - Service status")
    print()
    print("Make sure to run this BEFORE starting your Docker container!")
    print("Press Ctrl+C to stop...")
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down Controller Bridge Service...")
        server.shutdown()

if __name__ == "__main__":
    main()