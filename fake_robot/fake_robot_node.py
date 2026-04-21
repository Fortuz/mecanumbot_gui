#!/usr/bin/env python3
"""
fake_robot_node.py — Fake Robot Node for GUI Testing
=====================================================

Simulates the robot side so the host GUI can be tested without real hardware.

What it provides
----------------
Publishers (periodic fake data):
  /xbox_controller/connection_status   ControllerStatus   2 Hz
  /xbox_controller/button_events       ButtonEvent        on demand (via timer)
  /xbox_controller/joystick_events     JoystickEvent      10 Hz
  /opencr_state                        OpenCRState        10 Hz
  /cmd_vel                             Twist              (echo of what would be sent)
  /joy                                 Joy                10 Hz
  /odom                                Odometry           10 Hz
  /imu                                 Imu                10 Hz
  /joint_states                        JointState         10 Hz
  /battery_state                       BatteryState       1 Hz

Service servers (accept any request, return success):
  /mecanumbot/get_robot_actions        GetRobotActions
  /mecanumbot/get_mappings             GetMappings
  /mecanumbot/save_mapping             SaveMapping
  /mecanumbot/delete_mapping           DeleteRobotMapping
  /mecanumbot/apply_mapping            ApplyMapping
  /mecanumbot/save_action              SaveRobotAction
  /mecanumbot/delete_action            DeleteRobotAction
  set_led_status                       SetLedStatus
  get_led_status                       GetLedStatus
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, Imu, JointState, BatteryState
from nav_msgs.msg import Odometry

from mecanumbot_msgs.msg import (
    ControllerStatus, ButtonEvent, JoystickEvent, OpenCRState,
)
from mecanumbot_msgs.srv import (
    GetRobotActions,
    SetLedStatus, GetLedStatus,
    GetMappings, SaveMapping, DeleteRobotMapping, ApplyMapping,
    SaveRobotAction, DeleteRobotAction,
    GetRecordingSchemes, SaveRecordingScheme, DeleteRecordingScheme,
)


# ── LED state (in-memory store so get_led_status reflects set_led_status) ────
_led_state = {
    'fl_mode': 3, 'fl_color': 2,   # PULSE GREEN
    'fr_mode': 3, 'fr_color': 2,
    'bl_mode': 3, 'bl_color': 2,
    'br_mode': 3, 'br_color': 2,
}


class FakeRobotNode(Node):

    def __init__(self):
        super().__init__('fake_robot_node')
        self.get_logger().info('Fake Robot Node starting …')
        self._t = 0.0   # phase counter for oscillating fake values

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_status  = self.create_publisher(ControllerStatus, '/xbox_controller/connection_status', 10)
        self._pub_btn     = self.create_publisher(ButtonEvent,       '/xbox_controller/button_events',    10)
        self._pub_joy_ev  = self.create_publisher(JoystickEvent,     '/xbox_controller/joystick_events',  10)
        self._pub_opencr  = self.create_publisher(OpenCRState,       '/opencr_state',                     10)
        self._pub_cmd_vel = self.create_publisher(Twist,             '/cmd_vel',                          10)
        self._pub_joy     = self.create_publisher(Joy,               '/joy',                              10)
        self._pub_odom    = self.create_publisher(Odometry,          '/odom',                             10)
        self._pub_imu     = self.create_publisher(Imu,               '/imu',                              10)
        self._pub_joints  = self.create_publisher(JointState,        '/joint_states',                     10)
        self._pub_battery = self.create_publisher(BatteryState,      '/battery_state',                    10)

        # ── Service servers ───────────────────────────────────────────────────
        self.create_service(GetRobotActions,    '/mecanumbot/get_robot_actions', self._get_robot_actions_cb)
        self.create_service(SetLedStatus,       'set_led_status',              self._set_led_cb)
        self.create_service(GetLedStatus,       'get_led_status',              self._get_led_cb)
        self.create_service(GetMappings,        '/mecanumbot/get_mappings',    self._get_mappings_cb)
        self.create_service(SaveMapping,        '/mecanumbot/save_mapping',    self._save_mapping_cb)
        self.create_service(DeleteRobotMapping, '/mecanumbot/delete_mapping',  self._del_mapping_cb)
        self.create_service(ApplyMapping,       '/mecanumbot/apply_mapping',   self._apply_mapping_cb)
        self.create_service(SaveRobotAction,    '/mecanumbot/save_action',     self._save_action_cb)
        self.create_service(DeleteRobotAction,  '/mecanumbot/delete_action',   self._del_action_cb)
        self.create_service(GetRecordingSchemes,   '/mecanumbot/get_recording_schemes',    self._get_schemes_cb)
        self.create_service(SaveRecordingScheme,   '/mecanumbot/save_recording_scheme',    self._save_scheme_cb)
        self.create_service(DeleteRecordingScheme, '/mecanumbot/delete_recording_scheme',  self._del_scheme_cb)

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(0.1,  self._publish_10hz)   # 10 Hz
        self.create_timer(0.5,  self._publish_2hz)    # 2 Hz
        self.create_timer(1.0,  self._publish_1hz)    # 1 Hz
        self.create_timer(3.0,  self._publish_button) # occasional button press

        self.get_logger().info('Fake Robot Node ready.')
        self.get_logger().info('  Services : /mecanumbot/get_robot_actions  get_mappings  save_mapping  delete_mapping  apply_mapping  save_action  delete_action  get_recording_schemes  save_recording_scheme  delete_recording_scheme  set_led_status  get_led_status')
        self.get_logger().info('  Topics   : /opencr_state  /xbox_controller/*  /joy  /cmd_vel  /odom  /imu  /joint_states  /battery_state')

    # ── Periodic publishers ───────────────────────────────────────────────────

    def _publish_10hz(self):
        self._t += 0.1
        now = self.get_clock().now().to_msg()
        s   = math.sin(self._t)
        c   = math.cos(self._t)

        # OpenCR State — oscillating wheel velocities, fake IMU/battery
        ocr = OpenCRState()
        ocr.vel_fl = 0.3 * s;  ocr.vel_fr = 0.3 * s
        ocr.vel_bl = 0.3 * s;  ocr.vel_br = 0.3 * s
        ocr.pos_fl = self._t;  ocr.pos_fr = self._t
        ocr.pos_bl = self._t;  ocr.pos_br = self._t
        ocr.curr_fl = 0.2 + 0.05 * s;  ocr.curr_fr = 0.2 + 0.05 * s
        ocr.curr_bl = 0.2 + 0.05 * s;  ocr.curr_br = 0.2 + 0.05 * s
        ocr.pos_n  = 0.0;  ocr.pos_gl = 0.0;  ocr.pos_gr = 0.0
        ocr.battery_voltage   = 11.8 + 0.2 * s
        ocr.imu_angular_vel_x = 0.0
        ocr.imu_angular_vel_y = 0.0
        ocr.imu_angular_vel_z = 0.05 * s
        ocr.imu_linear_acc_x  = 0.01 * s
        ocr.imu_linear_acc_y  = 0.01 * c
        ocr.imu_linear_acc_z  = 9.81
        ocr.imu_orientation_w = 1.0
        ocr.imu_orientation_x = 0.0
        ocr.imu_orientation_y = 0.0
        ocr.imu_orientation_z = 0.0
        self._pub_opencr.publish(ocr)

        # Joy — left stick moves in a circle
        joy = Joy()
        joy.header.stamp = now
        joy.axes    = [0.5 * s, 0.5 * c, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        joy.buttons = [0] * 11
        self._pub_joy.publish(joy)

        # Joystick event
        je = JoystickEvent()
        je.joystick_name = 'Left Stick'
        je.x = 0.5 * s
        je.y = 0.5 * c
        self._pub_joy_ev.publish(je)

        # cmd_vel — gentle forward motion
        cv = Twist()
        cv.linear.x  = 0.1 * s
        cv.angular.z = 0.05 * c
        self._pub_cmd_vel.publish(cv)

        # Odometry
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x    = 0.1 * self._t * s
        odom.pose.pose.position.y    = 0.1 * self._t * c
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x    = 0.1 * s
        self._pub_odom.publish(odom)

        # IMU
        imu = Imu()
        imu.header.stamp            = now
        imu.header.frame_id         = 'imu_link'
        imu.orientation.w           = 1.0
        imu.angular_velocity.z      = 0.05 * s
        imu.linear_acceleration.z   = 9.81
        self._pub_imu.publish(imu)

        # Joint states — 4 wheels + head + 2 grabbers
        js = JointState()
        js.header.stamp = now
        js.name     = ['wheel_fl', 'wheel_fr', 'wheel_bl', 'wheel_br', 'head', 'grab_l', 'grab_r']
        js.position = [self._t, self._t, self._t, self._t, 0.0, 0.0, 0.0]
        js.velocity = [0.3 * s] * 4 + [0.0, 0.0, 0.0]
        self._pub_joints.publish(js)

    def _publish_2hz(self):
        # Controller status — always connected
        cs = ControllerStatus()
        cs.connected             = True
        cs.controller_type       = 'Xbox 360 (Fake)'
        cs.time_since_last_input = 0.1
        cs.timestamp             = time.time()
        self._pub_status.publish(cs)

    def _publish_1hz(self):
        # Battery state
        bs = BatteryState()
        bs.header.stamp    = self.get_clock().now().to_msg()
        bs.voltage         = 11.8
        bs.percentage      = 0.72
        bs.present         = True
        self._pub_battery.publish(bs)

    def _publish_button(self):
        # Simulate a brief A-button press then release
        for event_str in ['PRESSED', 'RELEASED']:
            be = ButtonEvent()
            be.button_name = 'A'
            be.event       = event_str
            be.button_id   = 0
            be.timestamp   = int(self.get_clock().now().nanoseconds / 1e9)
            self._pub_btn.publish(be)

    # ── Service callbacks ─────────────────────────────────────────────────────

    def _get_robot_actions_cb(self, request, response):
        self.get_logger().info(
            f'[GetRobotActions] called for user={request.user_name} — returning empty list')
        response.success       = True
        response.action_names  = []
        response.action_types  = []
        response.actions_json  = []
        return response

    def _get_mappings_cb(self, request, response):
        self.get_logger().info('[GetMappings] called — returning empty list')
        response.success       = True
        response.mapping_names = []
        response.message       = ''
        return response

    def _save_mapping_cb(self, request, response):
        self.get_logger().info(f'[SaveMapping] "{request.mapping_name}" — accepting')
        response.success = True
        response.message = 'OK (fake)'
        return response

    def _del_mapping_cb(self, request, response):
        self.get_logger().info(f'[DeleteRobotMapping] "{request.mapping_name}" — accepting')
        response.success = True
        response.message = 'OK (fake)'
        return response

    def _apply_mapping_cb(self, request, response):
        self.get_logger().info(f'[ApplyMapping] "{request.mapping_name}" — accepting')
        response.success               = True
        response.message               = 'OK (fake)'
        response.loaded_button_count   = len(request.button_names)
        response.loaded_joystick_count = len(request.joystick_names)
        return response

    def _save_action_cb(self, request, response):
        self.get_logger().info(f'[SaveRobotAction] "{request.action.name}" — accepting')
        response.success = True
        response.message = 'OK (fake)'
        return response

    def _del_action_cb(self, request, response):
        self.get_logger().info(f'[DeleteRobotAction] "{request.action_name}" — accepting')
        response.success = True
        response.message = 'OK (fake)'
        return response

    def _get_schemes_cb(self, request, response):
        self.get_logger().info(f'[GetRecordingSchemes] called for user={request.user_name} — returning empty list')
        response.success            = True
        response.scheme_names       = []
        response.scheme_topics_json = []
        response.message            = ''
        return response

    def _save_scheme_cb(self, request, response):
        self.get_logger().info(f'[SaveRecordingScheme] "{request.scheme_name}" — accepting')
        response.success = True
        response.message = 'OK (fake)'
        return response

    def _del_scheme_cb(self, request, response):
        self.get_logger().info(f'[DeleteRecordingScheme] "{request.scheme_name}" — accepting')
        response.success = True
        response.message = 'OK (fake)'
        return response

    def _set_led_cb(self, request, response):
        global _led_state
        _led_state = {
            'fl_mode':  request.fl_mode,  'fl_color': request.fl_color,
            'fr_mode':  request.fr_mode,  'fr_color': request.fr_color,
            'bl_mode':  request.bl_mode,  'bl_color': request.bl_color,
            'br_mode':  request.br_mode,  'br_color': request.br_color,
        }
        self.get_logger().info(
            f'[SetLedStatus] FL({request.fl_color},{request.fl_mode}) '
            f'FR({request.fr_color},{request.fr_mode}) '
            f'BL({request.bl_color},{request.bl_mode}) '
            f'BR({request.br_color},{request.br_mode})')
        response.success = True
        response.message = 'OK (fake)'
        return response

    def _get_led_cb(self, request, response):
        response.fl_mode  = _led_state['fl_mode'];  response.fl_color = _led_state['fl_color']
        response.fr_mode  = _led_state['fr_mode'];  response.fr_color = _led_state['fr_color']
        response.bl_mode  = _led_state['bl_mode'];  response.bl_color = _led_state['bl_color']
        response.br_mode  = _led_state['br_mode'];  response.br_color = _led_state['br_color']
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobotNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
