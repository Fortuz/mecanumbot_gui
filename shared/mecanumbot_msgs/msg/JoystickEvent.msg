# JoystickEvent.msg
# Published on /xbox_controller/joystick_events by xbox_controller_node.
# Replaces the std_msgs/String + JSON encoding.

string joystick_name  # "Left Stick" | "Right Stick" | "Left Trigger (LT)" | "Right Trigger (RT)"
float32 x             # horizontal axis value, range -1.0 … 1.0 (triggers: 0.0 … 1.0)
float32 y             # vertical   axis value, range -1.0 … 1.0 (triggers: always 0.0)
int32   timestamp     # ROS clock seconds
