# ControllerStatus.msg
# Published on /xbox_controller/connection_status by xbox_controller_node.
# Replaces the std_msgs/String + JSON encoding.

bool    connected             # true if a joy message was received within the last 3 s
string  controller_type       # e.g. "Xbox 360"
float32 time_since_last_input # seconds since the last /joy message
int32   timestamp             # ROS clock seconds
