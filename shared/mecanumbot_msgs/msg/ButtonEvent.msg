# ButtonEvent.msg
# Published on /xbox_controller/button_events by xbox_controller_node.
# Replaces the std_msgs/String + JSON encoding.

string event        # PRESSED | HOLD | RELEASED
string button_name  # e.g. "A", "LB", "DPadUp"
int32  button_id    # index in the joy.buttons array; -1 for D-Pad
int32  timestamp    # ROS clock seconds
