# ActionTuple.msg
# One publish step inside an action.
# Used as ActionTuple[] inside ActionDescriptor and the sync/register services.

string topic         # ROS2 topic (empty for service calls)
string message       # JSON-encoded message payload
string message_type  # e.g. "geometry_msgs/msg/Twist"
float32 scale_x      # joystick x-axis scale factor (default 1.0)
float32 scale_y      # joystick y-axis scale factor (default 1.0)
float32 offset_x     # joystick x-axis offset       (default 0.0)
float32 offset_y     # joystick y-axis offset       (default 0.0)
string publish_type  # "topic" | "service"
string service_name  # ROS2 service name (empty for topic publishes)
