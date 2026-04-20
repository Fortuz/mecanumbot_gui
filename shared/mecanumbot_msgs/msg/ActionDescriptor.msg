# ActionDescriptor.msg
# A complete named action with its publish steps.
# Used in GetRobotActions, SaveRobotAction services.

string       name         # unique action name
string       action_type  # "button_once" | "button_hold" | "joystick"
ActionTuple[] tuples      # ordered list of publish steps
