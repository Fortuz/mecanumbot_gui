# ActionDescriptor.msg
# A complete named action with its publish steps.
# Used as ActionDescriptor[] in SyncActions.

string       name         # unique action name
string       action_type  # "button_once" | "button_hold" | "joystick"
ActionTuple[] tuples      # ordered list of publish steps
