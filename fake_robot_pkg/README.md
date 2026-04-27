# fake_robot_pkg

A minimal ROS 2 Python package for **GUI development without a physical robot**.

It publishes the three controller topics the `mecanumbot_gui` expects, and provides
an interactive terminal CLI for simulating button and joystick input.

## Published topics

| Topic | Message type | Rate |
|---|---|---|
| `/controller/connection_status` | `mecanumbot_msgs/msg/ControllerStatus` | 1 Hz (heartbeat) |
| `/controller/button_events` | `mecanumbot_msgs/msg/ButtonEvent` | on demand |
| `/controller/joystick_events` | `mecanumbot_msgs/msg/JoystickEvent` | on demand |

## Build & run

```bash
# from your workspace root  (e.g. /home/adorjan/Documents/ws)
colcon build --packages-select fake_robot_pkg mecanumbot_msgs
source install/setup.bash

ros2 run fake_robot_pkg fake_controller
```

## Interactive CLI

```
╔══════════════════════════════════════════════════════════════╗
║          Fake Controller CLI – available commands            ║
╠══════════════════════════════════════════════════════════════╣
║  b   <button>                   press + release             ║
║  bh  <button>                   start holding               ║
║  br  <button>                   release held button         ║
║  bra                            release ALL held buttons    ║
║                                                              ║
║  j   <axis> <x> [<y>]           single joystick event       ║
║  ja  <axis> <x> [<y>] <n> [dt]  n events, dt s apart        ║
║                                                              ║
║  status                         show held buttons           ║
║  buttons                        list button names           ║
║  joysticks                      list joystick axes          ║
║  help / ?                       this help                   ║
║  quit / exit / q                exit                        ║
╚══════════════════════════════════════════════════════════════╝
```

### Examples

```
controller> b A               # tap A
controller> bh LB             # hold LB (sends HOLD at 10 Hz)
controller> br LB             # release LB
controller> bra               # release everything

controller> j ls 0.0 -1.0    # left stick full forward
controller> j lt 0.75         # left trigger at 75 %
controller> ja ls 0.0 0.0 20  # 20 neutral events (stick reset sweep)
```

### Button names (case-insensitive)

```
A  B  X  Y  LB  RB  BACK  START  HOME  LS  RS
DPadUp  DPadDown  DPadLeft  DPadRight
```

### Joystick axis aliases

| Alias | Full name |
|---|---|
| `ls` | Left Stick |
| `rs` | Right Stick |
| `lt` | Left Trigger (LT) |
| `rt` | Right Trigger (RT) |
