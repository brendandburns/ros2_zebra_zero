#!/bin/bash

ros2 control switch_controllers --deactivate zebra_zero
ros2 control switch_controllers --deactivate velocity_controller
ros2 control switch_controllers --deactivate trajectory_controller

ros2 control switch_controllers --activate zebra_zero

ros2 topic pub /zebra_zero/commands std_msgs/msg/Float64MultiArray "data:
- 0
- 1.57
- -1.57
- 0
- -1.57
- 0
" -1