#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
. ${SCRIPT_DIR}/deactivate_controllers.sh

ros2 control switch_controllers --activate velocity_controller

ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0
- 0
- 0
- 0
- 0
- 0
" -1
