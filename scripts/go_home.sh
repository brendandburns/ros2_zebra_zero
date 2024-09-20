#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
. ${SCRIPT_DIR}/deactivate_controllers.sh

ros2 control switch_controllers --activate zebra_zero

ros2 topic pub /zebra_zero/commands std_msgs/msg/Float64MultiArray "data:
- 0
- 1.57
- -1.57
- 0
- -1.57
- 0
" -1