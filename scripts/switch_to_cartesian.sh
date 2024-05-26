#!/bin/bash

#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

ros2 control switch_controllers --activate cartesian_motion_controller motion_control_handle
