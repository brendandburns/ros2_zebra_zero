#!/bin/bash

#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

$SCRIPT_DIR/deactivate_controllers.sh
ros2 control switch_controllers --activate zerog_controller
