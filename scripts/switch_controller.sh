#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
. ${SCRIPT_DIR}/deactivate_controllers.sh

controller=$1
if [ -z "$controller" ]; then
    echo "Usage: $0 <controller>"
    exit 1
fi

echo "Activating $controller"
ros2 control switch_controllers --activate $controller
