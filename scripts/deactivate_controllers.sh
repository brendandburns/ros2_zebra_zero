#!/bin/bash

controllers="zebra_zero velocity_controller trajectory_controller cartesian_motion_controller zerog_controller"

for controller in $controllers; do
    echo "Deactivating $controller"
done

ros2 control switch_controllers --deactivate $controllers
