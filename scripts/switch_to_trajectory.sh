#!/bin/bash

ros2 control switch_controllers --deactivate zebra_zero
ros2 control switch_controllers --deactivate velocity_controller
ros2 control switch_controllers --deactivate trajectory_controller

ros2 control switch_controllers --activate trajectory_controller
