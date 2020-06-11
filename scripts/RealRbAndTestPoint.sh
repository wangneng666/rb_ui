#!/bin/bash
roslaunch co605_dual_arm_gripper_moveit_config demo.launch &
sleep 2
roslaunch rb_ui dualRobotLaunch.launch
wait
exit 0

