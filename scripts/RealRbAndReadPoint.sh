#!/bin/bash
rosrun rubik_cube_solve set_robot_enable_true.sh &
sleep 2
rosrun gripper_bridge gripper.sh &
wait
exit 0
