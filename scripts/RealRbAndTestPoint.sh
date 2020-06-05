#!/bin/bash
rosrun rubik_cube_solve set_robot_enable_true.sh &
sleep 2
rosrun grasp_place test.py &
sleep 0.5
roslaunch grasp_place grasp.launch &
sleep 0.5
rosrun cubeParse cube &
wait
exit 0
