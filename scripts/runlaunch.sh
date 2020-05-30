#!/bin/bash
roslaunch rubik_cube_solve solve.launch &
sleep 0.1
roslaunch grasp_place grasp.launch &
wait
exit 0
