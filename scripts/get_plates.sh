#!/bin/bash

source ~/ros_ws/devel/setup.bash

cd ~/ros_ws/src/2020T1_competition/enph353/enph353_utils/scripts

for i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15

do
	timeout 28 ./run_sim.sh -vpg

	killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient

	sleep 1

	killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient	

	sleep 1
done