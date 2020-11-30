#! /usr/bin/env python

import os


PATH2 = "/home/fizzer/ros_ws/src/Competition_Package/pid_controller/nodes/CNN models/Artificial_Data"

files2 = os.listdir(PATH2)
folder2 = PATH2

for i in range(len(files2)):
	if(i%3 == 0):
		os.remove(PATH2 + '/' + files2[i])