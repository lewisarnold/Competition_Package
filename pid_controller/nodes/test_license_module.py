#!/usr/bin/env python

import license_plate_reader as lpr
import cv2

img = cv2.imread('/home/fizzer/ros_ws/src/Competition_Package/pid_controller/nodes/New_labeled/WN56.png',1)
read = lpr.Reader()
print('loaded')
license_string = read.license_read(img)
print('license_string')
