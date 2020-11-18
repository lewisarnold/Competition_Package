#!/usr/bin/env python

import license_plate_reader as lpr
import cv2

def main(img):
    print(type(img[0,0,0]))
    read = lpr.Reader()
    print('loaded')
    license_string = read.license_read(img)
    print(license_string)
    return license_string


if __name__ == '__main__':
	main(cv2.imread('/home/fizzer/ros_ws/src/Competition_Package/pid_controller/nodes/New_labeled/WN56.png',1))