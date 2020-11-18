#!/usr/bin/env python

#Code shell copied from ROS Tutorial on listenr node and subscriber node

import rospy
import keyboard
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

from datetime import datetime


def callback(image_msg):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    cv_im = frame

    cv2.imshow("image", cv_im)
    cv2.waitKey()
    cv2.imwrite("/home/fizzer/ros_ws/src/Competition_Package/image_saver/nodes/raw_images/" + str(datetime.now()) + ".jpg", cv_im)
    print("saved image")

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('R1/pi_camera/image_raw', Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
