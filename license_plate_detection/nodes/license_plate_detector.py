#!/usr/bin/env python

#Code shell copied from ROS Tutorial on listener node and subscriber node

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge

'''
team ID: max 8 characters (no spaces)
team password: max 8 characters (no spaces)
license plate location: int (1 to 8); (use 0 to start timer and -1 to stop)
license plate id: 4 characters (no spaces)

example: str('TeamRed,multi21,4,XR58')

'''
teamID = sdfsfd
team_pass = dfghrfd


def callback(ros_img):
    img = bridge.imgmsg_to_cv2(ros_img, "bgr8")
    #run plate location detection
    location = 
    #run plate id dection
    plate_id = 

    message = str(teamID,team_pass,location,plate_id)
    pub.publish(message)

def listener():

	bridge = CvBridge()

    rospy.init_node('license_plate_detector', anonymous=True)

    pub = rospy.Publisher('/licence_plate', String)
    sub = rospy.Subscriber('R1/pi_camera/image_raw', Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
