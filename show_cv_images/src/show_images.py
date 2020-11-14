#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


WAITKEY = 1
HSV = 0

TRUCK_COLOUR_LOWER_BOUND = (30, 30, 30)
TRUCK_COLOUR_UPPER_BOUND = (60, 60, 60)
TRUCK_MONITOR_ZONE_X = (700, 830)
TRUCK_MONITOR_ZONE_Y = (389, 400)




class Drive_With_Images():
    def drive_robot(self):

        self.bridge = CvBridge()

        rospy.init_node('image_shower')

        self.subscriber = rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.on_image_recieve)

        rospy.spin()


    def on_image_recieve(self, camera_image_raw):
        img = self.bridge.imgmsg_to_cv2(camera_image_raw, "bgr8")
        #img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        vision_square = img[TRUCK_MONITOR_ZONE_Y[0]:TRUCK_MONITOR_ZONE_Y[1],
                        TRUCK_MONITOR_ZONE_X[0]:TRUCK_MONITOR_ZONE_X[1]]

        mask = cv2.inRange(vision_square, TRUCK_COLOUR_LOWER_BOUND, TRUCK_COLOUR_UPPER_BOUND)
        print(np.average(mask))


        pts = np.array([[TRUCK_MONITOR_ZONE_X[0], TRUCK_MONITOR_ZONE_Y[0]],
                        [TRUCK_MONITOR_ZONE_X[0], TRUCK_MONITOR_ZONE_Y[1]],
                        [TRUCK_MONITOR_ZONE_X[1], TRUCK_MONITOR_ZONE_Y[1]],
                        [TRUCK_MONITOR_ZONE_X[1], TRUCK_MONITOR_ZONE_Y[0]]])

        pts = pts.reshape((-1, 1, 2))

        cv2.imshow("", cv2.polylines(img, [pts], True, (255, 0, 0), 1))
        cv2.imshow("mask", mask)
        cv2.waitKey()


if __name__ == '__main__':
    robot_driver = Drive_With_Images()
    robot_driver.drive_robot()
