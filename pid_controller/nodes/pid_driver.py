#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

FULL_SPEED_LINEAR = 0.6

ANGULAR_PROPORTIONAL = 0.15

AVERAGING_THRESHOLD = 110

ROAD_COLOUR_LOWER_BOUND = (70, 70, 70)
ROAD_COLOUR_UPPER_BOUND = (90, 90, 90)

DRIVE_VISION_STRIP = (450, 500)
STRIP_CROPPING_X = 500

NO_READING_LINEAR_DECREASE = 0.5
NO_READING_ANGULAR_VELOCITY = 20 * FULL_SPEED_LINEAR * NO_READING_LINEAR_DECREASE


class RobotDriver():
    def drive_robot(self):
        self.last_side_left = True
        self.velocity_command_publisher = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)

        self.bridge = CvBridge()

        rospy.init_node('pid_commander')
        rospy.sleep(1)

        self.initial_turn_sequence()

        rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.on_image_recieve)
        self.rate = rospy.Rate(2)

        rospy.spin()

    def initial_turn_sequence(self):
        velocity_command = Twist()

        velocity_command.angular.z = 0.5
        velocity_command.linear.x = FULL_SPEED_LINEAR
        #rospy.sleep(0.05)

        self.velocity_command_publisher.publish(velocity_command)

    def on_image_recieve(self, camera_image_raw):
        cv_raw = self.bridge.imgmsg_to_cv2(camera_image_raw, "bgr8")

        road_detection_mask = cv2.inRange(cv_raw, ROAD_COLOUR_LOWER_BOUND, ROAD_COLOUR_UPPER_BOUND)

        velocity_command = Twist()

        centre_x, center_found = self.find_road_center(road_detection_mask)
        centre_setpoint = int(cv_raw.shape[1] / 2)

        if not center_found:
            if self.last_side_left:
                velocity_command.angular.z = NO_READING_ANGULAR_VELOCITY
            else:
                velocity_command.angular.z = -NO_READING_ANGULAR_VELOCITY

            velocity_command.linear.x = FULL_SPEED_LINEAR * NO_READING_LINEAR_DECREASE

        else:
            velocity_command.linear.x = FULL_SPEED_LINEAR
            velocity_command.angular.z = FULL_SPEED_LINEAR * ANGULAR_PROPORTIONAL * (centre_setpoint - centre_x)
            if centre_x > centre_setpoint:
                self.last_side_left = False
            else:
                self.last_side_left = True

        self.velocity_command_publisher.publish(velocity_command)

        marked = cv2.circle(cv_raw, (centre_x, int((DRIVE_VISION_STRIP[0] + DRIVE_VISION_STRIP[1]) / 2)), 20,
                            (255, 255, 255), 20)
        marked = cv2.circle(marked, (centre_setpoint, int((DRIVE_VISION_STRIP[0] + DRIVE_VISION_STRIP[1]) / 2)), 20,
                            (150, 250, 250), 20)
        cv2.imshow("marked", marked)
        cv2.waitKey(1)


    def find_road_center(self, road_mask):
        cropped_strip = road_mask[DRIVE_VISION_STRIP[0]:DRIVE_VISION_STRIP[1], STRIP_CROPPING_X:-STRIP_CROPPING_X]
        cv2.imshow("mask", cropped_strip)

        width = cropped_strip.shape[1]

        column_average = np.zeros(width)

        for k in range(width):
            column_average[k] = np.average(cropped_strip[:, k])

        left = 0
        set_left = False
        for j in range(width):
            if column_average[j] > AVERAGING_THRESHOLD:
                left = j
                set_left = True
                break

        right = width - 1
        set_right = False
        for j in reversed(range(left, width)):
            if column_average[j] > AVERAGING_THRESHOLD:
                right = j
                set_right = True
                break

        centre_x = int((left + right) / 2)
        successfully_found_center = set_left and set_right
        return centre_x + STRIP_CROPPING_X, successfully_found_center

        # marked = cv2.circle(road_mask, (centre, int((DRIVE_VISION_STRIP[0] + DRIVE_VISION_STRIP[1]) / 2)), 20, (150, 250, 250))
        # cv2.imshow("marked", marked)
        # cv2.waitKey()


if __name__ == '__main__':
    robot_driver = RobotDriver()
    robot_driver.drive_robot()
