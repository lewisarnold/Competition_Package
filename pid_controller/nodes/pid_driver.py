#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import cv2
from cv_bridge import CvBridge
from datetime import datetime, timedelta

import csv

TEAM_ID = "KL"
TEAM_PW = "123"

START = 1
STOP = 0

PEDESTRIAN_COLOUR_LOWER_BOUND = (50, 40, 20)
PEDESTRIAN_COLOUR_UPPER_BOUND = (110, 80, 70)

PEDESTRAIN_MONITOR_ZONE_X = (420, 812)
PEDESTRAIN_MONITOR_ZONE_Y = (410, 500)

PEDESTRAIN_WAIT = 6 #*0.5s

PEDESTRAIN_THRESHOLD = 10

CROSSWALK_COLOUR_LOWER_BOUND = (115, 250, 250)
CROSSWALK_COLOUR_UPPER_BOUND = (125, 255, 255)

CROSSWALK_MONITOR_ZONE_X = (638, 641)
CROSSWALK_MONITOR_ZONE_Y = (600, 719)

CROSSWALK_THRESHOLD = 10
MIN_TIME_BETWEEN_CROSSWALKS = 5 #s

FULL_SPEED_LINEAR = 0.425

ANGULAR_PROPORTIONAL = 0.15

ROAD_FINDING_AVERAGING_THRESHOLD = 110

ROAD_COLOUR_LOWER_BOUND = (70, 70, 70)
ROAD_COLOUR_UPPER_BOUND = (90, 90, 90)

PARKED_CAR_VISION_X = (280, 330)
PARKED_CAR_VISION_Y = (370, 470)

LICENSE_PLATE_CAR_VISION_X = (225, 380)
LICENSE_PLATE_CAR_VISION_Y = (383, 510)

LICENSE_PLATE_HEIGHT = 20

LICENSE_PLATE_THRESHOLD_X = 120
LICENSE_PLATE_THRESHOLD_Y = 20

PARKED_CAR_AVG_THRESHOLD_SINGLE = 140
PARKED_CAR_ORDER = [2,3,4,5,6,1]
PARKED_CAR_MINIMUM_INTERVAL = 20

PARKED_CAR_COLOUR_LOWER_BOUND_1 = (0, 0, 95)
PARKED_CAR_COLOUR_UPPER_BOUND_1 = (10, 10, 105)

PARKED_CAR_COLOUR_LOWER_BOUND_2 = (0, 0, 197)
PARKED_CAR_COLOUR_UPPER_BOUND_2 = (10, 10, 203)

PARKED_CAR_COLOUR_LOWER_BOUND_3 = (0, 0, 118)
PARKED_CAR_COLOUR_UPPER_BOUND_3 = (10, 10, 128)

DRIVE_VISION_STRIP = (450, 500)
STRIP_CROPPING_X = 500

NO_READING_LINEAR_DECREASE = 0.5
NO_READING_ANGULAR_VELOCITY = 20 * FULL_SPEED_LINEAR * NO_READING_LINEAR_DECREASE

SHOW_ROAD_VISION = 0
WAITKEY = 0

WAIT = True

class RobotDriver():
    def drive_robot(self):
        self.last_side_left = True
        self.velocity_command_publisher = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
        self.license_plate_publisher = rospy.Publisher('/license_plate', String, queue_size=1)

        self.bridge = CvBridge()

        self.parked_car_interval_counter = 0
        self.parked_car_counter = 0

        self.last_crosswalk_time = datetime.now() - timedelta(seconds=MIN_TIME_BETWEEN_CROSSWALKS)
        self.pedestrian_aside = True
        self.in_crosswalk = False

        #MUST BE REMOVED FOR COMPETITION
        self.true_plates =  self.init_plate_value()

        rospy.init_node('pid_commander')

        self.rate = rospy.Rate(2)

        if WAIT:
            rospy.sleep(15)

        self.activate_timer(START)
        self.initial_turn_sequence()

        self.subscriber = rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.on_image_recieve)

        rospy.spin()


    def on_image_recieve(self, camera_image_raw):
        self.cv_raw = self.bridge.imgmsg_to_cv2(camera_image_raw, "bgr8")

        velocity_command = self.standard_drive_velocity(int(self.cv_raw.shape[1] / 2))

        safe_to_drive, just_stopped = self.safe_to_drive_through_crosswalk()

        if not safe_to_drive:
            velocity_command.linear.x = 0
            velocity_command.angular.z = 0
            self.velocity_command_publisher.publish(velocity_command)
            if just_stopped:
                self.subscriber.unregister()
                rospy.sleep(2)
                self.subscriber = rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.on_image_recieve)
        else:
            self.velocity_command_publisher.publish(velocity_command)

        self.find_license_plate()

    def safe_to_drive_through_crosswalk(self):
        if not self.pedestrian_aside:
            self.pedestrian_aside = self.pedestrian_clear()
            return self.pedestrian_aside, False
        elif self.at_crosswalk():
            self.pedestrian_aside = self.pedestrian_clear()
            return self.pedestrian_aside, True
        else:
            return True, False

    def at_crosswalk(self):
        time_change = datetime.now() - self.last_crosswalk_time
        if time_change.seconds < MIN_TIME_BETWEEN_CROSSWALKS:
            return False

        monitor_zone = self.cv_raw[CROSSWALK_MONITOR_ZONE_Y[0]:CROSSWALK_MONITOR_ZONE_Y[1],
                       CROSSWALK_MONITOR_ZONE_X[0]:CROSSWALK_MONITOR_ZONE_X[1]]

        hsv = cv2.cvtColor(monitor_zone, cv2.COLOR_RGB2HSV)

        mask = cv2.inRange(hsv, CROSSWALK_COLOUR_LOWER_BOUND, CROSSWALK_COLOUR_UPPER_BOUND)

        if np.average(mask) > CROSSWALK_THRESHOLD:
            self.last_crosswalk_time = datetime.now()
            return True

        else:

            return False

    def pedestrian_clear(self):
        danger_zone = self.cv_raw[PEDESTRAIN_MONITOR_ZONE_Y[0]:PEDESTRAIN_MONITOR_ZONE_Y[1],
                          PEDESTRAIN_MONITOR_ZONE_X[0]:PEDESTRAIN_MONITOR_ZONE_X[1]]

        mask = cv2.inRange(danger_zone, PEDESTRIAN_COLOUR_LOWER_BOUND, PEDESTRIAN_COLOUR_UPPER_BOUND)

        # pts = np.array([[PEDESTRAIN_MONITOR_ZONE_X[0], PEDESTRAIN_MONITOR_ZONE_Y[0]],
        #                 [PEDESTRAIN_MONITOR_ZONE_X[0], PEDESTRAIN_MONITOR_ZONE_Y[1]],
        #                 [PEDESTRAIN_MONITOR_ZONE_X[1], PEDESTRAIN_MONITOR_ZONE_Y[1]],
        #                 [PEDESTRAIN_MONITOR_ZONE_X[1], PEDESTRAIN_MONITOR_ZONE_Y[0]]])
        #
        # pts = pts.reshape((-1, 1, 2))
        #
        # #cv2.imshow("", cv2.polylines(self.cv_raw, [pts], True, (255, 0, 0), 1))
        # cv2.imshow("mask", mask)
        # cv2.waitKey(1)

        for i in range (mask.shape[1]):
            if np.average(mask[:,i]) > PEDESTRAIN_THRESHOLD:
                return False

        return True

    def activate_timer(self, start):
        if start:
            message = String()
            message.data = TEAM_ID + "," + TEAM_PW + "," + str(0) + "," + "AAAA"
            self.license_plate_publisher.publish(message)
        else:
            message = String()
            message.data = TEAM_ID + "," + TEAM_PW + "," + str(-1) + "," + "ZZZZ"
            self.license_plate_publisher.publish(message)

    def publish_plate(self, location, plate_number):
        message = String()
        message.data = TEAM_ID + "," + TEAM_PW + "," + str(location) + "," + plate_number
        self.license_plate_publisher.publish(message)

    def find_license_plate(self):
        found_car, location = self.found_parked_car()
        if found_car:
            print("car")
            left, right, top = self.find_edge_of_label()

            license_plate = self.cv_raw[LICENSE_PLATE_CAR_VISION_Y[0] + top:LICENSE_PLATE_CAR_VISION_Y[0] + top +
                                                                    LICENSE_PLATE_HEIGHT,
                    LICENSE_PLATE_CAR_VISION_X[0] + left: LICENSE_PLATE_CAR_VISION_X[0] + right]

            cv2.imwrite("/home/fizzer/ros_ws/src/2020T1_competition/pid_controller/nodes/At4Vel/"
                        + str(self.true_plates[location-1]) + ".png", license_plate)


            self.publish_plate(location, str(self.true_plates[location-1]))

            # pts = np.array([[LICENSE_PLATE_CAR_VISION_X[0] + left, LICENSE_PLATE_CAR_VISION_Y[0] + top],
            #        [LICENSE_PLATE_CAR_VISION_X[0] + left, LICENSE_PLATE_CAR_VISION_Y[0] + top + LICENSE_PLATE_HEIGHT],
            #        [LICENSE_PLATE_CAR_VISION_X[0] + right, LICENSE_PLATE_CAR_VISION_Y[0] + top + LICENSE_PLATE_HEIGHT],
            #        [LICENSE_PLATE_CAR_VISION_X[0] + right, LICENSE_PLATE_CAR_VISION_Y[0] + top]])
            #
            # pts = pts.reshape((-1, 1, 2))
            #
            # # cv2.imshow("", cv2.polylines(self.cv_raw, [pts], True, (255, 0, 0), 1))
            # cv2.imshow("main", self.cv_raw)
            # cv2.imshow(str(location), license_plate)
            # cv2.waitKey(1)

    ## CANNOT BE USED IN COMPETITION
    def init_plate_value(self):
        LICENSE_PLATE_FILE = '/home/fizzer/ros_ws/src/2020T1_competition/enph353/enph353_gazebo/scripts/plates.csv'
        with open(LICENSE_PLATE_FILE, "r") as plate_file:
            platereader = csv.reader(plate_file)
            true_plates = []
            i = 0
            for row in platereader:
                true_plates.append(row[0])
        return true_plates


    def find_edge_of_label(self):
        vision_section = self.cv_raw[LICENSE_PLATE_CAR_VISION_Y[0]:LICENSE_PLATE_CAR_VISION_Y[1],
                              LICENSE_PLATE_CAR_VISION_X[0]:LICENSE_PLATE_CAR_VISION_X[1]]

        hsv = cv2.cvtColor(vision_section, cv2.COLOR_RGB2HSV)

        car_mask_1 = cv2.inRange(hsv, PARKED_CAR_COLOUR_LOWER_BOUND_1, PARKED_CAR_COLOUR_UPPER_BOUND_1)
        car_mask_2 = cv2.inRange(hsv, PARKED_CAR_COLOUR_LOWER_BOUND_2, PARKED_CAR_COLOUR_UPPER_BOUND_2)
        car_mask_3 = cv2.inRange(hsv, PARKED_CAR_COLOUR_LOWER_BOUND_3, PARKED_CAR_COLOUR_UPPER_BOUND_3)

        license_plate_mask = car_mask_1 + car_mask_2 + car_mask_3

        width = license_plate_mask.shape[1]

        column_average = np.zeros(width)

        for k in range(width):
            column_average[k] = np.average(license_plate_mask[:, k])

        left = 0
        set_left = False
        for j in range(width):
            if column_average[j] > LICENSE_PLATE_THRESHOLD_X:
                left = j
                set_left = True
                break

        right = width - 1
        set_right = False
        for j in reversed(range(left, width)):
            if column_average[j] > LICENSE_PLATE_THRESHOLD_X:
                right = j
                set_right = True
                break

        height = license_plate_mask.shape[0]

        row_average = np.zeros(height)

        for k in range(height):
            row_average[k] = np.average(license_plate_mask[k, :])

        top = 0
        set_top = False
        for j in range(height):
            if row_average[j] < LICENSE_PLATE_THRESHOLD_Y:
                top = j
                set_top = True
                break
        return left, right, top

    def found_parked_car(self):
        self.parked_car_interval_counter = self.parked_car_interval_counter + 1

        vision_section = self.cv_raw[PARKED_CAR_VISION_Y[0]:PARKED_CAR_VISION_Y[1],
                              PARKED_CAR_VISION_X[0]:PARKED_CAR_VISION_X[1]]

        hsv = cv2.cvtColor(vision_section, cv2.COLOR_RGB2HSV)

        car_mask_1 = cv2.inRange(hsv, PARKED_CAR_COLOUR_LOWER_BOUND_1, PARKED_CAR_COLOUR_UPPER_BOUND_1)
        car_mask_2 = cv2.inRange(hsv, PARKED_CAR_COLOUR_LOWER_BOUND_2, PARKED_CAR_COLOUR_UPPER_BOUND_2)
        car_mask_3 = cv2.inRange(hsv, PARKED_CAR_COLOUR_LOWER_BOUND_3, PARKED_CAR_COLOUR_UPPER_BOUND_3)

        car_mask_total = car_mask_1 + car_mask_2 + car_mask_3

        vision_section_avg = np.average(car_mask_total)

        #this_cars_threshold = PARKED_CAR_AVG_THRESHOLD.get(PARKED_CAR_ORDER[self.parked_car_counter])

        #print(self.parked_car_counter)

        if vision_section_avg >  PARKED_CAR_AVG_THRESHOLD_SINGLE and self.parked_car_interval_counter > PARKED_CAR_MINIMUM_INTERVAL:
            location_id = PARKED_CAR_ORDER[self.parked_car_counter]
            self.parked_car_counter = 1 + self.parked_car_counter
            if self.parked_car_counter >= len(PARKED_CAR_ORDER):
                self.parked_car_counter = 0

            self.parked_car_interval_counter = 0
            return True, location_id

        # if vision_section_avg >  this_cars_threshold and self.parked_car_interval_counter > PARKED_CAR_MINIMUM_INTERVAL:
        #     print(PARKED_CAR_ORDER[self.parked_car_counter])
        #     self.parked_car_counter = 1 + self.parked_car_counter
        #     if self.parked_car_counter >= len(PARKED_CAR_ORDER):
        #         self.parked_car_counter = 0
        #
        #     self.parked_car_interval_counter = 0
        #     return True

        else:
            return False, -1

            # vision_section_real = self.cv_raw[PARKED_CAR_VISION_Y[0]:PARKED_CAR_VISION_Y[1],
            #                       PARKED_CAR_VISION_X[0]:PARKED_CAR_VISION_X[1]]
            # cv2.imshow("vision section", vision_section_real)

        # pts = np.array([[PARKED_CAR_VISION_X[0], PARKED_CAR_VISION_Y[0]],
        #        [PARKED_CAR_VISION_X[0], PARKED_CAR_VISION_Y[1]],
        #        [PARKED_CAR_VISION_X[1], PARKED_CAR_VISION_Y[1]],
        #        [PARKED_CAR_VISION_X[1], PARKED_CAR_VISION_Y[0]]])
        #
        # pts = pts.reshape((-1, 1, 2))
        #
        # car_mask_total_with_vs = cv2.polylines(self.cv_raw, [pts], True, (255, 0, 0), 3)

    def initial_turn_sequence(self):
        velocity_command = Twist()

        velocity_command.angular.z = 0.5
        velocity_command.linear.x = FULL_SPEED_LINEAR

        self.velocity_command_publisher.publish(velocity_command)

    def standard_drive_velocity(self, centre_setpoint):
        centre_x, center_found = self.find_road_center()

        velocity_command = Twist()

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

        if SHOW_ROAD_VISION:
            marked_with_detected_centre = cv2.circle(self.cv_raw, (
            centre_x, int((DRIVE_VISION_STRIP[0] + DRIVE_VISION_STRIP[1]) / 2)), 20,
                                                     (255, 255, 255), 20)
            marked_with_both_centres = cv2.circle(marked_with_detected_centre, (
            centre_setpoint, int((DRIVE_VISION_STRIP[0] + DRIVE_VISION_STRIP[1]) / 2)), 20,
                                                  (150, 250, 250), 20)
            cv2.imshow("Robot View", marked_with_both_centres)
            if WAITKEY:
                cv2.waitKey()
            else:
                cv2.waitKey(1)

        return velocity_command

    def find_road_center(self):
        strip = self.cv_raw[DRIVE_VISION_STRIP[0]:DRIVE_VISION_STRIP[1], STRIP_CROPPING_X:-STRIP_CROPPING_X]
        cropped_strip = cv2.inRange(strip, ROAD_COLOUR_LOWER_BOUND, ROAD_COLOUR_UPPER_BOUND)

        # cv2.imshow("mask", cropped_strip)
        # cv2.waitKey(1)

        width = cropped_strip.shape[1]

        column_average = np.zeros(width)

        for k in range(width):
            column_average[k] = np.average(cropped_strip[:, k])

        left = 0
        set_left = False
        for j in range(width):
            if column_average[j] > ROAD_FINDING_AVERAGING_THRESHOLD:
                left = j
                set_left = True
                break

        right = width - 1
        set_right = False
        for j in reversed(range(left, width)):
            if column_average[j] > ROAD_FINDING_AVERAGING_THRESHOLD:
                right = j
                set_right = True
                break

        centre_x = int((left + right) / 2)
        successfully_found_center = set_left and set_right
        return centre_x + STRIP_CROPPING_X, successfully_found_center


if __name__ == '__main__':
    robot_driver = RobotDriver()
    robot_driver.drive_robot()
