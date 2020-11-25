#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import cv2
from cv_bridge import CvBridge
import license_plate_reader

# For reading the true values of license plates.  Can be removed for competition
import csv

# Standard forward drive speed 0.425 is maximum reliable speed as of 14 November
FULL_SPEED_LINEAR = 0.425

# A multiplier for the angular speed.  Only the forward speed needs to be adjusted and the angular speed is
# automatically scaled accordingly
ANGULAR_PROPORTIONAL = 0.15

# Whether to delay on start or not.  Should be true if being run from a script, false if run by user after world
# has loaded
WAIT = False

# Parameters to display image feed
# Show a feed of images from the camera to see what the robot sees
SHOW_ROAD_VISION = False

# If SHOW_ROAD_VISION is engaged, this decides whether or not to wait on each image for a key press or just show a
# stream
WAITKEY = False

# Parameters for the timer
TEAM_ID = "KL"
TEAM_PW = "123"

# Codes for the activate_timer function
START = 1
STOP = 0

# Parameters for Finding the Truck

TRUCK_THRESHOLD_CORNER = 55
TRUCK_THRESHOLD_TOP = 100
TRUCK_THRESHOLD_BOTTOM = 10

# In BGR
TRUCK_COLOUR_LOWER_BOUND_CORNER = (30, 30, 30)
TRUCK_COLOUR_UPPER_BOUND_CORNER = (60, 60, 60)

TRUCK_MONITOR_ZONE_CORNER_X = (700, 830)
TRUCK_MONITOR_ZONE_CORNER_Y = (389, 400)

TRUCK_COLOUR_LOWER_BOUND_MIDDLE_AREA = (70, 70, 70)
TRUCK_COLOUR_UPPER_BOUND_MIDDLE_AREA = (100, 100, 100)

TRUCK_MONITOR_ZONE_TOP_X = (576, 593)
TRUCK_MONITOR_ZONE_TOP_Y = (365, 369)

TRUCK_MONITOR_ZONE_BOTTOM_X = (556, 593)
TRUCK_MONITOR_ZONE_BOTTOM_Y = (389, 400)

# Parameters for turning into the center
TURNING_MONITOR_ZONE_X = (300, 350)
TURNING_MONITOR_ZONE_Y = (400, 450)

TURNING_THRESHOLD = 200

CARS_ON_INSIDE = 6

# Hardcode turning angular speed
TURNING_OVERRIDE = 2.9 * FULL_SPEED_LINEAR

# Sim time it takes to get to the position to wait and look for truck
FIRST_HALF_TURNING_TIME = 1.0

# Sim time it takes to complete the turn from the waiting spot
SECOND_HALF_TURNING_TIME = 0.6

# Parameters for the pedestrian monitor
PEDESTRIAN_COLOUR_LOWER_BOUND = (50, 40, 20)
PEDESTRIAN_COLOUR_UPPER_BOUND = (110, 80, 70)

PEDESTRIAN_MONITOR_ZONE_X = (420, 812)
PEDESTRIAN_MONITOR_ZONE_Y = (410, 500)

PEDESTRIAN_THRESHOLD = 10

# Parameters for the Crosswalk Detector
CROSSWALK_COLOUR_LOWER_BOUND = (115, 250, 250)
CROSSWALK_COLOUR_UPPER_BOUND = (125, 255, 255)

CROSSWALK_MONITOR_ZONE_X = (638, 641)
CROSSWALK_MONITOR_ZONE_Y = (600, 719)

CROSSWALK_THRESHOLD = 10

# Minimum seconds in sim time between crosswalks so that the second red line at the same crosswalk isn't flagged as
# another crosswalk
MIN_TIME_BETWEEN_CROSSWALKS = 8

# Parameters for the road following
ROAD_FINDING_AVERAGING_THRESHOLD = 110

ROAD_COLOUR_LOWER_BOUND = (70, 70, 70)
ROAD_COLOUR_UPPER_BOUND = (90, 90, 90)

DRIVE_VISION_STRIP = (450, 500)

# The number of pixels to crop off each side of the drive vision strip
STRIP_CROPPING_X = 500

# The amount to decrease linear speed if not road is detected at all
NO_READING_LINEAR_DECREASE = 0.5
# The angular velocity if no road is detected at all
NO_READING_ANGULAR_VELOCITY = 20 * FULL_SPEED_LINEAR * NO_READING_LINEAR_DECREASE

# Parameters for the Parked Car Detector
PARKED_CAR_VISION_LEFT_X = (280, 330)
PARKED_CAR_VISION_LEFT_Y = (370, 470)

PARKED_CAR_VISION_RIGHT_X = (950, 1000)
PARKED_CAR_VISION_RIGHT_Y = (370, 470)

PARKED_CAR_AVG_THRESHOLD_SINGLE = 140

# The preplanned route: which parking locations it will see in order
PARKED_CAR_ORDER = [2, 3, 4, 5, 6, 1, 7, 8]

# Minimum tuime between parked cars
MIN_TIME_BETWEEN_CARS = 2

PARKED_CAR_COLOUR_LOWER_BOUND_1 = (0, 0, 95)
PARKED_CAR_COLOUR_UPPER_BOUND_1 = (10, 10, 105)

PARKED_CAR_COLOUR_LOWER_BOUND_2 = (0, 0, 197)
PARKED_CAR_COLOUR_UPPER_BOUND_2 = (10, 10, 203)

PARKED_CAR_COLOUR_LOWER_BOUND_3 = (0, 0, 118)
PARKED_CAR_COLOUR_UPPER_BOUND_3 = (10, 10, 128)

# Parameters for the License PLate Locator
LICENSE_PLATE_CAR_VISION_LEFT_X = (225, 380)
LICENSE_PLATE_CAR_VISION_LEFT_Y = (383, 510)

LICENSE_PLATE_CAR_VISION_RIGHT_X = (900, 1055)
LICENSE_PLATE_CAR_VISION_RIGHT_Y = (383, 510)

LICENSE_PLATE_HEIGHT = 20

LICENSE_PLATE_THRESHOLD_X = 120
LICENSE_PLATE_THRESHOLD_Y = 20

LEFT = 0
RIGHT = 1

# Parameters for saving license Plates
SAVE_DATA_PLATES = False
DATA_PLATE_FILE_PATH = "/home/fizzer/ros_ws/src/Competition_Package/pid_controller/nodes/LotsOFData/"
ERROR_PLATE_FILE_PATH = "/home/fizzer/ros_ws/src/Competition_Package/pid_controller/nodes/ERRORS/"


class RobotDriver():
    """
    Defines an object which holds all relevant parameters and information for driving the robot
    Robot can be driven with drive_robot()
    """


    def __init__(self):
        """
        Sets up all the parameters for the class
        """
        rospy.init_node('Main_Control_Node')

        self.velocity_command_publisher = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
        self.license_plate_publisher = rospy.Publisher('/license_plate', String, queue_size=1)

        self.image_to_cv_bridge = CvBridge()

        # a parameter to remember the last side the road was on when no road can be seen at all
        self.last_side_left = True

        self.parked_car_interval_counter = 0
        self.parked_car_counter = 0

        self.last_crosswalk_time = rospy.get_time() - MIN_TIME_BETWEEN_CROSSWALKS
        self.crosswalk_count = 0

        self.last_car_time = rospy.get_time() - MIN_TIME_BETWEEN_CARS

        # the time when a turning operation was commenced
        self.start_turning_time = None
        self.turning_initiated = False
        self.completed_turn = False

        # whether the pedestrian is clear from the road
        self.pedestrian_aside = True

        # True when we go to the inside loop
        self.on_inside = False

        # True when the truck has been spotted in the designated spot
        self.truck_known = False

        # True when the course has been run
        self.finished = False

        # True when timer stopped
        self.timer_stopped = False

        # MUST BE REMOVED FOR COMPETITION
        self.true_plates = self.init_plate_value()

        self.License_Plate_Reader =  license_plate_reader.Reader()

        self.rate = rospy.Rate(2)

        # To hold the current image
        self.cv_raw = None

        # Preparing a parameter to hold the subscriber
        self.image_subscriber = None

        if WAIT:
            rospy.sleep(15)

    def drive_robot(self):
        """
        The main method to drive the robot
        Starts timer and then drivers the robot
        """
        # Start the timer
        self.activate_timer(START)

        # Hardcode turn sequence so robot goes left off the start
        self.initial_turn_sequence()

        # Subscribe to images
        self.image_subscriber = rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.on_image_receive)

        # Control now happens from the on_image_receive callback
        while not self.timer_stopped and not rospy.core.is_shutdown():
            rospy.rostime.wallsleep(0.5)
        #rospy.spin()

    def on_image_receive(self, camera_image_raw):
        """
        The main control function to drive based on a new image
        :param camera_image_raw: the new image in message format
        :return: None
        """
        if self.timer_stopped:
            return

        # Bridge to cv and then hold it so all helpers can access it
        self.cv_raw = self.image_to_cv_bridge.imgmsg_to_cv2(camera_image_raw, "bgr8")

        # get the standard drive velocity based on the road following
        velocity_command = self.standard_drive_velocity(int(self.cv_raw.shape[1] / 2))

        # While on the outside loop
        if not self.on_inside:
            # If we've already started turning or it is now time to turn
            if self.turning_initiated or self.parked_car_counter >= CARS_ON_INSIDE and self.turn_available():
                # the hardcode turning velocity
                velocity_command.angular.z = TURNING_OVERRIDE

                # If we've just started turning, set the parameters accordingly
                if not self.turning_initiated:
                    self.turning_initiated = True
                    self.start_turning_time = rospy.get_time()

                # If we've completed first half of turn, set that we're now inside.  Control shifts to the inside
                # sequence
                if rospy.get_time() - self.start_turning_time > FIRST_HALF_TURNING_TIME:
                    self.on_inside = True

            # Check for the pedestrian
            safe_to_drive, just_stopped = self.safe_to_drive_through_crosswalk()

            if not safe_to_drive:
                # Immediately stop
                velocity_command.linear.x = 0
                velocity_command.angular.z = 0
                self.velocity_command_publisher.publish(velocity_command)

                # If we've just slammed on the brakes, the images will be of the ground as the robot rocks forward
                # We need to unsubscribe, wait, and resubscribe so the images are back to being of the road ahead
                if just_stopped:
                    self.image_subscriber.unregister()
                    rospy.sleep(2)
                    self.image_subscriber = rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.on_image_receive)

            # Clear ahead, just drive
            else:
                self.velocity_command_publisher.publish(velocity_command)

                # Look for a car to read license plates
                self.do_license_plate_work(LEFT)

        # Now we're on the inside
        else:
            # If we haven't seen the truck yet, wait for it
            if not self.truck_known:
                # Stop and wait
                velocity_command.linear.x = 0
                velocity_command.angular.z = 0
                self.velocity_command_publisher.publish(velocity_command)

                # Look for the truck
                if self.find_truck():
                    self.truck_known = True
                    self.start_turning_time = rospy.get_time()

                # cv2.imshow("", self.cv_raw)
                # cv2.waitKey()

            # Now we have seen the truck, so it's time to go
            else:
                # If we haven't finished the second half of the turn, finish it now
                if not self.completed_turn:
                    velocity_command.angular.z = TURNING_OVERRIDE

                    # Check if the turn is now over
                    if rospy.get_time() - self.start_turning_time > SECOND_HALF_TURNING_TIME:
                        self.completed_turn = True

                # Publish the velocity
                self.velocity_command_publisher.publish(velocity_command)

                # Look for a car to read license plates
                self.do_license_plate_work(RIGHT)

        # If done the course, stop timer then stop robot.  self.finished is updated by found_parked_car
        if self.finished:
            self.activate_timer(STOP)
            velocity_command.linear.x = 0
            velocity_command.angular.z = 0
            self.velocity_command_publisher.publish(velocity_command)
            self.timer_stopped = True

    def find_truck(self):
        """
        Check the designated vision square for the truck
        :return: True if the truck is there, False otherwise
        """
        vision_square_top = self.cv_raw[TRUCK_MONITOR_ZONE_TOP_Y[0]:TRUCK_MONITOR_ZONE_TOP_Y[1],
                            TRUCK_MONITOR_ZONE_TOP_X[0]:TRUCK_MONITOR_ZONE_TOP_X[1]]

        mask_top = cv2.inRange(vision_square_top, TRUCK_COLOUR_LOWER_BOUND_MIDDLE_AREA,
                               TRUCK_COLOUR_UPPER_BOUND_MIDDLE_AREA)

        vision_square_bottom = self.cv_raw[TRUCK_MONITOR_ZONE_BOTTOM_Y[0]:TRUCK_MONITOR_ZONE_BOTTOM_Y[1],
                               TRUCK_MONITOR_ZONE_BOTTOM_X[0]:TRUCK_MONITOR_ZONE_BOTTOM_X[1]]

        mask_bottom = cv2.inRange(vision_square_bottom, TRUCK_COLOUR_LOWER_BOUND_MIDDLE_AREA,
                                  TRUCK_COLOUR_UPPER_BOUND_MIDDLE_AREA)

        vision_square_corner = self.cv_raw[TRUCK_MONITOR_ZONE_CORNER_Y[0]:TRUCK_MONITOR_ZONE_CORNER_Y[1],
                               TRUCK_MONITOR_ZONE_CORNER_X[0]:TRUCK_MONITOR_ZONE_CORNER_X[1]]

        mask_corner = cv2.inRange(vision_square_corner, TRUCK_COLOUR_LOWER_BOUND_CORNER,
                                  TRUCK_COLOUR_UPPER_BOUND_CORNER)
        # if (np.average(mask_top) > TRUCK_THRESHOLD_TOP or np.average(mask_corner) > TRUCK_THRESHOLD_CORNER) and np.average(
        #     mask_bottom) < TRUCK_THRESHOLD_BOTTOM:
        #     print("top" + str(np.average(mask_top)))
        #     print("bottom" + str(np.average(mask_bottom)))
        #     print("corner" + str(np.average(mask_corner)))
        #     print("BOTTOM_THRESH" + str(TRUCK_THRESHOLD_BOTTOM))
        #     print("BOOL" + str(np.average(
        #     mask_bottom) < TRUCK_THRESHOLD_BOTTOM))

        # print(np.average(mask_corner))
        return (np.average(mask_top) > TRUCK_THRESHOLD_TOP or np.average(
            mask_corner) > TRUCK_THRESHOLD_CORNER) and np.average(
            mask_bottom) < TRUCK_THRESHOLD_BOTTOM

        # if np.average(mask_top) > TRUCK_THRESHOLD_1:
        #     print(np.average(mask_bottom))
        # #print(np.average(mask_top))

        # pts = np.array([[TRUCK_MONITOR_ZONE_2_X[0], TRUCK_MONITOR_ZONE_2_Y[0]],
        #                 [TRUCK_MONITOR_ZONE_2_X[0], TRUCK_MONITOR_ZONE_2_Y[1]],
        #                 [TRUCK_MONITOR_ZONE_2_X[1], TRUCK_MONITOR_ZONE_2_Y[1]],
        #                 [TRUCK_MONITOR_ZONE_2_X[1], TRUCK_MONITOR_ZONE_2_Y[0]]])
        #
        # pts = pts.reshape((-1, 1, 2))
        #
        # cv2.imshow("", cv2.polylines(self.cv_raw, [pts], True, (255, 0, 0), 3))
        # cv2.waitKey(1)

    def turn_available(self):
        """
        Check the designated vision (ahead and to the left) section to see if there is road there
        :return: True if there's road, False otherwise
        """
        vision_square = self.cv_raw[TURNING_MONITOR_ZONE_Y[0]:TURNING_MONITOR_ZONE_Y[1],
                        TURNING_MONITOR_ZONE_X[0]:TURNING_MONITOR_ZONE_X[1]]
        mask = cv2.inRange(vision_square, ROAD_COLOUR_LOWER_BOUND, ROAD_COLOUR_UPPER_BOUND)

        return np.average(mask) > TURNING_THRESHOLD

        # print(np.average(mask))
        #
        # pts = np.array([[TURNING_MONITOR_ZONE_X[0], TURNING_MONITOR_ZONE_Y[0]],
        #                 [TURNING_MONITOR_ZONE_X[0], TURNING_MONITOR_ZONE_Y[1]],
        #                 [TURNING_MONITOR_ZONE_X[1], TURNING_MONITOR_ZONE_Y[1]],
        #                 [TURNING_MONITOR_ZONE_X[1], TURNING_MONITOR_ZONE_Y[0]]])
        #
        # pts = pts.reshape((-1, 1, 2))
        #
        # cv2.imshow("", cv2.polylines(self.cv_raw, [pts], True, (255, 0, 0), 1))
        # cv2.imshow("mask", mask)
        # cv2.waitKey(1)

    def safe_to_drive_through_crosswalk(self):
        """
        Decide whether we can drive ahead based on if we're at a crosswalk and if there's a pedestrian in the way
        :return: Whether it is safe to drive, and whether we just stopped or not
        """
        # If we're already stopped, check for the pedestrian
        if not self.pedestrian_aside:
            self.pedestrian_aside = self.pedestrian_clear()
            # we've already been stopped, so return False for the 'justStopped' value
            return self.pedestrian_aside, False

        # Otherwise we are moving, so look for a crosswalk
        elif self.at_crosswalk():
            # If we are at a crosswalk, and the pedestrian is in the way, it is not safe.  Otherwise, just drive through
            self.pedestrian_aside = self.pedestrian_clear()
            # we would have just stopped in this case, so return true for the 'justStopped'
            return self.pedestrian_aside, True

        # We are not stopped and there's no crosswalk - just drive.  The justStopped parameter doesn't matter
        else:
            return True, False

    def at_crosswalk(self):
        """
        Determine whether we're at a crosswalk
        :return: True if we are, false otherwise
        """
        # get the time since the last crosswalk was detected
        time_change = rospy.get_time() - self.last_crosswalk_time

        # if it hasn't been long enough, don't even look - more efficient and avoids detecting the second
        # red line on the same crosswalk
        if time_change < MIN_TIME_BETWEEN_CROSSWALKS:
            return False

        # look in the designated zone for the red line
        monitor_zone = self.cv_raw[CROSSWALK_MONITOR_ZONE_Y[0]:CROSSWALK_MONITOR_ZONE_Y[1],
                       CROSSWALK_MONITOR_ZONE_X[0]:CROSSWALK_MONITOR_ZONE_X[1]]

        hsv = cv2.cvtColor(monitor_zone, cv2.COLOR_RGB2HSV)

        mask = cv2.inRange(hsv, CROSSWALK_COLOUR_LOWER_BOUND, CROSSWALK_COLOUR_UPPER_BOUND)

        # if we see the red line, we're at a crosswalk.  Update parameters accordingly
        if np.average(mask) > CROSSWALK_THRESHOLD:
            self.last_crosswalk_time = rospy.get_time()
            self.crosswalk_count = self.crosswalk_count + 1
            return True

        # We're not at the crosswalk
        else:
            return False

    def pedestrian_clear(self):
        """
        Check in the designated vision zone for the pedestrian
        :return: True if the pedestrian is in the vision zone, false otherwise
        """
        danger_zone = self.cv_raw[PEDESTRIAN_MONITOR_ZONE_Y[0]:PEDESTRIAN_MONITOR_ZONE_Y[1],
                      PEDESTRIAN_MONITOR_ZONE_X[0]:PEDESTRIAN_MONITOR_ZONE_X[1]]

        mask = cv2.inRange(danger_zone, PEDESTRIAN_COLOUR_LOWER_BOUND, PEDESTRIAN_COLOUR_UPPER_BOUND)

        for i in range(mask.shape[1]):
            # if any column is higher than the threshold, then return False since the pedestrian is not clear
            if np.average(mask[:, i]) > PEDESTRIAN_THRESHOLD:
                return False

        # The pedestrian is not in the vision section, so it is clear and return True
        return True

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

    def activate_timer(self, start):
        """
        Start or stop the timer
        :param start: True to start, False to stop
        :return: None
        """
        # Publish location 0 and our info to start
        if start:
            message = String()
            message.data = TEAM_ID + "," + TEAM_PW + "," + str(0) + "," + "AAAA"
            self.license_plate_publisher.publish(message)

        # Publish location -1 and our info to stop
        else:
            message = String()
            message.data = TEAM_ID + "," + TEAM_PW + "," + str(-1) + "," + "ZZZZ"
            self.license_plate_publisher.publish(message)

    def publish_plate(self, location, plate_number):
        """
        Publish the given plate number and location to the topic
        :param location: The parking location number
        :param plate_number: The plate number (String)
        :return: None
        """
        message = String()
        # Populate the message with the location and number provided
        message.data = TEAM_ID + "," + TEAM_PW + "," + str(location) + "," + plate_number

        self.license_plate_publisher.publish(message)

    def do_license_plate_work(self, side):
        """
            Main Control for the license plate detection process
            Looks for parked car.  If parked car found, takes license plate, gets the value, and publishes it
        """
        # Check for a parked car
        found_car, location = self.found_parked_car(side)

        # If we found it, do the work
        if found_car:
            self.last_car_time = rospy.get_time()
            license_plate = self.find_license_plate(side)

            predicted_license_plate_number = self.License_Plate_Reader.license_read(license_plate)
            true_license_plate_number = self.true_plates[location - 1]

            if not predicted_license_plate_number == true_license_plate_number:
                #self.save_plate(license_plate, str(true_license_plate_number) + "-" + str(predicted_license_plate_number),ERROR_PLATE_FILE_PATH, 1)
                self.save_plate(license_plate, str(true_license_plate_number) + "-ER",DATA_PLATE_FILE_PATH, 5)

            elif self.on_inside:
                self.save_plate(license_plate, str(true_license_plate_number) + "-I", DATA_PLATE_FILE_PATH, 3)

            else:
                self.save_plate(license_plate, str(true_license_plate_number), DATA_PLATE_FILE_PATH, 1)
            # if SAVE_DATA_PLATES:
            #     self.save_plate(license_plate, str(true_license_plate_number), DATA_PLATE_FILE_PATH, 1)

            self.publish_plate(location, str(predicted_license_plate_number))

            # cv2.imshow(license_plate_number, license_plate)
            # cv2.waitKey(1)

    def find_license_plate(self, side):
        """
        Find the actual license plate in the current frame
        :return: the license plate in a cv2 image
        """
        left, right, top = self.find_edge_of_label(side)

        if side == LEFT:
            license_plate = self.cv_raw[LICENSE_PLATE_CAR_VISION_LEFT_Y[0] + top:LICENSE_PLATE_CAR_VISION_LEFT_Y[
                                                                                     0] + top + LICENSE_PLATE_HEIGHT,
                            LICENSE_PLATE_CAR_VISION_LEFT_X[0] + left: LICENSE_PLATE_CAR_VISION_LEFT_X[0] + right]
        else:
            license_plate = self.cv_raw[
                            LICENSE_PLATE_CAR_VISION_RIGHT_Y[0] + top:LICENSE_PLATE_CAR_VISION_RIGHT_Y[0] + top +
                                                                      LICENSE_PLATE_HEIGHT,
                            LICENSE_PLATE_CAR_VISION_RIGHT_X[0] + left: LICENSE_PLATE_CAR_VISION_RIGHT_X[0] + right]

        return license_plate

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

    @staticmethod
    def save_plate(image, name, path, count):
        """
        Save the license plate to file
        :param image: the license_plate to save
        :param name: the name of the plate file
        """
        for i in range(count):
            cv2.imwrite(path + name + "-" + str(i) + ".png", image)

    # CANNOT BE USED IN COMPETITION
    @staticmethod
    def init_plate_value():
        """
        Get the real values of the license plates from the csv file
        :return:
        """
        LICENSE_PLATE_FILE = '/home/fizzer/ros_ws/src/Competition_Package/enph353/enph353_gazebo/scripts/plates.csv'
        with open(LICENSE_PLATE_FILE, "r") as plate_file:
            platereader = csv.reader(plate_file)
            true_plates = []
            i = 0
            for row in platereader:
                true_plates.append(row[0])
        return true_plates

    def find_edge_of_label(self, side):
        """
        Helper function to find the license plate
        :return: the left, right and top of the license plate
        """

        if side == LEFT:
            vision_section = self.cv_raw[LICENSE_PLATE_CAR_VISION_LEFT_Y[0]:LICENSE_PLATE_CAR_VISION_LEFT_Y[1],
                             LICENSE_PLATE_CAR_VISION_LEFT_X[0]:LICENSE_PLATE_CAR_VISION_LEFT_X[1]]
        else:
            vision_section = self.cv_raw[LICENSE_PLATE_CAR_VISION_RIGHT_Y[0]:LICENSE_PLATE_CAR_VISION_RIGHT_Y[1],
                             LICENSE_PLATE_CAR_VISION_RIGHT_X[0]:LICENSE_PLATE_CAR_VISION_RIGHT_X[1]]

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
        for j in range(width):
            if column_average[j] > LICENSE_PLATE_THRESHOLD_X:
                left = j
                set_left = True
                break

        right = width - 1
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
        for j in range(height):
            if row_average[j] < LICENSE_PLATE_THRESHOLD_Y:
                top = j
                break

        return left, right, top

    def found_parked_car(self, side):
        """
        Check for a parked car in the designated vision section
        :return: If a parked car is present, and the location id
        """

        if rospy.get_time() - self.last_car_time < MIN_TIME_BETWEEN_CARS:
            return False, -1




        if side == LEFT:
            vision_section = self.cv_raw[PARKED_CAR_VISION_LEFT_Y[0]:PARKED_CAR_VISION_LEFT_Y[1],
                             PARKED_CAR_VISION_LEFT_X[0]:PARKED_CAR_VISION_LEFT_X[1]]

        else:
            vision_section = self.cv_raw[PARKED_CAR_VISION_RIGHT_Y[0]:PARKED_CAR_VISION_RIGHT_Y[1],
                             PARKED_CAR_VISION_RIGHT_X[0]:PARKED_CAR_VISION_RIGHT_X[1]]

        hsv = cv2.cvtColor(vision_section, cv2.COLOR_RGB2HSV)

        car_mask_1 = cv2.inRange(hsv, PARKED_CAR_COLOUR_LOWER_BOUND_1, PARKED_CAR_COLOUR_UPPER_BOUND_1)
        car_mask_2 = cv2.inRange(hsv, PARKED_CAR_COLOUR_LOWER_BOUND_2, PARKED_CAR_COLOUR_UPPER_BOUND_2)
        car_mask_3 = cv2.inRange(hsv, PARKED_CAR_COLOUR_LOWER_BOUND_3, PARKED_CAR_COLOUR_UPPER_BOUND_3)

        car_mask_total = car_mask_1 + car_mask_2 + car_mask_3

        vision_section_avg = np.average(car_mask_total)

        # pts = np.array([[PARKED_CAR_VISION_RIGHT_X[0], PARKED_CAR_VISION_RIGHT_Y[0]],
        #                 [PARKED_CAR_VISION_RIGHT_X[0], PARKED_CAR_VISION_RIGHT_Y[1]],
        #                 [PARKED_CAR_VISION_RIGHT_X[1], PARKED_CAR_VISION_RIGHT_Y[1]],
        #                 [PARKED_CAR_VISION_RIGHT_X[1], PARKED_CAR_VISION_RIGHT_Y[0]]])
        #
        # pts = pts.reshape((-1, 1, 2))

        # if side == RIGHT:
        # cv2.imshow("", cv2.polylines(self.cv_raw, [pts], True, (255, 0, 0), 3))

        # cv2.imshow("mask", car_mask_total)
        # cv2.waitKey(1)

        # if vision_section_avg > 9:
        # cv2.waitKey()

        if vision_section_avg > PARKED_CAR_AVG_THRESHOLD_SINGLE:
            location_id = PARKED_CAR_ORDER[self.parked_car_counter]
            self.parked_car_counter = 1 + self.parked_car_counter
            if self.parked_car_counter >= len(PARKED_CAR_ORDER):
                self.finished = True

            self.parked_car_interval_counter = 0
            self.last_car_time = rospy.get_time()
            return True, location_id

        else:
            return False, -1

    def initial_turn_sequence(self):
        """
        The hardcode turn just to get the robot going a little left
        """
        velocity_command = Twist()

        velocity_command.angular.z = 0.5
        velocity_command.linear.x = FULL_SPEED_LINEAR

        self.velocity_command_publisher.publish(velocity_command)

    def standard_drive_velocity(self, centre_setpoint):
        """
        Do the road following
        :param centre_setpoint: the desired centre position of the road
        :return:
        """
        centre_x, center_found = self.find_road_center()

        velocity_command = Twist()

        # If no road at all, use the no road values
        if not center_found:
            if self.last_side_left:
                velocity_command.angular.z = NO_READING_ANGULAR_VELOCITY
            else:
                velocity_command.angular.z = -NO_READING_ANGULAR_VELOCITY

            velocity_command.linear.x = FULL_SPEED_LINEAR * NO_READING_LINEAR_DECREASE

        # Otherwise use proportional adjustment
        else:
            velocity_command.linear.x = FULL_SPEED_LINEAR
            velocity_command.angular.z = FULL_SPEED_LINEAR * ANGULAR_PROPORTIONAL * (centre_setpoint - centre_x)
            if centre_x > centre_setpoint:
                self.last_side_left = False
            else:
                self.last_side_left = True

        if SHOW_ROAD_VISION:
            self.show_road_vision(centre_x, centre_setpoint)

        return velocity_command

    def show_road_vision(self, centre_x, centre_setpoint):
        """
        Show the road vision for debugging
        """
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

    def find_road_center(self):
        """
        Find the road center using classical vision
        :return: the centre of the road, and whether or not the road was found
        """
        strip = self.cv_raw[DRIVE_VISION_STRIP[0]:DRIVE_VISION_STRIP[1], STRIP_CROPPING_X:-STRIP_CROPPING_X]
        cropped_strip = cv2.inRange(strip, ROAD_COLOUR_LOWER_BOUND, ROAD_COLOUR_UPPER_BOUND)

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
