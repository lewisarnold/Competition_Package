#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import homography_analyzer


class Listener():
    def run(self):
        self.homography_analyzer = homography_analyzer.Homography_Analyzer(
            "//pedestrian_detector/nodes/good_images/cropped_pedestrian_back.jpg")

        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber('R1/pi_camera/image_raw', Image, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def callback(self, image_msg):
        bridge = CvBridge()
        train_frame = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        #train_frame = cv2.cvtColor(train_frame, cv2.COLOR_BGR2GRAY)

        train_frame = train_frame[350:730, 400:1000, :]

        result = self.homography_analyzer.check_train_image(train_frame)

        print(result)

if __name__ == '__main__':
    listener = Listener()
    listener.run()
