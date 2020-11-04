#!/usr/bin/env python

import cv2

LEFT = 620
RIGHT = 709
TOP = 326
BOTTOM = 616


def load_and_crop_image(filename):
    raw_image = cv2.imread(filename)

    cv2.imshow("", raw_image)
    cv2.waitKey()

    cropped_image = raw_image[TOP:BOTTOM, LEFT:RIGHT, :]

    cv2.imshow("", cropped_image)
    cv2.waitKey()

    cv2.imwrite("/home/fizzer/ros_ws/src/2020T1_competition/image_saver/nodes/good_images/cropped_pedestrian_front.jpg", cropped_image)




if __name__ == "__main__":
    load_and_crop_image("/home/fizzer/ros_ws/src/2020T1_competition/image_saver/nodes/good_images/Pedestrianfront.jpg")
