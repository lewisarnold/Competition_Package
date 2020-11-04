#!/usr/bin/env python

import numpy as np

import cv2
import sys

DISTANCE_PARAMETER = 0.9
HOMOGRAPHY_THRESHOLD = 4


class Homography_Analyzer():
    def __init__(self, template_file_path):
        self._load_template(template_file_path)

        self.sift = None
        self.template_key_points = None
        self.template_descriptors = None
        self._make_sift()

        self.flann = None
        self._make_flann()

    def _load_template(self, template_path):
        # Load the template image
        try:
            template_image = cv2.imread(template_path)
            self.template = template_image
            #self.template = cv2.cvtColor(template_image, cv2.COLOR_BGR2GRAY)
        except:
            print("Could not load template image")

    def _make_sift(self):
        # Make a SIFT object
        self.sift = cv2.xfeatures2d.SIFT_create()
        self.template_key_points, self.template_descriptors = self.sift.detectAndCompute(self.template,
                                                                                         None)

    def _make_flann(self):
        # Make a flann object
        index_params = dict(algorithm=0, trees=5)
        search_params = dict()
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

    def check_train_image(self, train_frame):
        # Analyze train image with SIFT
        frame_key_points, frame_descriptors = self.sift.detectAndCompute(train_frame, None)
        # frame = cv2.drawKeypoints(frame, frame_key_points, frame)

        # Flann gets points that are close (in 128 dimensional space)
        matches = self.flann.knnMatch(self.template_descriptors, frame_descriptors, k=2)

        # We select points which are close to each other in distance
        good_points = []
        for m, n in matches:
            # The coefficient below determines the threshold.  Lower number makes it harder to be matched
            # (more restrictive)
            if m.distance < DISTANCE_PARAMETER * n.distance:
                good_points.append(m)

        image_with_matches = cv2.drawMatches(self.template, self.template_key_points, train_frame, frame_key_points,
                                             good_points, train_frame)

        # If we have more than a certain number of good points, we can make a homography
        train_pts = np.float32([frame_key_points[m.trainIdx].pt for m in good_points]).reshape(-1, 1, 2)
        print(train_pts)

        if len(good_points) > HOMOGRAPHY_THRESHOLD:
            # extract the actual point locations from the list of points
            query_pts = np.float32([self.template_key_points[m.queryIdx].pt for m in good_points]).reshape(-1, 1, 2)
            train_pts = np.float32([frame_key_points[m.trainIdx].pt for m in good_points]).reshape(-1, 1, 2)
            y_tot = 0
            y_count = 0
            x_tot = 0
            x_count = 0

            for y in train_pts[:, 0, 1]:
                y_tot = y_tot + y
                y_count = y_count + 1

            for x in train_pts[:, 0, 0]:
                x_tot = x_tot + x
                x_count = x_count + 1

            marked = cv2.circle(train_frame, (x,y), 20, (255, 0, 0), 2)




            # # Find a consensus on which points map to which using RANSAC
            # # Returns the perspective transform which needs to be applied to get from one image to the other
            # M, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)
            #
            # # Do the perspective transform
            # height = self.template.shape[0]
            # width = self.template.shape[1]
            #
            # # The corners of the template image
            # pts = np.float32([[0, 0], [0, height], [width, height], [width, 0]]).reshape(-1, 1, 2)
            # # Apply the perspective transform to get the corners in the train image
            # print(M)
            #
            # dst = cv2.perspectiveTransform(pts, M)
            #
            #
            #
            # # Draw the frame around the template as it appears in the train image
            # homography = cv2.polylines(train_frame, [np.int32(dst)], True, (255, 0, 0), 3)

            cv2.imshow("", marked)
            cv2.waitKey()
            return True

        else:
            cv2.imshow("", image_with_matches)
            cv2.waitKey()
            return False


#if __name__ == "__main__":

