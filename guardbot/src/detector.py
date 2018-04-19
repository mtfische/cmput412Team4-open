#!/usr/bin/env python

# Modified from https://www.pyimagesearch.com/2015/11/09/pedestrian-detection-opencv/
from __future__ import print_function
from imutils.object_detection import non_max_suppression
from imutils import paths
import argparse
import imutils
import rospy, cv2, cv_bridge, numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from copy import deepcopy
from std_msgs.msg import String

class Detector:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()

                self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                        Image, self.image_callback, queue_size=1)
                self.det_pub = rospy.Publisher('person_detect', String, queue_size=1)

                self.twist = Twist()

        def image_callback(self, msg):
            #print("callback")

            # Open Bridge
            image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
	    image = imutils.resize(image, width=min(400, image.shape[1]))

            hog = cv2.HOGDescriptor()
            hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

	    # detect people in the image
	    (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),
	    padding=(8, 8), scale=1.3, finalThreshold=3 )


            # apply non-maxima suppression to the bounding boxes using a
            # fairly large overlap threshold to try to maintain overlapping
            # boxes that are still people
            rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
            pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)

            #print(pick)
            # draw the final bounding boxes
            for (xA, yA, xB, yB) in pick:
                cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)

		# show the output images
	    cv2.imshow("After NMS", image)
            cv2.waitKey(1)
            if(len(pick) > 0):
                self.det_pub.publish("Found")

rospy.init_node('detector')
detector = Detector()
rospy.spin()
