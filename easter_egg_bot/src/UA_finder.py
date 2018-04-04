#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import cv_bridge
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

class PoseEstimator:

    def __init__(self):
        self.MIN_MATCH_COUNT = 15
        self.img1 = cv2.imread('/home/malcolm/turtlebot_ws/src/easter_egg_bot/ualberta_logo.png',0) # queryImage

	self.img1 = cv2.resize(self.img1, (0,0), fx = 0.5, fy = 0.5)
        self.orb = cv2.ORB()
	cv2.imshow("window1", self.img1)
        cv2.waitKey(3)
        # find the keypoints and descriptors with ORB
        self.kp1, self.des1 = self.orb.detectAndCompute(self.img1,None)

        self.FLANN_INDEX_KDTREE = 0
        self.index_params = dict(algorithm = self.FLANN_INDEX_KDTREE, trees = 5)
        self.search_params = dict(checks = 50)

        self.flann = cv2.FlannBasedMatcher(self.index_params, self.search_params)

        self.image_sub = rospy.Subscriber('camera/rgb/image_rect_color',Image, self.image_callback)
        self.image_info_sub = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.camera_info_callback)

	self.axis = np.float32([[100,0,0], [0,100,0], [0,0,-100], [0,0,0]]).reshape(-1,3)
        self.camera_matrix = np.ones(9).reshape(3,3)
        self.camera_dist = np.ones(3)

        self.bridge = cv_bridge.CvBridge()
        self.ua_pub = rospy.Publisher('side_camera_ua', String, queue_size=1)
        pass

    def camera_info_callback(self, msg):
        self.camera_matrix = np.asarray(msg.K).reshape(3,3)
        self.camera_dist = np.asarray(msg.D)

    def image_callback(self, msg):
	#print("img_cb")
        img2 = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        img2 =  cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
        cv2.imshow("window1", img2)
        cv2.waitKey(3)

        kp2, des2 = self.orb.detectAndCompute(img2,None)

        FLANN_INDEX_LSH = 6
        index_params= dict(algorithm = FLANN_INDEX_LSH,
                   table_number = 12,
                   key_size = 20,
                   multi_probe_level = 2)
        search_params = dict(checks = 100)

        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(self.des1, des2, k=2)

        # store all the good matches as per Lowe's ratio test.
        good = []
	try:
		for m,n in matches:
		    if m.distance < 0.75*n.distance:
		        good.append(m)
	except ValueError:
		pass

	if len(good) > self.MIN_MATCH_COUNT:
            print("UA Logo Found!")
	    self.ua_pub.publish("Found")
	else:
            #print "Not enough matches are found - %d/%d" % (len(good),self.MIN_MATCH_COUNT)
            pass

rospy.init_node('pose_estimator')
showImage = PoseEstimator()
rospy.spin()
