#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

class ShowImage:

    def __init__(self):
        self.image_sub = rospy.Subscriber('camera/rgb/image_rect_color',Image, self.image_callback)
        self.image_info_sub = rospy.Subscriber('/camera/rgb/image_proc_resize/camera_info', CameraInfo, self.camera_info_callback)

        self.camera_matrix = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_dist = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)


    def draw(self, img, corners, imgpts):
        corner = tuple(corners[0].ravel())
        cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
        cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
        cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)

    def camera_info_callback(self, msg):
        self.camera_matrix = np.asarray(msg.K).reshape(3,3)
        self.camera_dist = np.asarray(msg.D)

    def image_callback(self, msg):
        # Open Bridge and get greyscale image.
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

        # Set terminatiob criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        objp = np.zeros((6*8,3), np.float32)
        objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

        axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (8,6),None)
        print("cb")
        # If found, add object points, image points (after refining them)
        if ret == True:
            #corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

            # Find the rotation and translation vectors.
            #rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, self.camera_matrix, self.camera_dist)
            rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners, self.camera_matrix, self.camera_dist)
	    
            print(tvecs.reshape(1,3))
            # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, self.camera_matrix, self.camera_dist)

            #print(image)
            self.draw(image,corners,imgpts)
            #print(image)
            cv2.imshow('img',image)
            cv2.waitKey(3)

rospy.init_node('part1')
showImage = ShowImage()
rospy.spin()
