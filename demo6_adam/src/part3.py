#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import cv_bridge
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

# https://stackoverflow.com/questions/20259025/module-object-has-no-attribute-drawmatches-opencv-python
def drawMatches(img1, kp1, img2, kp2, matches):
    """
    My own implementation of cv2.drawMatches as OpenCV 2.4.9
    does not have this function available but it's supported in
    OpenCV 3.0.0

    This function takes in two images with their associated
    keypoints, as well as a list of DMatch data structure (matches)
    that contains which keypoints matched in which images.

    An image will be produced where a montage is shown with
    the first image followed by the second image beside it.

    Keypoints are delineated with circles, while lines are connected
    between matching keypoints.

    img1,img2 - Grayscale images
    kp1,kp2 - Detected list of keypoints through any of the OpenCV keypoint
              detection algorithms
    matches - A list of matches of corresponding keypoints through any
              OpenCV keypoint matching algorithm
    """

    # Create a new output image that concatenates the two images together
    # (a.k.a) a montage
    rows1 = img1.shape[0]
    cols1 = img1.shape[1]
    rows2 = img2.shape[0]
    cols2 = img2.shape[1]

    # Create the output image
    # The rows of the output are the largest between the two images
    # and the columns are simply the sum of the two together
    # The intent is to make this a colour image, so make this 3 channels
    out = np.zeros((max([rows1,rows2]),cols1+cols2,3), dtype='uint8')

    # Place the first image to the left
    out[:rows1,:cols1] = np.dstack([img1, img1, img1])

    # Place the next image to the right of it
    out[:rows2,cols1:] = np.dstack([img2, img2, img2])

    # For each pair of points we have between both images
    # draw circles, then connect a line between them
    for mat in matches:
        # Get the matching keypoints for each of the images
        img1_idx = mat.queryIdx
        img2_idx = mat.trainIdx

        # x - columns
        # y - rows
        (x1,y1) = kp1[img1_idx].pt
        (x2,y2) = kp2[img2_idx].pt

        # Draw a small circle at both co-ordinates
        # radius 4
        # colour blue
        # thickness = 1
        cv2.circle(out, (int(x1),int(y1)), 4, (255, 0, 0), 1)
        cv2.circle(out, (int(x2)+cols1,int(y2)), 4, (255, 0, 0), 1)

        # Draw a line in between the two points
        # thickness = 1
        # colour blue
        cv2.line(out, (int(x1),int(y1)), (int(x2)+cols1,int(y2)), (255,0,0), 1)

    # Show the image
    cv2.imshow('Matched Features', out)
    cv2.waitKey(3)
    # Also return the image if you'd like a copy

class PoseEstimator:

    def __init__(self):
        self.MIN_MATCH_COUNT = 10
        self.img1 = cv2.imread('ualberta_logo.png',0) # queryImage
        self.img1 = cv2.resize(self.img1, (0,0), fx = 0.25, fy = 0.25)
        self.orb = cv2.ORB()
        self.kp1 = self.orb.detect(self.img1,None)
        self.kp1, self.des1 = self.orb.compute(self.img1, self.kp1)
        self.des1 = np.float32(self.des1)

        self.axis = np.float32([[30,0,0], [0,30,0], [0,0,-30], [0,0,0,]]).reshape(-1,3)

        self.image_sub = rospy.Subscriber('camera/rgb/image_rect_color',Image, self.image_callback)
        self.image_info_sub = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.camera_info_callback)
        #self.image_sub = rospy.Subscriber('/usb_cam/image_raw',Image, self.image_callback)
        #self.image_info_sub = rospy.Subscriber('/usb_cam/camera_info', CameraInfo, self.camera_info_callback)

        self.camera_matrix = np.ones(9).reshape(3,3)
        self.camera_dist = np.ones(3)

        self.bridge = cv_bridge.CvBridge()
        pass

    def camera_info_callback(self, msg):
        self.camera_matrix = np.asarray(msg.K).reshape(3,3)
        self.camera_dist = np.asarray(msg.D)

    def image_callback(self, msg):
        img2 = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        gray =  cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)

        cv2.imshow("window1", img2)
        cv2.waitKey(3)

        orb = cv2.ORB()
        kp2 = orb.detect(gray,None)
        kp2, des2 = self.orb.compute(gray, kp2)
        des2 = np.float32(des2)

        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)

        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(self.des1, des2)

        # store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
        	if m.distance < 0.75*n.distance:
        		good.append(m)

        if len(good) > self.MIN_MATCH_COUNT:
            src_pts = np.float32([ self.kp[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()

            h,w = self.target_image.shape

            rect = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            rect3d = np.float32([ [0,0,0],[0,h-1,0],[w-1,h-1,0],[w-1,0,0] ]).reshape(-1,1,3)
            rect = cv2.perspectiveTransform(rect,M)

            # Center of rectangle = ( (x1 + x2) / 2 , (y1 + y2) / 2)
            # where the two points are diagonals of each other
            center_x = int((rect[0][0][0] + rect[2][0][0]) / 2)
            center_y = int((rect[0][0][1] + rect[2][0][1]) / 2)

            offset = np.array([0,0])

            self.center = (center_x, -center_y)

            img2 = cv2.polylines(gray,[np.int32(rect)],True,255,3, cv2.CV_AA)

            dst2 = dst_pts[matchesMask].reshape(dst_pts.shape[0], 2)
            src2 = src_pts[matchesMask].reshape(dst_pts.shape[0], 2)

            rvecs, tvecs, inliers = cv2.solvePnPRansac(rect3d, rect, self.camera_info, self.camera_dist)

            imgpts, jac = cv2.projectPoints(self.axis, rvecs, tvecs, self.camera_info, self.camera_dist)

            # Note: imgpts[3] contains the x and y coordinates of the top-left corner of the rectangle
            #		which surrounds the found object.
            image = cv2.line(img, tuple(imgpts[3].ravel()), tuple(imgpts[0].ravel()), (255,0,0), 5)
            image = cv2.line(img, tuple(imgpts[3].ravel()), tuple(imgpts[1].ravel()), (0,255,0), 5)
            image = cv2.line(img, tuple(imgpts[3].ravel()), tuple(imgpts[2].ravel()), (0,0,255), 5)

            cv2.imshow('img', img)
            k = cv2.waitKey(1) & 0xff

        else:
            print ("Not enough matches are found - %d/%d" % (len(good),self.MIN_MATCH_COUNT))
            matchesMask = None

rospy.init_node('pose_estimator')
showImage = PoseEstimator()
rospy.spin()
