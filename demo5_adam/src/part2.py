#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist
import math

class ShowImage:

    def __init__(self):

        self.DrawImage = False
        self.Drive = True

        self.FrameSkipRate = 4
        self.frame = 0

        self.twist = Twist()

        # Speed Vars
        self.cruiseSpeed = 0.1
        self.ramp_rate = 0.1
        # Ramping
        self.last_send_time = rospy.Time.now()
        self.last_x_vel = 0

        # PD controller vars.
        self.err = 0
        self.lastErr = 0
        self.Kp = 0.03
        self.Kd = -0.01

        self.image_sub = rospy.Subscriber('camera/rgb/image_rect_color',Image, self.image_callback)
        self.image_info_sub = rospy.Subscriber('/camera/rgb/image_proc_resize/camera_info', CameraInfo, self.camera_info_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.camera_matrix = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_dist = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.bridge = cv_bridge.CvBridge()

    # Modified from Textbook
    def ramped_vel ( self, v_prev , v_target , t_prev , t_now):
        # compute maximum velocity step
        step = self.ramp_rate * ( t_now - t_prev ).to_sec()
        sign = 1.0 if ( v_target > v_prev ) else -1.0
        error = math.fabs( v_target - v_prev )
        if error < step: # we can get there within this timestep-we're done.
            return v_target
        else:
            return v_prev + sign * step # take a step toward the target


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
        #print(((corners[0])[0])[0])

        # If found, add object points, image points (after refining them)
        if ret == True:
            #corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

            # Find the rotation and translation vectors.
            #rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, self.camera_matrix, self.camera_dist)
            rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners, self.camera_matrix, self.camera_dist)

            # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, self.camera_matrix, self.camera_dist)

            if self.DrawImage:
                self.draw(image,corners,imgpts)

            #print(rvecs)

            #Modified from https://github.com/n17r4m/kobuki/blob/master/demo5/src/docker.py
            horiz = 0
            if (rvecs[2])[0] > 0:
                horiz = (tvecs[0])[0] - 6
            elif (rvecs[2])[0] < 0:
                horiz = (tvecs[0])[0] + 6

            h, w, d = image.shape
            print("((imgpts[0])[0])[0]: " + str(((imgpts[0])[0])[0]) + " w/2: " + str(w/2))
            print("T: " + str(tvecs.reshape(1,3)))
            print("R: " + str(rvecs.reshape(1,3)))

            offset = 0
            if (rvecs[2])[0] < 0:
                offset = 20
            elif (rvecs[2])[0] > 0:
                offset = -20

            z = 0
            if ((imgpts[0])[0])[0] < (w/2 + offset):
                z = 0.2
            elif ((imgpts[0])[0])[0] > (w/2 + offset):
                z = -0.2

            x = 0
            if (tvecs[2])[0] > 14:
                x = 0.2

            self.twist.angular.z = (3*self.twist.angular.z + z) / 4
            self.twist.linear.x = (3*self.twist.linear.x + x) / 4

            print(self.twist)

            if self.Drive:
                self.cmd_vel_pub.publish(self.twist)
                self.last_send_time = rospy.Time.now()

        # Show Debug information in window
        if self.DrawImage and self.FrameSkipRate == self.frame:
            cv2.imshow("image", image)
            cv2.waitKey(3)
            self.frame = 0
        else:
            self.frame = self.frame + 1

rospy.init_node('part1')
showImage = ShowImage()
rospy.spin()
