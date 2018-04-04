#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from ar_track_alvar_msgs.msg import AlvarMarkers
import math

class ShowAR:

    def __init__(self):
        self.ar_sub = rospy.Subscriber('ar_pose_marker',AlvarMarkers, self.ar_callback)
        self.image_sub = rospy.Subscriber('camera/rgb/image_rect_color',Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber('camera/rgb/camera_info', CameraInfo, self.camera_info_callback)
        self.bridge = cv_bridge.CvBridge()
        self.camera_matrix = None
        self.camera_dist = None
        self.img = None

    def camera_info_callback(self, msg):
        self.camera_matrix = np.asarray(msg.K).reshape(3,3)
        self.camera_dist = np.asarray(msg.D)

    def ar_callback(self, msg):
        axis = np.float32([[0.1,0,0], [0,-0.1,0], [0,0,0.1],[0,0,0]]).reshape(-1,3)

        rvecs = np.array([0,0,0])
        tvecs = np.array([0,0,0])

        for i in range(len(msg.markers)):
            px = msg.markers[i].pose.pose.position.x
            py = msg.markers[i].pose.pose.position.y
            pz = msg.markers[i].pose.pose.position.z

            tvecs = np.array([px,py,pz])

            ox = msg.markers[i].pose.pose.orientation.x
            oy = msg.markers[i].pose.pose.orientation.y
            oz = msg.markers[i].pose.pose.orientation.z
            ow = msg.markers[i].pose.pose.orientation.w

            angle = 2 * math.acos(ow)
            x = ox / math.sqrt(1 - ow*ow)
            y = oy / math.sqrt(1 - ow*ow)
            z = oz / math.sqrt(1 - ow*ow)
            ratio = math.sqrt(x*x + y*y + z*z)

            #normalize them
            x = x / ratio*angle
            y = y / ratio*angle
            z = z / ratio*angle
            rvecs = np.array([x, y, z])

            print(tvecs)
            print(rvecs)

            imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, self.camera_matrix, self.camera_dist)
            print(imgpts)
            image = cv2.line(self.img, tuple(imgpts[3].ravel()), tuple(imgpts[2].ravel()), (255,0,0), 5)
            image = cv2.line(self.img, tuple(imgpts[3].ravel()), tuple(imgpts[0].ravel()), (0,255,0), 5)
            image = cv2.line(self.img, tuple(imgpts[3].ravel()), tuple(imgpts[1].ravel()), (0,0,255), 5)

        cv2.imshow('img',self.img)
        cv2.waitKey(1)

    def image_callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

    def camera_info_callback(self, msg):
        self.camera_matrix = np.asarray(msg.K).reshape(3,3)
        self.camera_dist = np.asarray(msg.D)

class ShowImage:

    def __init__(self):
        self.image_sub = rospy.Subscriber('camera/rgb/image_rect_color',Image, self.image_callback)
        self.bridge = cv_bridge.CvBridge()

    def draw(self, img, imgpts):
        cv2.line(img, tuple(imgpts[0][0]), tuple(imgpts[0][0]), (255,0,0), 5)
        #cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
        #cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
        #cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        global imgpts,corner
        self.draw(image, imgpts)

        cv2.imshow('img',image)
        cv2.waitKey(5)

rospy.init_node('part1')
showAR = ShowAR()
#showImage = ShowImage()
rospy.spin()
