#!/usr/bin/env python

# Modified from http://edu.gaitech.hk/turtlebot/line-follower.html accessed Feb 3rd 2018

#This Program is tested on Gazebo Simulator
#This script uses the cv_bridge package to convert images coming on the topic
#sensor_msgs/Image to OpenCV messages and then convert their colors from RGB to HSV
#then apply a threshold for hues near the color yellow to obtain the binary image
#to be able to see only the yellow line and then follow that line
#It uses an approach called proportional and simply means

import rospy, cv2, cv_bridge, numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from copy import deepcopy

class Follower:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()
                cv2.namedWindow("window", 1)

                self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=1)

                self.twist = Twist()

        def image_callback(self, msg):

                # Open Bridge and get greyscale image.
                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                img2gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

                # Get the threshold for the image to find white line
                ret, threshold = cv2.threshold(img2gray, 250, 255, cv2.THRESH_BINARY)
                #cv2.imshow("window_thresh", threshold)

                # "Open" the image for less error in countouring.
                kernel = np.ones((5,5),np.uint8)
                opening = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, kernel)
                cv2.imshow("window_opening", opening)

                ## Another option is to Blur it for smoothing for less error in countouring.
                #blur = cv2.GaussianBlur(img2gray,(9,9),0)
                #ret, thresh = cv2.threshold(blur, 253, 255, cv2.THRESH_BINARY)
                #cv2.imshow("window_blur", blur)

                # Find Close moment:
                h, w, d = image.shape
                search_top = 3*h/4
                search_bot = 3*h/4 + 20
                bottomThresh = deepcopy(threshold)
                bottomThresh[0:search_top, 0:w] = 0
                Mclose = cv2.moments(bottomThresh)

                # Find Far moment:
                search_top = 2*h/3
                search_bot = 2*h/3 + 20
                topThresh = deepcopy(threshold)
                topThresh[0:search_top, 0:w] = 0
                topThresh[search_bot:, 0:w] = 0
                Mfar = cv2.moments(topThresh)

                # Calculate with far information
                if Mclose['m00'] > 0 and Mfar['m00'] > 0:
                  # Draw Moments:
                  cxc = int(Mclose['m10']/Mclose['m00'])
                  cyc = int(Mclose['m01']/Mclose['m00'])
                  cv2.circle(image, (cxc, cyc), 20, (0,0,255), -1)

                  cxf = int(Mfar['m10']/Mfar['m00'])
                  cyf = int(Mfar['m01']/Mfar['m00'])
                  cv2.circle(image, (cxf, cyf), 20, (0,255,0), -1)

                  # BEGIN CONTROL
                  err = cxc - w/2
                  self.twist.linear.x = 0.2
                  self.twist.angular.z = -float(err) / 100
                  self.cmd_vel_pub.publish(self.twist)

                  # END CONTROL

                  # Show Debug information in window
                  cv2.imshow("window", image)
                  cv2.waitKey(3)

                # Calculate without far information
                elif Mclose['m00'] > 0:
                  cx = int(Mclose['m10']/Mclose['m00'])
                  cy = int(Mclose['m01']/Mclose['m00'])
                  cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

                  # BEGIN CONTROL
                  err = cx - w/2
                  self.twist.linear.x = 0.2
                  self.twist.angular.z = -float(err) / 100
                  self.cmd_vel_pub.publish(self.twist)
                  # END CONTROL

                  # Show Debug information in window
                  cv2.imshow("window", image)
                  cv2.waitKey(3)

"""
                # Contour method (not functional):

                # Find Contours
                (contours, hier) = cv2.findContours(threshold, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                cv2.imshow("window_blur", thresh)

                rows,cols = threshold.shape[:2]
                [vx,vy,x,y] = cv2.fitLine(contours[0], cv2.DIST_L2,0,0.01,0.01)
                lefty = int((-x*vy/vx) + y)
                righty = int(((cols-x)*vy/vx)+y)
                cv2.line(imgage,(cols-1,righty),(0,lefty),(0,255,0),2)
                cv2.drawContours(image, contours, -1, (0,255,0), 3)
                view = cv2.bitwise_and(mask_inv, image)

                cv2.imshow("window", img2gray)
                cv2.waitKey(3)
                cv2.imshow("window", mask)

                h, w, d = image.shape
                search_top = 3*h/4
                search_bot = 3*h/4 + 20
                threshold[0:search_top, 0:w] = 0
                threshold[search_bot:h, 0:w] = 0

                M = cv2.moments(contours[0])
                if M['m00'] > 0:
                        print("hi")
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
#The proportional controller is implemented in the following four lines which
#is reposible of linear scaling of an error to drive the control output.
                        err = cx - w/2
                        self.twist.linear.x = 1.0
                        self.twist.angular.z = -float(err*.3) / 100
                        #self.cmd_vel_pub.publish(self.twist)
                #cv2.imshow("window", mask)
                cv2.imshow("window", image)
                cv2.waitKey(3)
"""

rospy.init_node('line_follower')
follower = Follower()
rospy.spin()
