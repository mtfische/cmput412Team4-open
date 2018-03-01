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
from sensor_msgs.msg import Joy
from copy import deepcopy
import math

class Follower:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()
                cv2.namedWindow("window", 1)

                self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=1)

                self.twist = Twist()

                # PD controller vars.
                self.err = 0
                self.lastErr = 0
                self.Kp = -0.04
                self.Kd = 0.01
                # Speed Vars
                self.cruiseSpeed = 1.1
                self.turnSpeed = 0.8
                self.ramp_rate = 0.3
                # Constants
                self.slowDownThreshold = 35 # Percent value

                self.printNumberFrame = 10
                self.frame = 0

                # Ramping
                self.last_send_time = rospy.Time.now()
                self.last_x_vel = 0

                # Joystick Control
                self.drive_switch = False
                self.scan_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

        def joy_callback(self, msg):
            if(msg.buttons[2] == 1):
                self.drive_switch = True
            elif(msg.buttons[1] == 1):
                self.drive_switch = False

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

        def image_callback(self, msg):

                # Open Bridge and get greyscale image.
                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                img2gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

                # Get the threshold for the image to find white line
                ret, threshold = cv2.threshold(img2gray, 250, 255, cv2.THRESH_BINARY)
                #cv2.imshow("window_thresh", threshold)

                # "Open" the image for less error in countouring.
                #kernel = np.ones((5,5),np.uint8)
                #opening = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, kernel)
                #cv2.imshow("window_opening", opening)

                ## Another option is to Blur it for smoothing for less error in countouring.
                #blur = cv2.GaussianBlur(img2gray,(9,9),0)
                #ret, thresh = cv2.threshold(blur, 253, 255, cv2.THRESH_BINARY)
                #cv2.imshow("window_blur", blur)

                # Find Close moment:
                h, w, d = image.shape
                search_top = 4*h/8
                search_bot = 4*h/8 + 20
                bottomThresh = deepcopy(threshold)
                bottomThresh[0:search_top, 0:w] = 0
                bottomThresh[0:h, 0:w/2] = 0
                Mclose = cv2.moments(bottomThresh)

                # Find Far moment:
                search_top = 3*h/8
                search_bot = 3*h/8 + 20
                topThresh = deepcopy(threshold)
                topThresh[0:search_top, 0:w] = 0
                topThresh[search_bot:, 0:w] = 0
                topThresh[0:h, 0:w/2] = 0
                Mfar = cv2.moments(topThresh)

                # Define how far we want to be from the right line
                desiredLocation = 1 * w / 3 + 10 # from right

                # Calculate with far information
                if Mclose['m00'] > 0 and Mfar['m00'] > 0:
                    # Calculate Moments
                    cxc = int(Mclose['m10']/Mclose['m00'])
                    cyc = int(Mclose['m01']/Mclose['m00'])

                    cxf = int(Mfar['m10']/Mfar['m00'])
                    cyf = int(Mfar['m01']/Mfar['m00'])

                    midxc = cxc - desiredLocation
                    midxf = cxf - 1 * desiredLocation / 3 - 20

                    # Draw Moments:
                    cv2.circle(image, (cxc, cyc), 20, (0,0,255), -1)
                    cv2.circle(image, (cxf, cyf), 20, (0,255,0), -1)
                    cv2.circle(image, (midxc, cyc), 20, (255,0,0), -1)
                    cv2.circle(image, (midxf, cyf), 20, (255,255,0), -1)


                    # BEGIN CONTROL
                    # Linear X velocity
                    percentTurn = (abs(midxc) - abs(midxf)) / abs(midxf) * 100
                    self.last_x_vel = self.twist.linear.x
                    if percentTurn < self.slowDownThreshold:
                        self.twist.linear.x = self.ramped_vel( self.last_x_vel, self.cruiseSpeed, self.last_send_time, rospy.Time.now())
                    else:
                        self.twist.linear.x = self.ramped_vel( self.last_x_vel, self.turnSpeed, self.last_send_time, rospy.Time.now())

                    # Angular Z Velocity
                    self.lastErr = float(self.err)
                    self.err = float(midxc - desiredLocation)
                    self.twist.angular.z = (float(self.Kp) * float(self.err)) + (float(self.Kd) * (float(self.err) - float(self.lastErr)))

                    # Show Debug information in window
                    if self.printNumberFrame == self.frame:
                        cv2.imshow("window", image)
                        cv2.waitKey(3)
                        self.frame = 0
                    else:
                        self.frame = self.frame + 1

                # Calculate without far information
                elif Mclose['m00'] > 0:
                    # Calculate Moments
                    cx = int(Mclose['m10']/Mclose['m00'])
                    cy = int(Mclose['m01']/Mclose['m00'])

                    midx = cx - desiredLocation

                    # Draw Moment
                    cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                    cv2.circle(image, (midx, cy), 20, (255,0,0), -1)

                    # BEGIN CONTROL
                    # Linear X speed (play it safe with no extra info)
                    self.last_x_vel = self.twist.linear.x
                    self.twist.linear.x = self.ramped_vel( self.last_x_vel, self.turnSpeed, self.last_send_time, rospy.Time.now())

                    # Angular Z speed
                    self.lastErr = float(self.err)
                    self.err = float(midx - desiredLocation)
                    self.twist.angular.z = (float(self.Kp) * float(self.err)) + (float(self.Kd) * (float(self.err) - float(self.lastErr)))

                    # Test small right turn
                    self.twist.angular.z = self.twist.angular.z + 0.7

                    # Show Debug information in window
                    if self.printNumberFrame == self.frame:
                        cv2.imshow("window", image)
                        cv2.waitKey(3)
                        self.frame = 0
                    else:
                        self.frame = self.frame + 1

                # Publish Velocity
                if self.drive_switch:
                    self.cmd_vel_pub.publish(self.twist)
                    self.last_send_time = rospy.Time.now()
                    rospy.loginfo("P: %f D: %f", float(self.Kp) * float(self.err), float(self.Kd) * (float(self.err) - float(self.lastErr)))
                else:
                    #self.last_x_vel = self.twist.linear.x
                    #self.twist.linear.x = self.ramped_vel( self.last_x_vel, 0, self.last_send_time, rospy.Time.now())
                    #self.twist.angular.z = 0
                    #self.cmd_vel_pub.publish(self.twist)
                    #self.last_send_time = rospy.Time.now()
                    pass


rospy.init_node('line_follower')
follower = Follower()
rospy.spin()
