#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String
class ShowAR:

    def __init__(self):
        self.ar_sub = rospy.Subscriber('ar_pose_marker',AlvarMarkers, self.ar_callback)
        self.ar_pub = rospy.Publisher('side_camera_ar', String, queue_size=1)
        self.img = None
        self.timeout = rospy.Time.now()

    def ar_callback(self, msg):
	for i in range(len(msg.markers)):
            if(msg.markers[i].id != 4):
                return
            if(msg.markers[i].pose.pose.position.x < 0.1 and msg.markers[i].pose.pose.position.x> -0.1):
                #print("marker found")
                if(rospy.Time.now() >= self.timeout):
                    #publish
                    print("Tag Found:", msg.markers[i].id)
                    self.ar_pub.publish("Found")
                    self.timeout = rospy.Time.now() + rospy.Duration(20)
	            return
            else:
                print("out of ROI")

rospy.init_node('ARFinder')
showAR = ShowAR()
rospy.spin()
