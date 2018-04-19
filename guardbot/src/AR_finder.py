#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String
class ShowAR:

    def __init__(self):
        self.ar_sub = rospy.Subscriber('ar_pose_marker',AlvarMarkers, self.ar_callback)
        self.ar_pub = rospy.Publisher('camera_ar', String, queue_size=0)
        self.img = None
        self.timeout = rospy.Time.now()

    def ar_callback(self, msg):
	for i in range(len(msg.markers)):
            if(msg.markers[i].id != 4):
                return
            else:
                self.ar_pub.publish("Found")
                rospy.sleep(1)
	        return

rospy.init_node('ARFinder')
showAR = ShowAR()
rospy.spin()
