#!/usr/bin/env python

# Import ROS and Smach libraries
import rospy, time
from smach import State, StateMachine
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Undock(State):

    def __init__(self):
        State.__init__(self,
            outcomes=[
                'success',
                'lost'
            ],
            input_keys=[
                'base_pose',
                'base_covariance',
                'asset',
                'waypoints',
                'safe'
            ],
            output_keys=[
                'base_pose',
                'base_covariance',
                'asset',
                'waypoints',
                'safe'
            ]
        )
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        #self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def execute(self, userdata):

        #return 'success'

        tw = Twist()
        tw.linear.x = -0.2

        timelimit = rospy.Time.now() + rospy.Duration(5)
        while rospy.Time.now() < timelimit:
            self.cmd_vel_pub.publish(tw)

        rospy.sleep(1)
        return 'success'
