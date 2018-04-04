#!/usr/bin/env python

# Import ROS and Smach libraries
import rospy, time
from smach import State, StateMachine

class Initialization(State):

    def __init__(self):
        State.__init__(self,
            outcomes=[
                'success'
            ],
            output_keys=[
                'pose_number'
            ])

    def execute(self, userdata):
        rospy.sleep(1.)
        userdata.pose_number = 0
        return 'success'
