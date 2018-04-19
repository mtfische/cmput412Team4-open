#!/usr/bin/env python

# Import ROS and Smach libraries
import rospy, time
from smach import State, StateMachine

class CreateMap(State):

    def __init__(self):
        State.__init__(self,
            outcomes=[
                'success'
            ]
        )

    def execute(self, userdata):
        rospy.sleep(1.)
        return 'success'
