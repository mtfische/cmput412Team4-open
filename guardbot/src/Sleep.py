#!/usr/bin/env python

# Import ROS and Smach libraries
import rospy, time
from smach import State, StateMachine
from sensor_msgs.msg import Joy
#from VideoRecorder import VideoRecorder

class Sleep(State):

    def __init__(self):
        State.__init__(self,
            outcomes=[
                'start'
            ]
        )
        self.start = False
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        #self.recorder = VideoRecorder(0, "camera0.avi")

    def execute(self, userdata):
        #if self.recorder.isRecording():
        #    self.recorder.stop()

        while (not self.start):
            rospy.sleep(1)

        self.start = False

        #if not self.recorder.isRecording():
        #    self.recorder.start()

        return 'start'

    def joy_callback(self, msg):
        if(msg.buttons[1] == 1):
            self.start = True
        if(msg.buttons[2] == 1):
            self.start = False
