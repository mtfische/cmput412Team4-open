#!/usr/bin/env python

# Import ROS and Smach libraries
import rospy, time
from smach import State, StateMachine
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from kobuki_msgs.msg import Sound, Led
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import math
import random

def strip_nans(ranges):
  stripped_ranges = []
  for item in ranges:
      if not math.isnan(item):
          stripped_ranges.append(item)
  return stripped_ranges

class DockAR(State):

    def scan_callback(self, msg):
        #rospy.loginfo("The min scan = %f" % min(msg.ranges))
        #rospy.loginfo("The middle scan = %f" % msg.ranges[len(msg.ranges)/2])
        self.ranges = strip_nans(msg.ranges[155:485])
        if(len(self.ranges) == 0):
            self.ranges = [0]

    def __init__(self):
        self.ranges = [100]
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        State.__init__(self,
            outcomes=[
                'success',
                'lost'
            ],
            input_keys=[
                'pose_number'
            ],
            output_keys=[
                'pose_number'
            ])

        self.RED = 3
        self.ORANGE = 2
        self.GREEN = 1
        self.BLACK = 0
        self.led1 = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)
        self.led2 = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1)
        self.soundSource = "/home/malcolm/ar.wav"
        self.soundSource2 = "/home/malcolm/beep.wav"
        self.soundHandle = SoundClient()

    def execute(self, userdata):
        print("Docking Now!")
        rospy.sleep(0.1)
        self.soundHandle.playWave(self.soundSource)
        tw = Twist()
        # Spin to try and localize
        cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

        rospy.sleep(0.5)

        tw.angular.z = -0.6
        timelimit = rospy.Time.now() + rospy.Duration(3.5)
        while rospy.Time.now() < timelimit:
            cmd_vel_pub.publish(tw)

	tw.angular.z = 0.0
        tw.linear.x = 0.2
	while(min(self.ranges) > 0.5):
            cmd_vel_pub.publish(tw)
            rospy.sleep(0.1)
        rospy.sleep(1)
        self.soundHandle.playWave(self.soundSource2)
        rospy.sleep(2)
        return 'success'

class DockUA(State):
    def __init__(self):
        State.__init__(self,
            outcomes=[
                'success',
                'lost'
            ],
            input_keys=[
                'pose_number'
            ],
            output_keys=[
                'pose_number'
            ])

        self.RED = 3
        self.ORANGE = 2
        self.GREEN = 1
        self.BLACK = 0
        self.led1 = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)
        self.led2 = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1)

    def execute(self, userdata):
        rospy.sleep(1)
        temp = random.randint(0,50)
        if temp < 40:
            return 'success'
        else:
            return 'lost'


class Undock(State):
    def __init__(self):
        State.__init__(self,
            outcomes=[
                'success',
                'lost'
            ],
            input_keys=[
                'pose_number'
            ],
            output_keys=[
                'pose_number'
            ])

        self.RED = 3
        self.ORANGE = 2
        self.GREEN = 1
        self.BLACK = 0
        self.led1 = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)
        self.led2 = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1)

    def execute(self, userdata):

        cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        tw = Twist()
        tw.linear.x = -0.1
        timelimit = rospy.Time.now() + rospy.Duration(4)
        while rospy.Time.now() < timelimit:
            cmd_vel_pub.publish(tw)

        tw.linear.x = 0.0
        tw.angular.z = 0.6

        timelimit = rospy.Time.now() + rospy.Duration(3)
        while rospy.Time.now() < timelimit:
            cmd_vel_pub.publish(tw)

        return 'success'
