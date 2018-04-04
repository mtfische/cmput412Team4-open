#!/usr/bin/env python

# Import ROS and Smach libraries
import rospy, time
from smach import State, StateMachine

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from kobuki_msgs.msg import Led
import actionlib
import math

def strip_nans(ranges):
  stripped_ranges = []
  for item in ranges:
      if not math.isnan(item):
          stripped_ranges.append(item)
  return stripped_ranges

class Localization(State):

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
                'fail'
            ],
            input_keys=[
                'pose_number'
            ],
            output_keys=[
                'pose_number'
            ])

        rospy.wait_for_service('move_base/clear_costmaps')
        try:
            self.clear_costmaps = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
        except:
            rospy.logerr("error waiting for clear_costmaps")
            return 'fail'

        rospy.wait_for_service('global_localization')
        try:
            self.global_localization = rospy.ServiceProxy('global_localization', Empty)

        except:
            rospy.logerr('Error during global_localization, retrying.')
            return 'fail'

        self.RED = 3
        self.ORANGE = 2
        self.GREEN = 1
        self.BLACK = 0
        self.led1 = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)
        self.led2 = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1)
        self.led1.publish(self.BLACK)
        self.led2.publish(self.BLACK)

    def execute(self, userdata):

        userdata.pose_number = userdata.pose_number

        self.global_localization()
        self.clear_costmaps()

        self.led1.publish(self.RED)

        cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        tw = Twist()
        rospy.logdebug('trying to localize...')

        tw.linear.x = 0.4
        # Move forward to better refine (not hitting a wall)
        timelimit = rospy.Time.now() + rospy.Duration(8)
        while rospy.Time.now() < timelimit and min(self.ranges) > 1.5:
            cmd_vel_pub.publish(tw)

        # Spin to try and localize

        tw.linear.x = 0.0
        tw.angular.z = 0.8
        timelimit = rospy.Time.now() + rospy.Duration(8)
        while rospy.Time.now() < timelimit- rospy.Duration(4):
            cmd_vel_pub.publish(tw)
        tw.angular.z = -0.8
        while rospy.Time.now() < timelimit:
            cmd_vel_pub.publish(tw)

        tw.angular.z = 0.0
        tw.linear.x = 0.4

        self.led1.publish(self.ORANGE)

        # Move forward to better refine (not hitting a wall)
        timelimit = rospy.Time.now() + rospy.Duration(9)
        while rospy.Time.now() < timelimit and min(self.ranges) > 1:
            cmd_vel_pub.publish(tw)

        tw.linear.x = 0.0

        tw.angular.z = 0.8
        timelimit = rospy.Time.now() + rospy.Duration(8)
        while rospy.Time.now() < timelimit- rospy.Duration(4):
            cmd_vel_pub.publish(tw)
        tw.angular.z = -0.8
        while rospy.Time.now() < timelimit:
            cmd_vel_pub.publish(tw)

        self.led1.publish(self.GREEN)

        # Have approximate position, move to center of room and re-do;
        #goal_pose = MoveBaseGoal()
        #goal_pose.target_pose.header.frame_id = 'map'
        #goal_pose.target_pose.pose.position.x = 3.46605
        #goal_pose.target_pose.pose.position.y = 1.03051
        #goal_pose.target_pose.pose.position.z = 0.0
        #goal_pose.target_pose.pose.orientation.x = 0.0
        #goal_pose.target_pose.pose.orientation.y = 0.0
        #goal_pose.target_pose.pose.orientation.z = -0.202725
        #goal_pose.target_pose.pose.orientation.w = 0.979236

        #client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #client.wait_for_server()
        #client.send_goal(goal_pose)
        #client.wait_for_result()

        # Re-localize
#        tw.angular.z = 0.8

#        timelimit = rospy.Time.now() + rospy.Duration(8)
#        while rospy.Time.now() < timelimit:
#            cmd_vel_pub.publish(tw)

#        tw.angular.z = 0.0
#        tw.linear.x = 0.4

        # Move forward to better refine (not hitting a wall)
#        timelimit = rospy.Time.now() + rospy.Duration(6)
#        while rospy.Time.now() < timelimit and min(self.ranges) > 1:
#            cmd_vel_pub.publish(tw)

#        tw.linear.x = 0.0
#        tw.angular.z = 0.8

#        timelimit = rospy.Time.now() + rospy.Duration(8)
#        while rospy.Time.now() < timelimit:
#            cmd_vel_pub.publish(tw)


        return 'success'
