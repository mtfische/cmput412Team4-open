#!/usr/bin/env python

# Import ROS and Smach libraries
import rospy, time
from smach import State, StateMachine
import math

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

def strip_nans(ranges):
  stripped_ranges = []
  for item in ranges:
      if not math.isnan(item):
          stripped_ranges.append(item)
  if len(stripped_ranges) == 0:
      stripped_ranges.append(0)
  return stripped_ranges

class LocalizationStart(State):

    def __init__(self):

        rospy.wait_for_service('move_base/clear_costmaps')
        try:
            self.clear_costmaps = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
        except:
            rospy.logerr("error waiting for clear_costmaps")
            return 'fail'

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
        self.initialPose = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=3)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        #self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def execute(self, userdata):

        #return 'success'

        # Publish to AMCL initial Pose:
        posewcov = PoseWithCovarianceStamped()
        posewcov.header.frame_id = "map"
        posewcov.header.stamp = rospy.Time.now()
        posewcov.pose.pose.position.x = userdata.base_pose[0][0]
        posewcov.pose.pose.position.y = userdata.base_pose[0][1]
        posewcov.pose.pose.position.z = userdata.base_pose[0][2]
        posewcov.pose.pose.orientation.x = userdata.base_pose[1][0]
        posewcov.pose.pose.orientation.y = userdata.base_pose[1][1]
        posewcov.pose.pose.orientation.z = userdata.base_pose[1][2]
        posewcov.pose.pose.orientation.w = userdata.base_pose[1][3]
        posewcov.pose.covariance = userdata.base_covariance


        self.initialPose.publish(posewcov)
        self.clear_costmaps()


        tw = Twist()

        # Spin to try and localize
        tw.linear.x = 0.0
        tw.angular.z = 0.6
        timelimit = rospy.Time.now() + rospy.Duration(12)
        while rospy.Time.now() < timelimit- rospy.Duration(9):
            self.cmd_vel_pub.publish(tw)
        tw.angular.z = -0.6
        while rospy.Time.now() < timelimit - rospy.Duration(3):
            self.cmd_vel_pub.publish(tw)

        tw.angular.z = 0.6
        while rospy.Time.now() < timelimit:
            self.cmd_vel_pub.publish(tw)

        return 'success'

class LocalizationLost(State):

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
            ]
        )

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

    def execute(self, userdata):

        self.clear_costmaps()
        self.global_localization()

        cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        tw = Twist()
        rospy.logdebug('trying to localize...')

        tw.linear.x = 0.4
        # Move forward to better refine (not hitting a wall)
        timelimit = rospy.Time.now() + rospy.Duration(8)
        while rospy.Time.now() < timelimit and min(self.ranges) > 1.5:
            cmd_vel_pub.publish(tw)

        # Spin to try and localize
        tw.angular.z = 0.6
        timelimit = rospy.Time.now() + rospy.Duration(12)
        while rospy.Time.now() < timelimit- rospy.Duration(9):
            cmd_vel_pub.publish(tw)
        tw.angular.z = -0.6
        while rospy.Time.now() < timelimit - rospy.Duration(3):
            cmd_vel_pub.publish(tw)

        tw.angular.z = 0.6
        while rospy.Time.now() < timelimit:
            cmd_vel_pub.publish(tw)

        # Move forward to better refine (not hitting a wall)
        timelimit = rospy.Time.now() + rospy.Duration(9)
        while rospy.Time.now() < timelimit and min(self.ranges) > 1:
            cmd_vel_pub.publish(tw)

        tw.linear.x = 0.0
        tw.angular.z = 0.8

        # Spin to try and localize
        tw.angular.z = 0.6
        timelimit = rospy.Time.now() + rospy.Duration(12)
        while rospy.Time.now() < timelimit- rospy.Duration(9):
            cmd_vel_pub.publish(tw)
        tw.angular.z = -0.6
        while rospy.Time.now() < timelimit - rospy.Duration(3):
            cmd_vel_pub.publish(tw)
        tw.angular.z = 0.6
        while rospy.Time.now() < timelimit:
            cmd_vel_pub.publish(tw)

        return 'success'
