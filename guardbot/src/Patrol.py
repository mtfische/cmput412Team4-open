#!/usr/bin/env python

# Import ROS and Smach libraries
import rospy, time
from smach import State, StateMachine

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import Twist
import actionlib

class Patrol(State):

    def __init__(self):
        State.__init__(self,
            outcomes=[
                'success',
                'detected',
                'end_of_shift',
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

        self.goal_success = False
        self.lost = False
        self.end = False
        self.found = False
        self.person_sub = rospy.Subscriber('person_detect', String, self.person_callback)
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def joy_callback(self, msg):
        if(msg.buttons[2] == 1):
            self.end = True

    def person_callback(self, msg):
        if(msg.data == "Found"):
            self.found = True

    def feedbackCallback(self, feedback):
        #rospy.loginfo("in feedbackCallback with feedback: %s", str(feedback))
        pass
    def activeCallback(self):
        #rospy.loginfo('in activeCallback')
        pass

    def doneCallback(self, terminalState, result):
        #rospy.loginfo('in doneCallback with terminalState: %s', str(terminalState))
        if terminalState == 3:
            self.goal_success = True
            #print("true")
        if terminalState == 4:
            self.lost = True
            #print('lost')

    def goal_pose(self, pose):
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose.position.x = pose[0][0]
        goal_pose.target_pose.pose.position.y = pose[0][1]
        goal_pose.target_pose.pose.position.z = pose[0][2]
        goal_pose.target_pose.pose.orientation.x = pose[1][0]
        goal_pose.target_pose.pose.orientation.y = pose[1][1]
        goal_pose.target_pose.pose.orientation.z = pose[1][2]
        goal_pose.target_pose.pose.orientation.w = pose[1][3]
        return goal_pose

    def sentry(self):
        tw = Twist()
        tw.angular.z = 0.6

        timelimit = rospy.Time.now() + rospy.Duration(10)
        while rospy.Time.now() < timelimit and not self.end and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(tw)

    def execute(self, userdata):

        self.move_client.send_goal(self.goal_pose(userdata.safe))        
        
        success = self.move_client.wait_for_result(rospy.Duration(60))

        self.move_client.send_goal(self.goal_pose(userdata.waypoints[0]), self.doneCallback, self.activeCallback, self.feedbackCallback)
        i = 1
        rospy.logwarn("Going to position 0")
        #self.goal_success = True

        while not rospy.is_shutdown():

            # Check if end of shift.
            if self.end:
                self.move_client.cancel_all_goals()
                self.end = False
                return 'end_of_shift'

            # Check if we detected a person.
            if self.found:
                #self.move_client.cancel_all_goals()
                self.found = False
                print("Patrol: Person detected.")
                #return 'detected'

            # Check if lost.
            if self.lost:
                self.lost = False
                return 'lost'

            if self.goal_success:
                if i > 3:
                    break
                self.goal_success = False
                self.sentry()
                self.move_client.send_goal(self.goal_pose(userdata.waypoints[i]), self.doneCallback, self.activeCallback, self.feedbackCallback)
                rospy.logwarn("Going to position: %d", i)
                #self.goal_success = True
                i = i+1
            rospy.sleep(0.1)
        rospy.logwarn("Patrol finished")
        return 'success'
