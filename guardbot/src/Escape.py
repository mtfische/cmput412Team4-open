#!/usr/bin/env python

# Import ROS and Smach libraries
import rospy, time
from smach import State, StateMachine
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Joy
import actionlib

class GoToSafe(State):

    def __init__(self):
        State.__init__(self,
            outcomes=[
                'success',
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
        self.end = False
        self.timeout = rospy.Time.now()
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

    def execute(self, userdata):
        self.move_client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(userdata.safe[0][0],
                                     userdata.safe[0][1],
                                     userdata.safe[0][2]),
                                     Quaternion(userdata.safe[1][0],
                                     userdata.safe[1][1],
                                     userdata.safe[1][2],
                                     userdata.safe[1][3])
                                    )
        self.move_client.send_goal(goal)
        success = self.move_client.wait_for_result(rospy.Duration(60))

        if(success):
            self.timeout = rospy.Time.now() + rospy.Duration(20)
            self.end = False
            while not self.end and self.timeout > rospy.Time.now():
                rospy.sleep(1)
            if(self.end):
                rospy.loginfo("Ending Shift. Returning to dock")
                return 'end_of_shift'
            else:
                return 'success'
        else:
            rospy.loginfo("We are lost")
            return 'lost'

    def joy_callback(self, msg):
        if(msg.buttons[1] == 1):
            self.end = True
