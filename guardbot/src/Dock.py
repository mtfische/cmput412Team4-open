#!/usr/bin/env python

# Import ROS and Smach libraries
import rospy, time
from smach import State, StateMachine
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from kobuki_msgs.msg import AutoDockingAction, AutoDockingGoal
import actionlib

class Dock(State):

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
        self.dock_client = actionlib.SimpleActionClient('dock_drive_action', AutoDockingAction)
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, userdata):

        self.move_client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(userdata.base_pose[0][0],
                                     userdata.base_pose[0][1],
                                     userdata.base_pose[0][2]),
                                     Quaternion(userdata.base_pose[1][0],
                                     userdata.base_pose[1][1],
                                     userdata.base_pose[1][2],
                                     userdata.base_pose[1][3])
                                    )
        self.move_client.send_goal(goal)
        success = self.move_client.wait_for_result(rospy.Duration(60))

        if (success):
            self.dock_client.wait_for_server()
            print "Trying to autodock."
            goal = AutoDockingGoal()
            self.dock_client.send_goal(goal)
            success = self.dock_client.wait_for_result(rospy.Duration(180))
            return 'success'
        else:
            return 'lost'
