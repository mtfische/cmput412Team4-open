#!/usr/bin/env python

import rospy
import actionlib
import tf
import os

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Joy

global drive

# Set this flag if maprace will be run in the simulator.
sim = False

#massive
# Waypoints for track
waypoints = [
   #[(x    , y    , z  ), (x  , y  , z   , w   )]
    [(6.948, 3.936, 0.0), (0.0, 0.0, 0.969, 0.245)],
    [(3.136, 5.171, 0.0), (0.0, 0.0, -0.845, 0.530)],
    [(0.388, -1.631, 0.0), (0.0, 0.0, -0.144, 0.989)],
    [(4.756, -2.818, 0.0), (0.0, 0.0, 0.680, 0.732)]
]

#waypoints = [
#   #[(x    , y    , z  ), (x  , y  , z   , w   )]
#    [(5.764, 3.381, 0.0), (0.0, 0.0, 0.982, 0.185)],
#    [(3.686, 4.405, 0.0), (0.0, 0.0, -0.96, 0.278)],
#    [(1.411, -0.376, 0.0), (0.0, 0.0, -0.49, 0.87)],
#    [(4.523, -1.337, 0.0), (0.0, 0.0, 0.232, 0.97)]
#]

# Waypoints for corner
#waypoints = [
   #[(x    , y    , z  ), (x  , y  , z   , w   )]
#    [(4.091, -2.49, 0.0), (0.0, 0.0, 0.98, 0.14)],
#    [(0.845, -1.26, 0.0), (0.0, 0.0, -0.82, 0.57)],
#    [(0.198, -2.84, 0.0), (0.0, 0.0, -0.16, 0.98)],
#    [(3.924, -3.97, 0.0), (0.0, 0.0, 0.572, 0.82)]
#]

# Waypoints in Sumulator:
waypoints_sim = [
   #[(x    , y    , z  ), (x  , y  , z     , w   )]
    [(-3.10, 0.843, 0.0), (0.0, 0.0, -0.920, 0.36)],
    [(-0.58, -2.43, 0.0), (0.0, 0.0, -0.178, 0.98)],
    [(1.211, -1.82, 0.0), (0.0, 0.0, -0.589, 0.81)],
    [(1.104, 1.151, 0.0), (0.0, 0.0, 0.8890, 0.45)]
]

if sim:
    rospy.loginfo("\n\n\t\tRunning maprace in simulation mode\n\n")
    waypoints = waypoints_sim

client = None
curr_pose = None

if sim:
    global drive
    drive = True
else:
    global drive
    drive = False

def joy_callback(msg):
    if(msg.buttons[2] == 1):
        global drive
        drive = True
    elif(msg.buttons[1] == 1):
        global drive
        drive = False


def goal_pose(pose):
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


if __name__ == '__main__':
    rospy.init_node('maprace')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    joy_sub = rospy.Subscriber('joy', Joy, joy_callback)

    rospy.loginfo("\n\n\t\tWaiting to go to start position.\n\n")

    r = rospy.Rate(20)
    # Idle until Ready to move.
    while not drive:
        r.sleep()

    if not sim:
        global drive
        drive = False

    rospy.loginfo("\n\n\t\tGoing to start position\n\n")
    # Go to the start point:
    client.send_goal(goal_pose(waypoints[0]))
    client.wait_for_result()

    rospy.loginfo("\n\n\t\tReady\n\n")

    # Idle until start
    while not drive:
        r.sleep()

    rospy.loginfo("\n\n\t\tDriving\n\n")
    # Need to change this into a for loop
    #while drive:
    for i in range(2):
        for pose in waypoints:
            curr_pose = pose
            goal = goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result()
            if not drive:
                break
        else:
            continue
        break
