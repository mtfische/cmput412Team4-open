#!/usr/bin/env python

# Import ROS and Smach libraries
import rospy, time
from smach import State, StateMachine

from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from kobuki_msgs.msg import Sound, Led
from geometry_msgs.msg import Twist
import actionlib
import random

# Waypoints to keep path planner near wall.
# [(x, y, z), (x, y, z, w)]

waypoints = [
    [(-2.4098443985, 2.9629099369, 0.0), (0.0, 0.0, -0.713371253953, 0.700786311248)],
    [(-2.41705656052, 1.00516152382, 0.0), (0.0, 0.0, -0.710566681734, 0.703629867764)],
    [(-2.51861333847, -1.02190923691, 0.0), (0.0, 0.0, -0.712760355217, 0.701407638988)],
    [(-2.21153736115, -1.5652077198, 0.0), (0.0, 0.0, 0.00722444764113, 0.999973903338)],
    [(-0.0192983150482, -1.65524935722, 0.0), (0.0, 0.0, 0.00120135202427, 0.999999278376)],
    [(1.32005691528, -1.12467670441, 0.0), (0.0, 0.0, -0.0768131644277, 0.997045504363)],
    [(3.39394044876, -1.86863732338, 0.0), (0.0, 0.0, 0.00151927788257, 0.999998845897)],
    [(5.12094926834, -1.70167732239, 0.0), (0.0, 0.0, -0.00664022341122, 0.999977953473)],
    [(6.94781541824, -1.77671217918, 0.0), (0.0, 0.0, -0.00329103742007, 0.999994584522)],
    [(7.94531917572, -1.10400295258, 0.0), (0.0, 0.0, 0.686409823069, 0.727214930261)],
    [(8.09117794037, 0.665054798126, 0.0), (0.0, 0.0, 0.703293837943, 0.710899273815)],
    [(8.05735015869, 2.51469230652, 0.0), (0.0, 0.0, -0.999338637288, 0.0363632785119)],
    [(6.03279066086, 2.23185753822, 0.0), (0.0, 0.0, 0.999996421112, 0.00267539950155)],
    [(3.86107063293, 2.37595415115, 0.0), (0.0, 0.0, 0.999789369041, 0.0205235852791)],
    [(1.98014903069, 2.47150921822, 0.0), (0.0, 0.0, 0.999994587499, 0.00329013277446)],
    [(0.450833797455, 2.66241931915, 0.0), (0.0, 0.0, 0.999997970481, 0.00201470465034)]
]

#From Github
#waypoints = [
#  [(-2.12, 2.88, 0.0), (0.0, 0.0, -0.93, 0.36)],
#  [(-2.45, -1.25, 0.0), (0.0, 0.0, -0.36, 0.93)],
#  [(7.20, -1.44, 0.0), (0.0, 0.0, 0.35, 0.94)],
#  [(6.92, 2.12, 0.0), (0.0, 0.0, 0.71, 0.71)]
#]

class Explore(State):

    def foundARListener(self, msg):
	print("AR CB!!!!")
        self.successAR = True

    def foundUAListener(self, msg):
        self.successUA = True

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

    def feedbackCallback(self, feedback):
        #rospy.loginfo("in feedbackCallback with feedback: %s", str(feedback))
        pass
    def activeCallback(self):
        rospy.loginfo('in activeCallback')

    def doneCallback(self, terminalState, result):
        rospy.loginfo('in doneCallback with terminalState: %s', str(terminalState))
        if terminalState == 3:
            self.goal_success = True
            print("true")
        if terminalState == 4:
            self.lost = True
            print('lost')

    def __init__(self):
        State.__init__(self,
            outcomes=[
                'successAR',
                'successUA',
                'lost'
            ],
            input_keys=[
                'pose_number'
            ],
            output_keys=[
                'pose_number'
            ])

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.successAR = False
        self.successUA = False

        self.ARListener = rospy.Subscriber('side_camera_ar', String, self.foundARListener)
        self.UAListener = rospy.Subscriber('side_camera_ua', String, self.foundUAListener)

        self.goal_success = True
        self.lost = False

        self.pose_number = 0

        self.left_state = False

        self.RED = 3
        self.ORANGE = 2
        self.GREEN = 1
        self.BLACK = 0
        self.led1 = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)
        self.led2 = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1)

    def execute(self, userdata):
        cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        tw = Twist()
        #tw.linear.x = 0.1

        # Move forward to better refine (not hitting a wall)
        #while True:
        #    cmd_vel_pub.publish(tw)
        #    if self.successAR:
        #        self.pose_number = (self.pose_number - 1) % len(waypoints)
         #       print("found AR")
        #        return 'successAR'
        #    rospy.sleep(0.1)


        while not rospy.is_shutdown():

            self.led2.publish(self.ORANGE)

            #rospy.loginfo("going to wapoint: %s", str(self.pose_number))
            #rospy.loginfo("Variables:\nSuccessAR: %s\nSuccessUA: %s\ngoal_success: %s", str(self.successAR), str(self.successUA), str(self.goal_success))

            # Check if we found a tag:
            if self.successAR:
                self.client.cancel_all_goals()
                self.led1.publish(self.ORANGE)
                self.led2.publish(self.GREEN)
                self.successAR = False
                self.left_state = True
                return 'successAR'

            if self.successUA:
                self.client.cancel_all_goals()
                self.led2.publish(self.ORANGE)
                self.led1.publish(self.GREEN)
                self.successUA = False
                self.left_state = True
                return 'successUA'

            # We did not find a tag, either continue going to waypoint or go to the next one.
            if self.goal_success:
                self.goal_success = False
                self.client.send_goal(self.goal_pose(waypoints[self.pose_number]), self.doneCallback, self.activeCallback, self.feedbackCallback)
                self.pose_number = (self.pose_number + 1) % len(waypoints)
                self.led2.publish(self.GREEN)

            if self.client.get_state() == 9 or self.lost == True:
                self.lost = False
                self.pose_number = (self.pose_number - 1) % len(waypoints)
                self.led2.publish(self.RED)
                return 'lost'

            if self.left_state:
                self.left_state = False
                self.client.send_goal(self.goal_pose(waypoints[self.pose_number]))

            #rospy.sleep(0.1)
