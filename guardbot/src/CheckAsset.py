#!/usr/bin/env python

# Import ROS and Smach libraries
import rospy, time
from smach import State, StateMachine
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class CheckAsset(State):

    def __init__(self):
        State.__init__(self,
            outcomes=[
                'success',
                'missing',
                'end_of_shift',
                'detected'
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
        self.found = False
        self.end = False
        self.ar_sub = rospy.Subscriber('camera_ar', String, self.ar_callback)
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        self.person_detected = False
        self.person_sub = rospy.Subscriber('person_detect', String, self.person_callback)
        self.goal_success = False
        self.lost = False

    def execute(self, userdata):

        self.found = False
        self.end = False
        self.person_detected = False
        self.goal_success = False
        self.lost = False

        #return 'success'

        goal = MoveBaseGoal()
        rospy.loginfo("Checking the Asset")
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(userdata.asset[0][0],
                                     userdata.asset[0][1],
                                     userdata.asset[0][2]),
                                     Quaternion(userdata.asset[1][0],
                                     userdata.asset[1][1],
                                     userdata.asset[1][2],
                                     userdata.asset[1][3])
                                    )
        self.move_client.send_goal(goal, self.doneCallback, self.activeCallback, self.feedbackCallback)
        #self.goal_success = True
        while not rospy.is_shutdown() and not self.goal_success:
            if self.lost:
                self.lost = False
                return 'lost'

            elif self.person_detected:
                self.person_detected = False
                print("CheckAsset: Person detected.")
                #return 'detected'

            elif (self.end):
                self.end = False
                rospy.loginfo("Ending Shift. Returning to dock")
                return 'end_of_shift'

            rospy.sleep(0.5)

        self.goal_success = False

        self.timeout = rospy.Time.now() + rospy.Duration(5)
        while (self.timeout > rospy.Time.now() and not rospy.is_shutdown()):
            rospy.sleep(.5)
            rospy.loginfo("Waiting")
            if (self.end):
                self.end = False
                rospy.loginfo("Ending Shift. Returning to dock")
                return 'end_of_shift'
            elif (self.found):
                self.found = False
                rospy.loginfo("Asset confirmed")
                return 'success'
            elif(self.person_detected):
                self.person_detected = False
                rospy.loginfo("Check Asset: Person Detected")
                #return 'detected'

        # Temorary Person Detection:
        self.person_detected = False
        while not rospy.is_shutdown() and not self.person_detected:
            rospy.logwarn("Waiting to see a person...")
            rospy.sleep(1)
        self.person_detected = False
        return 'detected'

        return 'missing'

    def ar_callback(self, msg):
        if(msg.data == "Found"):
            self.found = True

    def joy_callback(self, msg):
        if(msg.buttons[2] == 1):
            self.end = True

    def person_callback(self, msg):
        if msg.data == "Found":
            self.person_detected = True

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

    def setVarsToInitial(self):
        self.found = False
        self.end = False
        self.person_detected = False
        self.person_sub = rospy.Subscriber('person_detect', String, self.person_callback)
        self.goal_success = False
        self.lost = False
