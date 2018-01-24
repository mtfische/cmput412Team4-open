#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import random 
import smach
import smach_ros

def scan_callback(msg):
  global g_range_ahead
  rospy.loginfo("The min scan = %f" % min(msg.ranges))
  rospy.loginfo("The middle scan = %f" % msg.ranges[len(msg.ranges)/2])
  g_range_ahead =  min(msg.ranges) # msg.ranges[len(msg.ranges)/2]


#input time to drive for
#if time elapses or an obstical is in front of us
#change state to spin
class Drive(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['stop_drive','drive'],
                               input_keys=['drive_interupt_time'],
                               output_keys=['state_interupt_out'])
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    self.scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
  
  def execute(self, userdata):
    rospy.sleep(.1)
    rospy.loginfo('Executing state Drive')

    if (g_range_ahead < 0.7 or rospy.Time.now() > userdata.drive_interupt_time):
      rospy.loginfo('Stop driving')
      userdata.state_interupt_out = rospy.Time.now() + rospy.Duration(5)
      return 'stop_drive'
    else:
      twist = Twist()
      twist.linear.x = 0.5
      self.cmd_vel_pub.publish(twist)
      return 'drive'

#input time to spin for
#if time elapses 
#change state to drive
class Spin(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['stop_spin','spin'],
                               input_keys=['spin_interupt_time'],
                               output_keys=['state_interupt_out'])
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

  def execute(self, userdata):
    rospy.sleep(.1)
    rospy.loginfo('Executing state Spin')

    if (rospy.Time.now() > userdata.spin_interupt_time):
      userdata.state_interupt_out = rospy.Time.now() + rospy.Duration(30)
      return 'stop_spin'
    else:
      twist = Twist()
      twist.angular.z = 0.5
      self.cmd_vel_pub.publish(twist)
      return 'spin'

    
def main():
  rospy.init_node("WanderSM")
  sm = smach.StateMachine(outcomes=['outcome4'])
  sm.userdata.state_interupt_time = rospy.Time.now()
  
  global g_range_ahead
  g_range_ahead = 2

  with sm:
    smach.StateMachine.add("Drive", Drive(), 
                           transitions={'stop_drive':'Spin', 'drive':'Drive'},
                           remapping={'drive_interupt_time':'state_interupt_time',
                                      'state_interupt_out':'state_interupt_time'})
    smach.StateMachine.add('Spin', Spin(), 
                           transitions={'stop_spin':'Drive', 'spin':'Spin'},
                           remapping={'spin_interupt_time':'state_interupt_time',
                                      'state_interupt_out':'state_interupt_time'})

  sis = smach_ros.IntrospectionServer('smach_Server', sm, '/SM_ROOT')
  sis.start()
  
  # Execute the state machine
  outcome = sm.execute()
  
  # Wait for ctrl-c to stop the application
  rospy.spin()
  sis.stop()

if __name__ == '__main__':
  main()


