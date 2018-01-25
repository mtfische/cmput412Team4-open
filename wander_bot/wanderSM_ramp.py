#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import random
import smach
import smach_ros
import math

g_last_twist = None
g_last_twist = Twist()
g_last_send_time = None
g_vel_scales = [0.1, 0.1] #default to very slow
g_vel_ramps = [3,3] #units: m/s^2

#def scan_callback(msg):
#  global g_range_ahead
#  rospy.loginfo("The min scan = %f" % min(msg.ranges))
#  rospy.loginfo("The middle scan = %f" % msg.ranges[len(msg.ranges)/2])
#  g_range_ahead =  min(msg.ranges) # msg.ranges[len(msg.ranges)/2]

def scan_callback(msg):
  global g_range_ahead
  #rospy.loginfo("The min scan = %f" % min(msg.ranges))
  #rospy.loginfo("The middle scan = %f" % msg.ranges[len(msg.ranges)/2])
  ranges = strip_nans(msg.ranges[163:477])
  rospy.loginfo("interested ranges: " + str(ranges))
  if len(ranges) == 0:
    g_range_ahead = 10
  else:
    g_range_ahead =  min(ranges) # msg.ranges[len(msg.ranges)/2]

def strip_nans(ranges):
  stripped_ranges = []
  for item in ranges:
      if not math.isnan(item):
          stripped_ranges.append(item)
  return stripped_ranges

def ramped_vel(v_prev, v_target, t_prev, t_now, ramp_rate):
  #compute the maximum velocity to step
  step = ramp_rate * (t_now - t_prev).to_sec()
  sign = 1.0 if (v_target > v_prev) else -1.0
  error = math.fabs(v_target - v_prev)
  if error < step: #we can get ther within this time step and we are done!
    return v_target
  else:
    return v_prev + sign * step #step towards target

def ramped_twist(prev, target, t_prev, t_now, ramps):
  tw = Twist()
  tw.angular.z = ramped_vel(prev.angular.z, target.angular.z, t_prev, t_now, ramps[0])
  tw.linear.x = ramped_vel(prev.linear.x, target.linear.x, t_prev, t_now, ramps[1])
  return tw

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
    self.target_twist = Twist()
    self.target_twist.linear.x = 0.2
    self.last_twist_send_time = rospy.Time.now()

  def execute(self, userdata):
    global g_target_twist, g_last_twist, g_vel_scales, g_vel_ramps
    rospy.sleep(.1)
    rospy.loginfo('Executing state Drive')

    if (g_range_ahead < 0.7 or rospy.Time.now() > userdata.drive_interupt_time):
      rospy.loginfo('Stop driving')
      userdata.state_interupt_out = rospy.Time.now() + rospy.Duration(3)
      return 'stop_drive'
    else:
      #twist = Twist()
      #twist.linear.x = 0.5
      t_now = rospy.Time.now()
      g_last_twist = ramped_twist(g_last_twist, self.target_twist, self.last_twist_send_time, t_now, g_vel_ramps)
      print("--------------------------------twist")
      self.last_twist_send_time = t_now
      self.cmd_vel_pub.publish(g_last_twist)
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
    self.target_twist = Twist()
    self.target_twist.angular.z = 0.5

  def execute(self, userdata):
    rospy.sleep(.1)
    rospy.loginfo('Executing state Spin')

    if (rospy.Time.now() > userdata.spin_interupt_time):
      userdata.state_interupt_out = rospy.Time.now() + rospy.Duration(15)
      return 'stop_spin'
    else:
      twist = Twist()
      twist.angular.z = 0.5
      self.cmd_vel_pub.publish(twist)
      return 'spin'


def main():
  rospy.init_node("WanderSM")

  g_last_twist = Twist()

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
