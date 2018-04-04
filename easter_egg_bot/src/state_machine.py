#!/usr/bin/env python

# Modified from https://github.com/BrianEverRoboticsTeam/Egg-hunting/blob/master/src/state_machine_controller/scripts/state_machine.py
#
# This script creates a ROS node that sets up the transitions between the different states the robot can be in.
#

import rospy
import time
from smach import State, StateMachine
from smach_ros import IntrospectionServer

# Import Functions for states
from Initialization import Initialization
from Localization import Localization
from Explore import Explore
from Docking import DockAR, DockUA, Undock

if __name__ == '__main__':
    rospy.init_node('state_machine')

    sm = StateMachine(outcomes=['success'])
    with sm:

        # Initialization
        StateMachine.add('Initialization', Initialization(),
            transitions={
                'success': 'Localization'
            })

        # Localization node
        StateMachine.add('Localization', Localization(),
            transitions={
                'success': 'Explore',
                'fail': 'Localization'
            })

        # Look for targets
        StateMachine.add('Explore', Explore(),
            transitions={
                'successAR': 'DockAR',
                'successUA': 'DockUA',
                'lost': 'Localization'
            })

        # Dock at AR target
        StateMachine.add('DockAR', DockAR(),
            transitions={
                'success': 'Undock',
                'lost': 'Localization'
            })

        # Dock at target
        StateMachine.add('DockUA', DockUA(),
            transitions={
                'success': 'Undock',
                'lost': 'Localization'
            })


        StateMachine.add('Undock', Undock(),
            transitions={
                'success': 'Explore',
                'lost': 'Localization'
            })

    time.sleep(1)
    sis = IntrospectionServer('smach_Server', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
