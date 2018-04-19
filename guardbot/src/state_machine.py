#!/usr/bin/env python

#
# This script creates the ROS State Machine for the GuardBot and sets up the transitions between the different states.
#

import rospy
import time
from smach import State, StateMachine
from smach_ros import IntrospectionServer

# Import Functions for states
from Initialization import Initialization
from Undock import Undock
from Localization import LocalizationStart, LocalizationLost
from CheckAsset import CheckAsset
from Patrol import Patrol
from Escape import GoToSafe
from Dock import Dock
from CreateMap import CreateMap
from Sleep import Sleep

if __name__ == '__main__':
    rospy.init_node('state_machine')

    sm = StateMachine(outcomes=['success'])
    with sm:

        # Initialization
        StateMachine.add('Initialization', Initialization(),
            transitions={
                'success': 'Sleep',
                'create_map': 'CreateMap'
            }
        )

        # sleep
        StateMachine.add('Sleep', Sleep(),
            transitions={
                'start': 'Undock'
            }
        )

        # Undocking
        StateMachine.add('Undock', Undock(),
            transitions={
                'success': 'LocalizationStart',
                'lost': 'LocalizationLost'
            }
        )

        # Refine Localization after undocking
        StateMachine.add('LocalizationStart', LocalizationStart(),
            transitions={
                'success': 'CheckAsset',
                'lost': 'LocalizationLost'
            })

        # Recover from lost state
        StateMachine.add('LocalizationLost', LocalizationLost(),
            transitions={
                'success': 'CheckAsset',
                'lost': 'LocalizationLost'
            }
        )

        # Check for asset, if its gone we lose go dock
        StateMachine.add('CheckAsset', CheckAsset(),
            transitions={
                'success': 'Patrol',
                'missing': 'GoToSafe',
                'end_of_shift': 'Dock',
                'detected': 'GoToSafe'
            }
        )

        # Patrol the waypoints
        StateMachine.add('Patrol', Patrol(),
            transitions={
                'success': 'CheckAsset',
                'detected': 'GoToSafe',
                'end_of_shift': 'Dock',
                'lost': 'LocalizationLost'
            }
        )

        # Go to a safe location if something is detected
        StateMachine.add('GoToSafe', GoToSafe(),
            transitions={
                'success': 'Patrol',
                'end_of_shift': 'Dock',
                'lost': 'LocalizationLost'
            }
        )

        # Dock at Charging station
        StateMachine.add('Dock', Dock(),
            transitions={
                'success': 'Sleep',
                'lost': 'LocalizationLost'
            })

        # Create A new map upon request
        StateMachine.add('CreateMap', CreateMap(),
            transitions={
                'success': 'Dock'
            }
        )


    time.sleep(1)
    sis = IntrospectionServer('smach_Server', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
