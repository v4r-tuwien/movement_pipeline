#! /usr/bin/env python3
import rospy
import smach
import smach_ros
from states.userinput import UserInput
from states.robot_control import GoToWaypoint


def create_statemachine(all_rooms = False):
    sm = smach.StateMachine(outcomes=['sm_done'])

    cb0525_waypoint = GoToWaypoint(0.84, -2.7, 0, timeout=120.0)
    cb0523_waypoint = GoToWaypoint(0.35, 0.41, 0, timeout=120.0) # table_waypoint
    cb0519_waypoint = GoToWaypoint(0.07, 6.95, 0, timeout=120.0)
    cb0515_waypoint = GoToWaypoint(-0.01, 13.1, 0, timeout=120.0)
    cb0511_waypoint = GoToWaypoint(0.2, 19.3, 0, timeout=120.0)
    cb0509_waypoint = GoToWaypoint(0.15, 22.7, 0, timeout=120.0)

    with sm:
        map = {'0': ['end', 'Stop statemachine'],
               '1': ['cb0523', 'CB0523: Home'],
               '2': ['cb0525', 'CB0525: Kitchen'],
               '3': ['cb0519', 'CB0519: Matthias, Peter & Tessa'],
               '4': ['cb0515', 'CB0515: JB, Paolo & Philipp'],
               '5': ['cb0511', 'CB0511: Bernhard & Hrishikesh'],
               '6': ['cb0509', 'CB0509: Markus'],
               '7': ['cb_all', 'All offices']}
        
        smach.StateMachine.add("HANDLE_USERINPUT", UserInput(map), transitions={'end': 'sm_done', 'cb0525': 'GO_TO_CB0525',
                                                                                'cb0523': 'GO_TO_CB0523', 'cb0519': 'GO_TO_CB0519',
                                                                                'cb0515': 'GO_TO_CB0515', 'cb0511': 'GO_TO_CB0511',
                                                                                'cb0509': 'GO_TO_CB0509', 'cb_all': 'GO_TO_ALL'})
        smach.StateMachine.add('GO_TO_CB0525', cb0525_waypoint, transitions={'succeeded': 'HANDLE_USERINPUT', 'aborted': 'GO_TO_CB0523'})
        smach.StateMachine.add('GO_TO_CB0523', cb0523_waypoint, transitions={'succeeded': 'HANDLE_USERINPUT', 'aborted': 'sm_done'})
        smach.StateMachine.add('GO_TO_CB0519', cb0519_waypoint, transitions={'succeeded': 'HANDLE_USERINPUT', 'aborted': 'GO_TO_CB0523'})
        smach.StateMachine.add('GO_TO_CB0515', cb0515_waypoint, transitions={'succeeded': 'HANDLE_USERINPUT', 'aborted': 'GO_TO_CB0523'})
        smach.StateMachine.add('GO_TO_CB0511', cb0511_waypoint, transitions={'succeeded': 'HANDLE_USERINPUT', 'aborted': 'GO_TO_CB0523'})
        smach.StateMachine.add('GO_TO_CB0509', cb0509_waypoint, transitions={'succeeded': 'HANDLE_USERINPUT', 'aborted': 'GO_TO_CB0523'})

        seq = smach.Sequence(outcomes=['succeeded', 'aborted'], connector_outcome='succeeded')


        with seq:

            smach.Sequence.add('CB0523_start', cb0523_waypoint, transitions={'succeeded': 'CB0525', 'aborted': 'aborted'})
            smach.Sequence.add('CB0525', cb0525_waypoint, transitions={'succeeded': 'CB0519', 'aborted': 'aborted'})
            smach.Sequence.add('CB0519', cb0519_waypoint, transitions={'succeeded': 'CB0515', 'aborted': 'aborted'})
            smach.Sequence.add('CB0515', cb0515_waypoint, transitions={'succeeded': 'CB0511', 'aborted': 'aborted'})

            if all_rooms:
                smach.Sequence.add('CB0511', cb0511_waypoint, transitions={'succeeded': 'CB0509', 'aborted': 'aborted'})
                smach.Sequence.add('CB0509', cb0509_waypoint, transitions={'succeeded': 'CB0523_end', 'aborted': 'aborted'})
            else:
                smach.Sequence.add('CB0511', cb0511_waypoint, transitions={'succeeded': 'CB0523_end', 'aborted': 'aborted'})

            smach.Sequence.add('CB0523_end', cb0523_waypoint, transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})


        smach.StateMachine.add('GO_TO_ALL', seq, transitions={'succeeded': 'HANDLE_USERINPUT', 'aborted': 'GO_TO_CB0523'})

        #smach.StateMachine.add('GO_TO_ALL', all_waypoint, transitions={'succeeded': 'HANDLE_USERINPUT', 'aborted': 'GO_TO_CB0523'})

    return sm

if __name__ == '__main__':
    rospy.init_node('movement_statemachine')
    sm = create_statemachine()

    try:
        # Create and start the introspection server
        sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()

        # Execute state machine
        outcome = sm.execute()

        # Wait for ctrl-c to stop the application
        # rospy.spin()
        sis.stop()
    except rospy.ROSInterruptException:
        pass
