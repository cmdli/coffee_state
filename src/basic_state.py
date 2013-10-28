#!/usr/bin/env python

import roslib; roslib.load_manifest('coffee_state')
import rospy
import smach
import smach_ros
import geometry_msgs
import move_base

pub = rospy.publisher('move_base_simple/goal', PoseStamped)
door_front = PoseStamped()
door_back = PoseStamped()
elevator_front = PoseStamped()

# define state CheckDoors
class CheckDoors(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['more_doors','no_more_doors'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state CheckDoors')
        # Figure out which door
        return 'no_more_doors'

# define state MoveToDoor
class MoveToDoor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['door_open','door_closed'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state MoveToDoor')
        # Move to the door here and detect if it's open
	return 'door_open'

# define state AskForHelpWithDoor
class AskForHelpWithDoor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['door_open'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state AskForHelpWithDoor')
        # Move to the door here and detect if it's open
        return 'door_open'


# define state GoThroughDoor
class GoThroughDoor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['successfull','unsuccessfull'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state GoThroughDoor')
        return 'successfull'

# define state GoToElevator
class GoToElevator(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['elevator_open'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state GoToElevator')
	#move_base_pub.publish(elevator_door3)
        elevator_front.header.frame_id = "/map"

        elevator_front.pose.position.x = 0.0
        elevator_front.pose.position.y = 0.0
        elevator_front.pose.position.z = 0.0

        elevator_front.pose.position.x = 0.0
        elevator_front.pose.position.y = 0.0
        elevator_front.pose.position.z = 0.0
        elevator_front.pose.position.w = 1.0

        pub.publish(elevator_front)
        return 'elevator_open'

def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['elevator_open'])

    # Open the container
    with sm:
        # Add states to the container
#        smach.StateMachine.add('CHECKDOORS', CheckDoors(), 
#                               transitions={'more_doors':'MOVETODOOR', 'no_more_doors':'GOTOELEVATOR'})
        # smach.StateMachine.add('MOVETODOOR', MoveToDoor(), 
        #                        transitions={'door_open':'GOTHROUGHDOOR','door_closed':'ASKFORHELPWITHDOOR'})
	# smach.StateMachine.add('ASKFORHELPWITHDOOR', AskForHelpWithDoor(),
        #                        transitions={'door_open':'GOTHROUGHDOOR'})
	# smach.StateMachine.add('GOTHROUGHDOOR', GoThroughDoor(),
        #                        transitions={'successfull':'GOTOELEVATOR','unsuccessfull':'ASKFORHELPWITHDOOR'})
	smach.StateMachine.add('GOTOELEVATOR', GoToElevator(),
                               transitions={'elevator_open':'elevator_open'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('coffee_state_server', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
