#!/usr/bin/env python

import roslib; roslib.load_manifest('coffee_state')
import rospy
import smach
import smach_ros
import geometry_msgs
import move_base
import move_base_msgs

door_front = PoseStamped()
door_back = PoseStamped()
elevator_front = PoseStamped()
client = SimpleActionClient()



def move(loc, name):

class Move(smach.State):
    def __init__(self, loc, name):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        goal = MoveBaseActionGoal()
        goal.header.frame_id = "/map"
        goal.goal = loc
        goal.goal_id.stamp = ros.Time()
        goal.goal_id.id = name
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(1000.0))
        return 'done'

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
        

        move(door_front, "door_front")
	return 'door_closed'

# define state AskForHelpWithDoor
class AskForHelpWithDoor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['door_open'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state AskForHelpWithDoor')
        # Move to the door here and detect if it's open
        return 'door_open'

# define state GoToElevator
class MoveToElevator1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['elevator_open'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state GoToElevator')
	#move_base_pub.publish(elevator_door3)
        elevator_front.header.frame_id = "/map"

        elevator_front.pose.position.x = 33.34
        elevator_front.pose.position.y = 15.22
        elevator_front.pose.position.z = 0.0

        elevator_front.pose.position.x = 0.0
        elevator_front.pose.position.y = 0.0
        elevator_front.pose.position.z = 0.0
        elevator_front.pose.position.w = 1.0

        move(elevator_front, "elevator_front")
        return 'wait_elevator'

class WaitForElevator1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['elevator_open'])

    def execute(self, userdata):
        return 'elevator_open'

class NavigateElevatorDown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['2nd_floor'])

    def execute(self, userdata):

        move(elevator_back, "elevator_back")

        move(elevator_front, "elevator_front")
        
        return '2nd_floor'

class MoveToCafe(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['at_cafe'])

    def execute(self, userdata):
        return 'at_cafe'

class MoveToElevator2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['at_elevator'])

    def execute(self, userdata):
        return 'at_elevator'

class WaitForElevator2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['elevator_open'])

    def execute(self, userdata):
        return 'elevator_open'

class NavigateElevatorUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['3rd_floor'])
        
    def execute(self, userdata):
        move(elevator_back, "elevator_back")

        move(elevator_front, "elevator_front")

        return '3rd_floor'

class MoveToLab(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['at_lab'])

    def execute(self, userdata):
        move(lab, "lab")

        return 'at_lab'

def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['elevator_open'])

    # Open the container
    with sm:
        # Add states to the container
	smach.StateMachine.add('GOTOELEVATOR1', Move(elevator3_front, "elevator3_front"),
                               transitions={'done':'WAITFORELEVATOR1'})
        smach.StateMachine.add('WAITFORELEVATOR1', Wait(),
                               transitions={'done':'NAVELEVATORDOWN'})
        smach.StateMachine.add('NAVELEVATORDOWN', NavigateElevator(elevator3_back, elevator3_front),
                               transitions={'done':'SWITCHTO2'})
        smach.StateMachine.add('SWITCHTO2', Wait(),
                               transitions={'done':'MOVETOCAFE'})
        smach.StateMachine.add('MOVETOCAFE', Move(cafe, 'cafe'),
                               transitions={'done':'MOVETOELEVATOR2'})
        smach.StateMachine.add('MOVETOELEVATOR2', Move(elevator2_front, 'elevator2_front'),
                               transitions={'done':'WAITFORELEVATOR2'})
        smach.StateMachine.add('WAITFORELEVATOR2', Wait(),
                               transitions={'done','NAVIGATEELEVATORUP'})
        smach.StateMachine.add('NAVELEVATORUP', NavigateElevator(elevator2_back, elevator2_front),
                               transitions={'done':'SWITCHTO3'})
        smach.StateMachine.add('SWITCHTO3', Wait(),
                               transitions={'done':'MOVETOLAB'})
        smach.StateMachine.add('MOVETOLAB', Move(lab, "lab"),
                               transitions={'done':'at_lab'})
                               

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
