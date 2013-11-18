#!/usr/bin/env python

import roslib; roslib.load_manifest('coffee_state')
import rospy
import smach
import smach_ros
import geometry_msgs
import move_base
import move_base_msgs

#Goal locations
elevator3_front = PoseStamped()
elevator3_back = PoseStamped()
elevator2_front = PoseStamped()
elevator2_back = PoseStamped()
cafe = PoseStamped()
lab = PoseStamped()

client = SimpleActionClient()

#Initializes the various goal locations from measured data
def init_locations():
    elevator3_front.header.frame_id = "/map"
    elevator3_front.pose.position.x = 16.78
    elevator3_front.pose.position.y = 19.13
    elevator3_front.pose.orientation.w = 1.0

#Move state to a specified location
class Move(smach.State):
    def __init__(self, loc, name):
        smach.State.__init__(self, outcomes=['done'])
        self.loc = loc
        self.name = name

    def execute(self, userdata):
        goal = MoveBaseActionGoal()
        goal.header.frame_id = "/map"
        goal.goal = loc
        goal.goal_id.stamp = ros.Time()
        goal.goal_id.id = name
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(1000.0))
        return 'done'

class Wait(smach.State):
    def __init__(self, msg):
        smach.State.__init__(self, outcomes=['done'])
        self.msg = msg

    def execute(self, userdata):
        print msg ++ ":"
        raw_input()
        return 'done'

class NavigateElevatorDown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['2nd_floor'])

    def execute(self, userdata):
        raw_input()
        move(elevator_back, "elevator_back")
        raw_input()
        move(elevator_front, "elevator_front")
        return '2nd_floor'

class NavigateElevatorUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['3rd_floor'])
        
    def execute(self, userdata):
        print "Press any key when elevator is here..."
        raw_input()
        move(elevator_back, "elevator_back")
        print "Press any key when elevator is at right floor..."
        raw_input()
        move(elevator_front, "elevator_front")
        return '3rd_floor'

def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['elevator_open'])

    # Open the container
    with sm:
        # Add states to the container
	smach.StateMachine.add('GOTOELEVATOR1', Move(elevator3_front, "elevator3_front"),
                               transitions={'done':'elevator_open'}) # MODIFIED LINE FOR DEMO
        smach.StateMachine.add('WAITFORELEVATOR1', Wait("Please call elevator and press a key..."),
                               transitions={'done':'NAVELEVATORDOWN'})
        smach.StateMachine.add('NAVELEVATORDOWN', NavigateElevator(elevator3_back, elevator3_front),
                               transitions={'done':'SWITCHTO2'})
        smach.StateMachine.add('SWITCHTO2', Wait('Please switch maps to the second floor and press a key...'),
                               transitions={'done':'MOVETOCAFE'})
        smach.StateMachine.add('MOVETOCAFE', Move(cafe, 'cafe'),
                               transitions={'done':'MOVETOELEVATOR2'})
        smach.StateMachine.add('MOVETOELEVATOR2', Move(elevator2_front, 'elevator2_front'),
                               transitions={'done':'WAITFORELEVATOR2'})
        smach.StateMachine.add('WAITFORELEVATOR2', Wait('Please call the elevator and press a key...'),
                               transitions={'done','NAVIGATEELEVATORUP'})
        smach.StateMachine.add('NAVELEVATORUP', NavigateElevator(elevator2_back, elevator2_front),
                               transitions={'done':'SWITCHTO3'})
        smach.StateMachine.add('SWITCHTO3', Wait('Please switch maps to the third floor and press a key...'),
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
