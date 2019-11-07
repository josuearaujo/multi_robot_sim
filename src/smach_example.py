#!/usr/bin/env python

# import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
import time

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        time.sleep(5)
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        time.sleep(5)
        return 'outcome1'
        




def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 'outcome2':'outcome4'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'FOO'})

    # Execute SMACH plan

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('viewer', sm, '/SM_ROOT2')
    sis.start()
    outcome = sm.execute()

    rospy.spin()
    sis.stop() 



if __name__ == '__main__':
    main()