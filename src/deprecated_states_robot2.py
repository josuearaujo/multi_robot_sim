#!/usr/bin/env python

# import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import random
import time


# define state Cozinha
class Cozinha(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['entregarPedido'],
                             output_keys=['goal'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Cozinha')
        new_goal = MoveBaseGoal()
        new_goal.target_pose.header.frame_id = "map"
        new_goal.target_pose.header.stamp = rospy.Time.now()
        new_goal.target_pose.pose.position.x = random.randint(-5, 2)
        new_goal.target_pose.pose.position.y = random.randint(-5, 2)
        new_goal.target_pose.pose.orientation.w = 1

        userdata.goal = new_goal

        return 'entregarPedido'

# define state RealizandoEntrega
class RealizandoEntrega(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['chegouDestino','retornarCozinha'],
                             input_keys=['goal'])
        self.client = actionlib.SimpleActionClient('robot2/move_base', MoveBaseAction)

    def execute(self, userdata):
        rospy.loginfo('Executing state RealizandoEntrega')
        
        self.client.wait_for_server()

        self.client.send_goal(userdata.goal)

        self.client.wait_for_result()

        state = self.client.get_state()

        if state == GoalStatus.SUCCEEDED:
            return 'chegouDestino'
        else:
            return 'retornarCozinha'
        

        


# define state Destino
class Destino(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['retornarCozinha'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Destino')
        time.sleep(5)
        return 'retornarCozinha'
        

# define state RetornandoCozinha
class RetornandoCozinha(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['chegouCozinha', 'retornarCozinha', 'final'])
        self.client = actionlib.SimpleActionClient('robot2/move_base', MoveBaseAction)
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = -1
        self.goal.target_pose.pose.position.y = 1
        self.goal.target_pose.pose.orientation.w = 1  

    def execute(self, userdata):
        rospy.loginfo('Executing state RetornandoCozinha')

        self.client.wait_for_server()

        self.client.send_goal(self.goal)

        self.client.wait_for_result()       

        state = self.client.get_state() 
        if state == GoalStatus.SUCCEEDED:
            return 'chegouCozinha'
        else:
            return 'retornarCozinha'
        return 'final'

def main():
    rospy.init_node('state_machine_robot2')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['FIM'])

    stateCozinha = Cozinha()
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('COZINHA', stateCozinha, 
                               transitions={'entregarPedido':'REALIZANDO_ENTREGA'})

        smach.StateMachine.add('REALIZANDO_ENTREGA', RealizandoEntrega(), 
                               transitions={'chegouDestino':'DESTINO', 'retornarCozinha':'RETORNANDO_COZINHA'},
                               remapping={'goal':'goal'})

        smach.StateMachine.add('DESTINO', Destino(), 
                               transitions={'retornarCozinha':'RETORNANDO_COZINHA'})

        smach.StateMachine.add('RETORNANDO_COZINHA', RetornandoCozinha(), 
                               transitions={'chegouCozinha':'COZINHA', 'retornarCozinha':'RETORNANDO_COZINHA', 'final':'FIM'})

     # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('sm_robot2', sm, '/SM_ROBOT2')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()