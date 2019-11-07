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
from std_msgs.msg import String
from multi_robot_sim.msg import RobotDeliveryAction, RobotDeliveryGoal, RobotDeliveryResult


# define state Cozinha
class Cozinha(smach.State):
    request = False
    new_goal = MoveBaseGoal()

    def __init__(self):      
        smach.State.__init__(self, outcomes=['entregarPedido'],
                             output_keys=['goal'])
        # rospy.init_node('robot1_listener', anonymous=True)
        # rospy.Subscriber("robot1_delivery_request", String, self.getPos)
        


    def execute(self, userdata):
        rospy.loginfo('Executing state Cozinha')

        while(True): 
            if(self.request):
                self.request = False
                
                userdata.goal = self.new_goal

                return 'entregarPedido'
            else:
                time.sleep(0.5)

        else:
            print('cheguei aqui')

    def setGoal(self, pos_x, pos_y):
        self.request = True
        self.new_goal.target_pose.header.frame_id = "map"
        self.new_goal.target_pose.header.stamp = rospy.Time.now()
        self.new_goal.target_pose.pose.position.x = pos_x
        self.new_goal.target_pose.pose.position.y = pos_y
        self.new_goal.target_pose.pose.orientation.w = 1
        

    def getPos(self, data):
        pos = data.data.split(',')
        self.destino_x = int(pos[0])
        self.destino_y = int(pos[1])
        self.request = True
        

# define state RealizandoEntrega
class RealizandoEntrega(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['chegouDestino','retornarCozinha'],
                             input_keys=['goal'])
        self.move_base = actionlib.SimpleActionClient('robot1/move_base', MoveBaseAction)
        # self.pub = rospy.Publisher("robots_status_topic", String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo('Executing state RealizandoEntrega')
        
        self.move_base.wait_for_server()

        self.move_base.send_goal(userdata.goal)

        self.move_base.wait_for_result()

        state = self.move_base.get_state()
        # self.pub.publish("robot1:" + str(state))
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
    def __init__(self, deliveryRobot):
        smach.State.__init__(self, outcomes=['chegouCozinha', 'retornarCozinha', 'final'])
        self.move_base = actionlib.SimpleActionClient('robot1/move_base', MoveBaseAction)
        # pub = rospy.Publisher('request_channel', String, queue_size=10)
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = 1
        self.goal.target_pose.pose.position.y = 1
        self.goal.target_pose.pose.orientation.w = 1
        self.deliveryRobot = deliveryRobot  

    def execute(self, userdata):
        rospy.loginfo('Executing state RetornandoCozinha')

        self.move_base.wait_for_server()

        self.move_base.send_goal(self.goal)

        self.move_base.wait_for_result()    

        state = self.move_base.get_state()
        msg = "Finalizado com status: " + str(state)   

        if state == GoalStatus.SUCCEEDED:
            result = state
            self.deliveryRobot.setResult(result)
            return 'chegouCozinha'
        else:
            return 'retornarCozinha'
        return 'final'

class DeliveryRobot():
    waiting_result = True
    robotDeliveryResult = RobotDeliveryResult()


    def __init__(self, stateCozinha):
        self._as = actionlib.SimpleActionServer("robot1/robot_delivery", RobotDeliveryAction, execute_cb=self.send_coffe, auto_start = False)
        self._as.start()
        self.stateCozinha = stateCozinha
        self.robotDeliveryResult.robot = 1  #robot1


    def send_coffe(self, goal):
        self.stateCozinha.setGoal(goal.x,goal.y)

        while(True):
            if not self.waiting_result:
                self.waiting_result = True
                self._as.set_succeeded(self.robotDeliveryResult)
                return
            else:
                time.sleep(0.5)

        

    def setResult(self, result):
        self.robotDeliveryResult.status = result
        self.waiting_result = False
        return

       

def main():
    rospy.init_node('state_machine_robot1')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['FIM'])

    stateCozinha = Cozinha()
    deliveryRobot = DeliveryRobot(stateCozinha)
    stateRetornandoCozinha = RetornandoCozinha(deliveryRobot)

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

        smach.StateMachine.add('RETORNANDO_COZINHA', stateRetornandoCozinha, 
                               transitions={'chegouCozinha':'COZINHA', 'retornarCozinha':'RETORNANDO_COZINHA', 'final':'FIM'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('sm_robot1', sm, '/SM_ROBOT1')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()