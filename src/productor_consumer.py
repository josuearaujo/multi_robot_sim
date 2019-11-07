#!/usr/bin/env python
# license removed for brevity

import threading
import time
import logging
import random
import Queue
import rospy
from std_msgs.msg import String
import actionlib
from move_base_msgs.msg import MoveBaseAction
from multi_robot_sim.msg import RobotDeliveryAction, RobotDeliveryGoal, RobotDeliveryResult


logging.basicConfig(level=logging.DEBUG,
                    format='(%(threadName)-9s) %(message)s',)

BUF_SIZE = 100
coffe_requests = Queue.Queue(BUF_SIZE)
robots_status = [0,0]  # 0 -> Free   1 -> Busy

class ProducerThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs=None, verbose=None):
        super(ProducerThread,self).__init__()
        self.target = target
        self.name = name

        rospy.init_node('producer_listener', anonymous=True)
        rospy.Subscriber("coffe_request_channel", String, self.addToQueue)

    # def run(self):
    #     


    #     while True:
    #         if not coffe_requests.full():
    #             item = random.randint(1,10)
    #             coffe_requests.put(item)
    #             logging.debug('Putting ' + str(item)  
    #                           + ' : ' + str(coffe_requests.qsize()) + ' items in queue')
    #             time.sleep(random.random())
    #     return

    def addToQueue(self, data):
        rospy.loginfo(data.data + '  -- size: ' + str(coffe_requests.qsize()))
        coffe_requests.put(data.data)

class ConsumerThread(threading.Thread):
    def __init__(self, name, r):
        super(ConsumerThread,self).__init__()
        self.name = name
        self.robots = r

    def run(self):
        while True:
            if not coffe_requests.empty():
                # item = coffe_requests.get()
                # logging.debug('Getting ' + str(item) + ' : ' + str(coffe_requests.qsize()) + ' items in queue')
                for i in range(len(robots_status)):
                    if(robots_status[i] == 0):
                        pos = coffe_requests.get().split(',')
                        pos_x = int(pos[0])
                        pos_y = int(pos[1])

                        self.robots[i].setGoal(pos_x, pos_y)
        return

class RobotController(threading.Thread):
    receive_goal = False
    robotDeliveryGoal = RobotDeliveryGoal()
    def __init__(self, robot_server_name, robot_number):
        super(RobotController,self).__init__()
        self.robot_server_name = robot_server_name
        self.robot = actionlib.SimpleActionClient(robot_server_name, RobotDeliveryAction)
        self.robot_number = robot_number

    def run(self):
        while(True):
            if self.receive_goal:
                self.receive_goal = False             
                self.robot.wait_for_server()

                print(self.robot_server_name + " Goal foi enviado! (" + str(self.robotDeliveryGoal.x) + "," + str(self.robotDeliveryGoal.y) + ")")

                self.robot.send_goal(self.robotDeliveryGoal)

                self.robot.wait_for_result()

                result = self.robot.get_result()
                print(self.robot_server_name + " Resultado recebido!!")
                robots_status[self.robot_number-1] = 0

            else:
                time.sleep(0.5)

        return

    def setGoal(self, x, y):
        robots_status[self.robot_number-1] = 1
        self.robotDeliveryGoal.x = x
        self.robotDeliveryGoal.y = y
        self.receive_goal = True
        print(self.robot_server_name + " Goal foi setado!")
        return





# def robots_status_control(data):
#     robot_number = int(data.data.split(':')[0][len(data.data.split(':')[0])-1])
#     robots_status[robot_number-1] = 0;


if __name__ == '__main__':
    r = []

    for i in range(len(robots_status)):
            robot_server_name = '/robot' + str(i+1) +'/robot_delivery'
            r.append(RobotController(robot_server_name, i+1))
            r[i].start()
    
    p = ProducerThread(name='producer')
    p.start()
    c = ConsumerThread('consumer', r)
    c.start()


    # rospy.Subscriber("robots_status_topic", String, robots_status_control)
    rospy.spin()