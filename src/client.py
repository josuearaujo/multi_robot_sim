#!/usr/bin/env python
# license removed for brevity

import rospy
import random
from std_msgs.msg import String
import time

def generate_list():
  lista = []
  coordinatesStream = "20,13.5,18,13.5,16,13.5,14,13.5,19,18,-8.1,18.2,-9.9,18.2,-11.7,18.2,-8.1,13.7,-9.9,3.7,-11.7,13.7,-8.1,9.2,-9.9,9.2,-11.7,9.2,-15.3,18.2,-17.1,18.2,-18.9,18.2,-20.7,18.2,-15.3,13.7,-17.1,13.7,-18.9,13.7,-20.7,13.7,-15.3,9.2,-17.1,9.2,-18.9,9.2,-20.7,9.2,-6.9,14.7,-8.7,14.7,-10.5,14.7,-6.9,10.2,-8.7,10.2,-10.5,10.2,-6.9,5.7,-8.7,5.7,-10.5,5.7,-14.1,14.7,-15.9,14.7,-17.7,14.7,-19.5,14.7,-14.1,10.2,-15.9,10.2,-17.7,10.2,-19.5,10.2,-14.1,5.7,-15.9,5.7,-17.7,5.7,-19.5,5.7,3,14.3,-3,12.6,3,8.2,-3,6.5,10,21,5,21,-11.5,21,-16,21,-20.5,21,-24.5,-18.8,-25,3,-19,3,-13,3".split(',')
  for i in xrange(0, len(coordinatesStream), 2):
    coord = coordinatesStream[i] + ',' + coordinatesStream[i+1]
    lista.append(coord)
  print(lista)
  return lista


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    lista = generate_list()
    try:
        rospy.init_node('coffee_client', anonymous=True)
        print("Iniciou o no coffee_client")
        
        pub = rospy.Publisher('coffe_request_channel', String, queue_size=10)

        time.sleep(2)

        raw_input("Aperte ENTER para enviar um cafe!") 
        while not rospy.is_shutdown():
          # msg = str(random.randint(-5, 2)) + ',' + str(random.randint(-5, 2))
          msg = random.choice(lista)
          print('Msg: ' + msg)
          pub.publish(msg)
          raw_input("Aperte ENTER para enviar outro cafe!")      
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Test finished.")