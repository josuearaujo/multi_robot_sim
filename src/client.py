#!/usr/bin/env python
# license removed for brevity

import rospy
import random
from std_msgs.msg import String
import time

def generate_list():
  lista = []
  coordinatesStream = "0,0,0,-1,0,-2,0,-5,1,0,1,2,1,-5,2,0,2,1,2,-5,-1,2,-1,0,-1,-1,-1,-2,-1,-3,-1,-4,-1,-5,-2,2,-2,1,-2,0,-2,-1,-2,-2,-2,-5,-4,-5,-5,-5,-3,-4,-4,-4,-5,-4,-3,-3,-4,-3,-5,-3,-4,-2,-5,-2".split(',')
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
        print("Iniciou o no teste_pub")
        
        pub = rospy.Publisher('coffe_request_channel', String, queue_size=10)

        time.sleep(2)


        while not rospy.is_shutdown():
          # msg = str(random.randint(-5, 2)) + ',' + str(random.randint(-5, 2))
          msg = random.choice(lista)
          print('Msg: ' + msg)
          pub.publish(msg)
          raw_input("Aperte ENTER para enviar outro cafe!")      
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Test finished.")