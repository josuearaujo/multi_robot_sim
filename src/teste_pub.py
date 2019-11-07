#!/usr/bin/env python
# license removed for brevity

import rospy
import random
from std_msgs.msg import String


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('teste_pub', anonymous=True)
        print("Iniciou o no teste_pub")
        
        pub = rospy.Publisher('pub_teste', String, queue_size=10)
        rate = rospy.Rate(0.2)

        while not rospy.is_shutdown():
          msg = str(random.randint(-5, 2)) + ',' + str(random.randint(-5, 2))
          print('Enviando a seguinte string: ' + msg)
          pub.publish(msg)
          rate.sleep()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Test finished.")