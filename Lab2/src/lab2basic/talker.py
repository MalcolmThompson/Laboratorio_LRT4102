#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def emisor():
    pub = rospy.Publisher('canal_secreto', String, queue_size=10)
    rospy.init_node('transmisor_oculto', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz

    contador = 0
    while not rospy.is_shutdown():
        mensaje = f"Mensaje encriptado {contador}"
        rospy.loginfo("[Emisor]: %s", mensaje)
        pub.publish(mensaje)
        rate.sleep()
        contador += 1

if __name__ == "__main__":
    try:
        emisor()
    except rospy.ROSInterruptException:
        pass
