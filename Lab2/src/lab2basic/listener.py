#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def recibir_mensaje(mensaje):
    rospy.loginfo("[%s] Mensaje recibido: %s", rospy.get_caller_id(), mensaje.data)

def nodo_escucha():
    rospy.init_node("escucha_oculta", anonymous=True)
    rospy.Subscriber("canal_secreto", String, recibir_mensaje)
    rospy.spin()

if __name__ == "__main__":
    nodo_escucha()
