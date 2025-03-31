#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill
import sys
import tty
import termios
import time

def capturar_caracter():
    """Obtiene una tecla del usuario sin esperar Enter."""
    descriptor = sys.stdin.fileno()
    estado_original = termios.tcgetattr(descriptor)
    try:
        tty.setraw(descriptor)
        tecla = sys.stdin.read(1)
    finally:
        termios.tcsetattr(descriptor, termios.TCSADRAIN, estado_original)
    return tecla

def borrar_tortuga(identificador):
    rospy.wait_for_service('/kill')
    try:
        comando_kill = rospy.ServiceProxy('/kill', Kill)
        comando_kill(identificador)
    except:
        rospy.loginfo(f"Tortuga {identificador} ya no estaba presente.")

def crear_tortuga(posicion_x, posicion_y, orientacion, nombre):
    rospy.wait_for_service('/spawn')
    try:
        comando_spawn = rospy.ServiceProxy('/spawn', Spawn)
        comando_spawn(posicion_x, posicion_y, orientacion, nombre)
    except Exception as error:
        rospy.logerr(f"Fallo en creación de tortuga: {error}")

def cuadrado_manual(pub):
    señal = Twist()

    # Giro inicial
    señal.linear.x = 0.0
    señal.angular.z = 1.57
    pub.publish(señal)
    time.sleep(2)

    # Movimiento recto
    señal.linear.x = 2.0
    señal.angular.z = 0.0
    pub.publish(señal)
    time.sleep(2)

    # Giro 2
    señal.linear.x = 0.0
    señal.angular.z = 1.57
    pub.publish(señal)
    time.sleep(2)

    # Movimiento recto 2
    señal.linear.x = 2.0
    señal.angular.z = 0.0
    pub.publish(señal)
    time.sleep(2)

    # Giro 3
    señal.linear.x = 0.0
    señal.angular.z = 1.57
    pub.publish(señal)
    time.sleep(2)

    # Movimiento recto 3
    señal.linear.x = 2.0
    señal.angular.z = 0.0
    pub.publish(señal)
    time.sleep(2)

    # Giro 4
    señal.linear.x = 0.0
    señal.angular.z = 1.57
    pub.publish(señal)
    time.sleep(2)

    # Movimiento recto final
    señal.linear.x = 2.0
    señal.angular.z = 0.0
    pub.publish(señal)
    time.sleep(2)

def triangulo_manual(pub):
    señal = Twist()

    # Giro 1
    señal.linear.x = 0.0
    señal.angular.z = 2.094
    pub.publish(señal)
    time.sleep(2)

    # Avance 1
    señal.linear.x = 2.0
    señal.angular.z = 0.0
    pub.publish(señal)
    time.sleep(2)

    # Giro 2
    señal.linear.x = 0.0
    señal.angular.z = 2.094
    pub.publish(señal)
    time.sleep(2)

    # Avance 2
    señal.linear.x = 2.0
    señal.angular.z = 0.0
    pub.publish(señal)
    time.sleep(2)

    # Giro 3
    señal.linear.x = 0.0
    señal.angular.z = 2.094
    pub.publish(señal)
    time.sleep(2)

    # Avance 3
    señal.linear.x = 2.0
    señal.angular.z = 0.0
    pub.publish(señal)
    time.sleep(2)

def mostrar_menu():
    print("\n=== Control de Trazado ===")
    print("Presiona A: figura cuadrada")
    print("Presiona B: figura triangular")
    print("Presiona E: salir del sistema")

def ciclo_operativo():
    rospy.init_node('figura_geometrica_virtual', anonymous=True)
    emisor = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    mostrar_menu()
    borrar_tortuga("turtle1")

    sistema_activo = True
    while sistema_activo and not rospy.is_shutdown():
        opcion = capturar_caracter().lower()

        if opcion == 'e':
            print("Cerrando simulador. Gracias.")
            sistema_activo = False

        elif opcion == 'a':
            crear_tortuga(2.0, 2.0, 0, "turtle1")
            cuadrado_manual(emisor)
            borrar_tortuga("turtle1")
            mostrar_menu()

        elif opcion == 'b':
            crear_tortuga(8.5, 8.5, 0, "turtle1")
            triangulo_manual(emisor)
            borrar_tortuga("turtle1")
            mostrar_menu()

if __name__ == '__main__':
    ciclo_operativo()
