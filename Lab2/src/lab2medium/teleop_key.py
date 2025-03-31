#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty

def leer_tecla():
    """Detecta una tecla presionada sin presionar Enter."""
    fd = sys.stdin.fileno()
    configuracion_inicial = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        tecla = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, configuracion_inicial)
    return tecla

def mostrar_ayuda():
    print("\n=== PANEL DE MOVIMIENTO ===")
    print("  u → Adelante (+X)")
    print("  o → Atrás (-X)")
    print("  i → Arriba (+Y)")
    print("  k → Abajo (-Y)")
    print("  a → Giro Izquierda")
    print("  d → Giro Derecha")
    print("  x → Salir")

def ejecutar_comando(tecla, publicador):
    movimiento = Twist()
    instrucciones = {
        'u': lambda: asignar(movimiento, lin_x=1.0, mensaje="Moviendo hacia la derecha"),
        'o': lambda: asignar(movimiento, lin_x=-1.0, mensaje="Retrocediendo hacia la izquierda"),
        'i': lambda: asignar(movimiento, lin_y=1.0, mensaje="Subiendo en el eje Y"),
        'k': lambda: asignar(movimiento, lin_y=-1.0, mensaje="Bajando en el eje Y"),
        'a': lambda: asignar(movimiento, ang_z=1.0, mensaje="Rotación antihoraria"),
        'd': lambda: asignar(movimiento, ang_z=-1.0, mensaje="Rotación horaria")
    }

    if tecla in instrucciones:
        instrucciones[tecla]()  # Ejecuta la lambda correspondiente
        publicador.publish(movimiento)

def asignar(twist_msg, lin_x=0.0, lin_y=0.0, ang_z=0.0, mensaje=""):
    twist_msg.linear.x = lin_x
    twist_msg.linear.y = lin_y
    twist_msg.angular.z = ang_z
    if mensaje:
        print(mensaje)

def iniciar():
    rospy.init_node('control_en_modo_comando', anonymous=True)
    canal = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    mostrar_ayuda()
    activo = True

    while activo and not rospy.is_shutdown():
        tecla_presionada = leer_tecla().lower()

        if tecla_presionada == 'x':
            print("Desactivando el nodo de control.")
            activo = False
        else:
            ejecutar_comando(tecla_presionada, canal)

if __name__ == '__main__':
    iniciar()
