#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import radians, sqrt, pi

class ControladorTortuga:
    def __init__(self):
        rospy.init_node('nodo_direccion_tortuga')
        self.publicador = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.suscriptor = rospy.Subscriber('/turtle1/pose', Pose, self.actualizar_estado)

        self.estado_x = 0.0
        self.estado_y = 0.0
        self.estado_angulo = 0.0

        self.frecuencia = rospy.Rate(10)

    def actualizar_estado(self, datos):
        self.estado_x = datos.x
        self.estado_y = datos.y
        self.estado_angulo = datos.theta

    def trasladar_a_objetivo(self, destino_x, destino_y):
        Kp_posicion = 0.75
        en_movimiento = True

        while en_movimiento and not rospy.is_shutdown():
            error_x = destino_x - self.estado_x
            error_y = destino_y - self.estado_y
            distancia = sqrt(error_x**2 + error_y**2)

            mensaje = Twist()
            mensaje.linear.x = Kp_posicion * error_x
            mensaje.linear.y = Kp_posicion * error_y
            self.publicador.publish(mensaje)

            rospy.loginfo("Moviendo → Actual: (%.2f, %.2f) | Error: (%.2f, %.2f)", 
                          self.estado_x, self.estado_y, error_x, error_y)

            if distancia < 0.1:
                rospy.loginfo("Destino alcanzado.")
                en_movimiento = False

            self.frecuencia.sleep()

        self.publicador.publish(Twist())

    def ajustar_orientacion(self, angulo_deseado):
        Kp_rotacion = 3.5
        rotando = True

        while rotando and not rospy.is_shutdown():
            error_theta = (angulo_deseado - self.estado_angulo + pi) % (2 * pi) - pi
            velocidad = Kp_rotacion * error_theta

            mensaje = Twist()
            mensaje.angular.z = velocidad
            self.publicador.publish(mensaje)

            rospy.loginfo("Rotando → Error: %.3f | z: %.3f", error_theta, velocidad)

            if abs(error_theta) < 0.05:
                rospy.loginfo("Orientación ajustada.")
                rotando = False

            self.frecuencia.sleep()

        self.publicador.publish(Twist())

    def capturar_entrada_usuario(self):
        print("\n--- Coordenadas destino ---")
        x = float(input("Posición X: "))
        y = float(input("Posición Y: "))
        grados = float(input("Ángulo deseado (°): "))
        return x, y, radians(grados)

    def iniciar(self):
        while not rospy.is_shutdown():
            destino_x, destino_y, destino_theta = self.capturar_entrada_usuario()
            self.trasladar_a_objetivo(destino_x, destino_y)
            self.ajustar_orientacion(destino_theta)

def main():
    try:
        robot = ControladorTortuga()
        robot.iniciar()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
