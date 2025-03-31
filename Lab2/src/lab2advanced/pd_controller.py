#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import radians, sqrt, pi

class ControlPD_Tortuga:
    def __init__(self):
        rospy.init_node('nodo_pd_tortuga')

        self.publicador = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.suscriptor = rospy.Subscriber('/turtle1/pose', Pose, self.actualizar_pose)

        self.x_actual = 0.0
        self.y_actual = 0.0
        self.orientacion_actual = 0.0

        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_ang = 0.0

        self.frecuencia = rospy.Rate(10)

    def actualizar_pose(self, datos_pose):
        self.x_actual = datos_pose.x
        self.y_actual = datos_pose.y
        self.orientacion_actual = datos_pose.theta

    def rotar_a_angulo(self, angulo_objetivo):
        Kp_ang = 3.7
        Kd_ang = 0.15

        while not rospy.is_shutdown():
            error_angulo = (angulo_objetivo - self.orientacion_actual + pi) % (2 * pi) - pi
            velocidad_ang = Kp_ang * error_angulo + Kd_ang * (error_angulo - self.prev_error_ang)
            self.prev_error_ang = error_angulo

            orden = Twist()
            orden.angular.z = velocidad_ang

            self.publicador.publish(orden)
            rospy.loginfo(" Giro → Desfase: %.3f | Vel z: %.3f", error_angulo, velocidad_ang)

            if abs(error_angulo) < 0.05:
                rospy.loginfo(" Orientación lograda.")
                break

            self.frecuencia.sleep()

        self.pausar_movimiento()

    def avanzar_hasta(self, destino_x, destino_y):
        Kp = 0.9
        Kd = 0.12

        avanzando = True
        while avanzando and not rospy.is_shutdown():
            dif_x = destino_x - self.x_actual
            dif_y = destino_y - self.y_actual
            distancia = sqrt(dif_x**2 + dif_y**2)

            vel_lin_x = Kp * dif_x + Kd * (dif_x - self.prev_error_x)
            vel_lin_y = Kp * dif_y + Kd * (dif_y - self.prev_error_y)

            self.prev_error_x = dif_x
            self.prev_error_y = dif_y

            comando = Twist()
            comando.linear.x = vel_lin_x
            comando.linear.y = vel_lin_y

            self.publicador.publish(comando)
            rospy.loginfo(" Avance → Pos: (%.2f, %.2f) | Error: (%.2f, %.2f)", 
                          self.x_actual, self.y_actual, dif_x, dif_y)

            if distancia < 0.1:
                rospy.loginfo(" Llegada al destino.")
                avanzando = False

            self.frecuencia.sleep()

        self.pausar_movimiento()

    def capturar_datos_objetivo(self):
        print("\n Ingrese coordenadas y ángulo destino:")
        objetivo_x = float(input(" Coordenada X → "))
        objetivo_y = float(input(" Coordenada Y → "))
        angulo_grados = float(input(" Ángulo deseado (grados) → "))
        return objetivo_x, objetivo_y, radians(angulo_grados)

    def iniciar_rutina(self):
        while not rospy.is_shutdown():
            destino_x, destino_y, destino_theta = self.capturar_datos_objetivo()
            self.avanzar_hasta(destino_x, destino_y)
            self.rotar_a_angulo(destino_theta)

    def pausar_movimiento(self):
        pausa = Twist()
        self.publicador.publish(pausa)
        rospy.sleep(0.4)

def ejecutar():
    try:
        robot = ControlPD_Tortuga()
        robot.iniciar_rutina()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    ejecutar()
