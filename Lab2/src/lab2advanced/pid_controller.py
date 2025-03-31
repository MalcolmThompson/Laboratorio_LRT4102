#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import radians, sqrt, pi

class SistemaPIDTortuga:
    def __init__(self):
        rospy.init_node('navegacion_pid_virtual')
        self.pub_movimiento = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.sub_ubicacion = rospy.Subscriber('/turtle1/pose', Pose, self.actualizar_estado)

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.angulo = 0.0

        self.e_anterior_x = 0.0
        self.e_anterior_y = 0.0
        self.e_acumulado_x = 0.0
        self.e_acumulado_y = 0.0

        self.e_anterior_rot = 0.0
        self.e_acumulado_rot = 0.0

        self.velocidad_fija = rospy.Rate(10)

    def actualizar_estado(self, datos_pose):
        self.pos_x = datos_pose.x
        self.pos_y = datos_pose.y
        self.angulo = datos_pose.theta

    def detener_movimiento(self):
        msg = Twist()
        self.pub_movimiento.publish(msg)
        rospy.sleep(0.4)

    def ir_a_destino(self, meta_x, meta_y):
        kp = 0.85
        kd = 0.09
        ki = 0.005

        while not rospy.is_shutdown():
            ex = meta_x - self.pos_x
            ey = meta_y - self.pos_y
            distancia_total = sqrt(ex**2 + ey**2)

            self.e_acumulado_x += ex
            self.e_acumulado_y += ey

            vx = kp * ex + kd * (ex - self.e_anterior_x) + ki * self.e_acumulado_x
            vy = kp * ey + kd * (ey - self.e_anterior_y) + ki * self.e_acumulado_y

            self.e_anterior_x = ex
            self.e_anterior_y = ey

            comando = Twist()
            comando.linear.x = vx
            comando.linear.y = vy
            comando.angular.z = 0.0

            self.pub_movimiento.publish(comando)
            rospy.loginfo("Pos actual: (%.2f, %.2f) | Error: (%.2f, %.2f) | Distancia: %.2f", 
                          self.pos_x, self.pos_y, ex, ey, distancia_total)

            if distancia_total < 0.1:
                rospy.loginfo("Destino alcanzado.")
                break

            self.velocidad_fija.sleep()

        self.detener_movimiento()

    def ajustar_rotacion(self, objetivo_theta):
        kp_theta = 3.6
        kd_theta = 0.12
        ki_theta = 0.008

        while not rospy.is_shutdown():
            et = (objetivo_theta - self.angulo + pi) % (2 * pi) - pi
            self.e_acumulado_rot += et
            wz = kp_theta * et + kd_theta * (et - self.e_anterior_rot) + ki_theta * self.e_acumulado_rot
            self.e_anterior_rot = et

            msg = Twist()
            msg.angular.z = wz
            msg.linear.x = 0.0

            self.pub_movimiento.publish(msg)
            rospy.loginfo("Ajuste angular → Error: %.4f | Vel angular: %.4f", et, wz)

            if abs(et) < 0.05:
                rospy.loginfo("Orientación final alcanzada.")
                break

            self.velocidad_fija.sleep()

        self.detener_movimiento()

    def pedir_datos_usuario(self):
        print("\n Ingrese coordenadas objetivo:")
        destino_x = float(input("X → "))
        destino_y = float(input("Y → "))
        grados = float(input("Ángulo en grados → "))
        return destino_x, destino_y, radians(grados)

    def iniciar_rutina(self):
        while not rospy.is_shutdown():
            x_meta, y_meta, angulo_meta = self.pedir_datos_usuario()
            self.ir_a_destino(x_meta, y_meta)
            self.ajustar_rotacion(angulo_meta)

def ejecutar_navegacion():
    try:
        robot_pid = SistemaPIDTortuga()
        robot_pid.iniciar_rutina()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    ejecutar_navegacion()
