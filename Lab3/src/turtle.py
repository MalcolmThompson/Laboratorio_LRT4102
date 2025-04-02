#!/usr/bin/env python3

import rospy
from turtlesim.srv import Kill, Spawn
from math import sqrt, atan2, degrees, radians

def eliminar_tortuga(nombre_tortuga):
    rospy.wait_for_service("/kill")
    try:
        solicitar_kill = rospy.ServiceProxy("/kill", Kill)
        solicitar_kill(nombre_tortuga)
    except rospy.ServiceException:
        rospy.logwarn(f"[Aviso] No se logró eliminar '{nombre_tortuga}', ya podría no existir.")

def crear_nueva_tortuga(px, py, angulo_grados, identificador):
    rospy.wait_for_service("/spawn")
    try:
        orientacion = radians(angulo_grados)
        nueva = rospy.ServiceProxy("/spawn", Spawn)
        nueva(px, py, orientacion, identificador)
        return (px, py, orientacion)
    except rospy.ServiceException as err:
        rospy.logerr(f"[ERROR] Falló el proceso de creación: {err}")
        return None

def capturar_entrada_usuario():
    print("Parámetros para la nueva tortuga")
    nuevo_x = float(input("Coordenada X: "))
    nuevo_y = float(input("Coordenada Y: "))
    nuevo_angulo = float(input("Ángulo inicial (en grados): "))
    return nuevo_x, nuevo_y, nuevo_angulo

def calcular_datos_orientacion(x_obj, y_obj, x_ini, y_ini):
    distancia = sqrt((x_obj - x_ini)**2 + (y_obj - y_ini)**2)
    angulo_rad = atan2((y_obj - y_ini), (x_obj - x_ini))
    return distancia, degrees(angulo_rad)

def iniciar_proceso():
    rospy.init_node("generador_de_tortuga", anonymous=True)

    x_meta, y_meta, theta_meta = capturar_entrada_usuario()
    
    eliminar_tortuga("turtle1")
    
    resultado = crear_nueva_tortuga(x_meta, y_meta, theta_meta, "turtle1")

    if resultado:
        x_actual, y_actual, ang_actual = resultado
        distancia_meta, angulo_deseado = calcular_datos_orientacion(x_meta, y_meta, x_actual, y_actual)

        print(f"\nDistancia calculada al objetivo: {distancia_meta:.3f} unidades")
        print(f"Dirección hacia el objetivo: {angulo_deseado:.2f}°")

if __name__ == "__main__":
    try:
        iniciar_proceso()
    except rospy.ROSInterruptException:
        pass
