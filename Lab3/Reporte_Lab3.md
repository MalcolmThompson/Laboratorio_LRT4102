# Reporte Laboratorio 3
## Introducción

En este laboratorio se desarrollan dos actividades fundamentales utilizando el simulador Turtlesim dentro del entorno de ROS (Robot Operating System). La primera consiste en calcular métricas de navegación como la distancia al objetivo (Distance to Goal, DTG) y el ángulo hacia el objetivo (Angle to Goal, ATG), para luego hacer aparecer una tortuga directamente en dicha posición sin necesidad de movimiento. La segunda actividad implica mover activamente una tortuga hacia una posición objetivo, calculando trayectorias a partir de coordenadas euclidianas y utilizando un controlador proporcional, repitiendo este proceso en un bucle.

Estas actividades tienen como finalidad introducir los conceptos de control proporcional, mapeo de velocidades y uso de servicios de ROS como `spawn` y `kill`, que permiten gestionar dinámicamente entidades dentro del simulador. Además, se trabaja con la suscripción y publicación de tópicos en ROS, reforzando la programación orientada a eventos y el diseño de bucles reactivos.

---

## Conceptos Clave para el Desarrollo

Para llevar a cabo estas prácticas se requiere conocer y aplicar ciertos conceptos fundamentales de navegación y control de robots móviles:

### Distancia Euclidiana (DTG)

La **distancia al objetivo** es una medida directa entre dos puntos en el plano, calculada mediante el teorema de Pitágoras:  
$$
\text{DTG} = \sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}
$$
Esta métrica permite estimar qué tan lejos está el robot de su destino (Siciliano et al., 2010).

### Ángulo hacia el objetivo (ATG)

El **ángulo hacia el objetivo** representa la dirección en la que el robot debe orientarse para dirigirse hacia la posición meta. Se obtiene mediante la función `atan2`, que calcula el arco tangente considerando el signo de ambos argumentos:
$$
\text{ATG} = \tan^{-1}\left(\frac{y_2 - y_1}{x_2 - x_1}\right)
$$
Este cálculo se hace en radianes y posteriormente se convierte a grados para facilitar la interpretación humana (Craig, 2005).

### Mapeo de Velocidades

Una vez conocidas la distancia y el ángulo hacia el objetivo, se puede establecer un sistema de control que relacione estos errores con velocidades. El **mapeo de velocidades** consiste en traducir el error espacial en comandos de movimiento, normalmente con ecuaciones proporcionales como:
$$
v = K_p \cdot \text{DTG} \quad ; \quad \omega = K_\theta \cdot (\text{ATG} - \theta_{\text{actual}})
$$
donde `v` es la velocidad lineal y `ω` la angular (Quigley et al., 2009).

### Control Proporcional

El **control proporcional** (P) es una estrategia de control donde la salida (en este caso las velocidades) es directamente proporcional al error. Esta técnica es común por su simplicidad y respuesta rápida en tareas de navegación (Nise, 2015). En este laboratorio, se aplica tanto al movimiento hacia la meta como a la corrección angular.

---

## Descripción de las Actividades

### Primera Actividad: Posicionamiento Estático y Cálculo de Parámetros

En este problema se solicita al usuario que introduzca las coordenadas y el ángulo deseado para una tortuga. En lugar de mover la tortuga existente, se utiliza el servicio `kill` para eliminarla y `spawn` para crearla exactamente en esa posición. Tras posicionarla, el sistema calcula y muestra en pantalla:

- **La distancia hasta la meta (DTG)** utilizando la fórmula de distancia euclidiana.
- **El ángulo hacia el objetivo (ATG)** en grados, con base en la posición objetivo.

Este ejercicio no involucra movimiento del robot, sino únicamente reposicionamiento por comandos de sistema. Porlo tanto la tortuga simplemente hará spawn en las coordenadas dadas por el usuario. Esto mismo hará que el DTG y ATG sean cero.

### Segunda Actividad: Movimiento Activo hacia un Objetivo

En esta segunda parte, el usuario vuelve a proporcionar coordenadas (x, y) y un ángulo deseado. A diferencia del primer caso, la tortuga ahora sí se desplaza desde su posición actual hacia el destino. El sistema calcula DTG y ATG, y los usa para determinar:

- **Velocidad lineal proporcional** a la distancia al objetivo.
- **Velocidad angular proporcional** a la diferencia de orientación.

El movimiento se realiza en un bucle que ajusta continuamente las velocidades, hasta que la tortuga se encuentra suficientemente cerca del punto destino. Posteriormente, se aplica una rotación proporcional adicional para que el robot coincida con el ángulo objetivo.

---

## Conclusiones

Durante el desarrollo de estas actividades se logró implementar un sistema básico de navegación para un robot simulado, comprendiendo tanto el posicionamiento directo mediante servicios como el control en bucle usando navegación reactiva. Se pudieron observar diferencias claras entre ambos métodos: mientras el reposicionamiento inmediato permite saltos espaciales, el control activo simula condiciones reales donde el robot debe desplazarse suavemente.

Además, se utilizaron librerías específicas de ROS:

- `rospy`: para inicializar nodos, gestionar la suscripción/publicación y servicios.
- `geometry_msgs.msg.Twist`: para generar comandos de movimiento.
- `turtlesim.srv.Spawn` y `Kill`: para la gestión de tortugas.

También se aplicaron funciones matemáticas (`sqrt`, `atan2`, `radians`, `degrees`) que permitieron implementar los cálculos de trayectoria y orientación requeridos para una navegación básica. El uso del control proporcional resultó ser una solución efectiva para alcanzar el objetivo deseado, aunque en aplicaciones reales sería necesario incorporar controladores más robustos ante perturbaciones.

---

## Referencias

- Craig, J. J. (2005). *Introduction to robotics: mechanics and control* (3rd ed.). Pearson Prentice Hall.
- Nise, N. S. (2015). *Control Systems Engineering* (7th ed.). Wiley.
- Quigley, M., Gerkey, B., & Smart, W. D. (2009). *Programming Robots with ROS: A Practical Introduction to the Robot Operating System*. O'Reilly Media.
- Siciliano, B., Sciavicco, L., Villani, L., & Oriolo, G. (2010). *Robotics: Modelling, Planning and Control*. Springer.
