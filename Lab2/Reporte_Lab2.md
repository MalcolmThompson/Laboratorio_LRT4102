# ROS Lab 2 Report: Basic to Advanced Control with Turtlesim

## *Introduction*

This laboratory session aims to explore the implementation and behavior of robotic systems using the Robot Operating System (ROS). The main goal is to understand ROS communication fundamentals, real-time control, and response evaluation of a robot through simulations. The practice includes developing a ROS package, executing communication nodes, implementing keyboard teleoperation, generating geometric shapes with Turtlesim, and finally, applying and comparing different position control strategies using Proportional (P), Proportional-Derivative (PD), and Proportional-Integral-Derivative (PID) controllers. These tasks provide a foundation for understanding control systems and message-based communication in robotic platforms (Quigley et al., 2009).

## *Theoretical Framework*

### *ROS (Robot Operating System)*

ROS is a flexible framework for writing robot software. It is composed of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. ROS uses nodes (processes) that communicate with each other using topics and messages.

### *Publishers and Subscribers*

In ROS, a publisher is a node that sends messages over a topic, and a subscriber is a node that receives messages from that topic. This communication is crucial in creating modular and scalable robotic systems.

### *Turtlesim*

Turtlesim is a lightweight simulator in ROS used for demonstrating basic concepts like node communication, services, and topics.

### *Control Systems*

A control system manages, commands, directs, or regulates the behavior of other devices. In this lab, three types of controllers are tested:

- P (Proportional) controller: Uses the current error to compute the control signal.

- PD (Proportional-Derivative) controller: Adds a term proportional to the rate of error change.

- PID (Proportional-Integral-Derivative) controller: Adds accumulation of past error to improve long-term precision (Ogata, 2010).

## *Lab2Basic: ROS Communication*

### *Description*

The objective of this part was to create a ROS package named Practicas_lab with dependencies on rospy, roscpp, and std_msgs. Two Python scripts, talker.py and listener.py, were included in the src directory to implement a basic publish/subscribe system. Once compiled, the talker node was executed to publish messages, while the listener node subscribed and printed them.

### *Steps*

Created the package using:

- catkin_create_pkg Practicas_lab rospy roscpp std_msgs

- Added talker.py and listener.py to src/

- Made the Python files executable and compiled the workspace:

    - chmod +x talker.py listener.py
    - catkin_make

- Executed the nodes in separate terminals:

    - rosrun Practicas_lab talker.py
    - rosrun Practicas_lab listener.py

### *Code*

#### *listener.py*
```python
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
```
#### *Description:*
This script defines a ROS node named escucha_oculta (hidden listener) that subscribes to a topic called "canal_secreto" and prints any message it receives using the rospy.loginfo() function.

#### *How it works:*
The function recibir_mensaje is a callback that is triggered every time a message is received on the topic canal_secreto. It prints the message content to the console.

The nodo_escucha function initializes the ROS node and sets up the subscriber.

rospy.spin() keeps the node alive and listening for incoming messages until the program is terminated.

#### *talker.py*
```pyhon
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
```
#### *Description:*
This script defines a ROS node named transmisor_oculto (hidden transmitter) that publishes a message every second to the topic "canal_secreto".

#### *How it works:*
The node initializes a publisher to the topic canal_secreto with message type String.

In a loop running at 1 Hz (once per second), it creates a message with an incrementing counter.

The message is logged using rospy.loginfo() and then published to the topic.

The loop continues until the node is interrupted or shut down.

### *Outcome*

The listener node successfully received and printed messages sent by the talker node, demonstrating effective ROS communication using the publish/subscribe model.

## *Lab2Medium: Teleoperation and Shape Drawing*

### *Description*

This section aimed to develop a keyboard teleoperation script for controlling the Turtlesim robot in real time. The robot was used to draw two shapes: a square and an equilateral triangle, by publishing velocity commands based on keyboard inputs. No controllers were used in this stage.

### *Objectives*

- Implement a keyboard-controlled interface for Turtlesim.

- Write code to draw a square.

- Write code to draw a triangle.

### *Code*

#### *teleop_key*
```python
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
```
#### *Description:*
This script implements real-time keyboard control for the turtle1 robot in Turtlesim. The user can manually move the turtle in the X and Y axes or rotate it using specific keys.

#### *How it works:*
It listens for keyboard inputs without requiring the Enter key, using termios and tty.

A dictionary maps each key (u, o, i, k, a, d) to a specific motion command:

- u → move forward in X

- o → move backward in X

- i → move up in Y

- k → move down in Y

- a → rotate counterclockwise (left)

- d → rotate clockwise (right)

For each valid key press, a Twist message is created and published to /turtle1/cmd_vel, which moves the turtle accordingly.

Pressing x stops the control session.

#### *Purpose:*
This script is useful for manual control and for testing the Turtlesim response to velocity commands published on the /cmd_vel topic.

#### *figures_robotic*
```python
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
```
#### *Description:*
This script automatically creates and deletes a turtle in Turtlesim and commands it to draw either a square or an equilateral triangle by publishing linear and angular velocity commands. It is a simple demonstration of automated motion sequences without using feedback controllers.

#### *How it works:*
The user is shown a menu with 3 options:

- A → draw a square

- B → draw a triangle

- E → exit the simulation

When a figure is selected:

- The turtle named turtle1 is deleted if it exists.

- A new turtle is spawned at a specific location.

- A function is called to publish a series of velocity commands with Twist messages and delays (time.sleep) to simulate precise movement:

    - Square: 4 straight moves and 90° turns.

    - Triangle: 3 straight moves and 120° turns.

- The turtle is deleted again, allowing a new shape to be drawn without interference.

#### *Purpose:*
This script provides an introduction to open-loop control for drawing geometric figures in a simulated environment using only timing and velocity commands — no feedback or sensors are used.

### *Outcome*

The turtle robot responded to keyboard inputs for manual control. The robot was also able to autonomously draw a square and a triangle using linear and angular commands.

## *Lab2Advanced: Controller Implementation*

### *Description*

This part focused on implementing three different controllers (P, PD, and PID) to control the position and orientation of the turtle in Turtlesim. Each controller was implemented in a separate Python script with different gain values, tuned for responsiveness and stability.

P Controller

- Gains used: Kp = 0.75

- Behavior: Basic tracking with some overshoot.

PD Controller

- Gains used: Kp = 0.9, Kd = 0.12

- Behavior: Improved stability, reduced oscillations.

PID Controller

- Gains used: Kp = 0.85, Kd = 0.09, Ki = 0.005

- Behavior: Fast response with reduced steady-state error.

### *Code*

#### *PID controller*
This code implements a position and orientation control system for a turtle in the Turtlesim simulator using a PID (Proportional-Integral-Derivative) control strategy. Only this PID controller code will be described in detail since the P and PD controllers are structurally very similar but use fewer gain terms, as explained at the end.

##### *1. Node Initialization*
```python
rospy.init_node('navegacion_pid_virtual')
```
A ROS node called navegacion_pid_virtual is initialized to manage the PID control system.

##### *2. Publishers and Subscribers*
```python
self.pub_movimiento = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
self.sub_ubicacion = rospy.Subscriber('/turtle1/pose', Pose, self.actualizar_estado)
```
- The node publishes velocity commands to the topic /turtle1/cmd_vel.

- It subscribes to turtle pose updates from /turtle1/pose.

##### *3. Pose Update Function*
```python
def actualizar_estado(self, datos_pose):
```
This callback updates the turtle's current position (x, y) and orientation (theta) in real time.

##### *4. Movement Control (Position)*
```python
def ir_a_destino(self, meta_x, meta_y):
```
This function:

- Calculates the position errors in x and y.

- Applies the PID control formula:
```python
control = Kp * error + Kd * derivative(error) + Ki * integral(error)
```
- Updates the accumulated and previous errors.

- Publishes a velocity command (Twist) based on the computed values.

- Stops when the turtle is close enough to the target (distance < 0.1).

##### *5. Orientation Control*
```python
def ajustar_rotacion(self, objetivo_theta):
```
This function:

- Computes the angular error and keeps it within [-pi, pi].

- Applies the PID control formula for angular velocity.

- Rotates the turtle until the orientation error is less than 0.05 radians.

- Publishes an angular Twist command.

##### *6. User Input*
```python
def pedir_datos_usuario(self):
```
This function prompts the user to input the target x, y coordinates and desired angle in degrees (converted to radians).

##### *7. Main Loop*
```python
def iniciar_rutina(self):
```
This loop continuously requests new destination coordinates and sends the turtle to the given position and orientation.

The P and PD controllers follow almost the same structure:

- The P controller only uses the proportional term: Kp * error.

- The PD controller adds a derivative term: Kd * delta_error.

- The PID controller adds an integral term: Ki * sum_error, which helps reduce steady-state error.

Thus, the only major difference between them is the number of gain terms used and how they influence the control signal. The full PID includes:

- Kp: Reacts to present error.

- Kd: Reacts to error rate of change (helps reduce overshoot).

- Ki: Reacts to accumulated past errors (reduces steady-state error).

Structurally, the P and PD controllers are almost identical to this PID controller, but with fewer lines and simpler calculations.

### *Visualization of Responses*

Each controller’s response was simulated using Python and matplotlib, and visualized using individual plots. Step responses were modeled as second-order systems for comparison.

- Tools used: numpy, matplotlib, scipy.signal

- System simulated: Underdamped second-order system with wn = 2 and zeta = 0.2

Graphical Results:

- Each controller output was plotted in a separate subplot.

- Reference lines were removed for clarity.

- Unique colors were used: purple (P), orange (PD), and cyan (PID).

### *Outcome*

The graphical results confirmed expected behaviors: the P controller showed a steady response with slight overshoot, PD improved the damping, and PID offered optimal performance in terms of settling time and steady-state error.

### *Libraries Used*

- rospy: Interface for ROS in Python.

- geometry_msgs: For velocity commands.

- turtlesim: Simulation environment.

- matplotlib, numpy, scipy: For plotting and signal analysis.

These libraries enabled the implementation, simulation, and evaluation of robot control strategies, providing both real-time and offline insights.

## *Conclusions*

Through this lab, we explored the core components of ROS including node communication, teleoperation, and control implementation. The keyboard control and shape drawing provided a hands-on understanding of topic-based communication and message publishing. Implementing P, PD, and PID controllers demonstrated how different feedback mechanisms affect system behavior. Graphical comparisons validated theoretical expectations, emphasizing the PID controller's ability to minimize error and oscillation.

## *References*

- Ogata, K. (2010). Modern Control Engineering (5th ed.). Prentice Hall.

- Quigley, M., Conley, K., Gerkey, B., Faust, J., Foote, T., Leibs, J., ... & Ng, A. Y. (2009). ROS: an open-source Robot Operating System. ICRA Workshop on Open Source Software, 3(3.2), 5.

- ROS Wiki. (n.d.). http://wiki.ros.org/

- Turtlesim - ROS Wiki. (n.d.). http://wiki.ros.org/turtlesim