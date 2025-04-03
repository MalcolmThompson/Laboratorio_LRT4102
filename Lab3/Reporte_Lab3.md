# Lab Report 3

## Introduction

This lab involves two fundamental activities using the Turtlesim simulator within the ROS (Robot Operating System) environment. The first task consists of calculating navigation metrics such as the Distance to Goal (DTG) and the Angle to Goal (ATG), then spawning a turtle directly at that position without requiring any movement. The second task involves actively moving a turtle toward a goal position, computing trajectories from Euclidean coordinates and using a proportional controller, repeating this process in a loop.

These activities aim to introduce the concepts of proportional control, velocity mapping, and the use of ROS services such as `spawn` and `kill`, which allow dynamic management of entities within the simulator. Additionally, the lab reinforces event-driven programming and reactive loop design by working with ROS topic subscription and publication.

---

## Key Concepts for Implementation

To carry out these practices, it is essential to understand and apply certain fundamental concepts of mobile robot navigation and control:

### Euclidean Distance (DTG)

**Distance to Goal** is a direct measurement between two points in the plane, calculated using the Pythagorean theorem. This metric allows estimation of how far the robot is from its destination (Siciliano et al., 2010):

$$
\text{DTG} = \sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}
$$

### Angle to Goal (ATG)

**Angle to Goal** represents the direction the robot must face to move toward the target position. It is obtained using the `atan2` function, which calculates the arctangent while considering the sign of both arguments. This result is given in radians and later converted to degrees for easier interpretation (Craig, 2005):

$$
\text{ATG} = \tan^{-1}\left(\frac{y_2 - y_1}{x_2 - x_1}\right)
$$

### Velocity Mapping

Once the distance and angle to the goal are known, a control system can be established to relate these errors to velocities. **Velocity mapping** consists of translating spatial error into motion commands, typically using proportional equations such as:

$$
v = K_p \cdot \text{DTG} \quad ; \quad \omega = K_\theta \cdot (\text{ATG} - \theta_{\text{actual}})
$$

where `v` is the linear velocity and `ω` is the angular velocity (Quigley et al., 2009).

### Proportional Control

**Proportional control** (P) is a control strategy where the output (in this case, the velocities) is directly proportional to the error. This technique is commonly used due to its simplicity and fast response in navigation tasks (Nise, 2015). In this lab, it is applied both to movement toward the goal and to angular correction.

---

## Activity Breakdown

### Activity One: Static Positioning and Parameter Calculation

In this task, the user is prompted to input the desired coordinates and angle for a turtle. Instead of moving the existing turtle, the `kill` service is used to remove it and the `spawn` service is used to create a new one exactly at the specified position. Once positioned, the system calculates and displays:

- **Distance to Goal (DTG)** using the Euclidean distance formula.
- **Angle to Goal (ATG)** in degrees, based on the target position.

This exercise does not involve any robot movement; instead, the turtle is repositioned through system commands. As a result, the turtle simply spawns at the coordinates provided by the user, and both DTG and ATG become zero.

#### Code Explained

##### *1. Shebang and Imports*
```python
#!/usr/bin/env python3

import rospy
from turtlesim.srv import Kill, Spawn
from math import sqrt, atan2, degrees, radians
```
- #!/usr/bin/env python3: Informs the system to execute the script using Python 3.

- rospy: Python library for interacting with ROS (Robot Operating System).

- turtlesim.srv: Imports the Kill and Spawn services used to delete or create turtles.

- math: Provides mathematical functions for distance and angle calculations.

##### *Function to Delete a Turtle*
```python
def eliminar_tortuga(nombre_tortuga):
    rospy.wait_for_service("/kill")
    try:
        solicitar_kill = rospy.ServiceProxy("/kill", Kill)
        solicitar_kill(nombre_tortuga)
    except rospy.ServiceException:
        rospy.logwarn(f"[Aviso] No se logró eliminar '{nombre_tortuga}', ya podría no existir.")
```
- Waits for the /kill service to be available.

- Creates a service proxy (solicitar_kill) to call the Kill service.

- Attempts to remove the turtle with the given name.

- Logs a warning if the deletion fails (e.g., the turtle doesn’t exist anymore).

##### *3. Function to Spawn a New Turtle*
```python
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
```
- Waits for the /spawn service to be ready.

- Converts the angle from degrees to radians (required by the Spawn service).

- Creates a new turtle at the specified position and orientation.

- Returns the turtle’s position if successful; logs an error otherwise.

##### *4. Function to Capture User Input*
```python
def capturar_entrada_usuario():
    print("Parámetros para la nueva tortuga")
    nuevo_x = float(input("Coordenada X: "))
    nuevo_y = float(input("Coordenada Y: "))
    nuevo_angulo = float(input("Ángulo inicial (en grados): "))
    return nuevo_x, nuevo_y, nuevo_angulo
```
- Asks the user to enter the desired X and Y coordinates, and orientation angle.

- Returns the user input as a tuple of float values.

##### *5. Function to Calculate DTG and ATG*
```python
def calcular_datos_orientacion(x_obj, y_obj, x_ini, y_ini):
    distancia = sqrt((x_obj - x_ini)**2 + (y_obj - y_ini)**2)
    angulo_rad = atan2((y_obj - y_ini), (x_obj - x_ini))
    return distancia, degrees(angulo_rad)
```
- Calculates the Euclidean distance (DTG) between two points.

- Calculates the Angle to Goal (ATG) using atan2, which takes into account the sign of both components.

- Converts the angle from radians to degrees.

- Returns both DTG and ATG.

##### *6. Main Execution Function*
```python
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
```
- Initializes a ROS node called "generador_de_tortuga".

- Calls the function to get user input (goal coordinates and angle).

- Deletes the existing turtle using /kill.

- Spawns a new turtle at the specified goal position using /spawn.

- If the turtle was successfully created, calculates DTG and ATG (which will be 0 since it spawns at the goal).

- Displays both values in the terminal.

##### *7. Entry Point*
```python
if __name__ == "__main__":
    try:
        iniciar_proceso()
    except rospy.ROSInterruptException:
        pass
```
- Ensures the iniciar_proceso() function runs if the script is executed directly.

- Catches ROSInterruptException (e.g., if the user presses Ctrl+C) to safely terminate the node.

## Generating Motion Commands from Positional Feedback

In robotic navigation, converting positional feedback into movement commands is essential for responsive and goal oriented behavior. This process, often referred to as velocity mapping, takes the positional error how far and in what direction the robot is from the target and transforms it into real-time velocity values. The greater the distance, the higher the linear velocity assigned, encouraging the robot to move quickly toward the goal. At the same time, the difference in orientation determines the angular velocity, ensuring that the robot faces the correct direction during movement. Rather than relying on fixed speed commands, this approach dynamically adjusts the robot’s motion based on its current position, resulting in smoother trajectories and more efficient convergence. This adaptive behavior is critical for continuous navigation systems and is easily implemented through proportional control mechanisms.

### Activity Two: Active Movement Toward a Goal

In this second part, the user is again asked to provide coordinates (x, y) and a desired angle. Unlike the first case, the turtle now moves from its current position toward the target. The system calculates DTG and ATG and uses them to determine:

- **Linear velocity proportional** to the distance to the goal.
- **Angular velocity proportional** to the orientation difference.

The movement occurs in a loop that continuously adjusts the velocities until the turtle is sufficiently close to the goal. Then, an additional proportional rotation is applied to align the robot with the target angle.

#### Code Explained

##### *1. Shebang and Imports*
```python
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, pow, pi, radians, degrees
```
- #!/usr/bin/env python3: Tells the system to use Python 3 to run the script.

- rospy: Used to interact with ROS.

- geometry_msgs.msg.Twist: Message type used to send velocity commands.

- turtlesim.msg.Pose: Message type used to receive the turtle’s current position and orientation.

- math functions: Used for calculating angles and distances, and converting between radians and degrees.

##### *2. TurtleGoal Class Initialization*
```python
class TurtleGoal:
    def __init__(self):
        rospy.init_node("guidance_system_node", anonymous=True)

        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=5)
        rospy.Subscriber("/turtle1/pose", Pose, self._update_coords)

        self.position = {"x": 0.0, "y": 0.0, "angle": 0.0}
        self.control_loop = rospy.Rate(10)
```
- Initializes a ROS node named "guidance_system_node".

- Creates a publisher for /turtle1/cmd_vel to send velocity commands.

- Subscribes to /turtle1/pose to track the turtle’s current state.

- Initializes an internal dictionary to store position (x, y) and heading angle.

- Sets the control loop rate to 10 Hz (10 updates per second).

##### *3. Update the Turtle’s Position*
```python
def _update_coords(self, msg):
    self.position["x"] = msg.x
    self.position["y"] = msg.y
    self.position["angle"] = msg.theta
```
- Callback function that updates the turtle’s current coordinates and orientation whenever a new pose message is received.

##### *4. Get User Input for the Goal*
```python
def _request_target(self):
    print("\nNueva posición destino:")
    px = float(input("X destino: "))
    py = float(input("Y destino: "))
    ang_deg = float(input("Ángulo final (°): "))
    return px, py, radians(ang_deg)
```
- Asks the user to input the X and Y coordinates and the final orientation (in degrees).

- Converts the angle to radians and returns all three values.

##### *5. Move Toward the Target Position*
```python
def _go_to(self, target):
    twist = Twist()
    k_lin = 1.2
    k_ang = 5.5

    while not rospy.is_shutdown():
        dx = target[0] - self.position["x"]
        dy = target[1] - self.position["y"]
        distance = sqrt(dx**2 + dy**2)

        desired_angle = atan2(dy, dx)
        angle_error = desired_angle - self.position["angle"]
        angle_error = (angle_error + pi) % (2 * pi) - pi

        twist.linear.x = k_lin * distance
        twist.angular.z = k_ang * angle_error

        self.pub.publish(twist)

        rospy.loginfo("Distancia restante: %.2f | Corrigiendo orientación: %.1f°",
                      distance, degrees(angle_error))

        if distance <= 0.15:
            break

        self.control_loop.sleep()

    self._halt_motion()
```
- This method performs proportional control to reach the target coordinates.

- Calculates:

    - The distance to the goal using Euclidean geometry.

    - The angle to the goal using atan2.

    - The angular error, normalized to the range [-π, π].

- The linear.x and angular.z velocities are calculated proportionally to their respective errors.

- The loop continues until the distance is less than or equal to 0.15 units.

- Once the goal is reached, it stops the turtle’s movement.

##### *6. Orient the Turtle to Final Angle*
```python
def _orient_to(self, final_angle):
    twist = Twist()
    angular_gain = 3.8

    while not rospy.is_shutdown():
        angle_gap = final_angle - self.position["angle"]
        angle_gap = (angle_gap + pi) % (2 * pi) - pi

        twist.angular.z = angular_gain * angle_gap
        self.pub.publish(twist)

        rospy.loginfo("Ajustando ángulo... [Error: %.2f°]", degrees(angle_gap))

        if abs(angle_gap) < 0.04:
            break

        self.control_loop.sleep()

    self._halt_motion()
```
- Rotates the turtle to the desired final orientation using angular proportional control.

- Calculates the angle difference between current and target angles.

- Publishes only angular velocity (angular.z) based on this error.

- Stops when the angle error is very small (below 0.04 radians ≈ 2.3 degrees).

##### *7. Stop Movement*
```python
def _halt_motion(self):
    self.pub.publish(Twist())
    rospy.loginfo("Movimiento finalizado.")
```
- Publishes a Twist() message with all velocities set to zero.

- Logs that the movement is complete.

##### *8. Main Execution Loop*
```python
def launch(self):
    while not rospy.is_shutdown():
        coords = self._request_target()
        self._go_to((coords[0], coords[1]))
        self._orient_to(coords[2])
```
- Repeatedly:

    - Requests a new target from the user.

    - Moves the turtle toward the position.

    - Rotates the turtle to match the final angle.

##### *9. Script Entry Point*
```python
if __name__ == "__main__":
    try:
        nav = TurtleGoal()
        nav.launch()
    except rospy.ROSInterruptException:
        pass
```
- Ensures that the class runs when the script is executed.

- Gracefully handles ROS interruption (e.g., from Ctrl+C).

---

## Conclusions

Throughout these activities, a basic navigation system for a simulated robot was successfully implemented, encompassing both direct positioning using services and reactive loop control. Clear differences were observed between the two methods: while direct repositioning enables instant spatial jumps, active control simulates real-world conditions where the robot must move smoothly.

In addition, specific ROS libraries were used:

- `rospy`: to initialize nodes, manage subscription/publication, and handle services.
- `geometry_msgs.msg.Twist`: to generate movement commands.
- `turtlesim.srv.Spawn` and `Kill`: to manage turtle instances.

Mathematical functions (`sqrt`, `atan2`, `radians`, `degrees`) were also used to implement the trajectory and orientation calculations needed for basic navigation. The proportional control method proved to be an effective solution for reaching the desired goal, though in real-world applications more robust controllers would be required to handle disturbances.

---

## References

- Craig, J. J. (2005). *Introduction to robotics: mechanics and control* (3rd ed.). Pearson Prentice Hall.  
- Nise, N. S. (2015). *Control Systems Engineering* (7th ed.). Wiley.  
- Quigley, M., Gerkey, B., & Smart, W. D. (2009). *Programming Robots with ROS: A Practical Introduction to the Robot Operating System*. O'Reilly Media.  
- Siciliano, B., Sciavicco, L., Villani, L., & Oriolo, G. (2010). *Robotics: Modelling, Planning and Control*. Springer.
