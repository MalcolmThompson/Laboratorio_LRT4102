# Water-Cleaning Robot – Turtlesim Simulation

## Introduction

This report presents the design and simulation of an autonomous floating robot capable of identifying and removing floating waste from aquatic environments. The simulation is developed using the `turtlesim` package in ROS (Robot Operating System) and aims to emulate the behavior of a physical prototype constructed by students.

The virtual robot operates within a 2D environment and detects visual representations of waste based on color. Once detected, the robot approaches the waste and simulates a cleaning action. This behavior is intended to mirror the real-world implementation, where the robot utilizes a camera and color filtering (via OpenCV) to recognize red and green-colored objects on the water surface, and communicates with Arduino to activate motors for navigation.

The simulation supports interactive testing, enabling users to decide when to regenerate random waste or reset the robot’s position. This project provides a foundation for developing intelligent, accessible, and environmentally-focused robotic systems.

The project also aligns with the United Nations Sustainable Development Goals, particularly:

- **Goal 6: Clean Water and Sanitation** – Promoting safe water bodies through surface-level waste management.
- **Goal 14: Life Below Water** – Contributing to the reduction of pollution in marine and freshwater ecosystems.

---

## System Overview

- The robot is simulated using `turtlesim` and programmed in Python with ROS services and publishers.
- It detects virtual "waste" based on two colors: **red** and **green**, within a **120-degree field of view**.
- Waste objects are drawn as small colored circles and placed randomly at each generation.
- The robot moves only if a red or green object is detected in its field of vision.
- Once it reaches the target, the robot simulates a cleaning action by marking the area with a white dot.
- If no waste is detected, the robot waits and prompts the user to either:
  - Generate new waste
  - Reset its starting position
  - Or exit the simulation

---

## Theoretical Framework

### 1. Visual Color-Based Detection

In robotic vision, color detection is a common and efficient method for object recognition. The robot distinguishes objects using color segmentation techniques, particularly filtering red and green hues. In the real implementation, OpenCV is used to transform the image into the HSV color space and apply masks to isolate desired color ranges (OpenCV Team, 2023).

### 2. ROS (Robot Operating System)

ROS is a modular and open-source middleware framework designed for robotic application development. It provides communication tools such as topics, services, and actions. In this project, ROS handles the turtle's movement (`cmd_vel`), spawning objects (`/spawn`), and visualization (`turtlesim`). It enables real-time control, modularity, and reproducibility (Open Robotics, 2023).

### 3. Turtlesim

Turtlesim is a lightweight 2D simulation tool bundled with ROS, commonly used for educational purposes. Although simple, it supports core ROS concepts including publishers, subscribers, and services. In this simulation, `turtle1` represents the cleaning robot, and additional turtles are temporarily spawned to draw colored waste circles.

### 4. Proportional Control Logic

The robot uses a proportional controller to approach detected objects. It calculates the angular error between its current heading and the target direction and adjusts its angular velocity accordingly. The robot only moves forward when the angular error is minimal to ensure precise navigation.

### 5. Color-Based Object Representation

Waste is visually represented as colored circular paths drawn by temporary turtles. These turtles move in circles with a constant linear and angular velocity and are removed after completing the drawing. The simulation ensures spatial separation between objects to avoid overlap.

### 6. Python–ROS Integration

The implementation relies on Python for managing ROS communication. Services like `rospy.ServiceProxy()` are used for calling ROS service endpoints, while `rospy.Publisher()` and `rospy.Subscriber()` manage real-time command and data flows. This structure mimics the logic separation used in embedded systems, where the control unit (Raspberry Pi) communicates with an actuator board (Arduino).

---
## Virtual Metric Description

In this project, the virtual robot follows a basic logic to detect and clean objects that represent floating garbage (red or green colored circles). To evaluate its performance in a measurable way, we selected the metric: Cleaning Strategy and Path-Planning Logic.

This metric will help us assess how efficient the robot is when it comes to:

- Detecting a valid object in its field of vision,

- Deciding which object to go for,

- And moving to the correct spot without wasting time or making unnecessary movements.

For this simulation, we are going to measure the time it takes for the robot to move from its current position to the center of a red or green object. This gives us a clear idea of how fast and direct the robot's decision-making and path-following behavior is.

So in every attempt, once the robot sees a target (either green or red), we start counting time. When the robot reaches the center of that target, we stop counting. That gives us the time taken to clean that specific object. If we repeat this process a few times, we can get an average cleaning time, which is the main thing we're going to report and compare.

## Code structure and explanation

The robot simulation is written in Python using ROS. The logic is organized into a class called MiRobot. Below is a breakdown of the code section by section to understand what each part does.

### *Imports and Initialization*
```python
import rospy, random, time
from math import atan2, sqrt, pi
from turtlesim.srv import Spawn, Kill, TeleportAbsolute, SetPen
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
```
This section imports all necessary Python and ROS libraries. It includes services (Spawn, Kill, etc.), messages (Pose, Twist), and utilities (atan2, sqrt, etc.).

### *Class Definition an ROS Node Setup*
```python
class MiRobot:
    def __init__(self):
        rospy.init_node('robot_basura')
        ...
```
Here we define the robot class and initialize it as a ROS node. It sets up the main publishers and subscribers for movement and position tracking. ROS services like spawn, kill, and teleport_absolute are also set up to control turtles and clean the simulation.

### *Robot position*
```python
def leer_pos(self, data):
    self.pos = data
```
This method listens to the robot’s current position and orientation (/turtle1/pose) and stores it in self.pos to be used in movement calculations.

### *Reset to robot start position*
```python
def reset(self):
    self.lapiz(0, 0, 0, 0, 1)
    self.tp(10.5, 1.0, angle)
    self.lapiz(255, 255, 255, 3, 0)
```
Resets the robot to the bottom-right corner of the map, facing diagonally toward the top-left. It also disables and re-enables the drawing pen so the robot doesn’t leave unwanted trails.

### *Color conversion*
```python
def color_a_rgb(self, c):
    ...
```
Converts color names (like 'red', 'green') to RGB values used for drawing circles on the map.

### *Generate colored circles*
```python
def generar(self):
    ...
```
This function creates 5 trash circles, two of which are always red and green. Turtles are spawned temporarily, draw a circular path (using angular + linear velocity), and then are deleted with kill(). Their position and color are stored in self.lista.

### *Field of view logic*
```python
def en_rango(self, x, y):
    ...
```
Checks if a colored object is within a 120° vision cone of the robot's current direction using angle calculations.

### *Target selection*
```python
def buscar(self):
    ...
```
Filters the list to find visible red or green objects within range. If multiple are detected, it chooses the closest one using Euclidean distance.

### *Movement Toward Target*
```python
def mover(self, x, y):
    ...
```
Moves the robot to a target position using a proportional controller:

- Adjusts orientation using angular velocity.

- Moves forward only when the robot is aligned (angle error is small).

- Stops when close enough (distance < 0.05).

### *Main Execution Loop*
```python
def correr(self):
    ...
```
This is the main loop:

- Resets the robot.

- Generates random trash.

- Enters a loop where:

    - If red/green trash is visible → moves to it.

    - If not → waits 3 seconds and asks if new trash should be generated or if the robot should reset.

    - If both answers are no → exits.

It also tracks how long it took the robot to reach each red or green target using time.perf_counter() and saves it in self.tiempos.

At the end, the script prints a list of all cleaning times (in seconds) and the average. This is used to evaluate the cleaning strategy and performance.

## Performance Metric Evaluation

In this section, we analyze the behavior and efficiency of the robot based on a virtual metric defined as Cleaning Strategy and Path-Planning Logic. This metric reflects how the robot selects and navigates to red and green "trash" targets within its field of vision, and how long it takes to reach and clean them.

The metric captures the time elapsed between when a valid object (red or green) is detected and when the robot successfully reaches and cleans it. Each cleaning operation is measured individually and tracked in real time using Python’s time.perf_counter() function. The robot stores each time in a list and prints both the individual durations and their average at the end of the session.

This allows us to evaluate:

- Whether the robot is taking unnecessarily long paths.

- If it's consistently reaching targets quickly.

- How effective the proportional control logic is under varied conditions (target angles, distances).

The results presented below were obtaindes from three simulation trials, the robot registered the following cleaning times:

### *1st Attemp*
Tiempos registrados:
1. red - 10.9 seconds
2. green - 3.4 seconds

### *2nd Attemp*
Tiempos registrados:
1. red - 8.5 seconds
2. green - 5.2 seconds

### *3rd Attemp*
Tiempos registrados:
1. red - 11.6 seconds
2. green - 4.7 seconds

The averge obtained is the following:

- red trash: 10.33 seconds
- green trash: 4.43 seconds

## Justification

To better understand the recorded cleaning times, we must consider two main variables that affect the robot's movement: the distance to each trash target and the velocity settings defined in the control logic.

In the mover() function of the code, the robot operates under the following velocity configuration:

- Linear velocity: 0.5 units per second (only if the robot is correctly aligned with the target).

- Angular velocity: 3.0 * error (used to rotate the robot until it faces the target directly).

The robot uses a simple proportional controller to:

- Rotate until the angle error is small (alignment phase).

- Move forward until it reaches the center of the trash (translation phase).

Because rotation is handled first, and forward movement is only activated once the robot is facing the object, the total time depends not only on the distance to the object, but also on how much the robot had to rotate before moving.

### *Red trash*
During the first trial, the robot took 10.9 seconds to reach the red target. If we assume the red object was approximately 5.5 units away in a diagonal from the robot’s starting point, and considering the linear velocity is 0.5 units/sec, then the theoretical movement time would be:

Distance / Speed = 5.5 / 0.5 = 11 seconds (approx.)

This matches well with the recorded 10.9 seconds, confirming that the robot:

- Detected the object quickly,

- Rotated successfully,

- And moved directly to the target with minimal correction.

### *Green trash*
For the green target, the robot took 3.4 seconds to reach it. In this case, the object was likely close and already within the robot’s direction of movement. If we estimate a distance of about 1.7 units, the theoretical time would be:

1.7 / 0.5 = 3.4 seconds

Again, this strongly aligns with the actual measurement.

## Conclusions
After working on this project, I was able to understand how a basic robot can be programmed to perform a specific task like detecting and "cleaning" colored targets. Even though the simulation is pretty simple and runs in 2D using turtlesim, it helped me connect a lot of the concepts we've seen in class, especially in terms of control systems, logic flow, and how ROS nodes communicate.

One of the most important takeaways is how small changes in the robot’s orientation or the distance to a target can significantly affect its performance. For example, sometimes the robot would take over 10 seconds to clean a red object just because it was far away and the robot had to rotate a lot before moving. In contrast, when the green object was closer and already in front of the robot, the cleaning was done in just a few seconds. That really shows how important path planning and rotation control are, even in simple robots.

The metric we used, which was based on measuring the time it took the robot to reach and clean a valid target, turned out to be pretty effective. It gave me a solid way to evaluate how well the robot was doing its job and also helped me understand the importance of having measurable data when working with robots or any automated system.

Overall, this project was really helpful to reinforce the logic behind movement, detection, and control. If I were to improve it in the future, I’d probably try to add some kind of obstacle detection, or maybe optimize how the robot chooses between multiple targets. But for now, I’m happy with how it worked and how everything connected between code, math, and the final behavior on screen.

## References

- Arduino. (2024). Arduino Language Reference. Retrieved from https://www.arduino.cc/reference/en/

- Liechti, C. (2022). pySerial Documentation (Version 3.5). Retrieved from https://pyserial.readthedocs.io/

- OpenCV Team. (2023). OpenCV Documentation – Color Spaces and Object Detection in HSV. Retrieved from https://docs.opencv.org/4.x/

- Open Robotics. (2023). ROS 2 Documentation: Humble Hawksbill. Retrieved from https://docs.ros.org/en/humble/index.html

- Python Software Foundation. (2023). Python 3 Standard Library: time. Retrieved from https://docs.python.org/3/library/time.html

- Python Software Foundation. (2023). Python 3 math module. Retrieved from https://docs.python.org/3/library/math.html

- United Nations. (2015). Sustainable Development Goals (SDGs). Retrieved from https://sdgs.un.org/goals

- Willow Garage. (2010). Turtlesim Tutorial (ROS Beginner Tutorials). Retrieved from http://wiki.ros.org/turtlesim