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

(Add your implementation of talker.py and listener.py here)

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

(Add your teleoperation and shape-drawing scripts here)

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

(Add your P, PD, and PID controller implementations here)

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