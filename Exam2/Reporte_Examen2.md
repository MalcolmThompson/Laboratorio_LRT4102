# Water-Cleaning Robot – Turtlesim Simulation

## Introduction

This project presents the design and simulation of an autonomous floating robot capable of identifying and removing floating waste from aquatic environments. The simulation is developed using the `turtlesim` package in ROS (Robot Operating System) and aims to emulate the behavior of a physical prototype constructed by students.

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

