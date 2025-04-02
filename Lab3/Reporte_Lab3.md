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

### Activity Two: Active Movement Toward a Goal

In this second part, the user is again asked to provide coordinates (x, y) and a desired angle. Unlike the first case, the turtle now moves from its current position toward the target. The system calculates DTG and ATG and uses them to determine:

- **Linear velocity proportional** to the distance to the goal.
- **Angular velocity proportional** to the orientation difference.

The movement occurs in a loop that continuously adjusts the velocities until the turtle is sufficiently close to the goal. Then, an additional proportional rotation is applied to align the robot with the target angle.

#### Code Explained

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
