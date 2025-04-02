#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, pow, pi, radians, degrees

class TurtleGoal:
    def __init__(self):
        rospy.init_node("guidance_system_node", anonymous=True)

        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=5)
        rospy.Subscriber("/turtle1/pose", Pose, self._update_coords)

        self.position = {"x": 0.0, "y": 0.0, "angle": 0.0}
        self.control_loop = rospy.Rate(10)

    def _update_coords(self, msg):
        self.position["x"] = msg.x
        self.position["y"] = msg.y
        self.position["angle"] = msg.theta

    def _request_target(self):
        print("\nNueva posición destino:")
        px = float(input("X destino: "))
        py = float(input("Y destino: "))
        ang_deg = float(input("Ángulo final (°): "))
        return px, py, radians(ang_deg)

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

    def _halt_motion(self):
        self.pub.publish(Twist())
        rospy.loginfo("Movimiento finalizado.")

    def launch(self):
        while not rospy.is_shutdown():
            coords = self._request_target()
            self._go_to((coords[0], coords[1]))
            self._orient_to(coords[2])

if __name__ == "__main__":
    try:
        nav = TurtleGoal()
        nav.launch()
    except rospy.ROSInterruptException:
        pass