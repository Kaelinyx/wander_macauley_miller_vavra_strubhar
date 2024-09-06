#!/usr/bin/env python

"""Example of a ROS2 node using Python's OO features.

The node is represented as a class.  Sensor messages are stored in
instance variables.

Author: NNicholas miller, Alex Macauley, Cole Strubhar, Sergio Vavra
Version: 8/31/2023

"""
import rclpy
import rclpy.node

from geometry_msgs.msg import Twist


class ThrusterNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('Twister')

        self.thrust_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_timer(.1, self.timer_callback)

    def timer_callback(self):
        thrust = Twist()

        thrust.linear.x = 0.2
        self.thrust_pub.publish(thrust)


def main(args=None):
    rclpy.init(args=args)
    thruster_node = ThrusterNode()
    rclpy.spin(thruster_node)

    thruster_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
