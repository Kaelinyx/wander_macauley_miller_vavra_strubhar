#!/usr/bin/env python

"""Example of a ROS2 node using Python's OO features.

The node is represented as a class.  Sensor messages are stored in
instance variables.

Author: Nicholas miller, Alex Macauley, Cole Strubhar, Sergio Vavra
Version: 8/31/2023

"""
import rclpy
import rclpy.node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


class ThrusterNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('Twister')

        self.thrust_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_timer(.1, self.timer_callback)

        self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)

        self.ranges = None

    def scan_callback(self, scan_msg):
        """scan_msg will be of type LaserScan."""
        self.ranges = scan_msg

    def timer_callback(self):
        useful_ranges = []

        thrust = Twist()

        obstacle_distance_threshold = 1.0

        thrust.linear.x = 0.2

        if self.ranges is not None:

            i = -15

            while i <= 15:
                if (self.ranges.ranges[i] != 0):
                    useful_ranges.append(self.ranges.ranges[i])
                i += 1

            print(useful_ranges)

            min_useful_ranges = 100
            if len(useful_ranges) != 0:
                min_useful_ranges = min(useful_ranges)

            if min_useful_ranges < obstacle_distance_threshold:
                thrust.linear.x = 0.0
                thrust.angular.z = 0.3
            else:
                thrust.linear.x = 0.2
                thrust.linear.z = 0.0
                print("Should be moving!")

        self.thrust_pub.publish(thrust)


def main(args=None):
    rclpy.init(args=args)
    thruster_node = ThrusterNode()
    rclpy.spin(thruster_node)

    thruster_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
