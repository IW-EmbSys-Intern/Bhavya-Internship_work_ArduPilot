#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):
    def __init__(self):
        super().__init__('draw_circle')
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.get_logger().info("Draw circle node has been started")
        self.counter = 1
        self.create_timer(0.5, self.timer_callback)
        self.create_timer(0.5, self.send_velocity_command)

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 5.0
        msg.angular.z = 1.0
        self.cmd_vel_pub_.publish(msg)

    def timer_callback(self):
        self.get_logger().info("Publisher published message " + str(self.counter))
        self.counter+=1



def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()