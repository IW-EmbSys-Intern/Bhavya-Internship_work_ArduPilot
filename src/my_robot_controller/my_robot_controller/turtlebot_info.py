#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# from turtlesim.msg import Pose
from sensor_msgs.msg import JointState

class JointStateSubscriberNode(Node):

    def __init__(self):
        super().__init__("apple_pie_info")
        self.joint_state_subscriber = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)

    def joint_state_callback(self, msg: JointState):
        self.get_logger().info(str(msg))

def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriberNode()
    rclpy.spin(node)