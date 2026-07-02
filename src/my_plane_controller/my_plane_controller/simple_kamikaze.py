#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class FixedWingStraightFlyer(Node):
    def __init__(self):
        super().__init__('fixed_wing_straight_flyer')
        
        # Publisher for target velocity mapping to ArduPilot's DDS interface
        self.velocity_pub = self.create_publisher(
            TwistStamped, 
            '/ap/cmd_vel', 
            10
        )
        
        # Timer to continuously send velocity commands at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_forward_velocity)
        
        self.get_logger().info('Fixed Wing Straight Flyer Node Started')
        self.speed = 15.0  # Target forward speed in m/s

    def publish_forward_velocity(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'  # Controls relative to the aircraft's current orientation
        
        # Set forward linear velocity (X-axis in ROS 2 REP-103 standard)
        msg.twist.linear.x = self.speed
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.2  # Gentle climb rate command
        
        # Keep angular velocities at zero to enforce straight-line flight
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        
        self.velocity_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FixedWingStraightFlyer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()