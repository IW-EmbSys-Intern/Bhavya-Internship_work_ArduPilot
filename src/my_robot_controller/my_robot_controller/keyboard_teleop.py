import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        pygame.init()
        self.screen = pygame.display.set_mode((100, 100))

        self.timer = self.create_timer(0.05, self.update)

    def update(self):
        linear = 0.0
        angular = 0.0

        for event in pygame.event.get():
            pass

        keys = pygame.key.get_pressed()

        # Arrow keys control
        if keys[pygame.K_UP]:
            linear = 0.2
        elif keys[pygame.K_DOWN]:
            linear = -0.2

        if keys[pygame.K_LEFT]:
            angular = 0.5
        elif keys[pygame.K_RIGHT]:
            angular = -0.5

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = KeyboardTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
