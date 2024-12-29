#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyCustomNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.get_logger().info('Hello World')
        self.counter_ = 0
        self.timer_ = self.create_timer(1.0, self.print_hello)

    def print_hello(self):
        self.get_logger().info(f'Hello {str(self.counter_)}')
        self.counter_ += 1


def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode()
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
