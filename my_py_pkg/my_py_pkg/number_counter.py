#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class NumberCounterNode(Node):
    def __init__(self):
        super().__init__('number_counter')
        self.counter_ = 0
        self.number_subscriber_ = self.create_subscription(
            msg_type=Int64, topic='number', callback=self.callback_number, qos_profile=10)
        self.get_logger().info('Number counter node has been started.')

    def callback_number(self, msg: Int64):
        self.counter_ += msg.data
        self.get_logger().info(f'Counter: {self.counter_}')


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
