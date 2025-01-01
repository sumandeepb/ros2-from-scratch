#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class NumberPublisher(Node):
    def __init__(self):
        super().__init__('number_publisher')
        self.number_ = 1
        self.number_publisher_ = self.create_publisher(
            msg_type=Int64, topic='number', qos_profile=10)
        self.number_timer_ = self.create_timer(
            timer_period_sec=1.0, callback=self.publish_number)
        self.get_logger().info('Number publisher has been started')

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher_.publish(msg=msg)
        self.get_logger().info(f'Number: {self.number_}')
        # self.number_ += 1


def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
