#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import HardwareStatus


class CustomPublisher(Node):
    def __init__(self):
        super().__init__('custom_publisher')
        self.custom_publisher_ = self.create_publisher(
            msg_type=HardwareStatus, topic='hardware_status', qos_profile=10)
        self.timer_ = self.create_timer(
            timer_period_sec=1.0, callback=self.publish_message)
        self.get_logger().info('Custom publisher has been started')

    def publish_message(self):
        msg = HardwareStatus()
        msg.temperature = 34.5
        self.custom_publisher_.publish(msg=msg)
        # self.get_logger().info(f'Number: {self.number_}')


def main(args=None):
    rclpy.init(args=args)
    node = CustomPublisher()
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
