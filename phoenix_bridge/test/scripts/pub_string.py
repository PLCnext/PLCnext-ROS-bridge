#!/usr/bin/env python3

import rclpy

from random import randint
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('twist_publisher')
        self.publisher_ = self.create_publisher(String, '/sub_string_1', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = "hi" + str(randint(1,100))

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
