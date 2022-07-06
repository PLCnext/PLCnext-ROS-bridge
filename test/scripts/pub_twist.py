#!/usr/bin/env python3

import rclpy

from random import randint
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('twist_publisher')
        self.publisher_ = self.create_publisher(Twist, '/sub_twist_1', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = randint(1, 500)/100
        msg.linear.y = randint(1, 500)/100
        msg.linear.z = randint(1, 500)/100
        msg.angular.x = randint(1, 314)/100
        msg.angular.y = randint(1, 314)/100
        msg.angular.z = randint(1, 314)/100

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
