#!/usr/bin/env python3

import rclpy

from random import randint
from rclpy.node import Node
from nav_msgs.msg import Odometry


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('odom_publisher')
        self.publisher_ = self.create_publisher(Odometry, '/sub_odom_1', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "test_publisher"
        msg.child_frame_id = "plc"
        msg.pose.pose.position.x = randint(1, 1000)/100
        msg.pose.pose.position.y = randint(1, 1000)/100
        msg.pose.pose.position.z = randint(1, 1000)/100
        msg.pose.pose.orientation.x = randint(1, 100)/100
        msg.pose.pose.orientation.y = randint(1, 100)/100
        msg.pose.pose.orientation.z = randint(1, 100)/100
        msg.pose.pose.orientation.w = randint(1, 100)/100
        msg.pose.covariance = [x/10 for x in range(0,36)]
        msg.twist.twist.linear.x = randint(1, 500)/100
        msg.twist.twist.linear.y = randint(1, 500)/100
        msg.twist.twist.linear.z = randint(1, 500)/100
        msg.twist.twist.angular.x = randint(1, 314)/100
        msg.twist.twist.angular.y = randint(1, 314)/100
        msg.twist.twist.angular.z = randint(1, 314)/100
        msg.twist.covariance = [x/10 for x in range(0,36)]

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
