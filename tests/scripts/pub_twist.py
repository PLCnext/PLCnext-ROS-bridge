#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('/sub_twist_1', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    i = 0
    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = 20*i + 1
        msg.linear.y = 20*i + 2
        msg.linear.z = 20*i + 3
        msg.angular.x = 20*i + 4
        msg.angular.y = 20*i + 5
        msg.angular.z = 20*i + 6

        rospy.loginfo(msg)
        pub.publish(msg)
        i += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass