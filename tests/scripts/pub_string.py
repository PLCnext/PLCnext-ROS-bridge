#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('/sub_string_1', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    i = 0
    while not rospy.is_shutdown():
        msg = String()
        msg.data = "Hi " + str(i)

        rospy.loginfo(msg)
        pub.publish(msg)
        i += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass