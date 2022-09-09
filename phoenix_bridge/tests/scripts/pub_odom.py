###
### Copyright 2022 Fraunhofer IPA
###
### Licensed under the Apache License, Version 2.0 (the "License");
### you may not use this file except in compliance with the License.
### You may obtain a copy of the License at
###
###   http:###www.apache.org/licenses/LICENSE-2.0
###
### Unless required by applicable law or agreed to in writing, software
### distributed under the License is distributed on an "AS IS" BASIS,
### WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
### See the License for the specific language governing permissions and
### limitations under the License.
###

#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

def talker():
    pub = rospy.Publisher('/sub_odom_1', Odometry, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    i = 0
    while not rospy.is_shutdown():
        msg = Odometry()
        msg.header.seq = i
        msg.header.stamp = rospy.Time().now()
        msg.header.frame_id = "header_frame_id"
        msg.child_frame_id = "child_frame_id"
        msg.pose.pose.position.x = 10*i + 1
        msg.pose.pose.position.y = 10*i + 2
        msg.pose.pose.position.z = 10*i + 3
        msg.pose.pose.orientation.x = 10*i + 4
        msg.pose.pose.orientation.y = 10*i + 5
        msg.pose.pose.orientation.z = 10*i + 6
        msg.pose.pose.orientation.w = 10*i + 7
        msg.twist.twist.linear.x = 20*i + 1
        msg.twist.twist.linear.y = 20*i + 2
        msg.twist.twist.linear.z = 20*i + 3
        msg.twist.twist.angular.x = 20*i + 4
        msg.twist.twist.angular.y = 20*i + 5
        msg.twist.twist.angular.z = 20*i + 6
        msg.pose.covariance.clear()
        msg.twist.covariance.clear()
        for x in range(36):
            msg.pose.covariance.append(x)
            msg.twist.covariance.append(x)

        rospy.loginfo(msg)
        pub.publish(msg)
        i += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
