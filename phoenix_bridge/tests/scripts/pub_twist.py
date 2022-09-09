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