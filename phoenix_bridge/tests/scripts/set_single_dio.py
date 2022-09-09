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

import sys
import rospy
from phoenix_bridge.srv import SingleSetIO, SingleSetIORequest, SingleSetIOResponse

def service_call(val : bool):
    rospy.wait_for_service('single_set_io')
    try:
        set_client = rospy.ServiceProxy('single_set_io', SingleSetIO)
        req = SingleSetIORequest()
        resp = SingleSetIOResponse()
        req.datapath = "Arp.Plc.Eclr/MainInstance.ROS1_Obj.bool_data1"
        req.value = val
        resp = set_client(req)
        return resp.status
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    rospy.init_node('service_caller', anonymous=True)
    value = True
    rate = rospy.Rate(1) # 1Hz
    while not rospy.is_shutdown():
        print("Calling single set with", value)
        service_call(value)
        value = not value
        rate.sleep()
