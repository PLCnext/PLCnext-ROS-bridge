#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
from phoenix_bridge.srv import SingleGetIO, SingleGetIORequest, SingleGetIOResponse

def service_call():
    rospy.wait_for_service('single_get_io')
    try:
        get_client = rospy.ServiceProxy('single_get_io', SingleGetIO)
        req = SingleGetIORequest()
        resp = SingleGetIOResponse()
        req.datapath = "Arp.Plc.Eclr/MainInstance.ROS1_Obj.bool_data1"
        resp = get_client(req)
        print("Got response", resp.value)
        return resp.status
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    rospy.init_node('service_caller', anonymous=True)
    rate = rospy.Rate(1) # 1Hz
    while not rospy.is_shutdown():
        print("Calling single get")
        service_call()
        rate.sleep()