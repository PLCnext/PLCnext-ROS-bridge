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
