#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
from phoenix_bridge.srv import AnalogIO, AnalogIORequest, AnalogIOResponse

def service_call(val : bool):
    rospy.wait_for_service('read_analog_io')
    try:
        set_client = rospy.ServiceProxy('read_analog_io', AnalogIO)
        req = AnalogIORequest()
        resp = AnalogIOResponse()
        req.instance_path = "Arp.Plc.Eclr/MainInstance.ROS1_Obj.bool_data1"
        resp = set_client(req)
        print("Got response", resp.value)
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