#!/usr/bin/env python3

import sys
import rospy
from phoenix_bridge.srv import BatchSetIO, BatchSetIORequest, BatchSetIOResponse
from phoenix_bridge.msg import SetIO

def service_call(value: bool):
    rospy.wait_for_service('batch_set_io')
    try:
        get_client = rospy.ServiceProxy('batch_set_io', BatchSetIO)
        req = BatchSetIORequest()
        resp = BatchSetIOResponse()
        req.payload.append(SetIO("Arp.Plc.Eclr/MainInstance.ROS1_Obj.bool_data1", value))
        req.payload.append(SetIO("Arp.Plc.Eclr/MainInstance.ROS1_Obj.bool_data2", not value))
        req.payload.append(SetIO("Arp.Plc.Eclr/MainInstance.ROS1_Obj.bool_data3", value))

        print("Sending request", req)
        resp = get_client(req)
        return resp.status
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    rospy.init_node('service_caller', anonymous=True)
    rate = rospy.Rate(1) # 1Hz
    value = True
    while not rospy.is_shutdown():
        print("Calling batch set with", value)
        service_call(value)
        value = not value
        rate.sleep()
