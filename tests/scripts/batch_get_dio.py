#!/usr/bin/env python3

import sys
import rospy
from phoenix_bridge.srv import BatchGetIO, BatchGetIORequest, BatchGetIOResponse
from phoenix_bridge.msg import SetIO

def service_call():
    rospy.wait_for_service('batch_get_io')
    try:
        get_client = rospy.ServiceProxy('batch_get_io', BatchGetIO)
        req = BatchGetIORequest()
        resp = BatchGetIOResponse()
        req.datapaths.append("Arp.Plc.Eclr/MainInstance.ROS1_Obj.bool_data1")
        req.datapaths.append("Arp.Plc.Eclr/MainInstance.ROS1_Obj.bool_data2")
        req.datapaths.append("Arp.Plc.Eclr/MainInstance.ROS1_Obj.bool_data3")

        print("Sending request", req)
        resp = get_client(req)
        print("Got response", resp)
        return resp.status
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    rospy.init_node('service_caller', anonymous=True)
    rate = rospy.Rate(1) # 1Hz
    while not rospy.is_shutdown():
        print("Calling batch set")
        service_call()
        rate.sleep()
