#!/usr/bin/env python3

import sys
import time

from phoenix_bridge.srv import BatchSetIO
from phoenix_bridge.msg import SetIO
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):

  def __init__(self):
    super().__init__('batch_set_IO')
    self.cli = self.create_client(BatchSetIO, '/batch_set_IO')
    while not self.cli.wait_for_service(timeout_sec=1.0):
        self.get_logger().info('service not available, waiting again...')
    self.req = BatchSetIO.Request()
    self.bool_ = True

    while True:
      try:
        self.req.payload.clear()

        msg = SetIO()
        msg.datapath = "Arp.Plc.Eclr/MainInstance.gRPC_Obj.bool_data"
        msg.value = self.bool_
        self.req.payload.append(msg)

        msg2 = SetIO()
        msg2.datapath = "Arp.Plc.Eclr/MainInstance.gRPC_Obj.bool_data2"
        msg2.value = not self.bool_
        self.req.payload.append(msg2)

        msg3 = SetIO()
        msg3.datapath = "Arp.Plc.Eclr/MainInstance.gRPC_Obj.bool_data3"
        msg3.value = self.bool_
        self.req.payload.append(msg3)

        self.bool_ = not self.bool_
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        print("Payload sent: ")
        for element in self.req.payload:
          print(element.datapath, element.value)
        print("Response status: ", response.status)
        time.sleep(1)
      except KeyboardInterrupt:
        break

def main(args=None):
  rclpy.init(args=args)
  minimal_client = MinimalClientAsync()
  minimal_client.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
