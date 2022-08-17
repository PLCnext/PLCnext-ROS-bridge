#!/usr/bin/env python3

import sys
import time

from phoenix_interfaces.srv import BatchGetIO
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):

  def __init__(self):
    super().__init__('batch_get_io')
    self.cli = self.create_client(BatchGetIO, '/batch_get_IO')
    while not self.cli.wait_for_service(timeout_sec=1.0):
        self.get_logger().info('service not available, waiting again...')
    self.req = BatchGetIO.Request()

    while True:
      try:
        self.req.datapaths.clear()
        self.req.datapaths.append("Arp.Plc.Eclr/MainInstance.gRPC_Obj.bool_data")
        self.req.datapaths.append("Arp.Plc.Eclr/MainInstance.gRPC_Obj.bool_data2")
        self.req.datapaths.append("Arp.Plc.Eclr/MainInstance.gRPC_Obj.bool_data3")

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        print("Payload sent: ")
        for element in self.req.datapaths:
          print(element)
        print("Response status: ", response.status)
        print("Values: ")
        for element in response.values:
          print(element)
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
