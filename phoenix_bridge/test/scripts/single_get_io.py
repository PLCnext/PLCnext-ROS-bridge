#!/usr/bin/env python3

import sys
import time 

from phoenix_bridge.srv import SingleGetIO
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):

  def __init__(self):
    super().__init__('single_get_IO')
    self.cli = self.create_client(SingleGetIO, '/single_get_IO')
    while not self.cli.wait_for_service(timeout_sec=1.0):
        self.get_logger().info('service not available, waiting again...')
    self.req = SingleGetIO.Request()

    while True:
      try:
        self.req.datapath = 'Arp.Plc.Eclr/MainInstance.gRPC_Obj.bool_data'
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        print("Get from", self.req.datapath, ", status", response.status, "value", response.value)
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
