#!/usr/bin/env python3

import sys
import time 

from phoenix_bridge.srv import AnalogIO
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):

  def __init__(self):
    super().__init__('write_analog_io')
    self.cli = self.create_client(AnalogIO, '/write_analog_IO')
    while not self.cli.wait_for_service(timeout_sec=1.0):
        self.get_logger().info('service not available, waiting again...')
    self.req = AnalogIO.Request()
    self.double = 1.0

    while True:
      try:
        self.req.instance_path = 'Arp.Plc.Eclr/MainInstance.gRPC_Obj.double_data'
        self.req.value = self.double
        self.double += 1.0
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        print("Set ", self.req.value, " to ", self.req.instance_path, " and got result ", response.status)
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
