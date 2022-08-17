#!/usr/bin/env python3

import sys
import time 

from phoenix_interfaces.srv import AnalogIO
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):

  def __init__(self):
    super().__init__('read_analog_IO')
    self.cli = self.create_client(AnalogIO, '/read_analog_IO')
    while not self.cli.wait_for_service(timeout_sec=1.0):
        self.get_logger().info('service not available, waiting again...')
    self.req = AnalogIO.Request()

    while True:
      try:
        self.req.instance_path = 'Arp.Plc.Eclr/MainInstance.gRPC_Obj.double_data'
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        print("Got from", self.req.instance_path, "result ", response.status, "value", response.value)
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
