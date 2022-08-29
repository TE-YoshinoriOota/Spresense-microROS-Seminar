from std_srvs.srv import Trigger
import rclpy
from rclpy.node import Node

class TriggerService(Node):
  def __init__(self):
    self.result = 0
    self.cnt = 0
    super().__init__('trigger_service')
    self.srv = self.create_service(Trigger, 'srv_trigger_py', self.trigger_callback)
  def trigger_callback(self, request, response):
    self.get_logger().info('incoming request')
    response.success = not self.result
    self.result = response.success
    response.message = str(" Response[" + str(self.cnt) + "]")
    self.get_logger().info(response.message)
    self.cnt = self.cnt + 1
    return response

def main():
    rclpy.init()
    trigger_service = TriggerService()
    rclpy.spin(trigger_service)
  #  rclpy.shutdown()

if __name__ == '__main__':
    main()
