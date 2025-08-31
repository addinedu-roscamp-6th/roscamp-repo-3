import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class StartPermissionServer(Node):
    def __init__(self):
        super().__init__('start_permission_server')
        self.srv = self.create_service(Trigger, 'start_permission', self.start_permission_callback)
        self.get_logger().info('Service server ready: start_permission')

    def start_permission_callback(self, request, response):
        self.get_logger().info('Received start permission request')
        response.success = True
        response.message = '출발하세요!'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = StartPermissionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
