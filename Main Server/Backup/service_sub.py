import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class StartPermissionClient(Node):
    def __init__(self):
        super().__init__('start_permission_client')
        self.client = self.create_client(Trigger, 'start_permission')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 대기 중...')

        self.send_request()

    def send_request(self):
        req = Trigger.Request()
        future = self.client.call_async(req)
        future.add_done_callback(self.callback_response)

    def callback_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'응답: {response.message} (성공: {response.success})')
        except Exception as e:
            self.get_logger().error(f'서비스 요청 실패: {e}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = StartPermissionClient()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
