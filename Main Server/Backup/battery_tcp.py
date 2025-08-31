import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import socket

class BatteryMonitorTCP(Node):
    def __init__(self):
        super().__init__('battery_monitor_tcp')

        # 초기값
        self.battery1 = None
        self.battery2 = None

        # 배터리 토픽 구독
        self.create_subscription(Float32, '/robot1/battery', self.robot1_callback, 10)
        self.create_subscription(Float32, '/robot2/battery', self.robot2_callback, 10)

        # 주기적으로 TCP 전송
        self.timer = self.create_timer(2.0, self.send_battery_tcp)  # 2초마다 전송

    def robot1_callback(self, msg):
        self.battery1 = round(msg.data, 1)  # 소수점 첫째 자리로 반올림
        self.get_logger().info(f"로봇1 배터리: {self.battery1}%")

    def robot2_callback(self, msg):
        self.battery2 = round(msg.data, 1)
        self.get_logger().info(f"로봇2 배터리: {self.battery2}%")

    def send_battery_tcp(self):
        if self.battery1 is None or self.battery2 is None:
            self.get_logger().warn("배터리 정보 부족 → 전송 안 함")
            return

        message = f"{self.battery1:.1f},{self.battery2:.1f}"
        HOST = '192.168.1.11'  # TCP 서버 IP
        PORT = 9999

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect((HOST, PORT))
                sock.sendall(message.encode())
                self.get_logger().info(f"🚀 배터리 전송: {message}")
        except Exception as e:
            self.get_logger().error(f"TCP 전송 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorTCP()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
