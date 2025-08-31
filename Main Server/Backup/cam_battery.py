# client_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import socket
import cv2
import numpy as np
import threading
import time

# ---------- 서버 정보 ----------
TCP_IP = '192.168.1.11'
TCP_PORT = 9999

UDP_IP = '192.168.1.11'
UDP_PORT = 5005

# ---------- ROS 2 + TCP 전송 ----------
class BatteryMonitorTCP(Node):
    def __init__(self):
        super().__init__('battery_monitor_tcp')

        self.battery1 = None
        self.battery2 = None

        self.create_subscription(Float32, '/logistic_pinky_battery_present', self.robot1_callback, 10)
        self.create_subscription(Float32, '/robot2/battery', self.robot2_callback, 10)

        self.timer = self.create_timer(2.0, self.send_battery_tcp)

    def robot1_callback(self, msg):
        self.battery1 = round(msg.data, 1)
        self.get_logger().info(f"로봇1 배터리: {self.battery1}%")

    def robot2_callback(self, msg):
        self.battery2 = round(msg.data, 1)
        self.get_logger().info(f"로봇2 배터리: {self.battery2}%")

    def send_battery_tcp(self):
        if self.battery1 is None or self.battery2 is None:
            self.get_logger().warn("배터리 정보 부족 → 전송 생략")
            return

        message = f"{self.battery1:.1f},{self.battery2:.1f}"
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect((TCP_IP, TCP_PORT))
                sock.sendall(message.encode())
                self.get_logger().info(f"🔋 TCP 전송: {message}")
        except Exception as e:
            self.get_logger().error(f"TCP 전송 실패: {e}")

# ---------- OpenCV + UDP 전송 ----------
def udp_video_sender():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cap = cv2.VideoCapture(0)  # 필요에 따라 1, 2로 변경
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        data = buffer.tobytes()

        for i in range(0, len(data), 60000):
            sock.sendto(data[i:i+60000], (UDP_IP, UDP_PORT))
        print(f"📸 UDP 프레임 전송 중 ({len(data)} bytes)")
        time.sleep(0.1)  # 너무 빠른 전송 방지

# ---------- 메인 실행 ----------
def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorTCP()

    # OpenCV UDP 송신을 별도 쓰레드로 실행
    video_thread = threading.Thread(target=udp_video_sender, daemon=True)
    video_thread.start()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
