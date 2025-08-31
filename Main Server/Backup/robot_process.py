import os
import pandas as pd
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import PoseStamped
from my_msgs.msg import OrderMsg

class SignalRelay(Node):
    def __init__(self):
        super().__init__('signal_relay')

        self.bool_sub = self.create_subscription(Bool, 'pinky_sent_flag', self.bool_callback, 10)
        self.bool_pub = self.create_publisher(Bool, 'start_signal', 10)
        self.pose_sub = self.create_subscription(PoseStamped, 'cam_point', self.pose_callback, 10)

        # waypoint 도달 메시지 구독
        self.reached_sub = self.create_subscription(Int32, 'reached_point', self.reached_callback, 10)

        # 주문 퍼블리셔
        self.order_pub = self.create_publisher(OrderMsg, 'order_topic', 10)

        self.last_signal = None
        self.order_published = False  # 중복 퍼블리시 방지

        # CSV 경로 설정
        base_path = os.path.dirname(os.path.realpath('project'))
        self.csv_path = os.path.abspath(os.path.join(base_path, 'src','project','project', 'gui_server', 'received_data.csv'))

        self.get_logger().info("📡 Signal Relay Node 시작됨")

    def bool_callback(self, msg: Bool):
        if self.last_signal != msg.data:
            self.bool_pub.publish(msg)
            self.last_signal = msg.data
            self.get_logger().info(f"✅ Bool 수신 및 재전송: {msg.data}")
        else:
            self.get_logger().info(f"⛔ 중복 Bool 수신: {msg.data} (무시됨)")

    def pose_callback(self, msg: PoseStamped):
        self.get_logger().info(f"📬 위치 수신: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")

    def reached_callback(self, msg: Int32):
        self.get_logger().info(f"🎯 reached_point 수신: {msg.data}")
        if msg.data == 1 and not self.order_published:
            self.read_and_publish_order()
            self.order_published = True
        else:
            self.get_logger().info("⚠️ 조건 미충족 또는 이미 퍼블리시됨")

    def read_and_publish_order(self):
        try:
            df = pd.read_csv(self.csv_path)

            # 진행중(1) 주문만 추출
            active_orders = df[df["Status"] == 1]

            if active_orders.empty:
                self.get_logger().info("⏳ 진행중 주문 없음, 퍼블리시 생략")
                return

            # 가장 오래된 주문 1건
            active_orders['datetime'] = pd.to_datetime(active_orders['datetime'])
            active_orders = active_orders.sort_values(by='datetime')
            first_order = active_orders.iloc[0]

            # 커스텀 메시지 생성
            msg = OrderMsg()
            msg.order_id = int(first_order["ID"])
            msg.item = [str(first_order["Item"])]
            msg.count = [int(first_order["Count"])]

            self.order_pub.publish(msg)
            self.get_logger().info(f"📤 주문 퍼블리시: ID={msg.order_id}, Item={msg.item}, Count={msg.count}")

        except FileNotFoundError:
            self.get_logger().error(f"❌ CSV 파일 없음: {self.csv_path}")
        except Exception as e:
            self.get_logger().error(f"❌ 주문 퍼블리시 실패: {e}")

def main():
    rclpy.init()
    node = SignalRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 종료")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
