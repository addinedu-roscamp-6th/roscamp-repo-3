import os
import time
import pandas as pd
import ast
import rclpy
from threading import Thread
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float32
from geometry_msgs.msg import PoseStamped
from my_msgs.msg import OrderMsg
import socket


class RobotProcess(Node):
    def __init__(self):
        super().__init__('robot_process')
        ######################## TCP 연결 설정 ########################
        TCP_IP = "192.168.1.11"
        TCP_PORT = 2525

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((TCP_IP, TCP_PORT))
            self.get_logger().info(f"🔗 TCP 연결 성공: {TCP_IP}:{TCP_PORT}")
        except Exception as e:
            self.get_logger().error(f"❌ TCP 연결 실패: {e}")
            self.sock = None

        # subscription
        ######################## logistic pinky ########################
        self.bool_pub = self.create_publisher(Bool, 'start_signal', 10)
        self.reached_sub = self.create_subscription(Int32, 'reached_point', self.reached_callback, 10)
        self.logistic_battery = self.create_subscription(Float32, 'logistic_pinky_battery_present', self.logistic_battery_callback, 10)
        ######################## mart pinky ########################
        self.return_sub = self.create_subscription(Bool, 'return_msg', self.return_callback, 10)
        self.mart_battery = self.create_subscription(Float32, 'mart_pinky_battery_present', self.mart_battery_callback, 10)
        ######################## Item Retrieval Arm ########################
        self.bool_sub = self.create_subscription(Bool, 'pinky_sent_flag', self.bool_callback, 10)
        ######################## Item Transfer Arm ########################
        self.finish_sub = self.create_subscription(Bool, 'finish_msg', self.finish_callback, 10)

        # publisher
        ######################## Item Retrieval Arm ########################
        self.order_pub1 = self.create_publisher(OrderMsg, 'order_topic1', 10)
        
        ######################## Item Transfer Arm ########################
        self.order_pub2 = self.create_publisher(OrderMsg, 'order_topic2', 10)
        
        ######################## mart pinky ########################
        self.order_pub3 = self.create_publisher(OrderMsg, 'order_topic3', 10)

        self.last_signal = None
        self.start_signal_sent = False
        
        self.order_published_1 = False
        self.order_published_2 = False
        self.order_published_3 = False

        base_path = os.path.dirname(os.path.realpath('project'))
        self.csv_path = os.path.abspath(os.path.join(base_path, 'src', 'project', 'project', 'gui_server', 'received_data.csv'))

        self.get_logger().info("📡 Signal Relay Node 시작됨")

        # CSV 읽어서 start_signal 보낼 준비
        self.read_order_and_start()


    def send_tcp_data(self, data: str):
        if self.sock:
            try:
                self.sock.sendall((data + "\n").encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"📡 TCP 전송 실패: {e}")

    def logistic_battery_callback(self, msg: Float32):
        self.get_logger().info(f"Logistic 배터리: {msg.data}")
        self.send_tcp_data(f"LOGISTIC:{msg.data}")

    def mart_battery_callback(self, msg: Float32):
        self.get_logger().info(f"Mart 배터리: {msg.data}")
        self.send_tcp_data(f"MART:{msg.data}")

    # CSV 읽고 Status==2 → 1 변경 후 start_signal 보냄
    def read_order_and_start(self):
        try:
            df = pd.read_csv(self.csv_path)
            pending_orders = df[df["Status"] == 2]
            if pending_orders.empty:
                self.get_logger().info("📭 대기 중인 주문 없음")
                return

            pending_orders['datetime'] = pd.to_datetime(pending_orders['datetime'])
            pending_orders = pending_orders.sort_values(by='datetime')
            first_order = pending_orders.iloc[0]

            idx = df[df["ID"] == first_order["ID"]].index[0]
            df.at[idx, "Status"] = 1
            df.to_csv(self.csv_path, index=False)
            self.get_logger().info(f"✏️ 주문 ID {first_order['ID']} 상태를 1로 변경 후 저장")

            self.publish_start_signal_once()

        except Exception as e:
            self.get_logger().error(f"❌ 주문 처리 실패: {e}")

    # start_signal True 1회 퍼블리시
    def publish_start_signal_once(self):
        if self.start_signal_sent:
            self.get_logger().info("⛔ start_signal 이미 보냄 (무시됨)")
            return

        msg = Bool()
        msg.data = True
        self.bool_pub.publish(msg)
        self.get_logger().info("🚀 start_signal=True 퍼블리시")
        self.start_signal_sent = True

        def reset_flag():
            time.sleep(2.0)
            self.start_signal_sent = False
            self.get_logger().info("🔄 start_signal_sent 플래그 리셋됨")

        Thread(target=reset_flag, daemon=True).start()

    # reached_point 콜백
    def reached_callback(self, msg: Int32):
        self.get_logger().info(f"🎯 reached_point 수신: {msg.data}")
        if msg.data == 1 and not self.order_published_1:
            self.publish_order_topic1()
            self.order_published_1 = True
        elif msg.data == 2 and not self.order_published_2:
            self.publish_order_topic2()
            self.order_published_2 = True
        else:
            self.get_logger().info("⚠️ 조건 미충족 또는 이미 퍼블리시됨")

    # pinky_sent_flag 콜백
    def bool_callback(self, msg: Bool):
        if self.last_signal != msg.data:
            self.last_signal = msg.data
            self.get_logger().info(f"✅ Bool 수신: {msg.data}")
            if msg.data:
                self.publish_start_signal_once()
        else:
            self.get_logger().info(f"⛔ 중복 Bool 수신: {msg.data} (무시됨)")

    # finish_msg 콜백
    def finish_callback(self, msg: Bool):
        self.get_logger().info(f"📥 finish_msg 수신: {msg.data}")
        if msg.data and not self.order_published_3:
            self.publish_order_topic3()
            self.order_published_3 = True
            self.reset_status_1_to_0()

    def return_callback(self, msg: Bool):
        self.get_logger().info(f"↩ return_msg 수신: {msg.data}")
        self.return_msg = msg.data

    # order_topic1 퍼블리시
    def publish_order_topic1(self):
        self._publish_order_from_csv(self.order_pub1, "order_topic1")

    # order_topic2 퍼블리시
    def publish_order_topic2(self):
        self._publish_order_from_csv(self.order_pub2, "order_topic2")

    # order_topic3 퍼블리시
    def publish_order_topic3(self):
        self._publish_order_from_csv(self.order_pub3, "order_topic3")

    # CSV에서 status==1 중 가장 오래된 주문 읽어 퍼블리시
    def _publish_order_from_csv(self, publisher, topic_name):
        try:
            df = pd.read_csv(self.csv_path)
            orders = df[df["Status"] == 1]
            if orders.empty:
                self.get_logger().info(f"📭 상태 1인 주문 없음 ({topic_name})")
                return

            orders['datetime'] = pd.to_datetime(orders['datetime'])
            orders = orders.sort_values(by='datetime')
            first_order = orders.iloc[0]

            msg = OrderMsg()
            msg.order_id = int(first_order["ID"])
            msg.item = ast.literal_eval(first_order["Item"])
            msg.count = ast.literal_eval(first_order["Count"])

            publisher.publish(msg)
            self.get_logger().info(f"📤 {topic_name} 퍼블리시: ID={msg.order_id}, Item={msg.item}, Count={msg.count}")
        except Exception as e:
            self.get_logger().error(f"❌ {topic_name} 퍼블리시 실패: {e}")

    # status==1 → 0 변경
    def reset_status_1_to_0(self):
        try:
            df = pd.read_csv(self.csv_path)
            status_1_idx = df[df["Status"] == 1].index
            if len(status_1_idx) == 0:
                self.get_logger().info("⚠️ CSV에 status==1인 항목 없음")
                return

            df.loc[status_1_idx, "Status"] = 0
            df.to_csv(self.csv_path, index=False)
            self.get_logger().info("✏️ CSV status==1인 항목 모두 0으로 변경 후 저장 완료")
        except Exception as e:
            self.get_logger().error(f"❌ CSV 상태 리셋 실패: {e}")


def main():
    rclpy.init()
    node = RobotProcess()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 종료")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
