import os
import time
import socket
import pandas as pd
import rclpy
from threading import Thread
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float32
from geometry_msgs.msg import PoseStamped
from my_msgs.msg import OrderMsg


class SignalRelay(Node):
    def __init__(self):
        super().__init__('signal_relay')

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

        ######################## logistic pinky ########################
        self.bool_pub = self.create_publisher(Bool, 'start_signal', 10)
        self.logistic_pose_sub = self.create_subscription(PoseStamped, 'cam_point_4', self.logistic_pose_callback, 10)
        self.reached_sub = self.create_subscription(Int32, 'reached_point', self.reached_callback, 10)
        self.logistic_battery = self.create_subscription(Float32, 'logistic_pinky_battery_present', self.logistic_battery_callback, 10)

        ######################## mart pinky ########################
        self.mart_battery = self.create_subscription(Float32, 'mart_pinky_battery_present', self.mart_battery_callback, 10)
        self.mart_pose_sub = self.create_subscription(PoseStamped, 'cam_point_5', self.mart_pose_callback, 10)

        ######################## Item Retrieval Arm ########################
        self.bool_sub = self.create_subscription(Bool, 'pinky_sent_flag', self.bool_callback, 10)
        self.unloading_finish_msg = self.create_subscription(Bool, 'finish_msg', self.unloading_finish_callback, 10)
        self.order_pub = self.create_publisher(OrderMsg, 'order_topic1', 10)

        ######################## Item Transfer Arm ########################
        self.other_robot_order_pub = self.create_publisher(OrderMsg, 'order_topic2', 10)

        self.last_signal = None
        self.order_published = False
        self.start_signal_sent = False
        self.other_order_published = False
        self.bool_triggered_other_order_sent = False

        base_path = os.path.dirname(os.path.realpath('project'))
        self.csv_path = os.path.abspath(os.path.join(base_path, 'src', 'project', 'project', 'gui_server', 'received_data.csv'))

        self.get_logger().info("📡 Signal Relay Node 시작됨")

    ######################## 콜백 함수 ########################
    def bool_callback(self, msg: Bool):
        if self.last_signal != msg.data:
            self.bool_pub.publish(msg)
            self.last_signal = msg.data
            self.get_logger().info(f"✅ Bool 수신 및 재전송: {msg.data}")
            
            if msg.data is True:
                self.publish_start_signal_once()

                if not self.bool_triggered_other_order_sent:
                    self.read_and_publish_order_for_other_robot()
                    self.bool_triggered_other_order_sent = True
        else:
            self.get_logger().info(f"⛔ 중복 Bool 수신: {msg.data} (무시됨)")

    def logistic_pose_callback(self, msg: PoseStamped):
        self.get_logger().info(f"logistic 위치 수신: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")

    def mart_pose_callback(self, msg: PoseStamped):
        self.get_logger().info(f"mart 위치 수신: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
        

    def reached_callback(self, msg: Int32):
        self.get_logger().info(f"🎯 reached_point 수신: {msg.data}")
        if msg.data == 1 and not self.order_published:
            self.read_and_publish_order()
            self.order_published = True
        elif msg.data == 2 and not self.other_order_published:
            self.read_and_publish_order_for_other_robot()
            self.other_order_published = True
        else:
            self.get_logger().info("⚠️ 조건 미충족 또는 이미 퍼블리시됨")

    def unloading_finish_callback(self, msg: Bool):
        self.get_logger().info(f"하차 로봇 작업 끝 메시지 수신 : {msg.data}")
        if msg.data:
            self.get_logger().info("🚀 unloading_finish=True 감지, start_signal=True 퍼블리시 시도")
            self.publish_start_signal_once()

    def logistic_battery_callback(self, msg: Float32):
        self.get_logger().info(f"Logistic 배터리: {msg.data}")
        self.send_tcp_data(f"LOGISTIC:{msg.data}")

    def mart_battery_callback(self, msg: Float32):
        self.get_logger().info(f"Mart 배터리: {msg.data}")
        self.send_tcp_data(f"MART:{msg.data}")

    ######################## TCP 전송 함수 ########################
    def send_tcp_data(self, data: str):
        if self.sock:
            try:
                self.sock.sendall((data + "\n").encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"📡 TCP 전송 실패: {e}")

    ######################## start_signal 퍼블리시 ########################
    def publish_start_signal_once(self):
        if self.start_signal_sent:
            self.get_logger().info("⛔ start_signal 이미 보냄 (무시됨)")
            return
        start_msg = Bool()
        start_msg.data = True
        self.bool_pub.publish(start_msg)
        self.get_logger().info("🚀 start_signal=True 퍼블리시")
        self.start_signal_sent = True

        def reset_flag():
            time.sleep(2.0)
            self.start_signal_sent = False
            self.get_logger().info("🔄 start_signal_sent 플래그 리셋됨")

        Thread(target=reset_flag, daemon=True).start()

    ######################## 주문 처리 ########################
    def read_and_publish_order(self):
        try:
            df = pd.read_csv(self.csv_path)
            pending_orders = df[df["Status"] == 2]
            if pending_orders.empty:
                self.get_logger().info("📭 대기 중인 주문 없음")
                return

            pending_orders['datetime'] = pd.to_datetime(pending_orders['datetime'])
            pending_orders = pending_orders.sort_values(by='datetime')
            first_order = pending_orders.iloc[0]

            order_index = df[df["ID"] == first_order["ID"]].index[0]
            df.at[order_index, "Status"] = 1
            df.to_csv(self.csv_path, index=False)
            self.get_logger().info(f"✏️ 주문 ID {first_order['ID']} 상태를 1로 변경 후 저장")

            msg = OrderMsg()
            msg.order_id = int(first_order["ID"])
            msg.item = [str(first_order["Item"])]
            msg.count = [int(first_order["Count"])]
            self.order_pub.publish(msg)
            self.get_logger().info(f"📤 주문 퍼블리시 (로봇 1): ID={msg.order_id}, Item={msg.item}, Count={msg.count}")
            self.publish_start_signal_once()

        except Exception as e:
            self.get_logger().error(f"❌ 주문 처리 실패: {e}")

    def read_and_publish_order_for_other_robot(self):
        try:
            df = pd.read_csv(self.csv_path)
            pending_orders = df[df["Status"] == 2]
            if pending_orders.empty:
                self.get_logger().info("📭 다른 로봇용 주문 없음")
                return

            pending_orders['datetime'] = pd.to_datetime(pending_orders['datetime'])
            pending_orders = pending_orders.sort_values(by='datetime')
            first_order = pending_orders.iloc[0]

            order_index = df[df["ID"] == first_order["ID"]].index[0]
            df.at[order_index, "Status"] = 1
            df.to_csv(self.csv_path, index=False)
            self.get_logger().info(f"✏️ [다른 로봇] 주문 ID {first_order['ID']} 상태 1로 변경 후 저장")

            msg = OrderMsg()
            msg.order_id = int(first_order["ID"])
            msg.item = [str(first_order["Item"])]
            msg.count = [int(first_order["Count"])]
            self.other_robot_order_pub.publish(msg)
            self.get_logger().info(f"📤 주문 퍼블리시 (다른 로봇): ID={msg.order_id}, Item={msg.item}, Count={msg.count}")
            self.publish_start_signal_once()

        except Exception as e:
            self.get_logger().error(f"❌ 다른 로봇 주문 처리 실패: {e}")


######################## 메인 함수 ########################
def main():
    rclpy.init()
    node = SignalRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 종료")
    if node.sock:
        node.sock.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
