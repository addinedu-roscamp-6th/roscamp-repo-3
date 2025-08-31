import os
import time
import pandas as pd
import rclpy
from threading import Thread
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import PoseStamped
from my_msgs.msg import OrderMsg


class SignalRelay(Node):
    def __init__(self):
        super().__init__('signal_relay')

        # 퍼블리셔 & 서브스크라이버 생성
        self.bool_pub = self.create_publisher(Bool, 'start_signal', 10)
        self.reached_sub = self.create_subscription(Int32, 'reached_point', self.reached_callback, 10)
        self.bool_sub = self.create_subscription(Bool, 'pinky_sent_flag', self.bool_callback, 10)
        self.finish_sub = self.create_subscription(Bool, 'finish_msg', self.finish_callback, 10)

        self.order_pub1 = self.create_publisher(OrderMsg, 'order_topic1', 10)
        self.order_pub2 = self.create_publisher(OrderMsg, 'order_topic2', 10)
        self.order_pub3 = self.create_publisher(OrderMsg, 'order_topic3', 10)

        self.last_signal = None
        self.start_signal_sent = False

        self.order_published_1 = False
        self.order_published_2 = False
        self.order_published_3 = False

        base_path = os.path.dirname(os.path.realpath('project'))
        self.csv_path = os.path.abspath(os.path.join(base_path, 'src', 'project', 'project', 'gui_server', 'received_data.csv'))

        self.get_logger().info("📡 Signal Relay Node 시작됨")

        # 1,2번: 시작 시 CSV에서 status==2인 가장 빠른 주문 읽고 상태 1로 바꾸고 start_signal true 1회 publish
        self.read_order_and_start()

        # 8, 9번: 8단계: status=2인 주문 계속 확인 + 9단계: 없으면 대기, 있으면 1단계로 복귀
        self.check_status2_orders_and_process()

    # 1,2번 구현
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

    # 2번: start_signal true 1회 publish
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

    # 3번: reached_point == 1 수신시 order_topic1 퍼블리시
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

    # 4번: pinky_sent_flag true 수신 시 start_signal true 1회 퍼블리시
    def bool_callback(self, msg: Bool):
        if self.last_signal != msg.data:
            self.last_signal = msg.data
            self.get_logger().info(f"✅ Bool 수신: {msg.data}")
            if msg.data:
                self.publish_start_signal_once()
        else:
            self.get_logger().info(f"⛔ 중복 Bool 수신: {msg.data} (무시됨)")

    # 6,7번: finish_msg true 수신 시 order_topic3 퍼블리시 + CSV에서 status==1인 항목 모두 0으로 수정 후 저장
    def finish_callback(self, msg: Bool):
        self.get_logger().info(f"📥 finish_msg 수신: {msg.data}")
        if msg.data and not self.order_published_3:
            self.publish_order_topic3()
            self.order_published_3 = True
            self.reset_status_1_to_0()

    # order_topic1 publish 함수
    def publish_order_topic1(self):
        try:
            df = pd.read_csv(self.csv_path)
            # status==1 중 가장 오래된 주문 1건
            orders = df[df["Status"] == 1]
            if orders.empty:
                self.get_logger().info("📭 상태 1인 주문 없음")
                return

            orders['datetime'] = pd.to_datetime(orders['datetime'])
            orders = orders.sort_values(by='datetime')
            first_order = orders.iloc[0]

            msg = OrderMsg()
            msg.order_id = int(first_order["ID"])
            msg.item = [str(first_order["Item"])]
            msg.count = [int(first_order["Count"])]

            self.order_pub1.publish(msg)
            self.get_logger().info(f"📤 order_topic1 퍼블리시: ID={msg.order_id}, Item={msg.item}, Count={msg.count}")
        except Exception as e:
            self.get_logger().error(f"❌ order_topic1 퍼블리시 실패: {e}")

    # order_topic2 publish 함수
    def publish_order_topic2(self):
        try:
            df = pd.read_csv(self.csv_path)
            orders = df[df["Status"] == 1]
            if orders.empty:
                self.get_logger().info("📭 상태 1인 주문 없음 (order_topic2)")
                return

            orders['datetime'] = pd.to_datetime(orders['datetime'])
            orders = orders.sort_values(by='datetime')
            first_order = orders.iloc[0]

            msg = OrderMsg()
            msg.order_id = int(first_order["ID"])
            msg.item = [str(first_order["Item"])]
            msg.count = [int(first_order["Count"])]

            self.order_pub2.publish(msg)
            self.get_logger().info(f"📤 order_topic2 퍼블리시: ID={msg.order_id}, Item={msg.item}, Count={msg.count}")
        except Exception as e:
            self.get_logger().error(f"❌ order_topic2 퍼블리시 실패: {e}")

    # order_topic3 publish 함수
    def publish_order_topic3(self):
        try:
            df = pd.read_csv(self.csv_path)
            orders = df[df["Status"] == 1]
            if orders.empty:
                self.get_logger().info("📭 상태 1인 주문 없음 (order_topic3)")
                return

            orders['datetime'] = pd.to_datetime(orders['datetime'])
            orders = orders.sort_values(by='datetime')
            first_order = orders.iloc[0]

            msg = OrderMsg()
            msg.order_id = int(first_order["ID"])
            msg.item = [str(first_order["Item"])]
            msg.count = [int(first_order["Count"])]

            self.order_pub3.publish(msg)
            self.get_logger().info(f"📤 order_topic3 퍼블리시: ID={msg.order_id}, Item={msg.item}, Count={msg.count}")
        except Exception as e:
            self.get_logger().error(f"❌ order_topic3 퍼블리시 실패: {e}")

    # 7번: CSV에서 status==1인 항목을 모두 0으로 수정 후 저장
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

    # 8, 9번: status==2인 주문 계속 확인
    def check_status2_orders_and_process(self):
        try:
            while True:
                df = pd.read_csv(self.csv_path)
                pending_orders = df[df["Status"] == 2]
                if pending_orders.empty:
                    self.get_logger().info("📭 대기 중인 주문 없음. 대기 중...")
                    time.sleep(5)  # 대기 후 재시작
                else:
                    self.get_logger().info("📥 새로운 주문 있음!")
                    self.read_order_and_start()  # 1단계 호출
                    break  # 상태를 1로 수정하고 반복 종료
        except Exception as e:
            self.get_logger().error(f"❌ 주문 상태 확인 및 처리 실패: {e}")

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
