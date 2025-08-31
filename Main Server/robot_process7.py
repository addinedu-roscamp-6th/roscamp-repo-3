#!/usr/bin/env python3
import os
import time
import pandas as pd
import ast
import rclpy
from threading import Thread
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float32
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

        ######################## ROS Subscribers ########################
        self.reached_sub = self.create_subscription(Int32, 'reached_point', self.reached_callback, 10)
        self.bool_sub = self.create_subscription(Bool, 'pinky_sent_flag', self.bool_callback, 10)
        self.finish_sub = self.create_subscription(Bool, 'finish_msg', self.finish_callback, 10)
        self.logistic_battery = self.create_subscription(Float32, 'logistic_pinky_battery_present', self.logistic_battery_callback, 10)
        self.mart_battery = self.create_subscription(Float32, 'mart_pinky_battery_present', self.mart_battery_callback, 10)
        self.return_sub = self.create_subscription(Bool, 'return_msg', self.return_callback, 10)

        ######################## ROS Publishers ########################
        self.bool_pub = self.create_publisher(Bool, 'start_signal', 10)
        self.order_pub1 = self.create_publisher(OrderMsg, 'order_topic1', 10)
        self.order_pub2 = self.create_publisher(OrderMsg, 'order_topic2', 10)
        self.order_pub3 = self.create_publisher(OrderMsg, 'order_topic3', 10)

        self.start_signal_sent = False
        self.last_signal = None

        base_path = os.path.dirname(os.path.realpath('project'))
        self.csv_path = os.path.abspath(os.path.join(base_path, 'src', 'project', 'project', 'gui_server', 'received_data.csv'))

        self.get_logger().info("📡 Robot Process Node 시작됨")

        # 처음 실행 시 status 3 주문이 있으면 start_signal 발행
        self.start_first_waiting_order()

    ######################## TCP 전송 ########################
    def send_tcp_data(self, data: str):
        if self.sock:
            try:
                self.sock.sendall((data + "\n").encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"📡 TCP 전송 실패: {e}")

    ######################## 배터리 콜백 ########################
    def logistic_battery_callback(self, msg: Float32):
        self.get_logger().info(f"Logistic 배터리: {msg.data}")
        self.send_tcp_data(f"LOGISTIC:{msg.data}")

    def mart_battery_callback(self, msg: Float32):
        self.get_logger().info(f"Mart 배터리: {msg.data}")
        self.send_tcp_data(f"MART:{msg.data}")

    ######################## CSV 주문 처리 ########################
    def start_first_waiting_order(self):
        row, key = self._pick_first_waiting()
        if row is None:
            self.get_logger().info("📭 대기(status==3) 주문 없음")
            return
        self._set_status_and_save(key, 2)  # status 3 → 2
        self.publish_start_signal_once()

    def _pick_first_waiting(self):
        try:
            df = pd.read_csv(self.csv_path)
            pending_orders = df[df["Status"] == 3]
            if pending_orders.empty:
                return None, None
            pending_orders['datetime'] = pd.to_datetime(pending_orders['datetime'])
            pending_orders = pending_orders.sort_values(by='datetime')
            first_order = pending_orders.iloc[0]
            key = df[df["ID"] == first_order["ID"]].index[0]
            return first_order, key
        except Exception as e:
            self.get_logger().error(f"❌ _pick_first_waiting 실패: {e}")
            return None, None

    def _set_status_and_save(self, idx, status):
        try:
            df = pd.read_csv(self.csv_path)
            df.at[idx, "Status"] = status
            df.to_csv(self.csv_path, index=False)
            self.get_logger().info(f"✏️ 주문 ID={idx} Status={status}로 변경 후 저장")
        except Exception as e:
            self.get_logger().error(f"❌ Status 변경 실패: {e}")

    ######################## start_signal 퍼블리시 ########################
    def publish_start_signal_once(self):
        msg = Bool()
        msg.data = True
        self.bool_pub.publish(msg)
        self.get_logger().info("🚀 start_signal=True 퍼블리시")
        self.start_signal_sent = True

        # 플래그 리셋
        def reset_flag():
            time.sleep(2.0)
            self.start_signal_sent = False
            self.last_signal = None  # 중복 수신 문제 해결
            self.get_logger().info("🔄 start_signal_sent 및 last_signal 리셋됨")
        Thread(target=reset_flag, daemon=True).start()

    ######################## ROS 콜백 ########################
    def reached_callback(self, msg: Int32):
        self.get_logger().info(f"🎯 reached_point 수신: {msg.data}")
        if msg.data == 1:
            self.publish_order_topic1()
        elif msg.data == 2:
            self.publish_order_topic2()

    def bool_callback(self, msg: Bool):
        # last_signal 체크 제거 후 항상 처리
        if msg.data:
            self.get_logger().info(f"✅ Bool 수신: {msg.data}")
            self.publish_start_signal_once()
        else:
            self.get_logger().info(f"Bool 수신: {msg.data} (False)")

    def finish_callback(self, msg: Bool):
        self.get_logger().info(f"📥 finish_msg 수신: {msg.data}")
        if msg.data:
            # status 2 → 1
            self._update_first_status(2, 1)
            self.publish_order_topic3()

    def return_callback(self, msg: Bool):
        self.get_logger().info(f"↩ return_msg 수신: {msg.data}")
        if msg.data:
            # status 1 → 0
            self._update_first_status(1, 0)
            # 다음 status 3 주문이 있으면 start_signal 재발행
            self.start_first_waiting_order()

    ######################## Order 퍼블리시 ########################
    def publish_order_topic1(self):
        self._publish_order_from_csv(self.order_pub1, "order_topic1")

    def publish_order_topic2(self):
        self._publish_order_from_csv(self.order_pub2, "order_topic2")

    def publish_order_topic3(self):
        self._publish_order_from_csv(self.order_pub3, "order_topic3")

    def _publish_order_from_csv(self, publisher, topic_name):
        try:
            df = pd.read_csv(self.csv_path)
            orders = df[df["Status"] == 2]  # 진행 중인 주문
            if orders.empty:
                self.get_logger().info(f"📭 상태 2인 주문 없음 ({topic_name})")
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

    ######################## Status 업데이트 ########################
    def _update_first_status(self, current_status, new_status):
        try:
            df = pd.read_csv(self.csv_path)
            idxs = df[df["Status"] == current_status].index
            if len(idxs) == 0:
                self.get_logger().info(f"⚠️ CSV에 Status={current_status}인 항목 없음")
                return
            first_idx = idxs[0]
            df.at[first_idx, "Status"] = new_status
            df.to_csv(self.csv_path, index=False)
            self.get_logger().info(f"✏️ 주문 ID={first_idx} Status={current_status}->{new_status} 변경 완료")
        except Exception as e:
            self.get_logger().error(f"❌ Status 업데이트 실패: {e}")

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
