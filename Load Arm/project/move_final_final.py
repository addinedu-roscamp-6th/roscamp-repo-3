#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import torch
import numpy as np
import time
from std_msgs.msg import Bool
from my_msgs.msg import OrderMsg
from pymycobot.mycobot280 import MyCobot280

class YoloOrderRobotNode(Node):
    def __init__(self):
        super().__init__('yolo_order_robot_node')
        self.get_logger().info("YOLO Order Robot Node 시작")

        # ROS2 통신
        self.subscription_order = self.create_subscription(OrderMsg, 'order_topic1', self.order_callback, 10)
        self.publisher_done = self.create_publisher(Bool, 'pinky_sent_flag', 10)

        # 로봇팔 초기화
        self.mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
        self.mc.set_gripper_value(100, 50)
        self.home_angles = [-0.43, 51.94, 0.43, -52.03, 11.07, -45.61]

        # 카메라 초기화
        self.cap = cv2.VideoCapture('/dev/jetcocam0')
        self.cap.set(cv2.CAP_PROP_FPS, 15)

        # Calibration
        calib = np.load('/home/jetcobot/project/calib/calibration.npz')
        self.K = calib['K']
        self.f = self.K[0, 0]
        self.W = 30  # mm

        # Homography
        points_world = np.array([[15.8, 51.4], [-32.9, 22.9], [-45.7, -20.9], [27.8, -20.4]], dtype=np.float32)
        points_img = np.array([[370, 395], [22, 388], [200, 93], [488, 104]], dtype=np.float32)
        self.H, _ = cv2.findHomography(points_img, points_world)

        # YOLO 모델
        self.model = torch.hub.load(
            '/home/jetcobot/project/calib/src/yolov5',
            'custom',
            path='/home/jetcobot/project/calib/src/yolov5/runs/train/exp13/weights/best.pt',
            source='local'
        )

        # 주문 정보
        self.target_items = []
        self.target_counts = []
        self.current_item_index = 0
        self.cnt = 0
        self.processing_order = False

        # 타이머
        self.timer = self.create_timer(0.5, self.main_loop)

    def order_callback(self, msg: OrderMsg):
        self.get_logger().info(f"  Item: {msg.item}")
        self.get_logger().info(f"  Count: {msg.count}")

        self.target_items = msg.item
        self.target_counts = msg.count
        self.current_item_index = 0
        self.cnt = 0
        self.processing_order = True

    def main_loop(self):
        if not self.processing_order:
            return
        if self.current_item_index >= len(self.target_items):
            return

        item_name = self.target_items[self.current_item_index]
        target_count = self.target_counts[self.current_item_index]
        print(f"item_name = {item_name}")

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("카메라 프레임 읽기 실패")
            return

        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.model(img_rgb)

        for *box, conf, cls in results.xyxy[0].cpu().numpy():
            print(cls)
            label_name = self.model.names[int(cls)]
            print(f"label_name = {label_name}")
            if label_name != item_name:
                return  # 현재 목표 아이템만 처리

            # ====== 바운딩박스 그리기 ======
            label_name = model.names[int(cls)]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(frame, f"{label_name} {conf:.2f}", (x1, y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

            # ====== 감지된 이미지 저장 ======
            save_path = f'./detected_{label_name}_{time.strftime("%Y%m%d_%H%M%S")}.jpg'
            cv2.imwrite(save_path, frame)
            print(f"[INFO] 감지 이미지 저장 완료: {save_path}")

            goal_coords = coords.copy()
            if (Y_mm > 40):
                goal_coords[2] = 340
                goal_coords[0] = -15
                if (X_mm < -35):
                    goal_coords[1] -= X_mm + 5
                    print("1")
                elif (X_mm > -36 and X_mm < 30) :
                    goal_coords[1] -= X_mm + 18
                    print("2")
                else :
                    goal_coords[1] -= X_mm + 3
                    print("3")

            else:
                goal_coords[2] = 382
                goal_coords[0] = -15
                if (X_mm < - 57):
                    goal_coords[1] -= X_mm + 5
                    print("1")
                elif (X_mm > -58 and X_mm < 15) :
                    goal_coords[1] -= X_mm + 15
                    print("2")
                else :
                    goal_coords[1] -= X_mm + 3
                    print("3")

            mc.send_coords(goal_coords, 50, 1)
            time.sleep(2)
            
            goal_coords[0] += 20
            mc.send_coords(goal_coords, 50, 1)
            time.sleep(1)

            mc.set_gripper_value(0, 50)
            time.sleep(1)

            mc.send_angles([-0.43, 51.94, 0.43, -52.03, 11.07, -45.61], 50)
            mc.set_gripper_value(70, 50)
            time.sleep(1)

            # --- 수량 카운트 ---
            self.cnt += 1
            self.get_logger().info(f"[INFO] {item_name} 처리 수량: {self.cnt}/{target_count}")

            if self.cnt >= target_count:
                self.cnt = 0
                self.current_item_index += 1
                if self.current_item_index >= len(self.target_items):
                    done_msg = Bool()
                    done_msg.data = True
                    self.publisher_done.publish(done_msg)
                    self.get_logger().info("[INFO] 모든 주문 완료. Bool True 전송.")
                    self.processing_order = False
            break

    def destroy_node(self):
        self.cap.release()
        self.get_logger().info("카메라 해제 및 종료")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloOrderRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
