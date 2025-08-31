#!/usr/bin/env python3
import os
import time
import cv2
import torch
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from my_msgs.msg import OrderMsg
from pymycobot.mycobot280 import MyCobot280


class YoloOrderRobotNode(Node):
    def __init__(self):
        super().__init__('yolo_order_robot_node')
        self.get_logger().info("YOLO Order Robot Node 시작")

        # ===== 사용자/환경 설정 =====
        self.PORT = '/dev/ttyJETCOBOT'
        self.BAUD = 1000000
        self.CAMERA_DEV = '/dev/jetcocam0'
        self.CALIB_PATH = '/home/jetcobot/project/calib/calibration.npz'
        self.YOLO_REPO = '/home/jetcobot/project/calib/src/yolov5'
        self.YOLO_WEIGHTS = '/home/jetcobot/project/calib/src/yolov5/runs/train/exp15/weights/best.pt'
        self.SAVE_DIR = os.path.expanduser('~/detections')
        os.makedirs(self.SAVE_DIR, exist_ok=True)

        self.LABEL_ALIAS = {
            'shoes': ['shoes', 'fruit'],
        }

        # ===== ROS2 통신 =====
        self.subscription_order = self.create_subscription(OrderMsg, 'order_topic1', self.order_callback, 10)
        self.publisher_done = self.create_publisher(Bool, 'pinky_sent_flag', 10)

        # ===== 로봇팔 초기화 =====
        self.mc = MyCobot280(self.PORT, self.BAUD)
        self.mc.set_gripper_value(100, 50)
        self.home_angles = [-0.43, 51.94, 0.43, -52.03, 11.07, -45.61]
        self.mc.send_angles(self.home_angles, 100)
        self.mc.set_gripper_value(70, 100)

        # ===== 카메라 초기화 =====
        self.cap = cv2.VideoCapture(self.CAMERA_DEV)
        self.cap.set(cv2.CAP_PROP_FPS, 15)
        if not self.cap.isOpened():
            self.get_logger().error(f"카메라 열기 실패: {self.CAMERA_DEV}")

        # ===== Calibration =====
        calib = np.load(self.CALIB_PATH)
        self.K = calib['K']
        self.f = float(self.K[0, 0])
        self.W = 30.0  # mm (박스 기준 너비)

        # ===== Homography =====
        points_world = np.array([[15.8, 51.4], [-32.9, 22.9], [-45.7, -20.9], [27.8, -20.4]], dtype=np.float32)
        points_img = np.array([[370, 395], [22, 388], [200, 93], [488, 104]], dtype=np.float32)
        self.H, _ = cv2.findHomography(points_img, points_world)

        # ===== YOLO 모델 =====
        self.model = torch.hub.load(
            self.YOLO_REPO,
            'custom',
            path=self.YOLO_WEIGHTS,
            source='local'   
        )
        self.model.conf = 0.25

        # ===== 주문 상태 =====
        self.target_items = []
        self.target_counts = []
        self.current_item_index = 0
        self.cnt = 0
        self.processing_order = False

        # ===== 메인 루프 타이머 =====
        self.timer = self.create_timer(0.3, self.main_loop)

    def order_callback(self, msg: OrderMsg):
        self.get_logger().info(f"  Item: {list(msg.item)}")
        self.get_logger().info(f"  Count: {list(msg.count)}")

        self.target_items = list(msg.item)
        self.target_counts = list(msg.count)
        self.current_item_index = 0
        self.cnt = 0
        self.processing_order = True

    @torch.no_grad()
    def main_loop(self):
        if not self.processing_order:
            return
        if self.current_item_index >= len(self.target_items):
            return

        item_name = self.target_items[self.current_item_index]
        target_count = int(self.target_counts[self.current_item_index])

        for _ in range(5):  # 버퍼에 쌓인 오래된 프레임 날리기
            self.cap.read()

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("카메라 프레임 읽기 실패")
            return

        # YOLO 추론
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.model(img_rgb)
        detections = results.xyxy[0].cpu().numpy()

        # 현재 목표 아이템만 필터링
        picked_boxes = []
        for *box, conf, cls in detections:
            label_name = self.model.names[int(cls)]
            allowed_labels = self.LABEL_ALIAS.get(item_name, [item_name])
            if label_name not in allowed_labels:
                continue
            picked_boxes.append((box, float(conf), label_name))

        if not picked_boxes:
            return  # 이번 프레임에 목표 물체 없음

        # 여러 박스를 순서대로 처리
        for (x1, y1, x2, y2), conf, label_name in picked_boxes:
            if self.cnt >= target_count:
                break  # 목표 수량 채우면 중단

            x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

            # 시각화 & 저장
            vis = frame.copy()
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(vis, f"{label_name} {conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            ts = time.strftime("%Y%m%d_%H%M%S")
            save_path = os.path.join(self.SAVE_DIR, f"detected_{label_name}_{ts}.jpg")
            try:
                cv2.imwrite(save_path, vis)
            except Exception as e:
                self.get_logger().warn(f"감지 이미지 저장 실패: {e}")

            # 픽셀 -> 실좌표 변환
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
            box_w = max(1, x2 - x1)
            cz = (self.f * self.W) / float(box_w)
            pixel = np.array([[[cx, cy]]], dtype=np.float32)
            real = cv2.perspectiveTransform(pixel, self.H)
            X_mm, Y_mm = float(real[0][0][0]) - 20.0, float(real[0][0][1]) + 20.0

            self.get_logger().info(f"[INFO] 실좌표: X:{X_mm:.1f}, Y:{Y_mm:.1f}, Z:{cz:.1f}")

            # 집기 좌표 계산
            coords = self.mc.get_coords()
            goal_coords = coords.copy()

            if (Y_mm > 50 and X_mm > -30) or (Y_mm > 40 and X_mm < -29):
                goal_coords[2] = 350
                goal_coords[0] = -15
                if (X_mm < -35):
                    goal_coords[1] -= X_mm + 10
                    print("1")
                elif (X_mm < -26) :
                    goal_coords[1] -= X_mm + 15
                    print("2")
                elif (X_mm < 6) :
                    goal_coords[1] -= X_mm + 20
                    print("3")
                elif (X_mm < 30):
                    goal_coords[1] -= X_mm + 12
                    print("4")
                else :
                    goal_coords[1] -= X_mm + 3
                    print("5")

            else:
                goal_coords[2] = 385
                goal_coords[0] = -15
                if (X_mm < - 53):
                    goal_coords[1] -= X_mm + 18
                    print("1")
                elif (X_mm > -58 and X_mm < -42) :
                    goal_coords[1] -= X_mm + 30
                    print("2")
                elif (X_mm < 5) :
                    goal_coords[1] -= X_mm + 35
                    print("3")
                elif (X_mm < 14) :
                    goal_coords[1] -= X_mm + 21
                    print("4")
                else :
                    goal_coords[1] -= X_mm + 17
                    print("5")

            self.cnt += 1     

            # 로봇팔 동작
            self.mc.send_coords(goal_coords, 70, 1)
            time.sleep(1)
            goal_coords[0] += 5
            self.mc.send_coords(goal_coords, 70, 1)
            goal_coords[0] += 10
            self.mc.send_coords(goal_coords, 70, 1)
            # time.sleep(0.5)
            self.mc.set_gripper_value(0, 100)
            # if (Y_mm > 50 and X_mm > -30) or (Y_mm > 40 and X_mm < -29):
            goal_coords[0] -= 20
            self.mc.send_coords(goal_coords, 50, 1)
            goal_coords[0] -= 20
            self.mc.send_coords(goal_coords, 50, 1)
            self.mc.send_angles(self.home_angles, 70)
            # time.sleep(0.3)
            self.mc.send_coords([-105.0, 109.2, 318.6, -76.34, -49.66, -137.43], 100, 1)
            # time.sleep(0.5)
            self.mc.send_coords([-238.5, 95.2, 208.4, -164.79, -26.14, -163.07], 100, 1)
            # time.sleep(0.3)
            self.mc.send_coords([-219.7, -132.6, 226.3, -156.61, -19.75, -144.94], 20, 1)
            time.sleep(1.5)
            self.mc.set_gripper_value(70, 100)
            self.mc.send_coords([-197.2, -142.0, 274.6, -168.54, -38.25, -132.49], 100, 1)
            # time.sleep(0.5)
            # 수량 카운트            
            self.get_logger().info(f"[INFO] {item_name} 처리 수량: {self.cnt}/{target_count}")

            # 목표 수량 채우면 다음 아이템
            if self.cnt >= target_count:
                self.cnt = 0
                self.current_item_index += 1
                if self.current_item_index >= len(self.target_items):
                    done_msg = Bool()
                    done_msg.data = True
                    self.publisher_done.publish(done_msg)
                    self.get_logger().info("[INFO] 모든 주문 완료. Bool True 전송.")
                    self.processing_order = False

            self.mc.send_angles(self.home_angles, 100)
            time.sleep(1)
            
            
    def destroy_node(self):
        try:
            if self.cap and self.cap.isOpened():
                self.cap.release()
        finally:
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
