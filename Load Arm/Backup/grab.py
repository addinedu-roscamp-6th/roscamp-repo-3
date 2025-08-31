#!/usr/bin/env python3
import cv2
import torch
import numpy as np
import time
from pymycobot.mycobot280 import MyCobot280

class YoloPickRobot:
    def __init__(self):
        print("[INFO] YOLO Pick Robot 시작")

        # 로봇팔 초기화
        self.mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
        self.mc.set_gripper_value(100, 50)
        self.home_angles = [-0.43, 51.94, 0.43, -52.03, 11.07, -45.61]

        # 초기 대기 자세
        self.mc.send_coords([-244.8, -87.1, 209.0, -152.12, 10.83, 142.16], 50, 1)

        # 카메라 초기화
        self.cap = cv2.VideoCapture('/dev/jetcocam0')
        self.cap.set(cv2.CAP_PROP_FPS, 15)

        # Calibration 로드
        calib = np.load('/home/jetcobot/project/calib/calibration.npz')
        self.K = calib['K']
        self.f = self.K[0, 0]
        self.W = 30  # mm

        # Homography 설정
        points_world = np.array([[15.8, 51.4], [-32.9, 22.9], [-45.7, -20.9], [27.8, -20.4]], dtype=np.float32)
        points_img = np.array([[370, 395], [22, 388], [200, 93], [488, 104]], dtype=np.float32)
        self.H, _ = cv2.findHomography(points_img, points_world)

        # YOLO 모델 로드
        self.model = torch.hub.load(
            '/home/jetcobot/project/calib/src/yolov5',
            'custom',
            path='/home/jetcobot/project/calib/src/yolov5/runs/train/exp15/weights/best.pt',
            source='local'
        )

    def run_once(self):
        # 프레임 읽기
        ret, frame = self.cap.read()
        if not ret:
            print("[WARN] 카메라 프레임 읽기 실패")
            return

        # YOLO 감지
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.model(img_rgb)

        # 감지 결과 없으면 종료
        if len(results.xyxy[0]) == 0:
            print("[INFO] 감지된 물체 없음. 종료")
            return

        # 첫 번째 감지된 객체 사용
        x1, y1, x2, y2, conf, cls = results.xyxy[0][0].cpu().numpy()
        label_name = self.model.names[int(cls)]
        print(f"[INFO] 감지됨: {label_name} (conf={conf:.2f})")

        # 바운딩 박스 표시 후 저장
        x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label_text = f"{label_name} {conf:.2f}"
        cv2.putText(frame, label_text, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        save_path = f"./detected_{label_name}.jpg"
        cv2.imwrite(save_path, frame)
        print(f"[INFO] 바운딩박스 이미지 저장: {save_path}")

        # 좌표 계산
        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
        box_w = x2 - x1
        cz = (self.f * self.W) / box_w

        pixel = np.array([[[cx, cy]]], dtype='float32')
        real = cv2.perspectiveTransform(pixel, self.H)
        X_mm, Y_mm = real[0][0]
        X_mm -= 20
        Y_mm += 20

        # 집기 동작
        coords = self.mc.get_coords()
        goal_coords = coords.copy()
        goal_coords[1] += X_mm + 25
        goal_coords[2] += Y_mm - 100
        goal_coords[0] += cz - 120
        self.mc.send_coords(goal_coords, 50, 1)
        time.sleep(2)

        goal_coords[0] -= 30
        self.mc.send_coords(goal_coords, 30, 1)
        time.sleep(1)

        self.mc.set_gripper_value(0, 50)  # 잡기
        time.sleep(1)

        self.mc.send_coords([-244.8, -87.1, 209.0, -152.12, 10.83, 142.16], 50, 1)
        self.mc.set_gripper_value(100, 50)  # 놓기
        time.sleep(1)

        print("[INFO] 집기 완료")

    def __del__(self):
        self.cap.release()
        print("[INFO] 카메라 해제")

if __name__ == '__main__':
    robot = YoloPickRobot()
    robot.run_once()
