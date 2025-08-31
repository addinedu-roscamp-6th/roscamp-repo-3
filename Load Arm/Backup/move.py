#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
from pymycobot.mycobot280 import MyCobot280
from std_msgs.msg import Int32

PORT = '/dev/ttyJETCOBOT'
BAUD = 1000000
CALIB_PATH = '/home/jetcobot/project/calib/calibration.npz'
CAMERA_DEV = '/dev/jetcocam0'
MARKER_LENGTH = 20  # mm


class ArucoRobotNode(Node):
    def __init__(self):
        super().__init__('aruco_robot_node')
        self.publisher_ = self.create_publisher(Int32, "shelf_to_pinky_cnt", 10)
        self.mc = MyCobot280(PORT, BAUD)
        self.calib = np.load(CALIB_PATH)
        self.K = self.calib['K']
        self.dist = self.calib['dist']
        self.cap = cv2.VideoCapture(CAMERA_DEV)
        self.cap.set(cv2.CAP_PROP_FPS, 15)
        self.ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.ARUCO_PARAMS = cv2.aruco.DetectorParameters()
        self.cnt = 0
        self.prev_marker_found = False  # 이전에 마커가 있었는지 저장

        print("[INFO] 아루코 마커 감지 중. Ctrl+C로 종료하려면 rqt_console에서 종료")
        # 최초 한 번만 실행되는 준비 동작
        self.mc.send_angles([-0.43, 51.94, 0.43, -52.03, 11.07, -45.61], 50)
        self.mc.set_gripper_value(100, 50)
        self.coords = self.mc.get_coords()
        print("[INFO] 현재 로봇팔 좌표:", self.coords)

        # 타이머 시작 (0.2초마다 callback)
        self.timer = self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        msg = Int32()
        ret, frame = self.cap.read()
        if not ret:
            print("[WARN] 카메라 프레임 읽기 실패")
            return

        corners, ids, _ = cv2.aruco.detectMarkers(
            frame, self.ARUCO_DICT, parameters=self.ARUCO_PARAMS
        )
        marker_found = (ids is not None and len(ids) > 0)

        if marker_found and not self.prev_marker_found:
            # === 이전에는 마커가 없었고, 이번에 새로 마커가 감지된 경우만 동작 ===
            print("[INFO] 새 아루코 마커 감지, 작업 시작")

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, MARKER_LENGTH, self.K, self.dist
            )

            for i in range(len(ids)):
                tvec = tvecs[i][0]
                print(f"[INFO] 아루코 마커 좌표: id:{ids[i][0]}, {tvec[2], tvec[0], tvec[1]}")
                # 카메라 x/y/z → 로봇 y/z/x 순서로 변환
                goal_coords = self.coords.copy()
                if tvec[0] < 290:
                    tvec[0] = tvec[0] + 18
                    tvec[1] = tvec[1] - 8
                elif tvec[0] > 300: 
                    tvec[1] = tvec[1] - 10
                    tvec[0] = tvec[0] - 20
                goal_coords[1] = goal_coords[1] - tvec[0] - 15   # x
                goal_coords[2] = goal_coords[2] - tvec[1] + 50   # y
                goal_coords[0] = goal_coords[0] + tvec[2] - 80  # z
                self.mc.send_coords(goal_coords, 50, 1)
                time.sleep(2)
                goal_coords[0] = goal_coords[0] + 20
                self.mc.send_coords(goal_coords, 30, 1)
                time.sleep(1)
                self.mc.set_gripper_value(0, 50)
                print(f"[INFO] 이동할 로봇팔 좌표: {self.coords}")

                self.mc.send_angles([-0.43, 51.94, 0.43, -52.03, 11.07, -45.61], 50)

                # id별 추가 작업
                if ids[i][0] == 1:
                    self.mc.send_coords([-122.7, -182.2, 173.7, -168.87, 4.93, -125.46], 30, 1)
                elif ids[i][0] == 2:
                    self.mc.send_coords([4.4, -233.9, 187.8, -171.06, 15.73, -128.12], 30, 1)
                elif ids[i][0] == 3:
                    self.mc.send_coords([110.7, -114.9, 275.1, -94.64, -39.35, -69.13], 30, 1)
                elif ids[i][0] == 4:
                    self.mc.send_coords([122.5, -29.8, 259.1, -95.48, -39.84, -67.22], 30, 1)
                time.sleep(2)
                self.mc.set_gripper_value(100, 50)
                self.mc.send_angles([-0.43, 51.94, 0.43, -52.03, 11.07, -45.61], 50)

            # 카운트 증가
            self.cnt += 1
            print(f"[INFO] 작업 완료 {self.cnt}번")
            msg.data = self.cnt
            self.publisher_.publish(msg)
        elif marker_found:
            # 이미 마커가 있었고 계속 보임 -> 대기
            pass
        else:
            # 마커가 안 보이면 원위치 복귀
            self.mc.send_angles([-0.43, 51.94, 0.43, -52.03, 11.07, -45.61], 50)
            self.mc.set_gripper_value(100, 50)

        # 현재 프레임의 마커 감지 상태 저장 (다음 콜백에서 사용)
        self.prev_marker_found = marker_found

        # 항상 좌표 갱신
        self.coords = self.mc.get_coords()

    def destroy_node(self):
        self.cap.release()
        print("[INFO] 카메라 해제 및 프로그램 종료")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[INFO] 종료 요청됨 (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
