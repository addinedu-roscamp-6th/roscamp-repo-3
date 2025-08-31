import cv2
import numpy as np
import time
from pymycobot.mycobot280 import MyCobot280

PORT = '/dev/ttyJETCOBOT'
BAUD = 1000000
CALIB_PATH = '/home/jetcobot/project/calib/calibration.npz'
CAMERA_DEV = '/dev/jetcocam0'
MARKER_LENGTH = 20  # mm

mc = MyCobot280(PORT, BAUD)
calib = np.load(CALIB_PATH)
K = calib['K']
dist = calib['dist']

cap = cv2.VideoCapture(CAMERA_DEV)
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()

print("[INFO] 아루코 마커 감지 중. Ctrl+C로 종료하세요.")

try:
    mc.send_angles([-0.43, 51.94, 0.43, -52.03, 11.07, -45.61], 50)
    mc.set_gripper_value(100, 50)
    coords = mc.get_coords()
    print("[INFO] 현재 로봇팔 좌표:", coords)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] 카메라 프레임 읽기 실패")
            time.sleep(0.1)
            continue

        corners, ids, _ = cv2.aruco.detectMarkers(frame, ARUCO_DICT, parameters=ARUCO_PARAMS)
        if ids is not None and len(ids) > 0:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, MARKER_LENGTH, K, dist
            )
            for i in range(len(ids)):
                tvec = tvecs[0][0]
                print(f"[INFO] 아루코 마커 좌표: id{ids[i][0]}, {tvec[2], tvec[0], tvec[1]}")

            # 카메라 x/y/z → 로봇 y/z/x 순서로 변환
            goal_coords = coords.copy()
            if tvec[0] > 280 : tevc[0] = tevc[0] + 5  
            goal_coords[1] = goal_coords[1] - tvec[0] - 20# x
            goal_coords[2] = goal_coords[2] - tvec[1] + 60 # y
            goal_coords[0] = goal_coords[0] + tvec[2] - 120 # z
            mc.send_coords(goal_coords, 50, 1)
            time.sleep(2)
            goal_coords[0] = goal_coords[0] + 60
            mc.send_coords(goal_coords, 30, 1)
            time.sleep(1)
            mc.set_gripper_value(0, 50)
            print(f"[INFO] 이동할 로봇팔 좌표: {coords}")
            

            mc.send_angles([-0.43, 51.94, 0.43, -52.03, 11.07, -45.61], 50)

            for i in range(len(ids)):
                if ids[i][0] == 1:
                    mc.send_coords([-122.7, -182.2, 173.7, -168.87, 4.93, -125.46], 30, 1)
                elif ids[i][0] == 2:
                    mc.send_coords([4.4, -233.9, 187.8, -171.06, 15.73, -128.12], 30, 1)
                elif ids[i][0] == 3:
                    mc.send_coords([110.7, -114.9, 275.1, -94.64, -39.35, -69.13], 30, 1)
                elif ids[i][0] == 4:
                    mc.send_coords([122.5, -29.8, 259.1, -95.48, -39.84, -67.22], 30, 1)

            time.sleep(1)
            mc.set_gripper_value(100, 50)
            mc.send_angles([-0.43, 51.94, 0.43, -52.03, 11.07, -45.61], 50)

            # === 여기서 루프를 빠져나감 ===
            break

        # 아루코가 안 보이면 계속 원위치 반복 (생략해도 무방)
        mc.send_angles([-0.43, 51.94, 0.43, -52.03, 11.07, -45.61], 50)
        mc.set_gripper_value(100, 50)
        time.sleep(1)

except KeyboardInterrupt:
    print("[INFO] 종료 요청됨 (Ctrl+C)")
finally:
    cap.release()
    print("[INFO] 카메라 해제 및 프로그램 종료")