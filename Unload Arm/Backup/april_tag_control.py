import cv2
import numpy as np
import time
from pymycobot.mycobot280 import MyCobot280
PORT = '/dev/ttyJETCOBOT'
BAUD = 1000000
CALIB_PATH = '/home/jetcobot/Desktop/test/cam2base/calibration.npz'
CAMERA_DEV = '/dev/jetcocam0'
HOME_coords   = [156.7, -43.0, 295.5, -161.23, -14.97, -36.5]
MARKER_LENGTH = 20  # mm
speed = 20
mc = MyCobot280(PORT, BAUD)
calib = np.load(CALIB_PATH)
K = calib['K']
dist = calib['dist']
cap = cv2.VideoCapture(CAMERA_DEV)
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()

def estimate_pose_single(corners_list, marker_len, K, dist):
    """solvePnP 기반 1-마커 pose 추정 (OpenCV ≥ 4.7)"""
    # 마커 4 모서리 3-D 좌표 (평면 Z=0)
    objp = np.array([
        [-marker_len/2,  marker_len/2, 0],
        [ marker_len/2,  marker_len/2, 0],
        [ marker_len/2, -marker_len/2, 0],
        [-marker_len/2, -marker_len/2, 0]
    ], dtype=np.float32)

    rvecs, tvecs = [], []
    for c in corners_list:                          # 각 마커마다
        ok, rvec, tvec = cv2.solvePnP(
            objp, c.squeeze().astype(np.float32),
            K, dist,
            flags=cv2.SOLVEPNP_IPPE_SQUARE         # 사각 태그 특화
        )
        if ok:
            rvecs.append(rvec)
            tvecs.append(tvec)
    return rvecs, tvecs

print("[INFO] 아루코 마커 감지 중. Ctrl+C로 종료하세요.")

try:
    #로봇을 먼저 초기 위치로 리셋------------------------------------
    mc.send_coords(HOME_coords, speed)
    mc.set_gripper_value(100, 50)
    base_coords = mc.get_coords()
    print("[INFO] 현재 로봇팔 좌표:", base_coords)
    #-----------------------------------------------------------
    while True:
        ok, frame = cap.read()
        # 화면에 출력
        cv2.imshow('Cam', frame)

        cv2.imwrite("/tmp/aruco_debug.jpg", frame) ##
        print("[DBG] saved /tmp/aruco_debug.jpg") ##
        if not ok:
            print("[DBG] frame grab failed")
            print("[WARN] 카메라 프레임 읽기 실패")
            time.sleep(0.1);  continue

        detector = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)
        corners, ids, rejected = detector.detectMarkers(frame)
        print("[DBG] ids:", ids)
        if ids is None:               # 마커 미검출
            print("마커가 미검출되었습니다.")
            continue

        if ids is not None and len(ids) > 0:
            rvecs, tvecs = estimate_pose_single(corners, MARKER_LENGTH, K, dist)
            tvec = tvecs[0].flatten()
            print(f"[INFO] 아루코 마커 좌표(카메라 기준, mm): {tvec}")
            # 카메라 x/y/z → 로봇 y/z/x 순서로 변환
            goal_coords = [0, 0, 0]
            goal_coords[0] = base_coords[0] + tvec[2] - 25  # z
            #goal_coords[1] = base_coords[1] + (tvec[0]/3) + 20  # x
            goal_coords[1] = base_coords[1] - tvec[0]  -0# x
            goal_coords[2] = base_coords[2] + tvec[1]  -0 # y
            # 자세는 base_coords 그대로 사용
            if len(base_coords) == 6:
                full_goal_coords = np.hstack([goal_coords, base_coords[3:]])
            else:
                full_goal_coords = goal_coords
            print(f"[INFO] 이동할 로봇팔 좌표: {full_goal_coords}")
            mc.send_coords(full_goal_coords.tolist(), 50, 1)
            mc.set_gripper_value(0, 50)
            time.sleep(2)
            #mc.send_angles([-0.43, 51.94, 0.43, -52.03, 11.07, -45.61], 50)
            # === 여기서 루프를 빠져나감 ===
            break
        # 아루코가 안 보이면 계속 원위치 반복 (생략해도 무방)
        #mc.send_angles([-0.43, 51.94, 0.43, -52.03, 11.07, -45.61], 50)
        #time.sleep(1)
except KeyboardInterrupt:
    print("[INFO] 종료 요청됨 (Ctrl+C)")
finally:
    cap.release()
    print("[INFO] 카메라 해제 및 프로그램 종료")