#!/usr/bin/env python3
"""
collect_handeye_data.py
────────────────────────
JetCobot을 랜덤 자세로 12번 움직이며
로봇 pose & 카메라 pose 쌍을 CSV로 저장.
"""
import cv2, numpy as np, csv, time, random
from pymycobot.mycobot280 import MyCobot280
# ── 설정 ───────────────────────────
PORT        = "/dev/ttyJETCOBOT"
BAUD        = 1_000_000
MARKER_SIZE = 35.0                 # [mm]
OUT_ROBOT   = "robot_poses.csv"
OUT_CAM     = "cam_poses.csv"
SAMPLES     = 15
# ──────────────────────────────────
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
params     = cv2.aruco.DetectorParameters()

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

# 1. 장비 초기화
cap = cv2.VideoCapture("/dev/jetcocam0")
if not cap.isOpened():
    raise RuntimeError("카메라 열기 실패")
mc = MyCobot280(PORT, BAUD)
mc.thread_lock = True
time.sleep(2)
# 2. 캘리브레이션 매트릭스(K, dist) 미리 로드
CALIB_PATH = "/home/jetcobot/Desktop/test/cam2base/calibration.npz"
data = np.load(CALIB_PATH)
K, dist = data["K"], data["dist"]
K, dist = np.load("calibration.npz")["K"], np.load("calibration.npz")["dist"]
# 3. 수집 루프
robot_rows, cam_rows = [], []

for i in range(SAMPLES):
    # 로봇을 임의 위치로 (안전범위 고려!):
    pose = [ random.uniform(170,270),  # x
             random.uniform(-50,-10),  # y
             random.uniform(150,270),  # z
             -161.23, -14.97, -36.5 ]            # Rx,Ry,Rz (deg) : 마커가 카메라 보게
    mc.send_coords(pose, 40, 0)
    time.sleep(2.0)
    # 로봇 실제 좌표 읽기
    robot_pose = mc.get_coords()      # x y z Rx Ry Rz
    robot_rows.append(robot_pose)
    # 카메라로 한 프레임 캡처 → ArUco
    ok, frame = cap.read()
    # 화면에 출력
    cv2.imshow('Cam', frame)
    if not ok:
        continue
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    corners, ids, _ = detector.detectMarkers(frame)
    if ids is None:
        print("[WARN] ArUco 미검출, 샘플 스킵")
        continue
    rvecs, tvecs = estimate_pose_single(corners, MARKER_SIZE, K, dist)
    rv, tv = rvecs[0], tvecs[0]
    cam_rows.append(np.concatenate([rv, tv]))  # rvec_x,y,z,tvec_x,y,z
    print(f"[{i+1}/{SAMPLES}] 수집 완료")
# 4. CSV 저장
with open(OUT_ROBOT, "w", newline="") as f:
    wr = csv.writer(f)
    wr.writerow(["x","y","z","Rx","Ry","Rz"])         # deg
    wr.writerows(robot_rows)
with open(OUT_CAM, "w", newline="") as f:
    wr = csv.writer(f)
    wr.writerow(["rvx","rvy","rvz","tvx","tvy","tvz"])  # rad, mm
    wr.writerows(cam_rows)
cap.release()
print(":흰색_확인_표시: 데이터 수집 완료")