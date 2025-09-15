#!/usr/bin/env python3
"""
JetCobot + ArUco 1-Shot Pick & Place
(OpenCV ≥ 4.10 / pymycobot ≥ 0.2 호환)
"""

import cv2, numpy as np, time
from pymycobot.mycobot280 import MyCobot280

# ───────── 사용자 설정 ─────────
PORT, BAUD      = '/dev/ttyJETCOBOT', 1_000_000
CAMERA_DEV      = '/dev/video0'                       # 또는 '/dev/video0'
CALIB_PATH      = '/home/jetcobot/Desktop/test/cam2base/calibration.npz'
EXTR_PATH       = '/home/jetcobot/Desktop/test/cam2base/extrinsic_cam2base.npz'

MARKER_LENGTH   = 20.0         # mm (한 변)
GRIPPER_ZOFF    = -70        # mm  (마커 가운데보다 더 내려가는 깊이)
HOME_COORDS     = [172.8, -56.1, 260, -171.07, 0.93, -45.1]
PICK_SPEED      = 50           # %
# ──────────────────────────────

# ─── MyCobot 연결 ───────────────────────────────────────────
mc = MyCobot280(PORT, BAUD);  mc.thread_lock = True

# ─── 파라미터 로드 ──────────────────────────────────────────
K, dist = (lambda d: (d["K"].astype(np.float64),
                      d["dist"].astype(np.float64)))(np.load(CALIB_PATH))
T_cb    = np.load(EXTR_PATH)["T"]          # 4×4 base ← cam

# ─── 카메라 & ArUco 설정 ──────────────────────────────────
params = cv2.aruco.DetectorParameters()
params.adaptiveThreshWinSizeMin = 3
params.adaptiveThreshWinSizeMax = 23
params.minMarkerPerimeterRate   = 0.02   # 기본 0.03
cap       = cv2.VideoCapture(CAMERA_DEV)
aru_dict  = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
detector  = cv2.aruco.ArucoDetector(aru_dict, params)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

def estimate_pose_one(corners):
    """OpenCV 4.10 빌드에 estimatePoseSingleMarkers 가 없는 경우 대비"""
    if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
        r, t = cv2.aruco.estimatePoseSingleMarkers(
                   [corners], MARKER_LENGTH, K, dist)[:2]
        return r[0], t[0]
    # fallback – solvePnP (IPPE_SQUARE)
    obj = np.array([[-MARKER_LENGTH/2,  MARKER_LENGTH/2, 0],
                    [ MARKER_LENGTH/2,  MARKER_LENGTH/2, 0],
                    [ MARKER_LENGTH/2, -MARKER_LENGTH/2, 0],
                    [-MARKER_LENGTH/2, -MARKER_LENGTH/2, 0]], np.float32)
    ok, rvec, tvec = cv2.solvePnP(
        obj, corners.squeeze().astype(np.float32),
        K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    return rvec, tvec

def cam_to_base(tvec_cam):
    return (T_cb @ np.append(tvec_cam, 1.0))[:3]

def wait_until_stop(dt=0.05):
    time.sleep(dt)
    while mc.is_moving():
        time.sleep(dt)

print("[INFO] ArUco 탐색 시작 (Ctrl-C 종료)")
try:
    # 1 홈 자세 복귀 & 그리퍼 open
    mc.send_coords(HOME_COORDS, PICK_SPEED, 0); wait_until_stop()
    mc.set_gripper_value(100, PICK_SPEED)        # open
    wait_until_stop()

    while True:
        ok, frame = cap.read()
        cv2.imwrite("/tmp/aruco_debug.jpg", frame) ##
        print("[DBG] saved /tmp/aruco_debug.jpg") ##
        if not ok:
            print("[DBG] frame grab failed")
            time.sleep(0.05);  continue

        corners, ids, _ = detector.detectMarkers(frame)
        print("[DBG] ids:", ids)
        if ids is None:               # 마커 미검출
            continue

        # 2 마커 1개만 사용
        rv, tv = estimate_pose_one(corners[0])
        xyz_base = cam_to_base(tv.flatten())
        print(f"[INFO] base 좌표(mm): {xyz_base.round(1)}")
        wait_until_stop(3)

        # 3 집기 동작
        goal = np.hstack([xyz_base + np.array([110,0,GRIPPER_ZOFF]),
                          HOME_COORDS[3:]])
        mc.send_coords(goal.tolist(), PICK_SPEED, 1); wait_until_stop()
        mc.set_gripper_value(0, PICK_SPEED);          wait_until_stop()

        # 4 홈 복귀
        mc.send_coords(HOME_COORDS, PICK_SPEED, 0);   wait_until_stop()
        mc.set_gripper_value(100, PICK_SPEED)
        break

except KeyboardInterrupt:
    print("[INFO] 사용자 종료")
finally:
    cap.release()
    print("[INFO] 프로그램 종료")
