import cv2, numpy as np, time
from pymycobot.mycobot280 import MyCobot280

PORT, BAUD      = '/dev/ttyJETCOBOT', 1_000_000
CAMERA_DEV      = '/dev/jetcocam0'
CALIB_PATH      = '/home/jetcobot/project/calib/src/cam2base/calibration.npz'
EXTR_PATH       = '/home/jetcobot/project/calib/src/cam2base/extrinsic_cam2base.npz'   # ★ 추가
MARKER_LENGTH   = 35.0      # mm

# ─── MyCobot 연결 ───────────────────────────────────────────
mc = MyCobot280(PORT, BAUD)
mc.thread_lock = True

# ─── 카메라 내부 파라미터 + Hand-Eye 행렬 로드 ─────────────
calib = np.load(CALIB_PATH)
K    = calib['K'].astype(np.float64).reshape(3,3)
dist = calib['dist'].astype(np.float64).flatten()

T_cb = np.load(EXTR_PATH)['T']          # 4×4 (base ← cam)

# ─── 카메라 & ArUco 설정 ──────────────────────────────────
cap = cv2.VideoCapture(CAMERA_DEV)
dict5x5  = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
params   = cv2.aruco.DetectorParameters()

print("[INFO] ArUco 감지 시작 (Ctrl+C 종료)")
try:
    home = [172.8, -56.1, 260, -171.07, 0.93, -45.1]
    mc.send_angles(home, 50)
    mc.set_gripper_value(100, 50)
    base_6d = mc.get_coords()           # [x y z Rx Ry Rz]

    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.1); continue

        corners, ids, _ = cv2.aruco.detectMarkers(frame, dict5x5, parameters=params)
        if ids is None:                 # 마커 없음 → 다음 프레임
            continue

        # pose 추정
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_LENGTH, K, dist)
        tvec_cam = tvecs[0][0]          # (x,y,z) in camera frame [mm]

        # ─── Hand-Eye 변환 cam→base ────────────────────────
        v_cam  = np.append(tvec_cam, 1.0)        # (x y z 1)^T
        v_base = T_cb @ v_cam                    # homogeneous
        xyz_b  = v_base[:3]
        print(f"[INFO] base 좌표(mm): {xyz_b.round(1)}")

        # 원하는 오프셋만 더해 목표 구성
        goal_xyz = xyz_b + np.array([0, 0, GRIPPER_ZOFF])
        full_goal = np.hstack([goal_xyz, base_6d[3:]])   # 자세는 그대로
        mc.send_coords(full_goal.tolist(), 50, 1)
        mc.set_gripper_value(0, 50)
        time.sleep(2)
        mc.send_angles(home, 50)
        break

except KeyboardInterrupt:
    print("[INFO] 사용자 종료")
finally:
    cap.release()
    print("[INFO] 종료 완료")
