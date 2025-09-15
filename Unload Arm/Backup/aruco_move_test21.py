#!/usr/bin/env python3
"""
JetCobot + ArUco 1-Shot Pick & Place
(OpenCV ≥ 4.10 / pymycobot ≥ 0.2 호환)
"""
import math
import cv2, numpy as np, time
from pymycobot.mycobot280 import MyCobot280

# ───────── 사용자 설정 ─────────
PORT, BAUD      = '/dev/ttyJETCOBOT', 1_000_000
CAMERA_DEV      = '/dev/video0'                       # 또는 '/dev/video0'
CALIB_PATH      = '/home/jetcobot/Desktop/test/cam2base/calibration.npz'
EXTR_PATH       = '/home/jetcobot/Desktop/test/cam2base/extrinsic_cam2base.npz'

MARKER_LENGTH   = 20.0         # mm (한 변)
GRIPPER_ZOFF    = 0        # mm  (마커 가운데보다 더 내려가는 깊이)
HOME_COORDS     = [156.7, -43.0, 295.5, -161.23, -14.97, -36.5]
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

# 1) Hand-Eye 결과 : EE ← Cam
T_ec = np.load('/home/jetcobot/Desktop/test/cam2base/handeye_EE_cam.npz')['T'].copy()      # 4×4
T_ec[:3, 3] *= 1000.0
def coords_to_matrix(coords):
    """
    coords : [x, y, z, rx, ry, rz]  (mm, deg)   ← mc.get_coords() 결과와 동일
    반환   : 4×4  homogeneous matrix  (EE ← Base)
    """
    x, y, z, rx, ry, rz = coords

    # 1) 회전각(deg) → 라디안
    rx, ry, rz = map(math.radians, (rx, ry, rz))

    # 2) Z-Y-X 순서(=myCobot 기본)로 회전행렬 생성
    cz, sz = math.cos(rz), math.sin(rz)
    cy, sy = math.cos(ry), math.sin(ry)
    cx, sx = math.cos(rx), math.sin(rx)

    Rz = np.array([[ cz,-sz, 0],
                   [ sz, cz, 0],
                   [  0,  0, 1]])
    Ry = np.array([[ cy, 0, sy],
                   [  0, 1,  0],
                   [-sy, 0, cy]])
    Rx = np.array([[1,  0,   0],
                   [0, cx, -sx],
                   [0, sx,  cx]])

    R = Rz @ Ry @ Rx             # 최종 3×3
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3,  3] = [x, y, z]        # mm 그대로 (단위 통일 주의)

    return T

# 카메라 축 보정 (+X 유지, Y·Z 뒤집기)
Fix = np.eye(4)
Fix[:3,:3] = np.diag([1,-1,-1])
T_ec = T_ec @ Fix

def cam_to_base(tvec_cam):
    # 로봇이 현재 EE(툴) 포즈를 보고 → T_BE (4×4) 계산
    pose = mc.get_coords()                     # [x,y,z,rx,ry,rz]  (mm, deg)
    T_be = coords_to_matrix(pose)              # 직접 구현 필요
    #T_be  = np.linalg.inv(T_eb)        # Base ← EE   ←★ 역행렬 추가
    p_b   = (T_be @ T_ec @ np.append(tvec_cam, 1.0))[:3]
    return p_b

def wait_until_stop(dt=0.05):
    time.sleep(dt)
    while mc.is_moving():
        time.sleep(dt)

T_be = coords_to_matrix(mc.get_coords())

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

        print("tv_cam          :", tv.flatten())            # 카메라 좌표
        print("det(T_ec R)    :", np.linalg.det(T_ec[:3,:3]))
        print("T_ec trans [mm]:", T_ec[:3,3])
        print("pose (xyzdeg)  :", mc.get_coords())
        print("det(T_be R)    :", np.linalg.det(T_be[:3,:3]))
        time.sleep(3)

        # 3 집기 동작
        goal = np.hstack([xyz_base + np.array([0,0,GRIPPER_ZOFF]),
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
