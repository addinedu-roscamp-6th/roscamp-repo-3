from pymycobot.mycobot280 import MyCobot280
import cv2, time, numpy as np

# ---- 설정 ----
MARKER_LENGTH = 20      # mm
SPEED_APPROACH = 25
SPEED_GRASP   = 8
HOME_coords   = [156.7, -43.0, 295.5, -161.23, -14.97, -36.5]
DEFAULT_RPY   = [-179.04, 3.54, -45.36]   # roll, pitch, yaw in deg
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()

# ---- 초기화 ----
mc = MyCobot280('/dev/ttyJETCOBOT', 1_000_000)
mc.send_coords(HOME_coords, 50)
mc.set_gripper_value(100, 50)

# ---- 보정 데이터 ----
calib = np.load('calibration.npz')
K, dist = calib['K'], calib['dist']
# ---- Eye-in-Hand extrinsic ----
tool_ex = np.load("extrinsic_cam2tool.npz")          # cam → tool
T_cam2tool = tool_ex["T_cam2tool"]
#T_tool2cam = np.linalg.inv(T_cam2tool)               # tool → cam   (고정)

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

def rpy_to_R(rx, ry, rz):
    """Roll-Pitch-Yaw(deg) → 회전행렬 (OpenCV용)"""
    r, p, y = np.deg2rad([rx, ry, rz])
    Rx = np.array([[1,0,0],
                   [0,np.cos(r),-np.sin(r)],
                   [0,np.sin(r), np.cos(r)]])
    Ry = np.array([[ np.cos(p),0,np.sin(p)],
                   [0,1,0],
                   [-np.sin(p),0,np.cos(p)]])
    Rz = np.array([[np.cos(y),-np.sin(y),0],
                   [np.sin(y), np.cos(y),0],
                   [0,0,1]])
    return Rz @ Ry @ Rx           # (Z → Y → X, 즉 Roll→Pitch→Yaw 순차 적용) #Rx @ Ry @ Rz , X → Y → Z

def tool_pose_to_T(coords):
    """MyCobot get_coords() → T_tool2base (4×4)"""
    x,y,z, rx,ry,rz = coords
    R = rpy_to_R(rx,ry,rz)
    T = np.eye(4)
    T[:3,:3] = R
    T[:3, 3] = [x,y,z]
    return T

cap = cv2.VideoCapture('/dev/jetcocam0')

print("[INFO] 아루코 마커 감지 중. Ctrl+C로 종료하세요.")

print("[INFO] 현재 로봇팔 좌표:", HOME_coords)

detector = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

T_tool2base = tool_pose_to_T(mc.get_coords())

# ── [★] 한 번만: 툴 내부축 보정 행렬 F 생성 ──
T_tool2base = tool_pose_to_T(mc.get_coords())        # 홈 자세
R_cb        = (T_tool2base @ T_cam2tool)[:3,:3]      # 카메라→베이스 회전
R_desired   = np.diag([1, 1, 1])                    # X→X, Y→Y, Z→−Z
F           = R_cb.T @ R_desired                     # 3×3  (**핵심 식**)

T_fix       = np.eye(4);  T_fix[:3,:3] = F
T_cam2tool  = T_cam2tool @ T_fix                     # ← **오른쪽**에 곱해 덮어쓰기
# ─────────────────────────────────────────
# 루프 앞쪽에서 한 번
R_test = (tool_pose_to_T(mc.get_coords()) @ T_cam2tool)[:3,:3]
print(R_test.round(3))
# 예상 출력 (±0.01):
# [[ 1.  0.  0.]
#  [ 0.  1.  0.]
#  [ 0.  0. -1.]]

cam_origin  = (T_tool2base @ T_cam2tool)[:3,3]
print("카메라 원점(Base):", cam_origin)

try:
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

        corners, ids, _ = detector.detectMarkers(frame)
        if ids is None: continue

        #  ↓↓↓ 기존 while True 안 ↓↓↓

        # 매 프레임 현재 그리퍼 pose 갱신
        tool_coords   = mc.get_coords()          # base 기준 x,y,z,rx,ry,rz
        T_tool2base   = tool_pose_to_T(tool_coords)
        T_cam2base    = T_tool2base @ T_cam2tool # (고정) tool→cam 붙이기

        R_cam2base    = T_cam2base[:3,:3]
        t_cam2base    = T_cam2base[:3, 3]

        z_cam_in_base = R_cam2base @ np.array([0,0,1])
        print("카메라 Z축(Base기준):", z_cam_in_base)

        rvecs, tvecs = estimate_pose_single(corners, MARKER_LENGTH, K, dist)
        tvec = tvecs[0].flatten()             # 첫번째 마커
        pos_base = R_cam2base @ tvec + t_cam2base             # (x,y,z) mm
        print(f"[INFO] 아루코 마커 좌표(카메라 기준, mm): {tvec}")
        print(f"[INFO] pos_base: {pos_base}")
        time.sleep(10)
        GRASP_OFFSET = np.array([+140,   # X 앞쪽 14 cm
                          +8,    # Y 왼쪽 8 mm
                          -152]) # Z 아래 152 mm
        grasp_pos    = pos_base + GRASP_OFFSET
        approach_pos = grasp_pos + np.array([0, 0, 50])
        print(f"[INFO] 접근 포즈: {approach_pos}")
        time.sleep(10)

        approach = pos_base + np.array([0,0,50])
        for target, spd in [(approach,SPEED_APPROACH), (pos_base,SPEED_GRASP)]:
            goal = np.hstack([target, DEFAULT_RPY])
            print(f"[INFO] goal: {goal}")
            time.sleep(5)
            mc.send_coords(goal.tolist(), spd, 1);  mc.wait()

        mc.set_gripper_value(0, 50)           # Grip
        time.sleep(0.5)

        # 후처리(상승 → 드랍 → 복귀) ...
        break
except KeyboardInterrupt:
    print("[INFO] 종료 요청됨 (Ctrl+C)")
finally:
    cap.release()
    print("[INFO] 카메라 해제 및 프로그램 종료")