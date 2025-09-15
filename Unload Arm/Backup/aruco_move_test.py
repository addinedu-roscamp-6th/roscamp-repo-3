#!/usr/bin/env python3
"""
JetCobot Aruco Pick-&-Place (v3 – offset 지원)
──────────────────────────────────────────────
1. 카메라에서 ArUco(5×5_250, 65 mm) 검출
2. tvec(camera) → T(base) = Extr · tvec  ↷ mm
3. 오프셋 적용 후
   (x, y, z+APPROACH_Z) → (x, y, z+GRIPPER_ZOFF) → (x, y, z+RETREAT_Z)
4. 그리퍼 열기 → 닫기
"""
import cv2
import numpy as np
import time
from pymycobot.mycobot280 import MyCobot280
# ─────────────── 사용자 설정 ─────────────── #
CAM_ID        = '/dev/video0'                                   # /dev/video0
CALIB_PATH    = "/home/jetcobot/project/calib/src/scanning/config/camera_calib_result.npz"
EXTR_PATH     = "/home/jetcobot/project/calib/src/scanning/config/extrinsic_cam2base.npz"
MARKER_SIZE   = 65.0                                # [mm]
TARGET_ID     = None                                # None이면 첫 마커 집음
APPROACH_Z    = 50.0   # 물체 위에서 대기 높이  +50 mm
GRIPPER_ZOFF  = -10.0  # 실제 집는 깊이       -10 mm
RETREAT_Z     = 80.0   # 집은 뒤 올라갈 높이   +80 mm
ROBOT_PORT    = "/dev/ttyJETCOBOT"
ROBOT_BAUD    = 1_000_000
PICK_SPEED    = 50      # [%] 0-100
HOLD_MS       = 1000    # [ms] 그리퍼 조작 간 대기
# 고정 자세 (도/deg) : 평평하게 내려찍는 드릴링 자세 예시
DEFAULT_ANGLES = [180, 0, 180]  # Rx, Ry, Rz in degree
# ─────────────────────────────────────────── #
def load_matrices():
    calib = np.load(CALIB_PATH)
    K, dist = calib["K"], calib["dist"]
    extr = np.load(EXTR_PATH)
    T_cb = extr["T"] if "T" in extr.files else extr["extr"]  # 4×4 [base←cam]
    return K, dist, T_cb
def cam_to_base(tvec_cam, T_cb):
    """3×1 camera 좌표(mm) → 3×1 base 좌표(mm)"""
    v = np.append(tvec_cam, 1.0)          # (x y z 1)^T
    v_b = T_cb @ v                        # homogeneous 변환
    return v_b[:3]
def open_gripper(mc):
    mc.set_gripper_value(100, PICK_SPEED)  # 완전 개방
    time.sleep(HOLD_MS/1000)
def close_gripper(mc):
    mc.set_gripper_value(0, PICK_SPEED)    # 완전 폐쇄
    time.sleep(HOLD_MS/1000)
def pick_sequence(mc, xyz):
    """xyz: base 좌표계(mm) – 물체 중심"""
    x, y, z = xyz
    approach = [x, y, z + APPROACH_Z, *DEFAULT_ANGLES]
    grip     = [x, y, z + GRIPPER_ZOFF, *DEFAULT_ANGLES]
    retreat  = [x, y, z + RETREAT_Z, *DEFAULT_ANGLES]
    mc.send_coords(approach, PICK_SPEED, 0)   # 1) 위에서 대기
    time.sleep(1.5)
    mc.send_coords(grip, PICK_SPEED, 0)       # 2) 내려가서
    time.sleep(1.0)
    close_gripper(mc)                         #    집고
    mc.send_coords(retreat, PICK_SPEED, 0)    # 3) 위로 탈출
    time.sleep(1.5)
def main():
    # 1. 카메라 & ArUco 설정
    cap = cv2.VideoCapture(CAM_ID)
    if not cap.isOpened():
        raise RuntimeError("카메라를 열 수 없습니다!")
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
    params = cv2.aruco.DetectorParameters()
    K, dist, T_cb = load_matrices()
    # 2. 로봇 연결
    mc = MyCobot280(ROBOT_PORT, ROBOT_BAUD)
    mc.thread_lock = True
    print("[JetCobot] 연결 완료")
    open_gripper(mc)                 # 출발 전 그리퍼 열기
    picked = False
    while not picked:
        ok, frame = cap.read()
        if not ok:
            continue
        corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=params)
        if ids is None:
            cv2.imshow("Search ArUco", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue
        # 원하는 마커 선택
        idx = 0
        if TARGET_ID is not None:
            match = np.where(ids.flatten() == TARGET_ID)[0]
            if len(match) == 0:
                continue
            idx = match[0]
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, MARKER_SIZE, K, dist
        )
        tvec = tvecs[idx][0]                  # (x,y,z)[mm] in cam frame
        xyz_base = cam_to_base(tvec, T_cb)    # base 좌표로 변환
        print(f"[INFO] target @ base {xyz_base.round(1)} mm")
        # 3. 집기 시퀀스
        pick_sequence(mc, xyz_base)
        picked = True
    # 4. 종료 처리
    cap.release()
    cv2.destroyAllWindows()
    print("완료 – 종료합니다.")
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("사용자 중단으로 종료")