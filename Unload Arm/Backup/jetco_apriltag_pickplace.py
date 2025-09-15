#!/usr/bin/env python3
"""
JetCobot + AprilTag(Aruco) 실시간 감지 → 로봇 이동(Pick) 통합 스크립트
=====================================================================

기능
----
1. 카메라 프레임 실시간 수신.
2. AprilTag (OpenCV ArUco 모듈) 검출 및 ID 오버레이.
3. solvePnP(IPPE_SQUARE) 기반 마커 6D 포즈 추정.
4. tvec(camera) → 로봇 베이스 기준 좌표 근사 변환 (사용자 제공 휴리스틱).
5. JetCobot(MyCobot280) 이동 + 그리퍼 동작.

필수 준비
---------
* calibration.npz  : 카메라 내부 파라미터(K) / 왜곡(dist) 저장.
* /dev/ttyJETCOBOT : 로봇 직렬 포트 (또는 인자로 전달).
* 카메라 장치 경로  : /dev/video0, /dev/jetcocam0 등.
* Python OpenCV ≥ 4.7 (aruco.ArucoDetector 사용).

사용 예시
---------
```
python3 jetcobot_apriltag_pickplace_combined.py \
    --port /dev/ttyJETCOBOT --baud 1000000 \
    --calib /home/jetcobot/Desktop/test/cam2base/calibration.npz \
    --cam   /dev/jetcocam0 \
    --marker 20 \
    --speed 20
```

노트
----
* 변환식(goal_coords 계산)은 사용자 실험 기반 휴리스틱입니다. 실제 핸드아이/외부 캘리브레이션 후 수정하세요.
* 화면 종료: 'q'
* Ctrl+C 로 긴급 종료 가능.
"""

import os
import time
import argparse
import numpy as np
import threading
import cv2
from pymycobot.genre import Angle, Coord

#변수 지정---------
place_to_put_coords = [231.7, -144.8, 152.7, -165.63, -7.66, -61.65]
TARGET_COUNT   = 0
picked_ids     = set()  # 이미 집은 마커 ID 기록
# -------------------------------------------------------------------
try:
    from pymycobot.mycobot280 import MyCobot280
    mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
    mc.thread_lock = True
    print("로봇이 연결되었습니다.")
except ImportError:
    MyCobot280 = None   # 로봇 없이 테스트 모드 허용

# ────────────────────────────────────────────────────────────────────────────
# 캘리브레이션 로더 (키 자동 호환)
# ────────────────────────────────────────────────────────────────────────────

def load_calibration_npz(path: str):
    """NPZ 파일에서 카메라 행렬(K)과 왜곡(dist)을 반환.
    지원 키: camera_matrix|K|mtx, dist_coeffs|dist.
    """
    if not os.path.isfile(path):
        raise FileNotFoundError(path)
    data = np.load(path)
    files = data.files
    if   "camera_matrix" in files: K = data["camera_matrix"]
    elif "K"             in files: K = data["K"]
    elif "mtx"           in files: K = data["mtx"]
    else: raise KeyError("camera matrix 키(camera_matrix/K/mtx) 없음")

    if   "dist_coeffs" in files: dist = data["dist_coeffs"]
    elif "dist"        in files: dist = data["dist"]
    else: raise KeyError("distortion 키(dist_coeffs/dist) 없음")
    return K, dist

# ────────────────────────────────────────────────────────────────────────────
# AprilTag / ArUco 감지기 생성
# ────────────────────────────────────────────────────────────────────────────

def create_detector(dict_name: str = "APRILTAG_36h11"):
    """dict_name 문자열을 받아 해당 사전 생성.
    허용 값: APRILTAG_36h11, 4X4_250, 5X5_250 등 OpenCV 명칭 일부.
    """
    dict_name = dict_name.upper()
    mapping = {
        "APRILTAG_36H11": cv2.aruco.DICT_APRILTAG_36h11,
        "4X4_250":       cv2.aruco.DICT_4X4_250,
        "5X5_250":       cv2.aruco.DICT_5X5_250,
    }
    if dict_name not in mapping:
        raise ValueError(f"지원되지 않는 딕셔너리: {dict_name}")
    aruco_dict  = cv2.aruco.getPredefinedDictionary(mapping[dict_name])
    aruco_param = cv2.aruco.DetectorParameters()
    detector    = cv2.aruco.ArucoDetector(aruco_dict, aruco_param)
    return detector

# ────────────────────────────────────────────────────────────────────────────
# solvePnP 기반 1-마커 포즈 추정
# ────────────────────────────────────────────────────────────────────────────

def estimate_pose_single(corners_list, marker_len, K, dist):
    """여러 마커 코너 목록(corners_list)에 대해 각 마커 rvec/tvec 계산.

    corners_list : detectMarkers()가 반환한 corners.
    marker_len   : 마커 한 변(mm).
    K, dist      : 카메라 내부 파라미터.

    반환: (rvecs, tvecs) 리스트.
    """
    objp = np.array([
        [-marker_len/2,  marker_len/2, 0],
        [ marker_len/2,  marker_len/2, 0],
        [ marker_len/2, -marker_len/2, 0],
        [-marker_len/2, -marker_len/2, 0],
    ], dtype=np.float32)

    rvecs, tvecs = [], []
    for c in corners_list:
        # c shape: (1,4,2) → (4,2)
        imgp = c.squeeze().astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(
            objp, imgp, K, dist,
            flags=cv2.SOLVEPNP_IPPE_SQUARE   # 사각 태그에 적합
        )
        if ok:
            rvecs.append(rvec)
            tvecs.append(tvec)
        else:
            rvecs.append(None)
            tvecs.append(None)
    return rvecs, tvecs

# ────────────────────────────────────────────────────────────────────────────
# 카메라 좌표 → 로봇 베이스 좌표 휴리스틱 변환
# ────────────────────────────────────────────────────────────────────────────

def camera_tvec_to_robot_goal(tvec_cam, base_coords, z_offset= -180.0,
                              xy_scale=1.0, x_offset=12.0, y_offset= -195.0):
    """사용자 제공 임시 변환 (정확한 Extrinsic 모를 때).

    tvec_cam: [tx, ty, tz] (mm), 카메라 좌표계.
    base_coords: 현재 로봇 TCP(base) 좌표 [x,y,z,...].

    매핑 (사용자 코드 기반):
        goal_x = base_x + tz - z_offset
        goal_y = base_y - tx*xy_scale + x_offset
        goal_z = base_z + ty*xy_scale + y_offset

    반환 goal_xyz(list length=3).
    """
    tx, ty, tz = tvec_cam
    bx, by, bz = base_coords[:3]
    rx, ry, rz = base_coords[3:6]       # 꺼내 쓰기만
    gx = bx + tz + z_offset
    gy = by - tx*xy_scale + x_offset + 7
    gz = bz + ty*xy_scale + y_offset
    if 250 < gx < 279:
        gx -= 15
    elif 280 < gx < 290:
        gx += 10
    elif gx > 290:
        gx += 5
        rx += 10
        ry -= 10
        rz -= 5

    if gz < 105:
        gz += 25
    elif gz > 115:
        gz -= 30
    
    pos = [gx, gy, gz]
    rot = [rx, ry, rz]
    return pos, rot

# ────────────────────────────────────────────────────────────────────────────
# 메인 루프: 감지 + 로봇 이동
# ────────────────────────────────────────────────────────────────────────────

def run(detector, K, dist, cam_dev, robot: MyCobot280 | None,
        home_coords, marker_len, speed, pick_once=True,
        target_id=None, show_axes=True):
    """실시간 감지 루프.

    pick_once : True → 첫 검출 후 집기 수행하고 종료.
    target_id : None → 아무 마커나 / 정수 → 해당 ID만.
    """
    cap = cv2.VideoCapture(cam_dev)
    # ------------------------------------------------------------------
    NO_TAG_LIMIT  = 60    # 태그가 안 보이는 연속 프레임 수(≈2 s) → 종료
    no_tag_frames = 0
    # ------------------------------------------------------------------
    if not cap.isOpened():
        raise RuntimeError(f"카메라 열기 실패: {cam_dev}")

    time.sleep(1)  # warm‑up

    # 로봇 초기화 -------------------------------------------------------------
    if robot is not None:
        robot.send_coords(home_coords, speed)
        time.sleep(1)
        try:
            robot.set_gripper_value(100, speed)   # open
        except Exception:
            pass
        base_coords = robot.get_coords()  # 기준 포즈
        print(f"[INFO] 현재 로봇팔 좌표: {base_coords}")
    else:
        base_coords = [0,0,0,0,0,0]
        print("[WARN] 로봇 모듈 없음 – 시뮬레이션 모드")

    print("[INFO] AprilTag 감지 시작. 'q' 로 종료.")

    #picked = False
    while True:
        ok, frame = cap.read()
        if not ok:
            print("[WARN] 프레임 읽기 실패"); time.sleep(0.05); continue

        # 왜곡 보정(선택) -> drawRect 왜곡이 심하면 사용
        frame_und = cv2.undistort(frame, K, dist)

        corners, ids, rejected = detector.detectMarkers(frame_und)
        if ids is not None: ids = ids.flatten()

        # ★ 항상 최신 TCP 좌표 사용
        if robot is not None:
            base_coords = robot.get_coords()

        # ────────────────────────── tag-missing watchdog ───────────────────────────
        if ids is None or len(ids) == 0:          # 이번 프레임에 태그가 0개
            no_tag_frames += 1
            print(f"[DBG] no_tag_frames = {no_tag_frames}", flush=True)
            if no_tag_frames >= NO_TAG_LIMIT:     # 60프레임(≈2 s) 지속 → 종료
                print("[INFO] 태그가 일정 시간 사라져 작업 종료")
                break                             # while 탈출
            continue                              # 다음 프레임
        else:
            no_tag_frames = 0                     # 태그가 보이면 카운터 리셋
        # ───────────────────────────────────────────────────────────────────────────

        if ids is not None and len(ids) > 0:
            # 선택적 target 필터
            if target_id is not None:
                sel = [i for i,(mid) in enumerate(ids) if mid == target_id]
                if sel:
                    use_idx = sel
                else:
                    use_idx = []
            else:
                use_idx = range(len(ids))

            draw_corners = []
            draw_ids     = []
            '''for i in use_idx:
                draw_corners.append(corners[i])
                draw_ids.append(ids[i])'''
            # ── [MOD] ID 낮은 순 & 미집은 것만 ----------------------------------  # <= SORT-FILTER
            pairs = [(ids[i], corners[i]) for i in use_idx if ids[i] not in picked_ids]
            pairs.sort(key=lambda p: p[0])    # ID 오름차순

            draw_ids, draw_corners = zip(*pairs) if pairs else ([], [])
            # -------------------------------------------------------------------

            if draw_corners:
                # 포즈 추정 --------------------------------------------------
                rvecs, tvecs = estimate_pose_single(draw_corners, marker_len, K, dist)
                for c, mid, rvec, tvec in zip(draw_corners, draw_ids, rvecs, tvecs):
                    if rvec is None: continue
                    c2 = c.squeeze()  # (4,2)
                    tl,tr,br,bl = c2
                    cX, cY = c2.mean(axis=0).astype(int)

                    # 폴리라인 & ID ------------------------------------------
                    cv2.polylines(frame_und, [c2.astype(int)], True, (0,255,0), 2)
                    for p in [tl,tr,br,bl]:
                        cv2.circle(frame_und, tuple(p.astype(int)), 4, (255,0,0), -1)
                    cv2.putText(frame_und, f"ID:{int(mid)}", (cX-10, cY-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

                    # 포즈 텍스트 --------------------------------------------
                    tx,ty,tz = tvec.flatten()
                    rx,ry,rz = np.rad2deg(rvec).flatten()
                    pos_txt = f"Pos:({tx:.1f},{ty:.1f},{tz:.1f})mm"
                    rot_txt = f"Rot:({rx:.1f},{ry:.1f},{rz:.1f})deg"
                    cv2.putText(frame_und, pos_txt, (cX-10, cY+10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45,(0,0,255),1)
                    cv2.putText(frame_und, rot_txt, (cX-10, cY+28),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45,(0,0,255),1)

                    # 좌표축 --------------------------------------------------
                    if show_axes:
                        cv2.drawFrameAxes(frame_und, K, dist, rvec, tvec, marker_len/2)

                    # 로봇 이동 (pick_once 조건) -----------------------------
                    if robot is not None: #(not picked) and (len(picked_ids) < TARGET_COUNT)
                        goal_xyz, rot_rpy = camera_tvec_to_robot_goal(
                            tvec.flatten(), base_coords,
                            z_offset= -180.0, xy_scale=1.0,
                            x_offset=12.0, y_offset= -195.0
                        )
                        '''if len(base_coords) >= 6:
                            full_goal = goal_xyz + list(base_coords[3:6])
                        else:
                            full_goal = goal_xyz'''
                        full_goal = goal_xyz + rot_rpy          # 위치 3 + 회전 3
                        print(f"[INFO] 이동할 로봇팔 좌표: {full_goal}")
                        time.sleep(2)
                        try:
                            robot.send_coords(full_goal, speed, 1)
                            time.sleep(2)

                            robot.set_gripper_value(0, speed)  # close(집기)
                            time.sleep(1)

                            picked_ids.add(int(mid))            # ← 여기에 삽입

                            robot.send_coords(home_coords, speed, 1)  # 복귀
                            time.sleep(2)

                            # 관절 1(베이스)을 -30도로 이동
                            joint_id = Angle.J1.value # 관절 1 (베이스)
                            angle = -20
                            print(f"관절 {joint_id}를 {angle}도로 이동합니다.")
                            mc.send_angle(joint_id, angle, speed)
                            time.sleep(2) # 움직임이 완료될 때까지 대기

                            mc.send_coords(place_to_put_coords, speed)
                            time.sleep(2) # 움직임이 완료될 때까지 대기

                            robot.set_gripper_value(100, speed)  # open(열기)
                            time.sleep(2) # 움직임이 완료될 때까지 대기

                            # 관절 2(베이스)을 10도로 이동
                            joint_id = Angle.J2.value # 관절 2
                            angle = -10
                            print(f"관절 {joint_id}를 {angle}도로 이동합니다.")
                            mc.send_angle(joint_id, angle, speed)
                            time.sleep(2) # 움직임이 완료될 때까지 대기

                            robot.send_coords(home_coords, speed, 1)  # 복귀

                        except Exception as e:
                            print(f"[ERR] 로봇 명령 실패: {e}")
                        picked = True
                        if pick_once:
                            # 한 번 집고 종료 요청 → while 루프 탈출 준비
                            pass

            '''if len(picked_ids) >= TARGET_COUNT:                          #pick_once and picked:
                # 잠시 화면 보여주고 종료
                cv2.imshow('AprilTag Detection', frame_und)
                time.sleep(3)
                cv2.waitKey(500)
                break'''
        # end if ids

        cv2.imshow('AprilTag Detection', frame_und)
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            break
    # end while

    cap.release()
    cv2.destroyAllWindows()

# ────────────────────────────────────────────────────────────────────────────
# 메인 엔트리
# ────────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument('--port',   default='/dev/ttyJETCOBOT', help='JetCobot 직렬 포트')
    ap.add_argument('--baud',   type=int, default=1_000_000, help='JetCobot baudrate')
    ap.add_argument('--calib',  default='/home/jetcobot/Desktop/test/cam2base/calibration.npz', help='카메라 캘리브레이션 npz')
    ap.add_argument('--cam',    default=0, help='카메라 장치 인덱스 또는 경로')
    ap.add_argument('--marker', type=float, default=20.0, help='마커 한 변 길이(mm)')
    ap.add_argument('--speed',  type=int, default=30, help='로봇 이동 속도(%)')
    ap.add_argument('--dict',   default='APRILTAG_36h11', help='Aruco/AprilTag 딕셔너리명')
    ap.add_argument('--target', type=int, default=None, help='특정 마커 ID만 추적')
    ap.add_argument('--no-pick', action='store_true', help='로봇 이동/집기 비활성화(디버그)')
    ap.add_argument('--show-axes', action='store_true', help='drawFrameAxes 표시')
    # 초기 홈좌표 간단 인자들 (x,y,z,rx,ry,rz) 선택적
    ap.add_argument('--home', nargs=6, type=float, metavar=('X','Y','Z','RX','RY','RZ'),
                    default=[156.7,-43.0,295.5,-161.23,-14.97,-36.5], help='로봇 초기 좌표')
    args = ap.parse_args()

    # 캘리브레이션 로드 ------------------------------------------------------
    try:
        K, dist = load_calibration_npz(args.calib)
    except Exception as e:
        print(f"[ERR] 캘리브레이션 로드 실패: {e}")
        return

    # 감지기 ---------------------------------------------------------------
    try:
        detector = create_detector(args.dict)
    except Exception as e:
        print(f"[ERR] 딕셔너리 생성 실패: {e}")
        return

    # 로봇 연결 ------------------------------------------------------------
    robot = None
    if not args.no_pick:
        if MyCobot280 is None:
            print('[WARN] pymycobot 불가. 로봇 없음 모드로 실행.')
        else:
            try:
                robot = MyCobot280(args.port, args.baud)
            except Exception as e:
                print(f'[WARN] 로봇 연결 실패: {e}. 로봇 없음 모드로 계속.')
                robot = None

    # 실행 -----------------------------------------------------------------
    run(detector, K, dist, args.cam, robot,
        home_coords=args.home, marker_len=args.marker, speed=args.speed,
        pick_once=False, target_id=args.target, show_axes=args.show_axes)
    #not args.no_pick

    print('[INFO] 프로그램 종료')

if __name__ == '__main__':
    main()
