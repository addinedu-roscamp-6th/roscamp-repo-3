#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, math, time, threading
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")

import numpy as np
import cv2
try:
    cv2.setNumThreads(1)
except Exception:
    pass

import torch
try:
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
except Exception:
    pass
import rclpy
from array import array as pyarray
from rclpy.node import Node
from std_msgs.msg import Bool
from my_msgs.msg import OrderMsg
from pymycobot.mycobot280 import MyCobot280
from pymycobot.genre import Angle  # ← J1~J6 열거형 사용

# ─────────────────────────────────────────────────────────────
# 공통 유틸
# ─────────────────────────────────────────────────────────────

def load_calib(path):
    data = np.load(path)
    K = data['K'] if 'K' in data.files else data.get('camera_matrix')
    dist = data['dist'] if 'dist' in data.files else data.get('dist_coeffs')
    if K is None or dist is None:
        raise RuntimeError("calibration.npz 에 K/dist 키가 필요합니다.")
    return K, dist


def undistort_pts(pts, K, dist):
    pts = np.asarray(pts, np.float32).reshape(-1, 1, 2)
    und = cv2.undistortPoints(pts, K, dist)            # 정규화 좌표
    und = cv2.convertPointsToHomogeneous(und).reshape(-1, 3)
    und = (K @ und.T).T                                 # 픽셀로 재투영
    und = und[:, :2] / und[:, 2:]
    return und.astype(np.float32)


def clamp_xyz(x, y, z, lim):
    X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MIN, Z_MAX = lim
    return [float(np.clip(x, X_MIN, X_MAX)),
            float(np.clip(y, Y_MIN, Y_MAX)),
            float(np.clip(z, Z_MIN, Z_MAX))]


def preprocess_bgr(bgr):
    # 밝기 보정(CLAHE) + 감마 보정으로 YOLO/컨투어 안정화
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l = cv2.createCLAHE(2.0, (8, 8)).apply(l)
    bgr = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
    gamma = 1.2
    lut = np.array([((i / 255.0) ** (1 / gamma)) * 255 for i in range(256)], dtype=np.uint8)
    return cv2.LUT(bgr, lut)


def wait_until_stop(timeout=8.0, poll=0.1):
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(poll)


# ─────────────────────────────────────────────────────────────
# (NEW) 그립 각도 관련 유틸
# ─────────────────────────────────────────────────────────────

def _roi_from_box(x1, y1, x2, y2, W, H, margin_ratio=0.15):
    """YOLO 박스 주변으로 margin을 두고 ROI 클리핑"""
    w = x2 - x1
    h = y2 - y1
    m = int(margin_ratio * max(w, h))
    rx1 = max(0, int(x1) - m)
    ry1 = max(0, int(y1) - m)
    rx2 = min(W, int(x2) + m)
    ry2 = min(H, int(y2) + m)
    return rx1, ry1, rx2, ry2


def _minarea_angle(rect):
    """
    cv2.minAreaRect 반환 angle 보정
    rect = ((cx,cy), (w,h), angle)
    OpenCV 규약: angle ∈ [-90,0), w < h 일 때 90도 보정해서 -90~90으로 맞춤
    """
    (w, h) = rect[1]
    ang = rect[2]
    if w < h:
        ang = ang + 90.0
    # -90 ~ 90 범위로 정규화
    if ang > 90:
        ang -= 180
    if ang < -90:
        ang += 180
    return float(ang)


def estimate_angle_from_roi(bgr, box_xyxy, use_otsu=True, fixed_thr=200, debug_draw=False):
    """
    YOLO 박스 주변 ROI에서 가장 큰 외곽선의 minAreaRect로 기울기(deg) 추정
    반환: (angle_deg or None, debug_image or None)
    """
    H, W = bgr.shape[:2]
    x1, y1, x2, y2 = [int(v) for v in box_xyxy]
    rx1, ry1, rx2, ry2 = _roi_from_box(x1, y1, x2, y2, W, H)
    roi = bgr[ry1:ry2, rx1:rx2]
    if roi.size == 0:
        return None, None

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    # 약한 블러 + 히스토그램 평활화
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    # 임계: Otsu 우선, 아니면 고정값
    if use_otsu:
        _, thr = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    else:
        _, thr = cv2.threshold(gray, fixed_thr, 255, cv2.THRESH_BINARY)

    # 컨투어
    contours, _ = cv2.findContours(thr, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, (roi if debug_draw else None)

    cnt = max(contours, key=cv2.contourArea)
    if cv2.contourArea(cnt) < 50:  # 너무 작은 잡음 제거
        return None, (roi if debug_draw else None)

    rect = cv2.minAreaRect(cnt)
    angle = _minarea_angle(rect)

    dbg = None
    if debug_draw:
        dbg = roi.copy()
        box = cv2.boxPoints(rect).astype(np.int32)
        cv2.drawContours(dbg, [box], 0, (0, 255, 0), 2)
        # 가로/세로 시각화(선택)
        cv2.line(dbg, tuple(box[0]), tuple(box[1]), (0, 0, 255), 2)
        cv2.line(dbg, tuple(box[2]), tuple(box[3]), (0, 0, 255), 2)
        cv2.line(dbg, tuple(box[1]), tuple(box[2]), (255, 0, 0), 2)
        cv2.line(dbg, tuple(box[3]), tuple(box[0]), (255, 0, 0), 2)
        cv2.putText(dbg, f"angle={angle:.1f} deg", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50, 220, 50), 2)

    return angle, dbg


# ─────────────────────────────────────────────────────────────
# rclpy Node
# ─────────────────────────────────────────────────────────────
class GrabBoxNode(Node):
    def __init__(self):
        super().__init__('grab_box')
        self.declare_and_get_params()

        # --- 주문/상태 공유 변수 ---
        self.target_count = 0
        self.done_count   = 0
        self._order_evt = threading.Event()
        self._last_order = None

        self.order_id     = ""
        self._busy        = False

        # --- 퍼블리셔/서브스크라이버 ---
        self.bool_publisher = self.create_publisher(Bool, 'finish_msg', 10)
        self.subscription = self.create_subscription(
            OrderMsg,
            'order_topic2',
            self.listener_callback,
            10
        )

        self._ok = True
        self._thread = threading.Thread(target=self.worker, daemon=True)
        self._thread.start()
    
    def _to_scalar_int(self, v, default=0, name='count'):
        try:
            if isinstance(v, (int, np.integer)):
                return int(v)
            if isinstance(v, (list, tuple, pyarray, np.ndarray)):
                return int(v[0]) if len(v) > 0 else default
            return int(v)
        except Exception:
            self.get_logger().warning(f'invalid {name}={v!r}, fallback={default}')
            return default

    def _first_or_self(self, v, default=''):
        if isinstance(v, (list, tuple, np.ndarray, pyarray)):
            return v[0] if len(v) > 0 else default
        return v if v is not None else default

    # 주문 콜백
    def listener_callback(self, msg: OrderMsg):
        self._last_order = msg
        self.order_id     = getattr(msg, "order_id", "")
        self.target_count = self._to_scalar_int(msg.count, default=1, name='count')
        self.current_item = self._first_or_self(getattr(msg, "item", ""), default="")
        self.done_count   = 0

        self.get_logger().info(f'order_id="{msg.order_id}", item="{self.current_item}", count={self.target_count}')
        self._order_evt.set()

    # ---------------- Params ----------------
    def declare_and_get_params(self):
        gp = self.declare_parameter

        # 카메라 경로는 현장에 맞게 덮어쓰기 가능(기본값은 /dev/jetcocam0 로 세팅)
        gp('port', '/dev/ttyJETCOBOT')
        gp('baud', 1_000_000)
        gp('camera_dev', '/dev/jetcocam0')  # ← 기존 '/dev/video0'였다면 런치/CLI로 바꿔 가능
        gp('calib_path', '/home/jetcobot/project/calib/calibration.npz')
        gp('yolo_repo', '/home/jetcobot/project/calib/src/yolov5')
        gp('yolo_wts',  '/home/jetcobot/project/calib/src/yolov5/runs/train/exp15/weights/best.pt')
        gp('out_img',   '/home/jetcobot/project/calib/src/detected_result.jpg')
        gp('px2base_npz', '/home/jetcobot/project/calib/src/H_px2base.npz')

        gp('home_coords',  [-131.4, 94.0, 279.8, -149.61, -11.19, 109.37])
        gp('place_coords', [217.4, -145.5, 194.3, -174.99, 2.08, -62.93])

        gp('x_min', -120.0); gp('x_max', 270.0)
        gp('y_min', -250.0); gp('y_max', 250.0)
        gp('z_min',   20.0); gp('z_max', 420.0)

        gp('table_z_base', 120.0)
        gp('approach_up',  80.0)
        gp('grip_down',    50.0)
        gp('retreat_up',   70.0)

        gp('speed_approach', 50)
        gp('speed_grip',     30)

        gp('bias_x', 8.0)
        gp('bias_y', -3.0)

        gp('single_shot', False)

        gp('yolo_imgsz', 640)      # ← 입력 이미지 크기(한 변), 기본 640
        gp('yolo_augment', False)  # ← TTA 비활성화(기본 False 권장)

        # (NEW) 그립 각도 관련 파라미터
        gp('angle_enable', True)           # True면 기울기 기반 J6 회전 적용
        gp('angle_offset', -5.0)            # 각도 보정(현장 보정용, deg)
        gp('angle_min', -90.0)             # 안전 클램프
        gp('angle_max',  90.0)
        gp('angle_speed', 50)              # send_angle 속도
        gp('angle_use_otsu', True)         # Otsu 임계 vs 고정 임계
        gp('angle_fixed_thr', 200.0)       # 고정 임계 사용시 스칼라 값
        gp('show_angle_debug', False)      # True면 ROI 디버그 창 표시

        # 읽기
        p = self.get_parameter
        self.port = p('port').value
        self.baud = int(p('baud').value)
        self.camera_dev = p('camera_dev').value
        self.calib_path = p('calib_path').value
        self.yolo_repo = p('yolo_repo').value
        self.yolo_wts  = p('yolo_wts').value
        self.out_img   = p('out_img').value
        self.px2base_npz = p('px2base_npz').value

        self.home = [float(x) for x in p('home_coords').value]
        self.place= [float(x) for x in p('place_coords').value]

        self.limits = (
            float(p('x_min').value), float(p('x_max').value),
            float(p('y_min').value), float(p('y_max').value),
            float(p('z_min').value), float(p('z_max').value),
        )

        self.table_z_base = float(p('table_z_base').value)
        self.approach_up  = float(p('approach_up').value)
        self.grip_down    = float(p('grip_down').value)
        self.retreat_up   = float(p('retreat_up').value)

        self.speed_approach = int(p('speed_approach').value)
        self.speed_grip     = int(p('speed_grip').value)

        self.bias_x = float(p('bias_x').value)
        self.bias_y = float(p('bias_y').value)

        self.single_shot = bool(p('single_shot').value)

        # angle params
        self.angle_enable     = bool(p('angle_enable').value)
        self.angle_offset     = float(p('angle_offset').value)
        self.angle_min        = float(p('angle_min').value)
        self.angle_max        = float(p('angle_max').value)
        self.angle_speed      = int(p('angle_speed').value)
        self.angle_use_otsu   = bool(p('angle_use_otsu').value)
        self.angle_fixed_thr  = float(p('angle_fixed_thr').value)
        self.show_angle_debug = bool(p('show_angle_debug').value)

        self.yolo_imgsz  = int(p('yolo_imgsz').value)
        self.yolo_augment= bool(p('yolo_augment').value)

    # ---------------- Worker ----------------
    def worker(self):
        log = self.get_logger()

        # HW 준비
        try:
            mc = MyCobot280(self.port, self.baud)
            try:
                # 드라이버 버전에 따라 옵션이 없을 수 있으므로 가드
                mc.thread_lock = True
            except Exception:
                pass
            mc.power_on()
        except Exception as e:
            log.error(f'MyCobot 연결 실패: {e}')
            rclpy.shutdown(); return

        cap = cv2.VideoCapture(self.camera_dev)
        if not cap.isOpened():
            log.error('카메라 열기 실패'); rclpy.shutdown(); return
        cap.set(cv2.CAP_PROP_FPS, 15)

        K, dist = load_calib(self.calib_path)

        cal = np.load(self.px2base_npz)
        H_px2base = cal['H']
        if 'K' in cal.files:    K = cal['K']
        if 'dist' in cal.files: dist = cal['dist']
        if H_px2base.shape != (3, 3):
            log.error('H_px2base.npz의 H가 3x3이 아닙니다.'); rclpy.shutdown(); return

        # YOLOv5 로드
        try:
            model = torch.hub.load(self.yolo_repo, 'custom', path=self.yolo_wts, source='local').eval()
            model.conf = 0.20
            model.iou  = 0.45
            names = model.names
        except Exception as e:
            log.error(f'YOLO 로드 실패: {e}')
            rclpy.shutdown(); return

        # 홈 포즈
        log.info('초기 자세 이동')
        mc.send_coords(self.home, self.speed_approach, 1); wait_until_stop()
        mc.set_gripper_value(100, 50); time.sleep(0.5)

        # 메인 루프
        log.info('메인 루프 시작 (Ctrl+C로 종료)')

        try:
            while rclpy.ok() and self._ok:
                # 주문 대기
                if not self._order_evt.is_set():
                    time.sleep(0.05)
                    continue

                # 주문 처리
                self._busy = True
                while rclpy.ok() and self._ok and self.done_count < self.target_count:
                    t_all0 = time.time()
                    ok, frame = cap.read()
                    if not ok:
                        log.warning('카메라 프레임 실패'); time.sleep(0.05); continue
                    t_cap = (time.time() - t_all0) * 1000.0

                    t0 = time.time()
                    bgr = preprocess_bgr(frame)
                    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                    t_pre = (time.time() - t0) * 1000.0

                    t0 = time.time()
                    with torch.no_grad():
                        res = model(rgb, size=self.yolo_imgsz, augment=self.yolo_augment)
                    t_infer = (time.time() - t0) * 1000.0

                    t0 = time.time()
                    df = res.pandas().xyxy[0]
                    t_post = (time.time() - t0) * 1000.0

                    log.info(f"[TIMING] cap={t_cap:.1f}ms pre={t_pre:.1f}ms infer={t_infer:.1f}ms post={t_post:.1f}ms detN={0 if df is None else len(df)}")

                    if df.empty:
                        mc.send_coords(self.home, self.speed_approach, 1); time.sleep(0.5)
                        if self.single_shot:
                            pass
                        continue

                    df = df[(df['confidence'] > 0.20)]
                    if df.empty:
                        continue

                    # 최고 점수 1개
                    r = df.sort_values('confidence', ascending=False).iloc[0]
                    x1, y1, x2, y2 = map(float, [r.xmin, r.ymin, r.xmax, r.ymax])
                    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0

                    # 왜곡 보정 픽셀 → base 평면으로 투영
                    und_c = undistort_pts(np.array([[cx, cy]], np.float32), K, dist)
                    base_xy = cv2.perspectiveTransform(und_c.reshape(1, 1, 2), H_px2base)[0, 0]
                    xb, yb = float(base_xy[0]), float(base_xy[1])
                    log.info(f'px→base: ({cx:.1f},{cy:.1f}) → ({xb:.1f},{yb:.1f}) mm')

                    # 목표 좌표 계산(필드 오프셋 포함)
                    x_goal = xb + self.bias_x + 14.0
                    y_goal = yb + self.bias_y - 20
                    z_goal = self.table_z_base

                    log.info(f'goal XY=({x_goal:.1f},{y_goal:.1f}), z={z_goal:.1f}')
                    lim = self.limits
                    approach = clamp_xyz(x_goal, y_goal, z_goal + self.approach_up, lim)
                    grip     = clamp_xyz(x_goal, y_goal, z_goal + self.grip_down, lim)
                    retreat  = clamp_xyz(x_goal, y_goal, z_goal + self.retreat_up, lim)

                    rpy = self.home[3:]

                    def finite6(v):
                        return all(math.isfinite(x) for x in v)

                    target_approach = approach + rpy
                    target_grip     = grip     + rpy
                    target_retreat  = retreat  + rpy

                    if not (finite6(target_approach) and finite6(target_grip) and finite6(target_retreat)):
                        log.error('NaN/Inf target → 이동 스킵');
                        continue

                    # ── (NEW) ROI로 기울기 추정 ──────────────────────────────
                    angle_deg, angle_dbg = estimate_angle_from_roi(
                        bgr, (x1, y1, x2, y2),
                        use_otsu=self.angle_use_otsu,
                        fixed_thr=self.angle_fixed_thr,
                        debug_draw=self.show_angle_debug
                    )
                    if angle_deg is not None:
                        a_cmd = float(np.clip(angle_deg + self.angle_offset, self.angle_min, self.angle_max))
                        log.info(f'물체 기울기={angle_deg:.1f}°, 보정={self.angle_offset:.1f}° → J6={a_cmd:.1f}°')
                    else:
                        a_cmd = None
                        log.info('물체 기울기 추정 실패 → J6 회전 건너뜀')

                    # 현재 좌표 로그
                    cur = mc.get_coords()
                    log.info(f'current6D: {cur}')

                    # ── 접근 → (선회) → 그랩 → 리트리트 ───────────────────────
                    #here = mc.get_coords()
                    #ex, ey = (x_goal - here[0]), (y_goal - here[1])
                    #log.info(f'XY error after approach: ({ex:.1f}, {ey:.1f}) mm → BIAS_X/Y 튜닝 참고')

                    # (NEW) 접근 높이에서 손목 회전(J6) 적용
                    if self.angle_enable and (a_cmd is not None):
                        try:
                            mc.send_angle(int(Angle.J6.value), a_cmd, self.angle_speed)
                            wait_until_stop()
                            time.sleep(0.6)  # 짧게 안정화
                            rotate = mc.get_coords()
                            if not rotate or len(rotate) != 6:
                                raise RuntimeError("get_coords() invalid")
                        except Exception as e:
                            log.warning(f'J6 회전 실패: {e}')
                            rotate = mc.get_coords() or target_grip   # 실패 시 현재자세/타깃으로 폴백

                    mc.send_coords(target_approach, self.speed_approach, 0); wait_until_stop()

                    # 내려가서 집기
                    goal = list(target_grip[:3]) + list(rotate[3:6])
                    mc.send_coords(goal, self.speed_grip, 1)
                    time.sleep(0.4)

                    approach = goal.copy(); approach[2] += 5.0
                    mc.send_coords(approach, self.speed_grip, 1); time.sleep(0.2)
                    for dz in (3.0, 1.5, 0.0):
                        step = goal.copy(); step[2] = goal[2] + dz
                        mc.send_coords(step, 15, 1); time.sleep(0.15)

                    mc.set_gripper_value(0, 50); time.sleep(0.8)
                    mc.send_coords(target_retreat, self.speed_approach, 1); wait_until_stop()

                    # 결과 프레임 저장 (각도 디버그도 오버레이)
                    out = res.render()[0].copy()  # RGB
                    cv2.circle(out, (int(cx), int(cy)), 5, (0, 255, 0), -1)
                    if angle_deg is not None:
                        cv2.putText(out, f"angle={angle_deg:.1f} deg", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (50, 220, 50), 2)
                    cv2.imwrite(self.out_img, cv2.cvtColor(out, cv2.COLOR_RGB2BGR))
                    log.info(f'결과 이미지 저장: {self.out_img}')

                    # 디버그 ROI 창 (옵션)
                    if self.show_angle_debug and (angle_dbg is not None):
                        cv2.imshow('angle_roi', angle_dbg)
                        cv2.waitKey(1)

                    # Place → Home
                    mc.send_coords(self.place, self.speed_approach, 1); wait_until_stop()
                    mc.set_gripper_value(100, 50); time.sleep(0.5)
                    mc.send_coords(self.home, self.speed_approach, 1); wait_until_stop()

                    # 카운터
                    self.done_count += 1
                    log.info(f'[PROGRESS] {self.done_count} / {self.target_count}')
                
                # 목표 달성 시 완료 신호
                if self.done_count >= self.target_count:
                    done = Bool(); done.data = True
                    self.bool_publisher.publish(done)
                    log.info(f'[DONE] order_id="{self.order_id}" 완료. finish_msg=True')
                    self._order_evt.clear()
                    self._busy = False
                    if self.single_shot:
                        self._ok = False
                        rclpy.shutdown()
                        break
                else:
                    self._busy = False
                    time.sleep(0.05)

        except KeyboardInterrupt:
            log.info('종료 요청(Ctrl+C)')
        finally:
            cap.release()
            cv2.destroyAllWindows()
            log.info('카메라 해제 및 종료')


def main():
    rclpy.init()
    node = GrabBoxNode()
    try:
        rclpy.spin(node)
    finally:
        node._ok = False
        if node._thread.is_alive():
            node._thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()