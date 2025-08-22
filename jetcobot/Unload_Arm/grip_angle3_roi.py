

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

from dataclasses import dataclass  # (★) 다품목 주문용

# ─────────────────────────────────────────────────────────────
# 공통 유틸
# ─────────────────────────────────────────────────────────────

def _ensure_dir(d):
    try:
        os.makedirs(d, exist_ok=True)
    except Exception:
        pass

def _save(path, img):
    try:
        cv2.imwrite(path, img)
    except Exception:
        pass

def _unwrap_to_prev(cur_deg, prev_deg):
    """prev와 가장 가까운 표현(cur, cur±180)으로 언랩"""
    cands = [cur_deg, cur_deg + 180.0, cur_deg - 180.0]
    diffs = [abs(c - prev_deg) for c in cands]
    return cands[int(np.argmin(diffs))]

def _ema(prev, cur, alpha=0.4):
    """지수이동평균(저역통과)"""
    return cur if prev is None else (prev + alpha * (cur - prev))


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

# ★ 오래된 프레임 확실히 비우기(USB/V4L2 버퍼가 말썽일 때 효과적)
def flush_camera(cap, secs=0.40, min_reads=15, sleep=0.01):
    """
    secs 동안(그리고 최소 min_reads회) cap.grab()을 반복해 카메라 큐를 비운다.
    일부 백엔드는 즉시 반환하므로 sleep을 주어 실제 최신 프레임이 도착할 시간을 확보.
    """
    end = time.time() + float(secs)
    n = 0
    while time.time() < end or n < int(min_reads):
        cap.grab()
        if sleep:
            time.sleep(sleep)
        n += 1


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

def _minarea_angle(rect, offset=0.0):
    """
    minAreaRect 결과 각도 보정(여기서만 보정 수행)
    rect = ((cx,cy), (w,h), angle)
    - 오프셋 적용
    - w<h면 angle = 180 - angle - 90; angle = -angle
    - 범위 정규화는 하지 않음(원하는 대로 -180~180 사용)
    """
    (w, h) = rect[1]
    ang = float(rect[2])
    print(f"실제 물체 기울어진 각도: {ang:.2f}도")

    # 1) 사용자 오프셋
    ang = ang - float(offset)

    # 2) 가로<세로 보정
    if w < h:
        print("가로 < 세로")
        ang = 180.0 - ang - 90.0
        ang = -ang
    print(f"그리퍼 각도: {ang:.2f}도")
        

    return float(ang)

def _intersect_rect(r1, r2):
    """두 사각형(r=(x1,y1,x2,y2))의 교집합을 반환. 없으면 None"""
    x1 = max(r1[0], r2[0]); y1 = max(r1[1], r2[1])
    x2 = min(r1[2], r2[2]); y2 = min(r1[3], r2[3])
    if x2 <= x1 or y2 <= y1:
        return None
    return (x1, y1, x2, y2)


def estimate_angle_from_roi(
    bgr, box_xyxy,
    use_otsu=False,
    fixed_thr=240,
    debug_draw=False,
    margin_ratio=0.22,
    offset=0.0,
    debug_save_dir=None,
    save_prefix=None,
    min_area=80,
    # ───── 추가 옵션 ─────
    fixed_crop=None,           # (fx1,fy1,fx2,fy2): 먼저 bgr을 이 영역으로 크롭
    exclude_rects=None         # [(x1,y1,x2,y2), ...]: 고정 크롭 이후 좌표계로 가릴 영역
):
    """
    반환: (angle_deg, dbg_img, area, aspect)
      - angle_deg: 컨투어 기반(minAreaRect) 각도(오프셋/보정 적용). 없으면 PCA 백업 또는 None
      - dbg_img  : 이진화/컨투어 디버그 이미지 (BGR). 없으면 None
      - area     : 선택된 컨투어 면적(px^2) 또는 None
      - aspect   : max(w,h) / min(w,h) (종횡비) 또는 None

    처리 순서:
      1) (옵션) fixed_crop으로 입력 bgr을 먼저 자름 → 좌표계가 고정 크롭 기준으로 변경
      2) (옵션) exclude_rects를 검은색으로 덮어 분석 제외
      3) 바운딩박스 좌표를 고정 크롭 좌표계로 평행이동
      4) 고정 크롭 프레임 경계와 교집합만 사용(부분만 겹치면 그 부분만 사용)
      5) margin_ratio로 ROI 확장, 프레임 경계로 클램프
      6) Gray/Blur → 이진화 → 컨투어 중 가장 큰 것(minAreaRect) → 각도 산출
         (컨투어 없음이면 PCA 백업)
    """
    H0, W0 = bgr.shape[:2]
    if debug_save_dir:
        _ensure_dir(debug_save_dir)
    if save_prefix is None:
        save_prefix = f"{int(time.time() * 1000)}"

    # 0) 고정 크롭 적용(선택)
    fx1 = fy1 = 0
    if fixed_crop is not None:
        fx1, fy1, fx2, fy2 = [int(v) for v in fixed_crop]
        fx1 = max(0, fx1); fy1 = max(0, fy1)
        fx2 = min(W0, fx2); fy2 = min(H0, fy2)
        if fx2 <= fx1 or fy2 <= fy1:
            return None, None, None, None
        bgr = bgr[fy1:fy2, fx1:fx2].copy()

        # ★ 추가: 고정 영역 ROI 저장
        if debug_save_dir:
            _ensure_dir(debug_save_dir)
            _save(os.path.join(debug_save_dir, f"{save_prefix or int(time.time()*1000)}_fixed.png"), bgr)

    H, W = bgr.shape[:2]

    # 0-1) (선택) 제외 영역 가리기 — 고정 크롭 좌표계 기준
    if exclude_rects:
        for (ex1, ey1, ex2, ey2) in exclude_rects:
            ex1 = int(np.clip(ex1, 0, W)); ex2 = int(np.clip(ex2, 0, W))
            ey1 = int(np.clip(ey1, 0, H)); ey2 = int(np.clip(ey2, 0, H))
            if ex2 > ex1 and ey2 > ey1:
                bgr[ey1:ey2, ex1:ex2] = 0

    # 1) 바운딩박스를 고정 크롭 좌표계로 변환
    x1, y1, x2, y2 = [float(v) for v in box_xyxy]
    x1 -= fx1; x2 -= fx1
    y1 -= fy1; y2 -= fy1

    # 1-1) 고정 크롭 프레임과의 교집합(부분만 겹치면 그 부분만 사용)
    inter = _intersect_rect((x1, y1, x2, y2), (0, 0, W, H))
    if inter is None:
        if debug_save_dir:
            _save(os.path.join(debug_save_dir, f"{save_prefix}_debug.png"),
                  np.zeros((8, 8, 3), np.uint8))
        return None, None, None, None
    x1, y1, x2, y2 = inter
    x1 = float(x1); y1 = float(y1); x2 = float(x2); y2 = float(y2)

    # 2) margin 포함 ROI 클립(클램프)
    side = max(x2 - x1, y2 - y1)
    m = int(margin_ratio * max(1.0, side))
    rx1 = max(0, int(x1) - m)
    ry1 = max(0, int(y1) - m)
    rx2 = min(W, int(x2) + m)
    ry2 = min(H, int(y2) + m)

    roi = bgr[ry1:ry2, rx1:rx2]
    if debug_save_dir:
        _ensure_dir(debug_save_dir)
        _save(os.path.join(debug_save_dir, f"{save_prefix or int(time.time()*1000)}_roi.png"), roi)

    if roi.size == 0:
        if debug_save_dir:
            z1 = np.zeros((4, 4), np.uint8)
            z3 = np.zeros((4, 4, 3), np.uint8)
            _save(os.path.join(debug_save_dir, f"{save_prefix}_gray.png"), z1)
            _save(os.path.join(debug_save_dir, f"{save_prefix}_thr.png"),  z1)
            _save(os.path.join(debug_save_dir, f"{save_prefix}_debug.png"), z3)
        return None, None, None, None
    
    # ★ 추가: 각도 성공/실패와 무관하게 ROI 원본을 항상 저장
    if debug_save_dir:
        _ensure_dir(debug_save_dir)
        _save(os.path.join(debug_save_dir, f"{save_prefix or int(time.time()*1000)}_roi.png"), roi)

    # 3) Gray + Blur
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    if debug_draw:
        cv2.imshow("angle_gray", gray); cv2.waitKey(1)
    if debug_save_dir:
        _save(os.path.join(debug_save_dir, f"{save_prefix}_gray.png"), gray)

    # 내부 헬퍼: 이진화 이미지에서 '가장 큰 컨투어 1개' 선택 (기존과 동일)
    def _best_from_thresh(thr_img):
        cnts, _ = cv2.findContours(thr_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        best = None
        best_area = -1.0
        for c in cnts:
            area = cv2.contourArea(c)
            if area < float(min_area):
                continue
            rect = cv2.minAreaRect(c)
            (w, h) = rect[1]
            if w < 1 or h < 1:
                continue
            if area > best_area:
                best_area = area
                aspect = float(max(w, h) / max(1e-6, min(w, h)))
                best = (c, rect, area, aspect)
        return best

    # 4) 이진화
    if use_otsu:
        _, thr     = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY     + cv2.THRESH_OTSU)
        _, thr_inv = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        cand1 = _best_from_thresh(thr)
        cand2 = _best_from_thresh(thr_inv)
        if cand1 and cand2:
            cand = cand1 if cand1[2] >= cand2[2] else cand2
            dbg_src = thr if cand is cand1 else thr_inv
        else:
            cand = cand1 or cand2
            dbg_src = thr if cand is cand1 else thr_inv
    else:
        _, thr = cv2.threshold(gray, int(fixed_thr), 255, cv2.THRESH_BINARY)
        cand = _best_from_thresh(thr)
        dbg_src = thr

    if debug_draw and dbg_src is not None:
        cv2.imshow("angle_thr", dbg_src); cv2.waitKey(1)
    if debug_save_dir and dbg_src is not None:
        _save(os.path.join(debug_save_dir, f"{save_prefix}_thr.png"), dbg_src)

    # 5) 컨투어 경로 (최우선)
    if cand:
        c, rect, area, aspect = cand
        angle = _minarea_angle(rect, offset=offset)

        dbg_img = cv2.cvtColor(dbg_src, cv2.COLOR_GRAY2BGR)
        box = cv2.boxPoints(rect).astype(np.int32)
        cv2.drawContours(dbg_img, [box], 0, (0, 255, 0), 2)
        cv2.putText(dbg_img, f"angle={angle:.1f} deg", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50, 220, 50), 2)
        # cv2.putText(dbg_img, f"area={area:.0f} ar={aspect:.2f}", (8, 40),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.55, (50, 200, 200), 1)

        if debug_draw:
            cv2.imshow("angle_debug", dbg_img); cv2.waitKey(1)
        if debug_save_dir:
            _save(os.path.join(debug_save_dir, f"{save_prefix}_debug.png"), dbg_img)

        return float(angle), dbg_img, float(area), float(aspect)

    # 6) PCA 백업 경로
    edges = cv2.Canny(gray, 50, 150)
    ys, xs = np.where(edges > 0)
    if xs.size >= 50:
        pts = np.column_stack((xs.astype(np.float32), ys.astype(np.float32)))
        mean, eigenvectors = cv2.PCACompute(pts, mean=None)[:2]
        v = eigenvectors[0]
        angle = math.degrees(math.atan2(v[1], v[0]))

        dbg_img = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        cv2.putText(dbg_img, f"PCA={angle:.1f} deg", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 50), 2)

        if debug_draw:
            cv2.imshow("angle_thr", edges); cv2.imshow("angle_debug", dbg_img); cv2.waitKey(1)
        if debug_save_dir:
            _save(os.path.join(debug_save_dir, f"{save_prefix}_thr.png"), edges)
            _save(os.path.join(debug_save_dir, f"{save_prefix}_debug.png"), dbg_img)

        return float(angle), dbg_img, None, None

    return None, None, None, None



# ─────────────────────────────────────────────────────────────
# (★) 다품목/다수량 주문을 위한 자료구조 & 헬퍼
# ─────────────────────────────────────────────────────────────
@dataclass
class OrderLine:
    item: str
    remaining: int

def _safe_list(x):
    if x is None:
        return []
    if isinstance(x, (list, tuple)):
        return list(x)
    data = getattr(x, 'data', None)
    if data is not None:
        return list(data)
    try:
        return list(x)
    except TypeError:
        return [x]


# ─────────────────────────────────────────────────────────────
# rclpy Node
# ─────────────────────────────────────────────────────────────
class GrabBoxNode(Node):
    def __init__(self):
        super().__init__('grab_box')
        self.declare_and_get_params()
        _ensure_dir(self.angle_debug_dir)


        # --- 주문/상태 공유 변수 (★ 개편) ---
        self._order_evt = threading.Event()
        self._last_order = None
        self._busy        = False

        self.order_lock = threading.Lock()
        self.order_lines: list[OrderLine] = []  # 예: [OrderLine('snack',2), OrderLine('vegetable',1)]
        self.done_count   = 0  # 집은 총 개수(정보성)
        self._last_pick_xy = None

        # 라벨 정규화 매핑
        self.label_alias = {
            # snack 계열
            "과자":"snack", "스낵":"snack",
            "snack":"snack", "snacks":"snack",
            "snack_box":"snack",
            "srack":"snack",   # ★ YOLO 오타 보정
            "snac":"snack",
            # vegetable 계열
            "채소":"vegetable", "야채":"vegetable",
            "veg":"vegetable", "vegetable":"vegetable",
            # drink/음료 계열
            "음료":"drink", "드링크":"drink",
            "drink":"drink", "drinks":"drink", "beverage":"drink",
            "drink_box":"drink",
            # ── shoes(신발) 계열 + 잘못 학습된 라벨 보정 ──
            "shoe":"shoes",
            "shoes":"shoes",
            "shoe_box":"shoes",
            "fruit":"shoes",      # ← 핵심: fruit을 shoes로 강제
            "fruits":"shoes",
            "과일":"shoes",
        }

        # (★) 소프트 락 상태
        self.lock_item = None      # 현재 잠금 라벨
        self.lock_until = 0.0      # 잠금 만료 시각
        self.lock_miss = 0         # 잠금 라벨 미검출 연속 카운트

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

    # ---------------- 주문 파싱/관리 ----------------
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

    def _canon(self, s: str) -> str:
        if s is None:
            return ""
        key = str(s).strip().lower()
        return self.label_alias.get(key, key)

    def _merge_same_items(self, lines: list[OrderLine]) -> list[OrderLine]:
        agg = {}
        for ln in lines:
            if ln.item == "" or ln.remaining <= 0:
                continue
            agg[ln.item] = agg.get(ln.item, 0) + int(ln.remaining)
        return [OrderLine(item=k, remaining=v) for k, v in agg.items()]

    def _parse_msg_to_lines(self, msg) -> list[OrderLine]:
        """
        확장 스키마: msg.items(string[]), msg.counts(int32[])  → 권장
        레거시 스키마: msg.item(string), msg.count(int|int[])  → 자동 호환
        """
        items, counts = [], []

        # 확장
        if hasattr(msg, 'items') and getattr(msg, 'items') not in (None, []):
            items = _safe_list(msg.items)
            if hasattr(msg, 'counts') and getattr(msg, 'counts') not in (None, []):
                counts = [int(c) for c in _safe_list(msg.counts)]
            else:
                counts = [1] * len(items)
        else:
            # 레거시
            item = getattr(msg, "item", "")
            items = _safe_list(item) if isinstance(item, (list, tuple)) else [item]
            cnt = getattr(msg, "count", 1)
            counts = [int(c) for c in (_safe_list(cnt) or [1])]
            if len(counts) == 1 and len(items) > 1:
                counts = counts * len(items)

        if len(items) != len(counts):
            raise ValueError(f'items({len(items)}) vs counts({len(counts)}) 길이 불일치')

        raw = [OrderLine(self._canon(i), int(c)) for i, c in zip(items, counts)]
        return self._merge_same_items(raw)

    def needed_set(self) -> set[str]:
        with self.order_lock:
            return {ln.item for ln in self.order_lines if ln.remaining > 0}

    def total_remaining(self) -> int:
        with self.order_lock:
            return sum(max(0, ln.remaining) for ln in self.order_lines)

    def is_order_done(self) -> bool:
        return self.total_remaining() <= 0

    def _format_order_lines_for_log(self, lines: list[OrderLine]) -> str:
        pretty = ', '.join([f'{ln.item}×{ln.remaining}' for ln in lines]) or '(비어있음)'
        return f'주문 수신: {pretty}'

    def _progress_string(self) -> str:
        with self.order_lock:
            parts = [f'{ln.item}×{ln.remaining}' for ln in self.order_lines if ln.remaining > 0]
            remain = ', '.join(parts) if parts else '없음'
            picked = self.done_count
            return f'진행상태: 남음=[{remain}] / 누적 집기={picked}'

    def mark_picked(self, detected_label: str):
        """YOLO 라벨로 품목 차감"""
        lab = self._canon(detected_label)
        changed = False
        with self.order_lock:
            for ln in self.order_lines:
                if ln.item == lab and ln.remaining > 0:
                    ln.remaining -= 1
                    self.done_count += 1
                    changed = True
                    break
            done = all(ln.remaining <= 0 for ln in self.order_lines)

        if changed:
            self.get_logger().info(self._progress_string())
        if done:
            self.get_logger().info("✅ 주문 완료")
            self._order_evt.clear()

    # (★) 소프트 락 헬퍼들
    def remaining_of(self, item: str) -> int:
        with self.order_lock:
            return sum(max(0, ln.remaining) for ln in self.order_lines if ln.item == item)

    # def choose_lock_item(self, df) -> str | None:
    #     """현재 프레임에 '보이는' 필요 품목 중 하나 선택"""
    #     needed = self.needed_set()
    #     if not needed or 'name' not in df.columns or df.empty:
    #         return None
    #     labs = df['name'].map(lambda s: self._canon(s))
    #     vis_needed = [lb for lb in labs if lb in needed]
    #     if not vis_needed:
    #         return None
    #     # 가장 많이 보이는 라벨 선택(동률이면 먼저 나오는 것)
    #     freq = {}
    #     for lb in vis_needed:
    #         freq[lb] = freq.get(lb, 0) + 1
    #     return max(freq.items(), key=lambda kv: kv[1])[0]
    def choose_lock_item(self, df) -> str | None:
        needed = self.needed_set()
        if not needed or 'name' not in df.columns or df.empty:
            return None
        cand = df[df['name'].map(self._canon).isin(needed)].copy()
        if cand.empty:
            return None

        # 1) 같은 라벨 개수 많은 순서
        cand['cnt'] = cand.groupby('name')['name'].transform('count')

        # 2) 동률이면 화면에서 '가장 위'(ymin가 작은) 우선
        # 3) 그래도 동률이면 confidence 높은 순
        cand = cand.sort_values(['cnt', 'ymin', 'confidence'],
                                ascending=[False, True, False])

        return str(cand.iloc[0]['name'])

    def update_lock(self, df) -> str | None:
        """소프트 락 유지/해제/갱신 후 현재 타깃 라벨 반환"""
        now = time.time()
        # 남은 수량 없으면 해제
        if self.lock_item and self.remaining_of(self.lock_item) <= 0:
            self.lock_item = None
        # 시간 만료로 해제
        if self.lock_item and now > self.lock_until:
            self.lock_item = None
        # 잠금 없으면 새로 선택
        if self.lock_item is None:
            pick = self.choose_lock_item(df)
            if pick:
                self.lock_item = pick
                self.lock_until = now + self.lock_timeout_s
                self.lock_miss = 0
        else:
            # 잠금 유지: 프레임에 보이면 연장, 안 보이면 miss++ 후 임계 초과 시 해제
            if 'name' in df.columns and not df.empty:
                vis = set(df['name'].map(lambda s: self._canon(s)))
                if self.lock_item in vis:
                    self.lock_miss = 0
                    self.lock_until = now + self.lock_timeout_s
                else:
                    self.lock_miss += 1
                    if self.lock_miss >= self.lock_miss_thresh:
                        self.lock_item = None
        return self.lock_item

    # 주문 콜백 (★ 개편)
    def listener_callback(self, msg: OrderMsg):
        self._last_order = msg
        try:
            lines = self._parse_msg_to_lines(msg)
            lines = [ln for ln in lines if ln.remaining > 0]
        except Exception as e:
            self.get_logger().error(f'주문 파싱 실패: {e}')
            return

        with self.order_lock:
            self.order_lines = lines
            self.done_count = 0

        self.get_logger().info(self._format_order_lines_for_log(lines))
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
        gp('yolo_wts',  '/home/jetcobot/project/calib/src/yolov5/runs/train/exp15/weights/best_new.pt')
        gp('out_img',   '/home/jetcobot/project/calib/src/detected_result.jpg')
        gp('px2base_npz', '/home/jetcobot/project/calib/src/H_px2base.npz')

        gp('home_coords',  [151.9, 108.2, 285.8, -172.54, -15.98, 39.95])
        gp('place_coords', [208.4, -63.1, 172.2, -178.94, 1.64, -139.28])

        gp('x_min', -120.0); gp('x_max', 270.0)
        gp('y_min', -250.0); gp('y_max', 250.0)
        gp('z_min',   20.0); gp('z_max', 400.0)

        gp('table_z_base', 125.0)
        gp('approach_up',  80.0)
        gp('grip_down',    45.0)
        gp('retreat_up',   110.0)

        gp('speed_approach', 80)
        gp('speed_grip',     80)
        gp('speed_place',    80)

        gp('bias_x', 27.0)
        gp('bias_y', 5)

        gp('single_shot', False)

        gp('yolo_imgsz', 640)      # ← 입력 이미지 크기(한 변), 기본 640
        gp('yolo_augment', False)  # ← TTA 비활성화(기본 False 권장)

        gp('angle_enable', True)           # True면 기울기 기반 J6 회전 적용
        gp('angle_offset', 76.77)            # ★ _minarea_angle에서 보정하므로 0으로
        gp('angle_min', -180.0)            # ★ 잘림 방지: -180으로 확장
        gp('angle_max',  180.0)            # ★ 잘림 방지:  180으로 확장
        gp('angle_speed', 80)              # send_angle 속도
        gp('angle_use_otsu', False)        # ★ 오츠 끄기(고정 임계 사용)
        gp('angle_fixed_thr', 230.0)       # ★ 고정 임계값 250
        gp('show_angle_debug', False)
        gp('angle_debug_dir', '/home/jetcobot/project/debug/angle')


        gp('angle_alpha', 1.0)            # EMA 계수(0.2~0.6 권장)
        gp('angle_jump_limit', 180.0)      # 한 번에 허용할 최대 변화량(deg)
        gp('angle_min_area', 200.0)       # ROI 컨투어 최소 면적(px)
        gp('angle_min_aspect', 1.35)      # minAreaRect 종횡비 임계(가늘수록 신뢰↑)

        gp('pick_dedupe_radius', 0.0)  # mm: 직전 픽 좌표로부터 이 반경 이내 후보는 건너뜀

        # (★) 소프트 락 파라미터
        gp('lock_timeout_s', 2.5)       # 잠금 유지 시간(보이면 연장)
        gp('lock_miss_thresh', 5)       # 잠금 품목이 안 보이는 프레임 연속 N회면 해제

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
        self.speed_place = int(p('speed_place').value)

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
        self.angle_debug_dir = p('angle_debug_dir').value

        self.yolo_imgsz  = int(p('yolo_imgsz').value)
        self.yolo_augment= bool(p('yolo_augment').value)

        self.angle_alpha       = float(self.get_parameter('angle_alpha').value)
        self.angle_jump_limit  = float(self.get_parameter('angle_jump_limit').value)
        self.angle_min_area    = float(self.get_parameter('angle_min_area').value)
        self.angle_min_aspect  = float(self.get_parameter('angle_min_aspect').value)
        self.pick_dedupe_radius = float(self.get_parameter('pick_dedupe_radius').value)

        # (★) 소프트 락 파라미터 읽기
        self.lock_timeout_s   = float(self.get_parameter('lock_timeout_s').value)
        self.lock_miss_thresh = int(self.get_parameter('lock_miss_thresh').value)

        # 상태
        self._angle_prev = None

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
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # 백엔드에 따라 무시될 수 있음

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
        mc.send_coords(self.home, self.speed_approach, 1); #wait_until_stop()
        mc.set_gripper_value(100, 100)

        # 메인 루프
        log.info('메인 루프 시작 (Ctrl+C로 종료)')

        try:
            while rclpy.ok() and self._ok:
                ret, frame = cap.read()
                if not ret:
                    self.get_logger().warn("카메라 프레임 실패, 재시도 중...")
                    continue  # 프레임 없으면 이번 루프 건너뛰고 다음 루프로

                # 주문 대기
                if not self._order_evt.is_set():
                    time.sleep(0.05)
                    continue

                # 주문 처리
                self._busy = True
                while rclpy.ok() and self._ok and (not self.is_order_done()):
                    # 🔑 여기서도 프레임을 계속 새로 읽어야 함
                    ret, frame = cap.read()
                    if not ret:
                        self.get_logger().warn("카메라 프레임 실패(주문 처리 중), 재시도 중...")
                        continue

                    t_all0 = time.time()

                    # >>> 읽기 직전 버퍼 확실히 비우기
                    flush_camera(cap, secs=0.25, min_reads=10, sleep=0.01)

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
                    # ▼ 추가: df 내 라벨을 전부 표준라벨로 정규화(이후 필터/락/차감 전 구간에 반영)
                    if 'name' in df.columns:
                        df['name'] = df['name'].map(self._canon)
                    t_post = (time.time() - t0) * 1000.0

                    log.info(f"[TIMING] cap={t_cap:.1f}ms pre={t_pre:.1f}ms infer={t_infer:.1f}ms post={t_post:.1f}ms detN={0 if df is None else len(df)}")

                    if df.empty:
                        mc.send_coords(self.home, self.speed_approach, 1); time.sleep(0.05)
                        if self.single_shot:
                            pass
                        continue

                    df = df[(df['confidence'] > 0.20)]
                    if df.empty:
                        continue

                    # --- (★) 현재 필요한 품목만 1차 필터 전 라벨 가시화
                    if 'name' in df.columns:
                        seen = sorted({ self._canon(s) for s in df['name'] })
                        log.info(f"[SEEN] labels={seen} needed={sorted(self.needed_set())}")

                    # --- (★) 현재 필요한 품목만 1차 필터 ---
                    needed = self.needed_set()
                    if 'name' in df.columns and needed:
                        df = df[df['name'].map(lambda s: self._canon(s)).isin(needed)]
                        if df.empty:
                            continue

                    # --- (★) 소프트 락으로 이번 프레임의 타깃 라벨 결정 ---
                    target_item = self.update_lock(df)
                    if target_item:
                        df = df[df['name'].map(lambda s: self._canon(s)) == target_item]
                        if df.empty:
                            log.info(f"[SKIP] target_item='{target_item}' 보이지 않음")
                            continue
                        else:
                            log.info(f"[TARGET] 이번 픽 목표 라벨 = {target_item}")

                    # --- 후보 선택: 직전 픽 좌표 근처는 스킵 ---
                    chosen = None
                    for _, rc in df.sort_values(['ymin','confidence'], ascending=[True, False]).iterrows():
                        cx_c = float((rc.xmin + rc.xmax) * 0.5)
                        cy_c = float((rc.ymin + rc.ymax) * 0.5)

                        und_cand = undistort_pts(np.array([[cx_c, cy_c]], np.float32), K, dist)
                        base_xy_cand = cv2.perspectiveTransform(und_cand.reshape(1,1,2), H_px2base)[0,0]
                        xb_c, yb_c = float(base_xy_cand[0]), float(base_xy_cand[1])

                        d_last = None if (self._last_pick_xy is None) else math.hypot(xb_c - self._last_pick_xy[0], yb_c - self._last_pick_xy[1])
                        if (self._last_pick_xy is None) or (d_last >= self.pick_dedupe_radius):
                            chosen = (rc, xb_c, yb_c, cx_c, cy_c)
                            break

                    # ★ 전부 가까우면: 예전엔 top-1로 폴백 → 같은 자리 재집기 위험
                    if chosen is None:
                        if self._last_pick_xy is not None:
                            # 가장 먼 후보를 선택(충분히 멀 때만)
                            far, far_d = None, -1.0
                            for _, rc2 in df.iterrows():
                                cx2 = float((rc2.xmin + rc2.xmax) * 0.5)
                                cy2 = float((rc2.ymin + rc2.ymax) * 0.5)
                                und2 = undistort_pts(np.array([[cx2, cy2]], np.float32), K, dist)
                                base2 = cv2.perspectiveTransform(und2.reshape(1,1,2), H_px2base)[0,0]
                                xb2, yb2 = float(base2[0]), float(base2[1])
                                d2 = math.hypot(xb2 - self._last_pick_xy[0], yb2 - self._last_pick_xy[1])
                                if d2 > far_d:
                                    far_d = d2
                                    far = (rc2, xb2, yb2, cx2, cy2)
                            if far is not None and far_d >= (self.pick_dedupe_radius * 0.6):
                                chosen = far
                                log.info(f"[DEDUPE] all near; picking farthest d={far_d:.1f}mm")
                            else:
                                log.info(f"[DEDUPE] all candidates within ~{self.pick_dedupe_radius}mm → wait fresh frame")
                                flush_camera(cap, secs=0.30, min_reads=12, sleep=0.01)
                                time.sleep(0.05)
                                continue
                        else:
                            # 첫 픽이면 confidence 기준
                            rc = df.sort_values(['ymin','confidence'], ascending=[True, False]).iloc[0]
                            cx_c = float((rc.xmin + rc.xmax) * 0.5)
                            cy_c = float((rc.ymin + rc.ymax) * 0.5)
                            und_cand = undistort_pts(np.array([[cx_c, cy_c]], np.float32), K, dist)
                            base_xy_cand = cv2.perspectiveTransform(und_cand.reshape(1,1,2), H_px2base)[0,0]
                            xb_c, yb_c = float(base_xy_cand[0]), float(base_xy_cand[1])
                            chosen = (rc, xb_c, yb_c, cx_c, cy_c)

                    # 선택 결과 언팩
                    r, xb, yb, cx, cy = chosen
                    x1, y1, x2, y2 = map(float, [r.xmin, r.ymin, r.xmax, r.ymax])
                    log.info(f'chosen base=({xb:.1f},{yb:.1f}) mm, conf={float(r.confidence):.2f}')

                    # 왜곡 보정 픽셀 → base 평면으로 투영
                    und_c = undistort_pts(np.array([[cx, cy]], np.float32), K, dist)
                    base_xy = cv2.perspectiveTransform(und_c.reshape(1, 1, 2), H_px2base)[0, 0]
                    xb, yb = float(base_xy[0]), float(base_xy[1])
                    log.info(f'px→base: ({cx:.1f},{cy:.1f}) → ({xb:.1f},{yb:.1f}) mm')

                    # 목표 좌표 계산(필드 오프셋 포함) — 하드 오프셋 제거
                    x_goal = xb + self.bias_x
                    y_goal = yb + self.bias_y
                    z_goal = self.table_z_base

                    if y_goal > 200:
                        y_goal -= 25
                        z_goal += 5
                    if 220 > x_goal > 205:
                        y_goal += 15
                    if 170 > x_goal:
                        y_goal -= 25
                    log.info(f'goal XY=({x_goal:.1f},{y_goal:.1f}), z={z_goal:.1f}')
                    lim = self.limits
                    approach = clamp_xyz(x_goal, y_goal, z_goal + self.approach_up, lim)
                    grip     = clamp_xyz(x_goal, y_goal, z_goal + self.grip_down, lim)
                    retreat  = clamp_xyz(x_goal, y_goal, z_goal + self.retreat_up, lim)

                    # 리트리트 Z는 접근 Z 이상 보장
                    retreat[2] = max(retreat[2], approach[2])

                    rpy = self.home[3:]

                    def finite6(v):
                        return all(math.isfinite(x) for x in v)

                    target_approach = approach + rpy
                    target_grip     = grip     + rpy
                    target_retreat  = retreat  + rpy

                    if not (finite6(target_approach) and finite6(target_grip) and finite6(target_retreat)):
                        log.error('NaN/Inf target → 이동 스킵')
                        continue

                    # 프레임 크기 구하기
                    H0, W0 = bgr.shape[:2]

                    # 고정 ROI 지정 (예: 이미지 가운데 60%만 분석)
                    fixed_crop = (int(W0*0.22), int(H0*0), int(W0*0.72), int(H0*0.75))

                    # 저장 파일 접두어(밀리초+픽셀좌표로 유일하게)
                    save_prefix = f"{int(time.time()*1000)}_{int(cx)}_{int(cy)}"

                    # ── (NEW) ROI로 기울기 추정 ──────────────────────────────
                    ang_res = estimate_angle_from_roi(
                        bgr, (x1, y1, x2, y2),
                        use_otsu=self.angle_use_otsu,
                        fixed_thr=self.angle_fixed_thr,
                        debug_draw=self.show_angle_debug,
                        offset=self.angle_offset,
                        debug_save_dir=self.angle_debug_dir,
                        save_prefix=save_prefix,
                        # fixed_crop=fixed_crop
                        fixed_crop=None
                    )
                    angle_deg, angle_dbg = ang_res[:2]
                    a_area, a_aspect = (ang_res[2], ang_res[3]) if len(ang_res) >= 4 else (None, None)

                    a_cmd = None
                    if angle_deg is not None:
    # ※ _minarea_angle()의 결과만 사용하는 게 목적이면,
    #    PCA 경로(angle_area/aspect가 None)일 때는 스킵합니다.
                        if (a_area is not None) and (a_aspect is not None):
                            a_cmd = float(np.clip(angle_deg, self.angle_min, self.angle_max))
                            log.info(
            f'angle(cmd from _minarea_angle)={a_cmd:.1f}° '
            f'(raw={angle_deg:.1f}°, area={a_area:.0f}, ar={a_aspect:.2f})'
        )
                        else:
                            log.info('컨투어 기반 각도 아님(PCA) → J6 회전 스킵')
                    else:
                        log.info('물체 기울기 추정 실패 → J6 회전 건너뜀')
                    

                    # 현재 좌표 로그
                    cur = mc.get_coords()
                    log.info(f'current6D: {cur}')

                    # --- J6 회전 로직은 필요 시 활성화 ---
                    rotate = mc.get_coords()
                    if not rotate or len(rotate) != 6 or not all(math.isfinite(v) for v in rotate):
                        rotate = target_grip
                    else:
                        rotate = list(rotate)
                    if self.angle_enable and (a_cmd is not None):
                        try:
                            mc.send_angle(int(Angle.J6.value), a_cmd, self.angle_speed)
                            print(f"모터: {a_cmd:.2f}도")
                            wait_until_stop(); #time.sleep(4)
                            _rot = mc.get_coords()
                            if _rot and len(_rot) == 6 and all(math.isfinite(v) for v in _rot):
                                rotate = list(_rot)
                        except Exception as e:
                            log.warning(f'J6 회전 실패: {e}')

                    goal_a = list(target_approach[:3]) + list(rotate[3:6])
                    mc.send_coords(goal_a, self.speed_approach, 0); time.sleep(0.05)

                    # 내려가서 집기
                    goal_b = list(target_grip[:3]) + list(rotate[3:6])
                    mc.send_coords(goal_b, self.speed_grip, 1)
#                    현재 좌표 로그

#                     cur = mc.get_coords()
#                     log.info(f'current6D: {cur}')

#                     # --- J6 회전 값이 send_coords로 덮어씌워지지 않도록 보존 ---
#                     # (get_coords가 도착 전일 수 있으니, 우리가 계산한 a_cmd를 직접 rotate[5]에 주입)
#                     rotate = mc.get_coords()
#                     if not rotate or len(rotate) != 6 or not all(math.isfinite(v) for v in rotate):
#                         rotate = target_grip  # fallback
#                     else:
#                         rotate = list(rotate)

#                     if self.angle_enable and (a_cmd is not None):
#                         try:
#         # 1) 조인트 회전
#                             mc.send_angle(int(Angle.J6.value), a_cmd, self.angle_speed)
#                             print(f"모터: {a_cmd:.2f}도")
#         # 충분히 대기(더 정확히 하려면 로봇 상태 폴링)
#                             time.sleep(3)

#         # 2) 이후 좌표 명령에서 RZ가 예전 값으로 덮이지 않도록 직접 주입
#                             rotate[5] = a_cmd
#                         except Exception as e:
#                             log.warning(f'J6 회전 실패: {e}')

# # 이후 pose 구성 시, 반드시 rotate[3:6] 사용
#                     goal_a = list(target_approach[:3]) + list(rotate[3:6])
#                     mc.send_coords(goal_a, self.speed_approach, 0); time.sleep(0.2)

# # 내려가서 집기
#                     goal_b = list(target_grip[:3]) + list(rotate[3:6])
#                     mc.send_coords(goal_b, self.speed_grip, 1)

                    time.sleep(0.05)

                    approach = goal_b.copy(); approach[2] += 5.0
                    mc.send_coords(approach, self.speed_grip, 1)
                    for dz in (3.0, 1.5):
                        step = goal_b.copy(); step[2] = goal_b[2] + dz
                        mc.send_coords(step, 15, 1); #time.sleep(0.15)

                    mc.set_gripper_value(0, 100); time.sleep(0.05)
                    goal_c = list(target_retreat[:3]) + list(rotate[3:6])
                    mc.send_coords(goal_c, self.speed_approach, 0); time.sleep(0.05)

                    # 결과 프레임 저장 (각도 디버그도 오버레이)
                    out = res.render()[0].copy()  # RGB
                    cv2.circle(out, (int(cx), int(cy)), 5, (0, 255, 0), -1)
                    if angle_deg is not None:
                        cv2.putText(out, f"angle={angle_deg:.1f} deg", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (50, 220, 50), 2)

                    # ▼ 추가: 표준화된 라벨(= shoes)을 이미지에도 표시
                    try:
                        lab_raw = r['name'] if 'name' in r.index else names[int(r['class'])]
                    except Exception:
                        lab_raw = str(r.get('name', ''))
                    canon_lab = target_item or self._canon(lab_raw)
                    cv2.putText(out, f"label={canon_lab}", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 200, 50), 2)

                    cv2.imwrite(self.out_img, cv2.cvtColor(out, cv2.COLOR_RGB2BGR))
                    log.info(f'결과 이미지 저장: {self.out_img}')

                    # 디버그 ROI 창 (옵션)
                    if self.show_angle_debug and (angle_dbg is not None):
                        cv2.imshow('angle_roi', angle_dbg)
                        cv2.waitKey(1)

                    # Place → Home
                    if y_goal > 200:
                        mc.send_coords([166.5, 209.3, 193.7, -173.36, -6.38, 39.33], self.speed_place, 1); time.sleep(0.05)
                        mc.send_coords([141.3, 200.5, 196.6, -153.22, 19.12, 38.53], self.speed_place, 1); time.sleep(0.05)
                    mc.send_coords([160.9, 99.4, 262.6, 179.27, -3.23, 42.79], self.speed_place, 1)
                    mc.send_coords([206.2, 7.0, 200.5, -178.01, 8.05, -136.42], self.speed_place, 1); time.sleep(0.05)
                    mc.send_coords(self.place, self.speed_place, 1); #wait_until_stop()
                    self._last_pick_xy = (xb, yb)
                    mc.set_gripper_value(100, 100); #time.sleep(0.5)
                    mc.send_coords(self.home, self.speed_approach, 1); #wait_until_stop()

                    # >>> 이동 동안 쌓인 오래된 프레임 폐기(조금 더 강하게)
                    flush_camera(cap, secs=0.30, min_reads=12, sleep=0.01)

                    # (★) 집은 품목 차감: target_item이 있으면 그것으로, 없으면 모델 라벨로
                    try:
                        picked_label = r['name'] if 'name' in r.index else names[int(r['class'])]
                    except Exception:
                        picked_label = str(r.get('name', ''))
                    decide_label = target_item or self._canon(picked_label)
                    log.info(f"[PICK] model='{self._canon(picked_label)}', target='{target_item}' → count-down '{decide_label}'")
                    self.mark_picked(decide_label)

                # 목표 달성 시 완료 신호
                if self.is_order_done():
                    done = Bool(); done.data = True
                    self.bool_publisher.publish(done)
                    log.info(f'[DONE] 완료. finish_msg=True')
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
