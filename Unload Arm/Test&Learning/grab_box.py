#!/usr/bin/env python3
import cv2, time, torch, numpy as np
from pymycobot.mycobot280 import MyCobot280
import warnings
warnings.filterwarnings('ignore')

# ───────── 사용자 설정 ───────── #
PORT        = '/dev/ttyJETCOBOT'
BAUD        = 1_000_000
CAMERA_DEV  = '/dev/video0'            # 필요시 0 또는 '/dev/video0'
CALIB_PATH  = '/home/jetcobot/project/calib/calibration.npz'
YOLO_REPO   = '/home/jetcobot/project/calib/src/yolov5'
YOLO_WTS    = '/home/jetcobot/project/calib/src/yolov5/runs/train/exp13/weights/best.pt'
OUT_IMG     = '/home/jetcobot/project/calib/src/detected_result.jpg'
PX2BASE_NPZ = '/home/jetcobot/project/calib/src/H_px2base.npz'   # 네가 저장한 경로로 변경

# 안전 리밋(대략, 필요시 수정)
X_MIN, X_MAX = -120, 270
Y_MIN, Y_MAX = -250, 250
Z_MIN, Z_MAX =  20, 420

HOME_COORDS  = [129.4, 101.4, 289.1, -163.52, -9.98, 16.38]
PLACE_COORDS = [217.4, -145.5, 194.3, -174.99, 2.08, -62.93]

TABLE_Z_BASE = 112.0

z_goal = TABLE_Z_BASE

APPROACH_UP =  50.0   # 위에서 접근(+Z)
GRIP_DOWN   =  60.0   # 집는 깊이 보정(+면 내려감)
RETREAT_UP  =  70.0   # 위로 빼기

SPEED_APPROACH = 50
SPEED_GRIP     = 30

# 파일 상단
BIAS_X, BIAS_Y = 8.0, -3.0  # 일단 0으로 시작
# ──────────────────────────── #

def load_calib(path):
    data = np.load(path)
    K = data['K'] if 'K' in data.files else data.get('camera_matrix')
    dist = data['dist'] if 'dist' in data.files else data.get('dist_coeffs')
    if K is None or dist is None:
        raise RuntimeError("calibration.npz 에 K/dist 키가 필요합니다.")
    return K, dist

def undistort_pts(pts, K, dist):
    """pts: (N,2) 픽셀 → 왜곡 보정된 정규화 픽셀 → 다시 픽셀로 투영"""
    pts = np.asarray(pts, np.float32).reshape(-1,1,2)
    und = cv2.undistortPoints(pts, K, dist)           # 정규화 좌표 (x',y')
    und = cv2.convertPointsToHomogeneous(und).reshape(-1,3)
    und = (K @ und.T).T                                # 다시 픽셀로
    und = und[:, :2] / und[:, 2:]
    return und.astype(np.float32)

def estimate_depth_px(f_px, obj_mm, box_w_px, box_h_px=None):
    """핀홀근사: z ≈ f*W / w. 기울면 오차↑ → (w+h)/2 사용 가능."""
    pw = float(box_w_px)
    if box_h_px is not None:
        pw = 0.5 * (float(box_w_px) + float(box_h_px))
    pw = max(pw, 1.0)
    return (f_px * obj_mm) / pw

def clamp_xyz(x, y, z):
    return [float(np.clip(x, X_MIN, X_MAX)),
            float(np.clip(y, Y_MIN, Y_MAX)),
            float(np.clip(z, Z_MIN, Z_MAX))]

def preprocess_bgr(bgr):
    """LAB-CLAHE + 감마 보정으로 하이라이트 완화"""
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    l,a,b = cv2.split(lab)
    l = cv2.createCLAHE(2.0, (8,8)).apply(l)
    bgr = cv2.cvtColor(cv2.merge([l,a,b]), cv2.COLOR_LAB2BGR)
    gamma = 1.2
    lut = np.array([((i/255.0)**(1/gamma))*255 for i in range(256)], dtype=np.uint8)
    return cv2.LUT(bgr, lut)

def wait_until_stop(mc, timeout=8.0, poll=0.1):
    """간단 대기 루틴(장비에 따라 is_moving 지원 여부 다름)"""
    t0 = time.time()
    while time.time()-t0 < timeout:
        time.sleep(poll)

def main():
    # ── 장치 준비 ──────────────────────────────────────────────
    mc = MyCobot280(PORT, BAUD)
    mc.power_on()
    cap = cv2.VideoCapture(CAMERA_DEV)
    if not cap.isOpened():
        raise RuntimeError("카메라 열기 실패")
    cap.set(cv2.CAP_PROP_FPS, 15)

    K, dist = load_calib(CALIB_PATH)
    f_px = float(K[0,0])

    # 픽셀(undistorted) → base(XY) 호모그래피 로드
    cal = np.load(PX2BASE_NPZ)
    H_px2base = cal['H']
    # npz 안에 K/dist도 저장해뒀다면 그대로 사용(권장)
    if 'K' in cal.files:    K = cal['K']
    if 'dist' in cal.files: dist = cal['dist']
    if H_px2base.shape != (3,3):
        raise RuntimeError("H_px2base.npz의 H가 3x3이 아닙니다.")

    # YOLOv5
    model = torch.hub.load(YOLO_REPO, 'custom', path=YOLO_WTS, source='local').eval()
    model.conf = 0.20
    model.iou  = 0.45
    # model.classes = [0]   # 'clothes'만 (원하면 주석 해제)
    names = model.names

     # ── [추가] 디버그 헬퍼 (루프 바깥에 1번만) ─────────────────
    import math
    def finite6(v):
        return all(math.isfinite(x) for x in v)

    def l1(a,b):
        return sum(abs(x - y) for x, y in zip(a, b))
    # ───────────────────────────────────────────────────────────

    # ── 홈 포즈 ────────────────────────────────────────────────
    print("[INFO] 초기 자세 이동")
    mc.send_coords(HOME_COORDS, SPEED_APPROACH, 1); wait_until_stop(mc)
    mc.set_gripper_value(100, 50); time.sleep(0.5)

    print("[INFO] 메인 루프 시작 (q=종료)")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[WARN] 카메라 프레임 실패"); time.sleep(0.05); continue

            # 전처리 + RGB
            bgr = preprocess_bgr(frame)
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

            with torch.no_grad():
                res = model(rgb, size=960, augment=True)

            # pandas 결과로 후처리
            df = res.pandas().xyxy[0]
            if df.empty:
                # 원위치 유지(선택)
                mc.send_coords(HOME_COORDS, SPEED_APPROACH, 1); time.sleep(0.5)
                continue

            # 원하는 클래스만(예: clothes) + 임계값
            df = df[(df['confidence'] > 0.20)]  # & (df['name'] == 'clothes')
            if df.empty:
                continue

            # 최고 점수 1개
            r = df.sort_values('confidence', ascending=False).iloc[0]
            x1, y1, x2, y2 = map(float, [r.xmin, r.ymin, r.xmax, r.ymax])
            cx, cy = (x1+x2)/2.0, (y1+y2)/2.0
            bw, bh  = (x2-x1), (y2-y1)
            label   = f"{r['name']} {float(r.confidence):.2f}"

            # 왜곡 보정된 픽셀 → base XY(mm) (npz의 H 사용)
            und_c   = undistort_pts(np.array([[cx, cy]], np.float32), K, dist)  # (1,2)
            base_xy = cv2.perspectiveTransform(und_c.reshape(1,1,2), H_px2base)[0,0]
            xb, yb  = float(base_xy[0]), float(base_xy[1])
            print(f"[DBG] px→base: ({cx:.1f},{cy:.1f}) → ({xb:.1f},{yb:.1f}) mm")

            # ── 이동 시퀀스 ─────────────────────────────────────
            # 목표 포즈: 평면 좌표를 그대로 base XYZ로 쓴다고 가정(호모그래피를 base 평면에 맞춰 잡았을 때!)
            # 축 방향이 다르면 부호를 조정하세요.
            x_goal = xb + BIAS_X + 18#X_mm_off
            y_goal = yb + BIAS_Y - 15#Y_mm_off
            z_goal = TABLE_Z_BASE #HOME_COORDS[2] + cz - 40.0    # 카메라-평면 높이 보정값(환경에 맞게 튜닝)

            print(f"[DBG] pre-clamp goal XY=({x_goal:.1f},{y_goal:.1f})")

            print(f"[DBG] z_goal(table)={z_goal:.1f}, approachZ={z_goal+APPROACH_UP:.1f}, gripZ={z_goal+GRIP_DOWN:.1f}, retreatZ={z_goal+RETREAT_UP:.1f}")

            # 접근 → 그립 → 탈출
            approach = clamp_xyz(x_goal, y_goal, z_goal + APPROACH_UP)
            grip     = clamp_xyz(x_goal, y_goal, z_goal + GRIP_DOWN)
            retreat  = clamp_xyz(x_goal, y_goal, z_goal + RETREAT_UP)

            print(f"[DBG] post-clamp approach XY=({approach[0]:.1f},{approach[1]:.1f})")

            # 자세는 홈 자세의 RPY 사용(필요 시 고정 각도 세팅 권장)
            rpy = HOME_COORDS[3:]

            # ── [추가] 디버그 블럭 시작 ─────────────────────────
            target_approach = approach + rpy
            target_grip     = grip     + rpy
            target_retreat  = retreat  + rpy

            print("[DBG] approach6D:", target_approach)
            print("[DBG] grip6D    :", target_grip)
            print("[DBG] retreat6D :", target_retreat)

            # 유한값 검사 (루프 안이므로 'return' 대신 'continue'가 안전)
            if not (finite6(target_approach) and finite6(target_grip) and finite6(target_retreat)):
                print("[ERR] NaN/Inf in targets → 이동 스킵")
                continue

            cur = mc.get_coords()
            print("[DBG] current6D :", cur)
            print("[DBG] d(approach,cur) =", l1(target_approach, cur))
            # ── [추가] 디버그 블럭 끝 ───────────────────────────

            mc.send_coords(approach + rpy, SPEED_APPROACH, 0); wait_until_stop(mc)
            here = mc.get_coords()
            ex, ey = (x_goal - here[0]), (y_goal - here[1])
            print(f"[TUNE] XY error after approach: ({ex:.1f}, {ey:.1f}) mm → BIAS_X/Y에 반영")
            mc.send_coords(grip     + rpy, SPEED_GRIP,     1); time.sleep(0.5)
            # grip 바로 전에 작은 단계로 3~5회 내려보기 (충돌 위험 없는 범위에서)
            STEP = 2.0   # mm
            N    = 3
            for _ in range(N):
                g = [grip[0], grip[1], grip[2] - STEP] + rpy
                mc.send_coords(g, 15, 0); time.sleep(0.2)
            mc.set_gripper_value(0, 50);                      time.sleep(0.8)
            mc.send_coords(retreat  + rpy, SPEED_APPROACH, 1); wait_until_stop(mc)

            # 결과 저장(YOLO 박스 + 현재 프레임)
            out = res.render()[0].copy()
            cv2.circle(out, (int(cx), int(cy)), 5, (0,255,0), -1)
            cv2.imwrite(OUT_IMG, cv2.cvtColor(out, cv2.COLOR_RGB2BGR))
            print(f"[INFO] 결과 이미지 저장: {OUT_IMG}")

            # 놓을 위치로 이동
            mc.send_coords(PLACE_COORDS, SPEED_APPROACH, 1); wait_until_stop(mc)
            mc.set_gripper_value(100, 50); time.sleep(0.5)

            # 홈 복귀 (필요 시)
            mc.send_coords(HOME_COORDS, SPEED_APPROACH, 1); wait_until_stop(mc)
            break

    except KeyboardInterrupt:
        print("[INFO] 종료 요청됨 (Ctrl+C)")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("[INFO] 카메라 해제 및 프로그램 종료")

if __name__ == '__main__':
    main()
