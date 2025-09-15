#!/usr/bin/env python3
import os, cv2, torch, numpy as np

repo = '/home/jetcobot/project/calib/src/yolov5'
w    = '/home/jetcobot/project/calib/src/yolov5/runs/train/exp11/weights/best.pt'

# 모델 로드
m = torch.hub.load(repo, 'custom', path=w, source='local').eval()
m.conf = 0.15          # 필요시 0.10~0.25 조절
m.iou  = 0.45
# 모델 레벨에서 'clothes'(index 0)만 보고 싶으면 주석 해제
# m.classes = [0]
# m.max_det = 1

def preprocess_bgr(bgr):
    """하이라이트/명암 보정: LAB-CLAHE + 감마"""
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    l,a,b = cv2.split(lab)
    l = cv2.createCLAHE(2.0, (8,8)).apply(l)
    bgr = cv2.cvtColor(cv2.merge([l,a,b]), cv2.COLOR_LAB2BGR)
    gamma = 1.2
    lut = np.array([((i/255.0)**(1/gamma))*255 for i in range(256)]).astype("uint8")
    return cv2.LUT(bgr, lut)

# 카메라 캡처 (필요시 '/dev/video0', cv2.CAP_V4L2)
cap = cv2.VideoCapture(0)
ok, frame = cap.read()
cap.release()
assert ok, "카메라 프레임 캡처 실패"

# 전처리 → RGB로 변환
rgb = cv2.cvtColor(preprocess_bgr(frame), cv2.COLOR_BGR2RGB)

# ── 추론 ──────────────────────────────────────────────────────
res = m(rgb, size=960, augment=True)

# ── 결과 필터링(임계값 + 클래스=clothes) ─────────────────────
df = res.pandas().xyxy[0]
df = df[(df['confidence'] > 0.15) & (df['name'] == 'clothes')]

if df.empty:
    print("[INFO] clothes 탐지 없음")
    cx = cy = None
else:
    # 최고 점수 1개 선택(인덱스 안전하게)
    r = df.sort_values('confidence', ascending=False).iloc[0]
    cx = float((r.xmin + r.xmax) / 2.0)
    cy = float((r.ymin + r.ymax) / 2.0)
    w  = float(r.xmax - r.xmin)
    h  = float(r.ymax - r.ymin)
    print(f"[INFO] 픽셀 중심=({cx:.1f}, {cy:.1f}), 크기=({w:.1f},{h:.1f}), conf={float(r.confidence):.3f}")

# ── 시각화/저장 ──────────────────────────────────────────────
out_dir = '/home/jetcobot/runs/yolo_cam1'
os.makedirs(out_dir, exist_ok=True)

# YOLO가 그린 박스 저장
res.save(save_dir=out_dir, exist_ok=True)

# read-only 문제 방지: 복사본에 마커 그리기
boxed = res.render()[0].copy()  # RGB, writeable
if cx is not None:
    cv2.drawMarker(
        boxed, (int(cx), int(cy)),
        color=(0,255,0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2
    )

cv2.imwrite(os.path.join(out_dir, 'frame_marked.jpg'),
            cv2.cvtColor(boxed, cv2.COLOR_RGB2BGR))
print("Saved into:", out_dir)
