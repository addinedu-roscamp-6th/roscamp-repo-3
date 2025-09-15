#!/usr/bin/env python3
"""
Hand-Eye Calibration (Eye-to-Hand)
──────────────────────────────────
csv/log 에 저장한 robot_poses, cam_poses를 읽어
extrinsic_cam2base.npz 를 생성한다.
"""

import cv2, numpy as np, pandas as pd

# ── 파일 경로 ────────────────────────────────
ROBOT_CSV = "robot_poses.csv"   # x y z Rx Ry Rz (deg)
CAM_CSV   = "cam_poses.csv"     # rvec_x rvec_y rvec_z  tvec_x tvec_y tvec_z
OUT_NPZ   = "extrinsic_cam2base.npz"

# 1. 로그 로드
r_df = pd.read_csv(ROBOT_CSV)
c_df = pd.read_csv(CAM_CSV)

assert len(r_df) == len(c_df) >= 10, "샘플은 최소 10개!"

Rg2b, tg2b = [], []
Rt2c, tt2c = [], []

for (_, r), (_, c) in zip(r_df.iterrows(), c_df.iterrows()):
    # base→marker (로봇이 보고하는 마커 pose)
    Rg2b.append(cv2.Rodrigues(np.deg2rad(r[3:6]).values)[0])
    tg2b.append(r[:3].values)

    # cam→marker (카메라가 추정한 마커 pose)
    Rt2c.append(cv2.Rodrigues(c[:3].values)[0])
    tt2c.append(c[3:6].values)

# 2. Hand-Eye
Rc2b, tc2b = cv2.calibrateHandEye(Rg2b, tg2b, Rt2c, tt2c,
                                  method=cv2.CALIB_HAND_EYE_TSAI)

# 3. 저장
T = np.eye(4)
T[:3,:3], T[:3,3] = Rc2b, tc2b.squeeze()
np.savez(OUT_NPZ, T=T)
print(f"[SAVE] {OUT_NPZ}  ← 완료")
