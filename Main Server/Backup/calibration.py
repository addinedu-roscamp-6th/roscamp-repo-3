import cv2
import numpy as np
import os

# === [1] 체커보드 내부 코너 수 설정 ===
# CHECKERBOARD = (7, 6)  # (열, 행) 내부 코너 수
CHECKERBOARD = (8, 6)  # (열, 행) 내부 코너 수

# === [2] 3D 기준 좌표 생성 ===
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

objpoints = []  # 실제 3D 좌표
imgpoints = []  # 이미지 좌표

# === [3] 카메라 열기 ===
cap = cv2.VideoCapture(2)

print("[INFO] 's' 키로 캘리브레이션 이미지 저장, 'q' 키로 종료")

count = 0
while True:
    ret, frame = cap.read()
    if not ret:
        print("[ERROR] 프레임 캡처 실패")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # === [4] 체커보드 코너 탐지 ===
    ret_corners, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret_corners:
        cv2.drawChessboardCorners(frame, CHECKERBOARD, corners, ret_corners)
        cv2.putText(frame, "Press 's' to save this image", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("Calibration View", frame)
    key = cv2.waitKey(1)

    # === [5] 저장 ===
    if key == ord('s') and ret_corners:
        objpoints.append(objp)
        imgpoints.append(corners)
        count += 1
        print(f"[INFO] 저장된 캘리브레이션 이미지 수: {count}")

    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# === [6] 캘리브레이션 계산 ===
if count >= 10:
    print("[INFO] 내부 파라미터 계산 중...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # === [7] 결과 저장 ===
    np.savez("camera_calib_data.npz", camera_matrix=mtx, dist_coeffs=dist)
    print("\n✅ [완료] 내부 파라미터 저장됨: camera_calib_data.npz")
    print("Camera matrix:\n", mtx)
    print("Distortion coefficients:\n", dist)
else:
    print("❗ 저장된 이미지 수가 부족합니다. 최소 10장 이상 필요합니다.")
