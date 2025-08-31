import cv2
import numpy as np
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs

# === [1] 캘리브레이션 파일 불러오기 ===
data = np.load("camera_calib_data.npz")
camera_matrix = data["camera_matrix"]
dist_coeffs = data["dist_coeffs"]

# === [2] ArUco 설정 ===
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
aruco_params = cv2.aruco.DetectorParameters()

# === [3] 카메라 열기 ===
cap = cv2.VideoCapture(2)
if not cap.isOpened():
    print("❌ 카메라 열기 실패!")
    exit()

marker_length = 0.05  # 마커 한 변의 길이 (단위: m)

print("[INFO] ArUco 마커 인식 시작 ('q' 누르면 종료)")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = frame.copy()[120:390,60:590]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # === [4] 마커 감지 ===
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # === [5] 마커 위치 추정 ===
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

        for i in range(len(ids)):
            marker_id = ids[i][0]
            tvec = tvecs[i][0]  # [x, y, z]
            rvec = rvecs[i][0]
            robot_x = tvec[0] + 0.75
            robot_y = -tvec[1] +0.03
            print(f"x:{robot_x:.2f}, y:{robot_y:.2f}")
            position_str = f"ID:{marker_id} Pos: [{robot_x:.2f}, {robot_y:.2f}, {tvec[2]:.2f}]"
            # position_str = f"ID:{marker_id} Pos: [{tvec[0]:.2f}, {tvec[1]:.2f}, {tvec[2]:.2f}]"
            # theta_rad = np.arctan2(R[1, 0], R[0, 0])
            # theta_deg = np.degrees(rvec[0])
            # print(f"ID:{marker_id} Pos: [{tvec[0]:.2f}, {tvec[1]:.2f}, {tvec[2]:.2f}]")
            # print(f"ID:{marker_id} rvec[0]:{rvec[0]:.2f}, rvec[1]:{rvec[1]:.2f}, rvec[2]:{rvec[2]:.2f}")
            # print(theta_deg)
            # 마커 중심 좌표에 텍스트로 표시
            corner = corners[i][0]
            center = np.mean(corner, axis=0).astype(int)
            cv2.putText(frame, position_str, (center[0] - 50, center[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    cv2.imshow("Aruco Marker Viewer", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
