import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2
import math
import socket

class ArucoLocalizer(Node):
    def __init__(self):
        super().__init__('aruco_localizer')
        self.pub_4 = self.create_publisher(PoseStamped, 'cam_point_4', 10)
        self.pub_5 = self.create_publisher(PoseStamped, 'cam_point_5', 10)

        self.udp_ip = "192.168.1.11"
        self.udp_port = 2626
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # 카메라 열기
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error('❌ 카메라 열기 실패!')
            exit()

        # 카메라 보정 데이터 불러오기
        data = np.load("/home/jun/kang/src/project/project/camera_calib_data.npz")
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs = data['dist_coeffs']

        # ArUco 설정
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.marker_length = 0.05

        # 지도 좌표 (기준 마커 0~3)
        self.marker_map_coords = {
            0: (0.0, 0.0, 0.0),
            1: (1.0, 0.0, 0.0),
            2: (0.0, 1.0, 0.0),
            3: (1.0, 1.0, 0.0)
        }

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("✅ ArUco 기반 위치 추정 + UDP 영상 전송 시작")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("⚠️ 카메라 프레임 읽기 실패")
            return
        cv2.imshow("Global Camera Origin", frame)

        frame = frame.copy()[120:390, 60:590]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None:
            return

        pixel_points = []
        map_points = []
        marker4_index = -1
        marker5_index = -1

        for i in range(len(ids)):
            marker_id = int(ids[i][0])

            # ID 4, 5, 그리고 기준 마커만 처리
            if marker_id not in [4, 5] and marker_id not in self.marker_map_coords:
                continue

            center = np.mean(corners[i][0], axis=0)

            if marker_id in self.marker_map_coords:
                pixel_points.append(center)
                map_points.append(self.marker_map_coords[marker_id][:2])
            elif marker_id == 4:
                marker4_index = i
            elif marker_id == 5:
                marker5_index = i

        # 기준 마커 3개 이상 있어야 변환행렬 계산
        if len(pixel_points) < 3:
            self.get_logger().info("ℹ️ 기준 마커 3개 이상 필요")
            return

        pixel_pts = np.array(pixel_points[:3], dtype=np.float32)
        map_pts = np.array(map_points[:3], dtype=np.float32)
        transform_matrix = cv2.getAffineTransform(pixel_pts, map_pts)

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

        # ID 4 위치 퍼블리시
        if marker4_index != -1:
            self.publish_marker_pose(4, marker4_index, corners, transform_matrix, rvecs)
        else:
            self.get_logger().info("⚠️ 마커 ID 4 없음")

        # ID 5 위치 퍼블리시
        if marker5_index != -1:
            self.publish_marker_pose(5, marker5_index, corners, transform_matrix, rvecs)
        else:
            self.get_logger().info("⚠️ 마커 ID 5 없음")

        # 화면에 ID 4, 5 마커 채우기
        for i in range(len(ids)):
            marker_id = int(ids[i][0])
            pts = corners[i][0].astype(int)
            if marker_id == 4:
                cv2.fillPoly(frame, [pts], color=(0, 255, 0))  # 초록
            elif marker_id == 5:
                cv2.fillPoly(frame, [pts], color=(255, 0, 0))  # 파랑

        # UDP 영상 전송
        try:
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            self.sock.sendto(buffer.tobytes(), (self.udp_ip, self.udp_port))
        except Exception as e:
            self.get_logger().error(f"UDP 전송 오류: {e}")

        # 화면 표시
        cv2.imshow("Global Camera ROI", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def publish_marker_pose(self, marker_id, index, corners, transform_matrix, rvecs):
        center_px = np.mean(corners[index][0], axis=0)
        pixel_coord = np.array([center_px[0], center_px[1], 1.0], dtype=np.float32)
        map_xy = transform_matrix @ pixel_coord
        map_x, map_y = float(map_xy[0]), float(map_xy[1])

        rvec = rvecs[index][0]
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        theta_rad = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

        qz = np.sin(theta_rad / 2.0)
        qw = np.cos(theta_rad / 2.0)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = round(map_x, 2)
        msg.pose.position.y = round(map_y, 2)
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)

        self.get_logger().info(
            f"Marker ID {marker_id} 위치: ({msg.pose.position.x}, {msg.pose.position.y}), θ={round(math.degrees(theta_rad),1)}°"
        )

        if marker_id == 4:
            self.pub_4.publish(msg)
        if marker_id == 5:
            self.pub_5.publish(msg)

def main():
    rclpy.init()
    node = ArucoLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 종료")
    node.cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
