import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2
import math

class ArucoRobotLocalizer(Node):
    def __init__(self):
        super().__init__('aruco_pose_pub')

        self.publisher_ = self.create_publisher(PoseStamped, 'cam_point', 10)

        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error('❌ 카메라 열기 실패!')
            exit()

        data = np.load("/home/jun/kang/src/project/project/camera_calib_data.npz")
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs = data['dist_coeffs']

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.marker_length = 0.05  # meter

        # 기준 마커들의 맵 좌표 (x, y)
        self.marker_map_coords = {
            0: (0.0, 0.0),
            1: (1.0, 0.0),
            2: (0.0, 1.0),
            3: (1.0, 1.0)
        }

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("✅ ArUco 기반 로봇 위치 추정 시작")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("⚠️ 카메라 프레임 읽기 실패")
            return

        frame = frame.copy()[120:390, 60:590]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None:
            return

        pixel_points = []
        map_points = []
        marker4_index = -1

        for i in range(len(ids)):
            marker_id = int(ids[i][0])
            center = np.mean(corners[i][0], axis=0)

            if marker_id in self.marker_map_coords:
                pixel_points.append(center)
                map_points.append(self.marker_map_coords[marker_id])
            elif marker_id == 4:
                marker4_index = i

        if len(pixel_points) < 3 or marker4_index == -1:
            self.get_logger().info("ℹ️ 기준 마커 3개 이상과 ID 4 필요")
            return

        # Affine 변환 계산
        pixel_pts = np.array(pixel_points[:3], dtype=np.float32)
        map_pts = np.array(map_points[:3], dtype=np.float32)
        transform_matrix = cv2.getAffineTransform(pixel_pts, map_pts)

        # ID 4 마커의 위치 및 자세 추정
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length,
                                                              self.camera_matrix, self.dist_coeffs)

        rvec = rvecs[marker4_index][0]
        tvec = tvecs[marker4_index][0]
        center_px = np.mean(corners[marker4_index][0], axis=0)
        pixel_coord = np.array([center_px[0], center_px[1], 1.0], dtype=np.float32)
        map_coord = transform_matrix @ pixel_coord

        # 회전 (yaw) 계산
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        # theta_rad = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        # qz = math.sin(theta_rad / 2.0)
        # qw = math.cos(theta_rad / 2.0)

        theta_rad = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        qz = np.sin(theta_rad / 2.0)
        qw = np.cos(theta_rad / 2.0)

        # PoseStamped 메시지 생성
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = round(float(map_coord[0]), 2)
        msg.pose.position.y = round(float(map_coord[1]), 2)
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        # msg.pose.orientation.z = qz
        # msg.pose.orientation.w = qw
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)

        self.publisher_.publish(msg)
        self.get_logger().info(f"📍 로봇 위치: ({msg.pose.position.x}, {msg.pose.position.y}, θ={round(math.degrees(theta_rad), 1)}°)")

        for i in range(len(ids)):
            marker_id = int(ids[i][0])
            corner = corners[i][0]
            center = np.mean(corner, axis=0).astype(int)
            cv2.putText(frame, f"ID:{marker_id}", (center[0]-20, center[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("Aruco Robot Tracker", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main():
    rclpy.init()
    node = ArucoRobotLocalizer()
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
