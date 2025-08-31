import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
import math
from tf_transformations import quaternion_from_euler  # 꼭 설치 필요!

class ArucoRobotLocalizer(Node):
    def __init__(self):
        super().__init__('aruco_robot_localizer')

        self.publisher_ = self.create_publisher(PoseStamped, 'cam_point', 10)

        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error('카메라 열기 실패!')
            exit()

        data = np.load("/home/jun/kang/src/project/project/camera_calib_data.npz")
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs = data['dist_coeffs']

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.marker_length = 0.05

        self.marker_map_coords = {
            0: (0.0, 0.0),
            1: (1.0, 0.0),
            2: (0.0, 0.5)
        }

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("ArUco 기반 PoseStamped 퍼블리시 시작")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("카메라 프레임 읽기 실패")
            return

        frame = frame.copy()[120:390, 60:590]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None:
            return

        pixel_points = []
        map_points = []
        marker4_center = None
        marker4_rvec = None

        # === 마커 정렬 및 정보 수집 ===
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

        for i in range(len(ids)):
            marker_id = int(ids[i][0])
            center = np.mean(corners[i][0], axis=0)
            if marker_id in self.marker_map_coords:
                pixel_points.append(center)
                map_points.append(self.marker_map_coords[marker_id])
            elif marker_id == 4:
                marker4_center = center
                marker4_rvec = rvecs[i][0]

        if len(pixel_points) < 3 or marker4_center is None or marker4_rvec is None:
            self.get_logger().info("기준 마커 3개 또는 ID 4 마커가 모두 필요함")
            return

        # === 픽셀 → 맵 변환 ===
        pixel_pts = np.array(pixel_points, dtype=np.float32)
        map_pts = np.array(map_points, dtype=np.float32)

        transform_matrix = cv2.getAffineTransform(pixel_pts[:3], map_pts[:3])
        pixel_coord = np.array([marker4_center[0], marker4_center[1], 1.0], dtype=np.float32)
        map_coord = transform_matrix @ pixel_coord

        # === rvec → 회전 행렬 → yaw → quaternion 변환 ===
        R, _ = cv2.Rodrigues(marker4_rvec)
        yaw = math.atan2(R[1, 0], R[0, 0])
        qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)

        # === PoseStamped 메시지 생성 ===
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = round(float(map_coord[0]), 2)
        msg.pose.position.y = round(float(map_coord[1]), 2)
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.publisher_.publish(msg)
        self.get_logger().info(f"📌 Pose → x: {msg.pose.position.x}, y: {msg.pose.position.y}, yaw: {math.degrees(yaw):.1f}°")

        for i in range(len(ids)):
            marker_id = int(ids[i][0])
            corner = corners[i][0]
            center = np.mean(corner, axis=0).astype(int)
            cv2.putText(frame, f"ID:{marker_id}", (center[0]-20, center[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("Aruco Pose Tracker", frame)
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
