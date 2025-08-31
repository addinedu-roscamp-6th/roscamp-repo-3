import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class ArucoTFPublisher(Node):
    def __init__(self):
        super().__init__('aruco_tf2_publisher')

        self.br = TransformBroadcaster(self)
        self.publisher_ = self.create_publisher(PoseStamped, 'cam_point', 10)

        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error('카메라 열기 실패!')
            exit()

        data = np.load("/home/jun/kang/src/project/project/camera_calib_data.npz")
        self.camera_matrix = data["camera_matrix"]
        self.dist_coeffs = data["dist_coeffs"]

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.marker_length = 0.05

        # 마커 ID → map 좌표 (직접 설정)
        self.marker_map_coords = {
            0: (0.0, 0.0),
            1: (1.0, 0.0),
            2: (0.0, 0.5),
        }

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("✅ ArUco 마커 + TF 노드 시작")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("카메라 프레임 못 읽음")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                if marker_id not in self.marker_map_coords:
                    continue

                tvec = tvecs[i][0]  # [x, y, z]
                rvec = rvecs[i][0]

                # 마커의 map 위치 기준 좌표 가져오기
                map_x, map_y = self.marker_map_coords[marker_id]

                # 로봇 좌표 추정 (마커 위치 자체가 로봇 위치라고 가정)
                robot_x = map_x
                robot_y = map_y

                # 회전 추정 (yaw만 추정)
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
                quat = tf_transformations.quaternion_from_euler(0, 0, yaw)

                # TF 전송
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'map'
                t.child_frame_id = f'robot_{marker_id}'
                t.transform.translation.x = robot_x
                t.transform.translation.y = robot_y
                t.transform.translation.z = 0.0
                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]
                self.br.sendTransform(t)

                # PoseStamped 퍼블리시
                pose_msg = PoseStamped()
                pose_msg.header = t.header
                pose_msg.pose.position.x = robot_x
                pose_msg.pose.position.y = robot_y
                pose_msg.pose.position.z = 0.0
                pose_msg.pose.orientation.x = quat[0]
                pose_msg.pose.orientation.y = quat[1]
                pose_msg.pose.orientation.z = quat[2]
                pose_msg.pose.orientation.w = quat[3]

                self.publisher_.publish(pose_msg)

def main():
    rclpy.init()
    node = ArucoTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
