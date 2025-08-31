import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoMarkerPublisher(Node):
    def __init__(self):
        super().__init__('aruco_marker')

        # ROS publisher
        self.publisher_ = self.create_publisher(PointStamped, 'cam_point', 10)

        # 카메라 열기
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error('카메라 열기 실패!')
            exit()

        # # Calibration 데이터 불러오기
        data = np.load("/home/jun/kang/src/project/project/camera_calib_data.npz")
        self.camera_matrix = data["camera_matrix"]
        self.dist_coeffs = data["dist_coeffs"]

        # ArUco 세팅
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.marker_length = 0.05  # 마커 길이 (단위 m)

        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

        self.get_logger().info("✅ ArUco 마커 인식 노드 시작!")


    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("카메라 프레임을 읽지 못했습니다.")
            return

        frame = frame.copy()[120:390, 60:590]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

            for i in range(len(ids)):
                marker_id = ids[i][0]
                tvec = tvecs[i][0]  # [x, y, z]

                # 좌표 보정
                robot_x = tvec[0] + 0.75
                robot_y = -tvec[1] + 0.03

                # 메시지 생성 및 퍼블리시
                msg = PointStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_frame'  # 나중에 TF 변환 시 기준 프레임
                msg.point.x = round(robot_x, 2)
                msg.point.y = round(robot_y, 2)
                msg.point.z = round(tvec[2], 2)

                self.publisher_.publish(msg)
                self.get_logger().info(
                    f"마커 {marker_id} → x: {robot_x:.2f}, y: {robot_y:.2f}, z: {tvec[2]:.2f}")

                # 마커 중심 좌표에 표시
                corner = corners[i][0]
                center = np.mean(corner, axis=0).astype(int)
                pos_text = f"ID:{marker_id} [{robot_x:.2f}, {robot_y:.2f}]"
                cv2.putText(frame, pos_text, (center[0]-50, center[1]-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # 디버깅용: 윈도우에 표시
        cv2.imshow("Aruco Marker Viewer", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("종료 요청")
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main():
    rclpy.init()
    node = ArucoMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("노드 종료")
    node.cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
