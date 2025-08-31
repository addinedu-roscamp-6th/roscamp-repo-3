import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2
import math
from scipy.spatial.transform import Rotation as R # scipy.spatial.transform 추가

class ArucoRobotLocalizer(Node):
    def __init__(self):
        super().__init__('gemi_aruco_pose_pub')

        self.publisher_ = self.create_publisher(PoseStamped, 'cam_point', 10)

        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error('❌ 카메라 열기 실패!')
            exit()

        # 카메라 캘리브레이션 데이터 로드 (경로 확인 필수!)
        try:
            data = np.load("/home/jun/kang/src/project/project/camera_calib_data.npz")
            self.camera_matrix = data['camera_matrix']
            self.dist_coeffs = data['dist_coeffs']
        except FileNotFoundError:
            self.get_logger().error('❌ camera_calib_data.npz 파일을 찾을 수 없습니다. 경로를 확인해주세요.')
            exit()
        except KeyError:
            self.get_logger().error('❌ camera_calib_data.npz 파일에 camera_matrix 또는 dist_coeffs가 없습니다.')
            exit()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.marker_length = 0.05  # meter, 실제 Aruco 마커의 한 변 길이

        # 기준 마커들의 맵 좌표 (x, y, z) - Z는 0으로 가정
        # 이 값들은 실제 맵과 마커 배치에 맞게 정확히 측정되어야 합니다.
        self.marker_map_coords = {
            0: (0.0, 0.0, 0.0),
            1: (1.0, 0.0, 0.0),
            2: (0.0, 1.0, 0.0),
            3: (1.0, 1.0, 0.0)
        }
        
        self.robot_marker_id = 4 # 로봇에 부착된 마커 ID

        # 로봇 마커와 로봇 중심(base_link) 간의 오프셋 (로봇 좌표계 기준)
        # 이 값은 로봇 마커가 로봇 본체의 어디에 부착되어 있는지에 따라 달라집니다.
        # 예: 로봇 마커가 로봇 중심에서 x축 방향으로 0.1m 앞에 부착.
        self.robot_marker_offset_x = 0.0
        self.robot_marker_offset_y = 0.0
        self.robot_marker_offset_z = 0.0 # 마커가 바닥에 평행하게 부착되었다면 0

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("✅ ArUco 기반 로봇 위치 추정 시작")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("⚠️ 카메라 프레임 읽기 실패")
            return

        # 크롭 영역 (카메라 캘리브레이션 시 사용된 이미지 크기와 일치하거나,
        # 캘리브레이션 파라미터가 크롭 영역에 맞게 조정되어야 합니다.)
        # frame = frame.copy()[120:390, 60:590] # 주석 처리: 캘리브레이션 후 크롭은 오차 유발 가능
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None:
            # self.get_logger().info("ℹ️ 인식된 마커 없음")
            # cv2.imshow("Aruco Robot Tracker", frame) # 마커 없어도 프레임 표시
            # cv2.waitKey(1)
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length,
                                                              self.camera_matrix, self.dist_coeffs)

        # 맵 마커와 로봇 마커의 pose 추출
        map_marker_poses_cam = {} # {id: {'rvec': ..., 'tvec': ...}}
        robot_marker_pose_cam = None

        for i in range(len(ids)):
            marker_id = int(ids[i][0])
            rvec = rvecs[i][0]
            tvec = tvecs[i][0]

            if marker_id in self.marker_map_coords:
                map_marker_poses_cam[marker_id] = {'rvec': rvec, 'tvec': tvec}
            elif marker_id == self.robot_marker_id:
                robot_marker_pose_cam = {'rvec': rvec, 'tvec': tvec}

            # 디버깅용 마커 정보 표시
            center = np.mean(corners[i][0], axis=0).astype(int)
            cv2.putText(frame, f"ID:{marker_id}", (center[0]-20, center[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length * 0.5)


        # 최소 3개의 맵 마커가 인식되어야 카메라-맵 변환을 안정적으로 추정할 수 있습니다.
        if len(map_marker_poses_cam) < 3:
            self.get_logger().info("ℹ️ 맵 마커 3개 이상 필요")
            cv2.imshow("Aruco Robot Tracker", frame)
            cv2.waitKey(1)
            return

        if robot_marker_pose_cam is None:
            self.get_logger().info("ℹ️ 로봇 마커 (ID 4) 인식 안 됨")
            cv2.imshow("Aruco Robot Tracker", frame)
            cv2.waitKey(1)
            return

        # 맵 마커들을 이용하여 카메라의 맵 좌표계에서의 pose 추정 (PnP 방식)
        obj_points = [] # 맵 좌표계의 3D 점
        img_points = [] # 카메라 영상의 2D 픽셀 점

        # 인식된 맵 마커들로부터 PnP에 필요한 데이터 추출
        for marker_id, data in map_marker_poses_cam.items():
            # 마커의 맵 좌표계에서의 각 코너의 3D 위치
            # ArUco 마커의 3D 모델 (Z=0인 평면)
            half_m = self.marker_length / 2.0
            marker_obj_coords = np.array([
                [-half_m, half_m, 0],
                [ half_m, half_m, 0],
                [ half_m, -half_m, 0],
                [-half_m, -half_m, 0]
            ], dtype=np.float32)

            # 마커의 맵 좌표계에서의 Pose (미리 정의된 self.marker_map_coords 사용)
            marker_map_pos = np.array(self.marker_map_coords[marker_id], dtype=np.float32)

            # 각 마커의 맵 좌표계에서의 방향 (yaw만 있다고 가정하면)
            # 여기서는 맵 마커의 방향은 0으로 가정합니다. 필요시 수정
            map_marker_rot_matrix = R.from_euler('z', 0, degrees=False).as_matrix() # 맵 마커의 초기 방향

            for i in range(4): # 각 마커의 4개 코너
                # 마커의 로컬 3D 코너를 맵 좌표계로 변환
                global_corner = np.dot(map_marker_rot_matrix, marker_obj_coords[i]) + marker_map_pos
                obj_points.append(global_corner)
                
                # 해당 코너의 이미지 픽셀 좌표
                img_points.append(corners[np.where(ids == marker_id)[0][0]][0][i])

        obj_points = np.array(obj_points, dtype=np.float32)
        img_points = np.array(img_points, dtype=np.float32)

        # 카메라의 맵 좌표계에서의 pose 추정 (solvePnP)
        # rvec_cam_map, tvec_cam_map은 카메라 -> 맵 변환의 역변환입니다.
        # 즉, 맵의 원점을 기준으로 카메라의 pose를 나타냅니다.
        success, rvec_cam_in_map, tvec_cam_in_map = cv2.solvePnP(obj_points, img_points, self.camera_matrix, self.dist_coeffs)

        if not success:
            self.get_logger().warning("⚠️ 카메라의 맵 pose 추정 실패!")
            cv2.imshow("Aruco Robot Tracker", frame)
            cv2.waitKey(1)
            return

        # 카메라의 맵 좌표계에서의 회전 행렬
        R_cam_in_map, _ = cv2.Rodrigues(rvec_cam_in_map)
        T_cam_in_map = np.eye(4)
        T_cam_in_map[:3, :3] = R_cam_in_map
        T_cam_in_map[:3, 3] = tvec_cam_in_map.flatten()

        # 맵 -> 카메라 변환 (T_map_cam = T_cam_in_map의 역변환)
        T_map_cam = np.linalg.inv(T_cam_in_map)

        # 로봇 마커의 카메라 좌표계 pose
        rvec_robot_cam = robot_marker_pose_cam['rvec']
        tvec_robot_cam = robot_marker_pose_cam['tvec']

        # 카메라 -> 로봇 마커 변환 행렬 (T_cam_robot_marker)
        R_cam_robot_marker, _ = cv2.Rodrigues(rvec_robot_cam)
        T_cam_robot_marker = np.eye(4)
        T_cam_robot_marker[:3, :3] = R_cam_robot_marker
        T_cam_robot_marker[:3, 3] = tvec_robot_cam.flatten()

        # 맵 -> 로봇 마커 변환 (T_map_robot_marker = T_map_cam @ T_cam_robot_marker)
        T_map_robot_marker = np.dot(T_map_cam, T_cam_robot_marker)

        # 로봇 마커와 로봇 본체 사이의 오프셋을 고려하여 로봇 본체의 맵 좌표계 pose 계산
        # 로봇 마커 프레임에서 로봇 본체 프레임으로의 변환 (T_marker_robot_body)
        T_marker_robot_body = np.eye(4)
        T_marker_robot_body[:3, 3] = np.array([self.robot_marker_offset_x,
                                               self.robot_marker_offset_y,
                                               self.robot_marker_offset_z])

        # 맵 -> 로봇 본체 변환 (T_map_robot_body = T_map_robot_marker @ T_marker_robot_body)
        T_map_robot_body = np.dot(T_map_robot_marker, T_marker_robot_body)

        robot_pos_map = T_map_robot_body[:3, 3]
        robot_rot_map_matrix = T_map_robot_body[:3, :3]

        # 회전 행렬을 쿼터니언으로 변환 (scipy 사용)
        r = R.from_matrix(robot_rot_map_matrix)
        robot_quat_map = r.as_quat() # (x, y, z, w)

        # PoseStamped 메시지 생성 및 발행
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(robot_pos_map[0])
        msg.pose.position.y = float(robot_pos_map[1])
        msg.pose.position.z = float(robot_pos_map[2]) # Z 좌표도 발행
        msg.pose.orientation.x = float(robot_quat_map[0])
        msg.pose.orientation.y = float(robot_quat_map[1])
        msg.pose.orientation.z = float(robot_quat_map[2])
        msg.pose.orientation.w = float(robot_quat_map[3])
        self.publisher_.publish(msg)

        # 로봇의 Yaw 값 출력 (디버깅용)
        _, _, current_yaw = r.as_euler('xyz') # Roll, Pitch, Yaw
        self.get_logger().info(f"📍 로봇 위치: (X:{msg.pose.position.x:.2f}, Y:{msg.pose.position.y:.2f}), Yaw:{math.degrees(current_yaw):.1f}°")

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
        node.get_logger().info("🛑 노드 종료.")
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()