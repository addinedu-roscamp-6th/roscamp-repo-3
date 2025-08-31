import os
import math
import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

CALIB_PATH = '/home/jun/kang/src/project/project/camera_calib_data.npz'
MARKER_WORLD = {0:(0.0, 0.0), 1:(1.0, 0.0), 2:(0.0, 1.0), 3:(1.0, 1.0)}
GOALS = [(0.89, 0.24, 120.0), (0.89, 0.7, 180.0), (0.3, 0.7, -90.0)]


def build_affine(corners, ids):
    img_pts, world_pts = [], []
    for i, mid in enumerate(ids.flatten()):
        if mid in MARKER_WORLD:
            c = corners[i][0]
            img_pts.append(np.mean(c, axis=0))
            world_pts.append(MARKER_WORLD[mid])
    if len(img_pts) < 3:
        return None
    return cv2.getAffineTransform(np.array(img_pts[:3], np.float32), np.array(world_pts[:3], np.float32))


def pixel_to_world(A, px, py):
    pt = np.array([px, py, 1.0], dtype=np.float32)
    res = A @ pt
    return float(res[0]), float(res[1])


class ArucoAutoNav(Node):
    def __init__(self):
        super().__init__('aruco_auto_nav')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        data = np.load(CALIB_PATH)
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs = data['dist_coeffs']

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_100)
        self.aruco_params = aruco.DetectorParameters()
        self.affine_matrix = None

        self.goals = GOALS
        self.goal_index = 0
        self.goal_tolerance = 0.05
        self.angle_tolerance = math.radians(10)

        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error('카메라 열기 실패')
            rclpy.shutdown()

        self.create_timer(1/30.0, self.loop)

    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        roi = frame[120:390, 60:590]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None or 4 not in ids:
            return

        self.affine_matrix = build_affine(corners, ids)
        if self.affine_matrix is None:
            return

        i4 = list(ids.flatten()).index(4)
        center_px = np.mean(corners[i4][0], axis=0)
        robot_x, robot_y = pixel_to_world(self.affine_matrix, center_px[0], center_px[1])

        # Orientation
        rvecs, _, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, self.camera_matrix, self.dist_coeffs)
        rvec = rvecs[i4][0]
        R, _ = cv2.Rodrigues(rvec)
        yaw = math.atan2(R[1, 0], R[0, 0])

        # 목표 위치 가져오기
        if self.goal_index >= len(self.goals):
            self.get_logger().info("모든 목표에 도달함")
            return

        gx, gy, gtheta = self.goals[self.goal_index]
        dx = gx - robot_x
        dy = gy - robot_y
        dist = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_goal - yaw)

        if dist < self.goal_tolerance and abs(self.normalize_angle(math.radians(gtheta) - yaw)) < self.angle_tolerance:
            self.get_logger().info(f"목표 {self.goal_index+1} 도달: ({gx:.2f}, {gy:.2f})")
            self.goal_index += 1
            return

        # 속도 제어
        twist = Twist()
        twist.linear.x = np.clip(0.2 * dist * max(0, math.cos(angle_diff)), 0.0, 0.15)
        twist.angular.z = np.clip(1.0 * angle_diff, -0.5, 0.5)
        self.pub.publish(twist)

        # 시각화
        aruco.drawDetectedMarkers(roi, corners, ids)
        cv2.putText(roi, f"Pose: ({robot_x:.2f}, {robot_y:.2f})", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow("ArucoNav", roi)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def destroy(self):
        self.cap.release()
        cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = ArucoAutoNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()