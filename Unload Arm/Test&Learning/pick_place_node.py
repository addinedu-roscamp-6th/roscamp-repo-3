#!/usr/bin/env python3
"""
ROS 2 노드: JetCobot + April/ArUco Pick‑&‑Place
================================================
* 카메라에서 태그 검출 → solvePnP 6D 포즈
* 휴리스틱 변환 → JetCobot TCP 좌표
* 그리퍼 Pick → 홈 복귀

OpenCV 4.4 ~ 4.10 모두 동작 (4.7↓ 폴백 포함)
"""

from __future__ import annotations
from std_srvs.srv import SetBool

import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from pymycobot.mycobot280 import MyCobot280
from pymycobot.genre import Angle
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge            # ★ NEW
from sensor_msgs.msg import Image         # ★ NEW
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy   # ★ 추가

# ─────────────────────────────────────────────────────────────────────┐
#  Utilities                                                          │
# ─────────────────────────────────────────────────────────────────────┘

def load_calib_npz(path: Path):
    data = np.load(str(path))
    k = data["camera_matrix"] if "camera_matrix" in data.files else data["K"]
    d = data["dist_coeffs"] if "dist_coeffs" in data.files else data["dist"]
    return k, d

def cam_to_robot(tvec, base, z_off=-160.0, xy_scale=1.0, x_off=12.0, y_off=-170.0):
    tx, ty, tz = tvec
    bx, by, bz, rx, ry, rz = base
    gx = bx + tz + z_off
    gy = by - tx * xy_scale + x_off + 7
    gz = bz + ty * xy_scale + y_off - 5

    if gx < 250:
        gx += 5
    elif 250 < gx < 279:
        gx -= 5
    elif 280 < gx < 290:
        gx += 20
    elif gx > 290:
        gx += 5; rx += 10; ry -= 10; rz -= 5

    if gz < 110:
        gz += 50
    '''elif gz > 115:
        gz -= 30'''

    return [gx, gy, gz], [rx, ry, rz]

# ─────────────────────────────────────────────────────────────────────┐
#  ROS 2  Node                                                        │
# ─────────────────────────────────────────────────────────────────────┘

class PickPlaceNode(Node):
    def __init__(self):
        super().__init__("jetcobot_pickplace")
        self.Z_CONST_MM = 150.0    # 예: 카메라-기준 물체까지 거리 300 mm
        self.processed: set[int] = set()   # ← 전역 대신 인스턴스 변수

        # ───────── Parameters ─────────
        self.declare_parameter("camera_device", "/dev/video0")
        #self.declare_parameter("marker_length_mm", 20.0)
        self.declare_parameter("tag_dict", "APRILTAG_36h11")
        self.declare_parameter("port", "/dev/ttyJETCOBOT")
        self.declare_parameter("baud", 1_000_000)
        self.declare_parameter("speed", 50)
        #self.declare_parameter("target_id", -1)
        self.declare_parameter("show_axes", True)
        self.declare_parameter("home_coords", [156.7, -43.0, 295.5, -161.23, -14.97, -36.5])
        self.declare_parameter("place_coords",[260.6, -133.7, 155.4, -169.07, -15.29, -50.88])
        self.place_coords = list(self.get_parameter("place_coords").value)

        #self.marker_len = self.get_parameter("marker_length_mm").value
        self.speed      = self.get_parameter("speed").value
        #self.target_id  = self.get_parameter("target_id").value
        self.show_axes  = self.get_parameter("show_axes").value
        self.paused = False               # ② 상태 변수

        # ───────── Robot ─────────
        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value
        try:
            self.mc = MyCobot280(port, baud)
            self.mc.thread_lock = True
            self.get_logger().info("MyCobot 연결 완료")
        except Exception as e:
            self.mc = None
            self.get_logger().warning(f"로봇 연결 실패 → 데모 모드: {e}")

        # ───────── Camera ─────────
        #device = self.get_parameter("camera_device").value
        #self.cap = cv2.VideoCapture(device)
        #if not self.cap.isOpened():
            #raise RuntimeError(f"카메라 열기 실패: {device}")

        # ───────── Calibration ─────────
        pkg_share = Path(get_package_share_directory("jetcobot_pickplace"))
        calib_path = pkg_share / "config" / "calibration.npz"
        self.K, self.dist = load_calib_npz(calib_path)

        # ───────── Home pose ─────────
        self.home_coords = list(self.get_parameter("home_coords").value)
        if self.mc:
            self.mc.send_coords(self.home_coords, self.speed)
            time.sleep(1)
            self.mc.set_gripper_value(100, self.speed)

        # ── YOLO BoundingBox 구독 추가 ───────────────────────────
        from vision_msgs.msg import Detection2DArray   # 파일 상단 import 해도 OK
        self.sub_bbox = self.create_subscription(
            Detection2DArray,         # 메시지 타입
            '/yolov5/bboxes',         # 토픽 이름 (YOLO 노드가 publish)
            self.bbox_cb,             # 콜백 함수
            10)                       # QoS(depth)
        # ----------------------------------------------------------

        # ───────── Timer ─────────
        self.timer = self.create_timer(1 / 30.0, self.loop)
        self.get_logger().info("노드 초기화 완료 – 박스 감지 시작")

        # ── ★ 이미지 퍼블리셔 추가 ──────────────────────
        '''self.bridge   = CvBridge()'''

        be_qos = QoSProfile(
            reliability = ReliabilityPolicy.RELIABLE,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 5)
        
        '''self.pub_raw    = self.create_publisher(Image, "/image_raw",    be_qos)
        self.pub_tagged = self.create_publisher(Image, "/image_tagged", be_qos)'''

        '''self.cam_pub  = self.create_publisher(
                            Image,            # 타입
                            "/image_raw",     # 토픽 이름
                            reliable_qos)  # Queue depth'''

        #self.cam_pub = self.create_publisher(Image, "/image_raw", 1)

        # ✨ ① OpenCV 창 생성 (한 번만)
        '''cv2.namedWindow("tag", cv2.WINDOW_NORMAL)'''
        
        # ── ★ 안정화 파라미터 & 상태 변수 ★ ─────────────────────────
        self.blur_th      = 100          # 모션블러 임계값 (조정)
        self.diff_th_mm   = 5.0          # 프레임간 위치 변화 허용치
        self.need_stable  = 6            # 연속으로 N프레임 안정해야 Pick
        self.last_tvec    = None
        self.stable_cnt   = 0
        # ──────────────────────────────────────────────────────────────

        # ──────────────────── 물건 옮긴 카운트 ────────────────────
        self.moved_cnt = 0             # 총 이동 건수
        # ──────────────────────────────────────────────────────────────
        # 서비스 서버 : 버튼(또는 CLI)에서 호출
        self.create_service(SetBool,       # ③
                            'pause_pickplace',
                            self.pause_cb)

        self.robot_busy = False        # ── __init__ 맨 아래에

        self.stable = {}          # ★ 태그별 안정도 기록 사전

    # ④ 서비스 콜백 함수
    def pause_cb(self, request:SetBool.Request, response:SetBool.Response):
        self.paused = request.data           # true ▶ 정지, false ▶ 재시작
        state      = "PAUSE" if self.paused else "RESUME"
        self.get_logger().info(f"[BTN] {state} 요청 수신")
        response.success = True
        response.message = state
        return response

    # ─────────────────────────────────────────────
    #  YOLO Bounding-Box 콜백
    # ─────────────────────────────────────────────
    def bbox_cb(self, msg: Detection2DArray):
        """YOLO 박스 처리"""
        if self.robot_busy or self.paused or not msg.detections:
            return

        det = max(msg.detections,
                key=lambda d: d.bbox.size_x * d.bbox.size_y)
        u, v = int(det.bbox.center.x), int(det.bbox.center.y)

        # Z 고정값 사용
        Z = self.Z_CONST_MM / 1000.0
        X = (u - self.K[0, 2]) * Z / self.K[0, 0]
        Y = (v - self.K[1, 2]) * Z / self.K[1, 1]
        tvec = np.array([X, Y, Z]) * 1000

        base = self.mc.get_coords()
        goal_pos, goal_rot = cam_to_robot(tvec, base)
        goal = goal_pos + goal_rot

        try:
            self.robot_busy = True
            # pick
            self.mc.send_coords(goal, self.speed, 1); time.sleep(2)
            self.mc.set_gripper_value(0, self.speed); time.sleep(1)
            # home
            self.mc.send_coords(self.home_coords, self.speed, 1); time.sleep(2)
            # place
            self.mc.send_coords(self.place_coords, self.speed, 1); time.sleep(2)
            self.mc.set_gripper_value(100, self.speed); time.sleep(1)
            # back to home
            self.mc.send_coords(self.home_coords, self.speed, 1); time.sleep(1)

            self.moved_cnt += 1
            self.get_logger().info(f"[YOLO] 이동 완료, 총 {self.moved_cnt}")
        finally:
            self.robot_busy = False


    def loop(self):
        """
        • 로봇이 움직이는 동안에는 V4L2 버퍼만 비우고 리턴
        • 일시정지(p) 상태면 검은 화면
        • 그 외에는 카메라 프리뷰만 보여 줌
        (YOLO → bbox_cb() 에서 pick-&-place 트리거)
        """
        # ─── ① 로봇 동작 중 ───────────────────────
        if self.robot_busy:
            for _ in range(3):          # 버퍼 날려서 지연 최소화
                self.cap.grab()
            return

        # ─── ② 일시정지 상태 ─────────────────────
        if self.paused:
            cv2.imshow("cam", np.zeros((40, 40, 3), np.uint8))
            cv2.waitKeyEx(10)
            return

        # ─── ③ 프리뷰 ────────────────────────────
        ret, frame = self.cap.read()
        if ret:
            cv2.imshow("cam", frame)

        # ─── ④ 키 입력 ───────────────────────────
        key = cv2.waitKeyEx(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("keyboard q → shutdown")
            rclpy.shutdown()
        elif key == ord('p'):
            self.paused = True
            self.get_logger().info("keyboard p → PAUSE")
        elif key == ord('r'):
            self.paused = False
            self.get_logger().info("keyboard r → RESUME")

# ─────────────────────────────────────────────────────────────────────┐
#  Main                                                               │
# ─────────────────────────────────────────────────────────────────────┘

def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
