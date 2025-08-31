import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math
from std_msgs.msg import Bool

class SimpleRobotController(Node):
    def __init__(self):
        super().__init__('simple_robot_controller_0714')

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.robot_pose_sub = self.create_subscription(
            PoseStamped,
            'cam_point',
            self.robot_pose_callback,
            10)

        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_pose_callback,
            10)
        
        # self.goal_arrived = self.create_subscription(
        #     Bool,
        #     'goal_arrived',
        #     self.goal_arrived_callback,
        #     10)

        self.goal_pose = None
        self.arrived = True

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0


        self.linear_speed = 0.05
        self.max_angular_speed = 0.08     # 최대 회전 속도
        self.min_angular_speed = 0.02    # 최소 회전 속도
        self.angle_kp = 0.25             # 비례 계수
        self.position_tolerance = 0.1  # meter
        # self.angle_tolerance = math.radians(13)  # 허용 오차: 약 3도
        self.angle_tolerance = math.radians(16)  # 허용 오차: 약 3도

        self.waiting = True
        self.start_time = self.get_clock().now()

        self.get_logger().info('🚗 SimpleRobotController 노드 시작!')
        self.control_timer = self.create_timer(0.1, self.control_loop)

    def robot_pose_callback(self, msg: PoseStamped):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_theta = self.get_yaw_from_quaternion(msg.pose.orientation)

    def goal_pose_callback(self, msg: PoseStamped):
        self.goal_pose = msg
        self.arrived = False
        theta_deg = math.degrees(self.get_yaw_from_quaternion(msg.pose.orientation))
        self.get_logger().info(f"🎯 목표 좌표 수신 → x: {msg.pose.position.x:.2f}, y: {msg.pose.position.y:.2f}, θ: {theta_deg:.1f}°")

    # def goal_arrived_callback(self, msg: Bool):
    #     self.goal_arrived = msg

    def control_loop(self):
        # print(self.goal_pose)
        # print(self.arrived)
        if self.goal_pose is None or self.arrived:
            self.publish_stop()
            return

        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y
        goal_theta = self.get_yaw_from_quaternion(self.goal_pose.pose.orientation)

        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance = math.sqrt(dx * dx + dy * dy)

        cmd = Twist()

        # if self.goal_arrived:
        #     self.publish_stop()

        if distance > self.position_tolerance:
            print(distance)
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
        else:
            # 도착 → 방향 정렬
            final_angle_diff = self.normalize_angle(goal_theta - self.current_theta)
            abs_diff = abs(final_angle_diff)

            if abs_diff > self.angle_tolerance:
                cmd.linear.x = 0.0

                # 비례 회전 속도 계산
                angular_z = self.angle_kp * abs_diff
                angular_z = max(self.min_angular_speed, min(angular_z, self.max_angular_speed))

                # 방향: 양수면 반시계, 음수면 시계 방향
                cmd.angular.z = -angular_z if final_angle_diff > 0 else angular_z

                self.get_logger().info(f"방향 조정 중: 남은 각도 {math.degrees(final_angle_diff):.1f}°, 속도: {cmd.angular.z:.3f}")
            else:
                self.arrived = True
                self.publish_stop()
                self.get_logger().info("목표 위치 및 방향에 도착!")
                return

        self.cmd_vel_pub.publish(cmd)

    def publish_stop(self):
        self.cmd_vel_pub.publish(Twist())
        if self.waiting:
            now = self.get_clock().now()
            elapsed = (now - self.start_time).nanoseconds / 1e9
            if elapsed >= 3.0:
                self.waiting = False

    @staticmethod
    def get_yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = SimpleRobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 종료 요청")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
