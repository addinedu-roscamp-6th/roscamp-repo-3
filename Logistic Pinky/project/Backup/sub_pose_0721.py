import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math
from std_msgs.msg import Bool
import time

class SimpleRobotController(Node):
    def __init__(self):
        super().__init__('simple_robot_controller_0721')

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

        self.goal_pose = None
        self.arrived = True

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.linear_speed = 0.01
        self.position_tolerance = 0.1
        self.angle_tolerance = math.radians(30)

        # PID 제어 변수
        self.kp = 0.15
        self.ki = 0.001
        self.kd = 0.03

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = self.get_clock().now()

        self.max_angular_speed = 0.08
        self.min_angular_speed = 0.02

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

        # PID 초기화
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = self.get_clock().now()

    def control_loop(self):
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

        if distance > self.position_tolerance:
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
        else:
            # 도착 → 방향 정렬
            angle_error = self.normalize_angle(goal_theta - self.current_theta)
            abs_error = abs(angle_error)

            if abs_error > self.angle_tolerance:
                cmd.linear.x = 0.0

                # 현재 시간 계산
                now = self.get_clock().now()
                dt = (now - self.prev_time).nanoseconds / 1e9
                self.prev_time = now

                # PID 제어 계산
                self.integral += angle_error * dt
                derivative = (angle_error - self.prev_error) / dt if dt > 0 else 0.0
                output = self.kp * angle_error + self.ki * self.integral + self.kd * derivative
                self.prev_error = angle_error

                # 회전 방향 (부호) 및 클리핑
                cmd.angular.z = max(self.min_angular_speed, min(abs(output), self.max_angular_speed))
                cmd.angular.z *= -1.0 if angle_error > 0 else 1.0

                self.get_logger().info(f"PID 회전 제어: 에러 {math.degrees(angle_error):.1f}°, 속도: {cmd.angular.z:.3f}")
            else:
                self.arrived = True
                self.publish_stop()
                self.get_logger().info("✅ 목표 위치 및 방향에 도착!")
                return

        self.cmd_vel_pub.publish(cmd)

    def publish_stop(self):
        self.cmd_vel_pub.publish(Twist())
        if self.waiting:
            now = self.get_clock().now()
            elapsed = (now - self.start_time).nanoseconds / 1e9
            if elapsed >= 5.0:
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
