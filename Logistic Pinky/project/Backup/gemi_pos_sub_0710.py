import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math
from scipy.spatial.transform import Rotation as R
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class SimpleRobotController(Node):
    def __init__(self):
        super().__init__('gemi_simple_robot_controller')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.robot_pose_sub = self.create_subscription(
            PoseStamped,
            'cam_point',
            self.robot_pose_callback,
            qos_profile)

        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_pose_callback,
            qos_profile)

        self.goal_pose = None
        self.arrived = True

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.kp_linear = 0.5
        self.kp_angular = 0.3

        self.max_linear_vel = 0.25
        self.max_angular_vel = 1.0

        self.position_tolerance = 0.05
        self.angle_tolerance = math.radians(3.0)

        self.get_logger().info('SimpleRobotController 노드 시작! PID 제어 활성화.')

    def robot_pose_callback(self, msg: PoseStamped):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_theta = self.get_yaw_from_quaternion(msg.pose.orientation)

    def goal_pose_callback(self, msg: PoseStamped):
        self.goal_pose = msg
        self.arrived = False
        self.get_logger().info(f"✅ 목표 좌표 수신: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, θ={math.degrees(self.get_yaw_from_quaternion(msg.pose.orientation)):.1f}°")

    def control_loop(self):
        # cmd 변수를 생성합니다.
        cmd = Twist()

        if self.goal_pose is None or self.arrived:
            self.publish_stop()
            return

        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y
        goal_theta = self.get_yaw_from_quaternion(self.goal_pose.pose.orientation)

        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)

        if distance > self.position_tolerance:
            target_angle_to_point = math.atan2(dy, dx)
            angle_error_to_point = self.normalize_angle(target_angle_to_point - self.current_theta)

            if abs(angle_error_to_point) > self.angle_tolerance:
                angular_vel = self.kp_angular * angle_error_to_point
                # 여기서 twist_msg 대신 cmd를 사용합니다.
                cmd.angular.z = max(min(angular_vel, self.max_angular_vel), -self.max_angular_vel)
                cmd.linear.x = 0.0
                self.get_logger().info(f"로봇 회전 중. 목표 지점까지 각도 오차: {math.degrees(angle_error_to_point):.1f}°")
            else:
                linear_vel = self.kp_linear * distance
                # 여기서 twist_msg 대신 cmd를 사용합니다.
                cmd.linear.x = max(min(linear_vel, self.max_linear_vel), 0.0)
                cmd.angular.z = 0.0
                self.get_logger().info(f"로봇 전진 중. 남은 거리: {distance:.2f}m")
        else:
            final_angle_error = self.normalize_angle(goal_theta - self.current_theta)

            if abs(final_angle_error) > self.angle_tolerance:
                angular_vel = self.kp_angular * final_angle_error
                # 여기서 twist_msg 대신 cmd를 사용합니다.
                cmd.angular.z = max(min(angular_vel, self.max_angular_vel), -self.max_angular_vel)
                cmd.linear.x = 0.0
                self.get_logger().info(f"로봇 최종 방향 정렬 중. 최종 각도 오차: {math.degrees(final_angle_error):.1f}°")
            else:
                self.arrived = True
                self.get_logger().info("✅ 목표 위치 및 방향에 도착했습니다. 정지합니다.")
                self.publish_stop()
                return

        # 여기서 twist_msg 대신 cmd를 퍼블리시합니다.
        self.cmd_vel_pub.publish(cmd)

    def publish_stop(self):
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

    @staticmethod
    def get_yaw_from_quaternion(q):
        r = R.from_quat([q.x, q.y, q.z, q.w])
        roll, pitch, yaw = r.as_euler('xyz')
        return yaw

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    node = SimpleRobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 노드 종료.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()