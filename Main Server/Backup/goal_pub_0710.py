import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class GoalSequencePublisher(Node):
    def __init__(self):
        super().__init__('goal_sequence_publisher')

        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.pose_subscriber_ = self.create_subscription(
            PoseStamped,
            'cam_point',  # 로봇 현재 위치
            self.robot_pose_callback,
            10
        )

        # 이동할 목표 좌표들 (x, y, theta_deg)
        self.goals = [
            (0.6, 0.6, 176.0),
            (0.6, 0.55, -88.0),
            (0.63, 0.4, -89.0),
            (0.6, 0.25, -87.0),
            (0.6, 0.01, -86.0),
            (0.62, 0.05, 175.0),
            (0.7, 0.04, 175.0)
        ]
        self.current_goal_index = 0
        self.goal_tolerance = 0.05  # 위치 허용 오차 (m)
        self.angle_tolerance = math.radians(15)  # 각도 허용 오차 (rad)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.goal_sent = False

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("GoalSequencePublisher 시작됨")

    def robot_pose_callback(self, msg: PoseStamped):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_theta = self.get_yaw_from_quaternion(msg.pose.orientation)

    def timer_callback(self):
        if self.current_goal_index >= len(self.goals):
            self.get_logger().info("모든 목표 좌표에 도달했습니다.")
            return

        goal_x, goal_y, goal_theta_deg = self.goals[self.current_goal_index]

        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance = math.sqrt(dx * dx + dy * dy)

        target_theta = math.radians(goal_theta_deg)
        angle_diff = self.normalize_angle(target_theta - self.current_theta)

        if distance < self.goal_tolerance and abs(angle_diff) < self.angle_tolerance:
            self.get_logger().info(f"{self.current_goal_index+1}번 좌표 도착")
            self.current_goal_index += 1
            self.goal_sent = False
        elif not self.goal_sent:
            self.publish_goal(goal_x, goal_y, goal_theta_deg)
            self.goal_sent = True

    def publish_goal(self, x, y, yaw_deg):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0

        yaw_rad = math.radians(yaw_deg)
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)

        self.publisher_.publish(msg)
        self.get_logger().info(f"📤 목표 퍼블리시: x={x:.2f}, y={y:.2f}, θ={yaw_deg}°")

    @staticmethod
    def get_yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = GoalSequencePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
