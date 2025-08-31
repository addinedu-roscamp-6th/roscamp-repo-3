import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_publisher')

        # Publisher 설정
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # 목표 좌표 설정 (필요 시 여기를 수정)
        self.goal_x = 0.89  # meters
        self.goal_y = 0.24  # meters
        self.goal_theta_deg = 120.0  # degrees

        # 타이머 1회성으로 목표 퍼블리시
        self.timer = self.create_timer(1.0, self.publish_goal_once)
        self.published = False

    def publish_goal_once(self):
        if self.published:
            return

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'  # 로봇과 동일한 좌표계 사용

        # 위치 설정
        goal.pose.position.x = self.goal_x
        goal.pose.position.y = self.goal_y
        goal.pose.position.z = 0.0

        # 각도 설정 (yaw → quaternion)
        theta_rad = math.radians(self.goal_theta_deg)
        goal.pose.orientation.z = math.sin(theta_rad / 2.0)
        goal.pose.orientation.w = math.cos(theta_rad / 2.0)

        self.publisher_.publish(goal)
        self.get_logger().info(f"✅ 목표 좌표 퍼블리시: x={self.goal_x}, y={self.goal_y}, θ={self.goal_theta_deg}°")
        self.published = True

def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
