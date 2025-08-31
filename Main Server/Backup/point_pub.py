import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, "goal_pose", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1초마다 실행
        self.count = 0

    def timer_callback(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'  # 좌표계 기준
        pose.header.stamp = self.get_clock().now().to_msg()

        # 위치 설정
        pose.pose.position.x = 0.3
        pose.pose.position.y = 0.3
        pose.pose.position.z = 0.0 + self.count * 2.0
        if pose.pose.position.z > 360:
            pose.pose.position.z = 0

        self.publisher_.publish(pose)
        self.get_logger().info(f"📤 Published PoseStamped: x={pose.pose.position.x}")
        self.get_logger().info(f"📤 Published PoseStamped: y={pose.pose.position.y}")
        self.get_logger().info(f"📤 Published PoseStamped: z={pose.pose.position.z}")

        self.count += 1

def main():
    rclpy.init()
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()