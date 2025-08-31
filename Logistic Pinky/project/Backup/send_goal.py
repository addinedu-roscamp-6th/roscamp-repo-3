import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PointStamped
import math
from rclpy.qos import QoSProfile

class Navigator(Node):
    def __init__(self):
        super().__init__('goal_sender')
        qos = QoSProfile(depth=10)
        self.sub_ = self.create_subscription(PointStamped, "cam_point", self.data_callback, qos)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.x_point = 0.0
        self.y_point = 0.0

    def data_callback(self, data):
        print(data)

    def send_goal(self, x, y, theta_deg):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # 위치 설정
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # 방향(각도 → quaternion)
        theta_rad = math.radians(theta_deg)
        goal_msg.pose.pose.orientation.z = math.sin(theta_rad / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta_rad / 2.0)

        self.client.wait_for_server()
        self.get_logger().info(f"목표 전송: x={x}, y={y}, θ={theta_deg}°")
        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("❌ 목표가 거절됨")
            return

        self.get_logger().info("✅ 목표 수락됨. 이동 중...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info("🏁 목표 도달 완료!")

def main():
    rclpy.init()
    navigator = Navigator()

    try:
        while True:
            user_input = input("목표 좌표 입력 (x y θ), 예: 1.5 2.0 90 (q: 종료): ")
            if user_input.lower() == 'q':
                break
            try:
                x, y, theta = map(float, user_input.strip().split())
                navigator.send_goal(x, y, theta)
            except:
                print("⚠️ 입력 오류! 예: 1.0 2.0 90")
    except KeyboardInterrupt:
        pass

    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
