import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class GoalSubscriber(Node):
    def __init__(self):
        super().__init__('goal_subscriber')

        self.subscriber_ = self.create_subscription(
            PoseStamped,
            'goal_point',  # ← 여기로 외부에서 PoseStamped 메시지가 들어옴
            self.goal_callback,
            10
        )

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("✅ 로봇 목표 수신 노드 실행 중")

    def goal_callback(self, msg):
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("❌ NavigateToPose 액션 서버 연결 실패")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg  # 외부에서 들어온 PoseStamped 메시지 그대로 사용

        self.get_logger().info(f"🚀 목표 수신: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        send_goal_future = self.client.send_goal_async(goal_msg)

        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('⛔ Goal rejected')
                return
            self.get_logger().info('✅ Goal accepted')

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)

        send_goal_future.add_done_callback(goal_response_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info("🏁 목표 도착!")

def main():
    rclpy.init()
    node = GoalSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
