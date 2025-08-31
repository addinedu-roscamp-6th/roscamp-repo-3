import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math

class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
    def send_goal(self, x, y, theta_deg):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        # 좌표 설정
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        # 방향(yaw → quaternion으로 변환)
        theta_rad = math.radians(theta_deg)
        qz = math.sin(theta_rad / 2.0)
        qw = math.cos(theta_rad / 2.0)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw
        # 액션 서버 연결 대기
        self.client.wait_for_server()
        self.get_logger().info(f"Sending goal: x={x}, y={y}, θ={theta_deg}°")
        send_goal_future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info(':x: Goal rejected.')
            return
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(':흰색_확인_표시: Goal completed!')
def main():
    rclpy.init()
    node = GoalSender()
    try:
        while True:
            user_input = input("좌표 입력 (x y θ), 예: 1.0 2.5 90 : ")
            if user_input.lower() == 'q':
                break
            try:
                x, y, theta = map(float, user_input.strip().split())
                node.send_goal(x, y, theta)
            except:
                print(":경고: 형식 오류! 예: 1.0 2.0 90")
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()