import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math
import time

class SimpleRobotController(Node):
    def __init__(self):
        super().__init__('simple_robot_controller')

        # cmd_vel 퍼블리셔 (로봇 구동용)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # 외부(천장 카메라)로부터 로봇 위치 수신
        self.robot_pose_sub = self.create_subscription(
            PoseStamped,
            'cam_point',
            self.robot_pose_callback,
            10)

        # 목표 좌표 수신
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_pose_callback,
            10)


        self.goal_pose = None
        self.arrived = True

        # 현재 위치 초기값
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.linear_speed = 0.005
        self.angular_speed = 0.005
        self.position_tolerance = 0.05
        self.angle_tolerance = 0.5
        # self.linear_speed = 0.2
        # self.angular_speed = 0.5
        # self.position_tolerance = 0.05
        # self.angle_tolerance = 0.1

        self.get_logger().info('SimpleRobotController 노드 시작!')

    def robot_pose_callback(self, msg: PoseStamped):
        # 외부에서 받은 로봇 위치로 현재 위치 업데이트
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_theta = self.get_yaw_from_quaternion(msg.pose.orientation)
        self.get_logger().debug(f"로봇 위치 업데이트: x={self.current_x:.2f}, y={self.current_y:.2f}, θ={math.degrees(self.current_theta):.1f}°")
        # self.get_logger().info(f"로봇 위치 업데이트: x={self.current_x:.2f}, y={self.current_y:.2f}, θ={math.degrees(self.current_theta):.1f}°")

    def goal_pose_callback(self, msg: PoseStamped):
        self.goal_pose = msg
        self.arrived = False
        # self.get_logger().info(f"목표 좌표 수신: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")

    def control_loop(self):
        # print(self.arrived)
        # print(self.goal_pose)
        if self.goal_pose is None or self.arrived:
            # print("1")
            self.publish_stop()
            return

        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y
        goal_theta = self.get_yaw_from_quaternion(self.goal_pose.pose.orientation)
        # print(f"current_x:{self.current_x}, current_y:{self.current_y}, current_theta:{self.current_theta}")
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)
        # print(f"distance:{distance}, gol_theta:{goal_theta:.2f}, current_theta:{self.current_theta:.2f}")
        target_angle = math.atan2(-dy, dx)
        angle_diff = self.normalize_angle(target_angle - self.current_theta)

        cmd = Twist()

        if distance > self.position_tolerance:
            # print(distance)
            if distance > self.position_tolerance:
                cmd.angular.z = 0.0
                cmd.linear.x = self.linear_speed
            # if abs(angle_diff) > self.angle_tolerance:
            else:
                cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
                cmd.linear.x = 0.0
            # else:
            #     cmd.angular.z = 0.0
            #     cmd.linear.x = self.linear_speed
        else:
            final_angle_diff = self.normalize_angle(goal_theta - self.current_theta)
            print(f"final_angle_diff:{abs(final_angle_diff):.2f}, ")
            if abs(final_angle_diff) > self.angle_tolerance:
                cmd.angular.z = self.angular_speed if final_angle_diff > 0 else -self.angular_speed
                cmd.linear.x = 0.0
            else:
                self.arrived = True
                self.get_logger().info("목표 위치에 도착했습니다.")
                cmd.angular.z = 0.0
                cmd.linear.x = 0.0
                self.publish_stop()
                return

        self.cmd_vel_pub.publish(cmd)

    def publish_stop(self):
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

    @staticmethod
    def get_yaw_from_quaternion(q):
        import math
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        import math
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = SimpleRobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
