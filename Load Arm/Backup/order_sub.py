import rclpy
from rclpy.node import Node
from my_msgs.msg import OrderMsg

class OrderSubscriber(Node):
    def __init__(self):
        super().__init__('order_subscriber')
        self.subscription = self.create_subscription(
            OrderMsg,
            'order_topic',
            self.listener_callback,
            10
        )
        self.get_logger().info("📦 'order_topic' 구독 시작됨")

    def listener_callback(self, msg: OrderMsg):
        self.get_logger().info(f"📥 주문 수신 - ID: {msg.order_id}")
        self.get_logger().info(f"   📦 Item: {msg.item}")
        self.get_logger().info(f"   🔢 Count: {msg.count}")

def main(args=None):
    rclpy.init(args=args)
    node = OrderSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 종료")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

