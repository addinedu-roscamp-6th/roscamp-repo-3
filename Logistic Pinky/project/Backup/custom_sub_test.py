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

    def listener_callback(self, msg):
        self.get_logger().info(
            f'Received Order ID: {msg.order_id}\n'
            f'Items: {msg.item}\n'
            f'Counts: {msg.count}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = OrderSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
