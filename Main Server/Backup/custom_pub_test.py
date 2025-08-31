import rclpy
from rclpy.node import Node
from my_msgs.msg import OrderMsg  # 커스텀 메시지 import

class OrderPublisher(Node):
    def __init__(self):
        super().__init__('order_publisher')
        self.publisher_ = self.create_publisher(OrderMsg, 'order_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = OrderMsg()
        msg.order_id = 1
        msg.item = ['apple', 'banana', 'carrot']
        msg.count = [3, 2, 5]

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: id={msg.order_id}, item={msg.item}, count={msg.count}')

def main(args=None):
    rclpy.init(args=args)
    node = OrderPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
