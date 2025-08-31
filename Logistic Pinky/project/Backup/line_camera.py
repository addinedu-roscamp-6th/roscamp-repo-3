import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from picamera2 import Picamera2
import cv2
from cv_bridge import CvBridge
import time

class PicameraPublisher(Node):
    def __init__(self):
        super().__init__('picamera_publisher')

        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.br = CvBridge()

        # Picamera 설정
        self.picam2 = Picamera2()
        self.picam2.configure(
            self.picam2.create_preview_configuration(
                main={"format": "RGB888", "size": (640, 480)}
            )
        )
        self.picam2.start()
        time.sleep(1.0)  # 카메라 워밍업 시간

        # 주기적으로 호출될 타이머
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def timer_callback(self):
        frame = self.picam2.capture_array()

        # ROS 메시지로 퍼블리시
        msg = self.br.cv2_to_imgmsg(frame, encoding="rgb8")
        self.publisher_.publish(msg)

        # OpenCV 창에 이미지 출력
        self.show_image(frame)

    def show_image(self, frame):
        cv2.imshow("Picamera2 Image", frame)
        # waitKey 없으면 창이 먹통됨. 1ms 대기
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("종료 명령(Q) 감지됨. 노드 종료합니다.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PicameraPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("사용자 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
