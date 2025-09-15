# yolov5_detector/yolov5_node.py
import rclpy, torch, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray
from cv_bridge import CvBridge
from vision_msgs.msg import ObjectHypothesisWithPose
from geometry_msgs.msg import Point32, Vector3

class YoloV5Node(Node):
    def __init__(self):
        super().__init__('yolov5_detector')
        w = self.declare_parameter('weights','best.pt').value
        topic = self.declare_parameter('image_topic','/image_raw').value
        self.model   = torch.hub.load('ultralytics/yolov5','custom',path=w)
        self.bridge  = CvBridge()
        self.sub = self.create_subscription(Image, topic, self.cb, 10)
        self.pub = self.create_publisher(Detection2DArray,'/yolov5/bboxes',10)

    def cb(self,msg):
        img = self.bridge.imgmsg_to_cv2(msg,'bgr8')
        res = self.model(img,size=640)
        det_msg = Detection2DArray()
        det_msg.header = msg.header
        for *xyxy, conf, cls in res.xyxy[0]:
            if conf < 0.5:
                continue

            det  = Detection2D()
            bbox = det.bbox

            # ❶ BoundingBox2D.center   (Point32)
            cx = float((xyxy[0] + xyxy[2]) / 2)
            cy = float((xyxy[1] + xyxy[3]) / 2)
            bbox.center = Point32(x=cx, y=cy, z=0.0)

            # ❷ BoundingBox2D.size     (Vector3)
            bbox.size = Vector3(
                x=float(xyxy[2] - xyxy[0]),
                y=float(xyxy[3] - xyxy[1]),
                z=0.0
            )

            # ❸ 결과(클래스/신뢰도)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = int(cls)
            hyp.hypothesis.score    = float(conf)
            det.results.append(hyp)

            det_msg.detections.append(det)
        self.pub.publish(det_msg)
        self.get_logger().info(f"boxes={len(det_msg.detections)}")  # 디버그

def main():
    rclpy.init(); rclpy.spin(YoloV5Node())
if __name__ == '__main__':
    main()
