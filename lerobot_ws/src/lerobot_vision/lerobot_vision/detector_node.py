"""YOLOv8 目标检测 ROS 节点

订阅: /camera/color/image_raw (sensor_msgs/Image)
发布: /vision/detections (VisionDetection 或 String)
"""

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    from lerobot_interfaces.msg import VisionDetection
    HAVE_CUSTOM_MSGS = True
except ImportError:
    VisionDetection = None
    HAVE_CUSTOM_MSGS = False

from lerobot_vision.detector import YOLODetector
from lerobot_vision.camera import RealSenseCamera, USBCamera


class DetectorNode(Node):
    """YOLO 检测节点"""

    def __init__(self):
        super().__init__('detector_node')

        # 参数
        self.declare_parameter('model_path', 'models/best.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('camera_type', 'realsense')

        model_path = self.get_parameter('model_path').value
        conf = self.get_parameter('confidence_threshold').value
        nms = self.get_parameter('nms_threshold').value
        device = self.get_parameter('device').value

        # 检测器
        self.detector = YOLODetector(model_path, conf, nms, device)

        # 尝试加载模型
        if os.path.exists(model_path):
            self.detector.load()
            self.get_logger().info(f'YOLO 模型已加载: {model_path}')
        else:
            self.get_logger().warn(
                f'模型文件不存在: {model_path}，请放置训练好的 best.pt'
            )

        # CV Bridge
        self.bridge = CvBridge()

        # 图像订阅
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10
        )

        # 检测结果发布
        if HAVE_CUSTOM_MSGS:
            self.detection_pub = self.create_publisher(
                VisionDetection, '/vision/detections', 10
            )
        else:
            from std_msgs.msg import String
            self.detection_pub = self.create_publisher(
                String, '/vision/detections', 10
            )

        self.get_logger().info('检测节点已启动, 等待图像...')

    def image_callback(self, msg):
        """每收到一帧图像, 做一次检测"""
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return

        detections = self.detector.detect(image)
        if not detections:
            return

        for det in detections:
            self._publish_detection(det)

    def _publish_detection(self, det):
        if HAVE_CUSTOM_MSGS:
            vd = VisionDetection()
            vd.object_class = det['class_name']
            vd.color = ''  # 颜色由 classifier_node 填入
            vd.x_pixel = float(det['center'][0])
            vd.y_pixel = float(det['center'][1])
            vd.depth = 0.0      # 深度由 classifier_node 从深度图填入
            vd.confidence = det['confidence']
            self.detection_pub.publish(vd)
        else:
            from std_msgs.msg import String
            msg = String()
            msg.data = (
                f"{det['class_name']}:"
                f"{det['center'][0]}:{det['center'][1]}:"
                f"{det['confidence']:.3f}"
            )
            self.detection_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
