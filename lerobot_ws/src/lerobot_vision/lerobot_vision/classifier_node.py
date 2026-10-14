"""深度增强节点 — 为检测结果补全深度值

订阅:
  - /vision/detections (VisionDetection) — YOLO 检测结果
  - /camera/depth/image_rect_raw (Image)  — 深度图
  - /camera/color/image_raw (Image)       — 彩色图 (缓存, 已不使用颜色分类)

发布:
  - /vision/classified (VisionDetection)  — 带深度值的检测结果
"""

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    from lerobot_interfaces.msg import VisionDetection
    HAVE_CUSTOM_MSGS = True
except ImportError:
    VisionDetection = None
    HAVE_CUSTOM_MSGS = False


class DepthEnhancerNode(Node):
    """深度增强节点 — 只为检测结果附加深度值, 不做颜色分类"""

    def __init__(self):
        super().__init__('classifier_node')
        self.bridge = CvBridge()

        self._latest_depth = None

        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self._depth_callback, 10
        )

        if HAVE_CUSTOM_MSGS:
            self.detection_sub = self.create_subscription(
                VisionDetection, '/vision/detections',
                self._detection_callback, 10
            )
        else:
            from std_msgs.msg import String
            self.detection_sub = self.create_subscription(
                String, '/vision/detections',
                self._detection_callback_string, 10
            )

        if HAVE_CUSTOM_MSGS:
            self.classified_pub = self.create_publisher(
                VisionDetection, '/vision/classified', 10
            )
        else:
            from std_msgs.msg import String
            self.classified_pub = self.create_publisher(
                String, '/vision/classified', 10
            )

        self.get_logger().info('深度增强节点已启动 (无颜色分类)')

    def _depth_callback(self, msg):
        self._latest_depth = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough'
        )

    def _robust_depth(self, px, py):
        """桌面深度: 检测在图上1/2则取下方桌面区, 否则在检测点周围采样"""
        h, w = self._latest_depth.shape
        if py < h // 2:
            bot = self._latest_depth[h*2//3:, w//4:3*w//4]
            valid = bot[bot > 0]
            d = float(np.median(valid)) * 0.001 if valid.size > 30 else 0.0
        else:
            r = 30
            y1, y2 = max(0, py-r), min(h, py+r)
            x1, x2 = max(0, px-r), min(w, px+r)
            region = self._latest_depth[y1:y2, x1:x2]
            valid = region[region > 0]
            d = float(np.median(valid)) * 0.001 if valid.size > 30 else 0.0
        return d

    def _detection_callback(self, msg):
        if self._latest_depth is None:
            return

        depth = self._robust_depth(int(msg.x_pixel), int(msg.y_pixel))

        vd = VisionDetection()
        vd.object_class = msg.object_class
        vd.color = ''            # 无颜色分类
        vd.x_pixel = msg.x_pixel
        vd.y_pixel = msg.y_pixel
        vd.depth = depth
        vd.confidence = msg.confidence
        self.classified_pub.publish(vd)

    def _detection_callback_string(self, msg):
        self.get_logger().info(f'收到检测 (string fallback): {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = DepthEnhancerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
