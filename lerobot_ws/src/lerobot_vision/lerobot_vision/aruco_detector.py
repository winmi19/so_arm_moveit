"""ChArUco 标定板检测 + TF 发布节点

眼在手上标定：检测桌面上的 ChArUco 板，发布 camera → board 的 TF。
easy_handeye2 从 TF 树读取此变换，结合机械臂位姿完成 AX=XB 手眼标定。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
import tf2_ros
from scipy.spatial.transform import Rotation
import cv2
from cv_bridge import CvBridge
import numpy as np


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('squares_x', 9)
        self.declare_parameter('squares_y', 9)
        self.declare_parameter('square_length', 0.0225)
        self.declare_parameter('marker_length', 0.020)
        self.declare_parameter('marker_dict', 'DICT_6X6_250')
        self.declare_parameter('parent_frame', 'camera_color_optical_frame')
        self.declare_parameter('marker_frame', 'charuco_board')

        squares_x = self.get_parameter('squares_x').value
        squares_y = self.get_parameter('squares_y').value
        square_length = self.get_parameter('square_length').value
        marker_length = self.get_parameter('marker_length').value
        dict_name = self.get_parameter('marker_dict').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.marker_frame = self.get_parameter('marker_frame').value

        dict_id = getattr(cv2.aruco, dict_name, cv2.aruco.DICT_6X6_250)
        dictionary = cv2.aruco.getPredefinedDictionary(dict_id)

        self.board = cv2.aruco.CharucoBoard(
            (squares_x, squares_y), square_length, marker_length, dictionary)

        params = cv2.aruco.DetectorParameters()
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 30
        params.minMarkerPerimeterRate = 0.01
        params.maxErroneousBitsInBorderRate = 0.5
        params.errorCorrectionRate = 1.0
        params.polygonalApproxAccuracyRate = 0.05

        self.detector = cv2.aruco.CharucoDetector(self.board)
        self.detector.setDetectorParameters(params)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.detect_count = 0
        self.skip_count = 0
        self.last_tf = None

        self.img_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_cb, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.info_cb, 10)
        self.tf_br = tf2_ros.TransformBroadcaster(self)

        corners_count = (squares_x - 1) * (squares_y - 1)
        self.get_logger().info(
            f'ChArUco board {squares_x}x{squares_y} ready: '
            f'square={square_length}m, marker={marker_length}m, '
            f'dict={dict_name}, {corners_count} chessboard corners')

    def info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info(
                f'Camera matrix loaded: fx={self.camera_matrix[0,0]:.1f}')

    def image_cb(self, msg):
        self.skip_count += 1
        if self.skip_count % 5 != 0:
            return

        if self.camera_matrix is None:
            self.get_logger().warn('No camera matrix yet', throttle_duration_sec=5.0)
            return

        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        charuco_corners, charuco_ids, marker_corners, marker_ids = \
            self.detector.detectBoard(img)

        if charuco_ids is None or len(charuco_ids) < 6:
            if marker_ids is not None and len(marker_ids) > 0:
                self.get_logger().info(
                    f'{len(marker_ids)} markers but only {len(charuco_ids) if charuco_ids is not None else 0} '
                    f'ChArUco corners interpolated', throttle_duration_sec=5.0)
            # republish last known TF to keep timeline continuous for easy_handeye2
            if self.last_tf is not None:
                self.last_tf.header.stamp = msg.header.stamp
                self.tf_br.sendTransform(self.last_tf)
            return

        all_obj = self.board.getChessboardCorners()
        ids_flat = charuco_ids.ravel()
        obj_pts = np.array([all_obj[int(idx)] for idx in ids_flat], dtype=np.float32)

        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, charuco_corners, self.camera_matrix, self.dist_coeffs)

        if not ok:
            self.get_logger().info('PnP failed', throttle_duration_sec=5.0)
            return

        self.detect_count += 1
        tvec = tvec.flatten()
        R, _ = cv2.Rodrigues(rvec.flatten())
        quat = Rotation.from_matrix(R).as_quat()

        T = TransformStamped()
        T.header.stamp = msg.header.stamp
        T.header.frame_id = self.parent_frame
        T.child_frame_id = self.marker_frame
        T.transform.translation.x = float(tvec[0])
        T.transform.translation.y = float(tvec[1])
        T.transform.translation.z = float(tvec[2])
        T.transform.rotation.x = float(quat[0])
        T.transform.rotation.y = float(quat[1])
        T.transform.rotation.z = float(quat[2])
        T.transform.rotation.w = float(quat[3])

        self.last_tf = T
        self.tf_br.sendTransform(T)
        if self.detect_count <= 3 or self.detect_count % 50 == 0:
            self.get_logger().info(
                f'#{self.detect_count} TF: {self.parent_frame}→{self.marker_frame} '
                f't=[{tvec[0]:.3f},{tvec[1]:.3f},{tvec[2]:.3f}] '
                f'corners={len(charuco_ids)}')


def main():
    rclpy.init()
    rclpy.spin(ArucoDetectorNode())


if __name__ == '__main__':
    main()
