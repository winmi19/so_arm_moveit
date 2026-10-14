"""相机像素坐标 → 机械臂基座坐标转换

数据流:
    pixel + depth → 相机3D (内参投影)
         → 夹爪3D (手眼矩阵 T_gripper2cam)
              → 基座3D (TF: base→gripper)
"""

import numpy as np
from scipy.spatial.transform import Rotation


class CoordinateTransform:
    """坐标转换：相机像素 → 机械臂基座3D坐标"""

    def __init__(self):
        # 相机内参 (从 camera_info 写入)
        self.camera_matrix = None
        self.dist_coeffs = None

        # 手眼矩阵 (从标定文件加载)
        self.T_gripper2cam = None  # 4x4, gripper→camera_optical

    def set_camera_intrinsics(self, k, d):
        self.camera_matrix = np.array(k).reshape(3, 3)
        self.dist_coeffs = np.array(d)

    def load_handeye_calib(self, translation, quat_xyzw):
        """加载手眼标定结果

        Args:
            translation: (x, y, z) gripper 坐标系下相机位置
            quat_xyzw:  (x, y, z, w) 四元数
        """
        R = Rotation.from_quat(quat_xyzw).as_matrix()
        self.T_gripper2cam = np.eye(4)
        self.T_gripper2cam[:3, :3] = R
        self.T_gripper2cam[:3, 3] = translation

    def pixel_to_camera_3d(self, x_pixel, y_pixel, depth):
        """像素 + 深度 → 相机光学坐标系 3D"""
        if self.camera_matrix is None:
            raise ValueError("相机内参未设置")

        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        x_norm = (x_pixel - cx) / fx
        y_norm = (y_pixel - cy) / fy

        return np.array([x_norm * depth, y_norm * depth, depth])

    def pixel_to_gripper_3d(self, x_pixel, y_pixel, depth):
        """像素 + 深度 → 夹爪坐标系 3D"""
        p_cam = self.pixel_to_camera_3d(x_pixel, y_pixel, depth)
        if self.T_gripper2cam is None:
            raise ValueError("手眼矩阵未加载")
        p_h = np.append(p_cam, 1.0)
        p_gripper = self.T_gripper2cam @ p_h
        return p_gripper[:3]

    def pixel_to_base_3d(self, x_pixel, y_pixel, depth, T_base2gripper):
        """像素 + 深度 + 机械臂当前位姿 → 基座坐标系 3D

        Args:
            T_base2gripper: 4x4 齐次矩阵, base→gripper (来自 TF)
        """
        p_gripper = self.pixel_to_gripper_3d(x_pixel, y_pixel, depth)
        p_h = np.append(p_gripper, 1.0)
        p_base = T_base2gripper @ p_h
        return p_base[0], p_base[1], p_base[2]
