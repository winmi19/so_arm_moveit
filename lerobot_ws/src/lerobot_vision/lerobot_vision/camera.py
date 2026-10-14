"""RGB-D 相机驱动封装 (RealSense / USB)

RealSense D435/D415: 使用 pyrealsense2
USB摄像头: 使用 OpenCV VideoCapture (无深度)
"""

import numpy as np


class RealSenseCamera:
    """Intel RealSense RGB-D 相机"""

    def __init__(self, width=640, height=480, fps=30):
        self.width = width
        self.height = height
        self.fps = fps
        self.pipeline = None
        self.align = None
        self.profile = None

    def start(self):
        try:
            import pyrealsense2 as rs
        except ImportError:
            raise ImportError("请安装: pip install pyrealsense2")

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, self.width, self.height,
                             rs.format.bgr8, self.fps)
        config.enable_stream(rs.stream.depth, self.width, self.height,
                             rs.format.z16, self.fps)

        self.align = rs.align(rs.stream.color)
        self.profile = self.pipeline.start(config)

        # 获取深度传感器尺度
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        # 跳过前几帧以稳定曝光
        for _ in range(30):
            self.pipeline.wait_for_frames()

    def get_frames(self):
        """获取对齐后的 color + depth frame

        Returns:
            color_image:  (H, W, 3) numpy array, BGR格式
            depth_image:  (H, W) numpy array, 单位: 米
            timestamp_ms: 时间戳(毫秒)
        """
        try:
            import pyrealsense2 as rs
        except ImportError:
            raise ImportError("请安装: pip install pyrealsense2")

        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = (np.asanyarray(depth_frame.get_data()) *
                       self.depth_scale)

        return color_image, depth_image, frames.get_timestamp()

    def get_depth_at_pixel(self, depth_image, x, y):
        """获取像素(x,y)处的深度值 (米)

        Args:
            depth_image: 深度图
            x, y: 像素坐标 (整数)

        Returns:
            depth: 深度值 (米)
        """
        h, w = depth_image.shape
        x = np.clip(x, 0, w - 1)
        y = np.clip(y, 0, h - 1)

        # 取 5x5 邻域中位数，去除噪声
        r = 2
        window = depth_image[max(0, y - r):min(h, y + r + 1),
                             max(0, x - r):min(w, x + r + 1)]
        return float(np.median(window[window > 0])) if window.any() else 0.0

    def stop(self):
        if self.pipeline:
            self.pipeline.stop()


class USBCamera:
    """USB 普通摄像头 (无深度)"""

    def __init__(self, device_id=0, width=640, height=480, fps=30):
        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps
        self.cap = None

    def start(self):
        import cv2
        self.cap = cv2.VideoCapture(self.device_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

    def get_frames(self):
        """获取 color frame (无深度)

        Returns:
            color_image:  (H, W, 3) numpy array, BGR
            depth_image:  None
            timestamp_ms: 时间戳(毫秒)
        """
        import time
        ret, frame = self.cap.read()
        if not ret:
            return None, None, 0
        return frame, None, int(time.time() * 1000)

    def get_depth_at_pixel(self, depth_image, x, y):
        return 0.0  # USB相机无深度

    def stop(self):
        if self.cap:
            self.cap.release()
