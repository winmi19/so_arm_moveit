"""YOLOv8 目标检测封装"""

import os


class YOLODetector:
    """YOLOv8 检测器

    Usage:
        detector = YOLODetector('models/best.pt', conf=0.5)
        detections = detector.detect(image)
        # detections: [{class, bbox, confidence}, ...]
    """

    def __init__(self, model_path, confidence_threshold=0.5,
                 nms_threshold=0.45, device='cpu'):
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.device = device
        self.model = None

    def load(self):
        """加载 YOLO 模型 (支持 .pt / .onnx)"""
        ext = os.path.splitext(self.model_path)[1]
        if ext in ('.pt', '.pth'):
            self._load_ultralytics()
        elif ext == '.onnx':
            self._load_onnx()
        else:
            raise ValueError(f"不支持的模型格式: {ext}")

    def _load_ultralytics(self):
        try:
            from ultralytics import YOLO
        except ImportError:
            raise ImportError("请安装: pip install ultralytics")
        self.model = YOLO(self.model_path)
        self.model.to(self.device)

    def _load_onnx(self):
        try:
            import onnxruntime as ort
        except ImportError:
            raise ImportError("请安装: pip install onnxruntime")
        self._ort_session = ort.InferenceSession(
            self.model_path,
            providers=['CUDAExecutionProvider' if 'cuda' in self.device
                       else 'CPUExecutionProvider']
        )

    def detect(self, image):
        """对单帧图像做目标检测

        Returns:
            list of dict:
                - class_name: 物体类别名
                - class_id:   类别ID
                - confidence: 置信度 [0,1]
                - bbox:       (x1, y1, x2, y2) 像素坐标
                - center:     (cx, cy) 检测框中心像素
        """
        if self.model is not None:
            return self._detect_ultralytics(image)
        elif hasattr(self, '_ort_session') and self._ort_session is not None:
            return self._detect_onnx(image)
        else:
            return []

    def _detect_ultralytics(self, image):
        results = self.model(image, conf=self.confidence_threshold,
                             iou=self.nms_threshold, verbose=False)
        detections = []
        for r in results:
            boxes = r.boxes
            if boxes is None:
                continue
            for i in range(len(boxes)):
                cls_id = int(boxes.cls[i].item())
                cls_name = r.names.get(cls_id, str(cls_id))
                conf = boxes.conf[i].item()
                xyxy = boxes.xyxy[i].cpu().numpy()
                detections.append({
                    'class_name': cls_name,
                    'class_id': cls_id,
                    'confidence': conf,
                    'bbox': tuple(xyxy.astype(int)),
                    'center': (int((xyxy[0] + xyxy[2]) / 2),
                               int((xyxy[1] + xyxy[3]) / 2)),
                })
        return detections

    def _detect_onnx(self, image):
        import cv2
        import numpy as np
        # ONNX 预处理 (640x640)
        input_size = 640
        h, w = image.shape[:2]
        scale = min(input_size / w, input_size / h)
        nw, nh = int(w * scale), int(h * scale)

        resized = cv2.resize(image, (nw, nh))
        canvas = np.full((input_size, input_size, 3), 114, dtype=np.uint8)
        canvas[(input_size - nh) // 2:(input_size - nh) // 2 + nh,
               (input_size - nw) // 2:(input_size - nw) // 2 + nw] = resized

        blob = np.transpose(canvas / 255.0, (2, 0, 1)).astype(np.float32)
        blob = np.expand_dims(blob, 0)

        outputs = self._ort_session.run(None, {'images': blob})
        # TODO: 解析ONNX输出 → detections
        return []
