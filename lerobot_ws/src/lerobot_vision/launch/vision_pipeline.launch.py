"""视觉管线启动文件

Usage:
    ros2 launch lerobot_vision vision_pipeline.launch.py \
        model_path:=/path/to/best.pt \
        camera_namespace:=/camera/camera

默认 camera_namespace 匹配 realsense2_camera 的默认配置
    (camera_name:=camera, camera_namespace:=camera)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('lerobot_vision')
    default_model = os.path.join(pkg, 'models', 'best.pt')

    # ── 声明参数 ──────────────────────────────
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value=default_model,
        description='YOLO 模型路径 (.pt 或 .onnx)'
    )
    confidence_arg = DeclareLaunchArgument(
        'confidence', default_value='0.5',
        description='YOLO 置信度阈值'
    )
    device_arg = DeclareLaunchArgument(
        'device', default_value='cpu',
        description='推理设备: cpu / cuda'
    )
    camera_ns_arg = DeclareLaunchArgument(
        'camera_namespace', default_value='/camera/camera',
        description='RealSense 相机命名空间'
    )

    model_path = LaunchConfiguration('model_path')
    confidence = LaunchConfiguration('confidence')
    device = LaunchConfiguration('device')
    camera_ns = LaunchConfiguration('camera_namespace')

    # ── Detector Node ─────────────────────────
    detector_node = Node(
        package='lerobot_vision',
        executable='detector_node.py',
        name='detector_node',
        output='screen',
        parameters=[{
            'model_path': model_path,
            'confidence_threshold': confidence,
            'device': device,
        }],
        remappings=[
            ('/camera/color/image_raw',
             [camera_ns, '/color/image_raw']),
        ],
    )

    # ── Classifier Node ───────────────────────
    classifier_node = Node(
        package='lerobot_vision',
        executable='classifier_node.py',
        name='classifier_node',
        output='screen',
        remappings=[
            ('/camera/color/image_raw',
             [camera_ns, '/color/image_raw']),
            ('/camera/depth/image_rect_raw',
             [camera_ns, '/aligned_depth_to_color/image_raw']),
        ],
    )

    return LaunchDescription([
        model_path_arg,
        confidence_arg,
        device_arg,
        camera_ns_arg,
        detector_node,
        classifier_node,
    ])
