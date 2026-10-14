from setuptools import setup, find_packages
import os

package_name = 'lerobot_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/vision_pipeline.launch.py',
             'launch/handeye_calibrate.launch.py',
             'launch/handeye_publish.launch.py']),
        ('share/' + package_name + '/config',
            ['config/vision_config.yaml',
             'config/aruco_config.yaml']),
        ('share/' + package_name + '/models',
            ['models/yolov8n.pt']),
        # 安装 wrapper 脚本到 lib/ 供 ros2 launch 使用
        (os.path.join('lib', package_name),
            ['scripts/detector_node.py', 'scripts/classifier_node.py',
             'scripts/aruco_detector.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Vision perception layer: YOLOv8 + color classification',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'detector_node.py   = lerobot_vision.detector_node:main',
            'classifier_node.py = lerobot_vision.classifier_node:main',
        ],
    },
)
