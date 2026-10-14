# SO101 手眼标定 (Eye-in-Hand) 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 基于 easy_handeye2 搭建 SO101 眼在手上标定流程，启动后可通过 GUI 采集样本、解算手眼矩阵、保存并发布 TF。

**Architecture:** 对标定来说，只需在 `lerobot_vision` 包内新增 3 个文件：ArUco 配置 YAML、标定 launch（ArUco 检测 + easy_handeye2 服务 + GUI）、发布 launch（标定结果 TF 发布）。不改动已有代码。

**Tech Stack:** ROS 2 Humble, easy_handeye2, aruco_ros, RealSense D405, SO101 URDF TF 树

---

### Task 1: 安装依赖 + 克隆 easy_handeye2

**此任务需要 sudo 权限，需用户手动执行。**

- [ ] **Step 1: 安装 ArUco 依赖**

```bash
sudo apt install ros-humble-aruco-ros ros-humble-aruco-opencv -y
```

- [ ] **Step 2: 克隆 easy_handeye2 到主工作空间**

```bash
cd /disk2/so_arm_try/lerobot_ws/src
git clone https://github.com/marcoesposito1988/easy_handeye2.git
```

- [ ] **Step 3: 编译整个工作空间**

```bash
cd /disk2/so_arm_try/lerobot_ws
colcon build
source install/setup.bash
```

验证: `ros2 pkg list | grep -E "aruco_ros|easy_handeye2"` 应输出两个包名。

---

### Task 2: 创建 ArUco 标定板配置文件

**文件:** 创建 `src/lerobot_vision/config/aruco_config.yaml`

- [ ] **Step 1: 写配置文件**

```yaml
# ArUco 标定板参数 — 眼在手上标定
# 标定板固定在桌面上，相机装在夹爪上

/**:
  ros__parameters:
    # 标记尺寸 (米) — 打印后实际测量
    marker_size: 0.050

    # ArUco 标记 ID (0~99 用于 DICT_4X4_50)
    marker_id: 0

    # 标定板输出 TF 帧名
    marker_frame: "marker_frame"

    # 相机光学帧 (标记位姿相对于此帧发布)
    reference_frame: "camera_color_frame"

    # 角点细化方法: NONE / HARRIS / LINES / SUBPIX
    corner_refinement: "LINES"
```

---

### Task 3: 创建标定 launch 文件

**文件:** 创建 `src/lerobot_vision/launch/handeye_calibrate.launch.py`

本 launch 组合三件事：ArUco 单标记检测 + easy_handeye2 `handeye_server` + `rqt_calibrator` GUI。

设计中不通过 `IncludeLaunchDescription` 嵌套 easy_handeye2 的 calibrate.launch.py，而是直接实例化它的两个节点，这样可以完全控制 topic 重映射。参数默认值匹配 SO101 的 TF 树和 RealSense D405 的 topic 命名空间。

- [ ] **Step 1: 写 launch 文件**

```python
"""手眼标定启动文件 — 眼在手上 (eye-in-hand)

ArUco 标记检测 (aruco_ros::single) + easy_handeye2 标定服务 + GUI

Usage:
    ros2 launch lerobot_vision handeye_calibrate.launch.py \
        marker_size:=0.050 \
        marker_id:=0

默认 camera_namespace 匹配 realsense2_camera 默认命名空间
    (camera_name:=camera, camera_namespace:=camera)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── ArUco 参数 ─────────────────────────────
    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.050',
        description='ArUco 标记实际边长 (米)'
    )
    marker_id_arg = DeclareLaunchArgument(
        'marker_id', default_value='0',
        description='ArUco 标记 ID'
    )
    marker_frame_arg = DeclareLaunchArgument(
        'marker_frame', default_value='marker_frame',
        description='标定板 TF 帧名'
    )
    corner_refinement_arg = DeclareLaunchArgument(
        'corner_refinement', default_value='LINES',
        description='角点细化: NONE / HARRIS / LINES / SUBPIX'
    )

    # ── 相机 topic 参数 ─────────────────────────
    camera_ns_arg = DeclareLaunchArgument(
        'camera_namespace', default_value='/camera/camera',
        description='RealSense 相机命名空间'
    )

    # ── easy_handeye2 参数 ──────────────────────
    name_arg = DeclareLaunchArgument(
        'name', default_value='so101_eih',
        description='标定名称 (唯一标识)'
    )
    calibration_type_arg = DeclareLaunchArgument(
        'calibration_type', default_value='eye_in_hand',
        choices=['eye_in_hand', 'eye_on_base'],
        description='标定类型'
    )
    robot_base_frame_arg = DeclareLaunchArgument(
        'robot_base_frame', default_value='base_link',
        description='机械臂底座 TF 帧'
    )
    robot_effector_frame_arg = DeclareLaunchArgument(
        'robot_effector_frame', default_value='gripper',
        description='相机所固定的机械臂连杆 TF 帧'
    )
    tracking_base_frame_arg = DeclareLaunchArgument(
        'tracking_base_frame', default_value='camera_color_frame',
        description='相机光学帧 (ArUco 检测的参考帧)'
    )
    tracking_marker_frame_arg = DeclareLaunchArgument(
        'tracking_marker_frame', default_value='marker_frame',
        description='标定板 TF 帧 (ArUco 检测输出)'
    )

    camera_ns = LaunchConfiguration('camera_namespace')

    # ── ArUco 单标记检测节点 ─────────────────────
    aruco_single_node = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        output='screen',
        parameters=[{
            'image_is_rectified': False,
            'marker_size': LaunchConfiguration('marker_size'),
            'marker_id': LaunchConfiguration('marker_id'),
            'reference_frame': LaunchConfiguration('tracking_base_frame'),
            'camera_frame': LaunchConfiguration('tracking_base_frame'),
            'marker_frame': LaunchConfiguration('tracking_marker_frame'),
            'corner_refinement': LaunchConfiguration('corner_refinement'),
        }],
        remappings=[
            ('/camera_info', [camera_ns, '/color/camera_info']),
            ('/image', [camera_ns, '/color/image_raw']),
        ],
    )

    # ── eye_in_hand 初始猜测 (dummy TF, 粗略值即可) ──
    dummy_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='handeye_dummy_publisher',
        condition=LaunchConfigurationEquals('calibration_type', 'eye_in_hand'),
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.1',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', LaunchConfiguration('robot_effector_frame'),
            '--child-frame-id', LaunchConfiguration('tracking_base_frame'),
        ],
    )

    # ── easy_handeye2 标定服务 ───────────────────
    handeye_server = Node(
        package='easy_handeye2',
        executable='handeye_server',
        name='handeye_server',
        parameters=[{
            'name': LaunchConfiguration('name'),
            'calibration_type': LaunchConfiguration('calibration_type'),
            'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
            'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
        }],
    )

    # ── easy_handeye2 标定 GUI (rqt 插件) ────────
    handeye_gui = Node(
        package='easy_handeye2',
        executable='rqt_calibrator.py',
        name='handeye_rqt_calibrator',
        parameters=[{
            'name': LaunchConfiguration('name'),
            'calibration_type': LaunchConfiguration('calibration_type'),
            'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
            'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
        }],
    )

    return LaunchDescription([
        marker_size_arg,
        marker_id_arg,
        marker_frame_arg,
        corner_refinement_arg,
        camera_ns_arg,
        name_arg,
        calibration_type_arg,
        robot_base_frame_arg,
        robot_effector_frame_arg,
        tracking_base_frame_arg,
        tracking_marker_frame_arg,
        aruco_single_node,
        dummy_tf,
        handeye_server,
        handeye_gui,
    ])
```

---

### Task 4: 创建标定结果发布 launch 文件

**文件:** 创建 `src/lerobot_vision/launch/handeye_publish.launch.py`

标定完成后，此 launch 读取 `~/.ros/easy_handeye2/<name>.yaml` 并发布静态 TF。可嵌入真机全链路启动。

- [ ] **Step 1: 写 launch 文件**

```python
"""发布手眼标定结果 TF

标定完成后使用, 从 ~/.ros/easy_handeye2/ 读取标定结果并发布为静态 TF。

Usage:
    ros2 launch lerobot_vision handeye_publish.launch.py name:=so101_eih
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    name_arg = DeclareLaunchArgument(
        'name', default_value='so101_eih',
        description='标定名称 (需与 calibrate 时一致)'
    )

    handeye_publisher = Node(
        package='easy_handeye2',
        executable='handeye_publisher',
        name='handeye_publisher',
        parameters=[{
            'name': LaunchConfiguration('name'),
        }],
    )

    return LaunchDescription([
        name_arg,
        handeye_publisher,
    ])
```

---

### Task 5: 更新 setup.py 注册新文件

**文件:** 修改 `src/lerobot_vision/setup.py`

- [ ] **Step 1: 在 data_files 中注册新的 launch 文件和 config 文件**

找到 `setup.py` 中 `data_files` 列表，在 `('share/' + package_name + '/launch', ...)` 行添加新 launch 文件，在 `('share/' + package_name + '/config', ...)` 行添加新 config 文件。

修改前：
```python
        ('share/' + package_name + '/launch',
            ['launch/vision_pipeline.launch.py']),
        ('share/' + package_name + '/config',
            ['config/vision_config.yaml']),
```

修改后：
```python
        ('share/' + package_name + '/launch',
            ['launch/vision_pipeline.launch.py',
             'launch/handeye_calibrate.launch.py',
             'launch/handeye_publish.launch.py']),
        ('share/' + package_name + '/config',
            ['config/vision_config.yaml',
             'config/aruco_config.yaml']),
```

---

### Task 6: 编译并验证

- [ ] **Step 1: 编译**

```bash
cd /disk2/so_arm_try/lerobot_ws
colcon build --packages-select lerobot_vision
source install/setup.bash
```

- [ ] **Step 2: 验证 launch 文件可解析**

```bash
ros2 launch lerobot_vision handeye_calibrate.launch.py --show-args
```

预期输出: 列出 `marker_size`, `marker_id`, `marker_frame`, `corner_refinement`, `camera_namespace`, `name`, `calibration_type`, `robot_base_frame`, `robot_effector_frame`, `tracking_base_frame`, `tracking_marker_frame` 共 11 个参数。

```bash
ros2 launch lerobot_vision handeye_publish.launch.py --show-args
```

预期输出: 列出 `name` 参数。

- [ ] **Step 3: 验证配置文件可访问**

```bash
ls $(ros2 pkg prefix lerobot_vision)/share/lerobot_vision/config/aruco_config.yaml
```

预期输出: 文件路径。

- [ ] **Step 4: 提交**

```bash
cd /disk2/so_arm_try/lerobot_ws
git add src/lerobot_vision/launch/handeye_calibrate.launch.py \
        src/lerobot_vision/launch/handeye_publish.launch.py \
        src/lerobot_vision/config/aruco_config.yaml \
        src/lerobot_vision/setup.py
git commit -m "feat: add eye-in-hand calibration launch files with easy_handeye2"
```
