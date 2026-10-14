# SO101 手眼标定方案 — 眼在手上 (Eye-in-Hand)

**日期：2026-05-25**

## 一、方案选型

使用成熟 ROS 2 生态包 **easy_handeye2**（[IFL-CAMP/easy_handeye2](https://github.com/IFL-CAMP/easy_handeye2)），不做自行开发。

标定类型：**eye_in_hand**（相机装在夹爪上，标定板固定在桌面）。

## 二、依赖安装

```bash
# ArUco 检测依赖
sudo apt install ros-humble-aruco-ros ros-humble-aruco-opencv -y

# easy_handeye2 源码安装到主工作空间
cd /disk2/so_arm_try/lerobot_ws/src
git clone https://github.com/IFL-CAMP/easy_handeye2.git
cd /disk2/so_arm_try/lerobot_ws
colcon build
```

## 三、四大坐标系

| 参数 | 值 | 含义 |
|------|-----|------|
| `robot_base_frame` | `base_link` | SO101 底座坐标系 |
| `robot_effector_frame` | `gripper` | 夹爪连杆（D405 相机固定在此连杆上） |
| `tracking_base_frame` | `camera_color_frame` | RealSense D405 彩色光学帧 |
| `tracking_marker_frame` | `marker_frame` | 桌面上标定板坐标系（ArUco/ChArUco 检测输出） |

**求解目标：** `T_cam2ee` — 相机相对于夹爪连杆的固定齐次变换矩阵。

## 四、新建文件清单

均在 `lerobot_vision` 包内：

| 文件 | 作用 |
|------|------|
| `launch/handeye_calibrate.launch.py` | 标定启动文件：ArUco 单标记检测 + easy_handeye2 calibrate |
| `launch/handeye_publish.launch.py` | 启动时发布标定结果静态 TF |
| `config/aruco_config.yaml` | 标定板参数（标记尺寸、字典 DICT_4X4_50、marker ID） |

## 五、标定操作流程

### 5.1 硬件准备

- 标定板（建议 ChArUco 或 ArUco）用胶带死死固定在桌面上，标定期间绝对不能碰
- RealSense D405 固定在 SO101 夹爪上
- **关键约束：标定板必须放在机械臂可到达的舒适位置（正前方偏下 30-50cm），确保从多个角度都能拍全。每次移动机械臂后，相机画面中必须能看到标定板，否则该姿态无法采集。**

### 5.2 启动命令

每个新终端先 source：

```bash
source /disk2/so_arm_try/lerobot_ws/install/setup.bash
source /disk2/so_arm_try/second_repo/lerobot_ros2/install/setup.bash
```

| 终端 | 命令 | 作用 |
|------|------|------|
| 1 | `ros2 run so101_hw_interface so101_motor_bridge --ros-args -p port:=/dev/ttyACM0` | 串口驱动（先启动） |
| 2 | `ros2 launch lerobot_controller so101_real_controller.launch.py` | ros2_control + 控制器 |
| 3 | `ros2 launch lerobot_moveit so101_moveit.launch.py` | MoveIt2 + RViz（发布 TF 树） |
| 4 | `ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=640x480x30 align_depth.enable:=true` | 相机驱动 |
| 5 | `ros2 launch lerobot_vision handeye_calibrate.launch.py` | ArUco 检测 + easy_handeye2 标定 GUI |

### 5.3 采集数据

1. RViz 左侧 **Motion Planning** 面板，**Planning Group** 选 `arm`
2. 拖动末端交互球改变机械臂姿态
   - **核心要领 1：不能只做 XYZ 平移，必须有大幅度的俯仰、偏航、翻滚（改变相机看标定板的倾斜角），否则方程无唯一解**
   - **核心要领 2：每次姿态变化后，确认相机画面中仍能看到标定板，否则 easy_handeye2 无法检测标记，该姿态白费**
3. 点击 **Plan** 查看绿色规划轨迹，确认安全后点 **Execute**
4. 等 1~2 秒（STS3215 舵机余震消除），在 easy_handeye2 GUI 点 **Take Sample**
5. 重复，采集 **15~20 组**不同姿态

### 5.4 解算与保存

1. 数据采够后，在 GUI 点击 **Compute**
2. 查看平移误差，2~5 mm 以内为优秀
3. 点击 **Save**，结果存入 `~/.ros/easy_handeye2/`

## 六、部署集成

### 6.1 启动时自动发布标定 TF

在真机全链路启动脚本中（或 `so101_real_controller.launch.py`），加入：

```python
# 发布手眼标定结果 TF（相机 → 夹爪）
from easy_handeye2.launch import publish
```

或直接命令行：

```bash
ros2 launch lerobot_vision handeye_publish.launch.py
```

easy_handeye2 会自动从 `~/.ros/easy_handeye2/` 读取标定结果，发布 `gripper → camera_color_frame` 的静态 TF。

### 6.2 坐标转换调用

`coordinate_transform.py` 无需改动。标定完成后，从 easy_handeye2 输出的 YAML 文件加载 4x4 矩阵 `T_cam2ee`，传入 `hand_eye_rotation` 和 `hand_eye_translation` 参数即可替代当前占位单位阵。

相机内参从 `/camera/camera/color/camera_info` topic 实时读取，替代 `vision_config.yaml` 中的硬编码值。

## 七、标定后 TF 树

```
base_link
  └── ... → gripper
              └── camera_color_frame  ← 标定结果发布的静态 TF
```

这样无论机械臂怎么动，系统都能实时推算出相机在空间中的绝对位置。
