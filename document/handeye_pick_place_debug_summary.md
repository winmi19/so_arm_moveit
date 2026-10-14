# SO101 手眼标定 + 抓取放置 踩坑总结

**日期：2026-05-25 ~ 2026-05-27**

---

## 一、手眼标定阶段

### 坑 1: easy_handeye2 rqt_calibrator.py shebang 错误
- **现象：** `FileNotFoundError: rqt_calibrator.py`
- **原因：** 安装后 shebang 是 `#!python`，系统找不到解释器
- **解决：** 改 shebang 为 `#!/usr/bin/python3`

### 坑 2: TF 帧名不匹配
- **现象：** `base_link does not exist`
- **原因：** SO101 URDF 底座帧名是 `base`，不是通用的 `base_link`
- **解决：** 所有地方用 `base`，写入 launch 默认值

### 坑 3: 相机 TF 帧名不匹配
- **现象：** `camera_color_frame does not exist`
- **原因：** RealSense D405 遵循 REP-118，光学帧名为 `camera_color_optical_frame`
- **解决：** 统一用 `camera_color_optical_frame`

### 坑 4: aruco_ros::single 无法检测 DICT_4X4_50
- **现象：** 标定板在画面中但检测不到
- **原因：** `aruco_ros` 的 `single` 节点没有 `dictionary` 参数，默认字典与 DICT_4X4_50 不兼容
- **解决：** 换用 `aruco_opencv::aruco_tracker`

### 坑 5: aruco_opencv lifecycle 节点未激活
- **现象：** Subscription count = 0
- **原因：** `aruco_tracker` 需要 lifecycle 激活，默认停在 unconfigured
- **解决：** 换用 `aruco_tracker_autostart`

### 坑 6: aruco_opencv 检测参数过严
- **现象：** 手动 OpenCV 能检测，aruco_opencv 不行
- **原因：** 默认 DetectorParameters 过于严格
- **解决：** 放松 `adaptiveThreshWinSizeMax=30`, `minMarkerPerimeterRate=0.01`, `errorCorrectionRate=1.0`

### 坑 7: aruco_opencv TF 不发布
- **现象：** `publish_tf: True` 但 aruco_marker_0 帧不出现
- **原因：** 未知（静默失败）
- **解决：** **放弃 aruco_ros 和 aruco_opencv**，自建 `aruco_detector.py` 节点直接调 OpenCV API + `tf2_ros.TransformBroadcaster`

### 坑 8: cv2.aruco.estimatePoseSingleMarkers 不存在
- **现象：** `AttributeError: no attribute 'estimatePoseSingleMarkers'`
- **原因：** OpenCV >4.6 移除了旧 API
- **解决：** 手动构建 object points，改用 `cv2.solvePnP`

---

## 二、抓取放置阶段

### 坑 9: MoveGroup 全部规划失败 (code=99999)
- **现象：** MoveGroup goal accepted 但 result 全是 99999
- **原因：** 规划帧 `world` vs 约束帧 `base` 不匹配；arm 规划组末端是 `wrist` 不是 `gripper`
- **尝试的修复：** 改 `frame_id='world'`, `link_name='wrist'`
- **最终结果：** 即使修复后，位置约束的 MoveGroup 仍全部失败，**彻底放弃 MoveGroup 路径规划**

### 坑 10: compute_ik 通过但坐标算错
- **现象：** IK 对 (0.10, 0.00, 0.15) 成功，对我们算出的 (-0.003, 0.287, 0.444) 失败
- **原因：** **相机 Roll=-94°，镜头朝右不朝桌面。** depth 沿相机 Z 轴跑到了世界 Y 方向，目标偏出 workspace
- **解决：** 手动在 RViz 把机械臂压到相机朝桌面的姿态（Roll ≈ -37°），目标坐标大幅改善

### 坑 11: 深度采样拍到夹爪/线缆
- **现象：** depth=0.066~0.095m（夹爪距离），实际桌面 0.16m+
- **原因：** YOLO 检测中心 (y=100~137) 在图上半部，采样窗口拍到夹爪结构件
- **解决：** 检测 y < h/2 时，直接用图像下半部桌面区中位深度，不再在检测点取样

### 坑 12: 绝对坐标 Z 偏差巨大
- **现象：** 目标 Z 从 0.07 到 0.77，反复跳动
- **原因：** depth 沿倾斜的相机光轴映射到世界 Z 时，相机 Roll 角导致 Z 被放大
- **解决：** **放弃 depth→Z 的直接映射。** 改用射线-平面求交：从相机发射线穿过像素，与 Z=table_z 桌面平面求交，XY 由几何算出，Z 固定为桌面高度

### 坑 13: TF 树频繁断裂
- **现象：** `Tf has two or more unconnected trees`
- **原因：** handeye_publisher 只发一次 static TF，发送时若 gripper 帧未就绪则丢失
- **解决：** 严格遵守启动顺序：终端 1→2→3→4→**最后 5（handeye_publisher）**。TF 断裂时重启终端 5

### 坑 14: 夹爪行程不够
- **现象：** 夹爪还没闭合，臂就开始抬起
- **原因：** 闭合轨迹 `time_from_start=1s` 太短，STS3215 舵机 1 秒走不完行程
- **解决：** 闭合改 3 秒，张开改 2 秒

### 坑 15: 物体像素位置影响可达性
- **现象：** cup (499,137) IK 通过，carrot (592,424) IK 失败
- **原因：** 不同像素对应桌上不同位置，图边缘的物体射线交到 workspace 外
- **解决：** 物体放在图像中央区域测试

### 坑 16: 手眼标定系统偏差
- **现象：** 臂每次都偏同一个方向（前 5cm）
- **原因：** 12 组样本、纯舵机臂误差累积
- **解决：** `fsm_config.yaml` 加 `grasp_x`, `grasp_y`, `grasp_z` 手动微调补偿，通过 colcon build 快速迭代

---

## 三、最终可用方案

```
视觉检测(YOLO) → 深度(桌面区中位值) → 射线-平面求交(XY) → compute_ik → arm_controller
                 ↑ 避夹爪采样策略          ↑ Z固定桌面高度   ↑ 跳过MoveGroup
```

### 关键参数 (`fsm_config.yaml`)

| 参数 | 值 | 作用 |
|------|-----|------|
| `table_z` | 0.14 | 桌面在基座坐标系高度 |
| `pregrasp_z` | 0.05 | 预抓取高于桌面 |
| `grasp_x` | -0.02 | X 手动补偿 |
| `grasp_y` | -0.025 | Y 手动补偿 |
| 夹爪闭合时间 | 3.0s | 舵机走完物理行程 |

### 启动顺序

| 终端 | 命令 |
|------|------|
| 1 | `ros2 run so101_hw_interface so101_motor_bridge --ros-args -p port:=/dev/ttyACM0` |
| 2 | `ros2 launch lerobot_controller so101_real_controller.launch.py` |
| 3 | `ros2 launch lerobot_moveit so101_moveit.launch.py` |
| 4 | `ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=640x480x30 align_depth.enable:=true publish_tf:=false` |
| 5 | `ros2 launch lerobot_vision handeye_publish.launch.py name:=so101_eih_3 &`（最后启） |
| 6 | `ros2 launch lerobot_vision vision_pipeline.launch.py` |
| 7 | `ros2 launch lerobot_decision decision_fsm.launch.py` |

### 核心教训

1. **相机必须朝桌面。** 眼在手上时，机械臂姿态决定了相机朝向。Roll 接近 0° 或 180° 才朝下
2. **深度不能直接在检测点取。** 眼在手上必拍到夹爪，要偏下方/桌面区采样
3. **不用 depth 算 Z。** 射线-平面求交，Z 固定桌面高度，更稳定
4. **MoveGroup 不可靠时用 compute_ik。** 直接算关节角发给 arm_controller
5. **人工补偿是工程常态。** 系统偏差不改算法，改 config offset
