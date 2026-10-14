# SO101 手眼标定调试记录

**日期：2026-05-25**

## 节点拓扑

```
终端 1: so101_motor_bridge          (so101_hw_interface)   串口驱动
终端 2: ros2_control_node            (lerobot_controller)  控制器管理
        robot_state_publisher         ↓                     TF: world→base→shoulder→upper_arm→lower_arm→wrist→gripper→jaw
终端 3: move_group                   (lerobot_moveit)       MoveIt2 + IK + RViz 交互
终端 4: camera                       (realsense2_camera)   发布:
          /camera/camera/color/image_raw          (848×480 BGR8)
          /camera/camera/color/camera_info         (fx≈434, D405)
          /tf_static: camera_link → camera_color_frame → camera_color_optical_frame
                      camera_link → camera_depth_frame → camera_depth_optical_frame
终端 5: aruco_detector               (lerobot_vision)      ArUco 检测 + 发布 TF (自定义)
        handeye_dummy_publisher      (tf2_ros)             gripper → camera_color_optical_frame (初始猜测)
        handeye_server               (easy_handeye2)       标定服务
        handeye_rqt_calibrator       (easy_handeye2)       标定 GUI
```

**数据流：**
```
camera /color/image_raw  ──→ aruco_detector ──→ /tf: camera_color_optical_frame → aruco_marker_0
camera /color/camera_info ──→ aruco_detector (POSE 估算)
                                                     ↓
                     handeye_server ←── /tf: base→gripper (robot_state_publisher)
                                     ←── /tf: camera_color_optical_frame→aruco_marker_0
                                     ←── /tf: gripper→camera_color_optical_frame (dummy)
```

**标定结果：** `~/.ros/easy_handeye2/so101_eih.yaml` → `handeye_publisher` → `/tf`

## 问题清单

### 问题 1: rqt_calibrator.py shebang 错误

**现象：** `FileNotFoundError: /install/easy_handeye2/lib/.../rqt_calibrator.py`

**原因：** easy_handeye2 安装后的 shebang 是 `#!python`，系统找不到解释器

**解决：** 将 shebang 替换为 `#!/usr/bin/python3`

---

### 问题 2: TF 帧名不匹配 — `base_link` vs `base`

**现象：** `"base_link" passed to lookupTransform argument target_frame does not exist`

**原因：** SO101 URDF 中底座连杆名为 `base`，不是通用的 `base_link`

**解决：** 启动时指定 `robot_base_frame:=base`（已写入 launch 默认值）

---

### 问题 3: 相机 TF 帧名不匹配 — `camera_color_frame` vs `camera_color_optical_frame`

**现象：** `"camera_color_frame" does not exist`

**原因：** RealSense D405 遵循 REP-118，光学帧名为 `camera_color_optical_frame`

**解决：** `tracking_base_frame:=camera_color_optical_frame`（已写入 launch 默认值）

---

### 问题 4: aruco_ros::single 无 dictionary 参数

**现象：** ArUco 检测不到标记，`markers: []`

**原因：** `aruco_ros` 包的 `single` 节点没有 `dictionary` 参数，默认字典与 DICT_4X4_50 不兼容，无法检测到 4×4 标记

**解决：** 放弃 `aruco_ros`，换用 `aruco_opencv`

---

### 问题 5: aruco_opencv::aruco_tracker 是 lifecycle 节点

**现象：** Subscription count = 0，节点不订阅图像

**原因：** `aruco_tracker` 需要 lifecycle 激活（configure→activate），静默停在 unconfigured 状态

**解决：** 换用 `aruco_tracker_autostart`（自动完成 lifecycle 激活）

---

### 问题 6: aruco_opencv 默认检测参数过严

**现象：** 手动 OpenCV 脚本能检测到标记，但 aruco_opencv 节点输出 `markers: []`

**原因：** aruco_opencv 的默认 `DetectorParameters` 过于严格：
- `adaptiveThreshWinSizeMax=23` 太小
- `minMarkerPerimeterRate=0.03` 太高
- `errorCorrectionRate=0.6` 不够宽容

**解决：** 放松参数：
```yaml
adaptiveThreshWinSizeMin: 3
adaptiveThreshWinSizeMax: 30
minMarkerPerimeterRate: 0.01
maxErroneousBitsInBorderRate: 0.5
errorCorrectionRate: 1.0
polygonalApproxAccuracyRate: 0.05
```

---

### 问题 7: aruco_opencv TF 发布失效

**现象：** `publish_tf: True` 已设置，节点发布到 `/tf`，但 `aruco_marker_0` 帧从不出现

**原因：** 未知（节点发布到 `/tf` 但标记帧被丢弃或静默失败）

**解决：** 放弃 aruco_opencv，自建 `aruco_detector.py` 节点直接调 OpenCV API

---

### 问题 8: cv2.aruco.estimatePoseSingleMarkers 不存在

**现象：** `AttributeError: module 'cv2.aruco' has no attribute 'estimatePoseSingleMarkers'`

**原因：** `estimatePoseSingleMarkers` 是旧 OpenCV API（<4.6），当前环境 OpenCV 已移除该函数

**解决：** 手动构建目标点，改用 `cv2.solvePnP`：
```python
half = marker_size / 2.0
obj_points = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]])
ok, rvec, tvec = cv2.solvePnP(obj_points, corners[idx][0], camera_matrix, dist_coeffs)
```

---

### 问题 9: setup.py 可执行文件权限

**现象：** `executable 'aruco_detector.py' not found on the libexec directory`

**原因：** 新建的 `scripts/aruco_detector.py` 缺少可执行权限（644），colcon 保留源文件权限到 install

**解决：** `chmod +x scripts/aruco_detector.py`

---

## 最终方案

自建 `aruco_detector.py` 节点，绕过 aruco_ros 和 aruco_opencv 的兼容性问题：

- 直接订阅 `sensor_msgs/Image` + `CameraInfo`
- 直接调 `cv2.aruco.ArucoDetector` + `cv2.solvePnP`
- 直接通过 `tf2_ros.TransformBroadcaster` 发布 TF
- 使用 `scipy.spatial.transform.Rotation` 做旋转矩阵→四元数转换
