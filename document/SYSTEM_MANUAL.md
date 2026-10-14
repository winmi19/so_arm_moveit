# LeRobot SO101 真机 & 仿真 操作手册

## 一、前置准备

```bash
# 安装依赖 (仅首次)
sudo apt install ros-humble-topic-based-ros2-control python3-serial -y
sudo cp -r /disk2/miniconda3/envs/lerobot/lib/python3.10/site-packages/scservo_sdk \
           /usr/lib/python3/dist-packages/

# 编译两个仓库
cd /disk2/so_arm_try/lerobot_ws && colcon build
cd /disk2/so_arm_try/second_repo/lerobot_ros2 && colcon build
```

---

## 二、全链路架构

```
┌─────────────────────────────────────────────────────────┐
│                    规划层                                │
│  RViz (拖拽) → MoveIt2 (IK + 轨迹)                       │
│  Package: lerobot_moveit                                 │
│  Node: move_group                                        │
└──────────────────────┬──────────────────────────────────┘
                       │ Action: /arm_controller/follow_joint_trajectory
                       │ Action: /gripper_controller/follow_joint_trajectory
                       ▼
┌─────────────────────────────────────────────────────────┐
│                    控制层                                │
│  controller_manager → arm_controller (joint 1-5)        │
│                     → gripper_controller (joint 6)      │
│                     → joint_state_broadcaster           │
│  Package: lerobot_controller                             │
│  Node: ros2_control_node                                 │
└──────────┬───────────────────────┬──────────────────────┘
           │ 仿真                  │ 真机
           ▼                       ▼
┌──────────────────┐  ┌───────────────────────────────────┐
│   仿真执行层       │  │         真机执行层                  │
│                  │  │                                    │
│ Hardware:         │  │ Hardware:                         │
│  GazeboSimSystem │  │  TopicBasedSystem                  │
│                  │  │   ↓ Topic /joint_commands          │
│ Process: Gazebo   │  │  motor_bridge                      │
│  内嵌 ros2_control │  │   ↓ 串口 /dev/ttyACM0              │
│                  │  │  STS3215 舵机 ×6                   │
│ Package:         │  │                                    │
│ lerobot_description│ │ Package: so101_hw_interface       │
│                  │  │ Node: so101_motor_bridge            │
└──────────────────┘  └───────────────────────────────────┘
```

---

## 三、真机操作流程

### 3.1 环境准备

```bash
# 每个新终端都要 source 两个仓库
source /disk2/so_arm_try/lerobot_ws/install/setup.bash
source /disk2/so_arm_try/second_repo/lerobot_ros2/install/setup.bash
```

### 3.2 启动命令

```bash
# 终端1: 串口驱动层 (必须先启动)
ros2 run so101_hw_interface so101_motor_bridge \
  --ros-args -p port:=/dev/ttyACM0

# 终端2: ros2_control + 控制器 (等终端1显示连接成功后)
ros2 launch lerobot_controller so101_real_controller.launch.py

# 终端3: MoveIt2 + RViz
ros2 launch lerobot_moveit so101_moveit.launch.py
```

### 3.3 RViz 控制真机

1. 左侧 **Motion Planning** 面板
2. **Planning Group** 选 `arm`（机械臂）或 `gripper`（夹爪）
3. 拖拽末端彩色箭头/球到目标位置
4. **Planning Library** 选 `ompl`
5. 点 **Plan** → 看绿色轨迹
6. 点 **Execute** → 真机执行

### 3.4 命令行控制真机

```bash
# 全关节回零
ros2 action send_goal -f /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['1','2','3','4','5'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3, nanosec: 0}}]}}"

# 闭合夹爪
ros2 action send_goal -f /gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['6'], points: [{positions: [-0.17], time_from_start: {sec: 1, nanosec: 0}}]}}"

# 张开夹爪
ros2 action send_goal -f /gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['6'], points: [{positions: [1.74], time_from_start: {sec: 1, nanosec: 0}}]}}"

# 查看关节状态
ros2 topic echo /joint_states
```

---

## 四、仿真操作流程

```bash
source /disk2/so_arm_try/lerobot_ws/install/setup.bash

# 终端1: Gazebo
ros2 launch lerobot_description so101_gazebo.launch.py

# 终端2: ros2_control + 控制器
ros2 launch lerobot_controller so101_controller.launch.py

# 终端3: MoveIt2 + RViz
ros2 launch lerobot_moveit so101_moveit.launch.py

# RViz 操作同上，Plan → Execute 控制 Gazebo 中的机器人
```

---

## 五、节点拓扑

```
节点                          包                    功能
────────────────────────── ────────────────────── ──────────────────────
so101_motor_bridge          so101_hw_interface     串口↔话题双向桥
ros2_control_node           controller_manager     控制器管理器
move_group                  moveit_ros_move_group  IK求解+轨迹规划
rviz2                       rviz2                  可视化+交互
robot_state_publisher       robot_state_publisher  发布TF (URDF→TF)
detector_node               lerobot_vision         YOLOv8检测 (TODO)
classifier_node             lerobot_vision         颜色分类 (TODO)
fsm_node                    lerobot_decision       决策状态机 (TODO)
```

---

## 六、Topic 列表

```
Topic                            类型                    方向         说明
─────────────────────────────── ─────────────────────── ─────────── ─────────────────
/joint_states                    sensor_msgs/JointState  发布        当前关节角度
/arm_controller/joint_trajectory trajectory_msgs/JointTrajectory 订阅 arm控制命令
/arm_controller/state            control_msgs/JTCState   发布        arm状态
/arm_controller/controller_state control_msgs/CS         发布        arm控制器状态
/gripper_controller/joint_trajectory (同上)              订阅        夹爪控制命令
/gripper_controller/state        (同上)                  发布        夹爪状态
/so101_follower/joint_commands   sensor_msgs/JointState  订阅        真机: ros2_control→motor_bridge
/so101_follower/joint_states     sensor_msgs/JointState  发布        真机: motor_bridge→ros2_control
/tf                              tf2_msgs/TFMessage      发布        机器人坐标系
/tf_static                       tf2_msgs/TFMessage      发布        静态坐标系
/robot_description               String                   发布        URDF
/clock                           rosgraph_msgs/Clock     发布        仿真时钟
/dynamic_joint_states             control_msgs/DJState    发布        详细关节状态
/parameter_events                rcl_interfaces/...      发布        参数变更事件
```

---

## 七、Action 列表

```
Action                                    类型                       用途
─────────────────────────────────────── ────────────────────────── ─────────────────
/arm_controller/follow_joint_trajectory  FollowJointTrajectory      控制关节1-5
/gripper_controller/follow_joint_trajectory FollowJointTrajectory   控制关节6(夹爪)
```

---

## 八、终端配置速查

### 仿真

| 终端 | 命令 | 作用 |
|------|------|------|
| 1 | `ros2 launch lerobot_description so101_gazebo.launch.py` | Gazebo + 机器人 |
| 2 | `ros2 launch lerobot_controller so101_controller.launch.py` | ros2_control + 控制器 |
| 3 | `ros2 launch lerobot_moveit so101_moveit.launch.py` | MoveIt2 + RViz |

### 真机

| 终端 | 命令 | 作用 |
|------|------|------|
| 1 | `ros2 run so101_hw_interface so101_motor_bridge --ros-args -p port:=/dev/ttyACM0` | 串口驱动 |
| 2 | `ros2 launch lerobot_controller so101_real_controller.launch.py` | ros2_control + 控制器 + motor_bridge |
| 3 | `ros2 launch lerobot_moveit so101_moveit.launch.py` | MoveIt2 + RViz |

---

## 九、关键文件一览

```
主仓库 (lerobot_ws/src)
├── lerobot_description/urdf/
│   ├── so101.urdf.xacro              仿真入口 (include gazebo + GazeboSimSystem)
│   ├── so101_real.urdf.xacro         真机入口 (include TopicBasedSystem, 无gazebo)
│   ├── so101_ros2_control.xacro      仿真硬件 (GazeboSimSystem)
│   ├── so101_real_ros2_control.xacro 真机硬件 (TopicBasedSystem)
│   ├── so101_base.xacro              机器人本体 (关节/连杆/传动)
│   └── so101_gazebo.xacro            Gazebo插件配置
│
├── lerobot_controller/
│   ├── config/
│   │   ├── so101_controllers.yaml        仿真控制器配置 (use_sim_time:true)
│   │   └── so101_controllers_real.yaml   真机控制器配置 (无use_sim_time)
│   └── launch/
│       ├── so101_controller.launch.py     仿真启动
│       └── so101_real_controller.launch.py 真机启动
│
├── lerobot_moveit/config/
│   └── so101.srdf                     MoveIt2语义配置
│
├── lerobot_vision/                    视觉层 (YOLO+分类)
├── lerobot_decision/                  决策层 (FSM)
└── lerobot_interfaces/                自定义消息

second_repo (lerobot_ros2/src)
└── so101_hw_interface/
    ├── so101_hw_interface/
    │   ├── motor_bridge.py             串口↔话题核心
    │   └── motors/feetech/feetech.py   Feetech STS3215 协议
    ├── config/
    │   └── so101_calibration.yaml      舵机标定数据
    └── launch/
        └── so101_hw.launch.py
```

---

## 十、关节对照表

| 主仓库 | second_repo(旧) | 舵机ID | 模型 | 作用 |
|--------|----------------|--------|------|------|
| 1 | shoulder_pan | 1 | sts3215 | 底座旋转 |
| 2 | shoulder_lift | 2 | sts3215 | 肩部 |
| 3 | elbow_flex | 3 | sts3215 | 肘部 |
| 4 | wrist_flex | 4 | sts3215 | 腕部俯仰 |
| 5 | wrist_roll | 5 | sts3215 | 腕部旋转 |
| 6 | gripper | 6 | sts3215 | 夹爪 |
