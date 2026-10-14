"""FSM 决策节点 — IK + 关节轨迹方案

PICK: 像素+深度 → 相机3D → TF查camera→world → compute_ik → 关节轨迹
PLACE/RETURN: 预定义关节角轨迹
"""

import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_msgs.srv import GetPositionIK
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
import tf2_ros
from scipy.spatial.transform import Rotation

try:
    from lerobot_interfaces.msg import VisionDetection, RobotCommand
    HAVE_CUSTOM_MSGS = True
except ImportError:
    VisionDetection = None
    HAVE_CUSTOM_MSGS = False

from lerobot_decision.fsm import FiniteStateMachine, FSMState


class FSMNode(Node):

    def __init__(self):
        super().__init__('fsm_node')

        config_path = self.declare_parameter(
            'fsm_config',
            os.path.join(os.path.dirname(__file__), '..', 'config', 'fsm_config.yaml')
        ).value

        self.fsm = FiniteStateMachine(config_path)
        self.fsm.set_arm_callback(self.move_arm_to_pose)
        self.fsm.set_gripper_callback(self.move_gripper)
        self.fsm.set_servo_callback(self.servo_center)
        self.fsm.set_servo_approach_callback(self.servo_approach)
        self.fsm.set_joint_callback(self.move_joints)
        self.fsm.set_status_callback(self.publish_status)

        # 伺服参数
        self.servo_cx = self.fsm.servo_cfg.get('image_cx', 424)
        self.servo_cy = self.fsm.servo_cfg.get('image_cy', 240)
        self.servo_kp = self.fsm.servo_cfg.get('kp', 0.00008)
        self.servo_thresh = self.fsm.servo_cfg.get('pixel_threshold', 40)
        self.servo_step = self.fsm.servo_cfg.get('step_limit', 0.10)
        self.servo_max_iters = self.fsm.servo_cfg.get('max_iterations', 12)
        self.servo_settle = self.fsm.servo_cfg.get('settle_time', 0.8)
        self.approach_step = self.fsm.servo_cfg.get('approach_step', 0.10)
        self.grasp_depth = self.fsm.servo_cfg.get('grasp_depth', 0.03)
        self.lift_j2 = self.fsm.servo_cfg.get('lift_j2', 0.15)

        # 相机内参
        self.camera_fx = None
        self.camera_cx = None
        self.camera_fy = None
        self.camera_cy = None

        # 最新检测 + 关节
        self._latest_detection = None
        self._joint_positions = [0.0] * 6

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Action Clients
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

        # 订阅
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_cb, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_cb, 10)

        if HAVE_CUSTOM_MSGS:
            self.detection_sub = self.create_subscription(
                VisionDetection, '/vision/classified', self.detection_cb, 10)
        else:
            self.detection_sub = self.create_subscription(
                String, '/vision/classified', self.detection_cb_string, 10)

        self.status_pub = self.create_publisher(String, '/robot/status', 10)

        self.get_logger().info('FSM节点已启动 (IK + 关节轨迹)')

    # ── 订阅 ──────────────────────────────

    def joint_cb(self, msg):
        for i, name in enumerate(msg.name):
            if name.isdigit():
                idx = int(name) - 1
                if idx < 6:
                    self._joint_positions[idx] = msg.position[i]

    def camera_info_cb(self, msg):
        if self.camera_fx is None:
            self.camera_fx = msg.k[0]
            self.camera_fy = msg.k[4]
            self.camera_cx = msg.k[2]
            self.camera_cy = msg.k[5]
            self.get_logger().info(f'相机内参: fx={self.camera_fx:.1f}, fy={self.camera_fy:.1f}')

    def detection_cb(self, msg):
        self._latest_detection = msg
        if self.fsm.state == FSMState.IDLE and msg.depth > 0:
            self.get_logger().info(
                f'检测: {msg.object_class} pixel=({msg.x_pixel:.0f},{msg.y_pixel:.0f}) '
                f'depth={msg.depth:.3f}m')
            # 计算世界坐标并传给 FSM
            world_xyz = self._pixel_to_world(msg.x_pixel, msg.y_pixel, msg.depth)
            if world_xyz is not None:
                self.fsm.on_detection(world_xyz, msg.object_class)

    def detection_cb_string(self, msg):
        self.get_logger().info(f'检测 (string): {msg.data}')

    # ── 坐标转换: pixel → world ───────────

    def _pixel_to_world(self, x_pixel, y_pixel, depth):
        """像素 → 射线方向 → 与桌面平面 (Z=table_z) 求交 → 世界3D"""
        if self.camera_fx is None:
            return None

        x_norm = (x_pixel - self.camera_cx) / self.camera_fx
        y_norm = (y_pixel - self.camera_cy) / self.camera_fy

        try:
            t = self.tf_buffer.lookup_transform(
                'base', 'camera_color_optical_frame',
                rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().warn(f'TF camera→base 失败: {e}')
            return None

        trans = t.transform.translation
        rot = t.transform.rotation
        R = Rotation.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()
        cam_pos = np.array([trans.x, trans.y, trans.z])

        # 射线方向 (相机坐标系 → 世界坐标系)
        ray_dir = R @ np.array([x_norm, y_norm, 1.0])

        # 与桌面平面 Z = table_z 求交
        table_z = self.fsm.offsets.get('table_z', 0.0)
        dz = ray_dir[2]
        if abs(dz) < 0.001:
            return None
        lam = (table_z - cam_pos[2]) / dz
        p_base = cam_pos + lam * ray_dir

        # 加手动微调偏移 (补偿标定系统误差)
        dx = self.fsm.offsets.get('grasp_x', 0.0)
        dy = self.fsm.offsets.get('grasp_y', 0.0)
        dz = self.fsm.offsets.get('grasp_z', 0.0)
        return (float(p_base[0] + dx), float(p_base[1] + dy), float(p_base[2] + dz))

    # ── 视觉伺服 ───────────────────────────

    def servo_center(self, on_done):
        for i in range(self.servo_max_iters):
            det = self._latest_detection
            if det is None or det.depth <= 0:
                time.sleep(0.5); continue
            ex, ey = det.x_pixel - self.servo_cx, det.y_pixel - self.servo_cy
            self.get_logger().info(f'[伺服] {i+1} err=({ex:.0f},{ey:.0f})')
            if abs(ex) < self.servo_thresh and abs(ey) < self.servo_thresh:
                self.get_logger().info('[伺服] 居中')
                if on_done: on_done(); return
            if abs(ex) > self.servo_thresh:
                d = max(-self.servo_step, min(self.servo_step, -self.servo_kp * ex))
                self._send_arm_traj(['1'], [self._joint_positions[0] + d], 1.0)
            elif abs(ey) > self.servo_thresh:
                d = max(-self.servo_step, min(self.servo_step, self.servo_kp * ey))
                self._send_arm_traj(['2'], [self._joint_positions[1] + d], 1.0)
            time.sleep(self.servo_settle)
        if on_done: on_done()

    def servo_approach(self, on_done):
        for i in range(self.servo_max_iters):
            det = self._latest_detection
            depth = det.depth if det else 999
            self.get_logger().info(f'[下降] {i+1} depth={depth:.4f}')
            if 0 < depth < self.grasp_depth:
                self.get_logger().info('[下降] 到达')
                if on_done: on_done(); return
            self._send_arm_traj(['2'], [self._joint_positions[1] - self.approach_step], 1.0)
            time.sleep(self.servo_settle)
        if on_done: on_done()

    def move_joints(self, target, on_done):
        if target == 'LIFT':
            t = self._joint_positions[1] + self.lift_j2
            self._send_arm_traj_async(['2'], [t], 2.0, on_done)
        elif isinstance(target, (list, tuple)) and len(target) == 5:
            self._send_arm_traj_async(['1','2','3','4','5'], list(target), 3.0, on_done)
        else:
            if on_done: on_done()

    # ── IK + 关节轨迹 ────────────────────

    def move_arm_to_pose(self, target_xyz, on_done):
        x, y, z = float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2])
        self.get_logger().info(f'[IK] 目标: ({x:.3f},{y:.3f},{z:.3f})')

        if not self.ik_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('compute_ik 不可用')
            if on_done: on_done(); return

        seed = JointState()
        seed.name = ['1', '2', '3', '4', '5']
        seed.position = [float(p) for p in self._joint_positions[:5]]

        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.robot_state.joint_state = seed
        req.ik_request.pose_stamped = PoseStamped(
            header=self._make_header('base'),
            pose=Pose(position=Point(x=x, y=y, z=z),
                      orientation=Quaternion(x=0., y=0., z=0., w=1.)))
        req.ik_request.timeout.sec = 3

        future = self.ik_client.call_async(req)
        future.add_done_callback(lambda f: self._ik_done(f, on_done))

    def _ik_done(self, future, on_done):
        resp = future.result()
        if resp and resp.solution.joint_state.name:
            positions = list(resp.solution.joint_state.position[:5])
            self.get_logger().info(f'[IK] 关节角: {[f"{p:.3f}" for p in positions]}')
            self._send_arm_traj_async(['1','2','3','4','5'], positions, 3.0, on_done)
        else:
            self.get_logger().error(f'IK 失败')
            if on_done: on_done()

    def _make_header(self, frame):
        from std_msgs.msg import Header
        h = Header()
        h.frame_id = frame
        h.stamp = self.get_clock().now().to_msg()
        return h

    # ── 关节轨迹发送 ────────────────────────

    def _send_arm_traj(self, joint_names, positions, duration):
        all_names = ['1','2','3','4','5']
        all_pos = [p for p in self._joint_positions[:5]]
        for n, p in zip(joint_names, positions):
            idx = int(n) - 1
            if 0 <= idx < 5: all_pos[idx] = float(p)

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = all_names
        pt = JointTrajectoryPoint()
        pt.positions = all_pos
        pt.time_from_start.sec = int(duration)
        traj.points = [pt]
        goal.trajectory = traj

        future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        gh = future.result()
        if gh and gh.accepted:
            rf = gh.get_result_async()
            rclpy.spin_until_future_complete(self, rf, timeout_sec=duration + 3.0)

    def _send_arm_traj_async(self, joint_names, positions, duration, on_done):
        all_names = ['1','2','3','4','5']
        all_pos = [p for p in self._joint_positions[:5]]
        for n, p in zip(joint_names, positions):
            idx = int(n) - 1
            if 0 <= idx < 5: all_pos[idx] = float(p)

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = all_names
        pt = JointTrajectoryPoint()
        pt.positions = all_pos
        pt.time_from_start.sec = int(duration)
        traj.points = [pt]
        goal.trajectory = traj

        f = self.arm_client.send_goal_async(goal)
        f.add_done_callback(lambda fut: self._traj_sent(fut, on_done))

    def _traj_sent(self, future, on_done):
        gh = future.result()
        if gh and gh.accepted:
            gh.get_result_async().add_done_callback(lambda f: self._traj_done(f, on_done))
        else:
            if on_done: on_done()

    def _traj_done(self, future, on_done):
        if on_done: on_done()

    # ── 夹爪 ───────────────────────────────

    def move_gripper(self, position, on_done):
        self.get_logger().info(f'[夹爪] → {position:.2f}')
        if not self.gripper_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('gripper 不可用')
            if on_done: on_done(); return

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = ['6']
        pt = JointTrajectoryPoint()
        pt.positions = [float(position)]
        # 闭合(负值)给 3 秒, 张开(正值)给 2 秒
        duration = 3.0 if position < 0 else 2.0
        pt.time_from_start.sec = int(duration)
        traj.points = [pt]
        goal.trajectory = traj

        sf = self.gripper_client.send_goal_async(goal)
        sf.add_done_callback(lambda f: self._grip_sent(f, on_done))

    def _grip_sent(self, future, on_done):
        gh = future.result()
        if gh and gh.accepted:
            gh.get_result_async().add_done_callback(lambda f: self._grip_done(f, on_done))
        else:
            if on_done: on_done()

    def _grip_done(self, future, on_done):
        self.get_logger().info('[夹爪] 完成')
        if on_done: on_done()

    def publish_status(self, message):
        msg = String(); msg.data = message
        self.status_pub.publish(msg)
        self.get_logger().info(message)


def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.get_logger().info('FSM 被中断')
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
