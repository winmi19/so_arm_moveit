"""FSM 状态机核心 — MoveIt2 + 关节轨迹方案

状态转换:
    IDLE → DETECT → PICK → PLACE → RETURN → IDLE

PICK 阶段使用 MoveIt2 规划 (绝对坐标)。
PLACE/RETURN 使用预定义关节角。
"""

import os
import yaml


class FSMState:
    IDLE = 'IDLE'
    DETECT = 'DETECT'
    PICK = 'PICK'
    PLACE = 'PLACE'
    RETURN = 'RETURN'


class FiniteStateMachine:

    def __init__(self, config_path=None):
        self.state = FSMState.IDLE
        self.current_detection = None
        self.config = self._load_config(config_path)

        self.place_zones = self.config.get('place_zones', {})
        self.gripper = self.config.get('gripper', {})
        self.servo_cfg = self.config.get('servo', {})
        self.offsets = self.config.get('offsets', {})
        self.home_joints = self.config.get('home_joints', [0,0,0,0,0])
        self.traj_times = self.config.get('trajectory_time', {})

        self._arm_callback = None
        self._gripper_callback = None
        self._servo_callback = None
        self._servo_approach_callback = None
        self._joint_callback = None
        self._status_callback = None

    def _load_config(self, config_path):
        if config_path is None:
            config_path = os.path.join(
                os.path.dirname(__file__), '..', 'config', 'fsm_config.yaml')
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        return {}

    def set_arm_callback(self, cb):
        self._arm_callback = cb
    def set_gripper_callback(self, cb):
        self._gripper_callback = cb
    def set_servo_callback(self, cb):
        self._servo_callback = cb
    def set_servo_approach_callback(self, cb):
        self._servo_approach_callback = cb
    def set_joint_callback(self, cb):
        self._joint_callback = cb
    def set_status_callback(self, cb):
        self._status_callback = cb

    def transition(self, new_state):
        old = self.state
        self.state = new_state
        if self._status_callback:
            self._status_callback(f'[FSM] {old} → {new_state}')
        return new_state

    def on_detection(self, pick_pose_3d, object_class):
        if self.state != FSMState.IDLE:
            return

        self.current_detection = {
            'pose': pick_pose_3d, 'object_class': object_class
        }
        self.transition(FSMState.DETECT)

        zone = self.place_zones.get(object_class)
        if zone is None:
            self._status_callback(f'未知物体类型: {object_class}')
            self.transition(FSMState.IDLE)
            return

        place_joints = zone.get('joints', self.home_joints)
        self._do_pick(pick_pose_3d, place_joints)

    def _do_pick(self, pick_xyz, place_joints):
        self.transition(FSMState.PICK)

        px, py, pz = pick_xyz
        pregrasp_z = self.offsets.get('pregrasp_z', 0.08)
        gox = self.offsets.get('grasp_x', 0.0)
        goy = self.offsets.get('grasp_y', 0.0)
        goz = self.offsets.get('grasp_z', 0.0)
        grip_open = self.gripper.get('open', 1.74)
        grip_close = self.gripper.get('close', -0.17)

        # IK + 关节轨迹: 预抓取 → 下降 → 抬起
        gx, gy, gz = px + gox, py + goy, pz + goz
        steps = [
            ('张开夹爪',      None,                         grip_open),
            ('移动到预抓取位', (gx, gy, gz + pregrasp_z),    None),
            ('下降到抓取位',   (gx, gy, gz),                 None),
            ('闭合夹爪',       None,                         grip_close),
            ('抬起物体',       (gx, gy, gz + pregrasp_z),    None),
        ]
        self._execute_steps(steps, on_done=lambda: self._do_place(place_joints))

    def _do_place(self, place_joints):
        self.transition(FSMState.PLACE)
        grip_open = self.gripper.get('open', 1.74)

        steps = [
            ('移动到放置区', place_joints,    None),
            ('张开释放',     None,            grip_open),
            ('回到初始位',   self.home_joints, None),
        ]
        self._execute_steps(steps, on_done=lambda: self.transition(FSMState.IDLE))

    def _execute_steps(self, steps, on_done):
        if not steps:
            if on_done: on_done()
            return

        desc, target_pose, gripper_pos = steps[0]
        remaining = steps[1:]
        if self._status_callback:
            self._status_callback(desc)

        if gripper_pos is not None and target_pose is not None:
            self._gripper_callback(gripper_pos, lambda: None)
            self._dispatch_arm(target_pose,
                               lambda: self._execute_steps(remaining, on_done))
        elif gripper_pos is not None:
            self._gripper_callback(gripper_pos,
                                   lambda: self._execute_steps(remaining, on_done))
        elif target_pose is not None:
            self._dispatch_arm(target_pose,
                               lambda: self._execute_steps(remaining, on_done))
        else:
            self._execute_steps(remaining, on_done)

    def _dispatch_arm(self, target_pose, on_done):
        if target_pose == 'SERVO' and self._servo_callback:
            self._servo_callback(on_done)
        elif target_pose == 'APPROACH' and self._servo_approach_callback:
            self._servo_approach_callback(on_done)
        elif target_pose == 'LIFT' and self._joint_callback:
            self._joint_callback('LIFT', on_done)
        elif isinstance(target_pose, (list, tuple)) and len(target_pose) == 5:
            if self._joint_callback:
                self._joint_callback(target_pose, on_done)
        elif isinstance(target_pose, (list, tuple)) and len(target_pose) == 3:
            if self._arm_callback:
                self._arm_callback(target_pose, on_done)
        elif self._joint_callback:
            self._joint_callback(target_pose, on_done)
