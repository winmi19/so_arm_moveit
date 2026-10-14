"""真机启动: ros2_control + motor_bridge + MoveIt2

前提: 需要 source 两个仓库
  source /disk2/so_arm_try/lerobot_ws/install/setup.bash
  source /disk2/so_arm_try/second_repo/lerobot_ros2/install/setup.bash
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    desc_pkg = get_package_share_directory("lerobot_description")
    ctrl_pkg = get_package_share_directory("lerobot_controller")

    # ── URDF: 真机版本 (无 Gazebo, 用 TopicBasedSystem) ──
    robot_description = ParameterValue(
        Command([
            "xacro ",
            os.path.join(desc_pkg, "urdf", "so101_real.urdf.xacro"),
        ]),
        value_type=str,
    )

    # 1. robot_state_publisher (真机不用 sim_time)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # 2. ros2_control_node
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            os.path.join(ctrl_pkg, "config", "so101_controllers_real.yaml"),
        ],
        output="screen",
    )

    # 3. motor_bridge 需要先手动启动: ros2 run so101_hw_interface so101_motor_bridge ...

    # 4. 控制器 spawners (串行)
    spawner_base = [
        "-c", "/controller_manager",
        "--controller-manager-timeout", "600",
        "--service-call-timeout", "600",
    ]

    spawn_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "joint_state_broadcaster",
            "-t", "joint_state_broadcaster/JointStateBroadcaster",
        ] + spawner_base,
        output="screen",
    )

    spawn_arm_controller = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "arm_controller",
            "-t", "joint_trajectory_controller/JointTrajectoryController",
        ] + spawner_base,
        output="screen",
    )

    spawn_gripper_controller = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "gripper_controller",
            "-t", "joint_trajectory_controller/JointTrajectoryController",
        ] + spawner_base,
        output="screen",
    )

    arm_after_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_arm_controller],
        )
    )

    gripper_after_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_arm_controller,
            on_exit=[spawn_gripper_controller],
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        spawn_joint_state_broadcaster,
        arm_after_broadcaster,
        gripper_after_arm,
    ])
