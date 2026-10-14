import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    is_sim = LaunchConfiguration("is_sim")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("lerobot_description"),
                    "urdf",
                    "so101.urdf.xacro",
                ),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        condition=UnlessCondition(is_sim),
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": is_sim},
            os.path.join(
                get_package_share_directory("lerobot_controller"),
                "config",
                "so101_controllers.yaml",
            ),
        ],
        condition=UnlessCondition(is_sim),
    )

    spawner_common_args = [
        "-c", "/controller_manager",
        "--controller-manager-timeout", "600",
        "--service-call-timeout", "600",
    ]

    spawn_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "joint_state_broadcaster",
            "-t", "joint_state_broadcaster/JointStateBroadcaster",
        ] + spawner_common_args,
        output="screen",
    )

    spawn_arm_controller = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "arm_controller",
            "-t", "joint_trajectory_controller/JointTrajectoryController",
        ] + spawner_common_args,
        output="screen",
    )

    spawn_gripper_controller = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "gripper_controller",
            "-t", "joint_trajectory_controller/JointTrajectoryController",
        ] + spawner_common_args,
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

    return LaunchDescription(
        [
            is_sim_arg,
            robot_state_publisher_node,
            controller_manager,
            spawn_joint_state_broadcaster,
            arm_after_broadcaster,
            gripper_after_arm,
        ]
    )