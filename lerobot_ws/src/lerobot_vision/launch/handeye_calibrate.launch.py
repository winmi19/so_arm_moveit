"""手眼标定启动文件 — 眼在手上 (eye-in-hand)

自包含 ChArUco 板检测 + easy_handeye2 标定服务 + GUI

Usage:
    ros2 launch lerobot_vision handeye_calibrate.launch.py \
        square_length:=0.0225 marker_length:=0.020
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    squares_x_arg = DeclareLaunchArgument(
        'squares_x', default_value='9',
        description='ChArUco 棋盘格 X 方向方块数'
    )
    squares_y_arg = DeclareLaunchArgument(
        'squares_y', default_value='9',
        description='ChArUco 棋盘格 Y 方向方块数'
    )
    square_length_arg = DeclareLaunchArgument(
        'square_length', default_value='0.0225',
        description='棋盘格单一方格边长 (米)'
    )
    marker_length_arg = DeclareLaunchArgument(
        'marker_length', default_value='0.020',
        description='ArUco 标记边长 (米)'
    )
    marker_dict_arg = DeclareLaunchArgument(
        'marker_dict', default_value='DICT_6X6_250',
        description='ArUco 字典: DICT_4X4_50, DICT_6X6_250 等'
    )
    marker_frame_arg = DeclareLaunchArgument(
        'marker_frame', default_value='charuco_board',
        description='ChArUco 板 TF 帧名'
    )

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
        'robot_base_frame', default_value='base',
        description='机械臂底座 TF 帧'
    )
    robot_effector_frame_arg = DeclareLaunchArgument(
        'robot_effector_frame', default_value='gripper',
        description='相机所固定的机械臂连杆 TF 帧'
    )
    tracking_base_frame_arg = DeclareLaunchArgument(
        'tracking_base_frame', default_value='camera_color_optical_frame',
        description='相机光学帧'
    )

    # ── ChArUco 板检测 + TF 发布 ──────────────
    aruco_detector = Node(
        package='lerobot_vision',
        executable='aruco_detector.py',
        name='aruco_detector',
        output='screen',
        parameters=[{
            'squares_x': LaunchConfiguration('squares_x'),
            'squares_y': LaunchConfiguration('squares_y'),
            'square_length': LaunchConfiguration('square_length'),
            'marker_length': LaunchConfiguration('marker_length'),
            'marker_dict': LaunchConfiguration('marker_dict'),
            'parent_frame': LaunchConfiguration('tracking_base_frame'),
            'marker_frame': LaunchConfiguration('marker_frame'),
        }],
    )

    # ── 初始猜测 TF (gripper → camera) ──────────
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
            'tracking_marker_frame': LaunchConfiguration('marker_frame'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
        }],
    )

    # ── easy_handeye2 标定 GUI ───────────────────
    handeye_gui = Node(
        package='easy_handeye2',
        executable='rqt_calibrator.py',
        name='handeye_rqt_calibrator',
        parameters=[{
            'name': LaunchConfiguration('name'),
            'calibration_type': LaunchConfiguration('calibration_type'),
            'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
            'tracking_marker_frame': LaunchConfiguration('marker_frame'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
        }],
    )

    return LaunchDescription([
        squares_x_arg,
        squares_y_arg,
        square_length_arg,
        marker_length_arg,
        marker_dict_arg,
        marker_frame_arg,
        name_arg,
        calibration_type_arg,
        robot_base_frame_arg,
        robot_effector_frame_arg,
        tracking_base_frame_arg,
        aruco_detector,
        dummy_tf,
        handeye_server,
        handeye_gui,
    ])
