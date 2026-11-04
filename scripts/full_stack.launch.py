"""
영양제 배달 AMR 전체 스택 launch 파일

실행:
  ros2 launch ./full_stack.launch.py
  ros2 launch ./full_stack.launch.py use_perception:=false use_mission:=false   # 주행 스택만
  ros2 launch ./full_stack.launch.py agent_device:=/dev/ttyACM1

사전 조건 (launch 실행 전 같은 터미널에서 source):
  source /opt/ros/humble/setup.bash
  source ~/lidar_ws/install/setup.bash
  source ~/bno055_ws/install/setup.bash
  source ~/perception_ws/install/setup.bash
  (micro-ROS agent 워크스페이스도 source 필요)

기동 순서:
  0s  : micro-ROS agent, 라이다, BNO055, IMU TF, wheel_odom relay, EKF
  3s  : SLAM (localization) - 라이다/odom TF가 먼저 올라와 있어야 함
  8s  : Nav2
  10s : 카메라, YOLO-World
  15s : schedule manager, supplement delivery

주의:
  - laser_frame static TF는 ydlidar launch 내부에서 이미 발행됨 (중복 실행 시 TF 충돌)
  - localization 모드에서는 RViz의 2D Pose Estimate로 초기 위치를 지정해야 함
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

HOME = os.path.expanduser('~')


def generate_launch_description():
    # ---------------- Launch arguments ----------------
    args = [
        DeclareLaunchArgument('use_perception', default_value='true',
                              description='카메라 + YOLO-World 실행 여부'),
        DeclareLaunchArgument('use_mission', default_value='true',
                              description='스케줄/배달 노드 실행 여부'),
        DeclareLaunchArgument('agent_device', default_value='/dev/ttyACM0',
                              description='STM32 micro-ROS 시리얼 장치'),
        DeclareLaunchArgument('agent_baud', default_value='115200'),
        DeclareLaunchArgument(
            'slam_params',
            default_value=f'{HOME}/rover_nav/config/my_slam_params.yaml'),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=f'{HOME}/nav2_config/nav2_params.yaml'),
        DeclareLaunchArgument(
            'ekf_params',
            default_value=f'{HOME}/rover_nav/config/ekf.yaml'),
        DeclareLaunchArgument(
            'bno055_params',
            default_value=f'{HOME}/bno055_ws/src/bno055/bno055/params/bno055_params_i2c.yaml'),
        DeclareLaunchArgument(
            'cam_params',
            default_value=f'{HOME}/cam_ws/config/usb_cam_params.yaml'),
        DeclareLaunchArgument(
            'wheel_odom_relay',
            default_value=f'{HOME}/rover_nav/scripts/wheel_odom_relay.py'),
        DeclareLaunchArgument(
            'venv_python',
            default_value=f'{HOME}/yolo_venv/bin/python3',
            description='YOLO/ultralytics가 설치된 venv의 python'),
        DeclareLaunchArgument(
            'perception_lib',
            default_value=f'{HOME}/perception_ws/install/yolo_perception/lib/yolo_perception'),
    ]

    use_perception = LaunchConfiguration('use_perception')
    use_mission = LaunchConfiguration('use_mission')
    venv_python = LaunchConfiguration('venv_python')
    perception_lib = LaunchConfiguration('perception_lib')

    # ---------------- 0s: 하드웨어 인터페이스 + odom ----------------
    microros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', LaunchConfiguration('agent_device'),
                   '-b', LaunchConfiguration('agent_baud')],
        output='screen',
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ydlidar_ros2_driver'), 'launch', 'ydlidar_launch.py'
        ])),
    )

    bno055 = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        parameters=[LaunchConfiguration('bno055_params')],
        output='screen',
    )

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_bno055',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'bno055'],
    )

    wheel_odom_relay = ExecuteProcess(
        cmd=['python3', LaunchConfiguration('wheel_odom_relay')],
        name='wheel_odom_relay',
        output='screen',
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[LaunchConfiguration('ekf_params')],
        remappings=[('/odometry/filtered', '/odom')],
        output='screen',
    )

    # ---------------- 3s: SLAM (localization) ----------------
    slam = TimerAction(period=3.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'
            ])),
            launch_arguments={
                'slam_params_file': LaunchConfiguration('slam_params'),
                'use_sim_time': 'false',
            }.items(),
        )
    ])

    # ---------------- 8s: Nav2 ----------------
    nav2 = TimerAction(period=8.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'
            ])),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': LaunchConfiguration('nav2_params'),
            }.items(),
        )
    ])

    # ---------------- 10s: Perception ----------------
    perception = TimerAction(period=10.0, actions=[
        GroupAction(condition=IfCondition(use_perception), actions=[
            Node(
                package='usb_cam',
                executable='usb_cam_node_exe',
                name='usb_cam',
                parameters=[LaunchConfiguration('cam_params')],
                output='screen',
            ),
            ExecuteProcess(
                cmd=[venv_python,
                     PathJoinSubstitution([perception_lib, 'yolo_world_node'])],
                name='yolo_world_node',
                output='screen',
            ),
        ])
    ])

    # ---------------- 15s: Mission ----------------
    mission = TimerAction(period=15.0, actions=[
        GroupAction(condition=IfCondition(use_mission), actions=[
            ExecuteProcess(
                cmd=['python3',
                     PathJoinSubstitution([perception_lib, 'schedule_manager_node'])],
                name='schedule_manager_node',
                output='screen',
            ),
            ExecuteProcess(
                cmd=[venv_python,
                     PathJoinSubstitution([perception_lib, 'supplement_delivery_node'])],
                name='supplement_delivery_node',
                output='screen',
            ),
        ])
    ])

    return LaunchDescription(args + [
        microros_agent,
        lidar,
        bno055,
        imu_tf,
        wheel_odom_relay,
        ekf,
        slam,
        nav2,
        perception,
        mission,
    ])
