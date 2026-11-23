"""One-shot bringup for the OrexBot drone tracker (host side).

Starts, in one command:
  * the RealSense camera (with IMU: gyro + accel, united),
  * the vision_node tracker (run under the workspace .venv so Ultralytics loads), and
  * the micro-ROS agent that bridges the ESP32 head controller.

The ESP32 itself is NOT started here — it runs the flashed firmware on the chip; the
agent connects to it over serial. Flash + power it separately.

Usage:
    # source ROS, the OrexBot install, AND the agent workspace first:
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    source ~/uros_ws/install/setup.bash
    ros2 launch orex_head_vision bringup.launch.py agent_port:=/dev/ttyACM1

Handy overrides:
    agent_port:=/dev/ttyACM1     # the S3's native-USB port re-enumerates; check `ls -t /dev/ttyACM*`
    start_agent:=false           # run the agent yourself in its own terminal instead
    start_camera:=false          # camera already running elsewhere
    venv_python:=/path/to/python # interpreter that has Ultralytics
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_venv = os.path.join(os.path.expanduser('~'), 'github', 'OrexBot', '.venv', 'bin', 'python')

    args = [
        DeclareLaunchArgument('start_camera', default_value='true',
                              description='Start the RealSense camera'),
        DeclareLaunchArgument('start_vision', default_value='true',
                              description='Start the vision_node tracker'),
        DeclareLaunchArgument('start_agent', default_value='true',
                              description='Start the micro-ROS agent (requires ~/uros_ws sourced)'),
        DeclareLaunchArgument('agent_port', default_value='/dev/ttyACM0',
                              description='ESP32 serial device; native USB re-enumerates, '
                                          'check `ls -t /dev/ttyACM*`'),
        DeclareLaunchArgument('venv_python', default_value=default_venv,
                              description='Python interpreter with Ultralytics (workspace .venv)'),
        DeclareLaunchArgument('camera_profile', default_value='640x480x30',
                              description='RealSense color+depth profile WxHxFPS. Keep low on '
                                          'WSL2 (usbipd bandwidth) — 640x480x30, or 640x480x15 '
                                          'if you still see "Incomplete video frame" warnings.'),
    ]

    # RealSense camera — include its own launch file with the known-good IMU args.
    rs_launch = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch),
        condition=IfCondition(LaunchConfiguration('start_camera')),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '1',
            # Lower resolution keeps the USB bandwidth within what WSL2's usbipd can carry,
            # avoiding "Incomplete video frame / Frame Corrupted". 640x480 is ample for YOLO.
            'rgb_camera.color_profile': LaunchConfiguration('camera_profile'),
            'depth_module.depth_profile': LaunchConfiguration('camera_profile'),
        }.items(),
    )

    # vision_node — run the installed console script under the venv Python (via prefix) so
    # YOLO/Ultralytics import regardless of the installed shebang.
    vision = Node(
        condition=IfCondition(LaunchConfiguration('start_vision')),
        package='orex_head_vision',
        executable='vision_node',
        name='vision_tracking',
        prefix=[LaunchConfiguration('venv_python')],
        output='screen',
    )
    # Give the camera a few seconds to advertise its topics before vision subscribes.
    vision_delayed = TimerAction(period=4.0, actions=[vision])

    # micro-ROS agent — bridges the ESP32 (/head_controller). Needs ~/uros_ws sourced so
    # the package resolves.
    agent = Node(
        condition=IfCondition(LaunchConfiguration('start_agent')),
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', LaunchConfiguration('agent_port'), '-v4'],
        output='screen',
    )

    return LaunchDescription(args + [camera, agent, vision_delayed])
