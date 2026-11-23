"""Head state / TF bring-up (Phase 1).

Turns the ESP32's head-joint feedback into a live TF tree so the drone's camera-frame
position can be transformed into base_link (and onward to the blaster). Starts:

  1. robot_state_publisher  — URDF + /joint_states  ->  /tf for every link.
  2. joint_state_publisher  — merges the ESP32's /head/joint_states (head_pan, head_tilt)
     with default 0.0 for every other movable joint (gun arm, etc.), publishing /joint_states.
  3. static_transform_publisher — head_camera_center -> camera_link. Identity, because
     head_camera_center already shares the camera_link convention (x-forward, y-left, z-up).
     The RealSense driver fills in camera_link -> camera_color_optical_frame itself.

Prereqs (once):  sudo apt install ros-jazzy-joint-state-publisher ros-jazzy-robot-state-publisher
Run:  ros2 launch orex_head_vision head_state.launch.py
Then the micro-ROS agent (so /head/joint_states flows) and, for visualization, RViz.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution


def generate_launch_description():
    urdf = PathJoinSubstitution(
        [FindPackageShare('orex_description'), 'urdf', 'robot_w_gun.urdf']
    )
    # Read the URDF at launch time into the robot_description parameter.
    robot_description = ParameterValue(Command(['cat ', urdf]), value_type=str)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            # Take head_pan/head_tilt from the ESP32; default every other joint to 0.0.
            parameters=[{'source_list': ['/head/joint_states'], 'rate': 30}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='head_camera_to_camera_link',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'head_camera_center',
                '--child-frame-id', 'camera_link',
            ],
        ),
    ])
