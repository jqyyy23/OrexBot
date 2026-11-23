from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orex_sim',
            executable='ik_math',  # The entry point name defined in setup.py
            name='ik_solver_node',
            output='screen'        # Prints node logs directly to your current terminal
        ),

        Node(
            package='orex_sim',
            executable='mock_hardware',  # The entry point name defined in setup.py
            name='mock_hardware_node',
            output='screen'        # Prints node logs directly to your current terminal
        ),

        Node(
            package='orex_sim',
            executable='test_target',  # The entry point name defined in setup.py
            name='fake_target_node',
            output='screen'        # Prints node logs directly to your current terminal
        )
    ])