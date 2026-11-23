"""
This node is in place for the actual hardware. It will integrate the velocity commands from IK solver node
and output the joint positions for visualization in RViz for testing.

To make it more realistic, I've added a transport delay in the form of a queue so there's a slight artifcial
latency before the arm actuallly matches the target position.
"""

import rclpy
from rclpy.node import Node

# Receive the joint velocities and publish to joint states for RViz
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

import time


class MockHardwareNode(Node):
    def __init__(self):
        super().__init__('mock_hardware_node')

        self.joint_names = ['gun_shoulder_rotate', 'gun_shoulder_lift', 'gun_pan']
        self.current_positions = [0.0, 0.0, 0.0]
        self.current_velocities = [0.0, 0.0, 0.0]

        self.dt = 0.02 # 50Hz
        self.latency = 0.05 # 50ms of artifical delay from servos

        # Buffer for the command queue
        self.command_buffer = []

        # Subs to joint velocities
        self.joint_velocity_sub = self.create_subscription(
            Float64MultiArray,
            '/joint_velocities',
            self.velocity_callback,
            10
        )

        self.joint_position_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Hardware loop, time driven because non-blocking 
        self.timer = self.create_timer(self.dt, self.hardware_loop)

        # Print in terminal node status
        try:
            self.get_logger().info("Interactive Target Node is up and running!")
        except Exception as e:
            self.get_logger().fatal(f"Failed to initialize Mock Hardware Node: {str(e)}")
            raise e 


    def velocity_callback(self, msg):
        recieve_time = time.time()
        self.command_buffer.append((recieve_time, msg.data))


    def hardware_loop(self):
        current_time = time.time()

        active_cmd = None

        # Set active command to oldest command (after latency time has passed) in the queue
        while len(self.command_buffer) > 0 and (current_time - self.command_buffer[0][0] >= self.latency):
            active_cmd = self.command_buffer.pop(0)

        # If delayed command came through, make it the current velocity to integrate
        if active_cmd is not None:
            self.current_velocities = active_cmd[1]

        # Integrate velocity to find new position
        for i in range(len(self.current_positions)):
            self.current_positions[i] += self.current_velocities[i] * self.dt  # pos. = speedd * time

        # Create joint states message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_positions

        # Publish positions to joint states for RViz
        self.joint_position_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = MockHardwareNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()