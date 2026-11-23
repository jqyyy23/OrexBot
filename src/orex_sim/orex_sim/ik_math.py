"""
This nodes solves the joint velocity needed for each joint on the gun arm
for the gun's barrel to be pointed at the published target. 

It finds the angular error between where the gun's pointing to where it should point
using the cross product. Then, it applies a PD controller to minimize the angular displacement.
To find the control needed for indivual joints, I build a 3x3 Jacobian Matrix on the 3DOF arm
to preform inverse differential kinematics that calculates and publishes the joint velocities.
"""

import rclpy 
from rclpy.node import Node

# math for matrix operations
import numpy as np
from scipy.spatial.transform import Rotation as R

# for building the Jacobian matrix using TF2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import (
    TransformException,
    LookupException,
    ConnectivityException,
    ExtrapolationException
)

# for recieving position data of the target & publishing joint velocities
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

# PD Controller Variables
PROPORTIONAL = 2.0
DERIVATIVE = 0.1
FREQUENCY = 50 # Hz

class IKSolverNode(Node):
    def __init__(self):
        super().__init__("ik_solver")

        # Create buffer to store incoming frames & start listener 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishes joint velocities
        self.velocities_pub = self.create_publisher(Float64MultiArray, '/joint_velocities', 10)

        # Subscribes to the target position
        self.target_subscription = self.create_subscription(
            PoseStamped,           
            '/target_point',       
            self.target_callback,  
            10 # The QoS Profile (Queue Size), 10 is standard for coordinate tracking
        )

        self.Kp = PROPORTIONAL
        self.Kd = DERIVATIVE
        self.dt = 1.0 / FREQUENCY

        self.prev_error = np.array([0.0, 0.0, 0.0])
        self.target_vector = None # initiate to avoid crashes

        # Time-based loop for real-time control
        self.timer = self.create_timer(self.dt, self.control_loop_callback)

        # Print in terminal status of node
        self.get_logger().info("IK Solver Node is up and running!")

    def target_callback(self, msg):
        self.target_vector = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])


    def control_loop_callback(self):
        # Do nothing when target vector has not been spawned
        if self.target_vector is None:
            return
        
        # Order of joints to end effector: shoulder rotate (1) -> shoulder lift (2) -> gun pan (3) -> gun barrel (4)
        try:
            joint1 = self.tf_buffer.lookup_transform('root', 'shoul_bearing_shaft_v8', rclpy.time.Time())
            joint2 = self.tf_buffer.lookup_transform('root', 'should_to_upper_arm_back_v2_v4', rclpy.time.Time())
            joint3 = self.tf_buffer.lookup_transform('root', 'upper_arm_output_shaft_v7', rclpy.time.Time())
            joint4 = self.tf_buffer.lookup_transform('root', 'gun_barrel', rclpy.time.Time())

            # Extract quaternion components from joints in [x, y, z, w] format where x is real and y, z, w are imaginary
            quat1 = np.array([
                joint1.transform.rotation.x,
                joint1.transform.rotation.y,
                joint1.transform.rotation.z,
                joint1.transform.rotation.w])
            quat2 = np.array([
                joint2.transform.rotation.x,
                joint2.transform.rotation.y,
                joint2.transform.rotation.z,
                joint2.transform.rotation.w])
            quat3 = np.array([
                joint3.transform.rotation.x,
                joint3.transform.rotation.y,
                joint3.transform.rotation.z,
                joint3.transform.rotation.w])
            quat4 = np.array([
                joint4.transform.rotation.x,
                joint4.transform.rotation.y,
                joint4.transform.rotation.z,
                joint4.transform.rotation.w])

            # Quaternion -> 3x3 rotation matrix to extract axis informaton
            R1 = R.from_quat(quat1).as_matrix()
            R2 = R.from_quat(quat2).as_matrix()
            R3 = R.from_quat(quat3).as_matrix()
            R4 = R.from_quat(quat4).as_matrix()

            # Local rotational axis of each joint (from URDF):
            l_axis1 = np.array([0.0, 0.0, -1.0]) 
            l_axis2 = np.array([0.0, 0.0, -1.0]) 
            l_axis3 = np.array([0.0, 0.0, 1.0])

            # Global rotational axis for each joint:
            g_axis1 = R1 @ l_axis1
            g_axis2 = R2 @ l_axis2
            g_axis3 = R3 @ l_axis3

            # Constructing the jacobian matrix
            J_omega = np.stack((g_axis1, g_axis2, g_axis3))

            # Grabs the x-axis vector from the end effector (gun barrel)
            v_current = R4 @ np.array([1.0, 0.0, 0.0])
            v_current /= np.linalg.norm(v_current)

            # Subtract the target's position from the gun's position to find direction vector 
            gun_position = np.array([
                joint4.transform.translation.x,
                joint4.transform.translation.y,
                joint4.transform.translation.z
            ])
            v_desired = self.target_vector - gun_position
            v_desired /= np.linalg.norm(v_desired)

            # Calculating the angular error between target and gun barrel
            ang_error = np.cross(v_current, v_desired)

            # PD controller
            error_deriv = (ang_error - self.prev_error) / self.dt
            pd_error = (self.Kp * ang_error) + (self.Kd * error_deriv)

            # Updates current error for next iteration
            self.prev_error = ang_error

            # Calculate the Pseudoinverse of the Jacobian
            J_pseudo_inverse = np.linalg.pinv(J_omega)

            # Finds the joint velocities needed to aim at target 
            joint_velocities = J_pseudo_inverse @ pd_error

            # Create & publish message with the joint velocities
            joint_vel_msg = Float64MultiArray()
            joint_vel_msg.data = joint_velocities.flatten().tolist()
            self.velocities_pub.publish(joint_vel_msg)

        except (TransformException, LookupException, ConnectivityException, ExtrapolationException) as e:
            # If errors from TF2
            self.get_logger().warn(f'TF2 Error: {e}')
            return


def main(args=None):
    rclpy.init(args = args)

    node = IKSolverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()