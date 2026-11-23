#!/usr/bin/env python3
"""Diagnose the IMU ego-motion compensation.

Hold the drone STATIONARY, lock onto it, then pan/tilt the head (by hand or by letting it
track) while this runs. For a static target, perfect ego-motion compensation means the
reported target velocity stays ~0 even as the head turns. This measures the leak as a ratio:

    ratio = |target_velocity| / (depth * head_angular_rate)

    ratio ~ 0   -> compensation WORKING (velocity cancels)         GOOD
    ratio ~ 1   -> NOT compensating (raw rotation leaks through)    gyro unused / R_cur stuck
    ratio ~ 2   -> gyro SIGN INVERTED (motion is doubled)           negate rotvec in imu_callback

Only samples taken while the head is actually rotating (gyro rate above a threshold) count,
since the ratio is meaningless when the head is still.
 
Run (needs the vision node + camera running, drone locked):
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 src/orex_head_vision/utils/diagnose_egomotion.py
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped, Vector3
from std_msgs.msg import Bool

# Only count samples where the head is clearly rotating (rad/s), so noise doesn't dominate.
MIN_HEAD_RATE = 0.15
# Ignore implausibly close/zero depths.
MIN_DEPTH_M = 0.2


class EgoMotionDiagnostic(Node):
    def __init__(self):
        super().__init__('diagnose_egomotion')
        self.gyro_perp = 0.0     # sqrt(wx^2 + wy^2): rotation that moves the image centroid
        self.depth = None        # target_point.z (m)
        self.vel_mag = None      # |target_velocity| lateral (m/s)
        self.locked = False
        self.ratios = []

        self.create_subscription(Imu, '/camera/camera/imu', self._imu, qos_profile_sensor_data)
        self.create_subscription(PointStamped, '/head/target_point', self._point, 10)
        self.create_subscription(Vector3, '/head/target_velocity', self._vel, 10)
        self.create_subscription(Bool, '/head/target_locked', self._locked, 10)
        self.create_timer(0.1, self._sample)
        self.create_timer(2.0, self._report)
        self.get_logger().info('Ego-motion diagnostic: lock a STATIONARY drone, then move the head.')

    def _imu(self, m: Imu):
        wx, wy = m.angular_velocity.x, m.angular_velocity.y
        self.gyro_perp = math.hypot(wx, wy)  # roll (wz, about optical axis) barely shifts the centroid

    def _point(self, m: PointStamped):
        self.depth = float(m.point.z)

    def _vel(self, m: Vector3):
        # Lateral velocity (x,y) is what head rotation induces; z is depth drift.
        self.vel_mag = math.hypot(m.x, m.y)

    def _locked(self, m: Bool):
        self.locked = m.data

    def _sample(self):
        if not self.locked or self.depth is None or self.vel_mag is None:
            return
        if self.depth < MIN_DEPTH_M or self.gyro_perp < MIN_HEAD_RATE:
            return
        expected_uncompensated = self.depth * self.gyro_perp  # m/s the target would appear to move with NO comp
        if expected_uncompensated < 1e-6:
            return
        self.ratios.append(self.vel_mag / expected_uncompensated)

    def _report(self):
        if not self.ratios:
            self.get_logger().info(
                'waiting for samples... (need: locked target + head rotating). '
                'Is the drone locked and are you moving the head?')
            return
        n = len(self.ratios)
        mean = sum(self.ratios) / n
        self.ratios.sort()
        med = self.ratios[n // 2]
        if mean < 0.35:
            verdict = 'GOOD — ego-motion is cancelling head rotation.'
        elif mean < 0.7:
            verdict = 'PARTIAL — under-compensating (weak gain / stale IMU / some parallax).'
        elif mean < 1.4:
            verdict = 'NOT COMPENSATING — gyro effectively unused (R_cur stuck? IMU missing?).'
        elif mean < 2.5:
            verdict = 'SIGN INVERTED — negate rotvec in imu_callback (motion is being DOUBLED).'
        else:
            verdict = 'WORSE THAN NONE — likely wrong axis mapping or scale.'
        self.get_logger().info(
            f'ratio mean={mean:.2f} median={med:.2f}  (n={n})  ->  {verdict}')
        self.ratios.clear()


def main():
    rclpy.init()
    node = EgoMotionDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
