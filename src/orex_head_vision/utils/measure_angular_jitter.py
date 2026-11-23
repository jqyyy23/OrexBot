#!/usr/bin/env python3
"""Measure the residual jitter on /head/angular_error, in degrees.

Use it to size the firmware deadband: hold the target STILL and roughly centered,
with the head NOT moving (stop the micro-ROS agent / head control so the closed loop
isn't chasing), run this, and read the numbers.

    python3 measure_angular_jitter.py [seconds]

Reported per axis (yaw = pan, pitch = tilt):
    mean  - static centering offset (should be small if roughly centered)
    std   - 1-sigma jitter
    p2p   - peak-to-peak spread over the window

Rule of thumb: set the deadband a bit ABOVE p2p of the noisier axis so the head sits
still within the jitter band. Deadband(rad) = p2p_deg * pi / 180.
"""
import math
import statistics as st
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3


class JitterMeter(Node):
    def __init__(self, secs):
        super().__init__('jitter_meter')
        self.secs = secs
        self.yaw = []
        self.pitch = []
        self.t0 = None
        self.done = False
        self.create_subscription(Vector3, '/head/angular_error', self.cb, 50)
        self.get_logger().info(
            f'Collecting /head/angular_error for {secs:.0f}s — hold the target still + centered...')

    def cb(self, m):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = now
        self.yaw.append(m.x)
        self.pitch.append(m.y)
        if now - self.t0 >= self.secs:
            self.done = True

    def report(self):
        def stats(rad_samples, name):
            deg = [v * 180.0 / math.pi for v in rad_samples]
            if len(deg) < 2:
                print(f'  {name}: too few samples ({len(deg)}) — is the target detected/LOCKED?')
                return
            print(f'  {name}: n={len(deg):4d}  mean={st.mean(deg):+.3f}°  '
                  f'std={st.pstdev(deg):.3f}°  p2p={max(deg) - min(deg):.3f}°')
        print('\n=== /head/angular_error jitter (degrees) ===')
        stats(self.yaw, 'yaw  (pan) ')
        stats(self.pitch, 'pitch(tilt)')
        allp2p = []
        for a in (self.yaw, self.pitch):
            if len(a) >= 2:
                d = [v * 180.0 / math.pi for v in a]
                allp2p.append(max(d) - min(d))
        if allp2p:
            worst = max(allp2p)
            print(f'\nSuggested deadband >= {worst:.3f}°  =  {worst * math.pi / 180.0:.4f} rad '
                  f'(set PAN_DEADBAND_RAD / TILT_DEADBAND_RAD near this).')


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
    rclpy.init()
    node = JitterMeter(secs)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.report()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
