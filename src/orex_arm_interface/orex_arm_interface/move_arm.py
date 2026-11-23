"""
Arm interface node — bridges the real Hiwonder HX-30HM gun-arm servos and ROS.

Servo ID : Name             : URDF joint
    1     : Trigger          : (no joint — not mirrored)
    2     : Gun pan          : gun_pan
    3     : Shoulder lift    : gun_shoulder_lift
    4     : Shoulder rotate  : gun_shoulder_rotate

Priorities:
  p1 (DONE): mirror the real servo positions into RViz 1:1 — read present
             position from each servo and publish /joint_states.
  p2 (SCAFFOLDED): software brakes / bounds. Fill min_steps/max_steps per joint,
             then clamp_rad() enforces them (called by p4 before commanding). A
             live monitor already warns when a joint is read past its bounds.
  p3: listen to /joint_velocities and convert rad/s -> servo units.
  p4: integrate velocity over dt to drive the servos (mirrors mock hardware node).

--- P2 bounds capture -----------------------------------------------------------
For each joint, drive/backdrive it to each mechanical stop and read the servo's
present position (same as zero_steps: `python3 -m orex_arm_interface.utils.hiwonder_bus`
or SerialServo.read_pos). Put the two readings in min_steps / max_steps below.
Order doesn't matter — joint_bounds_rad() sorts them after applying direction.

--- P1 calibration (steps <-> radians) ------------------------------------------
Present position is a SIGNED servo value in steps (4096 steps / 360deg). To place
the arm correctly in RViz we map steps -> radians about each URDF joint axis:

    rad = direction * (steps - zero_steps) * (2*pi / 4096) / gear_ratio

For each joint fill in JOINT_CALIB below:
  * zero_steps : the servo's present-position reading when the joint is at the
                 URDF zero pose. Park the arm at the URDF zero, run
                 `python3 -m orex_arm_interface.utils.hiwonder_bus` (or read_pos)
                 and copy the value here.
  * direction  : +1 or -1 — whether increasing servo steps moves the joint in the
                 +axis direction of the URDF joint (all three arm axes are 0 0 -1).
                 If RViz moves the wrong way when you backdrive, flip the sign.
  * gear_ratio : servo revolutions per joint revolution (mechanical reduction /
                 belt / lever). The encoder reads the SERVO shaft, not the joint,
                 so we divide by this. 1.0 = direct drive. If RViz sweeps a bigger
                 angle than the real joint, gear_ratio > 1 (raise it until they
                 match). Measure it: rotate the real joint through a known angle A
                 (or stop-to-stop), note the servo step change dS, then
                     gear_ratio = (dS / 4096 * 360) / A_degrees
                 i.e. (degrees the servo turned) / (degrees the joint turned).
"""

import math

import rclpy
from rclpy.node import Node
import serial

# Publishes the servo positions to RViz (via robot_state_publisher)
from sensor_msgs.msg import JointState

from orex_arm_interface.utils.hiwonder_bus import SerialServo


STEPS_PER_REV = 4096
RAD_PER_STEP = 2.0 * math.pi / STEPS_PER_REV

# Servo ID -> URDF joint name + calibration. Fill zero_steps / direction from the
# backdrive limit sweeps (see module docstring).
# min_steps / max_steps: the servo present-position readings at each mechanical
# stop (same units/frame as zero_steps — capture them the same way, see below).
# Leave as None until measured; the brake is a no-op for a joint with no bounds.
JOINT_CALIB = {
    4: {"name": "gun_shoulder_rotate", "zero_steps": 5464, "direction": +1, "gear_ratio": 4.0,
        "min_steps": None, "max_steps": None},
    3: {"name": "gun_shoulder_lift",   "zero_steps": 1800, "direction": +1, "gear_ratio": 4.0,
        "min_steps": None, "max_steps": None},
    2: {"name": "gun_pan",             "zero_steps": 254,  "direction": +1, "gear_ratio": 4.0,
        # left stop = -469; right stop read -485 but had wrapped one servo rev,
        # so use the continuous value -485 + 4096 = 3611 (mid 1582 ~= midpoint).
        "min_steps": -469, "max_steps": 3611},
}
# Read/publish order (shoulder rotate -> lift -> pan). Excludes the trigger (ID 1).
SERVO_IDS = [4, 3, 2]


class ArmInterfaceNode(Node):
    def __init__(self):
        super().__init__('arm_interface_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        port = self.get_parameter('port').get_parameter_value().string_value

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        try:
            self.bus = SerialServo(port)
        except serial.SerialException as e:
            self.get_logger().fatal(f"Could not open servo bus on {port}: {e}")
            raise

        self.joint_names = [JOINT_CALIB[sid]["name"] for sid in SERVO_IDS]
        # last known radian value per joint — held if a read momentarily times out
        self.last_rad = {sid: 0.0 for sid in SERVO_IDS}
        # unwrap state: present position jumps a full rev (4096) at the encoder seam;
        # accumulate turns so the published angle stays continuous across it.
        self.prev_raw = {sid: None for sid in SERVO_IDS}
        self.turns = {sid: 0 for sid in SERVO_IDS}

        # Anchor the turn count so bounds are valid regardless of boot pose.
        self._home()

        self.dt = 0.02  # 50 Hz
        self.timer = self.create_timer(self.dt, self.mirror_loop)

        self.get_logger().info(
            f"Arm Interface (P1 mirror) up on {port} — mirroring {self.joint_names}")

    def _home(self):
        """
        Startup homing (no motion): anchor each joint's turn count so its continuous
        position lands inside its known [min_steps, max_steps] range. This makes the
        bounds valid no matter what pose the node booted in — even for a joint whose
        travel crosses the encoder seam (e.g. gun_pan). Joints without bounds stay at
        turn 0. Only works when a joint's range is under one servo rev (< 4096 steps);
        a wider range can't be disambiguated from a single reading and would need a
        drive-to-stop homing routine.
        """
        feedback = {}
        for _ in range(10):   # retry a few times in case of transient read misses
            feedback = self.bus.sync_read_cur_pos_ex(SERVO_IDS)
            if all(sid in feedback for sid in SERVO_IDS):
                break

        for sid in SERVO_IDS:
            if sid not in feedback:
                self.get_logger().warn(
                    f"home: no reading for servo {sid}; turn anchor deferred to first move")
                continue
            raw = feedback[sid]["pos"]
            self.prev_raw[sid] = raw
            c = JOINT_CALIB[sid]
            if c["min_steps"] is None or c["max_steps"] is None:
                self.turns[sid] = 0
                continue
            if abs(c["max_steps"] - c["min_steps"]) >= STEPS_PER_REV:
                self.get_logger().warn(
                    f"home {c['name']}: range >= 1 servo rev — turn anchor may be off "
                    f"by a rev near the extremes; drive-to-stop homing needed")
            center = (c["min_steps"] + c["max_steps"]) / 2.0
            self.turns[sid] = round((center - raw) / STEPS_PER_REV)
            anchored = raw + self.turns[sid] * STEPS_PER_REV
            self.get_logger().info(
                f"home {c['name']}: raw {raw} -> {anchored} steps "
                f"({math.degrees(self.steps_to_rad(sid, anchored)):.1f} deg)")

    def unwrap(self, servo_id, raw):
        """
        Turn the servo's single-turn present position into a continuous value.
        A move can't cross half a rev between 20 ms samples, so any jump larger
        than that is an encoder-seam wrap (+/-4096) — absorb it into a turn count.
        """
        prev = self.prev_raw[servo_id]
        self.prev_raw[servo_id] = raw
        if prev is not None:
            delta = raw - prev
            if delta > STEPS_PER_REV // 2:
                self.turns[servo_id] -= 1
            elif delta < -STEPS_PER_REV // 2:
                self.turns[servo_id] += 1
        return raw + self.turns[servo_id] * STEPS_PER_REV

    def steps_to_rad(self, servo_id, steps):
        c = JOINT_CALIB[servo_id]
        # gear_ratio = servo revs per joint rev (reduction). The encoder reads the
        # servo shaft, so divide by the ratio to get the actual URDF joint angle.
        return c["direction"] * (steps - c["zero_steps"]) * RAD_PER_STEP / c["gear_ratio"]
 
    # --- p2: software brakes / bounds --------------------------------------
    def joint_bounds_rad(self, servo_id):
        """(lo, hi) joint bounds in radians, or None if this joint has no limits set."""
        c = JOINT_CALIB[servo_id]
        if c["min_steps"] is None or c["max_steps"] is None:
            return None
        a = self.steps_to_rad(servo_id, c["min_steps"])
        b = self.steps_to_rad(servo_id, c["max_steps"])
        return (min(a, b), max(a, b))   # direction may invert which stop is 'min'

    def clamp_rad(self, servo_id, target_rad):
        """Software brake: clamp a commanded joint angle to its bounds. p4 calls this
        before converting a target to servo units, so an integrated velocity can never
        push a joint past a mechanical stop."""
        bounds = self.joint_bounds_rad(servo_id)
        if bounds is None:
            return target_rad
        return max(bounds[0], min(bounds[1], target_rad))

    def mirror_loop(self):
        # One bus round-trip reads present position for all arm joints at once.
        feedback = self.bus.sync_read_cur_pos_ex(SERVO_IDS)

        if not feedback:
            # No servo answered this cycle — keep last pose, warn (throttled).
            self.get_logger().warn("No servo feedback this cycle", throttle_duration_sec=2.0)

        positions = []
        for sid in SERVO_IDS:
            if sid in feedback:
                steps = self.unwrap(sid, feedback[sid]["pos"])
                self.last_rad[sid] = self.steps_to_rad(sid, steps)
                self._check_bounds(sid, self.last_rad[sid])
            positions.append(self.last_rad[sid])

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = positions
        self.joint_state_pub.publish(msg)

    def _check_bounds(self, servo_id, rad):
        """Warn (throttled) when a joint is read past its defined bounds — handy for
        verifying the min/max numbers while backdriving."""
        bounds = self.joint_bounds_rad(servo_id)
        if bounds is None:
            return
        if rad < bounds[0] or rad > bounds[1]:
            self.get_logger().warn(
                f"{JOINT_CALIB[servo_id]['name']} out of bounds: "
                f"{math.degrees(rad):.1f}deg not in "
                f"[{math.degrees(bounds[0]):.1f}, {math.degrees(bounds[1]):.1f}]",
                throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)

    node = ArmInterfaceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.bus.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
