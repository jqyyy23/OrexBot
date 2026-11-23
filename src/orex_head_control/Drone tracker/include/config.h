/**
 * @file config.h
 * @brief User-editable configuration for the OrexBot head tracking firmware.
 *
 * Control model: RATE / VISUAL SERVOING on the vision node's centering error.
 *   The firmware subscribes to /head/angular_error (geometry_msgs/Vector3, radians):
 *       msg.x = yaw   error (+ = target is to the RIGHT of boresight)
 *       msg.y = pitch error (+ = target is BELOW boresight, image y-down)
 *   and to /head/target_locked (std_msgs/Bool).
 *
 *   Pan  (speed mode):  pan_speed  = PAN_KP*yaw + PAN_KD*d(yaw)/dt   -> nulls the error.
 *   Tilt (position mode): tilt_target += TILT_RATE * pitch * dt      -> nulls the error.
 *
 * Transport is Serial (USB CDC). On the host run:
 *   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v4
 *
 * BENCH-VERIFY the two direction knobs (PAN_DIR / TILT_DIR) before trusting motion:
 *   nudge the target off-center and confirm the head moves TO re-center it, not away.
 */
#pragma once

// ─────────────────────────────────────────────────────────────────
//  Milestone / bring-up flags  (flash once per milestone)
// ─────────────────────────────────────────────────────────────────
//  M1 servo bus check : BENCH_TEST 1                      (no ROS, sweeps servos)
//  M2 micro-ROS link  : BENCH_TEST 0, ENABLE_PAN 0, ENABLE_TILT 0  (prints, no motion)
//  M3 pan only        : BENCH_TEST 0, ENABLE_PAN 1, ENABLE_TILT 0
//  M4 tilt only       : BENCH_TEST 0, ENABLE_PAN 0, ENABLE_TILT 1
//  M5 both            : BENCH_TEST 0, ENABLE_PAN 1, ENABLE_TILT 1

/** When 1: ignore ROS entirely and run a bench self-test sweep of the servos. */
#define BENCH_TEST   0

/** Enable the pan axis control law (0 = pan motor forced to stop). */
#define ENABLE_PAN   1

/** Enable the tilt axis control law (0 = tilt held at its startup target). */
#define ENABLE_TILT  1

/**
 * Tilt re-center (only meaningful when BENCH_TEST 1). Run this ONCE.
 * The tilt servo's position reference floats across power cycles (multi-turn reset),
 * so absolute count targets drift. This makes the CURRENT physical position become
 * 2048 (mid-scale, away from the encoder wrap) via cali_pos(). Procedure: hold the
 * head at a mid/level reference, boot, wait for the countdown; then the routine drops
 * to torque-off read-out so you can immediately record the new stable up/down limits.
 */
#define BENCH_TILT_RECENTER  0

/**
 * Tilt limit calibration (only meaningful when BENCH_TEST 1).
 * When 1: the tilt motor torque is DISABLED and the firmware prints read_pos(TILT)
 * continuously. Hand-move the head to the full-up and full-down mechanical stops and
 * record the printed values — those go into TILT_POS_UP / TILT_POS_DOWN below.
 * (Run BENCH_TILT_RECENTER first so these values are stable across reboots.)
 */
#define BENCH_TILT_CALIB  0

// ─────────────────────────────────────────────────────────────────
//  Pan axis  (servo ID 20 – closed-loop speed mode, continuous rotation)
// ─────────────────────────────────────────────────────────────────

/** Proportional gain: servo speed units per radian of yaw error. Tune up until snappy. */
#define PAN_KP            2500.0f

/** Derivative gain: dampens overshoot/oscillation. Increase if the head hunts. */
#define PAN_KD            100.0f

/** Max pan speed command (servo range 0–3400). Caps slew rate for safety. */
#define PAN_MAX_SPEED     1500

/** Stiction breakaway floor: once the error is PAST the deadband, command at least this
 *  |speed| in the correct direction. Without it, small errors produce a speed (KP*err) below
 *  the servo's friction threshold, so the head is commanded-but-not-moving ("sees it, doesn't
 *  react"). Raise until the head reliably breaks free on small offsets; lower if it hunts. */
#define PAN_MIN_SPEED     100

/** Deadband (radians): inside this the pan motor is stopped. This must be >= how far one
 *  MIN_SPEED lunge overshoots, or the head limit-cycles (overshoot -> correct -> overshoot).
 *  ~0.030 rad ≈ 1.7 deg: it settles "close enough" instead of hunting dead-center. */
#define PAN_DEADBAND_RAD  0.030f

/** Direction sign (+1 or -1). Flip if the head pans AWAY from the target. */
#define PAN_DIR           (+1.0f)

// ─────────────────────────────────────────────────────────────────
//  Tilt axis  (servo ID 18 – position mode)
// ─────────────────────────────────────────────────────────────────
//  Tilt is controlled directly in the servo's own position counts (read_pos /
//  write_pos space), so the mechanical gear ratio and angle conversions drop out.
//
//  The REAL travel limits live ON THE SERVO (written into its EEPROM). The firmware
//  does NOT hardcode them: instead update_tilt() polls the servo's actual position and
//  refuses to let the integrator target lead it by more than TILT_MAX_LEAD_COUNTS
//  (anti-windup). When the head hits a stop the actual position stops advancing, so the
//  target is pinned right there and recovers instantly on reversal — the firmware tracks
//  until the servo itself can't go any higher/lower, whatever limits are written to it.
//
//  TILT_POS_UP / TILT_POS_DOWN below are now only an OUTER SAFETY NET: keep them well
//  OUTSIDE the real stops so they never bind before the servo does. They only matter if
//  position reads fail (servo unplugged), as a runaway backstop.

/** Outer safety-net UP bound (counts). Keep comfortably ABOVE the real up stop. */
#define TILT_POS_UP      1500

/** Outer safety-net DOWN bound (counts). Keep comfortably BELOW the real down stop. */
#define TILT_POS_DOWN    2700

/** Boot fallback position (counts), only used if reading the servo at startup fails.
 *  Normally the head just holds wherever it powers on and tracks from there. */
#define TILT_POS_START   2110

/**
 * Anti-windup lead (counts). The integrated tilt target may lead the servo's ACTUAL
 * read position by at most this much. Must exceed the counts the servo can travel
 * between reads at full rate (~TILT_RATE_CPS * TILT_READ_EVERY_MS/1000) plus margin, but
 * small enough that recovery off a stop feels snappy. ~30 counts/read normal → 150 is safe.
 */
#define TILT_MAX_LEAD_COUNTS  150

/** How often to poll the servo's actual position for anti-windup (ms). Throttled so the
 *  blocking bus read never runs on every control loop. */
#define TILT_READ_EVERY_MS    100

/** Slew rate: position counts per second, per radian of pitch error. */
#define TILT_RATE_CPS    700.0f

/** Deadband (radians): inside this the tilt target is held steady.
 *  Set to ~3 sigma of measured tilt jitter (0.20 deg 1-sigma) = ~0.6 deg. */
#define TILT_DEADBAND_RAD 0.012f

/**
 * Direction sign (+1 or -1). Chosen so pitch>0 (target BELOW boresight) drives the
 * head DOWN. Because the target is clamped to the real limits, a wrong sign merely
 * parks the head at a stop (no binding) — flip it in M4 if it tracks the wrong way.
 */
#define TILT_DIR          (+1.0f)

// ─────────────────────────────────────────────────────────────────
//  Common
// ─────────────────────────────────────────────────────────────────

/**
 * Command watchdog (ms). If no /head/angular_error arrives within this window
 * (target lost -> vision node stops publishing), the pan motor is stopped and the
 * tilt is held. Should comfortably exceed one camera frame period.
 */
#define CMD_TIMEOUT_MS    250

/** Control loop period (ms). ~5 ms → ~200 Hz; no blocking bus reads in the loop. */
#define CONTROL_PERIOD_MS 5

// ─────────────────────────────────────────────────────────────────
//  Joint-state feedback  →  /head/joint_states  →  robot_state_publisher → TF
// ─────────────────────────────────────────────────────────────────
//  The firmware reports the two head joints (head_pan, head_tilt) in RADIANS so RViz /
//  TF know where the camera points. Conversion from a servo encoder count:
//        angle_rad = DIR * (count - ZERO_COUNT) / COUNTS_PER_RAD
//
//  COUNTS_PER_RAD: a Feetech STS servo is 4096 counts / revolution = 4096/(2*pi) ≈ 651.9
//  counts/rad at the SERVO shaft. Multiply by the servo→joint gear ratio to get counts/rad
//  at the JOINT (e.g. the tilt's 2:1 gear → ~1303.8).
//
//  CALIBRATION (do once, per axis):
//   1) ZERO_COUNT: point the head straight FORWARD (pan) / LEVEL (tilt), read the encoder
//      (the tilt value prints; for pan add a temporary read_pos print), put it here.
//   2) COUNTS_PER_RAD: rotate the joint by a known angle (e.g. 90° = pi/2 rad by protractor),
//      read the count delta, then COUNTS_PER_RAD = delta / angle_rad.
//   3) DIR: flip the sign if RViz shows the head rotating the WRONG way.

#define PAN_JOINT_ZERO_COUNT   2048.0f   // encoder count with the head facing FORWARD  (CALIBRATE)
#define PAN_COUNTS_PER_RAD     651.9f    // 4096/(2*pi) * pan gear ratio                 (CALIBRATE)
#define PAN_JOINT_DIR          (+1.0f)   // flip if pan appears reversed in RViz

#define TILT_JOINT_ZERO_COUNT  2048.0f   // encoder count with the head LEVEL            (CALIBRATE)
#define TILT_COUNTS_PER_RAD    1303.8f   // 4096/(2*pi) * 2:1 tilt gear                   (CALIBRATE)
#define TILT_JOINT_DIR         (+1.0f)   // flip if tilt appears reversed in RViz

/** /head/joint_states publish period (ms). ~50 ms → ~20 Hz, plenty for TF/RViz. */
#define JOINT_STATE_PERIOD_MS  50
