/**
 * @file main.cpp
 * @brief OrexBot Head Tracking – ESP32-S3 micro-ROS Firmware (rate / visual servoing)
 *
 * Edit include/config.h for gains, limits, and the milestone bring-up flags.
 *
 * Subscribes to:
 *   /head/angular_error (geometry_msgs/Vector3, radians)
 *       msg.x = yaw   error (+ = target RIGHT of boresight)
 *       msg.y = pitch error (+ = target BELOW boresight, image y-down)
 *   /head/target_locked (std_msgs/Bool)  – true while tracking/coasting.
 *
 * Control law (see config.h):
 *   Pan  (ID 20, speed mode)    : speed = PAN_DIR*(PAN_KP*yaw + PAN_KD*dyaw/dt), continuous.
 *   Tilt (ID 18, position mode) : tilt_target += TILT_DIR*TILT_RATE*pitch*dt, 2:1 gear.
 *
 * The head only moves while the target is LOCKED and a fresh error message has arrived
 * within CMD_TIMEOUT_MS; otherwise the pan motor stops and the tilt holds its position.
 *
 * Transport: micro-ROS over Serial (USB CDC). On the host:
 *   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v4
 */

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>              // time sync (rmw_uros_epoch_*)
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/joint_state.h>            // /head/joint_states feedback
#include <rosidl_runtime_c/string_functions.h>      // rosidl_runtime_c__String__assign

#include "HX_30HM.h"
#include "config.h"

// ─────────────────────────────────────────────
//  Hardware pinout  (PCB-level, not user-facing)
// ─────────────────────────────────────────────
#define SERVO_RX_PIN  5
#define SERVO_TX_PIN  4
#define STATUS_LED    2

// ─────────────────────────────────────────────
//  Servo IDs
// ─────────────────────────────────────────────
#define PAN_ID   20   // Closed-loop speed mode (Mode 1)
#define TILT_ID  18   // Position mode           (Mode 0)

// ─────────────────────────────────────────────
//  Globals
// ─────────────────────────────────────────────
SerialServo servo(Serial1, 1000000, SERVO_TX_PIN, SERVO_RX_PIN);

rcl_subscription_t          angle_sub;
rcl_subscription_t          locked_sub;
rcl_publisher_t             debug_pub;     // /head/tilt_debug (x=target, y=actual, z=pitch_err)
rcl_publisher_t             joint_pub;     // /head/joint_states (head_pan, head_tilt in rad)
geometry_msgs__msg__Vector3 g_angle_msg;   // /head/angular_error
geometry_msgs__msg__Vector3 g_debug_msg;   // /head/tilt_debug
std_msgs__msg__Bool         g_locked_msg;  // /head/target_locked
sensor_msgs__msg__JointState g_js_msg;     // /head/joint_states payload
// Static backing storage for the JointState arrays (2 head joints). Allocated once so no
// heap churn on the MCU; the names match the URDF joints exactly.
static rosidl_runtime_c__String g_js_names[2];
static double                    g_js_positions[2];
rclc_executor_t             executor;
rclc_support_t              support;
rcl_allocator_t             allocator;
rcl_node_t                  node;

// Latest command state (updated by ROS callbacks)
volatile float        g_yaw_err       = 0.0f;   // radians
volatile float        g_yaw_rate      = 0.0f;   // rad/s, derivative at measurement rate
volatile float        g_pitch_err     = 0.0f;   // radians
volatile bool         g_locked        = false;
volatile unsigned long g_last_cmd_ms  = 0;

// Tilt integrator (servo position counts)
float          g_tilt_target_pos  = (float)TILT_POS_START;
float          g_tilt_actual_pos  = (float)TILT_POS_START;  // last valid read_pos(TILT)
bool           g_tilt_actual_valid = false;                 // have we read a plausible pos yet?
unsigned long  g_tilt_read_last_ms = 0;                     // last actual-position poll (anti-windup)
unsigned long  loop_last_us       = 0;

// Plausibility band for accepting a tilt read_pos value (reject bus-error/garbage reads).
// Wider than any real travel so it never rejects a legitimate position.
#define TILT_READ_PLAUSIBLE_LO  800
#define TILT_READ_PLAUSIBLE_HI  3400

// ─────────────────────────────────────────────
//  micro-ROS state machine
// ─────────────────────────────────────────────
typedef enum { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } ros_state_t;
static ros_state_t ros_state = WAITING_AGENT;

static void error_loop() {
  while (true) { digitalWrite(STATUS_LED, !digitalRead(STATUS_LED)); delay(50); }
}

// ─────────────────────────────────────────────
//  Servo helpers
// ─────────────────────────────────────────────
static void stop_pan()  { servo.write_speed(PAN_ID, 0); }

/** Lower/upper tilt clamp in servo counts (independent of which stop is up vs down). */
static inline float tilt_lo() { return (float)min(TILT_POS_UP, TILT_POS_DOWN); }
static inline float tilt_hi() { return (float)max(TILT_POS_UP, TILT_POS_DOWN); }

/** Command the tilt servo to the current integrator target (position counts, clamped). */
static void write_tilt_target() {
  g_tilt_target_pos = constrain(g_tilt_target_pos, tilt_lo(), tilt_hi());
  servo.write_pos(TILT_ID, (int16_t)g_tilt_target_pos);
}

static void stop_all_servos() { stop_pan(); write_tilt_target(); }

/** Pan rate law: speed proportional to yaw error, with derivative damping + deadband.
 *  The derivative (g_yaw_rate) is computed at measurement rate in angle_callback, so it
 *  is not spiked by the faster control loop. */
static void update_pan_rate() {
  float err = g_yaw_err;
  if (fabsf(err) < PAN_DEADBAND_RAD) { stop_pan(); return; }

  float output = PAN_DIR * (PAN_KP * err + PAN_KD * g_yaw_rate);
  // Stiction breakaway: past the deadband, ensure the command is big enough to actually
  // move the head (small errors otherwise sit below the servo's friction threshold).
  if (fabsf(output) < (float)PAN_MIN_SPEED) output = copysignf((float)PAN_MIN_SPEED, output);
  int16_t speed_cmd = (int16_t)constrain(output, -(float)PAN_MAX_SPEED, (float)PAN_MAX_SPEED);
  servo.write_speed(PAN_ID, speed_cmd);
}

/** Poll the servo's actual tilt position (throttled). Updates g_tilt_actual_pos only on
 *  a plausible read, so a bus error can't snap the anti-windup band to garbage. */
static void poll_tilt_actual() {
  unsigned long now = millis();
  if (now - g_tilt_read_last_ms < TILT_READ_EVERY_MS) return;
  g_tilt_read_last_ms = now;

  int16_t pos = 0;
  servo.read_pos(TILT_ID, &pos);
  if (pos > TILT_READ_PLAUSIBLE_LO && pos < TILT_READ_PLAUSIBLE_HI) {
    g_tilt_actual_pos   = (float)pos;
    g_tilt_actual_valid = true;
  }
}

/** Tilt rate law: integrate pitch error into the position target, with anti-windup that
 *  defers to the servo's OWN travel limits.
 *
 *  The target may never lead the servo's actual position by more than TILT_MAX_LEAD_COUNTS.
 *  When the head reaches a stop (mechanical or the limit written into the servo) the actual
 *  position stops advancing, so the target is pinned right at the stop instead of winding up
 *  past it — and it recovers instantly the moment the pitch error reverses. No hardcoded
 *  limit counts required. */
static void update_tilt(float dt) {
  poll_tilt_actual();

  if (fabsf(g_pitch_err) >= TILT_DEADBAND_RAD) {
    g_tilt_target_pos += TILT_DIR * TILT_RATE_CPS * g_pitch_err * dt;
  }

  // Anti-windup: keep the target within +/- lead of where the servo actually is.
  if (g_tilt_actual_valid) {
    g_tilt_target_pos = constrain(g_tilt_target_pos,
                                  g_tilt_actual_pos - (float)TILT_MAX_LEAD_COUNTS,
                                  g_tilt_actual_pos + (float)TILT_MAX_LEAD_COUNTS);
  }

  write_tilt_target();   // outer safety-net clamp [tilt_lo, tilt_hi] + command
}

// ─────────────────────────────────────────────
//  ROS callbacks
// ─────────────────────────────────────────────
static void angle_callback(const void* msgin) {
  const geometry_msgs__msg__Vector3* m = (const geometry_msgs__msg__Vector3*)msgin;
  unsigned long now = millis();
  float new_yaw = (float)m->x;
  // Derivative at the true measurement interval (not the faster control-loop rate).
  if (g_last_cmd_ms != 0) {
    float dtm = (now - g_last_cmd_ms) * 1e-3f;
    if (dtm > 1e-4f) g_yaw_rate = (new_yaw - g_yaw_err) / dtm;
  }
  g_yaw_err      = new_yaw;
  g_pitch_err    = (float)m->y;
  g_last_cmd_ms  = now;
}

static void locked_callback(const void* msgin) {
  const std_msgs__msg__Bool* m = (const std_msgs__msg__Bool*)msgin;
  g_locked = m->data;
}

// ─────────────────────────────────────────────
//  Joint-state feedback (/head/joint_states → robot_state_publisher → TF)
// ─────────────────────────────────────────────

/** One-time wiring of the JointState message to its static backing arrays. Names MUST
 *  match the URDF joint names (head_pan, head_tilt) so robot_state_publisher resolves them. */
static void init_joint_state_msg() {
  sensor_msgs__msg__JointState__init(&g_js_msg);
  rosidl_runtime_c__String__assign(&g_js_msg.header.frame_id, "");
  rosidl_runtime_c__String__assign(&g_js_names[0], "head_pan");
  rosidl_runtime_c__String__assign(&g_js_names[1], "head_tilt");
  g_js_msg.name.data = g_js_names;         g_js_msg.name.size = 2;     g_js_msg.name.capacity = 2;
  g_js_msg.position.data = g_js_positions; g_js_msg.position.size = 2; g_js_msg.position.capacity = 2;
  // velocity / effort left empty (size 0) — robot_state_publisher only needs position.
}

/** Convert a servo encoder count to the JOINT angle in radians. */
static inline float counts_to_rad(int16_t count, float zero, float per_rad, float dir) {
  return dir * ((float)count - zero) / per_rad;
}

/** Read both head encoders, convert to radians, timestamp, and publish /head/joint_states. */
static void publish_joint_states() {
  int16_t pan_c = 0, tilt_c = 0;
  servo.read_pos(PAN_ID, &pan_c);
  servo.read_pos(TILT_ID, &tilt_c);
  g_js_positions[0] = (double)counts_to_rad(pan_c,  PAN_JOINT_ZERO_COUNT,  PAN_COUNTS_PER_RAD,  PAN_JOINT_DIR);
  g_js_positions[1] = (double)counts_to_rad(tilt_c, TILT_JOINT_ZERO_COUNT, TILT_COUNTS_PER_RAD, TILT_JOINT_DIR);

  // Stamp with agent-synced ROS time so downstream TF lookups line up with camera frames.
  if (rmw_uros_epoch_synchronized()) {
    int64_t t_ns = rmw_uros_epoch_nanos();
    g_js_msg.header.stamp.sec = (int32_t)(t_ns / 1000000000LL);
    g_js_msg.header.stamp.nanosec = (uint32_t)(t_ns % 1000000000LL);
  }
  (void) rcl_publish(&joint_pub, &g_js_msg, NULL);
}

// ─────────────────────────────────────────────
//  micro-ROS lifecycle
// ─────────────────────────────────────────────
static bool create_entities() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "head_controller", "", &support) != RCL_RET_OK) return false;

  if (rclc_subscription_init_best_effort(
        &angle_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "/head/angular_error") != RCL_RET_OK) return false;

  if (rclc_subscription_init_best_effort(
        &locked_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/head/target_locked") != RCL_RET_OK) return false;

  // Debug telemetry publisher — echo with:  ros2 topic echo /head/tilt_debug
  //   x = tilt target (counts), y = tilt actual (counts), z = pitch error (rad)
  if (rclc_publisher_init_best_effort(
        &debug_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "/head/tilt_debug") != RCL_RET_OK) return false;

  // Head joint-state feedback -> robot_state_publisher builds the camera TF from these.
  if (rclc_publisher_init_default(
        &joint_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/head/joint_states") != RCL_RET_OK) return false;
  init_joint_state_msg();

  if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &angle_sub, &g_angle_msg,
        &angle_callback, ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &locked_sub, &g_locked_msg,
        &locked_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

  return true;
}

static void destroy_entities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  rcl_subscription_fini(&angle_sub, &node);
  rcl_subscription_fini(&locked_sub, &node);
  rcl_publisher_fini(&debug_pub, &node);
  rcl_publisher_fini(&joint_pub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ─────────────────────────────────────────────
//  Control step (M2–M5): applies the enabled axes' laws with the watchdog gate.
// ─────────────────────────────────────────────
static void control_step() {
  unsigned long now_us = micros();
  float dt = (loop_last_us > 0) ? (float)(now_us - loop_last_us) * 1e-6f
                                : (float)CONTROL_PERIOD_MS * 1e-3f;
  loop_last_us = now_us;

  // Active only while locked AND a fresh error message is arriving.
  bool active = g_locked && (millis() - g_last_cmd_ms < CMD_TIMEOUT_MS);

#if ENABLE_PAN
  if (active) update_pan_rate();
  else        stop_pan();
#else
  stop_pan();
#endif

#if ENABLE_TILT
  if (active) update_tilt(dt);
  else        write_tilt_target();   // hold position
#endif

  // Throttled telemetry over ROS (NOT Serial — the USB CDC is the micro-ROS transport,
  // so prints there are invisible/garbage while the agent runs). Echo with:
  //   ros2 topic echo /head/tilt_debug
  static unsigned long last_pub = 0;
  if (millis() - last_pub > 100) {
    last_pub = millis();
    g_debug_msg.x = g_tilt_target_pos;   // tilt target (counts)
    g_debug_msg.y = g_tilt_actual_pos;   // tilt actual (counts)
    g_debug_msg.z = g_pitch_err;         // pitch error (rad)
    (void) rcl_publish(&debug_pub, &g_debug_msg, NULL);
  }
}

// ─────────────────────────────────────────────
//  M1 bench self-test (no ROS): verify wiring, modes, and DIRECTION signs.
// ─────────────────────────────────────────────
#if BENCH_TEST
static void bench_loop() {
  int16_t p = 0;

#if BENCH_TILT_RECENTER
  // One-shot: make the current physical position the servo's mid-scale (2048), then
  // drop to torque-off read-out so the new stable up/down limits can be recorded.
  static bool recentered = false;
  if (!recentered) {
    Serial.println("[RECENTER] Hold the head at a MID / LEVEL reference position...");
    for (int s = 8; s > 0; s--) { Serial.printf("  recentering in %d...\n", s); delay(1000); }
    servo.cali_pos(TILT_ID);           // current position -> 2048
    delay(200);
    servo.disable_torque(TILT_ID);     // limp again for stop-finding
    recentered = true;
    servo.read_pos(TILT_ID, &p);
    Serial.printf("[RECENTER] done – this position now reads %d (expect ~2048).\n", p);
    Serial.println("[RECENTER] Now hand-move to the UP and DOWN stops and note the values.");
  }
  servo.read_pos(TILT_ID, &p);
  Serial.printf("[CALIB] tilt read_pos=%d\n", p);
  delay(200);
  return;
#endif

#if BENCH_TILT_CALIB
  // Tilt torque is OFF (see setup). Hand-move the head to each mechanical stop and
  // record the printed read_pos value into TILT_POS_UP / TILT_POS_DOWN in config.h.
  servo.read_pos(TILT_ID, &p);
  Serial.printf("[CALIB] tilt read_pos=%d  (move head to UP stop, then DOWN stop, note values)\n", p);
  delay(200);
  return;
#else
  Serial.println("[BENCH] Pan +400 (note which way it turns)...");
  servo.write_speed(PAN_ID, 400);
  for (int i = 0; i < 15; i++) { servo.read_pos(PAN_ID, &p); Serial.printf("  pan pos=%d\n", p); delay(100); }
  stop_pan(); delay(600);

  Serial.println("[BENCH] Pan -400 (should turn the OTHER way)...");
  servo.write_speed(PAN_ID, -400);
  for (int i = 0; i < 15; i++) { servo.read_pos(PAN_ID, &p); Serial.printf("  pan pos=%d\n", p); delay(100); }
  stop_pan(); delay(600);

  // Tilt sweep between the calibrated stops (safe: clamped, no binding).
  Serial.println("[BENCH] Tilt sweep UP -> DOWN -> START (position counts)...");
  int seq[3] = {TILT_POS_UP, TILT_POS_DOWN, TILT_POS_START};
  for (int i = 0; i < 3; i++) {
    g_tilt_target_pos = (float)seq[i];
    write_tilt_target();
    servo.read_pos(TILT_ID, &p);
    Serial.printf("  tilt cmd=%d read_pos=%d\n", seq[i], p);
    delay(1200);
  }
  delay(1000);
#endif
}
#endif

// ─────────────────────────────────────────────
//  setup()
// ─────────────────────────────────────────────
void setup() {
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  Serial.begin(115200);
  delay(500);

  // Servo bus (HX_30HM @ 1 Mbps on Serial1). Constructor already begins Serial1.
  delay(200);
  servo.select_mode(PAN_ID,  1);  // closed-loop speed
  servo.select_mode(TILT_ID, 0);  // position
  servo.enable_torque(PAN_ID);

#if BENCH_TEST && (BENCH_TILT_CALIB || BENCH_TILT_RECENTER)
  // Calibration / re-center: leave tilt limp so it can be hand-positioned.
  servo.disable_torque(TILT_ID);
  stop_pan();
  Serial.println("[HeadCtrl] tilt torque OFF for calibration/re-center.");
#else
  servo.enable_torque(TILT_ID);
  // Seed the tilt integrator with the servo's ACTUAL current position so the head holds
  // where it powers on and tracks from there — no jerk to a (possibly drifted) absolute
  // START count. Falls back to TILT_POS_START only if the read is implausible.
  {
    int16_t pos = 0;
    servo.read_pos(TILT_ID, &pos);
    if (pos > TILT_READ_PLAUSIBLE_LO && pos < TILT_READ_PLAUSIBLE_HI) {
      g_tilt_target_pos   = (float)pos;
      g_tilt_actual_pos   = (float)pos;
      g_tilt_actual_valid = true;
      Serial.printf("[HeadCtrl] tilt seeded at actual pos %d (holding on boot).\n", pos);
    } else {
      Serial.printf("[HeadCtrl] tilt read implausible (%d); using START=%d.\n", pos, TILT_POS_START);
    }
  }
  stop_all_servos();
#endif

  loop_last_us = micros();

#if BENCH_TEST
  Serial.println("[HeadCtrl] BENCH_TEST mode – ROS disabled, running servo self-test.");
#else
  // micro-ROS Serial transport uses the USB CDC port.
  set_microros_serial_transports(Serial);
  Serial.println("[HeadCtrl] Serial transport set. Waiting for micro-ROS agent...");
#endif
}

// Run block X at most once every MS milliseconds (official micro-ROS reconnect pattern).
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static unsigned long _ee_last = 0; \
  if (millis() - _ee_last > (unsigned long)(MS)) { X; _ee_last = millis(); } \
} while (0)

// ─────────────────────────────────────────────
//  loop()
// ─────────────────────────────────────────────
void loop() {
#if BENCH_TEST
  bench_loop();
  return;
#else
  switch (ros_state) {
    case WAITING_AGENT:
      if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) {
        ros_state = AGENT_AVAILABLE;
      } else {
        digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
        delay(300);
      }
      break;

    case AGENT_AVAILABLE:
      if (create_entities()) {
        ros_state = AGENT_CONNECTED;
        digitalWrite(STATUS_LED, HIGH);
        // Align the MCU clock with the agent's ROS time so /head/joint_states stamps match
        // the camera frames for TF lookups.
        rmw_uros_sync_session(1000);
        Serial.println("[HeadCtrl] Agent connected – /head/angular_error + /head/target_locked");
      } else {
        destroy_entities();
        ros_state = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:
      // Health-check the agent only every ~2 s with a forgiving timeout. Pinging every
      // control loop with a 10 ms timeout caused constant false "disconnects" and an
      // endless reconnect loop (session torn down before it could receive commands).
      EXECUTE_EVERY_N_MS(2000, {
        ros_state = (RMW_RET_OK == rmw_uros_ping_agent(200, 3)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      });
      if (ros_state == AGENT_CONNECTED) {
        (void) rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        control_step();
        // Publish head joint feedback for TF (throttled, ~20 Hz — not every control loop).
        EXECUTE_EVERY_N_MS(JOINT_STATE_PERIOD_MS, { publish_joint_states(); });
        delay(CONTROL_PERIOD_MS);
      }
      break;

    case AGENT_DISCONNECTED:
      stop_all_servos();
      destroy_entities();
      g_locked = false;
      ros_state = WAITING_AGENT;
      Serial.println("[HeadCtrl] Agent disconnected – reconnecting...");
      break;
  }
#endif
}
