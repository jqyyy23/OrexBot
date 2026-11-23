import os

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from message_filters import ApproximateTimeSynchronizer, Subscriber

import cv2
import numpy as np
from geometry_msgs.msg import PointStamped, Vector3
from sensor_msgs.msg import Image, CameraInfo, Imu
from std_msgs.msg import Bool
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
except Exception as exc:  # pragma: no cover - runtime fallback
    YOLO = None
    YOLO_IMPORT_ERROR = exc
else:
    YOLO_IMPORT_ERROR = None


MODEL_ID = "https://huggingface.co/TomSmail/drone-yolo-v1/resolve/main/best.pt"

# Unified IMU topic produced by rs_launch.py with unite_imu_method:=1 (linear interpolation).
# Carries both gyro (angular_velocity) and accel (linear_acceleration) in the camera IMU
# optical frame, which shares the x-right / y-down / z-forward convention of the color
# optical frame, so we integrate the gyro directly without an extra extrinsic rotation.
IMU_TOPIC = '/camera/camera/imu'

# How long the constant-velocity filter may coast (extrapolate) after the last real
# detection before the target is declared lost. Beyond this the straight-line
# prediction has drifted too far to trust, so we stop publishing and flag "not locked".
COAST_TIMEOUT_S = 0.5

# Detection confidence thresholds. The full-frame search must be conservative to avoid
# false positives, but once we hold a lock we search only a small predicted region, so
# a lower threshold there recovers weak/small detections without flooding false hits.
FULL_CONF = 0.4
ROI_CONF = 0.2

# Track-before-detect search window (Tier 1). When locked, YOLO runs on a crop centered
# on the filter's predicted pixel, expressed as a fraction of the frame dimensions. The
# window is padded and grows with each consecutive missed frame so a maneuvering or
# briefly-lost target is re-captured before the ROI is abandoned for a full-frame search.
ROI_BASE_HALF_FRAC = 0.18
ROI_GROW_FRAC = 0.06
# The ROI center is led by the predicted pixel motion (pos + vel*dt). ROI_VEL_GAIN is the
# fraction of that predicted displacement added to the half-size as uncertainty margin, so
# a fast (less certain) target gets a wider window without discarding the zoom benefit.
ROI_VEL_GAIN = 0.6
# Hard cap on the ROI half-size (fraction of the frame) so it can never fully negate the
# zoom-in that makes small/distant drones detectable; beyond this we just search full-frame.
ROI_MAX_HALF_FRAC = 0.45
# Clamp on the look-ahead used to lead the ROI center, so a long frame gap cannot fling
# the window far off the target.
ROI_LEAD_DT_MAX_S = 0.2

# Validation gate (data association). A detection farther than this (meters, in 3D) from
# the filter's prediction is rejected as a distractor and treated as a miss instead of
# snapping the track to it. The gate widens per consecutive miss because the prediction
# grows less certain while coasting.
GATE_BASE_M = 0.75
GATE_GROW_M = 0.50

# Half-size (pixels) of the square window used for robust median depth sampling.
DEPTH_WIN_HALF = 3

# Consecutive in-gate corrections required before the track is declared LOCKED, so a
# transient false positive cannot claim a lock on its own.
CONFIRM_HITS = 3

# Periodic full-frame rescan (robustness #1): every N processed frames, ignore the ROI
# and search the whole frame, so a target that has left the ROI can be re-found without
# waiting out the full coast timeout.
RESCAN_EVERY_N = 20

# Motion-based lock validation (robustness #2): a confirmed lock whose ego-compensated
# (reference-frame) speed stays below STATIC_SPEED_MPS for STATIC_TIMEOUT_S is treated as
# stationary clutter — dropped, and its location blacklisted for BLACKLIST_S so
# acquisition doesn't immediately re-lock it, giving a moving target a chance instead.
# Tune STATIC_SPEED_MPS between a truly-static object's residual jitter and your drone's
# slowest real motion. STATIC_TIMEOUT_S long enough that brief stillness won't drop a real track.
STATIC_SPEED_MPS = 0.15
STATIC_TIMEOUT_S = 2.5
BLACKLIST_S = 3.0
BLACKLIST_RADIUS_M = 0.4

# Kalman filter smoothing. The output smoothness is set by the ratio (measurement noise) /
# KF_PROCESS_NOISE: higher = trusts the constant-velocity model more over the noisy
# measurement, so the 3D estimate is steadier — at the cost of more lag on fast motion.
KF_PROCESS_NOISE = 0.015

# Anisotropic + range-adaptive measurement noise (variances on the R diagonal).
# The measurement is a deprojected 3D point whose axes have very different reliability:
#   - X/Y come from the box-center PIXEL (reliable) → low, near-constant noise.
#   - Z comes from RealSense DEPTH (flaky on small/distant/rotor targets) → higher noise
#     that GROWS with range, since the D435i's depth RMS error rises roughly ~z^2.
# These are built in the CAMERA optical frame each correction and rotated into the filter's
# ego-stabilized reference frame (so the depth axis stays correct when the head is turned).
KF_MEAS_NOISE_XY = 0.25       # lateral (pixel-derived) variance — trusted
KF_MEAS_NOISE_Z_BASE = 0.5    # depth variance at the reference range
KF_MEAS_Z_REF_M = 2.0         # range (m) at which Z noise == the base value
KF_MEAS_Z_MAX_FACTOR = 8.0    # cap on the range-inflation factor (very far → distrust Z, not infinitely)

# The head-centering angular error is computed from the target PIXEL (depth-free), not the
# depth-scaled 3D point, so depth noise never enters the head command. The pixel is
# EMA-smoothed for smooth head motion: lower alpha = smoother but more lag.
PIXEL_EMA_ALPHA = 0.2

# --- Detector + tracker fusion (robustness #3) ---------------------------------------
# YOLO designates the drone, then a single-object tracker (OpenCV TrackerVit) follows it
# every frame — holding the lock by appearance through YOLO flicker and fast motion — and
# YOLO re-validates/re-anchors it periodically. This supersedes the ROI-crop follow and
# the motion-based lock keeper.
TRACKER_MODEL_PATH = os.path.join(os.path.dirname(__file__), '..', 'models', 'vittrack.onnx')
# Run YOLO to re-anchor/re-validate the tracker every N frames while following.
TRACKER_VALIDATE_EVERY_N = 6
# ViT tracking score below this = the tracker has lost the target.
TRACKER_MIN_SCORE = 0.25
# Consecutive YOLO re-validations with no matching drone before the tracker is dropped.
TRACKER_VALIDATE_MAX_MISSES = 5
# Max center distance (fraction of frame width) for a YOLO box to "match" the tracker box.
TRACKER_MATCH_DIST_FRAC = 0.15
# Box conditioning before the tracker is (re-)initialized. vittrack builds its template and
# per-frame search window from the init box, so a tiny/tight box gives a low-detail template
# and a small search window that a small or fast drone escapes. We pad the YOLO box for
# context and enforce a minimum pixel size so distant drones still get a usable window.
TRACKER_INIT_PAD = 1.25       # expand the YOLO box by this factor (context for the template)
TRACKER_MIN_BOX_PX = 40       # floor on box width/height (px) so small boxes still track

# Interval for the IMU health check that warns if the gyro stream is missing/stalled.
IMU_HEALTH_PERIOD_S = 5.0
# IMU counts as stalled if nothing has arrived for longer than this.
IMU_STALE_S = 2.0

class vision_tracking(Node):
    """ROS2 node that subscribes to RealSense camera topics for drone tracking."""

    def __init__(self):
        super().__init__('vision_tracking')

        self.bridge = CvBridge()
        self.model = None
        self.latest_color = None
        self.latest_depth = None
        self.latest_camera_info = None
        self.last_sync_time = None
        self.last_3d_point = None
        self.last_3d_time_ns = None
        self.last_3d_velocity = None
        self.filter_initialized = False
        self.filter_time_ns = None

        # Ego-motion compensation state (Step 5).
        # R_cur rotates a point from the current camera frame into a fixed reference
        # frame anchored at the first IMU sample: P_ref = R_cur @ P_cam. The Kalman
        # filter runs in this reference frame so head pan/tilt does not show up as
        # target velocity. Outputs are rotated back into the current camera frame.
        self.R_cur = np.eye(3, dtype=np.float64)
        self.last_imu_time_ns = None
        self.imu_ready = False
        # Wall-clock time of the most recent IMU message, for the health check.
        self.last_imu_rx_time = None

        # Coast tracking: sensor-stamp time of the last real (corrected) detection and
        # whether we currently believe we hold a lock on the target.
        self.last_detection_time_ns = None
        self.target_locked = False
        # Consecutive frames with no valid measurement; grows the ROI search window.
        self.consecutive_misses = 0
        # Consecutive in-gate corrections; the track is only LOCKED once this reaches
        # CONFIRM_HITS. Reset to 0 whenever the track is lost.
        self.confirm_hits = 0
        # Processed-frame counter, drives the periodic full-frame rescan (robustness #1).
        self.frame_count = 0
        # Sensor-stamp time the current lock first went below STATIC_SPEED_MPS (robustness #2);
        # None while it is moving. Blacklist = list of (ref_point, expiry_ns) to skip.
        self.static_since_ns = None
        self.blacklist = []

        # Detector+tracker fusion (robustness #3). _vit_ok gates whether TrackerVit can be
        # used; if not, we fall back to YOLO-only acquisition (no per-frame follow).
        self._vit_ok = (
            hasattr(cv2, 'TrackerVit_create') and os.path.isfile(TRACKER_MODEL_PATH)
        )
        self.tracker = None
        self.tracker_ok = False
        self.tracker_validate_misses = 0
        # EMA-smoothed target pixel (u, v) in full-res color coords; drives the depth-free
        # head-centering angular error. Reset to None on target loss.
        self.pixel_filt = None
        if self._vit_ok:
            self.get_logger().info(f'TrackerVit available: {os.path.abspath(TRACKER_MODEL_PATH)}')
        else:
            self.get_logger().warn(
                'TrackerVit unavailable (missing cv2.TrackerVit_create or model file); '
                'falling back to YOLO-only acquisition.'
            )
        # Residual gyro bias (rad/s) subtracted before integration. The hardware
        # calibration tool (rs-imu-callibration.py) writes intrinsics to NVRAM, so
        # this stays near zero; override here if you observe drift.
        self.gyro_bias = np.zeros(3, dtype=np.float64)
        self.kf = cv2.KalmanFilter(6, 3, 0)
        self.kf.transitionMatrix = np.array(
            [[1, 0, 0, 0, 0, 0],
             [0, 1, 0, 0, 0, 0],
             [0, 0, 1, 0, 0, 0],
             [0, 0, 0, 1, 0, 0],
             [0, 0, 0, 0, 1, 0],
             [0, 0, 0, 0, 0, 1]], dtype=np.float32
        )
        self.kf.measurementMatrix = np.array(
            [[1, 0, 0, 0, 0, 0],
             [0, 1, 0, 0, 0, 0],
             [0, 0, 1, 0, 0, 0]], dtype=np.float32
        )
        self.kf.processNoiseCov = np.eye(6, dtype=np.float32) * KF_PROCESS_NOISE
        # Default (isotropic) R; overwritten per-correction by _adaptive_meas_noise().
        self.kf.measurementNoiseCov = np.diag(
            [KF_MEAS_NOISE_XY, KF_MEAS_NOISE_XY, KF_MEAS_NOISE_Z_BASE]
        ).astype(np.float32)
        self.kf.errorCovPost = np.eye(6, dtype=np.float32) * 1.0
        self.kf.statePost = np.zeros((6, 1), dtype=np.float32)

        if YOLO is not None:
            try:
                self.model = YOLO(MODEL_ID)
                self.get_logger().info(f'Loaded YOLO model: {MODEL_ID}')
            except Exception as exc:  # pragma: no cover - runtime fallback
                self.get_logger().warn(f'Failed to load YOLO model: {exc}')
        elif YOLO_IMPORT_ERROR is not None:
            self.get_logger().warn(f'YOLO import failed: {YOLO_IMPORT_ERROR}')

        self.color_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/camera/depth/image_rect_raw')
        self.camera_info_sub = Subscriber(self, CameraInfo, '/camera/camera/color/camera_info')

        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.camera_info_sub],
            queue_size=10,
            slop=0.2,
        )
        self.sync.registerCallback(self.sync_callback)

        # IMU arrives at ~200 Hz, far faster than frames, so it is handled on its own
        # subscription (not time-synchronized) with sensor-data QoS (best effort).
        self.imu_sub = self.create_subscription(
            Imu, IMU_TOPIC, self.imu_callback, qos_profile_sensor_data
        )

        self.tracked_point_pub = self.create_publisher(PointStamped, '/head/target_point', 10)
        self.angular_error_pub = self.create_publisher(Vector3, '/head/angular_error', 10)
        # Filtered target velocity (m/s) in the current camera frame, for the HUD's
        # predicted-motion arrow. Not required by head control.
        self.target_velocity_pub = self.create_publisher(Vector3, '/head/target_velocity', 10)
        # True while detecting or coasting within COAST_TIMEOUT_S, False once the target
        # is declared lost. Downstream head control should stop slewing when False.
        self.target_locked_pub = self.create_publisher(Bool, '/head/target_locked', 10)

        # Periodic IMU health check: warns loudly if the gyro stream never arrives
        # (e.g. launched without unite_imu_method) or stalls mid-run, since ego-motion
        # compensation silently degrades to identity (no compensation) in that case.
        self.imu_health_timer = self.create_timer(IMU_HEALTH_PERIOD_S, self._check_imu_health)

        # Lightweight processing-rate meter: count frames and log FPS every 2 s (no per-frame
        # console spam). Tells you the real throughput of the vision pipeline.
        self._fps_count = 0
        self.fps_timer = self.create_timer(2.0, self._report_fps)

        self.get_logger().info('vision_tracking node started')

    def _report_fps(self):
        fps = self._fps_count / 2.0
        self._fps_count = 0
        self.get_logger().info(f'vision pipeline: {fps:.1f} FPS')

    def _check_imu_health(self):
        now = self.get_clock().now()
        if self.last_imu_rx_time is None:
            self.get_logger().warn(
                f'No IMU messages on {IMU_TOPIC} yet - ego-motion compensation is DISABLED '
                f'(R_cur stays identity). Did you launch with unite_imu_method:=1 '
                f'enable_gyro:=true enable_accel:=true?'
            )
            return
        stale_s = (now - self.last_imu_rx_time).nanoseconds / 1e9
        if stale_s > IMU_STALE_S:
            self.get_logger().warn(
                f'IMU stream stalled ({stale_s:.1f}s since last message) - ego-motion '
                f'compensation is coasting on a stale rotation.'
            )

    def imu_callback(self, imu_msg: Imu):
        """Integrate gyro angular velocity into the running camera->reference rotation.

        Uses strapdown integration: over a small dt the incremental body rotation is
        exp([w]_x * dt), obtained via Rodrigues, and right-multiplied onto R_cur so that
        R_cur maps the current camera frame back to the fixed reference frame.
        """
        self.last_imu_rx_time = self.get_clock().now()
        t_ns = Time.from_msg(imu_msg.header.stamp).nanoseconds
        if self.last_imu_time_ns is None:
            self.last_imu_time_ns = t_ns
            self.imu_ready = True
            return

        dt_s = (t_ns - self.last_imu_time_ns) / 1e9
        self.last_imu_time_ns = t_ns
        # Drop non-monotonic stamps and large gaps (e.g. after a stall) so a single
        # bad dt cannot inject a huge spurious rotation.
        if dt_s <= 0.0 or dt_s > 0.1:
            return

        omega = np.array(
            [imu_msg.angular_velocity.x,
             imu_msg.angular_velocity.y,
             imu_msg.angular_velocity.z],
            dtype=np.float64,
        ) - self.gyro_bias

        # If empirically the target velocity DOUBLES instead of cancelling when the head
        # moves (verification step 3), the gyro sign is inverted for this frame: negate
        # rotvec here.
        rotvec = omega * dt_s
        dR, _ = cv2.Rodrigues(rotvec)
        self.R_cur = self.R_cur @ dR

    def _project_ref_point(self, pos_ref, camera_info_msg, R_now):
        """Project a reference-frame 3D point to a full-resolution color pixel.

        Rotates the point back into the current camera frame first. Returns a float
        (u, v) or None if intrinsics/geometry are unusable.
        """
        if camera_info_msg is None:
            return None
        fx = float(camera_info_msg.k[0]) if len(camera_info_msg.k) > 0 else 0.0
        fy = float(camera_info_msg.k[4]) if len(camera_info_msg.k) > 4 else 0.0
        cx = float(camera_info_msg.k[2]) if len(camera_info_msg.k) > 2 else 0.0
        cy = float(camera_info_msg.k[5]) if len(camera_info_msg.k) > 5 else 0.0
        if fx <= 0.0 or fy <= 0.0:
            return None
        pos_cam = R_now.T @ pos_ref if R_now is not None else pos_ref
        z = pos_cam[2]
        if z <= 0.0:
            return None
        return (fx * pos_cam[0] / z + cx, fy * pos_cam[1] / z + cy)

    def _compute_search_roi(self, color_shape, camera_info_msg, R_now, current_time_ns, force_rescan=False):
        """Velocity-aware track-before-detect ROI.

        The window is centered on the *velocity-led* predicted pixel (pos + vel*dt) so it
        stays tight even for fast targets, and its half-size is base + per-miss growth +
        a capped share (ROI_VEL_GAIN) of the predicted pixel displacement as uncertainty
        margin. Returns (x0, y0, x1, y1) in color pixels, or None to fall back to a
        full-frame search (cold start, off-frame prediction, effectively full-frame, or a
        periodic rescan).
        """
        if not self.filter_initialized or force_rescan:
            return None

        pos = np.array(
            [float(self.kf.statePost[i, 0]) for i in range(3)], dtype=np.float64
        )
        vel = np.array(
            [float(self.kf.statePost[i, 0]) for i in range(3, 6)], dtype=np.float64
        )
        if self.filter_time_ns is None:
            dt_s = 0.033
        else:
            dt_s = max(0.0, (current_time_ns - self.filter_time_ns) / 1e9)
        dt_s = min(dt_s, ROI_LEAD_DT_MAX_S)

        center = self._project_ref_point(pos + vel * dt_s, camera_info_msg, R_now)
        if center is None:
            return None
        u, v = center

        h, w = color_shape[:2]
        base_frac = ROI_BASE_HALF_FRAC + ROI_GROW_FRAC * self.consecutive_misses
        half_w = base_frac * w
        half_h = base_frac * h

        # Add uncertainty margin proportional to the predicted pixel displacement.
        base_px = self._project_ref_point(pos, camera_info_msg, R_now)
        if base_px is not None:
            half_w += ROI_VEL_GAIN * abs(u - base_px[0])
            half_h += ROI_VEL_GAIN * abs(v - base_px[1])

        # Cap so the ROI keeps its zoom-in benefit.
        half_w = min(half_w, ROI_MAX_HALF_FRAC * w)
        half_h = min(half_h, ROI_MAX_HALF_FRAC * h)

        x0 = max(0, int(u - half_w))
        y0 = max(0, int(v - half_h))
        x1 = min(w, int(u + half_w))
        y1 = min(h, int(v + half_h))
        # Degenerate (prediction off-frame) or effectively full-frame -> full search.
        if x1 - x0 < 16 or y1 - y0 < 16:
            return None
        if (x1 - x0) >= w and (y1 - y0) >= h:
            return None
        return (x0, y0, x1, y1)

    def _sample_depth_m(self, depth_frame, depth_x, depth_y):
        """Robust depth (meters): median of nonzero values in a small window.

        A single pixel on an object edge or a depth hole can read a wildly wrong value;
        the median over a window rejects those outliers. Returns 0.0 if no valid depth.
        """
        h, w = depth_frame.shape[:2]
        x0 = max(0, depth_x - DEPTH_WIN_HALF)
        x1 = min(w, depth_x + DEPTH_WIN_HALF + 1)
        y0 = max(0, depth_y - DEPTH_WIN_HALF)
        y1 = min(h, depth_y + DEPTH_WIN_HALF + 1)
        patch = depth_frame[y0:y1, x0:x1].astype(np.float32)
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size == 0:
            return 0.0
        return float(np.median(valid)) / 1000.0

    def _deproject_center(self, center_x, center_y, depth_frame, camera_info_msg, R_now, color_w, color_h):
        """Deproject a full-resolution color pixel to a reference-frame 3D point.

        Returns (point_ref float32[3], z_m), or (None, 0.0) if depth/intrinsics are unusable.
        """
        depth_w = depth_frame.shape[1]
        depth_h = depth_frame.shape[0]
        depth_x = max(0, min(depth_w - 1, int(round(center_x * depth_w / color_w))))
        depth_y = max(0, min(depth_h - 1, int(round(center_y * depth_h / color_h))))
        z_m = self._sample_depth_m(depth_frame, depth_x, depth_y)

        fx = float(camera_info_msg.k[0]) if len(camera_info_msg.k) > 0 else 0.0
        fy = float(camera_info_msg.k[4]) if len(camera_info_msg.k) > 4 else 0.0
        cx = float(camera_info_msg.k[2]) if len(camera_info_msg.k) > 2 else 0.0
        cy = float(camera_info_msg.k[5]) if len(camera_info_msg.k) > 5 else 0.0
        if fx <= 0.0 or fy <= 0.0 or z_m <= 0.0:
            return None, 0.0
        x_m = (center_x - cx) * z_m / fx
        y_m = (center_y - cy) * z_m / fy
        point_cam = np.array([x_m, y_m, z_m], dtype=np.float64)
        return (R_now @ point_cam).astype(np.float32), z_m

    def _deproject_box(self, box, geom, depth_frame, camera_info_msg, R_now):
        """Deproject one YOLO box to a candidate (3D point + full-frame pixel bbox).

        box.xyxy is already in full-res color pixel coords (Ultralytics letterboxes and maps
        back internally). geom = (color_w, color_h), used for the depth-pixel scaling.
        """
        color_w, color_h = geom
        conf = float(box.conf[0]) if len(box.conf) > 0 else 0.0
        x1f, y1f, x2f, y2f = [float(v) for v in box.xyxy[0].tolist()]
        center_x = int(round((x1f + x2f) / 2.0))
        center_y = int(round((y1f + y2f) / 2.0))
        point_ref, z_m = self._deproject_center(
            center_x, center_y, depth_frame, camera_info_msg, R_now, color_w, color_h)
        if point_ref is None:
            return None
        bbox = (int(x1f), int(y1f), max(1, int(x2f - x1f)), max(1, int(y2f - y1f)))
        return {
            'conf': conf,
            'center_x': center_x,
            'center_y': center_y,
            'z_m': z_m,
            'point_ref': point_ref,
            'bbox': bbox,
        }

    # --- Tracker fusion helpers ----------------------------------------------
    def _create_tracker(self):
        if not self._vit_ok:
            return None
        try:
            params = cv2.TrackerVit_Params()
            params.net = TRACKER_MODEL_PATH
            return cv2.TrackerVit_create(params)
        except Exception as exc:  # pragma: no cover - runtime fallback
            self.get_logger().warn(f'Failed to create TrackerVit: {exc}')
            return None

    def _condition_bbox(self, bbox, w, h):
        """Pad the YOLO box for template context and floor its size, then clamp to frame.
        Keeps the box CENTER fixed (so the measured target pixel is unchanged) while giving
        vittrack a bigger template + search window on small/distant drones."""
        x, y, bw, bh = (float(v) for v in bbox)
        cx, cy = x + bw / 2.0, y + bh / 2.0
        bw = max(bw * TRACKER_INIT_PAD, float(TRACKER_MIN_BOX_PX))
        bh = max(bh * TRACKER_INIT_PAD, float(TRACKER_MIN_BOX_PX))
        xi = max(0, min(int(round(cx - bw / 2.0)), w - 2))
        yi = max(0, min(int(round(cy - bh / 2.0)), h - 2))
        bwi = max(1, min(int(round(bw)), w - xi))
        bhi = max(1, min(int(round(bh)), h - yi))
        return (xi, yi, bwi, bhi)

    def _init_tracker(self, frame, bbox):
        # Reuse the tracker instance across (re-)inits so the ONNX model loads only once.
        if self.tracker is None:
            self.tracker = self._create_tracker()
            if self.tracker is None:
                return False
        h, w = frame.shape[:2]
        box = self._condition_bbox(bbox, w, h)   # pad + min-size + clamp
        try:
            self.tracker.init(frame, box)
        except Exception as exc:  # pragma: no cover - runtime fallback
            self.get_logger().warn(f'Tracker init failed: {exc}')
            return False
        self.tracker_ok = True
        self.tracker_validate_misses = 0
        return True

    def _tracker_score(self):
        try:
            return float(self.tracker.getTrackingScore())
        except Exception:
            return 1.0  # trackers without a score are trusted while update() returns True

    def _yolo_candidates(self, color_frame, depth_frame, camera_info_msg, R_now, color_w, color_h):
        """Full-frame YOLO detection → deprojected candidate list.

        The NATIVE (rectangular) frame is handed straight to Ultralytics with imgsz=640.
        It LETTERBOXES internally (pads to square, preserving aspect — no stretch distortion)
        and maps detections back, so box.xyxy is already in full-res color pixel coords.
        device=0 -> CUDA GPU; verbose=False -> suppress the per-call console print.
        """
        results = self.model.predict(source=color_frame, conf=FULL_CONF, stream=False,
                                     classes=[0], device=0, verbose=False, imgsz=640)
        geom = (color_w, color_h)
        cands = []
        for box in results[0].boxes:
            c = self._deproject_box(box, geom, depth_frame, camera_info_msg, R_now)
            if c is not None:
                cands.append(c)
        return cands

    def _match_candidate(self, tracker_box, cands, color_w):
        """Nearest YOLO candidate to the tracker box center, within the match radius."""
        if not cands:
            return None
        tx = tracker_box[0] + tracker_box[2] / 2.0
        ty = tracker_box[1] + tracker_box[3] / 2.0
        thresh = TRACKER_MATCH_DIST_FRAC * color_w
        best, best_d = None, None
        for c in cands:
            d = ((c['center_x'] - tx) ** 2 + (c['center_y'] - ty) ** 2) ** 0.5
            if d <= thresh and (best_d is None or d < best_d):
                best_d, best = d, c
        return best

    def _adaptive_meas_noise(self, point_ref, R_now):
        """Range-adaptive, anisotropic measurement-noise covariance (3x3, ref frame).

        Depth (camera optical Z) is the unreliable axis and its error grows with range, so
        its variance is inflated by (z / KF_MEAS_Z_REF_M)^2 (clamped). X/Y are pixel-derived
        and kept low. The covariance is built in the camera frame and rotated into the
        filter's reference frame by R_now so the "distrust depth" direction stays correct
        even when the head is turned.
        """
        if R_now is not None:
            z = float((R_now.T @ point_ref.astype(np.float64))[2])
        else:
            z = float(point_ref[2])
        # Guard against zero/negative depth, then inflate Z variance with range^2 (capped).
        z = max(z, KF_MEAS_Z_REF_M * 0.1)
        z_factor = min(max((z / KF_MEAS_Z_REF_M) ** 2, 1.0), KF_MEAS_Z_MAX_FACTOR)
        sigma_cam = np.diag(
            [KF_MEAS_NOISE_XY, KF_MEAS_NOISE_XY, KF_MEAS_NOISE_Z_BASE * z_factor]
        ).astype(np.float64)
        if R_now is not None:
            sigma_ref = R_now @ sigma_cam @ R_now.T
        else:
            sigma_ref = sigma_cam
        return sigma_ref.astype(np.float32)

    def _predict_ref_position(self, current_time_ns):
        """Constant-velocity prediction of the reference-frame position, for gating."""
        if not self.filter_initialized:
            return None
        if self.filter_time_ns is None:
            dt_s = 0.033
        else:
            dt_s = max(1e-6, (current_time_ns - self.filter_time_ns) / 1e9)
        pos = np.array([float(self.kf.statePost[i, 0]) for i in range(3)], dtype=np.float64)
        vel = np.array([float(self.kf.statePost[i, 0]) for i in range(3, 6)], dtype=np.float64)
        return pos + vel * dt_s

    def _prune_blacklist(self, now_ns):
        """Drop expired blacklist entries (stationary-clutter locations)."""
        if self.blacklist:
            self.blacklist = [(p, e) for (p, e) in self.blacklist if e > now_ns]

    def _is_blacklisted(self, point_ref):
        """True if a candidate sits near a recently-dropped stationary-clutter location."""
        pr = point_ref.astype(np.float64)
        for (p, _e) in self.blacklist:
            if np.linalg.norm(pr - p) < BLACKLIST_RADIUS_M:
                return True
        return False

    def _select_measurement(self, candidates, current_time_ns):
        """Data association: choose which detection (if any) to feed the filter.

        Acquisition (filter not initialized): highest confidence, no gate.
        Tracking: the in-gate candidate closest to the prediction; None if every
        candidate is outside the validation gate (-> coast instead of jumping).
        Candidates near a blacklisted (stationary-clutter) location are excluded.
        Returns (selected_or_None, rejected_count, gate_m).
        """
        self._prune_blacklist(current_time_ns)
        if self.blacklist:
            candidates = [c for c in candidates if not self._is_blacklisted(c['point_ref'])]
        if not candidates:
            return None, 0, 0.0
        if not self.filter_initialized:
            return max(candidates, key=lambda c: c['conf']), 0, 0.0

        pred = self._predict_ref_position(current_time_ns)
        gate = GATE_BASE_M + GATE_GROW_M * self.consecutive_misses
        best = None
        best_dist = None
        rejected = 0
        for c in candidates:
            dist = float(np.linalg.norm(c['point_ref'].astype(np.float64) - pred))
            if dist > gate:
                rejected += 1
                continue
            if best_dist is None or dist < best_dist:
                best_dist = dist
                best = c
        if best is not None:
            best['gate_dist'] = best_dist
        return best, rejected, gate

    def sync_callback(self, color_msg: Image, depth_msg: Image, camera_info_msg: CameraInfo):
        self.last_sync_time = self.get_clock().now()
        self.latest_color = color_msg
        self.latest_depth = depth_msg
        self.latest_camera_info = camera_info_msg

        try:
            color_frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert synchronized messages: {exc}')
            return

        self.get_logger().debug(
            f'Synchronized pair received: color={color_frame.shape[1]}x{color_frame.shape[0]} '
            f'depth={depth_frame.shape[1]}x{depth_frame.shape[0]} camera_info={camera_info_msg.header.frame_id}'
        )

        if self.model is None:
            self.get_logger().info('YOLO model not ready; skipping inference for this synchronized pair')
            return

        try:
            # Snapshot the ego-motion rotation once per frame.
            R_now = self.R_cur.copy()
            color_h, color_w = color_frame.shape[:2]
            current_time_ns = Time.from_msg(color_msg.header.stamp).nanoseconds
            self.frame_count += 1
            self._fps_count += 1

            measurement_point = None   # reference-frame 3D point to feed the filter
            pixel_meas = None          # target pixel (u, v) in full-res color coords
            src = None                 # log label for where the measurement came from

            # ---- FOLLOW: tracker holds the lock by appearance, every frame ----
            tracker_box = None
            if self.tracker_ok:
                ok, box = self.tracker.update(color_frame)
                score = self._tracker_score()
                if ok and score >= TRACKER_MIN_SCORE:
                    tracker_box = tuple(int(v) for v in box)
                    src = f'track({score:.2f})'
                    # Periodically re-validate/re-anchor against YOLO to correct drift and
                    # confirm it is still a drone.
                    if self.frame_count % TRACKER_VALIDATE_EVERY_N == 0:
                        cands = self._yolo_candidates(
                            color_frame, depth_frame, camera_info_msg, R_now, color_w, color_h)
                        match = self._match_candidate(tracker_box, cands, color_w)
                        if match is not None:
                            self._init_tracker(color_frame, match['bbox'])  # re-anchor
                            tracker_box = match['bbox']
                            src = f'track+yolo({score:.2f})'
                        else:
                            self.tracker_validate_misses += 1
                            self.get_logger().info(
                                f'Tracker validation miss '
                                f'{self.tracker_validate_misses}/{TRACKER_VALIDATE_MAX_MISSES}'
                            )
                            if self.tracker_validate_misses >= TRACKER_VALIDATE_MAX_MISSES:
                                self.get_logger().warn('Tracker unconfirmed by YOLO; dropping.')
                                self.tracker_ok = False
                                tracker_box = None
                else:
                    # On-loss diagnostic: box size (px) + score tell us WHY it dropped —
                    # tiny box => small/distant target escaping the search window; low score
                    # on a big box => appearance/blur. box is vittrack's last estimate.
                    bw_l, bh_l = int(box[2]), int(box[3])
                    self.get_logger().warn(
                        f'Tracker DROPPED: score={score:.2f} (< {TRACKER_MIN_SCORE}), ok={ok}, '
                        f'last_box={bw_l}x{bh_l}px. Re-acquiring full-frame.')
                    self.tracker_ok = False

            # ---- ACQUIRE: no active tracker -> full-frame YOLO, pick best (blacklist-aware) ----
            if not self.tracker_ok:
                cands = self._yolo_candidates(
                    color_frame, depth_frame, camera_info_msg, R_now, color_w, color_h)
                self._prune_blacklist(current_time_ns)
                if self.blacklist:
                    cands = [c for c in cands if not self._is_blacklisted(c['point_ref'])]
                if cands:
                    best = max(cands, key=lambda c: c['conf'])
                    if self._init_tracker(color_frame, best['bbox']):
                        tracker_box = best['bbox']
                        src = f'acquire(conf={best["conf"]:.2f})'
                        self.get_logger().info(
                            f'Acquired target conf={best["conf"]:.2f}; tracker initialized.')
                    else:
                        # No tracker available: use the YOLO detection directly.
                        measurement_point = best['point_ref']
                        pixel_meas = (best['center_x'], best['center_y'])
                        src = f'yolo-only(conf={best["conf"]:.2f})'

            # ---- Deproject the current box center -> measurement ----
            if tracker_box is not None and measurement_point is None:
                cx = tracker_box[0] + tracker_box[2] / 2.0
                cy = tracker_box[1] + tracker_box[3] / 2.0
                pixel_meas = (cx, cy)
                pr, _z = self._deproject_center(
                    int(cx), int(cy), depth_frame, camera_info_msg, R_now, color_w, color_h)
                if pr is not None:
                    measurement_point = pr

            # ---- Feed the Kalman filter ----
            if measurement_point is None:
                self.consecutive_misses += 1
                self._update_kalman_filter(None, current_time_ns, camera_info_msg, R_now)
            else:
                self.consecutive_misses = 0
                # EMA-smooth the target pixel for the depth-free head-centering command.
                if pixel_meas is not None:
                    if self.pixel_filt is None:
                        self.pixel_filt = np.array(pixel_meas, dtype=np.float64)
                    else:
                        self.pixel_filt = (PIXEL_EMA_ALPHA * np.array(pixel_meas, dtype=np.float64)
                                           + (1.0 - PIXEL_EMA_ALPHA) * self.pixel_filt)
                self.get_logger().debug(
                    f'[{src}] ref=({measurement_point[0]:.2f},'
                    f'{measurement_point[1]:.2f},{measurement_point[2]:.2f}) m'
                )
                self._update_kalman_filter(measurement_point, current_time_ns, camera_info_msg, R_now)

        except Exception as exc:  # pragma: no cover - runtime fallback
            self.get_logger().warn(f'Detection/tracking failed: {exc}')

    def _update_kalman_filter(self, measurement_point, current_time_ns, camera_info_msg=None, R_now=None):
        if measurement_point is None:
            if not self.filter_initialized:
                return
            # Coast timeout: if we have been extrapolating without a real detection for
            # longer than COAST_TIMEOUT_S, the constant-velocity guess has drifted too
            # far to trust. Declare the target lost, stop publishing a position, and
            # reset the filter so the next detection re-initializes cleanly instead of
            # snapping from a stale coasted state.
            if self.last_detection_time_ns is not None:
                coast_s = (current_time_ns - self.last_detection_time_ns) / 1e9
                if coast_s > COAST_TIMEOUT_S:
                    if self.target_locked:
                        self.get_logger().warn(
                            f'Target lost: no detection for {coast_s:.2f}s '
                            f'(> {COAST_TIMEOUT_S}s coast limit); halting output.'
                        )
                    self.filter_initialized = False
                    self.filter_time_ns = None
                    self.confirm_hits = 0
                    self.pixel_filt = None
                    self._set_target_locked(False)
                    return
            dt_s = 0.033 if self.filter_time_ns is None else max(1e-6, (current_time_ns - self.filter_time_ns) / 1e9)
            self.filter_time_ns = current_time_ns
            F = np.array(
                [[1, 0, 0, dt_s, 0, 0],
                 [0, 1, 0, 0, dt_s, 0],
                 [0, 0, 1, 0, 0, dt_s],
                 [0, 0, 0, 1, 0, 0],
                 [0, 0, 0, 0, 1, 0],
                 [0, 0, 0, 0, 0, 1]], dtype=np.float32
            )
            self.kf.transitionMatrix = F
            predicted = self.kf.predict()
            self.kf.statePost = predicted
            filtered_state = self.kf.statePost
            self.get_logger().debug(
                f'KF predict: pos=({filtered_state[0,0]:.3f}, {filtered_state[1,0]:.3f}, {filtered_state[2,0]:.3f}) '
                f'vel=({filtered_state[3,0]:.3f}, {filtered_state[4,0]:.3f}, {filtered_state[5,0]:.3f})'
            )
            self._publish_tracking_outputs(filtered_state, camera_info_msg, R_now)
            return

        if not self.filter_initialized:
            self.kf.statePost = np.zeros((6, 1), dtype=np.float32)
            self.kf.statePost[:3, 0] = measurement_point
            self.kf.statePost[3:, 0] = 0.0
            self.filter_initialized = True
            self.filter_time_ns = current_time_ns
            self.last_detection_time_ns = current_time_ns
            self.confirm_hits = 1
            self.get_logger().info(
                f'KF init: pos=({measurement_point[0]:.3f}, {measurement_point[1]:.3f}, {measurement_point[2]:.3f})'
            )
            self._publish_tracking_outputs(self.kf.statePost, camera_info_msg, R_now)
            return

        dt_s = max(1e-6, (current_time_ns - self.filter_time_ns) / 1e9) if self.filter_time_ns is not None else 0.033
        self.filter_time_ns = current_time_ns
        F = np.array(
            [[1, 0, 0, dt_s, 0, 0],
             [0, 1, 0, 0, dt_s, 0],
             [0, 0, 1, 0, 0, dt_s],
             [0, 0, 0, 1, 0, 0],
             [0, 0, 0, 0, 1, 0],
             [0, 0, 0, 0, 0, 1]], dtype=np.float32
        )
        self.kf.transitionMatrix = F
        self.kf.predict()

        measurement = np.array([[measurement_point[0]], [measurement_point[1]], [measurement_point[2]]], dtype=np.float32)
        # Trust pixel-derived X/Y; distrust depth Z more as the target gets farther.
        self.kf.measurementNoiseCov = self._adaptive_meas_noise(measurement_point, R_now)
        self.kf.correct(measurement)
        self.last_detection_time_ns = current_time_ns
        self.confirm_hits = min(CONFIRM_HITS, self.confirm_hits + 1)
        filtered_state = self.kf.statePost

        # Motion validation: if this lock has been stationary too long, it's clutter.
        if self._check_static_lock(filtered_state, current_time_ns):
            return

        self.get_logger().debug(
            f'KF update: pos=({filtered_state[0,0]:.3f}, {filtered_state[1,0]:.3f}, {filtered_state[2,0]:.3f}) '
            f'vel=({filtered_state[3,0]:.3f}, {filtered_state[4,0]:.3f}, {filtered_state[5,0]:.3f})'
        )
        self._publish_tracking_outputs(filtered_state, camera_info_msg, R_now)

    def _check_static_lock(self, filtered_state, current_time_ns):
        """Drop (and blacklist) a lock that has stayed stationary too long.

        Uses the reference-frame velocity, which is ego-motion compensated, so head
        pan/tilt does not count as target motion. Returns True if the lock was dropped.

        Disabled when the appearance tracker is in use: the tracker + YOLO re-validation
        is a better clutter rejector, and motion-dropping would falsely shed a hovering
        drone (the lock/search flashing we saw).
        """
        if self._vit_ok:
            return False
        speed = float(np.linalg.norm(
            np.array([filtered_state[3, 0], filtered_state[4, 0], filtered_state[5, 0]], dtype=np.float64)
        ))
        if speed >= STATIC_SPEED_MPS:
            self.static_since_ns = None
            return False
        if self.static_since_ns is None:
            self.static_since_ns = current_time_ns
            return False
        if (current_time_ns - self.static_since_ns) / 1e9 < STATIC_TIMEOUT_S:
            return False

        pos_ref = np.array(
            [filtered_state[0, 0], filtered_state[1, 0], filtered_state[2, 0]], dtype=np.float64
        )
        self.blacklist.append((pos_ref, current_time_ns + int(BLACKLIST_S * 1e9)))
        self.get_logger().warn(
            f'Dropping stationary lock as clutter (speed {speed:.3f} < {STATIC_SPEED_MPS} m/s '
            f'for > {STATIC_TIMEOUT_S}s); blacklisting for {BLACKLIST_S}s.'
        )
        self.filter_initialized = False
        self.filter_time_ns = None
        self.confirm_hits = 0
        self.static_since_ns = None
        self._set_target_locked(False)
        return True

    def _publish_tracking_outputs(self, filtered_state, camera_info_msg=None, R_now=None):
        # The filter runs in the ego-motion reference frame; rotate the filtered
        # position back into the current camera optical frame for centering/output.
        pos_ref = np.array(
            [float(filtered_state[0, 0]), float(filtered_state[1, 0]), float(filtered_state[2, 0])],
            dtype=np.float64,
        )
        if R_now is not None:
            pos_cam = R_now.T @ pos_ref
        else:
            pos_cam = pos_ref

        filtered_point = PointStamped()
        filtered_point.header.stamp = self.get_clock().now().to_msg()
        filtered_point.header.frame_id = 'camera_color_optical_frame'
        filtered_point.point.x = float(pos_cam[0])
        filtered_point.point.y = float(pos_cam[1])
        filtered_point.point.z = float(pos_cam[2])
        self.tracked_point_pub.publish(filtered_point)

        # Filtered velocity, rotated from the reference frame back into the current
        # camera frame (same transform as position) for the HUD motion arrow.
        vel_ref = np.array(
            [float(filtered_state[3, 0]), float(filtered_state[4, 0]), float(filtered_state[5, 0])],
            dtype=np.float64,
        )
        vel_cam = R_now.T @ vel_ref if R_now is not None else vel_ref
        velocity_msg = Vector3()
        velocity_msg.x = float(vel_cam[0])
        velocity_msg.y = float(vel_cam[1])
        velocity_msg.z = float(vel_cam[2])
        self.target_velocity_pub.publish(velocity_msg)

        angular_error_msg = Vector3()
        angular_error_msg.x = 0.0
        angular_error_msg.y = 0.0
        angular_error_msg.z = 0.0

        if camera_info_msg is not None:
            fx = float(camera_info_msg.k[0]) if len(camera_info_msg.k) > 0 else 0.0
            fy = float(camera_info_msg.k[4]) if len(camera_info_msg.k) > 4 else 0.0
            cx = float(camera_info_msg.k[2]) if len(camera_info_msg.k) > 2 else 0.0
            cy = float(camera_info_msg.k[5]) if len(camera_info_msg.k) > 5 else 0.0

            if fx > 0.0 and fy > 0.0 and self.pixel_filt is not None:
                # Depth-free head centering: the angle to the target is purely a function
                # of its (smoothed) pixel position, so depth noise never twitches the head.
                u = float(self.pixel_filt[0])
                v = float(self.pixel_filt[1])
                angular_error_msg.x = float(np.arctan2(u - cx, fx))
                angular_error_msg.y = float(np.arctan2(v - cy, fy))
                angular_error_msg.z = 0.0
                self.get_logger().debug(
                    f'Angular error: yaw={angular_error_msg.x:.4f} rad pitch={angular_error_msg.y:.4f} rad'
                )

        self.angular_error_pub.publish(angular_error_msg)
        # Only claim a lock once the track has been confirmed by several consecutive
        # in-gate detections; a single false positive stays "SEARCHING". Once confirmed,
        # the lock latches through the coast window (confirm_hits is only reset on loss).
        self._set_target_locked(self.filter_initialized and self.confirm_hits >= CONFIRM_HITS)

    def _set_target_locked(self, locked: bool):
        """Publish lock state, and log the edge when it changes."""
        if locked != self.target_locked:
            self.target_locked = locked
            self.get_logger().info(f'Target lock -> {"LOCKED" if locked else "LOST"}')
        self.target_locked_pub.publish(Bool(data=locked))


def main(args=None):
    rclpy.init(args=args)
    node = vision_tracking()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
