"""On-demand turret-cam HUD for the drone tracker.

This node draws NOTHING heavy: it renders a lightweight turret overlay (boresight
crosshair, target reticle, predicted-motion arrow, status + range readout) on top of
the live color feed. All tracking math lives in the headless vision_tracking node; this
node is a pure consumer of the published track topics, so it can be opened or closed at
any time without affecting tracking.

The feed can be shown as a TINTED MONOCHROME (IR-seeker look) so the colored symbology
reads clearly against a desaturated background. Cycle the tint with the 'm' key.

Run standalone:  ros2 run orex_head_vision turret_view
Keys:  m = cycle sensor tint    q / ESC = close
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data

import cv2
import numpy as np
from geometry_msgs.msg import PointStamped, Vector3
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from cv_bridge import CvBridge


WINDOW_NAME = 'Turret Cam'

# A track older than this (seconds) is treated as stale -> HUD shows SEARCHING.
TRACK_STALE_S = 0.4
# A camera frame older than this (seconds) -> HUD shows a WAITING placeholder.
IMAGE_STALE_S = 1.0
# HUD render rate (Hz). Rendering is timer-driven so the window always appears, even
# before any camera frame has arrived.
RENDER_HZ = 30.0

# --- Sensor tint modes (cycled with 'm') --------------------------------------------
# Each mode maps the frame's grayscale LUMINANCE to a BGR tint via per-channel weights,
# so the video desaturates but the colored overlay drawn on top still shows in full color.
# 'COLOR' (weights None) leaves the feed untouched. Default is the green IR-seeker look.
TINT_MODES = [
    ('IR GREEN', (0.40, 0.66, 0.40)),   # muted, grayed-down phosphor green (softer background)
    ('WHITE HOT', (0.60, 0.60, 0.60)),  # dimmed neutral grey (not full-brightness white)
    ('COLOR', None),                    # untouched color feed
]
DEFAULT_TINT_IDX = 0

# All symbology (text, crosshair, reticle, brackets) is drawn in bright green, in every
# tint mode — a single uniform seeker palette.
SYMBOL_GREEN = (0, 255, 0)  # BGR
HUD_COLORS = {'struct': SYMBOL_GREEN, 'locked': SYMBOL_GREEN, 'search': SYMBOL_GREEN}

# Corsair parameters
XHAIR_CENTER_GAP = 16   # empty hole radius: distance from exact center to the inner tip
XHAIR_INNER_LEN  = 50  # length of the inner segment (nearest the center)
XHAIR_BREAK_GAP  = 30   # gap between the inner and outer segments
XHAIR_OUTER_LEN  = 100   # length of the outer segment (= 3 x inner, per the 1/3 ratio)
XHAIR_T_HALF     = 5    # half-width of the 'T' cap at the inner tip (full cap = 2x this)
XHAIR_THICK      = 1    # line thickness

# --- Target box drawn over the locked drone (see drone tracker.png / warthunder ui.png) ---
# A square box with a break in the MIDDLE of each edge; from each break a thin line runs
# out to the very edge of the frame. Inside sits a small circle with a fixed-length line
# showing the drone's travel DIRECTION (not speed), and the range is printed beside the box.
BOX_HALF          = 28   # half side length of the square target box (px)
BOX_EDGE_GAP      = 18   # width of the center break on each box edge (px)
BOX_THICK         = 2    # box edge thickness (px)
TARGET_LINE_THICK = 1    # thickness of the 4 lines running from the box breaks to the frame edges
DIR_CIRCLE_R      = 5    # radius of the direction-indicator circle (px)
DIR_LINE_LEN      = 20   # FIXED length of the direction line (px) — does NOT scale with speed
DIR_LOOKAHEAD_S   = 0.3  # look-ahead used only to derive the velocity DIRECTION in pixels
DIR_THICK         = 2    # direction line / circle thickness (px)
DIR_BOX_GAP       = 8    # gap between the box's right edge and the direction circle (px)
DIR_MIN_SPEED_MPS = 0.25 # below this lateral speed the drone is "stationary": hide the spinning
                         # heading line (its direction is just noise) and show only the circle
DIR_EMA_ALPHA     = 0.25 # smoothing of the drawn heading (lower = steadier, more lag)


class turret_view(Node):
    def __init__(self):
        super().__init__('turret_view')

        self.bridge = CvBridge()
        self.camera_info = None
        self.target_point = None          # geometry_msgs/Point (camera frame, meters)
        self.target_velocity = None       # geometry_msgs/Vector3 (camera frame, m/s)
        self.target_locked = False
        self.last_track_time = None       # rclpy Time of last target_point
        self.latest_frame = None          # most recent BGR camera frame
        self.last_image_time = None       # rclpy Time of last camera frame
        self.tint_idx = DEFAULT_TINT_IDX  # current sensor tint mode
        self._dir_ema = None              # EMA-smoothed heading unit vector (u,v), or None

        self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10
        )
        self.create_subscription(PointStamped, '/head/target_point', self.point_callback, 10)
        self.create_subscription(Vector3, '/head/target_velocity', self.velocity_callback, 10)
        self.create_subscription(Bool, '/head/target_locked', self.locked_callback, 10)

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        # Render on a timer (not only on image callbacks) so the window shows up right
        # away with a WAITING placeholder even before any camera frame arrives.
        self.render_timer = self.create_timer(1.0 / RENDER_HZ, self._render)
        self.get_logger().info('turret_view HUD started (m = tint, q or ESC to close)')

    # --- subscription handlers ------------------------------------------------
    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def point_callback(self, msg: PointStamped):
        self.target_point = msg.point
        self.last_track_time = self.get_clock().now()

    def velocity_callback(self, msg: Vector3):
        self.target_velocity = msg

    def locked_callback(self, msg: Bool):
        self.target_locked = msg.data

    def image_callback(self, msg: Image):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_image_time = self.get_clock().now()
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert image: {exc}')

    # --- tint / colors --------------------------------------------------------
    def _apply_tint(self, frame):
        """Desaturate the feed to a single-hue monochrome, kept as a 3-channel BGR image
        so the colored overlay still renders on top. 'COLOR' mode returns it untouched."""
        _name, weights = TINT_MODES[self.tint_idx]
        if weights is None:
            return frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)
        tinted = gray[:, :, None] * np.array(weights, dtype=np.float32)
        return np.clip(tinted, 0, 255).astype(np.uint8)

    def _colors(self):
        """Uniform bright-green symbology palette for every tint mode."""
        return HUD_COLORS

    # --- rendering ------------------------------------------------------------
    def _project(self, x, y, z):
        """Project a camera-frame point (m) to a pixel (u, v), or None if invalid."""
        if self.camera_info is None or z <= 0.0:
            return None
        k = self.camera_info.k
        fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
        if fx <= 0.0 or fy <= 0.0:
            return None
        return int(round(fx * x / z + cx)), int(round(fy * y / z + cy))

    def _track_is_fresh(self):
        if self.target_point is None or self.last_track_time is None:
            return False
        age_s = (self.get_clock().now() - self.last_track_time).nanoseconds / 1e9
        return age_s <= TRACK_STALE_S

    def _draw_center_crosshair(self, frame, color):
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        # Cumulative distances from the center, measured outward along an arm.
        d_tip = XHAIR_CENTER_GAP                                              # inner tip (T sits here)
        d_in_end = d_tip + XHAIR_INNER_LEN                                    # end of the inner segment
        d_out_start = d_in_end + XHAIR_BREAK_GAP                              # start of the outer segment
        d_out_end = d_out_start + XHAIR_OUTER_LEN                             # end of the outer segment
        # Draw each of the four arms via its outward unit direction (dx, dy).
        for dx, dy in ((0, -1), (0, 1), (-1, 0), (1, 0)):
            px, py = -dy, dx                                                  # perpendicular, for the T cap
            tip = (cx + dx * d_tip, cy + dy * d_tip)
            in_end = (cx + dx * d_in_end, cy + dy * d_in_end)
            out_start = (cx + dx * d_out_start, cy + dy * d_out_start)
            out_end = (cx + dx * d_out_end, cy + dy * d_out_end)
            cv2.line(frame, tip, in_end, color, XHAIR_THICK, cv2.LINE_AA)     # inner segment
            cv2.line(frame, out_start, out_end, color, XHAIR_THICK, cv2.LINE_AA)  # outer segment (after the break)
            # 'T' cap: short perpendicular line centered on the inner tip.
            t1 = (tip[0] - px * XHAIR_T_HALF, tip[1] - py * XHAIR_T_HALF)
            t2 = (tip[0] + px * XHAIR_T_HALF, tip[1] + py * XHAIR_T_HALF)
            cv2.line(frame, t1, t2, color, XHAIR_THICK, cv2.LINE_AA)

    def _draw_boresight(self, frame, color):
        # Center reticle (War Thunder style) + corner frame brackets.
        self._draw_center_crosshair(frame, color)
        h, w = frame.shape[:2]
        # Corner frame brackets for a turret feel.
        b = 28
        for (x, y, dx, dy) in [(8, 8, 1, 1), (w - 8, 8, -1, 1), (8, h - 8, 1, -1), (w - 8, h - 8, -1, -1)]:
            cv2.line(frame, (x, y), (x + dx * b, y), color, 1, cv2.LINE_AA)
            cv2.line(frame, (x, y), (x, y + dy * b), color, 1, cv2.LINE_AA)

    def _draw_direction(self, frame, u, v, color):
        """Small circle placed just OUTSIDE the box at its lower-right corner (circle bottom
        aligned with the box bottom), with a FIXED-length line pointing along the drone's
        travel direction (magnitude ignored). Direction comes from the projected pixel motion
        (about the target center u,v), so perspective is handled. Circle only when still."""
        ccx = u + BOX_HALF + DIR_BOX_GAP + DIR_CIRCLE_R   # sits to the right of the box
        ccy = v + BOX_HALF - DIR_CIRCLE_R                 # circle bottom == box bottom edge
        cv2.circle(frame, (ccx, ccy), DIR_CIRCLE_R, color, DIR_THICK, cv2.LINE_AA)
        if self.target_point is None or self.target_velocity is None:
            return
        # Speed gate: a near-stationary drone has a pure-noise velocity, so its DIRECTION is
        # meaningless and spins. Below the threshold, show only the circle (no heading line).
        speed = (self.target_velocity.x ** 2 + self.target_velocity.y ** 2) ** 0.5
        if speed < DIR_MIN_SPEED_MPS:
            self._dir_ema = None
            return
        ahead = self._project(
            self.target_point.x + self.target_velocity.x * DIR_LOOKAHEAD_S,
            self.target_point.y + self.target_velocity.y * DIR_LOOKAHEAD_S,
            self.target_point.z + self.target_velocity.z * DIR_LOOKAHEAD_S,
        )
        if ahead is None:
            return
        dx, dy = ahead[0] - u, ahead[1] - v
        mag = (dx * dx + dy * dy) ** 0.5
        if mag < 1e-3:
            return  # not moving -> just the circle
        ux, uy = dx / mag, dy / mag  # unit direction; length is FIXED below, not from speed
        # EMA-smooth the heading so it settles instead of twitching frame to frame.
        if self._dir_ema is None:
            self._dir_ema = (ux, uy)
        else:
            ex = DIR_EMA_ALPHA * ux + (1.0 - DIR_EMA_ALPHA) * self._dir_ema[0]
            ey = DIR_EMA_ALPHA * uy + (1.0 - DIR_EMA_ALPHA) * self._dir_ema[1]
            n = (ex * ex + ey * ey) ** 0.5 or 1.0
            self._dir_ema = (ex / n, ey / n)
        ux, uy = self._dir_ema
        start = (int(ccx + ux * DIR_CIRCLE_R), int(ccy + uy * DIR_CIRCLE_R))
        end = (int(ccx + ux * (DIR_CIRCLE_R + DIR_LINE_LEN)), int(ccy + uy * (DIR_CIRCLE_R + DIR_LINE_LEN)))
        cv2.line(frame, start, end, color, DIR_THICK, cv2.LINE_AA)

    def _draw_target(self, frame, u, v, color):
        """Target box over the locked drone: a square with a break in the middle of each
        edge (thick), four thin lines from those breaks out to the frame edges, a
        direction indicator inside, and the range printed beside it."""
        h, w = frame.shape[:2]
        half = BOX_HALF
        g = BOX_EDGE_GAP // 2   # half of the center-break width

        # Square box with a centered break in each edge (thickness BOX_THICK).
        for ey in (v - half, v + half):                 # top & bottom edges (horizontal)
            cv2.line(frame, (u - half, ey), (u - g, ey), color, BOX_THICK, cv2.LINE_AA)
            cv2.line(frame, (u + g, ey), (u + half, ey), color, BOX_THICK, cv2.LINE_AA)
        for ex in (u - half, u + half):                 # left & right edges (vertical)
            cv2.line(frame, (ex, v - half), (ex, v - g), color, BOX_THICK, cv2.LINE_AA)
            cv2.line(frame, (ex, v + g), (ex, v + half), color, BOX_THICK, cv2.LINE_AA)

        # A thin line from each edge-break out to the frame edge (thickness TARGET_LINE_THICK).
        cv2.line(frame, (u, v - half), (u, 0), color, TARGET_LINE_THICK, cv2.LINE_AA)  # up
        cv2.line(frame, (u, v + half), (u, h), color, TARGET_LINE_THICK, cv2.LINE_AA)  # down
        cv2.line(frame, (u - half, v), (0, v), color, TARGET_LINE_THICK, cv2.LINE_AA)  # left
        cv2.line(frame, (u + half, v), (w, v), color, TARGET_LINE_THICK, cv2.LINE_AA)  # right

        # Direction indicator (circle + fixed-length heading line) inside the box.
        self._draw_direction(frame, u, v, color)

        # Range (depth) readout beside the box, war-thunder style.
        if self.target_point is not None:
            cv2.putText(frame, f'{self.target_point.z:.2f} m', (u + half + 8, v - half + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    def _draw_header(self, frame, color, target_xy):
        """Top overlay: 'OREX' centered, and a SEEKER DATA / TARGET XY block top-left.
        Same font/size/color/style as the rest of the HUD."""
        font, scale, thick = cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
        w = frame.shape[1]
        # # Center-top title.
        # title = 'OREX SYSTEMS'
        # (tw, _th), _ = cv2.getTextSize(title, font, scale, thick)
        # cv2.putText(frame, title, ((w - tw) // 2, 28), font, scale, color, thick, cv2.LINE_AA)
        # Top-left seeker data block.
        cv2.putText(frame, 'SEEKER DATA', (14, 28), font, scale, color, thick, cv2.LINE_AA)
        xy = f'TARGET XY: {target_xy[0]}, {target_xy[1]}' if target_xy is not None else 'TARGET XY: ---, ---'
        cv2.putText(frame, xy, (14, 50), font, scale, color, thick, cv2.LINE_AA)

    def _draw_command_bar(self, frame, color, status):
        """Bottom command/legend row (echoes the reference seeker HUD)."""
        h, w = frame.shape[:2]
        mode = TINT_MODES[self.tint_idx][0]
        left = f'SENSOR MODE: {status}   TINT: {mode}'
        cv2.putText(frame, left, (14, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        keys = '[M] TINT   [Q] QUIT'
        (kw, _kh), _ = cv2.getTextSize(keys, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.putText(frame, keys, (w - kw - 14, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    def _have_fresh_image(self):
        if self.latest_frame is None or self.last_image_time is None:
            return False
        age_s = (self.get_clock().now() - self.last_image_time).nanoseconds / 1e9
        return age_s <= IMAGE_STALE_S

    def _placeholder(self):
        """Dark frame used before/without a camera feed, sized to the last frame if any."""
        h, w = (self.latest_frame.shape[:2] if self.latest_frame is not None else (480, 640))
        frame = np.full((h, w, 3), 22, dtype=np.uint8)
        msg = 'WAITING FOR CAMERA'
        (tw, th), _ = cv2.getTextSize(msg, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
        cv2.putText(frame, msg, ((w - tw) // 2, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                    SYMBOL_GREEN, 2, cv2.LINE_AA)
        hint = 'no /camera/camera/color/image_raw'
        (hw, _hh), _ = cv2.getTextSize(hint, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.putText(frame, hint, ((w - hw) // 2, h // 2 + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    SYMBOL_GREEN, 1, cv2.LINE_AA)
        return frame

    def _render(self):
        have_image = self._have_fresh_image()
        frame = self._apply_tint(self.latest_frame.copy()) if have_image else self._placeholder()
        colors = self._colors()

        self._draw_boresight(frame, colors['struct'])

        target_xy = None
        if not have_image:
            status = 'NO SIGNAL'
        else:
            locked = self.target_locked and self._track_is_fresh()
            status = 'SEARCHING'
            if locked:
                px = self._project(self.target_point.x, self.target_point.y, self.target_point.z)
                if px is not None:
                    u, v = px
                    target_xy = (u, v)
                    self._draw_target(frame, u, v, colors['locked'])
                    status = 'LOCKED'

        self._draw_header(frame, colors['struct'], target_xy)
        # Status now lives only in the bottom command bar (no redundant top-left label).
        self._draw_command_bar(frame, colors['struct'], status)

        cv2.imshow(WINDOW_NAME, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):
            self.get_logger().info('Close requested; shutting down turret_view')
            raise KeyboardInterrupt
        if key == ord('m'):
            self.tint_idx = (self.tint_idx + 1) % len(TINT_MODES)
            self.get_logger().info(f'Sensor tint -> {TINT_MODES[self.tint_idx][0]}')


def main(args=None):
    rclpy.init(args=args)
    node = turret_view()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
