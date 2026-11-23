#!/usr/bin/env python3
"""Standalone TrackerVit test — isolates the tracker from YOLO / Kalman / head.

Draw a box around the drone (or anything) and TrackerVit follows it every frame with NO YOLO
involved. This answers one question: **is TrackerVit good enough on its own?**
  - Holds a clean manual box well  -> the tracker is solid; YOLO's flaky re-validation was the
    problem (matches the "Tracker validation miss" drops we saw).
  - Drifts/loses even from a clean box -> the tracker itself is the weak link (tune the init
    box size / try a different target scale).

Keys:  s = select / re-select target (drag a box, then ENTER or SPACE)
       r = clear the tracker      q / ESC = quit

Run under the venv that has OpenCV 5 + the vittrack model (camera must be streaming):
    source /opt/ros/jazzy/setup.bash && source .venv/bin/activate && source install/setup.bash
    python3 src/orex_head_vision/utils/trackervit_test.py
If the window doesn't appear on WSL:  QT_QPA_PLATFORM=xcb python3 ...
"""
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Same model the vision node uses (orex_head_vision/models/vittrack.onnx).
MODEL_PATH = os.path.join(os.path.dirname(__file__), 'models', 'vittrack.onnx')
WINDOW = 'TrackerVit test  (s=select  r=clear  q=quit)'


class TrackerVitTest(Node):
    def __init__(self):
        super().__init__('trackervit_test')
        self.bridge = CvBridge()
        self.frame = None
        self.tracker = None
        self.lost_frames = 0

        if not hasattr(cv2, 'TrackerVit_create'):
            self.get_logger().error('cv2.TrackerVit_create missing — run under the OpenCV-5 venv.')
        if not os.path.isfile(MODEL_PATH):
            self.get_logger().error(f'Model not found: {os.path.abspath(MODEL_PATH)}')

        self.create_subscription(
            Image, '/camera/camera/color/image_raw', self._image, qos_profile_sensor_data)
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        self.create_timer(1.0 / 30.0, self._render)
        self.get_logger().info('Ready. Press "s" and drag a box around the drone.')

    def _image(self, msg: Image):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'convert failed: {exc}')

    def _new_tracker(self):
        params = cv2.TrackerVit_Params()
        params.net = MODEL_PATH
        return cv2.TrackerVit_create(params)

    def _render(self):
        if self.frame is None:
            return
        disp = self.frame.copy()

        if self.tracker is not None:
            ok, box = self.tracker.update(self.frame)
            try:
                score = float(self.tracker.getTrackingScore())
            except Exception:
                score = -1.0
            x, y, w, h = (int(v) for v in box)
            self.lost_frames = 0 if ok else self.lost_frames + 1
            color = (0, 255, 0) if ok else (0, 0, 255)
            cv2.rectangle(disp, (x, y), (x + w, y + h), color, 2)
            cv2.circle(disp, (x + w // 2, y + h // 2), 3, color, -1)
            cv2.putText(disp, f'score={score:.2f}  ok={ok}  box={w}x{h}px  lost={self.lost_frames}',
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
        else:
            cv2.putText(disp, 'press  s  and drag a box around the target',
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (60, 200, 255), 2, cv2.LINE_AA)

        cv2.imshow(WINDOW, disp)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):
            raise KeyboardInterrupt
        if key == ord('r'):
            self.tracker = None
            self.get_logger().info('Tracker cleared.')
        if key == ord('s') and self.frame is not None and hasattr(cv2, 'TrackerVit_create'):
            # Modal box selection (pauses the stream on the current frame).
            roi = cv2.selectROI(WINDOW, self.frame, showCrosshair=True, fromCenter=False)
            if roi[2] > 0 and roi[3] > 0:
                self.tracker = self._new_tracker()
                self.tracker.init(self.frame, tuple(int(v) for v in roi))
                self.lost_frames = 0
                self.get_logger().info(f'Tracking box={roi}')


def main():
    rclpy.init()
    node = TrackerVitTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
