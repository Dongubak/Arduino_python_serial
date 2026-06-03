#!/usr/bin/env python3
"""
Lane detector node for Gunsan autonomous driving.

Subscribes to a camera image topic, applies:
  1. Lens undistortion (from camera_info)
  2. Trapezoid ROI mask
  3. Bird's-eye (IPM) perspective warp
  4. Canny edge detection + HoughLinesP
  5. Left / right lane fitting and center-error calculation

Publishes:
  /lane_error   (std_msgs/Float32)  — normalized cross-track error: -1.0 (left) ~ +1.0 (right)
  /lane_debug   (sensor_msgs/Image) — side-by-side: original+ROI | bird's-eye+lines

ROI trapezoid (all in pixels, default tuned for 1280×720):
  Adjust the six roi_* parameters until the trapezoid tightly frames the road
  ahead of the car bonnet in the original image.

        roi_top_left_x        roi_top_right_x
               ┌──────────────────┐   ← roi_top_y
              /                    \
             /                      \
            └────────────────────────┘  ← roi_bottom_y
    roi_bottom_left_x          roi_bottom_right_x
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32

import cv2
import numpy as np
from cv_bridge import CvBridge


class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector')

        # ── camera topic ────────────────────────────────────────────────────
        self.declare_parameter('camera_topic', '/camera5/image_raw')
        cam_topic = self.get_parameter('camera_topic').value

        # ── ROI trapezoid (pixels, default for 1280×720) ─────────────────
        # Top edge of trapezoid (near horizon)
        self.declare_parameter('roi_top_y',          330)
        self.declare_parameter('roi_top_left_x',     570)
        self.declare_parameter('roi_top_right_x',    710)
        # Bottom edge of trapezoid (near bonnet)
        self.declare_parameter('roi_bottom_y',       610)
        self.declare_parameter('roi_bottom_left_x',   500)
        self.declare_parameter('roi_bottom_right_x', 780)

        # ── Heading reference offset (pixels, in bird's-eye space) ──────────
        # 0 = image center is the car heading (valid after physical alignment).
        # Set to non-zero only if residual misalignment remains after physical
        # bonnet-line alignment (positive = heading is right of image center).
        self.declare_parameter('heading_offset_x',    0)

        # ── Lane width in bird's-eye pixels ──────────────────────────────────
        # Used only when a single lane line is detected, to estimate lane center.
        # Measure in the bird's-eye debug image: pixel distance between the two
        # lane lines when both are visible.  Default 280 px is a rough estimate.
        self.declare_parameter('lane_width_px',     280)

        # ── Canny / Hough parameters ─────────────────────────────────────
        self.declare_parameter('canny_low',          50)
        self.declare_parameter('canny_high',        150)
        self.declare_parameter('hough_threshold',    30)
        self.declare_parameter('hough_min_length',   50)
        self.declare_parameter('hough_max_gap',     100)

        # ── internal state ────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.K: np.ndarray | None = None   # camera matrix 3×3
        self.D: np.ndarray | None = None   # distortion coeffs

        # ── subscriptions ─────────────────────────────────────────────────
        info_topic = cam_topic.replace('image_raw', 'camera_info')
        self.create_subscription(CameraInfo, info_topic,
                                 self._camera_info_cb, qos_profile=1)
        self.create_subscription(Image, cam_topic,
                                 self._image_cb, qos_profile=10)

        # ── publishers ────────────────────────────────────────────────────
        # ── debug image publish rate ─────────────────────────────────────────
        # 디버그 이미지는 튜닝 시에만 필요. 0 = 끔, N = N프레임마다 1회 발행
        self.declare_parameter('debug_every_n_frames', 10)

        self.error_pub   = self.create_publisher(Float32, '/lane_error',   10)
        self.heading_pub = self.create_publisher(Float32, '/lane_heading', 10)
        self.debug_pub   = self.create_publisher(Image,   '/lane_debug',   10)
        self._frame_count = 0

        self.get_logger().info(f'LaneDetector ready  |  camera: {cam_topic}')
        self.get_logger().info(
            'Topics: /lane_error (lateral), /lane_heading (deg, +right/-left)')

    # ── callbacks ─────────────────────────────────────────────────────────

    def _camera_info_cb(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('Camera intrinsics received and applied.')

    def _image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # 1. undistort
        if self.K is not None:
            frame = cv2.undistort(frame, self.K, self.D)

        h, w = frame.shape[:2]

        # 2. build ROI source trapezoid from parameters
        src = self._get_src_pts(w, h)

        # 3. perspective warp → bird's-eye
        dst = np.float32([[100, 0], [w - 100, 0],
                           [w - 100, h], [100, h]])
        M   = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, src)
        warped = cv2.warpPerspective(frame, M, (w, h))

        # 4. edge detection in bird's-eye view
        # Mask out the left/right border columns (warpPerspective fills them with
        # black=0, creating sharp edges that Canny detects as false lane lines).
        warp_margin = 100   # must be >= dst x-offset (100) used in getPerspectiveTransform
        mask = np.zeros(warped.shape[:2], dtype=np.uint8)
        mask[:, warp_margin: w - warp_margin] = 255
        warped_masked = cv2.bitwise_and(warped, warped, mask=mask)

        gray  = cv2.cvtColor(warped_masked, cv2.COLOR_BGR2GRAY)
        blur  = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(
            blur,
            self.get_parameter('canny_low').value,
            self.get_parameter('canny_high').value)

        # 5. Hough line detection
        lines = cv2.HoughLinesP(
            edges,
            rho=1, theta=np.pi / 180,
            threshold=self.get_parameter('hough_threshold').value,
            minLineLength=self.get_parameter('hough_min_length').value,
            maxLineGap=self.get_parameter('hough_max_gap').value)

        # 6. lane-center error + heading angle
        offset = self.get_parameter('heading_offset_x').value
        lane_error, heading_deg, left_fit, right_fit = self._calc_error(
            lines, w, h, offset)

        # 7. publish
        err_msg = Float32()
        err_msg.data = float(lane_error)
        self.error_pub.publish(err_msg)

        hdg_msg = Float32()
        hdg_msg.data = float(heading_deg)
        self.heading_pub.publish(hdg_msg)

        # 8. debug image — N 프레임마다 1회만 발행 (CPU 부하 절감)
        every_n = self.get_parameter('debug_every_n_frames').value
        self._frame_count += 1
        if every_n > 0 and self._frame_count % every_n == 0:
            debug = self._make_debug(frame, warped, src, lines,
                                     left_fit, right_fit, lane_error, heading_deg,
                                     w, h, offset)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, 'bgr8'))

    # ── helpers ───────────────────────────────────────────────────────────

    def _get_src_pts(self, w: int, h: int) -> np.ndarray:
        tl_x = self.get_parameter('roi_top_left_x').value
        tr_x = self.get_parameter('roi_top_right_x').value
        t_y  = self.get_parameter('roi_top_y').value
        bl_x = self.get_parameter('roi_bottom_left_x').value
        br_x = self.get_parameter('roi_bottom_right_x').value
        b_y  = self.get_parameter('roi_bottom_y').value
        return np.float32([[tl_x, t_y], [tr_x, t_y],
                            [br_x, b_y], [bl_x, b_y]])

    def _calc_error(self, lines, w: int, h: int, heading_offset: int = 0):
        """Separate lines into left/right by slope+position, fit each side.

        heading_offset: pixel offset from image center representing the car's
        true heading direction (set after physical bonnet-line alignment).
        Positive = heading is to the right of image center.
        """
        half_w = w / 2.0
        ref_x  = half_w + heading_offset   # actual heading reference in bird's-eye x

        left_pts, right_pts = [], []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x2 == x1:
                    continue
                slope = (y2 - y1) / (x2 - x1)
                if abs(slope) < 0.3:          # ignore near-horizontal lines
                    continue
                avg_x = (x1 + x2) / 2.0
                avg_y = (y1 + y2) / 2.0
                if slope < 0 and avg_x < ref_x:   # left lane (negative slope in image)
                    left_pts.append((avg_x, avg_y, x1, y1, x2, y2))
                elif slope > 0 and avg_x > ref_x:  # right lane
                    right_pts.append((avg_x, avg_y, x1, y1, x2, y2))

        left_fit  = self._fit_line(left_pts,  h)
        right_fit = self._fit_line(right_pts, h)

        # heading angle: deviation of lane direction from vertical in bird's-eye.
        # fit coefficient 'a' in x = a*y + b  →  a = dx/dy (tan of angle from vertical)
        # positive a → line leans right → car points left of lane  → positive heading error
        # negative a → line leans left  → car points right of lane → negative heading error
        slopes = []
        if left_fit  is not None: slopes.append(left_fit[0])
        if right_fit is not None: slopes.append(right_fit[0])
        if slopes:
            avg_slope   = float(np.mean(slopes))
            heading_deg = float(np.degrees(np.arctan(avg_slope)))
        else:
            heading_deg = 0.0

        # lateral cross-track error
        bottom_y  = h - 1
        half_lane = self.get_parameter('lane_width_px').value / 2.0
        if left_fit is not None and right_fit is not None:
            # both lanes: true center
            lx = self._x_at_y(left_fit,  bottom_y)
            rx = self._x_at_y(right_fit, bottom_y)
            center = (lx + rx) / 2.0
        elif left_fit is not None:
            # only left lane: estimate center by adding half lane width
            lx = self._x_at_y(left_fit, bottom_y)
            center = lx + half_lane
        elif right_fit is not None:
            # only right lane: estimate center by subtracting half lane width
            rx = self._x_at_y(right_fit, bottom_y)
            center = rx - half_lane
        else:
            return 0.0, 0.0, None, None

        lateral_error = (center - ref_x) / half_w   # -1.0 ~ +1.0
        return float(np.clip(lateral_error, -1.0, 1.0)), heading_deg, left_fit, right_fit

    def _fit_line(self, pts, h: int):
        if len(pts) < 2:
            return None
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        try:
            coeffs = np.polyfit(ys, xs, 1)   # x = a*y + b
            return coeffs
        except Exception:
            return None

    def _x_at_y(self, coeffs, y: float) -> float:
        return float(coeffs[0] * y + coeffs[1])

    def _make_debug(self, original, warped, src_pts,
                    lines, left_fit, right_fit, error, heading_deg, w, h,
                    heading_offset: int = 0):
        left_vis  = original.copy()
        right_vis = warped.copy()

        img_cx  = w // 2                      # image center (gray)
        head_cx = img_cx + heading_offset      # heading reference (yellow)

        # --- left panel: original image ---
        # ROI trapezoid
        pts = src_pts.astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(left_vis, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
        # image center (gray, for reference during alignment)
        cv2.line(left_vis, (img_cx, 0), (img_cx, h), (160, 160, 160), 1)
        # heading reference line (yellow) — align bonnet mark to this line
        cv2.line(left_vis, (head_cx, 0), (head_cx, h), (0, 220, 220), 2)
        cv2.putText(left_vis, 'HEADING', (head_cx + 5, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 220), 2)

        # --- right panel: bird's-eye view ---
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(right_vis, (x1, y1), (x2, y2), (0, 0, 255), 1)

        ys = np.linspace(0, h - 1, 50)
        if left_fit is not None:
            pts_l = np.int32([[self._x_at_y(left_fit, y), y] for y in ys])
            cv2.polylines(right_vis, [pts_l.reshape(-1, 1, 2)],
                          False, (255, 100, 0), 3)
        if right_fit is not None:
            pts_r = np.int32([[self._x_at_y(right_fit, y), y] for y in ys])
            cv2.polylines(right_vis, [pts_r.reshape(-1, 1, 2)],
                          False, (0, 165, 255), 3)

        # image center (gray) and heading reference (yellow) in bird's-eye too
        cv2.line(right_vis, (img_cx, 0),  (img_cx, h),  (160, 160, 160), 1)
        cv2.line(right_vis, (head_cx, 0), (head_cx, h), (0, 220, 220), 2)

        # heading direction arrow in bird's-eye (shows lane tilt at image center)
        if left_fit is not None or right_fit is not None:
            slope = (left_fit[0] if left_fit is not None else 0.0)
            if right_fit is not None:
                slope = (slope + right_fit[0]) / (2.0 if left_fit is not None else 1.0)
            arrow_len = h // 3
            cx_arrow  = head_cx
            y_mid     = h // 2
            dx = int(slope * arrow_len)
            # arrow points in lane direction (from bottom to top in bird's-eye)
            cv2.arrowedLine(right_vis,
                            (cx_arrow + dx, y_mid + arrow_len // 2),
                            (cx_arrow - dx, y_mid - arrow_len // 2),
                            (0, 255, 0), 3, tipLength=0.3)

        # text: lateral error + heading angle
        txt1 = f'lateral: {error:+.3f}  heading: {heading_deg:+.1f}deg'
        txt2 = f'offset: {heading_offset:+d}px'
        cv2.putText(left_vis,  txt1, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(left_vis,  txt2, (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(right_vis, txt1, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        # resize to half for side-by-side display
        half_h = h // 2
        half_w = w // 2
        l = cv2.resize(left_vis,  (half_w, half_h))
        r = cv2.resize(right_vis, (half_w, half_h))
        return np.hstack([l, r])


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
