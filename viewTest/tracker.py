"""
Overhead ArUco Tracker
Tracks robots using pixel positions — no camera calibration required.
Coordinate system: origin at screen center, X right, Y up (cartesian).
Units: millimetres, derived from a one-time real-world field-of-view measurement.
"""

import cv2
import numpy as np
import time
import requests
from threading import Thread
from webcamvideostream import WebcamVideoStream


# ---------------------------------------------------------------------------
# Configuration — edit these values to match your setup
# ---------------------------------------------------------------------------

# Camera source index
CAMERA_SRC = 1

# Capture resolution
FRAME_WIDTH  = 1920
FRAME_HEIGHT = 1080

# Target framerate
FPS = 30

# Manual camera focus (0 = far, 255 = near; set 0 if shooting from far above)
FOCUS = 0

# Real-world size of the camera's field of view, in millimetres.
# Measure the width and height of the rectangle the camera sees on the ground.
FOV_WIDTH_MM  = 3600   # <-- replace with your measured value
FOV_HEIGHT_MM = 2030    # <-- replace with your measured value

# ArUco dictionary — 4x4_50 gives up to 50 unique markers with a simple pattern
ARUCO_DICT_NAME = cv2.aruco.DICT_4X4_50

# Marker IDs that belong to robots (add or remove as needed)
ROBOT_IDS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

# Grid line spacing in mm (drawn over the video feed)
GRID_SPACING_MM = 100.0

# Server address for PUT requests (stub — replace when server is ready)
SERVER_ADDRESS = "http://localhost:5000/"

# PUT rate in Hz
PUT_RATE_HZ = 20

# ---------------------------------------------------------------------------


class Tracker:

    def __init__(self):
        # Pixels per mm scale factors
        self.px_per_mm_x = FRAME_WIDTH  / FOV_WIDTH_MM
        self.px_per_mm_y = FRAME_HEIGHT / FOV_HEIGHT_MM

        # ArUco setup
        self.aruco_dict   = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_NAME)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        # Robot state: {id: {"x_mm": float, "y_mm": float, "heading_rad": float}}
        self.robots = {}

        self.stop_flag = False
        self.out_frame = None

        # FPS tracking
        self._t_prev = time.perf_counter()

    # ------------------------------------------------------------------
    # Coordinate conversion helpers
    # ------------------------------------------------------------------

    def pixel_to_mm(self, px, py):
        """
        Convert pixel coordinates to millimetres with origin at frame centre.
        Y is flipped so that up on screen = positive Y (cartesian convention).
        """
        cx = FRAME_WIDTH  / 2.0
        cy = FRAME_HEIGHT / 2.0
        x_mm =  (px - cx) / self.px_per_mm_x
        y_mm = -(py - cy) / self.px_per_mm_y   # flip Y
        return x_mm, y_mm

    def mm_to_pixel(self, x_mm, y_mm):
        """Inverse of pixel_to_mm — used for drawing the grid."""
        cx = FRAME_WIDTH  / 2.0
        cy = FRAME_HEIGHT / 2.0
        px = int( x_mm * self.px_per_mm_x + cx)
        py = int(-y_mm * self.px_per_mm_y + cy)   # flip Y back
        return px, py

    # ------------------------------------------------------------------
    # Heading
    # ------------------------------------------------------------------

    def marker_heading(self, corners):
        """
        Compute heading angle (radians) from a single marker's corners.
        The heading is the direction the top-edge of the marker points,
        mapped into cartesian convention (positive Y = up).
        Returns angle in [-pi, pi].
        """
        pts = corners[0]          # shape (4, 2): TL, TR, BR, BL
        top_mid    = (pts[0] + pts[1]) / 2.0
        bottom_mid = (pts[3] + pts[2]) / 2.0
        dx =  (top_mid[0] - bottom_mid[0])
        dy = -(top_mid[1] - bottom_mid[1])  # flip Y
        return np.arctan2(dy, dx)

    # ------------------------------------------------------------------
    # Frame processing
    # ------------------------------------------------------------------

    def process_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id not in ROBOT_IDS:
                    continue

                c = corners[i]                          # shape (1, 4, 2)
                centre_px = c[0].mean(axis=0)           # (x, y) in pixels
                x_mm, y_mm = self.pixel_to_mm(centre_px[0], centre_px[1])
                heading    = self.marker_heading(c)

                self.robots[int(marker_id)] = {
                    "x_mm":       round(x_mm,   2),
                    "y_mm":       round(y_mm,   2),
                    "heading_rad": round(heading, 4),
                }

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            self._draw_robot_labels(frame)

        self._draw_grid(frame)
        self._draw_origin(frame)
        self._draw_fps(frame)
        return frame

    # ------------------------------------------------------------------
    # Drawing helpers
    # ------------------------------------------------------------------

    def _draw_grid(self, frame):
        """Draw mm-spaced grid lines centred on the frame."""
        # Vertical lines
        x = 0.0
        while True:
            for sign in (1, -1):
                px, _ = self.mm_to_pixel(sign * x, 0)
                if 0 <= px <= FRAME_WIDTH:
                    color = (60, 60, 60) if x != 0 else (100, 100, 100)
                    cv2.line(frame, (px, 0), (px, FRAME_HEIGHT), color, 1)
                    if x != 0:
                        label = f"{int(sign * x)}"
                        cv2.putText(frame, label, (px + 2, FRAME_HEIGHT - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1)
            if sign == 1 and px > FRAME_WIDTH:
                break
            x += GRID_SPACING_MM
            if x > FOV_WIDTH_MM:
                break

        # Horizontal lines
        y = 0.0
        while True:
            for sign in (1, -1):
                _, py = self.mm_to_pixel(0, sign * y)
                if 0 <= py <= FRAME_HEIGHT:
                    color = (60, 60, 60) if y != 0 else (100, 100, 100)
                    cv2.line(frame, (0, py), (FRAME_WIDTH, py), color, 1)
                    if y != 0:
                        label = f"{int(sign * y)}"
                        cv2.putText(frame, label, (5, py - 3),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1)
            if sign == 1 and py < 0:
                break
            y += GRID_SPACING_MM
            if y > FOV_HEIGHT_MM:
                break

    def _draw_origin(self, frame):
        """Draw a small crosshair at the origin (screen centre)."""
        cx, cy = FRAME_WIDTH // 2, FRAME_HEIGHT // 2
        cv2.line(frame, (cx - 10, cy), (cx + 10, cy), (0, 255, 255), 1)
        cv2.line(frame, (cx, cy - 10), (cx, cy + 10), (0, 255, 255), 1)
        cv2.putText(frame, "(0,0)", (cx + 5, cy - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

    def _draw_robot_labels(self, frame):
        """Draw ID, position, and heading arrow for each tracked robot."""
        for rid, state in self.robots.items():
            px, py = self.mm_to_pixel(state["x_mm"], state["y_mm"])

            # Heading arrow
            arrow_len = 30  # pixels
            h = state["heading_rad"]
            ex = int(px + arrow_len * np.cos(h))
            ey = int(py - arrow_len * np.sin(h))   # flip Y for screen
            cv2.arrowedLine(frame, (px, py), (ex, ey), (0, 255, 0), 2, tipLength=0.3)

            # Label
            label = (f"ID:{rid}  "
                     f"({state['x_mm']:.0f}, {state['y_mm']:.0f}) mm  "
                     f"{np.degrees(state['heading_rad']):.1f}deg")
            cv2.putText(frame, label, (px + 5, py - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)

    def _draw_fps(self, frame):
        now = time.perf_counter()
        dt = now - self._t_prev
        self._t_prev = now
        if dt > 0:
            cv2.putText(frame, f"FPS: {1/dt:.1f}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # ------------------------------------------------------------------
    # Threads
    # ------------------------------------------------------------------

    def start(self):
        self.stop_flag = False
        self.vs = WebcamVideoStream(
            src=CAMERA_SRC,
            width=FRAME_WIDTH,
            height=FRAME_HEIGHT,
            fps=FPS,
            focus=FOCUS
        ).start()
        self.out_frame = self.vs.frame

        Thread(target=self._process_loop,  daemon=False).start()
        Thread(target=self._display_loop,  daemon=False).start()
        Thread(target=self._put_loop,      daemon=False).start()
        return self

    def stop(self):
        self.stop_flag = True
        self.vs.stop()
        self.vs.stream.release()
        cv2.destroyAllWindows()

    def _process_loop(self):
        while not self.stop_flag:
            if self.vs.grabbed:
                self.out_frame = self.process_frame(self.vs.frame.copy())

    def _display_loop(self):
        frame_delta = 1.0 / FPS
        cv2.namedWindow("Overhead Tracker", cv2.WINDOW_NORMAL)  # add this
        while not self.stop_flag:
            prev = time.time()
            if self.out_frame is not None:
                cv2.imshow("Overhead Tracker", self.out_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.stop()
                break
            sleep = frame_delta - (time.time() - prev)
            time.sleep(max(0.0, sleep))

    def _put_loop(self):
        """
        Send robot positions to the server at PUT_RATE_HZ.
        Payload: {"robots": [{id, x_mm, y_mm, heading_rad}, ...]}
        Swap this out when the UDP broadcast is implemented.
        """
        interval = 1.0 / PUT_RATE_HZ
        prev = time.time()
        while not self.stop_flag:
            if (time.time() - prev) >= interval:
                prev = time.time()
                payload = {
                    "robots": [
                        {"id": rid, **state}
                        for rid, state in self.robots.items()
                    ]
                }
                try:
                    requests.put(SERVER_ADDRESS + "allPos", json=payload, timeout=0.1)
                except requests.exceptions.RequestException:
                    pass  # server not running yet — silently skip


# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print("Starting overhead tracker. Press 'q' to quit.")
    print(f"FOV: {FOV_WIDTH_MM} x {FOV_HEIGHT_MM} mm")
    print(f"Scale: {1920/FOV_WIDTH_MM:.3f} px/mm  x  {1080/FOV_HEIGHT_MM:.3f} px/mm")
    tracker = Tracker()
    tracker.start()
