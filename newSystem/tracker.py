"""
tracker.py - Overhead Camera Tracker for Robot Swarm Localisation
Detects ArUco markers (4x4_50 dictionary) from a 1920x1080 camera feed,
converts pixel positions to real-world coordinates centred at (0, 0),
and pushes (x, y, theta) data to the UDP server.
"""

import cv2
import numpy as np
import math
import time
import platform
from threading import Thread
from udp import UDPServer

# ── Camera / Scene Configuration ──────────────────────────────────────────────
CAMERA_INDEX      = 1          # OpenCV camera index
FRAME_WIDTH       = 1920       # Camera feed width  (pixels)
FRAME_HEIGHT      = 1080       # Camera feed height (pixels)
FPS               = 30

# Physical dimensions of the area visible to the camera (metres).
# Measure the floor region that fills the camera frame at the mounted height.
SCENE_WIDTH_M     = 3.60        # Real-world width  the camera sees (metres)
SCENE_HEIGHT_M    = 2.03      # Real-world height the camera sees (metres)
#   Pixel → metre scale factors
PX_PER_M_X = FRAME_WIDTH  / SCENE_WIDTH_M    # pixels per metre (horizontal)
PX_PER_M_Y = FRAME_HEIGHT / SCENE_HEIGHT_M   # pixels per metre (vertical)

# ArUco marker side length in metres (used for pose estimation).
MARKER_LENGTH_M   = 0.16       # 5 cm markers

# Centre pixel (maps to world origin (0, 0))
CX = FRAME_WIDTH  // 2         # 960
CY = FRAME_HEIGHT // 2         # 540
# ─────────────────────────────────────────────────────────────────────────────


def pixel_to_world(px: float, py: float) -> tuple[float, float]:
    """
    Convert pixel coordinates to world coordinates in metres.
    Centre pixel (CX, CY) maps to (0.0, 0.0).
    X increases rightward; Y increases upward (inverted from image convention).
    """
    x_m = (px - CX) / PX_PER_M_X
    y_m = -(py - CY) / PX_PER_M_Y   # invert Y so up = positive
    return x_m, y_m


def marker_heading(corners) -> float:
    """
    Compute heading from the top edge of the printed ArUco marker.
    'Top' is the edge between corner 0 (TL) and corner 1 (TR) as the
    marker appears when printed and viewed on screen.
    Returns angle in radians in [-pi, pi], cartesian convention (CCW positive).
    """
    pts = corners[0]                        # TL, TR, BR, BL
    top_mid    = (pts[0] + pts[1]) / 2.0
    bottom_mid = (pts[3] + pts[2]) / 2.0
    dx =  (top_mid[0] - bottom_mid[0])
    dy = -(top_mid[1] - bottom_mid[1])     # flip Y for cartesian
    return math.atan2(dy, dx)


class WebcamVideoStream:
    """
    Threaded camera capture — opens the C922 at full 1920x1080 on Windows
    using CAP_DSHOW + MJPG, which is the only reliable way to get 1080p
    out of this camera via OpenCV on Windows.
    """
    def __init__(self, src=1, width=1920, height=1080, fps=30, focus=0):
        if platform.system() == "Windows":
            self.stream = cv2.VideoCapture(src, cv2.CAP_DSHOW)
        else:
            self.stream = cv2.VideoCapture(src)

        self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.stream.set(cv2.CAP_PROP_FPS,          fps)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE,   1)
        self.stream.set(cv2.CAP_PROP_AUTOFOCUS,    0)
        self.stream.set(cv2.CAP_PROP_FOCUS,        focus)

        self.stopped  = False
        self.grabbed, self.frame = self.stream.read()

    def start(self):
        t = Thread(target=self._update, daemon=True)
        t.start()
        return self

    def _update(self):
        frame_delta = 1.0 / FPS
        while not self.stopped:
            prev = time.time()
            self.grabbed, self.frame = self.stream.read()
            sleep = frame_delta - (time.time() - prev)
            if sleep > 0:
                time.sleep(sleep)

    def stop(self):
        self.stopped = True
        self.stream.release()


class CameraTracker:
    def __init__(self, server: UDPServer, active_robot_ids: list[int]):
        self.server = server
        self.active_robot_ids = set(active_robot_ids)

        # ArUco setup
        self.aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector     = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Camera — threaded stream for reliable 1080p on the C922
        self.vs = WebcamVideoStream(
            src=CAMERA_INDEX,
            width=FRAME_WIDTH,
            height=FRAME_HEIGHT,
            fps=FPS,
            focus=0,
        ).start()

        if not self.vs.grabbed:
            raise RuntimeError(f"[Tracker] Cannot open camera index {CAMERA_INDEX}")
        print(f"[Tracker] Camera opened ({FRAME_WIDTH}x{FRAME_HEIGHT})")

    def process_frame(self, frame: np.ndarray) -> dict[int, tuple[float, float, float]]:
        """
        Detect ArUco markers in frame and return position data for active robots.
        Returns: { robot_id: (x_m, y_m, theta_rad), ... }
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        positions = {}
        if ids is None:
            return positions

        for corner, marker_id in zip(corners, ids.flatten()):
            if marker_id not in self.active_robot_ids:
                continue

            pts = corner[0]
            cx_px = float(np.mean(pts[:, 0]))
            cy_px = float(np.mean(pts[:, 1]))
            x_m, y_m = pixel_to_world(cx_px, cy_px)
            theta = marker_heading(corner)

            positions[int(marker_id)] = (x_m, y_m, theta)

        return positions

    def annotate_frame(
        self,
        frame: np.ndarray,
        positions: dict[int, tuple[float, float, float]],
    ) -> np.ndarray:
        """Draw detections and the world-origin crosshair onto the frame."""
        annotated = frame.copy()

        # World origin crosshair
        cross_size = 30
        cv2.line(annotated, (CX - cross_size, CY), (CX + cross_size, CY), (0, 255, 0), 2)
        cv2.line(annotated, (CX, CY - cross_size), (CX, CY + cross_size), (0, 255, 0), 2)
        cv2.putText(annotated, "(0,0)", (CX + 8, CY - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Annotate each detected robot
        for robot_id, (x_m, y_m, theta) in positions.items():
            px = int(CX + x_m * PX_PER_M_X)
            py = int(CY - y_m * PX_PER_M_Y)

            # Heading arrow — points in the direction the front of the robot faces
            arrow_len = 40
            ex = int(px + arrow_len * math.cos(theta))
            ey = int(py - arrow_len * math.sin(theta))
            cv2.arrowedLine(annotated, (px, py), (ex, ey), (0, 0, 255), 2, tipLength=0.3)

            label = f"ID:{robot_id}  ({x_m:.2f}m, {y_m:.2f}m)  {math.degrees(theta):.1f}deg"
            cv2.putText(annotated, label, (px + 10, py - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            cv2.circle(annotated, (px, py), 6, (255, 255, 0), -1)

        return annotated

    def run(self):
        """Main tracking loop — processes frames and pushes data to the UDP server."""
        print("[Tracker] Starting tracking loop. Press 'q' in the video window to quit.")
        cv2.namedWindow("Robot Tracker", cv2.WINDOW_NORMAL)

        while True:
            if not self.vs.grabbed:
                print("[Tracker] Failed to read frame.")
                break

            frame = self.vs.frame.copy()
            positions = self.process_frame(frame)

            if positions:
                self.server.update_positions(positions)

            annotated = self.annotate_frame(frame, positions)
            cv2.imshow("Robot Tracker", annotated)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("[Tracker] Quit signal received.")
                break

        self.vs.stop()
        cv2.destroyAllWindows()
        print("[Tracker] Camera released.")


# ── Standalone test (no server required) ─────────────────────────────────────
if __name__ == "__main__":
    class StubServer:
        def update_positions(self, pos):
            print(f"[StubServer] Positions: {pos}")
        def set_command(self, run):
            pass

    tracker = CameraTracker(
        server=StubServer(),
        active_robot_ids=[0, 1, 2, 3, 4],
    )
    tracker.run()
