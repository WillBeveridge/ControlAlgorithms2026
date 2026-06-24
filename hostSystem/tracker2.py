"""
tracker2.py  —  pixel-based tracking, no origin/corner markers needed.

Coordinate system:
  - Frame centre = (0, 0) in metres
  - +X right, +Y up  (Y is flipped from pixel space where +Y is down)
  - Scale: PIXELS_PER_M derived from known arena size and frame resolution

Robot markers: ArUco IDs 11-16  (robot i uses ID 10+i)
No origin marker needed. Camera just needs to be roughly overhead and centred.

Press q — quit
Press r — no-op, kept for compatibility
"""

import cv2
import numpy as np
import time
import requests
import os
from threading import Thread
from webcamvideostream import WebcamVideoStream

os.environ["OPENCV_LOG_LEVEL"] = "SILENT"

NUM_ROBOTS = 6

# ── Arena / camera config ─────────────────────────────────────────────────────
FRAME_W       = 1920
FRAME_H       = 1080
ARENA_W_M     = 3.60          # physical arena width  in metres
ARENA_H_M     = 2.03          # physical arena height in metres

PIXELS_PER_M_X = FRAME_W / ARENA_W_M
PIXELS_PER_M_Y = FRAME_H / ARENA_H_M

CX = FRAME_W // 2
CY = FRAME_H // 2

# ── Boundary display constants (pixels from frame edge) ──────────────────────
SOFT_PX = 100
HARD_PX = 50

# ── Safety thresholds (metres) ────────────────────────────────────────────────
WARN_DISTANCE = 0.5
STOP_DISTANCE = 0.3


def pixel_to_metres(px, py):
    x_m =  (px - CX) / PIXELS_PER_M_X
    y_m = -(py - CY) / PIXELS_PER_M_Y
    return x_m, y_m


def metres_to_pixel(x_m, y_m):
    px = int( x_m * PIXELS_PER_M_X + CX)
    py = int(-y_m * PIXELS_PER_M_Y + CY)
    return px, py


def marker_heading(corners):
    c = corners[0]
    right_mid = (c[1] + c[2]) / 2.0
    left_mid  = (c[0] + c[3]) / 2.0
    dx =  (right_mid[0] - left_mid[0]) / PIXELS_PER_M_X
    dy = -(right_mid[1] - left_mid[1]) / PIXELS_PER_M_Y
    angle = np.arctan2(dy, dx) + np.pi / 2
    while angle >  np.pi: angle -= 2 * np.pi
    while angle < -np.pi: angle += 2 * np.pi
    return angle


class Tracker:

    try:
        npfile = np.load("calibration.npz")
        mtx  = npfile["mtx"]
        dist = npfile["dist"]
    except Exception:
        mtx  = np.eye(3)
        dist = np.zeros(5)

    ARUCO_DICT = {
        "DICT_4X4_50":         cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100":        cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250":        cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000":       cv2.aruco.DICT_4X4_1000,
        "DICT_5X5_50":         cv2.aruco.DICT_5X5_50,
        "DICT_5X5_100":        cv2.aruco.DICT_5X5_100,
        "DICT_5X5_250":        cv2.aruco.DICT_5X5_250,
        "DICT_5X5_1000":       cv2.aruco.DICT_5X5_1000,
        "DICT_6X6_50":         cv2.aruco.DICT_6X6_50,
        "DICT_6X6_100":        cv2.aruco.DICT_6X6_100,
        "DICT_6X6_250":        cv2.aruco.DICT_6X6_250,
        "DICT_6X6_1000":       cv2.aruco.DICT_6X6_1000,
        "DICT_7X7_50":         cv2.aruco.DICT_7X7_50,
        "DICT_7X7_100":        cv2.aruco.DICT_7X7_100,
        "DICT_7X7_250":        cv2.aruco.DICT_7X7_250,
        "DICT_7X7_1000":       cv2.aruco.DICT_7X7_1000,
        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
        "DICT_APRILTAG_16h5":  cv2.aruco.DICT_APRILTAG_16h5,
        "DICT_APRILTAG_25h9":  cv2.aruco.DICT_APRILTAG_25h9,
        "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
        "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
    }

    def __init__(self, marker_width, aruco_type, address, fps=30, wideAngle=False):
        self.markerWidth   = marker_width
        self.arucoDict     = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT[aruco_type])
        self.arucoParams   = cv2.aruco.DetectorParameters()
        self.address       = address
        self.frameRate     = fps
        self.wideAngle     = wideAngle
        self._safety_clear = set()

        # Per-instance state (not class-level to avoid sharing between instances)
        self.pos      = np.zeros((1 + NUM_ROBOTS, 3))
        self.Corners  = {11: tuple(), 12: tuple(), 13: tuple(),
                         14: tuple(), 15: tuple(), 16: tuple()}
        self.lastSeen = {}
        self.Stop     = False
        self.out_frame = None

        self._t_prev = time.perf_counter()

    # ── Frame processing ──────────────────────────────────────────────────────

    def find_markerPos(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.arucoDict, parameters=self.arucoParams)

        if ids is not None and len(corners) > 0:
            for i, mid in enumerate(ids.flatten()):
                if mid in self.Corners:
                    self.Corners[mid]  = corners[i]
                    self.lastSeen[mid] = time.time()

        for i in range(1, NUM_ROBOTS + 1):
            mid = 10 + i
            c   = self.Corners.get(mid)
            if c is not None and len(c) > 0:
                px_c = float(np.mean(c[0][:, 0]))
                py_c = float(np.mean(c[0][:, 1]))
                x_m, y_m = pixel_to_metres(px_c, py_c)
                theta     = marker_heading(c)
                self.pos[i] = [x_m, y_m, theta]

                px_i, py_i = int(px_c), int(py_c)
                cv2.drawMarker(frame, (px_i, py_i), (0, 255, 0),
                               cv2.MARKER_CROSS, 20, 2)
                cv2.putText(frame, f"R{i}  ({x_m:.2f},{y_m:.2f})",
                            (px_i + 12, py_i - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # FPS
        now = time.perf_counter()
        dt  = now - self._t_prev
        self._t_prev = now
        if dt > 0:
            cv2.putText(frame, f"FPS: {1/dt:.1f}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Boundary rectangles
        cv2.rectangle(frame, (SOFT_PX, SOFT_PX),
                      (FRAME_W - SOFT_PX, FRAME_H - SOFT_PX), (0, 255, 0), 1)
        cv2.rectangle(frame, (HARD_PX, HARD_PX),
                      (FRAME_W - HARD_PX, FRAME_H - HARD_PX), (0, 0, 255), 2)

        # Centre lines and origin
        cv2.line(frame, (CX, 0), (CX, FRAME_H), (0, 255, 255), 1)
        cv2.line(frame, (0, CY), (FRAME_W, CY), (0, 255, 255), 1)
        cv2.drawMarker(frame, (CX, CY), (0, 0, 255), cv2.MARKER_CROSS, 40, 2)
        cv2.putText(frame, "(0,0)", (CX + 10, CY - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        return frame

    # ── Thread management ─────────────────────────────────────────────────────

    def startThreads(self, check_ready=True):
        self.Stop = False

        # Start camera — wait until first frame is ready before launching threads
        Focus = 20 if self.wideAngle else 0
        self.vs = WebcamVideoStream(
            src=1, fps=self.frameRate, focus=Focus,
            width=FRAME_W, height=FRAME_H).start()

        # Block until camera delivers a real frame
        while not self.vs.grabbed or self.vs.frame is None:
            time.sleep(0.05)
        self.out_frame = self.vs.frame.copy()

        Thread(target=self._process_loop, daemon=False).start()
        
        Thread(target=self._put_loop,     daemon=False).start()
        Thread(target=self.checkSafety,   daemon=True ).start()

        if check_ready:
            Thread(target=self.checkReady, daemon=False).start()

        return self

    def stopThread(self):
        self.Stop = True
        self.vs.stop()
        self.vs.stream.release()
        cv2.destroyAllWindows()

    def clearStoppedRobot(self, robot_id: int):
        self._safety_clear.add(robot_id)
        self.lastSeen.pop(10 + robot_id, None)

    # ── Worker threads ────────────────────────────────────────────────────────

    def _process_loop(self):
        while not self.Stop:
            if self.vs.grabbed:
                self.out_frame = self.find_markerPos(self.vs.frame.copy())

    def _display_loop(self):
        frame_delta = 1.0 / self.frameRate
        cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
        while not self.Stop:
            prev = time.time()
            if self.out_frame is not None:
                cv2.imshow('frame', self.out_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.stopThread()
                break
            sleep = frame_delta - (time.time() - prev)
            time.sleep(max(0.0, sleep))

    def _put_loop(self):
        WINDOW     = 5
        posHistory = [np.zeros((NUM_ROBOTS, 3)) for _ in range(WINDOW)]
        prev       = time.time()
        while not self.Stop:
            if (time.time() - prev) > 0.05:
                prev = time.time()
                posHistory.append(self.pos[1:].copy())
                posHistory.pop(0)
                smoothed = np.mean(posHistory, axis=0)
                data = {"id": 1, "pos": smoothed.tolist()}
                try:
                    requests.put(self.address + "allPos/1", json=data, timeout=0.1)
                except requests.exceptions.RequestException:
                    pass

    def checkReady(self):
        prev      = time.time()
        connected = set()
        while True:
            if (time.time() - prev) > 2:
                prev = time.time()
                try:
                    req  = requests.get(self.address + "agentReady", timeout=1)
                    DATA = req.json()
                except Exception:
                    continue
                total = 0
                for i in range(NUM_ROBOTS):
                    total += DATA[i]["ready"]
                    if DATA[i]["ready"] == 1 and i not in connected:
                        connected.add(i)
                        print(f"Robot {i+1} connected!")
                if total == NUM_ROBOTS:
                    print("All robots connected! Starting...")
                    for i in range(NUM_ROBOTS):
                        requests.put(self.address + "agentGo/" + str(i + 1),
                                     json={'id': i + 1, 'ready': 1})
                    break

    def checkSafety(self):
        RESET  = "\033[0m"
        YELLOW = "\033[93m"
        RED    = "\033[91m"
        stopped_robots = set()
        missed_frames  = {}

        while not self.Stop:
            time.sleep(0.1)
            now = time.time()

            if self._safety_clear:
                stopped_robots -= self._safety_clear
                for rid in self._safety_clear:
                    missed_frames.pop(rid, None)
                self._safety_clear.clear()

            # Lost marker check
            for i in range(1, NUM_ROBOTS + 1):
                marker_id = 10 + i
                last = self.lastSeen.get(marker_id)
                if last is None:
                    continue
                if (now - last) > 0.15:
                    missed_frames[i] = missed_frames.get(i, 0) + 1
                else:
                    missed_frames[i] = 0
                if missed_frames.get(i, 0) >= 5:
                    if i not in stopped_robots:
                        stopped_robots.add(i)
                        print(f"\n{RED}[SAFETY STOP] Robot {i} marker lost — stop!{RESET}")
                        try:
                            requests.put(self.address + f"agentStop/{i}",
                                         json={"id": i, "stop": 1}, timeout=1)
                        except Exception as e:
                            print(f"  [SAFETY] Server unreachable: {e}")

            # Proximity check
            tracked = []
            for i in range(1, NUM_ROBOTS + 1):
                xy = self.pos[i][:2]
                if not np.allclose(xy, [0.0, 0.0]):
                    tracked.append((f"Robot {i}", np.array(xy), i))

            currently_stopped = set()
            for a in range(len(tracked)):
                for b in range(a + 1, len(tracked)):
                    label_a, xy_a, rid_a = tracked[a]
                    label_b, xy_b, rid_b = tracked[b]
                    dist = float(np.linalg.norm(xy_a - xy_b))
                    if dist <= STOP_DISTANCE:
                        pair_robots = {rid_a, rid_b}
                        currently_stopped |= pair_robots
                        for rid in pair_robots:
                            if rid not in stopped_robots:
                                stopped_robots.add(rid)
                                print(f"\n{RED}[SAFETY STOP] {label_a} <-> {label_b} "
                                      f"dist={dist:.3f}m — stop Robot {rid}!{RESET}")
                                try:
                                    requests.put(self.address + f"agentStop/{rid}",
                                                 json={"id": rid, "stop": 1}, timeout=1)
                                except Exception as e:
                                    print(f"  [SAFETY] Server unreachable: {e}")
                    elif dist <= WARN_DISTANCE:
                        print(f"{YELLOW}[SAFETY WARN] {label_a} <-> {label_b} "
                              f"dist={dist:.3f}m{RESET}", end="\r")

            stopped_robots &= currently_stopped
