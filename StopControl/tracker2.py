"""
Code authored by Keegan Kelly
Modified: added corner-marker spatial calibration (homography) to correct
distance/scale errors without relying solely on the intrinsic camera calibration.

Modified 2025: unified safety-stop thread added.

SETUP:
  1. Print 4 ArUco markers from DICT_4X4_1000 with IDs 20, 21, 22, 23.
     Use makeMarker.py, just change the id on the cv2.aruco.generateImageMarker call.
  2. Place them at measured real-world positions relative to the origin marker.
     Edit CORNER_MARKER_REAL_POSITIONS below with those measurements (in metres).
  3. Run as normal.  The script will wait until all corner markers are seen,
     average CORNER_CALIB_SAMPLES frames for each, fit the homography transform,
     then print the calibration error and start tracking.
  4. Press 'c' in the video window to redo calibration without restarting.

OBSTACLE (ArUco ID 18):
  Optionally place marker 18 anywhere in the field.  It is tracked like a robot
  marker but is never used for calibration.  The checkSafety thread treats it
  exactly like a robot — if any two tracked markers (robots or obstacle) come
  within STOP_DISTANCE of each other, agentStop is set to 1 for both.
"""

import cv2
import numpy as np
import time
import requests
import os

os.environ["OPENCV_LOG_LEVEL"] = "SILENT"

from ast import Pass
from threading import Thread
from webcamvideostream import WebcamVideoStream

def clear():
    os.system('cls' if os.name == 'nt' else 'clear')

NUM_ROBOTS = 6

# ── Safety thresholds (metres) — apply to any two tracked markers ─────────────
# This covers robot↔robot and robot↔obstacle (ArUco 18) equally.
WARN_DISTANCE = 0.5   # yellow warning printed to terminal
STOP_DISTANCE = 0.3   # agentStop set to 1 for both robots involved
# ─────────────────────────────────────────────────────────────────────────────

# ── Corner marker configuration ───────────────────────────────────────────────
# Edit these to match your physical setup (metres from origin marker).
CORNER_MARKER_REAL_POSITIONS = {
    20: (-1.68,  0.865),
    21: ( 1.63,  0.865),
    22: ( 1.63, -0.88),
    23: (-1.6775, -0.88),
}

CORNER_CALIB_SAMPLES = 30
# ─────────────────────────────────────────────────────────────────────────────


class Tracker:
    npfile = np.load("calibration.npz")
    mtx = npfile["mtx"]
    dist = npfile["dist"]

    # Robot markers (10=origin, 11-16=agents), corner calibration markers (20-23),
    # and obstacle marker (18).
    Corners = {
        10: tuple(),
        11: tuple(), 12: tuple(), 13: tuple(),
        14: tuple(), 15: tuple(), 16: tuple(),
        18: tuple(),   # ← obstacle
        20: tuple(), 21: tuple(), 22: tuple(), 23: tuple(),
    }

    NUMMARKERS = 1 + NUM_ROBOTS

    pos = np.zeros((NUMMARKERS, 3))
    pos[0] = [0, 0, np.pi/2]

    # Tracks the last time each marker ID was actually detected in a frame.
    # Used by checkSafety to fire a stop when a robot leaves the camera view.
    lastSeen = {}          # {marker_id: float (time.time())}
    LOST_FRAMES = 5        # consecutive checkSafety ticks without detection before stop fires

    # Obstacle state
    obstaclePos   = None   # [x, y, θ] once first seen; updated every frame
    obstacleFound = False

    originFound = False

    homoM = np.eye(3, dtype=np.float64)   # 3x3 homography, identity until calibrated
    spatialCalibDone = False

    ARUCO_DICT = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
        "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
        "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
        "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
        "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
        "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
        "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
        "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
        "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
        "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
        "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
        "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
    }

    def __init__(self, marker_width, aruco_type, address, fps=60, wideAngle=False):
        self.markerWidth = marker_width
        self.arucoDict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT[aruco_type])
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.arucoDetector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)
        self.startTime = time.perf_counter()
        self.address = address
        self.frameRate = fps
        self.wideAngle = wideAngle
        if self.wideAngle:
            npfile = np.load("wideAngleCalibration.npz")
            self.mtx = npfile["mtx"]
            self.dist = npfile["dist"]
        self._cornerSamples = {mid: [] for mid in CORNER_MARKER_REAL_POSITIONS}
        self._recalibrateFlag = False
        self._safety_clear = set()   # robot IDs to unblock on next checkSafety tick

    # ── Angle utility ─────────────────────────────────────────────────────────

    def fixAngle(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    # ── Spatial calibration ───────────────────────────────────────────────────

    def _getRawPosition(self, marker_id):
        corners = self.Corners.get(marker_id)
        if corners is None or len(corners) == 0:
            return None
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.markerWidth, self.mtx, self.dist)
        position = np.matmul(self.rodrigues, tvec[0][0] - self.originT[0][0])
        return float(position[0]), float(position[1])

    def runSpatialCalibration(self):
        print("\n[SpatialCalib] Waiting for origin marker...")
        while not self.originFound:
            time.sleep(0.1)

        print(f"[SpatialCalib] Origin found. Collecting {CORNER_CALIB_SAMPLES} "
              f"samples for each of {list(CORNER_MARKER_REAL_POSITIONS.keys())}...")

        self._cornerSamples = {mid: [] for mid in CORNER_MARKER_REAL_POSITIONS}

        while True:
            all_done = True
            status_parts = []
            for mid in CORNER_MARKER_REAL_POSITIONS:
                samples = self._cornerSamples[mid]
                n = len(samples)
                if n < CORNER_CALIB_SAMPLES:
                    all_done = False
                    raw = self._getRawPosition(mid)
                    if raw is not None:
                        samples.append(raw)
                status_parts.append(f"ID{mid}:{len(self._cornerSamples[mid])}/{CORNER_CALIB_SAMPLES}")
            print("\r[SpatialCalib] " + "  ".join(status_parts), end="", flush=True)
            if all_done:
                break
            time.sleep(1.0 / self.frameRate)

        print("\n[SpatialCalib] Fitting homography...")

        raw_pts  = []
        true_pts = []
        for mid, true_xy in CORNER_MARKER_REAL_POSITIONS.items():
            samples = self._cornerSamples[mid]
            avg_x = np.mean([s[0] for s in samples])
            avg_y = np.mean([s[1] for s in samples])
            raw_pts.append([avg_x, avg_y])
            true_pts.append(list(true_xy))
            print(f"  Marker {mid}: raw avg=({avg_x:.4f}, {avg_y:.4f})  "
                  f"true=({true_xy[0]:.4f}, {true_xy[1]:.4f})")

        raw_pts  = np.array(raw_pts,  dtype=np.float32)
        true_pts = np.array(true_pts, dtype=np.float32)

        # findHomography needs shape (N,1,2)
        M, mask = cv2.findHomography(raw_pts.reshape(-1, 1, 2),
                                     true_pts.reshape(-1, 1, 2))

        # Reprojection error
        errors = []
        for i in range(len(raw_pts)):
            src = np.array([raw_pts[i, 0], raw_pts[i, 1], 1.0], dtype=np.float64)
            dst = M @ src
            dst /= dst[2]
            err = np.linalg.norm(dst[:2] - true_pts[i])
            errors.append(err)
        mean_err = np.mean(errors)
        max_err  = np.max(errors)
        print(f"[SpatialCalib] Done.  Mean reprojection error: {mean_err*100:.2f} cm  "
              f"Max: {max_err*100:.2f} cm")
        print(f"[SpatialCalib] Homography matrix:\n{M}")

        self.homoM = M
        self.spatialCalibDone = True
        self._recalibrateFlag = False

    def _applyHomography(self, x, y):
        pt = self.homoM @ np.array([x, y, 1.0], dtype=np.float64)
        pt /= pt[2]   # perspective divide
        return float(pt[0]), float(pt[1])

    # ── Main marker detection ─────────────────────────────────────────────────

    def find_markerPos(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        (corners, ids, rejectedImgPoints) = cv2.aruco.detectMarkers(
            gray, self.arucoDict, parameters=self.arucoParams)

        if len(corners) > 0:
            ids.flatten()
            for i in range(len(ids)):
                mid = ids[i][0]
                if mid in self.Corners:
                    self.Corners[mid] = corners[i]
                    self.lastSeen[mid] = time.time()   # record fresh detection

        if self.originFound or len(self.Corners[10]) != 0:
            if not self.originFound:
                self.originR, self.originT, _ = cv2.aruco.estimatePoseSingleMarkers(
                    self.Corners[10], self.markerWidth, self.mtx, self.dist)
                self.rodrigues = cv2.Rodrigues(self.originR[0][0])[0]
                self.originFound = True

            # ── Robot markers (11–16) ─────────────────────────────────────────
            for i in range(1, self.NUMMARKERS):
                if len(self.Corners[10 + i]) != 0:
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        self.Corners[i + 10], self.markerWidth, self.mtx, self.dist)
                    position = np.matmul(self.rodrigues, tvec[0][0] - self.originT[0][0])
                    Rod = cv2.Rodrigues(rvec[0][0])[0]
                    heading = cv2.Rodrigues(np.matmul(self.rodrigues, Rod))[0][2] + np.pi / 2

                    raw_x, raw_y = float(position[0]), float(position[1])
                    if self.spatialCalibDone:
                        cal_x, cal_y = self._applyHomography(raw_x, raw_y)
                    else:
                        cal_x, cal_y = raw_x, raw_y

                    self.pos[i] = [cal_x, cal_y, self.fixAngle(heading)[0]]
                    cv2.drawFrameAxes(frame, self.mtx, self.dist, Rod, tvec, self.markerWidth)

            # ── Obstacle marker (18) ──────────────────────────────────────────
            if len(self.Corners[18]) != 0:
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    self.Corners[18], self.markerWidth, self.mtx, self.dist)
                position = np.matmul(self.rodrigues, tvec[0][0] - self.originT[0][0])
                Rod = cv2.Rodrigues(rvec[0][0])[0]
                heading = cv2.Rodrigues(np.matmul(self.rodrigues, Rod))[0][2] + np.pi / 2

                # Obstacle is a free-floating marker — use raw position directly.
                # Do NOT apply the affine calibration: marker 18 is not a fixed
                # reference point and its true coordinates are unknown.
                raw_x, raw_y = float(position[0]), float(position[1])

                self.obstaclePos   = [raw_x, raw_y, self.fixAngle(heading)[0]]
                self.obstacleFound = True
                cv2.drawFrameAxes(frame, self.mtx, self.dist, Rod, tvec, self.markerWidth)

                # Label on the video feed
                img_pts, _ = cv2.projectPoints(
                    np.array([[0.0, 0.0, 0.0]]),
                    rvec[0][0], tvec[0][0], self.mtx, self.dist)
                px = int(img_pts[0][0][0])
                py = int(img_pts[0][0][1])
                cv2.putText(frame, "OBSTACLE", (px + 10, py - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        if self.originFound:
            cv2.drawFrameAxes(frame, self.mtx, self.dist,
                              self.rodrigues, self.originT[0][0], self.markerWidth * 5)

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # FPS counter
        self.endTime = time.perf_counter()
        dt = self.endTime - self.startTime
        self.startTime = self.endTime
        if dt != 0:
            cv2.putText(frame, "FPS: " + format(1 / dt, '.2f'),
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Calibration status overlay
        calib_text  = "Spatial calib: OK" if self.spatialCalibDone else "Spatial calib: PENDING (need corner markers)"
        calib_color = (0, 255, 0) if self.spatialCalibDone else (0, 165, 255)
        cv2.putText(frame, calib_text, (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, calib_color, 2)
        

        # ── Draw virtual boundary rectangles on frame ─────────────────────
        if self.originFound:
            # Boundary constants (metres) — match RUNME3.ino
            X_MIN, X_MAX   = -1.68,  1.63
            Y_MIN, Y_MAX   = -0.88,  0.865
            SOFT_MARGIN    =  0.45   # repulsion starts here (larger rectangle)
            HARD_MARGIN    =  0.10   # hard limit / recovery zone (smaller rectangle)

            def world_to_pixel(wx, wy):
                world_pt = np.array([wx, wy, 0.0])
                cam_pt   = np.matmul(self.rodrigues.T, world_pt) + self.originT[0][0]
                px, _    = cv2.projectPoints(
                    cam_pt.reshape(1, 1, 3),
                    np.zeros(3), np.zeros(3),
                    self.mtx, self.dist)
                return (int(px[0][0][0]), int(px[0][0][1]))

            def draw_boundary_rect(x_min, x_max, y_min, y_max, color, thickness):
                tl = world_to_pixel(x_min, y_max)
                tr = world_to_pixel(x_max, y_max)
                br = world_to_pixel(x_max, y_min)
                bl = world_to_pixel(x_min, y_min)
                pts = np.array([tl, tr, br, bl], dtype=np.int32)
                cv2.polylines(frame, [pts], isClosed=True,
                              color=color, thickness=thickness)

            # Green — soft zone boundary, robot starts turning inside here
            draw_boundary_rect(
                X_MIN + SOFT_MARGIN, X_MAX - SOFT_MARGIN,
                Y_MIN + SOFT_MARGIN, Y_MAX - SOFT_MARGIN,
                color=(0, 255, 0), thickness=1)

            # Red — hard limit, robot should never cross this line
            draw_boundary_rect(
                X_MIN + HARD_MARGIN, X_MAX - HARD_MARGIN,
                Y_MIN + HARD_MARGIN, Y_MAX - HARD_MARGIN,
                color=(0, 0, 255), thickness=2)

        return frame

    # ── Thread management ─────────────────────────────────────────────────────

    def startThreads(self, check_ready=True):
        self.Stop = False
        self.runGetFrame(frameRate=self.frameRate)

        t2 = Thread(target=self.runProcessFrame)
        t2.daemon = False
        t2.start()

        t3 = Thread(target=self.runShowFrame)
        t3.daemon = False
        t3.start()

        t1 = Thread(target=self.runPutThread)
        t1.daemon = False
        t1.start()

        if check_ready:
            t4 = Thread(target=self.checkReady)
            t4.daemon = False
            t4.start()

        t5 = Thread(target=self.runSpatialCalibration)
        t5.daemon = True
        t5.start()

        t6 = Thread(target=self.checkSafety)
        t6.daemon = True   # daemon so it doesn't block shutdown
        t6.start()

        return self

    def stopThread(self):
        self.Stop = True
        self.vs.stop()
        self.vs.stream.release()
        cv2.destroyAllWindows()

    def clearStoppedRobot(self, robot_id: int):
        """
        Called by move.py at the start of every 'go' command to reset the
        safety state for a robot so it isn't immediately re-stopped from a
        previous lost-marker event. Also clears lastSeen so the timeout
        doesn't fire until the marker has been seen again in a fresh frame.
        """
        self._safety_clear.add(robot_id)
        self.lastSeen.pop(10 + robot_id, None)

    # ── Worker threads ────────────────────────────────────────────────────────

    def runPutThread(self):
        """Push smoothed robot positions to /allPos/1 at ~20 Hz.
        Also publishes the obstacle position to /obstacle whenever detected."""
        prevTime = time.time()
        WINDOW = 5
        posHistory = [np.zeros((NUM_ROBOTS, 3)) for _ in range(WINDOW)]
        while not self.Stop:
            if (time.time() - prevTime) > 0.05:
                prevTime = time.time()
                posHistory.append(self.pos[1:].copy())
                posHistory.pop(0)
                smoothedPos = np.mean(posHistory, axis=0)
                data = {"id": 1, "pos": smoothedPos.tolist()}
                requests.put(self.address + "allPos/1", json=data)

                if self.obstacleFound and self.obstaclePos is not None:
                    obs_data = {
                        "id": 18,
                        "position": self.obstaclePos,
                        "found": True,
                    }
                    try:
                        requests.put(self.address + "obstacle", json=obs_data, timeout=0.1)
                    except Exception:
                        pass

    def runProcessFrame(self):
        while True:
            if self.Stop:
                return
            if self.vs.grabbed:
                self.outFrame = self.find_markerPos(self.vs.frame)

    def runGetFrame(self, frameRate):
        Focus = 20 if self.wideAngle else 0
        self.vs = WebcamVideoStream(src=1, fps=frameRate, focus=Focus).start()
        self.vs.start()
        self.outFrame = self.vs.frame

    def runShowFrame(self):
        prevTime = time.time()
        frameDelta = 1 / self.frameRate
        while True:
            if self.Stop:
                return
            if self.vs.grabbed:
                prevTime = time.time()
                cv2.imshow('frame', self.outFrame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.stopThread()
                break
            if key == ord('r'):
                self.originFound = False
            if key == ord('c'):
                self.spatialCalibDone = False
                t = Thread(target=self.runSpatialCalibration)
                t.daemon = True
                t.start()
            sleepTime = frameDelta - (time.time() - prevTime)
            time.sleep(sleepTime * (sleepTime > 0))
        return self

    def checkReady(self):
        prevTime = time.time()
        connected = set()
        while True:
            if (time.time() - prevTime) > 2:
                prevTime = time.time()
                req = requests.get(self.address + "agentReady")
                DATA = req.json()
                SUM = 0
                for i in range(NUM_ROBOTS):
                    SUM += DATA[i]["ready"]
                    if DATA[i]["ready"] == 1 and i not in connected:
                        connected.add(i)
                        print(f"Robot {i+1} connected!")
                if SUM == NUM_ROBOTS:
                    print("All robots connected! Starting...")
                    for i in range(NUM_ROBOTS):
                        requests.put(self.address + "agentGo/" + str(int(i + 1)),
                                     json={'id': i + 1, 'ready': 1})
                    break

    def checkSafety(self):
        """
        Runs at ~10 Hz. Checks every pair of currently tracked markers —
        robots (IDs 11-16) and the obstacle (ID 18) if visible — against
        WARN_DISTANCE and STOP_DISTANCE.

        On a STOP event both markers involved have agentStop set to 1.
        For the obstacle (which has no agentStop of its own) only the robot
        in the pair gets stopped. Once both markers in a pair move back out
        of range the pair is cleared so they can be re-sent.
        """
        RESET  = "\033[0m"
        YELLOW = "\033[93m"
        RED    = "\033[91m"

        # Set of robot IDs currently held stopped
        stopped_robots: set = set()

        missed_frames = {}   # {robot_id: int} consecutive ticks without detection

        while not self.Stop:
            time.sleep(0.1)

            now = time.time()

            # Unblock any robots that move.py has explicitly re-armed
            if self._safety_clear:
                stopped_robots -= self._safety_clear
                for rid in self._safety_clear:
                    missed_frames.pop(rid, None)
                self._safety_clear.clear()

            # ── Lost-marker check ─────────────────────────────────────────────
            # Count consecutive ticks where the marker wasn't seen. Only fire
            # after LOST_FRAMES misses in a row to avoid false stops from a
            # single dropped detection.
            for i in range(1, NUM_ROBOTS + 1):
                marker_id = 10 + i
                last = self.lastSeen.get(marker_id, None)
                if last is None:
                    continue   # never seen yet, don't count
                if (now - last) > 0.15:   # marker not seen this tick (100ms loop + buffer)
                    missed_frames[i] = missed_frames.get(i, 0) + 1
                else:
                    missed_frames[i] = 0   # reset on fresh detection

                if missed_frames.get(i, 0) >= self.LOST_FRAMES:
                    if i not in stopped_robots:
                        stopped_robots.add(i)
                        print(
                            f"\n{RED}[SAFETY STOP]  Robot {i} marker lost "
                            f"({missed_frames[i]} consecutive misses) — setting stop signal!{RESET}"
                        )
                        try:
                            requests.put(
                                self.address + f"agentStop/{i}",
                                json={"id": i, "stop": 1},
                                timeout=1,
                            )
                        except Exception as e:
                            print(f"  [SAFETY] Server unreachable: {e}")

            # Build the list of currently tracked markers as (label, xy, robot_id_or_None)
            # robot_id is None for the obstacle since it has no agentStop entry
            tracked = []
            for i in range(1, NUM_ROBOTS + 1):
                xy = self.pos[i][:2]
                if not np.allclose(xy, [0.0, 0.0]):
                    tracked.append((f"Robot {i}", np.array(xy), i))
            if self.obstacleFound and self.obstaclePos is not None:
                tracked.append(("Obstacle", np.array(self.obstaclePos[:2]), None))

            # Check every unique pair
            currently_stopped: set = set()
            for a in range(len(tracked)):
                for b in range(a + 1, len(tracked)):
                    label_a, xy_a, rid_a = tracked[a]
                    label_b, xy_b, rid_b = tracked[b]
                    dist = float(np.linalg.norm(xy_a - xy_b))

                    if dist <= STOP_DISTANCE:
                        # Collect the robot IDs involved (obstacle has no ID)
                        pair_robots = {r for r in (rid_a, rid_b) if r is not None}
                        currently_stopped |= pair_robots

                        # Fire stop for any robot in this pair not already stopped
                        for rid in pair_robots:
                            if rid not in stopped_robots:
                                stopped_robots.add(rid)
                                print(
                                    f"\n{RED}[SAFETY STOP]  {label_a} ↔ {label_b}  "
                                    f"dist={dist:.3f} m (limit={STOP_DISTANCE} m) — "
                                    f"setting stop signal for Robot {rid}!{RESET}"
                                )
                                try:
                                    requests.put(
                                        self.address + f"agentStop/{rid}",
                                        json={"id": rid, "stop": 1},
                                        timeout=1,
                                    )
                                except Exception as e:
                                    print(f"  [SAFETY] Server unreachable: {e}")

                    elif dist <= WARN_DISTANCE:
                        print(
                            f"{YELLOW}[SAFETY WARN]  {label_a} ↔ {label_b}  "
                            f"dist={dist:.3f} m{RESET}",
                            end="\r",
                        )

            # Clear robots that are no longer in any stopped pair
            stopped_robots &= currently_stopped
