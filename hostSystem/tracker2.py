"""
Code authored by Keegan Kelly
Modified: added corner-marker spatial calibration (2D affine) to correct
distance/scale errors without relying solely on the intrinsic camera calibration.

SETUP:
  1. Print 4 ArUco markers from DICT_4X4_1000 with IDs 20, 21, 22, 23.
     Use makeMarker.py, just change the id on the cv2.aruco.generateImageMarker call.
  2. Place them at measured real-world positions relative to the origin marker.
     Edit CORNER_MARKER_REAL_POSITIONS below with those measurements (in metres).
  3. Run as normal.  The script will wait until all corner markers are seen,
     average CORNER_CALIB_SAMPLES frames for each, fit the affine transform,
     then print the calibration error and start tracking.
  4. Press 'c' in the video window to redo calibration without restarting.
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

# ---------------------------------------------------------------------------
# CORNER MARKER CONFIGURATION
# Edit these to match your physical setup.
#
# CORNER_MARKER_REAL_POSITIONS maps marker ID -> (true_x, true_y) in metres,
# measured from the origin marker.  Use as many corners as you have (minimum 3,
# but 4 gives a nicely overdetermined system for a more robust fit).
#
# Example: a 2.0 m x 1.5 m rectangle of corners around the origin:
#   ID 20 = far-left  corner  (-1.0,  0.75)
#   ID 21 = far-right corner  ( 1.0,  0.75)
#   ID 22 = near-right corner ( 1.0, -0.75)
#   ID 23 = near-left  corner (-1.0, -0.75)
# ---------------------------------------------------------------------------
CORNER_MARKER_REAL_POSITIONS = {
    20: (-1.68,  0.865),
    21: ( 1.63,  0.865),
    22: ( 1.63, -0.88),
    23: (-1.6775, -0.88), 
}

# Number of frames averaged per corner marker to reduce noise during calibration
CORNER_CALIB_SAMPLES = 30


class Tracker:
    # importing the camera matrix and distortion coefficients
    npfile = np.load("calibration.npz")
    mtx = npfile["mtx"]
    dist = npfile["dist"]

    # Corners dict now also holds the corner calibration marker IDs
    Corners = {10: tuple(), 11: tuple(), 12: tuple(), 13: tuple(),
               14: tuple(), 15: tuple(), 16: tuple(),
               20: tuple(), 21: tuple(), 22: tuple(), 23: tuple()}

    NUMMARKERS = 1 + NUM_ROBOTS

    # positions of each marker (robots only, indices 1..NUM_ROBOTS)
    pos = np.zeros((NUMMARKERS, 3))
    pos[0] = [0, 0, np.pi/2]

    originFound = False

    # Affine calibration state
    # affineM is a 2x3 matrix applied as:  corrected_xy = affineM @ [raw_x, raw_y, 1]
    # Initialised to identity so tracking still works before calibration runs.
    affineM = np.array([[1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0]])
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
        "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
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
        # Accumulator for corner marker samples during spatial calibration
        # { marker_id: list of (raw_x, raw_y) observations }
        self._cornerSamples = {mid: [] for mid in CORNER_MARKER_REAL_POSITIONS}
        self._recalibrateFlag = False

    # ------------------------------------------------------------------
    # Angle utilities
    # ------------------------------------------------------------------

    def fixAngle(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    # ------------------------------------------------------------------
    # Spatial calibration
    # ------------------------------------------------------------------

    def _getRawPosition(self, marker_id):
        """Return the raw (uncorrected) (x, y) of a marker relative to the
        origin, or None if the marker hasn't been seen yet."""
        corners = self.Corners.get(marker_id)
        if corners is None or len(corners) == 0:
            return None
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.markerWidth, self.mtx, self.dist)
        position = np.matmul(self.rodrigues, tvec[0][0] - self.originT[0][0])
        return float(position[0]), float(position[1])

    def runSpatialCalibration(self):
        """Block until all corner markers have been observed CORNER_CALIB_SAMPLES
        times, then fit a 2D affine transform and store it in self.affineM.
        Designed to be called from a thread after the origin is found."""

        print("\n[SpatialCalib] Waiting for origin marker...")
        while not self.originFound:
            time.sleep(0.1)

        print(f"[SpatialCalib] Origin found. Collecting {CORNER_CALIB_SAMPLES} "
              f"samples for each of {list(CORNER_MARKER_REAL_POSITIONS.keys())}...")

        # Reset accumulators
        self._cornerSamples = {mid: [] for mid in CORNER_MARKER_REAL_POSITIONS}

        while True:
            # Collect one observation per corner marker per iteration
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

        print("\n[SpatialCalib] Fitting affine transform...")

        # Average the samples for each marker to get a stable raw estimate
        raw_pts  = []   # observed (x, y) from tracker
        true_pts = []   # known real-world (x, y)
        for mid, true_xy in CORNER_MARKER_REAL_POSITIONS.items():
            samples = self._cornerSamples[mid]
            avg_x = np.mean([s[0] for s in samples])
            avg_y = np.mean([s[1] for s in samples])
            raw_pts.append([avg_x, avg_y])
            true_pts.append(list(true_xy))
            print(f"  Marker {mid}: raw avg=({avg_x:.4f}, {avg_y:.4f})  "
                  f"true=({true_xy[0]:.4f}, {true_xy[1]:.4f})")

        raw_pts  = np.array(raw_pts,  dtype=np.float64)   # shape (N, 2)
        true_pts = np.array(true_pts, dtype=np.float64)   # shape (N, 2)

        # Build the least-squares system for a 2x3 affine matrix M such that
        #   true_pt ≈ M @ [raw_x, raw_y, 1]^T
        # Expanded per coordinate:
        #   true_x = a*raw_x + b*raw_y + c
        #   true_y = d*raw_x + e*raw_y + f
        # We solve each row independently.
        N = len(raw_pts)
        A = np.hstack([raw_pts, np.ones((N, 1))])   # (N, 3)

        # Solve for x-row and y-row of the affine matrix
        row_x, _, _, _ = np.linalg.lstsq(A, true_pts[:, 0], rcond=None)
        row_y, _, _, _ = np.linalg.lstsq(A, true_pts[:, 1], rcond=None)

        M = np.array([row_x, row_y])   # 2x3

        # Report reprojection error on the calibration markers
        errors = []
        for i in range(N):
            pred = M @ np.array([raw_pts[i, 0], raw_pts[i, 1], 1.0])
            err  = np.linalg.norm(pred - true_pts[i])
            errors.append(err)
        mean_err = np.mean(errors)
        max_err  = np.max(errors)
        print(f"[SpatialCalib] Done.  Mean reprojection error: {mean_err*100:.2f} cm  "
              f"Max: {max_err*100:.2f} cm")
        print(f"[SpatialCalib] Affine matrix:\n{M}")

        self.affineM = M
        self.spatialCalibDone = True
        self._recalibrateFlag = False

    def _applyAffine(self, x, y):
        """Apply the calibration affine transform to a raw (x, y) position."""
        pt = self.affineM @ np.array([x, y, 1.0])
        return float(pt[0]), float(pt[1])

    # ------------------------------------------------------------------
    # Main marker detection
    # ------------------------------------------------------------------

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

        if self.originFound or len(self.Corners[10]) != 0:
            if not self.originFound:
                self.originR, self.originT, _ = cv2.aruco.estimatePoseSingleMarkers(
                    self.Corners[10], self.markerWidth, self.mtx, self.dist)
                self.rodrigues = cv2.Rodrigues(self.originR[0][0])[0]
                self.originFound = True

            # Robot markers
            for i in range(1, self.NUMMARKERS):
                if len(self.Corners[10 + i]) != 0:
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        self.Corners[i + 10], self.markerWidth, self.mtx, self.dist)
                    position = np.matmul(self.rodrigues, tvec[0][0] - self.originT[0][0])
                    Rod = cv2.Rodrigues(rvec[0][0])[0]
                    heading = cv2.Rodrigues(np.matmul(self.rodrigues, Rod))[0][2] + np.pi / 2

                    raw_x, raw_y = float(position[0]), float(position[1])

                    # Apply spatial calibration transform if available
                    if self.spatialCalibDone:
                        cal_x, cal_y = self._applyAffine(raw_x, raw_y)
                    else:
                        cal_x, cal_y = raw_x, raw_y

                    self.pos[i] = [cal_x, cal_y, self.fixAngle(heading)[0]]
                    cv2.drawFrameAxes(frame, self.mtx, self.dist, Rod, tvec, self.markerWidth)

            # Draw corner calibration markers if they are visible
            #for mid in CORNER_MARKER_REAL_POSITIONS:
            #    if len(self.Corners[mid]) != 0:
            #        rvec_c, tvec_c, _ = cv2.aruco.estimatePoseSingleMarkers(
            #            self.Corners[mid], self.markerWidth, self.mtx, self.dist)
            #        cv2.drawFrameAxes(frame, self.mtx, self.dist,
            #                          cv2.Rodrigues(self.rodrigues)[0],
            #                          tvec_c, self.markerWidth * 2)

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
        calib_text = "Spatial calib: OK" if self.spatialCalibDone else "Spatial calib: PENDING (need corner markers)"
        calib_color = (0, 255, 0) if self.spatialCalibDone else (0, 165, 255)
        cv2.putText(frame, calib_text, (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, calib_color, 2)

        ##LINE_CLEAR = '\x1b[2K'
        ##printPositions = ""
        ##for i in range(1, NUM_ROBOTS + 1):
        ##    printPositions += ("(" + format(self.pos[i][0], '.3f') + ", "
        ##                        + format(self.pos[i][1], '.3f') + ", "
        ##                       + format(self.pos[i][2], '.2f') + ")")
        ##print(LINE_UP + LINE_CLEAR + printPositions)

        return frame

    # ------------------------------------------------------------------
    # Thread management  (unchanged except for the new calibration thread)
    # ------------------------------------------------------------------

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
        return self

    def stopThread(self):
        self.Stop = True
        self.vs.stop()
        self.vs.stream.release()
        cv2.destroyAllWindows()

    def runPutThread(self):
        prevTime = time.time()
        WINDOW = 5  # number of frames to average — higher = smoother but more lag
        posHistory = [np.zeros((NUM_ROBOTS, 3)) for _ in range(WINDOW)]
        while not self.Stop:
            if (time.time() - prevTime) > 0.05:
                prevTime = time.time()
                # add latest reading to history, drop oldest
                posHistory.append(self.pos[1:].copy())
                posHistory.pop(0)
                # average across the window
                smoothedPos = np.mean(posHistory, axis=0)
                data = {"id": 1, "pos": smoothedPos.tolist()}
                requests.put(self.address + "allPos/1", json=data)

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
            # 'c' triggers a fresh spatial calibration
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
