"""
Code authored by Keegan Kelly
"""
import cv2
import numpy as np
import time
import requests
import os

from ast import Pass
from threading import Thread
from webcamvideostream import WebcamVideoStream

def clear():
    os.system('cls' if os.name == 'nt' else 'clear')

NUM_ROBOTS = 6

class Tracker:
    # importing the camera matrix and distortion coefficients
    # if the file path is working get rid of the folder or add a non relative path
    npfile = np.load("calibration.npz")
    mtx = npfile["mtx"]
    dist = npfile["dist"]
    # declaring the dictionary that will store the corners of the markers at a specific id
    # marker 10 is the origin and then each agent is id 10 + agent number
    Corners = {10: tuple(), 11: tuple(), 12: tuple(), 13: tuple(), 14: tuple(), 15: tuple(), 16: tuple()}
    
    NUMMARKERS = 1 + NUM_ROBOTS
    
    # positions of each marker
    pos = np.zeros((NUMMARKERS, 3))
    pos[0] = [0, 0, np.pi/2]
    # bool used to only determine the position of the origin once to eliminate a bit of noise
    originFound = False
    # dictionary of aruco types
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
    # constructor that takes the marker width and the aruco type

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

    def fixAngle(self, angle):
        # return an angle to -pi and pi
        while(angle > np.pi):
            angle -= 2*np.pi
        while(angle < -np.pi):
            angle += 2*np.pi
        return angle

    def find_markerPos(self, frame, allPos=None):
        # accepts a frame and locates markers and updates their positions and draws their position and info onto the frame
        # converts to gray scale and finds the aruco markers
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        (corners, ids, rejectedImgPoints) = cv2.aruco.detectMarkers(gray, self.arucoDict, parameters=self.arucoParams)

        # ads the corners to the dictionary if they are detected
        if len(corners) > 0:
            ids.flatten()
            for i in range(len(ids)):
                self.Corners[ids[i][0]] = corners[i]
        # only calculates position if the origin is found. Backup condition if the origin id has found markers because the origin will be marked as found inside the loop (second condition isn't checked if first condition is true)
        if self.originFound or len(self.Corners[10]) != 0:
            # locates the position of the origin marker only once to eliminate a bit of noise (if camera is not rigid and it moves this should be changed)
            if not self.originFound:
                # gets the rotation and translation vector of the origin marker
                self.originR, self.originT, markerpos = cv2.aruco.estimatePoseSingleMarkers(self.Corners[10], self.markerWidth, self.mtx, self.dist)
                # calculates rotation matrix from the rotation vector
                self.rodrigues = cv2.Rodrigues(self.originR[0][0])[0]
                self.originFound = True
            # calculates position of all other markers
            for i in range(1, self.NUMMARKERS):
                # checks if there is a marker found at the specific id
                if len(self.Corners[10+i]) != 0:
                    # finds marker position in the camera reference frame
                    rvec, tvec, markerpos = cv2.aruco.estimatePoseSingleMarkers(self.Corners[i+10], self.markerWidth, self.mtx, self.dist)
                    # finds the difference in position between the origin and the marker and rotates it to the origin reference frame
                    position = np.matmul(self.rodrigues, tvec[0][0]-self.originT[0][0])
                    # rotation matrix of the marker
                    Rod = cv2.Rodrigues(rvec[0][0])[0]
                    # multiply the rotation matrix of the marker with the rotation matrix of the origin and convert it back to a rotation vector. R_Z is the heading and the heading of the origin marker is added to the heading
                    heading = cv2.Rodrigues(np.matmul(self.rodrigues, Rod))[0][2] + np.pi/2
                    # updates the position of the marker
                    self.pos[i] = [position[0], position[1], self.fixAngle(heading)[0]]
                    # drawing axis on the markers
                    cv2.drawFrameAxes(frame, self.mtx, self.dist, Rod, tvec, self.markerWidth)
        # draws the marker outlines, ids, and position onto the frame

        if self.originFound:
            # draws the axis on the origin marker
            cv2.drawFrameAxes(frame, self.mtx, self.dist, self.rodrigues, self.originT[0][0], self.markerWidth * 5)
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        # calculating FPS and drawing it onto the frame
        self.endTime = time.perf_counter()
        dt = (self.endTime - self.startTime)
        self.startTime = self.endTime
        # preventing division by zero error
        if dt != 0:
            cv2.putText(frame, "FPS: " + format(1/dt, '.2f'), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        # prints data
        LINE_UP = '\033[1A'
        LINE_CLEAR = '\x1b[2K'

        #printPositions = ""

        #for i in range(1, NUM_ROBOTS + 1):
            #printPositions = printPositions + "(" + format(self.pos[i][0], '.2f') + ", " + format(self.pos[i][1], '.2f') + ", " + format(self.pos[i][2], '.2f') + ")"

        #print(LINE_UP + LINE_CLEAR + printPositions)

        # Old Printing Method:
        #print(LINE_UP + LINE_CLEAR + "(" + format(self.pos[1][0], '.2f') + ", " + format(self.pos[1][1], '.2f') + ", " + format(self.pos[1][2], '.2f') + ")" + "(" + format(self.pos[2][0], '.2f') + ", " +
        #      format(self.pos[2][1], '.2f') + ", " + format(self.pos[2][2], '.2f') + ")" + "(" + format(self.pos[3][0], '.2f') + ", " + format(self.pos[3][1], '.2f') + ", " + format(self.pos[3][2], '.2f') + ")"
        #      + "(" + format(self.pos[4][0], '.2f') + ", " + format(self.pos[4][1], '.2f') + ", " + format(self.pos[4][2], '.2f') + ")")
        
        if allPos is not None:
            frame = self.draw_grid(frame, allPos)
        return frame

    def draw_grid(self, frame, allPos, grid_step=0.25):
        if not self.originFound:
            return frame

        x_all = allPos[:, 1::2]
        y_all = allPos[:, 2::2]
        x_min = np.floor(x_all.min() / grid_step) * grid_step
        x_max = np.ceil(x_all.max() / grid_step) * grid_step
        y_min = np.floor(y_all.min() / grid_step) * grid_step
        y_max = np.ceil(y_all.max() / grid_step) * grid_step

        grid_x = np.arange(x_min, x_max + grid_step, grid_step)
        grid_y = np.arange(y_min, y_max + grid_step, grid_step)

        for x in grid_x:
            pts = np.array([[x, y, 0.0] for y in grid_y], dtype=np.float32)
            imgpts, _ = cv2.projectPoints(pts, self.originR[0][0], self.originT[0][0],
                                        self.mtx, self.dist)
            imgpts = imgpts.reshape(-1, 2).astype(int)
            for i in range(len(imgpts) - 1):
                cv2.line(frame, tuple(imgpts[i]), tuple(imgpts[i+1]), (80, 80, 80), 1)

        for y in grid_y:
            pts = np.array([[x, y, 0.0] for x in grid_x], dtype=np.float32)
            imgpts, _ = cv2.projectPoints(pts, self.originR[0][0], self.originT[0][0],
                                        self.mtx, self.dist)
            imgpts = imgpts.reshape(-1, 2).astype(int)
            for i in range(len(imgpts) - 1):
                cv2.line(frame, tuple(imgpts[i]), tuple(imgpts[i+1]), (80, 80, 80), 1)

        for x in grid_x:
            for y in grid_y:
                pt, _ = cv2.projectPoints(np.array([[x, y, 0.0]], dtype=np.float32),
                                        self.originR[0][0], self.originT[0][0],
                                        self.mtx, self.dist)
                px, py = pt.reshape(2).astype(int)
                cv2.circle(frame, (px, py), 2, (80, 80, 80), -1)
                if x == 0 or y == 0:
                    cv2.putText(frame, f"({x:.2f},{y:.2f})", (px + 3, py - 3),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 220, 220), 1)

        colors = [(0,0,255),(0,255,0),(255,0,0),(0,255,255),(255,0,255),(255,255,0)]
        for robot_idx in range(NUM_ROBOTS):
            x_col = 2 * robot_idx + 1
            y_col = 2 * robot_idx + 2
            path_pts = np.array([[allPos[i, x_col], allPos[i, y_col], 0.0]
                                for i in range(len(allPos))], dtype=np.float32)
            imgpts, _ = cv2.projectPoints(path_pts, self.originR[0][0], self.originT[0][0],
                                        self.mtx, self.dist)
            imgpts = imgpts.reshape(-1, 2).astype(int)
            for pt in imgpts:
                cv2.circle(frame, tuple(pt), 3, colors[robot_idx], -1)
            for i in range(len(imgpts) - 1):
                cv2.line(frame, tuple(imgpts[i]), tuple(imgpts[i+1]), colors[robot_idx], 1)

        return frame

    def startThreads(self):
        # starts threads for reading in new frames, displaying frames, processing frames, and sending data to the server
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
        t4 = Thread(target=self.checkReady)
        t4.daemon = False
        t4.start()
        return self

    # ends the thread for put requests

    def stopThread(self):
        # stops all threads
        self.Stop = True
        self.vs.stop()
        self.vs.stream.release()
        cv2.destroyAllWindows()

    # sends a put request to the server with the locations of all markers at a poll rate of 20Hz

    def runPutThread(self):
        prevTime = time.time()
        while not self.Stop:
            if (time.time() - prevTime) > 0.05:
                prevTime = time.time()
                data = {"id": 1, "pos": self.pos[1:].tolist()}
                requests.put(self.address + "allPos/1", json=data)

    def runProcessFrame(self):
        # finds markers in the most recent frame in a loop
        while(True):
            if self.Stop:
                return
            if self.vs.grabbed:
                self.outFrame = self.find_markerPos(self.vs.frame)

    def runGetFrame(self, frameRate):
        if self.wideAngle:
            Focus = 20
        else:
            Focus = 0
        # initializes the video stream
        self.vs = WebcamVideoStream(src=1, fps=frameRate, focus=Focus).start()
        self.vs.start()
        # sets an initial outframe to prevent crashing before the first frame is processed
        self.outFrame = self.vs.frame

    def runShowFrame(self):
        prevTime = time.time()
        frameDelta = 1/self.frameRate
        #output = cv2.VideoWriter("formation1.avi", cv2.VideoWriter_fourcc(*'MJPG'), 20, (1280, 720))
        while(True):
            # stops loop if thread is stopped
            if self.Stop:
                return
            # shows frame
            if self.vs.grabbed:
                prevTime = time.time()
                cv2.imshow('frame', self.outFrame)
                # output.write(self.outFrame)
            # stops all threads when q is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.stopThread()
                break
            # reloads the origin position if r is pressed
            if cv2.waitKey(1) & 0xFF == ord('r'):
                self.originFound = False
            sleepTime = frameDelta - (time.time() - prevTime)
            time.sleep(sleepTime*(sleepTime > 0))
        return self

    # checks if all NUM_ROBOTS agents are ready and then sets the start flag to true
    def checkReady(self):
        prevTime = time.time()
        connected = set()
        while True:
            if (time.time() - prevTime) > 2:
                prevTime = time.time()
                req = requests.get(self.address+"agentReady")
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
                        requests.put(self.address+"agentGo/"+str(int(i+1)), json={'id': i+1, 'ready': 1})
                    break
