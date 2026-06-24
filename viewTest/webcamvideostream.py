# Modified version of imutils.video.WebcamVideoStream
from threading import Thread
import cv2
import time
import platform


class WebcamVideoStream:
    def __init__(self, src=1, name="WebcamVideoStream", height=1080, width=1920, fps=30, focus=0):
        if platform.system() == "Windows":
            self.stream = cv2.VideoCapture(src, cv2.CAP_DSHOW)
        else:
            self.stream = cv2.VideoCapture(src)

        self.fps = fps
        self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.stream.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.stream.set(cv2.CAP_PROP_FOCUS, focus)
        self.stream.set(cv2.CAP_PROP_FPS, fps)

        self.name = name
        self.stopped = False
        (self.grabbed, self.frame) = self.stream.read()

    def start(self):
        t = Thread(target=self.update, name=self.name, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        frameDelta = 1 / self.fps
        while True:
            if self.stopped:
                return
            prevTime = time.time()
            (self.grabbed, self.frame) = self.stream.read()
            sleepTime = frameDelta - (time.time() - prevTime)
            time.sleep(sleepTime * (sleepTime > 0))

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True
