import cv2
import time
from tracker2 import Tracker

tracker = Tracker(
    marker_width=0.1585,
    aruco_type='DICT_4X4_1000',
    address='http://192.168.0.101:3000/',
    wideAngle=False,
)

tracker.startThreads(check_ready=False)

cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
while not tracker.Stop:
    if tracker.out_frame is not None:
        cv2.imshow('frame', tracker.out_frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        tracker.stopThread()
        break

print("done")