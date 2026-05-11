import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

from tracker import Tracker
import cv2
import pandas as pd
import numpy as np

df = pd.read_excel('xlsxPaths/square.xlsx')
allPos = df.to_numpy()

tracker = Tracker(marker_width=0.1585, aruco_type="DICT_4X4_1000",
                address="http://192.168.0.101:3000/", fps=60)

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

while True:
    ret, frame = cap.read()
    if ret:
        frame = tracker.find_markerPos(frame, allPos)
        cv2.imshow("frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()