"""
Code authored by Keegan Kelly
"""
import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

from tracker import Tracker
import requests
import pandas as pd
import numpy as np
from math import ceil
import time
filename = 'xlsxPaths/square.xlsx'
address = 'http://192.168.0.101:3000/'

NUM_ROBOTS = 6

# prompt user for if they are using a fisheye lens
invalid = False
while (True):
    if invalid == True:
        input("Invalid input. Is the lens a wide angle lens (120 fov)? (y/n): ")
    else:
        wideAngle = input('Is the lens a wide angle lens (120 fov)? (y/n): ')
    LINE_UP = '\033[1A'
    LINE_CLEAR = '\x1b[2K'
    print(LINE_UP, end=LINE_CLEAR)
    if wideAngle == 'y' or wideAngle == 'Y':
        wideAngle = True
        break
    elif wideAngle == 'n' or wideAngle == 'N':
        wideAngle = False
        break
    else:
        invalid = True
        continue

# prompt user for how often they want robots to localize (update every x positions in the path)
invalid = False
while (True):
    if invalid == True:
        updateRate = input("Invalid input. After how many movements should the robots localize? (1-5): ")
    else:
        updateRate = input('After how many movements should the robots localize? (1-5): ')
    LINE_UP = '\033[1A'
    LINE_CLEAR = '\x1b[2K'
    print(LINE_UP, end=LINE_CLEAR)
    if len(updateRate) == 1 and ord(updateRate) >= 49 and ord(updateRate) <= 53:
        updateRate = int(updateRate)
        break
    else:
        invalid = True
        continue

# puts the target positions onto the server
df = pd.read_excel(filename)
allPos = df.to_numpy()
dt = allPos[1, 0] - allPos[0, 0]
# split the arrays into chunks of size 15
numChunks = ceil(len(allPos) / 15)
allPos = np.array(np.array_split(allPos, numChunks), dtype=object)
for i in range(numChunks):
    # needs a temp array because the splicing gets screwed up because the split doesnt ensure constant size
    temp = allPos[i]

    for j in range(NUM_ROBOTS):
        # posts the sub array
        # format of the json is id (index of the chunk), total (total number of chunks), dt (time between positions), and pos (the position of the target)
        data = {'id': i+1, 'total': numChunks, 'dt': dt, 'update': updateRate, 'path': temp[:, 2*j+1:2*j+3].tolist()}
        resp = requests.put(address+"goal"+str(j+1)+"/"+str(i+1), json=data)
        # sends a post if there is a 404 because there isnt a field with the id
        if resp.status_code == 404:
            requests.post(address+"goal"+str(j+1)+"/", json=data)

# checks if there is data on the server beyond the last chunk and then deletes it
for i in range(NUM_ROBOTS):
    j = numChunks
    while True:
        # HEAD request returns the header from if a GET request is made. If it the code is 404 then there is no data on the server
        resp = requests.head(address+"goal"+str(i+1)+"/"+str(j+1))
        if resp.status_code == 404:
            break
        # code wasnt 404 so there is data in that slot so delete it and increment j to check next id
        else:
            requests.delete(address+"goal"+str(i+1)+"/"+str(j+1))
            j += 1

# sets the go signal to false
for i in range(NUM_ROBOTS):
    requests.put(address+"agentGo/"+str(int(i+1)), json={'id': i+1, 'ready': 0})
# prompt the user to ask if each agent is being used
for i in range(NUM_ROBOTS):
    invalid = False
    while True:
        if invalid:
            Input = input("Invalid input, try again. Is agent " + str(i+1) + " being used? (y/n): ")
        else:
            Input = input("Is agent " + str(i+1) + " being used? (y/n): ")
        LINE_UP = '\033[1A'
        LINE_CLEAR = '\x1b[2K'
        if i < (NUM_ROBOTS - 1):
            print(LINE_UP, end=LINE_CLEAR)
        if Input == 'y' or Input == 'Y':
            data = {'id': i+1, 'ready': 0}
            print("", end='\r')
            break
        elif Input == 'n' or Input == 'N':
            data = {'id': i+1, 'ready': 1}
            print("", end='\r')
            break
        # check if some other character was entered
        else:
            invalid = True
            # go back to the beginning of the loop
            continue
    requests.put(address+"agentReady/" + str(i+1), json=data)
# declares the aruco tracker class
tracker = Tracker(marker_width=0.1585, aruco_type="DICT_4X4_1000", address=address, wideAngle=wideAngle)
# starts threads for reading frames, outputing frames, processing frames, and sending data to the server
tracker.startThreads()
