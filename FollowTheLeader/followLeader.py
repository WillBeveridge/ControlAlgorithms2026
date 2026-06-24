"""
followLeader.py
Host script for the follow-leader test.
- Generates a circle path and uploads it to the server for Robot 1
- Starts the camera tracker (same as main.py)
- Waits for both robots to report ready, then sends the go signal
"""

import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

from tracker import Tracker
import requests
import math
import time

# ── Config ────────────────────────────────────────────────────────────────────
address     = 'http://192.168.0.101:3000/'
NUM_ROBOTS  = 2

# Circle parameters - adjust radius to keep the leader inside the camera view
# Origin is the ArUco marker with id 10, which should be near the centre of the frame
CIRCLE_RADIUS  = 0.3   # metres
NUM_WAYPOINTS  = 12    # points around the circle
UPDATE_RATE    = 3     # leader localizes every this many waypoints
DT             = 0.5   # time step (seconds) - not used by leader logic but stored on server
# ──────────────────────────────────────────────────────────────────────────────

# Generate circle waypoints centred at origin
circle_path = [
    [
        round(CIRCLE_RADIUS * math.cos(2 * math.pi * i / NUM_WAYPOINTS), 4),
        round(CIRCLE_RADIUS * math.sin(2 * math.pi * i / NUM_WAYPOINTS), 4)
    ]
    for i in range(NUM_WAYPOINTS)
]

# Upload circle path to server for Robot 1 (single chunk)
goal_data = {
    'id':     1,
    'total':  1,
    'dt':     DT,
    'update': UPDATE_RATE,
    'path':   circle_path
}
resp = requests.put(address + 'goal1/1', json=goal_data)
if resp.status_code not in (200, 201):
    # Slot didn't exist yet - use POST instead
    requests.post(address + 'goal1/1', json=goal_data)

# Clean up any leftover chunks beyond index 1
j = 2
while True:
    r = requests.head(address + 'goal1/' + str(j))
    if r.status_code == 404:
        break
    requests.delete(address + 'goal1/' + str(j))
    j += 1

print(f"Circle path uploaded ({NUM_WAYPOINTS} waypoints, radius {CIRCLE_RADIUS}m)")

# Reset agentGo and agentReady for both robots
for i in range(1, NUM_ROBOTS + 1):
    requests.put(address + f'agentGo/{i}',    json={'id': i, 'ready': 0})
    requests.put(address + f'agentReady/{i}', json={'id': i, 'ready': 0})

# Mark all unused robot slots (3-6) as ready so checkReady in tracker doesn't wait for them
# (tracker.py checks NUM_ROBOTS which is hardcoded to 6 in that file)
for i in range(NUM_ROBOTS + 1, 7):
    requests.put(address + f'agentReady/{i}', json={'id': i, 'ready': 1})
    requests.put(address + f'agentGo/{i}',    json={'id': i, 'ready': 1})

# Ask about wide angle lens (same as main.py)
while True:
    wideAngle = input('Is the lens a wide angle lens (120 fov)? (y/n): ').strip().lower()
    if wideAngle in ('y', 'n'):
        break
wideAngle = (wideAngle == 'y')

# Start camera tracking - reuses existing Tracker class unchanged
tracker = Tracker(
    marker_width=0.1585,
    aruco_type="DICT_4X4_1000",
    address=address,
    wideAngle=wideAngle
)
tracker.startThreads()

# Wait for both robots to reach start position and report ready
print("Waiting for robots to reach start position...")
connected = set()
while True:
    time.sleep(2)
    try:
        data = requests.get(address + 'agentReady').json()
        for i in range(NUM_ROBOTS):
            if data[i]['ready'] == 1 and i not in connected:
                connected.add(i)
                print(f"Robot {i+1} ready!")
        if len(connected) == NUM_ROBOTS:
            print("Both robots ready - sending GO signal!")
            for i in range(1, NUM_ROBOTS + 1):
                requests.put(address + f'agentGo/{i}', json={'id': i, 'ready': 1})
            break
    except Exception as e:
        print(f"Server error: {e}, retrying...")
