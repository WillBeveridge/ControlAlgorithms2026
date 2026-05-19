"""
Coordinate Control Script
Controls a single robot by sending it to (x, y) coordinates.
Uses the camera tracker to localize the robot and confirm arrival.
Usage: python coordinateControl.py
"""

import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"

from flask import Flask, request, jsonify
from threading import Thread
from tracker import Tracker
import time
import logging

logging.getLogger("werkzeug").setLevel(logging.ERROR)

HOST = "192.168.0.102"
PORT = 3000
ROBOT_ID = 4
DT = 0.5    # time step between waypoints (seconds)
UPDATE = 1  # localize every N waypoints

app = Flask(__name__)

state = {
    "go": 0,
    "path": [[0.0, 0.0]],
    "ready": 0,
    "connected": False,
}

# stores camera position for each robot
agent_positions = {i: [0.0, 0.0, 0.0] for i in range(1, 7)}


# ── Server Routes ─────────────────────────────────────────────────────────────

@app.route("/agentGo/<id>", methods=["GET", "PUT"])
def agent_go(id):
    if not state["connected"]:
        state["connected"] = True
        print(f"\n  Robot {ROBOT_ID} connected! You can now give commands.\n[Robot {ROBOT_ID}] > ", end="", flush=True)
    if request.method == "PUT":
        state["go"] = request.json["ready"]
    return jsonify({"id": int(id), "ready": state["go"]})


@app.route(f"/goal{ROBOT_ID}/<idx>", methods=["GET"])
def goal(idx):
    return jsonify({
        "id": 1,
        "total": 1,
        "dt": DT,
        "update": UPDATE,
        "path": state["path"]
    })


@app.route("/agentReady/<id>", methods=["GET", "PUT"])
def agent_ready(id):
    if request.method == "PUT":
        state["ready"] = request.json["ready"]
    return jsonify({"id": int(id), "ready": state["ready"]})


@app.route("/agentReady", methods=["GET"])
def agent_ready_all():
    return jsonify([{"id": i, "ready": 0} for i in range(1, 7)])


@app.route("/agents/<id>", methods=["GET", "PUT"])
def agents(id):
    id = int(id)
    if request.method == "PUT":
        agent_positions[id] = request.json["position"]
    return jsonify({"id": id, "position": agent_positions[id]})


@app.route("/agentsLocal/<id>", methods=["PUT"])
def agents_local(id):
    return jsonify(request.json)


@app.route("/allPos/1", methods=["PUT"])
def all_pos():
    pos = request.json["pos"]
    for i in range(len(pos)):
        agent_positions[i + 1] = pos[i]
    return jsonify(request.json)


# ── Control Loop ──────────────────────────────────────────────────────────────

def dispatch(waypoints, tolerance=0.1):
    """
    1. Load new path onto server
    2. Set go=1 to trigger the robot
    3. Wait for camera to confirm robot arrived OR robot signals done
    4. Clear go=0 so robot loops back waiting
    """
    state["ready"] = 0
    state["path"] = waypoints
    state["go"] = 1

    target_x = waypoints[-1][0]
    target_y = waypoints[-1][1]

    print("  Moving...", end="", flush=True)
    start = time.time()
    while time.time() - start < 120:
        if state["ready"] == 1:
            state["go"] = 0
            print(" done.")
            return
        pos = agent_positions[ROBOT_ID]
        dist = ((pos[0] - target_x)**2 + (pos[1] - target_y)**2) ** 0.5
        if dist < tolerance:
            state["go"] = 0
            print(f" arrived (camera: {dist:.3f}m from target).")
            return
        time.sleep(0.5)
        print(".", end="", flush=True)

    print(" TIMEOUT.")
    state["go"] = 0


def print_help():
    print("""
Commands:
go <x> <y>      Move to a single point, e.g.  go 0.5 1.0
path            Enter multiple waypoints interactively
pos             Show current camera position of the robot
dt <seconds>    Change the time step between waypoints (default 0.5)
update <n>      Localize every n waypoints (default 1)
help            Show this message
quit / exit     Exit
""")

def control_loop():
    global DT, UPDATE
    print(f"Server running on {HOST}:{PORT}")
    print(f"Waiting for Robot {ROBOT_ID} to connect...\n")

    while True:
        try:
            raw = input(f"[Robot {ROBOT_ID}] > ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            break

        if not raw:
            continue

        parts = raw.split()
        cmd = parts[0].lower()

        if cmd in ("quit", "exit"):
            print("Goodbye.")
            break

        elif cmd == "help":
            print_help()

        elif cmd == "dt":
            if len(parts) < 2:
                print(f"  Current dt = {DT}s")
            else:
                try:
                    DT = float(parts[1])
                    print(f"  dt set to {DT}s")
                except ValueError:
                    print("  Invalid value.")

        elif cmd == "update":
            if len(parts) < 2:
                print(f"  Current update rate = every {UPDATE} waypoints")
            else:
                try:
                    UPDATE = int(parts[1])
                    print(f"  Update rate set to every {UPDATE} waypoints")
                except ValueError:
                    print("  Invalid value.")

        elif cmd == "pos":
            pos = agent_positions[ROBOT_ID]
            print(f"  Camera position: x={pos[0]:.3f}  y={pos[1]:.3f}  θ={pos[2]:.3f} rad")

        elif cmd == "go":
            if not state["connected"]:
                print("  Robot not connected yet, please wait.")
            elif len(parts) < 3:
                print("  Usage: go <x> <y>")
            else:
                try:
                    x, y = float(parts[1]), float(parts[2])
                    print(f"  Sending Robot {ROBOT_ID} to ({x}, {y})...")
                    dispatch([[x, y]])
                except ValueError:
                    print("  Invalid coordinates.")

        elif cmd == "path":
            if not state["connected"]:
                print("  Robot not connected yet, please wait.")
            else:
                print("  Enter waypoints as 'x y', one per line. Empty line to finish.")
                waypoints = []
                while True:
                    line = input("    waypoint> ").strip()
                    if not line:
                        break
                    try:
                        x, y = map(float, line.split())
                        waypoints.append([x, y])
                        print(f"    Added ({x}, {y})")
                    except ValueError:
                        print("    Invalid - enter two numbers separated by a space.")
                if waypoints:
                    print(f"  Sending {len(waypoints)}-point path to Robot {ROBOT_ID}...")
                    dispatch(waypoints)
                else:
                    print("  No waypoints entered, nothing sent.")

        else:
            print(f"  Unknown command '{cmd}'. Type 'help' for commands.")

if __name__ == "__main__":
    # ask about wide angle lens same as main.py does
    while True:
        wideAngle = input("Is the lens a wide angle lens (120 fov)? (y/n): ").strip().lower()
        if wideAngle in ("y", "n"):
            wideAngle = wideAngle == "y"
            break
        print("  Invalid input.")

    # start flask server
    server = Thread(
        target=lambda: app.run(host=HOST, port=PORT, debug=False, use_reloader=False),
        daemon=True
    )
    server.start()
    time.sleep(1)

    # start camera tracker
    tracker = Tracker(
        marker_width=0.1585,
        aruco_type="DICT_4X4_1000",
        address=f"http://{HOST}:{PORT}/",
        wideAngle=wideAngle
    )
    tracker.startThreads()

    control_loop()
