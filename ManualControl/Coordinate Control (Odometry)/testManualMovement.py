"""
Standalone Robot Control Script
Runs a minimal server for the robot to talk to, and a control loop for you to send commands.
Usage: python control.py
"""

from flask import Flask, request, jsonify
from threading import Thread
import time
import logging

# Suppress Flask request logs so they don't clutter the prompt
logging.getLogger("werkzeug").setLevel(logging.ERROR)

HOST = "192.168.0.102"
PORT = 3000
ROBOT_ID = 4
DT = 0.5    # time step between waypoints (seconds)

app = Flask(__name__)

# go=0 so robot waits on startup
state = {
    "go": 0,
    "path": [[0.0, 0.0]],
    "ready": 0,
    "connected": False,
}


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
    response = {
        "id": 1,
        "total": 1,
        "dt": DT,
        "update": 1,
        "path": state["path"]
    }
    print(f"\n  [DEBUG] Robot requested path, serving: {state['path']}\n[Robot {ROBOT_ID}] > ", end="", flush=True)
    return jsonify(response)


@app.route("/agentReady/<id>", methods=["GET", "PUT"])
def agent_ready(id):
    if request.method == "PUT":
        state["ready"] = request.json["ready"]
    return jsonify({"id": int(id), "ready": state["ready"]})


@app.route("/agentsLocal/<id>", methods=["PUT"])
def agents_local(id):
    return jsonify(request.json)


@app.route("/agents/<id>", methods=["GET", "PUT"])
def agents(id):
    return jsonify({"id": int(id), "position": [0.0, 0.0, 0.0]})


# ── Control Loop ──────────────────────────────────────────────────────────────

def dispatch(waypoints):
    """
    1. Load new path onto server
    2. Set go=1 to trigger the robot
    3. Wait for robot to call setReady() signalling it finished
    4. Clear go=0 so robot goes back to waiting
    """
    state["ready"] = 0
    state["path"] = waypoints
    state["go"] = 1

    print("  Moving...", end="", flush=True)
    start = time.time()
    while time.time() - start < 120:
        if state["ready"] == 1:
            state["go"] = 0
            print(" done.")
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
  dt <seconds>    Change the time step between waypoints (default 0.5)
  help            Show this message
  quit / exit     Exit
""")


def control_loop():
    global DT
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
    server = Thread(
        target=lambda: app.run(host=HOST, port=PORT, debug=False, use_reloader=False),
        daemon=True
    )
    server.start()
    time.sleep(1)

    control_loop()
