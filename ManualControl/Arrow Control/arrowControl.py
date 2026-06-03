"""
Direct Motor Control Script
Arrow keys drive the robot in real time.
Up/Down = forward/backward, Left/Right = spin in place.
Usage: python driveControl.py
"""

from flask import Flask, request, jsonify
from threading import Thread
import time
import logging
import keyboard

logging.getLogger("werkzeug").setLevel(logging.ERROR)

HOST = "192.168.0.102"
PORT = 3000
ROBOT_ID = 4

# Linear and angular velocity sent to the robot
V = 1.0   # forward/backward speed
W = 3.0   # spin speed

app = Flask(__name__)

state = {
    "v": 0.0,
    "w": 0.0,
    "connected": False,
}


# ── Server Routes ─────────────────────────────────────────────────────────────

@app.route("/agentGo/<id>", methods=["GET", "PUT"])
def agent_go(id):
    return jsonify({"id": int(id), "ready": 0})


@app.route("/driveCmd/<id>", methods=["GET"])
def drive_cmd(id):
    if not state["connected"]:
        state["connected"] = True
        print(f"\n  Robot {ROBOT_ID} connected! Use arrow keys to drive.")
        print("  UP    = forward")
        print("  DOWN  = backward")
        print("  LEFT  = spin left")
        print("  RIGHT = spin right")
        print("  ESC   = exit\n")
    return jsonify({"v": state["v"], "w": state["w"]})


@app.route("/agentReady/<id>", methods=["GET", "PUT"])
def agent_ready(id):
    return jsonify({"id": int(id), "ready": 1})


@app.route("/agentsLocal/<id>", methods=["PUT"])
def agents_local(id):
    return jsonify(request.json)


@app.route("/agents/<id>", methods=["GET", "PUT"])
def agents(id):
    return jsonify({"id": int(id), "position": [0.0, 0.0, 0.0]})


# ── Control Loop ──────────────────────────────────────────────────────────────

def control_loop():
    print(f"Server running on {HOST}:{PORT}")
    print(f"Waiting for Robot {ROBOT_ID} to connect...\n")

    while not state["connected"]:
        time.sleep(0.5)

    while True:
        try:
            if keyboard.is_pressed("up"):
                state["v"] = V
                state["w"] = 0.0

            elif keyboard.is_pressed("down"):
                state["v"] = -V
                state["w"] = 0.0

            elif keyboard.is_pressed("left"):
                state["v"] = 0.0
                state["w"] = W

            elif keyboard.is_pressed("right"):
                state["v"] = 0.0
                state["w"] = -W

            else:
                state["v"] = 0.0
                state["w"] = 0.0

            if keyboard.is_pressed("esc"):
                state["v"] = 0.0
                state["w"] = 0.0
                print("Goodbye.")
                break

            time.sleep(0.05)

        except KeyboardInterrupt:
            state["v"] = 0.0
            state["w"] = 0.0
            print("\nExiting.")
            break


if __name__ == "__main__":
    server = Thread(
        target=lambda: app.run(host=HOST, port=PORT, debug=False, use_reloader=False),
        daemon=True
    )
    server.start()
    time.sleep(1)

    control_loop()
