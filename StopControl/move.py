#!/usr/bin/env python3
"""
move.py — Combined tracker + robot navigation controller.
Run alongside api2.py only.

Usage:
    python move.py              # will ask which agent at startup
    python move.py --agent 2    # skip the startup prompt

Interactive commands:
    go <x> <y>   — Send the robot to a coordinate
    pos          — Print the robot's current camera-tracked position
    obstacle     — Print the current obstacle position and distance to this robot
    quit         — Exit (also stops the tracker)

Press 'q' in the camera window to quit.
Press 'r' in the camera window to reset the origin marker.
Press 'c' in the camera window to redo spatial calibration.

The tracker's checkSafety thread runs continuously and will set agentStop=1 for
any two markers (robots or obstacle) that come within STOP_DISTANCE of each other.
The robot polls agentStop inside moveTo and halts immediately when it fires.
"""

import time
import argparse
import requests
import threading
import numpy as np
from tracker2 import Tracker, STOP_DISTANCE

# ── Config ────────────────────────────────────────────────────────────────────
SERVER        = "http://192.168.0.101:3000"
PATH_STEPS    = 5     # interpolated waypoints between current pos and target
DT            = 0.5   # seconds between waypoints (matches db.json)
READY_TIMEOUT = 30    # seconds to wait for robot to reach staging
NUM_ROBOTS    = 6
# ─────────────────────────────────────────────────────────────────────────────


# ── API helpers ───────────────────────────────────────────────────────────────

def send_path(agent_id: int, waypoints: list, update: int) -> None:
    payload = {
        "id":     1,
        "path":   [[round(x, 4), round(y, 4)] for x, y in waypoints],
        "dt":     DT,
        "update": update,
        "total":  1,
    }
    resp = requests.put(f"{SERVER}/goal{agent_id}/1", json=payload, timeout=3)
    resp.raise_for_status()


def wait_for_ready(agent_id: int, timeout: float = READY_TIMEOUT) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            resp = requests.get(f"{SERVER}/agentReady/{agent_id}", timeout=3)
            if resp.json().get("ready") == 1:
                return True
        except requests.RequestException:
            pass
        time.sleep(0.1)
    return False


def send_go(agent_id: int) -> None:
    requests.put(f"{SERVER}/agentGo/{agent_id}",
                 json={"id": agent_id, "ready": 1}, timeout=3)
    time.sleep(1.0)
    requests.put(f"{SERVER}/agentGo/{agent_id}",
                 json={"id": agent_id, "ready": 0}, timeout=3)


# ── Path generation ───────────────────────────────────────────────────────────

def make_path(cx: float, cy: float, tx: float, ty: float,
              steps: int = PATH_STEPS) -> list:
    """Straight-line interpolation from current position to target."""
    return [
        (cx + (tx - cx) * i / steps,
         cy + (ty - cy) * i / steps)
        for i in range(1, steps + 1)
    ]


# ── Main REPL ─────────────────────────────────────────────────────────────────

def run(tracker: Tracker, agent_id: int, steps: int, update_rate: int) -> None:
    print(f"\nmove.py  |  Agent {agent_id}  |  {SERVER}")
    print("Commands:  go <x> <y>   pos   obstacle   quit\n")

    while True:
        try:
            raw = input(">>> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nStopping tracker...")
            tracker.stopThread()
            break

        if not raw:
            continue

        parts = raw.split()
        cmd   = parts[0].lower()

        # ── quit ──────────────────────────────────────────────────────────────
        if cmd in ("quit", "exit", "q"):
            print("Stopping tracker...")
            tracker.stopThread()
            break

        # ── pos ───────────────────────────────────────────────────────────────
        elif cmd == "pos":
            x, y, t = tracker.pos[agent_id]
            print(f"  Agent {agent_id}  x={x:.4f}  y={y:.4f}  θ={t:.4f}")

        # ── obstacle ──────────────────────────────────────────────────────────
        elif cmd == "obstacle":
            if tracker.obstacleFound and tracker.obstaclePos is not None:
                ox, oy, ot = tracker.obstaclePos
                robot_xy = np.array(tracker.pos[agent_id][:2])
                obs_xy   = np.array([ox, oy])
                dist     = float(np.linalg.norm(robot_xy - obs_xy))
                print(f"  Obstacle (ArUco 18)  x={ox:.4f}  y={oy:.4f}  θ={ot:.4f}")
                print(f"  Distance from Agent {agent_id}: {dist:.4f} m  "
                      f"(stop at {STOP_DISTANCE} m)")
            else:
                print("  Obstacle (ArUco 18) not yet detected in frame.")

        # ── go <x> <y> ────────────────────────────────────────────────────────
        elif cmd == "go":
            if len(parts) < 3:
                print("  Usage: go <x> <y>")
                continue
            try:
                tx, ty = float(parts[1]), float(parts[2])
            except ValueError:
                print("  x and y must be numbers, e.g.:  go 0.5 1.2")
                continue

            cx, cy = tracker.pos[agent_id][0], tracker.pos[agent_id][1]
            print(f"  Current  ({cx:.4f}, {cy:.4f})")
            print(f"  Target   ({tx:.4f}, {ty:.4f})")

            waypoints = make_path(cx, cy, tx, ty, steps=steps)

            # 1. Write path to server
            try:
                send_path(agent_id, waypoints, update_rate)
                print(f"  Path written  ({len(waypoints)} waypoints)")
            except requests.RequestException as e:
                print(f"  Error writing path: {e}")
                continue

            # 2. Reset signals after path is written
            try:
                tracker.clearStoppedRobot(agent_id)
                requests.put(f"{SERVER}/agentStop/{agent_id}",
                             json={"id": agent_id, "stop": 0}, timeout=3)
                requests.put(f"{SERVER}/agentGo/{agent_id}",
                             json={"id": agent_id, "ready": 0}, timeout=3)
                requests.put(f"{SERVER}/agentReady/{agent_id}",
                             json={"id": agent_id, "ready": 0}, timeout=3)
            except requests.RequestException as e:
                print(f"  Error resetting signals: {e}")
                continue

            # 3. Wait for robot to call setReady()
            print(f"  Waiting for Agent {agent_id} to reach staging...",
                  end="", flush=True)
            if not wait_for_ready(agent_id):
                print(f"\n  Timed out after {READY_TIMEOUT}s — is the robot running?")
                continue
            print(" ready!")

            # 4. Fire go signal
            try:
                send_go(agent_id)
                print(f"  Go!  Agent {agent_id} is moving.")
                print("  (press Enter to stop monitoring)")
                stop_monitor = threading.Event()

                def monitor():
                    while not stop_monitor.is_set():
                        x, y, t = tracker.pos[agent_id]
                        obs_str = ""
                        if tracker.obstacleFound and tracker.obstaclePos is not None:
                            d = float(np.linalg.norm(
                                np.array([x, y]) - np.array(tracker.obstaclePos[:2])
                            ))
                            obs_str = f"  obs_dist={d:.3f} m"
                        print(f"  [pos]  x={x:.4f}  y={y:.4f}  θ={t:.4f}  "
                              f"target=({tx:.4f},{ty:.4f}){obs_str}")
                        time.sleep(2.0)

                m = threading.Thread(target=monitor, daemon=True)
                m.start()
                input()
                stop_monitor.set()
                print("  Monitoring stopped.")
            except requests.RequestException as e:
                print(f"  Error sending go: {e}")

        # ── unknown ───────────────────────────────────────────────────────────
        else:
            print(f"  Unknown command: '{cmd}'")
            print("  Commands:  go <x> <y>   pos   obstacle   quit")


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    global SERVER

    parser = argparse.ArgumentParser(
        description="Tracker + robot navigation controller."
    )
    parser.add_argument("--agent", type=int, default=None, metavar="ID")
    parser.add_argument("--steps", type=int, default=PATH_STEPS, metavar="N")
    parser.add_argument("--server", default=SERVER, metavar="URL")
    args = parser.parse_args()

    SERVER  = args.server
    address = SERVER + "/"

    while True:
        ans = input("Is the lens a wide angle lens (120 fov)? (y/n): ")
        if ans.lower() in ("y", "n"):
            wide_angle = ans.lower() == "y"
            break
        print("Invalid input.")

    while True:
        ans = input("After how many movements should the robot localize? (1-5): ")
        if ans.isdigit() and 1 <= int(ans) <= 5:
            update_rate = int(ans)
            break
        print("Invalid input.")

    agent_id = args.agent
    if agent_id is None:
        while True:
            ans = input(f"Which agent do you want to control? (1-{NUM_ROBOTS}): ")
            if ans.isdigit() and 1 <= int(ans) <= NUM_ROBOTS:
                agent_id = int(ans)
                break
            print("Invalid input.")

    for i in range(1, NUM_ROBOTS + 1):
        requests.put(f"{address}agentStop/{i}", json={"id": i, "stop": 0})
        requests.put(f"{address}agentGo/{i}", json={"id": i, "ready": 0})
        requests.put(f"{address}agentReady/{i}", json={"id": i, "ready": 0})

    print("Starting tracker... (ensure origin marker is visible in frame)")
    tracker = Tracker(
        marker_width=0.1585,
        aruco_type="DICT_4X4_1000",
        address=address,
        wideAngle=wide_angle,
    )
    tracker.startThreads(check_ready=False)

    run(tracker, agent_id, args.steps, update_rate)


if __name__ == "__main__":
    main()
