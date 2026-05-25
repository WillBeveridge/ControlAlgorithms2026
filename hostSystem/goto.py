#!/usr/bin/env python3
"""
goto.py — Combined tracker + robot navigation controller.
Replaces main.py. Run alongside api.py only.

Usage:
    python goto.py              # will ask which agent at startup
    python goto.py --agent 2    # skip the startup prompt

Interactive commands:
    go <x> <y>   — Send the robot to a coordinate
    pos          — Print the robot's current camera-tracked position
    quit         — Exit (also stops the tracker)

Press 'q' in the camera window to quit.
Press 'r' in the camera window to reset the origin marker.
"""

import time
import argparse
import requests
from tracker import Tracker

# ── Config ────────────────────────────────────────────────────────────────────
SERVER        = "http://192.168.0.100:3000"
PATH_STEPS    = 10     # interpolated waypoints between current pos and target
DT            = 0.5    # seconds between waypoints (matches db.json)
READY_TIMEOUT = 30     # seconds to wait for robot to reach staging
NUM_ROBOTS    = 6
# ─────────────────────────────────────────────────────────────────────────────


# ── API helpers ───────────────────────────────────────────────────────────────

def reset_signals(agent_id: int) -> None:
    requests.put(f"{SERVER}/agentGo/{agent_id}",
                json={"id": agent_id, "ready": 0}, timeout=3)
    requests.put(f"{SERVER}/agentReady/{agent_id}",
                json={"id": agent_id, "ready": 0}, timeout=3)

def send_path(agent_id: int, waypoints: list, update: int) -> None:
    """
    PUT /goal{id}/1
    path[0] = current position so the robot's staging step is instant.
    """
    payload = {
        "id":     1,        # chunk index (not agent ID) — RUNME uses this as the loop counter
        "path":   [[round(x, 4), round(y, 4)] for x, y in waypoints],
        "dt":     DT,
        "update": update,
        "total":  1,
    }
    resp = requests.put(f"{SERVER}/goal{agent_id}/1", json=payload, timeout=3)
    resp.raise_for_status()


def wait_for_ready(agent_id: int, timeout: float = READY_TIMEOUT) -> bool:
    """Poll agentReady until the robot signals ready=1. Returns False on timeout."""
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


# ── Path generation ───────────────────────────────────────────────────────────

def make_path(cx: float, cy: float,
            tx: float, ty: float,
            steps: int = PATH_STEPS) -> list:
    """
    Straight-line interpolation from current pos to target.
    Returns `steps` waypoints ending exactly at the target.
    """
    return [
        (cx + (tx - cx) * i / steps,
         cy + (ty - cy) * i / steps)
        for i in range(1, steps + 1)
    ]


# ── Main REPL ─────────────────────────────────────────────────────────────────

def run(tracker: Tracker, agent_id: int, steps: int, update_rate: int) -> None:
    print(f"\ngoto.py  |  Agent {agent_id}  |  {SERVER}")
    print("Commands:  go <x> <y>   pos   quit\n")

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
            x = tracker.pos[agent_id][0]
            y = tracker.pos[agent_id][1]
            t = tracker.pos[agent_id][2]
            print(f"  Agent {agent_id}  x={x:.4f}  y={y:.4f}  θ={t:.4f}")

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

            # Read current position directly from tracker (no HTTP round-trip)
            cx = tracker.pos[agent_id][0]
            cy = tracker.pos[agent_id][1]
            print(f"  Current  ({cx:.4f}, {cy:.4f})")
            print(f"  Target   ({tx:.4f}, {ty:.4f})")

            # 1. Reset stale signals from last run
            try:
                reset_signals(agent_id)
            except requests.RequestException as e:
                print(f"  Error resetting signals: {e}")
                continue

            # 2. Write path — path[0] is current pos so staging is instant
            waypoints = make_path(cx, cy, tx, ty, steps=steps)
            try:
                send_path(agent_id, waypoints, update_rate)
                print(f"  Path written  ({len(waypoints)} waypoints)")
            except requests.RequestException as e:
                print(f"  Error writing path: {e}")
                continue

            # 3. Wait for robot to loop back, fetch path, reach staging, setReady()
            print(f"  Waiting for Agent {agent_id} to reach staging...",
                end="", flush=True)
            if not wait_for_ready(agent_id):
                print(f"\n  Timed out after {READY_TIMEOUT}s — is the robot running?")
                continue
            print(" ready!")

            # 4. Fire go signal — robot starts executing the path
            try:
                send_go(agent_id)
                print(f"  Go!  Agent {agent_id} is moving.")
            except requests.RequestException as e:
                print(f"  Error sending go: {e}")

        # ── unknown ───────────────────────────────────────────────────────────
        else:
            print(f"  Unknown command: '{cmd}'")
            print("  Commands:  go <x> <y>   pos   quit")


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    global SERVER

    parser = argparse.ArgumentParser(
        description="Tracker + robot navigation controller. Replaces main.py."
    )
    parser.add_argument(
        "--agent", type=int, default=None, metavar="ID",
        help="Agent ID to start with, 1-6 (will prompt if not provided)"
    )
    parser.add_argument(
        "--steps", type=int, default=PATH_STEPS, metavar="N",
        help=f"Interpolated waypoints in path (default: {PATH_STEPS})"
    )
    parser.add_argument(
        "--server", default=SERVER, metavar="URL",
        help=f"API server URL (default: {SERVER})"
    )
    args = parser.parse_args()

    SERVER = args.server
    address = SERVER + "/"

    # ── Ask lens type (same as original main.py) ──────────────────────────────
    while True:
        ans = input("Is the lens a wide angle lens (120 fov)? (y/n): ")
        if ans.lower() in ("y", "n"):
            wide_angle = ans.lower() == "y"
            break
        print("Invalid input.")

    # ── Ask localization update rate (same as original main.py) ──────────────
    while True:
        ans = input("After how many movements should the robot localize? (1-5): ")
        if ans.isdigit() and 1 <= int(ans) <= 5:
            update_rate = int(ans)
            break
        print("Invalid input.")

    # ── Ask which agent to start with (if not passed as --agent) ─────────────
    agent_id = args.agent
    if agent_id is None:
        while True:
            ans = input(f"Which agent do you want to control? (1-{NUM_ROBOTS}): ")
            if ans.isdigit() and 1 <= int(ans) <= NUM_ROBOTS:
                agent_id = int(ans)
                break
            print("Invalid input.")

    # ── Reset all go signals (not agentReady — robots set that themselves) ──────
    for i in range(1, NUM_ROBOTS + 1):
        requests.put(f"{address}agentGo/{i}", json={"id": i, "ready": 0})

    # ── Start tracker without checkReady ─────────────────────────────────────
    # checkReady fires go for ALL robots at once — goto.py handles this
    # per-robot instead, so we pass check_ready=False to skip that thread.
    print("Starting tracker... (ensure origin marker is visible in frame)")
    tracker = Tracker(
        marker_width=0.1585,
        aruco_type="DICT_4X4_1000",
        address=address,
        wideAngle=wide_angle
    )
    tracker.startThreads(check_ready=False)

    # ── Run interactive command loop ──────────────────────────────────────────
    run(tracker, agent_id, args.steps, update_rate)


if __name__ == "__main__":
    main()
