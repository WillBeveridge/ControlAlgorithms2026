"""
Proximity checker for ArUco tracked robots.
Polls the API server and warns when any two robots are within a threshold distance.
This is a standalone script - run it alongside api.py and main.py.

Usage:
    python checkProximity.py
"""

import requests
import time
import numpy as np
from itertools import combinations

# ── Config ────────────────────────────────────────────────────────────────────
ADDRESS       = "http://192.168.0.101:3000/"
NUM_ROBOTS    = 6
WARN_DISTANCE = 0.7   # metres  – yellow warning
STOP_DISTANCE = 0.4   # metres  – red alert (robots are about to collide)
POLL_RATE_HZ  = 10     # how often to check
# ─────────────────────────────────────────────────────────────────────────────


def get_positions(address: str, num_robots: int) -> np.ndarray | None:
    """
    Fetch the latest camera-tracked positions from the API server.
    Returns an (N, 2) array of [x, y] for each robot, or None on failure.
    Positions come from /allPos/1 which the tracker updates at ~20 Hz.
    """
    try:
        resp = requests.get(address + "allPos/1", timeout=0.5)
        resp.raise_for_status()
        data = resp.json()
        pos = np.array(data["pos"])          # shape (NUM_ROBOTS, 3) → x, y, θ
        return pos[:num_robots, :2]          # keep only x, y
    except Exception as e:
        print(f"[WARN] Could not reach server: {e}")
        return None


def check_proximity(positions: np.ndarray, warn_dist: float, stop_dist: float) -> list[dict]:
    """
    Check every pair of robots and return a list of proximity events.
    Each event is a dict with keys: robot_a, robot_b, distance, level ('WARN'|'STOP').
    """
    events = []
    for i, j in combinations(range(len(positions)), 2):
        dist = float(np.linalg.norm(positions[i] - positions[j]))
        if dist <= stop_dist:
            events.append({"robot_a": i + 1, "robot_b": j + 1,
                           "distance": dist, "level": "STOP"})
        elif dist <= warn_dist:
            events.append({"robot_a": i + 1, "robot_b": j + 1,
                           "distance": dist, "level": "WARN"})
    return events


def print_status(positions: np.ndarray, events: list[dict]) -> None:
    """Pretty-print the current positions and any proximity events."""
    # Move cursor up to overwrite previous output (NUM_ROBOTS + header + blank)
    RESET  = "\033[0m"
    YELLOW = "\033[93m"
    RED    = "\033[91m"
    GREEN  = "\033[92m"

    lines = []
    lines.append("── Robot Positions ─────────────────────────────")
    for idx, (x, y) in enumerate(positions):
        lines.append(f"  Robot {idx+1}: x={x:+.3f} m   y={y:+.3f} m")
    lines.append("")
    lines.append("── Proximity Alerts ────────────────────────────")
    if not events:
        lines.append(f"  {GREEN}All clear – no robots within warning distance.{RESET}")
    else:
        for ev in events:
            color = RED if ev["level"] == "STOP" else YELLOW
            lines.append(
                f"  {color}[{ev['level']}]  Robot {ev['robot_a']} ↔ Robot {ev['robot_b']}"
                f"   dist={ev['distance']:.3f} m{RESET}"
            )
    lines.append("─" * 49)

    # Rewrite block in place
    UP = f"\033[{len(lines)}A"
    print(UP + "\n".join(lines))


def main():
    period = 1.0 / POLL_RATE_HZ
    print(f"Monitoring {NUM_ROBOTS} robots  |  "
          f"warn={WARN_DISTANCE:.2f} m  stop={STOP_DISTANCE:.2f} m  "
          f"@ {POLL_RATE_HZ} Hz\n")

    # Reserve blank lines so the first rewrite doesn't scroll
    LINES_PER_BLOCK = NUM_ROBOTS + 6
    print("\n" * LINES_PER_BLOCK, end="")

    while True:
        t0 = time.time()

        positions = get_positions(ADDRESS, NUM_ROBOTS)
        if positions is not None:
            events = check_proximity(positions, WARN_DISTANCE, STOP_DISTANCE)
            print_status(positions, events)

            # ── Hook: extend here to act on events ──────────────────────────
            # Example: send an emergency stop to robots that are too close
            #
            # for ev in events:
            #     if ev["level"] == "STOP":
            #         for robot_id in (ev["robot_a"], ev["robot_b"]):
            #             requests.put(ADDRESS + f"agentGo/{robot_id}",
            #                          json={"id": robot_id, "ready": 0})
            # ────────────────────────────────────────────────────────────────

        elapsed = time.time() - t0
        time.sleep(max(0.0, period - elapsed))


if __name__ == "__main__":
    main()
