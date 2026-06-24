"""
random_wander.py  –  soft-boundary wander mode
Uses tracker2.py for camera localisation.
Prints robot positions, speedScale, and w every 500ms for debugging.
"""

import time, math, requests
from threading import Thread
from tracker2 import Tracker

ADDRESS    = 'http://192.168.0.101:3000/'
NUM_ROBOTS = 6

# Must match RUNME3.ino
X_MIN, X_MAX    = -1.68,  1.63
Y_MIN, Y_MAX    = -0.88,  0.865
OUTER_MARGIN    =  0.45
INNER_MARGIN    =  0.20
DRIVE_SPEED     =  0.18
MAX_W           =  6.0


def get_robot_position(robot_id):
    try:
        r = requests.get(ADDRESS + f'agents/{robot_id}', timeout=1)
        if r.status_code == 200:
            pos = r.json().get('position', [0.0, 0.0, 0.0])
            return float(pos[0]), float(pos[1]), float(pos[2])
    except Exception:
        pass
    return None


def compute_boundary_debug(px, py, heading):
    """Mirror of RUNME3's computeBoundaryCorrection — returns speedScale and w."""
    zone_width = OUTER_MARGIN - INNER_MARGIN

    def depth(d):
        if d <= 0: return 0.0
        return min(d / zone_width, 1.0)

    dL = depth((X_MIN + OUTER_MARGIN) - px)
    dR = depth(px - (X_MAX - OUTER_MARGIN))
    dB = depth((Y_MIN + OUTER_MARGIN) - py)
    dT = depth(py - (Y_MAX - OUTER_MARGIN))

    max_depth   = max(dL, dR, dB, dT)
    speed_scale = 1.0 - max_depth

    if max_depth == 0:
        return speed_scale, 0.0, dL, dR, dB, dT

    rep_x = dL - dR
    rep_y = dB - dT
    w = (rep_x * (-math.sin(heading)) + rep_y * math.cos(heading)) * MAX_W
    w = max(-MAX_W, min(MAX_W, w))

    return speed_scale, w, dL, dR, dB, dT


def debug_loop(active_robots, stop_flag):
    """Prints position and boundary state for each active robot every 500ms."""
    print("\n{:<6} {:>8} {:>8} {:>7} | {:>7} {:>7} | {:>5} {:>5} {:>5} {:>5}".format(
        "Robot", "x", "y", "theta", "scale", "w", "dL", "dR", "dB", "dT"))
    print("-" * 75)

    while not stop_flag[0]:
        for rid in active_robots:
            pos = get_robot_position(rid)
            if pos:
                px, py, theta = pos
                scale, w, dL, dR, dB, dT = compute_boundary_debug(px, py, theta)
                warn = ""
                if px == 0.0 and py == 0.0:
                    warn = "  ⚠ position is 0,0 — camera may not see marker"
                elif scale < 0.1:
                    warn = "  ⚠ speed near zero — deep in boundary zone"
                print("{:<6} {:>8.3f} {:>8.3f} {:>7.3f} | {:>7.3f} {:>7.3f} | {:>5.2f} {:>5.2f} {:>5.2f} {:>5.2f}{}".format(
                    f"R{rid}", px, py, theta, scale, w, dL, dR, dB, dT, warn))
            else:
                print(f"R{rid}    -- no position data from server --")
        print()
        time.sleep(0.5)


def main():
    while True:
        ans = input('Is the lens a wide angle lens (120 fov)? (y/n): ').strip().lower()
        print('\033[1A\x1b[2K', end='')
        if ans in ('y', 'n'):
            wideAngle = (ans == 'y')
            break
        print("Invalid input.")

    for i in range(NUM_ROBOTS):
        requests.put(ADDRESS + f'agentGo/{i+1}', json={'id': i+1, 'ready': 0})

    active_robots = []
    for i in range(NUM_ROBOTS):
        while True:
            ans = input(f'Is agent {i+1} being used? (y/n): ').strip().lower()
            if i < NUM_ROBOTS - 1:
                print('\033[1A\x1b[2K', end='')
            if ans == 'y':
                active_robots.append(i + 1)
                requests.put(ADDRESS + f'agentReady/{i+1}', json={'id': i+1, 'ready': 0})
                break
            elif ans == 'n':
                requests.put(ADDRESS + f'agentReady/{i+1}', json={'id': i+1, 'ready': 1})
                break
            else:
                print("Invalid input.")

    print(f"\nActive robots: {active_robots}")
    print("Starting tracker2...\n")

    tracker = Tracker(
        marker_width=0.1585,
        aruco_type='DICT_4X4_1000',
        address=ADDRESS,
        wideAngle=wideAngle,
    )
    tracker.startThreads(check_ready=True)

    stop_flag = [False]
    t = Thread(target=debug_loop, args=(active_robots, stop_flag), daemon=True)
    t.start()

    while not tracker.Stop:
        time.sleep(0.5)

    stop_flag[0] = True
    print("Done.")


if __name__ == '__main__':
    main()
