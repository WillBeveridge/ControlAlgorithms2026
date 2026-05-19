"""
Grid Overlay Script
Draws a real-world coordinate grid onto the camera feed using the ArUco origin
marker (id 10) as the reference frame. Grid lines are in meters and align with
the positions sent to the robots.

Controls:
    q - quit
    r - re-detect origin marker
    +/= - increase grid spacing
    - - decrease grid spacing
"""
import cv2
import numpy as np
import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

# ── Configuration ──────────────────────────────────────────────────────────────
CAMERA_INDEX   = 1          # same as tracker.py
FRAME_WIDTH    = 1280
FRAME_HEIGHT   = 720
FOCUS          = 0          # set to match your calibrated focus value
FPS            = 60

GRID_SPACING   = 0.25       # metres between grid lines (matches path increments)
GRID_X_RANGE   = (-1.0, 1.0)   # metres, world x-axis extent of grid
GRID_Y_RANGE   = (-1.5, 1.5)   # metres, world y-axis extent of grid

# Visual settings
GRID_COLOR        = (0, 0, 0)        # BGR black for normal lines
AXIS_COLOR_X      = (0, 0, 255)      # red   for X axis (matches drawFrameAxes)
AXIS_COLOR_Y      = (0, 255, 0)      # green for Y axis (matches drawFrameAxes)
ORIGIN_DOT_COLOR  = (0, 255, 255)    # yellow dot at origin
LABEL_COLOR       = (255, 255, 255)  # white labels
GRID_THICKNESS    = 1
AXIS_THICKNESS    = 2
FONT              = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE        = 0.4
LABEL_STEP        = 2   # label every Nth grid line to reduce clutter
# ──────────────────────────────────────────────────────────────────────────────

# Load camera calibration
npfile = np.load("calibration.npz")
mtx    = npfile["mtx"]
dist   = npfile["dist"]

# ArUco setup — must match what was used when calibration images were captured
arucoDict     = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
arucoParams   = cv2.aruco.DetectorParameters()
arucoDetector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

# Marker physical width in metres (keep consistent with main.py / tracker.py)
MARKER_WIDTH = 0.1585


def build_grid_lines(x_range, y_range, spacing):
    """Return lists of (start, end) 3-D point pairs for every grid line."""
    xs = np.arange(x_range[0], x_range[1] + spacing * 0.5, spacing)
    ys = np.arange(y_range[0], y_range[1] + spacing * 0.5, spacing)

    lines = []  # each entry: (pt_start, pt_end, is_axis)

    # Lines parallel to Y axis (constant X)
    for x in xs:
        p0 = np.array([x, y_range[0], 0], dtype=np.float32)
        p1 = np.array([x, y_range[1], 0], dtype=np.float32)
        lines.append((p0, p1, abs(x) < spacing * 0.01))  # flag axis line

    # Lines parallel to X axis (constant Y)
    for y in ys:
        p0 = np.array([x_range[0], y, 0], dtype=np.float32)
        p1 = np.array([x_range[1], y, 0], dtype=np.float32)
        lines.append((p0, p1, abs(y) < spacing * 0.01))

    return lines, xs, ys


def project_point(pt_3d, rvec, tvec):
    """Project a single 3-D world point to 2-D image coordinates."""
    pts, _ = cv2.projectPoints(
        pt_3d.reshape(1, 1, 3), rvec, tvec, mtx, dist
    )
    return tuple(pts[0, 0].astype(int))


def draw_grid(frame, rvec, tvec, x_range, y_range, spacing):
    """Project and draw the coordinate grid onto frame."""
    lines, xs, ys = build_grid_lines(x_range, y_range, spacing)

    for (p0, p1, is_axis) in lines:
        # Colour the X and Y axis lines differently
        if is_axis and abs(p0[0]) < spacing * 0.01:   # Y-axis (x=0)
            color, thickness = AXIS_COLOR_Y, AXIS_THICKNESS
        elif is_axis and abs(p0[1]) < spacing * 0.01: # X-axis (y=0)
            color, thickness = AXIS_COLOR_X, AXIS_THICKNESS
        else:
            color, thickness = GRID_COLOR, GRID_THICKNESS

        ip0 = project_point(p0, rvec, tvec)
        ip1 = project_point(p1, rvec, tvec)

        # Only draw if at least one endpoint is within the frame
        def in_frame(p):
            return 0 <= p[0] < FRAME_WIDTH and 0 <= p[1] < FRAME_HEIGHT

        if in_frame(ip0) or in_frame(ip1):
            cv2.line(frame, ip0, ip1, color, thickness)

    # Draw coordinate labels at intersections every LABEL_STEP lines
    x_indices = np.where(np.abs(np.round(xs / spacing) % LABEL_STEP) < 0.01)[0]
    y_indices = np.where(np.abs(np.round(ys / spacing) % LABEL_STEP) < 0.01)[0]

    for xi in x_indices:
        for yi in y_indices:
            x_val, y_val = xs[xi], ys[yi]
            pt = np.array([x_val, y_val, 0], dtype=np.float32)
            ip = project_point(pt, rvec, tvec)
            if 0 <= ip[0] < FRAME_WIDTH and 0 <= ip[1] < FRAME_HEIGHT:
                label = f"({x_val:.2f},{y_val:.2f})"
                cv2.putText(frame, label, ip, FONT, FONT_SCALE,
                            LABEL_COLOR, 1, cv2.LINE_AA)

    # Yellow dot at origin
    origin_pt = project_point(np.array([0, 0, 0], dtype=np.float32), rvec, tvec)
    cv2.circle(frame, origin_pt, 6, ORIGIN_DOT_COLOR, -1)

    return frame


def open_camera():
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
    cap.set(cv2.CAP_PROP_AUTOFOCUS,    0)
    cap.set(cv2.CAP_PROP_FOCUS,        FOCUS)
    cap.set(cv2.CAP_PROP_FPS,          FPS)
    return cap


def main():
    cap = open_camera()

    originFound = False
    origin_rvec = None
    origin_tvec = None
    spacing = GRID_SPACING
    x_range = list(GRID_X_RANGE)
    y_range = list(GRID_Y_RANGE)

    print("Grid Overlay running.")
    print("  Place the origin ArUco marker (id 10) in view.")
    print("  Controls: q=quit  r=re-detect origin  +/-=grid spacing")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera read failed — check CAMERA_INDEX.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = arucoDetector.detectMarkers(gray)

        # ── Detect origin marker ──────────────────────────────────────────────
        if ids is not None:
            ids_flat = ids.flatten()
            for idx, marker_id in enumerate(ids_flat):
                if marker_id == 10 and not originFound:
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners[idx], MARKER_WIDTH, mtx, dist
                    )
                    origin_rvec = rvec[0][0]
                    origin_tvec = tvec[0][0]
                    originFound  = True
                    print("Origin marker found — grid locked.")

            # Draw detected marker outlines for feedback
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # ── Draw grid if origin is known ──────────────────────────────────────
        if originFound:
            # Show the origin marker's own axis so you can verify orientation
            cv2.drawFrameAxes(frame, mtx, dist,
                              origin_rvec, origin_tvec, MARKER_WIDTH * 4)
            draw_grid(frame, origin_rvec, origin_tvec,
                      x_range, y_range, spacing)

            # HUD: current grid info
            cv2.putText(frame,
                        f"Grid spacing: {spacing:.2f} m  "
                        f"X[{x_range[0]:.1f},{x_range[1]:.1f}]  "
                        f"Y[{y_range[0]:.1f},{y_range[1]:.1f}]",
                        (10, FRAME_HEIGHT - 10), FONT, 0.45,
                        (200, 200, 200), 1, cv2.LINE_AA)
        else:
            cv2.putText(frame, "Searching for origin marker (id 10)...",
                        (10, 30), FONT, 0.6, (0, 0, 255), 2)

        cv2.imshow("Grid Overlay", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            originFound = False
            print("Re-detecting origin marker...")
        elif key in (ord('+'), ord('=')):
            spacing = min(spacing + 0.05, 1.0)
            print(f"Grid spacing: {spacing:.2f} m")
        elif key == ord('-'):
            spacing = max(spacing - 0.05, 0.05)
            print(f"Grid spacing: {spacing:.2f} m")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
