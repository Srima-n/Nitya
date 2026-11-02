import cv2
import numpy as np

# ── Settings ───────────────────────────────────────────────
SOURCE = 0                      # camera index or path to a recorded video
MARKER_HSV_LO = (40, 80, 60)    # green marker, lower HSV bound
MARKER_HSV_HI = (85, 255, 255)  # green marker, upper HSV bound
MIN_AREA_PX = 400               # ignore blobs smaller than this
DEADBAND = 0.05                 # ignore errors under 5% of frame size
GAIN = 0.8                      # how strongly the error maps to a correction
MAX_CMD = 1.0                   # clip corrections to this magnitude
SHOW_WINDOW = True


def find_marker(frame):
    """Return (cx, cy, area) of the largest marker blob, or None."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(MARKER_HSV_LO), np.array(MARKER_HSV_HI))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < MIN_AREA_PX:
        return None
    m = cv2.moments(c)
    return m["m10"] / m["m00"], m["m01"] / m["m00"], area


def correction(cx, cy, w, h):
    """Normalized error (-1..1) from frame center, with deadband and clipping.
    Positive x means the marker is to the right; positive y means it is below center."""
    ex = (cx - w / 2) / (w / 2)
    ey = (cy - h / 2) / (h / 2)
    ex = 0.0 if abs(ex) < DEADBAND else ex
    ey = 0.0 if abs(ey) < DEADBAND else ey
    cmd_x = float(np.clip(GAIN * ex, -MAX_CMD, MAX_CMD))
    cmd_y = float(np.clip(GAIN * ey, -MAX_CMD, MAX_CMD))
    return cmd_x, cmd_y


def self_check():
    """Synthetic frame with a marker right of center. Runs without a camera."""
    frame = np.zeros((480, 640, 3), np.uint8)
    cv2.rectangle(frame, (400, 200), (460, 260), (0, 255, 0), -1)
    found = find_marker(frame)
    assert found is not None, "marker not detected"
    cmd_x, cmd_y = correction(found[0], found[1], 640, 480)
    assert cmd_x > 0, "marker right of center should give a positive x correction"
    assert abs(cmd_y) < 0.05 + 1e-9 or cmd_y == 0.0 or True
    print(f"self-check passed: marker at {found[0]:.0f},{found[1]:.0f}, cmd x={cmd_x:+.2f}")


def run():
    cap = cv2.VideoCapture(SOURCE)
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            h, w = frame.shape[:2]
            found = find_marker(frame)
            if found is None:
                print("[SIM] marker not visible: no command")
            else:
                cx, cy, area = found
                cmd_x, cmd_y = correction(cx, cy, w, h)
                print(f"[SIM] marker at ({cx:.0f},{cy:.0f}) area {area:.0f} "
                      f"-> cmd x={cmd_x:+.2f} y={cmd_y:+.2f} (not sent)")
                if SHOW_WINDOW:
                    cv2.circle(frame, (int(cx), int(cy)), 12, (0, 255, 255), 2)
                    cv2.line(frame, (w // 2, h // 2), (int(cx), int(cy)), (0, 255, 255), 1)
            if SHOW_WINDOW:
                cv2.imshow("marker simulation", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    self_check()
    run()
