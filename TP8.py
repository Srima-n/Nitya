import time
from collections import deque

import cv2
import mediapipe as mp

# ── Settings ───────────────────────────────────────────────
HOLD_FRAMES = 15            # a sign must appear in 15 consecutive frames (~0.5 s) to count
CONFIRM_WINDOW_S = 5.0      # a takeoff request expires if not confirmed within this time
UP_STEP_M = 0.5             # height change per UP sign
MAX_ALT_M = 10.0            # ceiling for gesture-commanded altitude changes
DRY_RUN = True              # True = print actions only. Keep True until tested with props off.

FINGER_TIPS = [8, 12, 16, 20]   # index, middle, ring, pinky tips
FINGER_PIPS = [6, 10, 14, 18]   # joints just below each tip


def classify(lm):
    """Return a sign name from 21 hand landmarks (image coords, y grows downward)."""
    up = [lm[t].y < lm[p].y for t, p in zip(FINGER_TIPS, FINGER_PIPS)]  # index, middle, ring, pinky
    n_up = sum(up)
    thumb_up = lm[4].y < lm[3].y and lm[4].y < lm[2].y and n_up == 0

    if n_up == 4:
        return 'PALM'
    if thumb_up:
        return 'THUMB_UP'
    if n_up == 0:
        return 'FIST'
    if up == [True, False, False, False]:
        return 'INDEX'
    if up == [True, True, True, False]:
        return 'THREE'
    return None


class CommandGate:
    """Turns stable signs into safe intents. Nothing reaches the flight controller directly."""

    def __init__(self):
        self.pending_takeoff_at = None
        self.altitude_m = 0.0   # tracked estimate; replace with real telemetry

    def handle(self, sign):
        now = time.time()

        # Stop and land are always honored, and they cancel any pending request
        if sign == 'PALM':
            self.pending_takeoff_at = None
            return 'HOLD'
        if sign == 'FIST':
            self.pending_takeoff_at = None
            return 'LAND'

        if sign == 'INDEX':
            if self.altitude_m + UP_STEP_M > MAX_ALT_M:
                return 'REJECTED: ceiling'
            self.altitude_m += UP_STEP_M
            return f'UP {UP_STEP_M} m'

        if sign == 'THREE':
            self.pending_takeoff_at = now
            return 'TAKEOFF requested: show THUMB_UP within 5 s to confirm'

        if sign == 'THUMB_UP':
            if self.pending_takeoff_at and now - self.pending_takeoff_at <= CONFIRM_WINDOW_S:
                self.pending_takeoff_at = None
                return 'TAKEOFF confirmed'
            return None  # no pending request, so ignore

        return None


def dispatch(intent):
    if DRY_RUN:
        print(f"[SIM] {intent}")
    else:
        # Connect to the autonomous controller here, which has its own checks
        # (GPS fix, battery, geofence, mode). Do not send raw commands from this script.
        raise NotImplementedError("Hook up the safety-checked flight controller first.")


def main():
    hands = mp.solutions.hands.Hands(max_num_hands=1,
                                     min_detection_confidence=0.7,
                                     min_tracking_confidence=0.6)
    cap = cv2.VideoCapture(0)
    recent = deque(maxlen=HOLD_FRAMES)
    gate = CommandGate()
    last_sent = None
    last_seen = time.time()

    print("Gesture control running. Press q in the window to quit.")
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        result = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        sign = None
        if result.multi_hand_landmarks:
            last_seen = time.time()
            sign = classify(result.multi_hand_landmarks[0].landmark)
            mp.solutions.drawing_utils.draw_landmarks(
                frame, result.multi_hand_landmarks[0], mp.solutions.hands.HAND_CONNECTIONS)

        # Safety: if no hand is seen for 2 s, hold position instead of acting on stale input
        if time.time() - last_seen > 2.0 and last_sent != 'HOLD':
            dispatch('HOLD (no hand detected)')
            last_sent = 'HOLD'

        recent.append(sign)
        # Debounce: act only when the same sign fills the whole window
        if len(recent) == HOLD_FRAMES and recent[0] is not None and all(s == recent[0] for s in recent):
            if recent[0] != last_sent:
                intent = gate.handle(recent[0])
                if intent:
                    dispatch(intent)
                last_sent = recent[0]

        cv2.putText(frame, f"sign: {sign}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow('Gesture control', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
