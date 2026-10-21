import cv2, json, time
from collections import Counter, defaultdict, deque
from ultralytics import YOLO

MODEL_PATH = 'yolov8n.pt'        # or your fine-tuned 'best.pt'
SOURCE = 0                       # 0 = camera, or an RTSP URL from the drone
CONF_MIN = 0.4                   # ignore detections below this confidence
HISTORY = 10                     # past frames used for label voting
MAX_MISSED = 5                   # keep a lost object on screen for this many frames
LOG_FILE = 'detections.jsonl'    # one JSON record per frame, reusable for training

model = YOLO(MODEL_PATH)
cap = cv2.VideoCapture(SOURCE)

label_history = defaultdict(lambda: deque(maxlen=HISTORY))  # track_id -> recent labels
last_seen = {}                                              # track_id -> (frame, label, box, conf)
frame_idx = 0

with open(LOG_FILE, 'a') as log:
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame_idx += 1

        result = model.track(frame, persist=True, tracker='bytetrack.yaml',
                             conf=CONF_MIN, verbose=False)[0]
        boxes = result.boxes
        frame_record = []

        if boxes.id is not None:
            ids = boxes.id.int().tolist()
            classes = boxes.cls.int().tolist()
            confs = boxes.conf.tolist()
            xyxy = boxes.xyxy.tolist()

            for tid, cls, conf, box in zip(ids, classes, confs, xyxy):
                label = result.names[cls]
                label_history[tid].append(label)

                # temporal voting: the most common recent label wins
                stable_label = Counter(label_history[tid]).most_common(1)[0][0]
                last_seen[tid] = (frame_idx, stable_label, box, conf)

                frame_record.append({'id': tid, 'label': stable_label,
                                     'conf': round(conf, 3),
                                     'box': [round(v, 1) for v in box]})

        log.write(json.dumps({'frame': frame_idx, 'time': round(time.time(), 2),
                              'objects': frame_record}) + '\n')

        # draw current objects plus recently lost ones
        for tid, (f, label, box, conf) in list(last_seen.items()):
            age = frame_idx - f
            if age > MAX_MISSED:
                del last_seen[tid]          # forget tracks that are truly gone
                continue
            x1, y1, x2, y2 = map(int, box)
            color = (0, 255, 0) if age == 0 else (0, 160, 255)  # orange = persisted
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f'#{tid} {label} {conf:.2f}', (x1, max(y1 - 8, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        cv2.imshow('Live detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
