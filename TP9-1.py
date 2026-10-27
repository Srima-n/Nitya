import json, logging, math, os, tempfile, threading, time, uuid
from dataclasses import dataclass, field, fields
from typing import Callable, List, Optional, Tuple

import cv2
import numpy as np
import requests

log = logging.getLogger("sighting")


# ── Configuration ──────────────────────────────────────────
@dataclass
class Config:
    drone_id: str = "drone_01"

    # detection
    detector: str = "yolo"                  # "yolo" or "thermal"
    model_path: str = "yolov8n.pt"          # use a model fine-tuned on aerial/disaster data
    confidence_min: float = 0.35
    thermal_hot_threshold: int = 190        # 0-255 gray level treated as warm
    thermal_min_area: int = 80              # pixels
    thermal_max_area: int = 5000

    # camera and telemetry
    camera_source: str = "0"                # device index or RTSP URL
    camera_hfov_deg: float = 62.0
    camera_vfov_deg: float = 49.0
    default_gimbal_pitch_deg: float = -90.0 # used if the autopilot reports no gimbal angle
    mavlink_connection: str = "udpin:0.0.0.0:14550"
    mavlink_baud: int = 921600
    telemetry_max_age_s: float = 1.0
    min_depression_deg: float = 10.0        # ignore rays too close to the horizon
    max_range_m: float = 150.0              # ignore sightings farther than this from the drone

    # confirmation and de-duplication
    confirm_hits: int = 3                   # frames with a detection before reporting
    confirm_window_s: float = 5.0
    cluster_radius_m: float = 8.0

    # reporting
    coordinator_url: str = ""               # must be https://, empty = queue locally only
    coordinator_token_env: str = "RESCUE_COORD_TOKEN"
    retry_every_s: float = 15.0
    outbox_path: str = "sighting_outbox.jsonl"
    dead_letter_path: str = "sighting_rejected.jsonl"
    save_snapshots: bool = True             # saved locally only, never sent
    snapshot_dir: str = "snapshots"

    def __post_init__(self):
        if self.detector not in ("yolo", "thermal"):
            raise ValueError("detector must be 'yolo' or 'thermal'")
        if self.coordinator_url and not self.coordinator_url.startswith("https://"):
            raise ValueError("coordinator_url must use https://")
        if self.confirm_hits < 1:
            raise ValueError("confirm_hits must be at least 1")

    @classmethod
    def from_file(cls, path: str) -> "Config":
        if not os.path.exists(path):
            return cls()
        with open(path) as f:
            data = json.load(f)
        known = {f_.name for f_ in fields(cls)}
        unknown = set(data) - known
        if unknown:
            raise ValueError(f"Unknown config keys: {sorted(unknown)}")
        return cls(**data)


# ── Data types ─────────────────────────────────────────────
@dataclass
class Detection:
    box: Tuple[float, float, float, float]  # x1, y1, x2, y2 in pixels
    score: float
    source: str


@dataclass
class Telemetry:
    lat: float
    lon: float
    alt_agl_m: float
    heading_deg: float          # direction the top edge of the image points, clockwise from north
    gimbal_pitch_deg: float     # 0 = horizontal, -90 = straight down
    timestamp: float


@dataclass
class Track:
    id: int
    lat: float
    lon: float
    source: str
    first_seen: float
    last_seen: float
    pos_n: int = 1
    hit_times: List[float] = field(default_factory=list)
    score_sum: float = 0.0
    score_n: int = 0
    last_frame: int = -1
    reported: bool = False


# ── Detectors (swap by config) ─────────────────────────────
class YoloPersonDetector:
    def __init__(self, cfg: Config):
        from ultralytics import YOLO
        self.model = YOLO(cfg.model_path)
        self.conf = cfg.confidence_min

    def detect(self, frame) -> List[Detection]:
        r = self.model(frame, classes=[0], conf=self.conf, verbose=False)[0]
        return [Detection(tuple(b), float(s), "yolo")
                for b, s in zip(r.boxes.xyxy.tolist(), r.boxes.conf.tolist())]


class ThermalBlobDetector:
    """Finds warm blobs in a thermal frame. A simple baseline. Replace with a trained model when you have data."""

    def __init__(self, cfg: Config):
        self.thr = cfg.thermal_hot_threshold
        self.min_area = cfg.thermal_min_area
        self.max_area = cfg.thermal_max_area
        self.kernel = np.ones((3, 3), np.uint8)

    def detect(self, frame) -> List[Detection]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        mask = np.where(gray >= self.thr, 255, 0).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in contours:
            area = cv2.contourArea(c)
            if not (self.min_area <= area <= self.max_area):
                continue
            x, y, w, h = cv2.boundingRect(c)
            score = float(gray[y:y + h, x:x + w].mean()) / 255.0
            out.append(Detection((x, y, x + w, y + h), score, "thermal"))
        return out


def build_detector(cfg: Config):
    return YoloPersonDetector(cfg) if cfg.detector == "yolo" else ThermalBlobDetector(cfg)


# ── Camera with automatic reconnect ────────────────────────
class Camera:
    def __init__(self, source: str):
        self.source = int(source) if source.isdigit() else source
        self.cap = None
        self._fail_count = 0

    def read(self):
        if self.cap is None or not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.source)
        ok, frame = self.cap.read()
        if not ok:
            self._fail_count += 1
            self.cap.release()
            self.cap = None
            time.sleep(min(10, 0.5 * 2 ** min(self._fail_count, 5)))  # backoff
            return None
        self._fail_count = 0
        return frame

    def close(self):
        if self.cap is not None:
            self.cap.release()


# ── Telemetry from the flight controller ───────────────────
class MavlinkTelemetry:
    """Reads position, yaw, and gimbal pitch. Returns None when data is missing or stale."""

    def __init__(self, cfg: Config):
        from pymavlink import mavutil
        self.m = mavutil.mavlink_connection(cfg.mavlink_connection, baud=cfg.mavlink_baud)
        self.m.wait_heartbeat(timeout=15)
        self.default_pitch = cfg.default_gimbal_pitch_deg
        self.pos = None
        self.yaw = 0.0
        self.gimbal_pitch = self.default_pitch
        self.ts = 0.0

    def latest(self) -> Optional[Telemetry]:
        while True:
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                break
            t = msg.get_type()
            if t == "GLOBAL_POSITION_INT":
                self.pos = (msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0)
                self.ts = time.time()
            elif t == "ATTITUDE":
                self.yaw = math.degrees(msg.yaw) % 360
            elif t == "MOUNT_ORIENTATION":
                self.gimbal_pitch = msg.pitch
        if self.pos is None or (self.pos[0] == 0 and self.pos[1] == 0):
            return None  # no GPS fix
        lat, lon, alt = self.pos
        return Telemetry(lat, lon, alt, self.yaw, self.gimbal_pitch, self.ts)


# ── Geometry: pixel to ground position ─────────────────────
def project_to_ground(px, py, W, H, tel: Telemetry, cfg: Config):
    """Estimate the ground position of an image pixel. Small-angle pinhole model, flat ground.
    Returns (lat, lon, horizontal_distance_m) or None if the ray is too close to the horizon."""
    depression = math.radians(-tel.gimbal_pitch)                 # 90 deg = straight down
    right_ang = (px - W / 2) / (W / 2) * math.radians(cfg.camera_hfov_deg) / 2
    fwd_ang = (H / 2 - py) / (H / 2) * math.radians(cfg.camera_vfov_deg) / 2
    ray_dep = depression - fwd_ang                               # top of image looks further ahead
    if math.degrees(ray_dep) < cfg.min_depression_deg:
        return None

    h = tel.alt_agl_m / math.tan(ray_dep)                        # horizontal distance to the hit point
    fwd = h * math.cos(right_ang)
    right = h * math.sin(right_ang)
    hd = math.radians(tel.heading_deg)
    north = fwd * math.cos(hd) - right * math.sin(hd)
    east = fwd * math.sin(hd) + right * math.cos(hd)
    dist = math.hypot(north, east)

    lat = tel.lat + north / 111320.0
    lon = tel.lon + east / (111320.0 * math.cos(math.radians(tel.lat)))
    return lat, lon, dist


def meters_between(lat1, lon1, lat2, lon2):
    dlat = (lat1 - lat2) * 111320.0
    dlon = (lon1 - lon2) * 111320.0 * math.cos(math.radians(lat1))
    return math.hypot(dlat, dlon)


# ── Sighting engine ────────────────────────────────────────
class SightingEngine:
    """Turns per-frame detections into tracks, and reports a track once it is confirmed.
    It never sends flight commands."""

    def __init__(self, cfg: Config, detector, on_confirmed: Callable):
        self.cfg = cfg
        self.detector = detector
        self.on_confirmed = on_confirmed
        self.tracks: List[Track] = []
        self._next_id = 1
        self._frame_no = 0

    def _match(self, lat, lon):
        best, best_d = None, self.cfg.cluster_radius_m
        for tr in self.tracks:
            d = meters_between(tr.lat, tr.lon, lat, lon)
            if d <= best_d:
                best, best_d = tr, d
        return best

    def process(self, frame, tel: Telemetry, now: float):
        self._frame_no += 1
        H, W = frame.shape[:2]
        for det in self.detector.detect(frame):
            x1, y1, x2, y2 = det.box
            g = project_to_ground((x1 + x2) / 2, (y1 + y2) / 2, W, H, tel, self.cfg)
            if g is None:
                continue
            lat, lon, dist = g
            if dist > self.cfg.max_range_m:
                continue

            tr = self._match(lat, lon)
            if tr is None:
                tr = Track(self._next_id, lat, lon, det.source, now, now)
                self._next_id += 1
                self.tracks.append(tr)
            else:
                tr.pos_n += 1                                    # running mean of position
                tr.lat += (lat - tr.lat) / tr.pos_n
                tr.lon += (lon - tr.lon) / tr.pos_n

            # count at most one hit per track per frame
            if tr.last_frame != self._frame_no:
                tr.last_frame = self._frame_no
                tr.hit_times.append(now)
                tr.score_sum += det.score
                tr.score_n += 1
                tr.last_seen = now
                cutoff = now - self.cfg.confirm_window_s
                tr.hit_times = [t for t in tr.hit_times if t >= cutoff]

            if not tr.reported and len(tr.hit_times) >= self.cfg.confirm_hits:
                tr.reported = True
                self.on_confirmed(tr, tel, frame)

        self._prune(now)

    def _prune(self, now):
        keep_for = 3 * self.cfg.confirm_window_s
        # drop unconfirmed tracks that went quiet; keep reported ones so they de-duplicate
        self.tracks = [t for t in self.tracks
                       if t.reported or now - t.last_seen <= keep_for]


# ── Storage: crash-safe outbox and dead-letter file ────────
class Outbox:
    def __init__(self, path: str, dead_letter_path: str):
        self.path = path
        self.dead_path = dead_letter_path
        self.lock = threading.Lock()

    def add(self, record: dict):
        with self.lock:
            with open(self.path, "a") as f:
                f.write(json.dumps(record) + "\n")
                f.flush()
                os.fsync(f.fileno())

    def _read_all(self):
        if not os.path.exists(self.path):
            return []
        records = []
        with open(self.path) as f:
            for line in f:
                if not line.strip():
                    continue
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError:
                    log.error("Skipping corrupt outbox line")
        return records

    def pending(self):
        with self.lock:
            return self._read_all()

    def _rewrite(self, records):
        d = os.path.dirname(os.path.abspath(self.path))
        fd, tmp = tempfile.mkstemp(dir=d, suffix=".tmp")
        with os.fdopen(fd, "w") as f:
            for r in records:
                f.write(json.dumps(r) + "\n")
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self.path)     # atomic on the same filesystem

    def remove(self, report_ids: set):
        """Delete delivered records. Reads the file under the lock, so records added meanwhile are kept."""
        with self.lock:
            kept = [r for r in self._read_all() if r.get("report_id") not in report_ids]
            self._rewrite(kept)

    def dead_letter(self, record: dict):
        with self.lock:
            with open(self.dead_path, "a") as f:
                f.write(json.dumps(record) + "\n")


# ── Sending with retry rules ───────────────────────────────
class Sender:
    OK, RETRY, REJECT, AUTH = "ok", "retry", "reject", "auth"

    def __init__(self, cfg: Config, outbox: Outbox):
        self.url = cfg.coordinator_url
        self.token = os.environ.get(cfg.coordinator_token_env, "")
        self.outbox = outbox
        self.session = requests.Session()
        self.enabled = bool(self.url)
        if not self.enabled:
            log.warning("No coordinator_url set. Reports are queued locally only.")

    def _post(self, record) -> str:
        try:
            r = self.session.post(
                self.url, json=record, timeout=5,
                headers={"Authorization": f"Bearer {self.token}",
                         "Idempotency-Key": record["report_id"]})
        except requests.RequestException:
            return self.RETRY
        if r.status_code in (200, 201, 202, 409):   # 409 = already received
            return self.OK
        if r.status_code in (401, 403):
            return self.AUTH
        if r.status_code in (400, 422):              # bad data: retrying will not help
            return self.REJECT
        return self.RETRY                            # 429, 5xx, anything else

    def flush(self):
        if not self.enabled:
            return
        pending = self.outbox.pending()
        if not pending:
            return
        delivered = set()
        for rec in pending:
            result = self._post(rec)
            if result == self.OK:
                delivered.add(rec["report_id"])
            elif result == self.REJECT:
                self.outbox.dead_letter(rec)
                delivered.add(rec["report_id"])
                log.error("Coordinator rejected report %s. Moved to dead letter.", rec["report_id"])
            elif result == self.AUTH:
                log.error("Coordinator rejected the token. Check %s.", "RESCUE_COORD_TOKEN")
                break
            else:
                log.warning("Link down. %d reports still queued.", len(pending) - len(delivered))
                break
        if delivered:
            self.outbox.remove(delivered)
            log.info("Delivered %d report(s).", len(delivered))


def flush_loop(sender: Sender, every_s: float, stop: threading.Event):
    while not stop.is_set():
        try:
            sender.flush()
        except Exception:
            log.exception("Flush failed")
        stop.wait(every_s)


# ── Report building and handling ───────────────────────────
def make_confirm_handler(cfg: Config, outbox: Outbox):
    def handle(track: Track, tel: Telemetry, frame):
        snapshot = None
        if cfg.save_snapshots:
            os.makedirs(cfg.snapshot_dir, exist_ok=True)
            snapshot = f"track_{track.id}_{int(time.time())}.jpg"
            cv2.imwrite(os.path.join(cfg.snapshot_dir, snapshot), frame)

        record = {
            "schema": 1,
            "report_id": str(uuid.uuid4()),
            "drone_id": cfg.drone_id,
            "track_id": track.id,
            "kind": "possible_person",
            "lat": round(track.lat, 6),
            "lon": round(track.lon, 6),
            "position_radius_m": cfg.cluster_radius_m,
            "hits": len(track.hit_times),
            "mean_score": round(track.score_sum / max(track.score_n, 1), 3),
            "detector": track.source,
            "drone_alt_m": round(tel.alt_agl_m, 1),
            "first_seen_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(track.first_seen)),
            "last_seen_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(track.last_seen)),
            "note": "Unverified. Requires human confirmation before any response.",
        }
        outbox.add(record)                                   # snapshot stays local
        log.warning("Possible person #%d at %.6f, %.6f (queued)",
                    track.id, track.lat, track.lon)
    return handle


# ── Main ───────────────────────────────────────────────────
def main(config_path: str = "sighting_config.json"):
    cfg = Config.from_file(config_path)
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")

    outbox = Outbox(cfg.outbox_path, cfg.dead_letter_path)
    sender = Sender(cfg, outbox)
    engine = SightingEngine(cfg, build_detector(cfg), make_confirm_handler(cfg, outbox))
    telemetry = MavlinkTelemetry(cfg)
    camera = Camera(cfg.camera_source)

    stop = threading.Event()
    threading.Thread(target=flush_loop, args=(sender, cfg.retry_every_s, stop),
                     daemon=True).start()

    log.info("Running. Detector=%s, confirm after %d hits in %.0f s",
             cfg.detector, cfg.confirm_hits, cfg.confirm_window_s)
    stale_warned = False
    try:
        while True:
            frame = camera.read()
            if frame is None:
                continue
            tel = telemetry.latest()
            now = time.time()
            if tel is None or now - tel.timestamp > cfg.telemetry_max_age_s:
                if not stale_warned:
                    log.warning("Telemetry missing or stale. Skipping frames.")
                    stale_warned = True
                continue
            stale_warned = False
            engine.process(frame, tel, now)
    except KeyboardInterrupt:
        log.info("Stopped.")
    finally:
        stop.set()
        camera.close()


# ── Quick self-check for the geometry ──────────────────────
def _selftest():
    cfg = Config()
    tel = Telemetry(13.08, 80.27, 30.0, 0.0, -90.0, time.time())
    lat, lon, dist = project_to_ground(320, 240, 640, 480, tel, cfg)   # image center
    assert dist < 0.01, "a straight-down center pixel should land under the drone"
    lat2, _, d2 = project_to_ground(320, 0, 640, 480, tel, cfg)        # top edge, looks ahead
    assert lat2 > tel.lat and d2 > 0, "top of image should land north when heading north"
    print("geometry self-check passed")


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        _selftest()
    else:
        main(sys.argv[1] if len(sys.argv) > 1 else "sighting_config.json")
