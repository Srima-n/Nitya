import math
import numpy as np

from sighting import project_to_ground, Config, Telemetry  # from the sighting module

EARTH_M_PER_DEG = 111320.0


# ── Local metric frame ─────────────────────────────────────
class LocalFrame:
    """Converts lat/lon to east/north meters around a fixed origin. Accurate for areas of a few km."""

    def __init__(self, lat0: float, lon0: float):
        self.lat0, self.lon0 = lat0, lon0
        self.kx = EARTH_M_PER_DEG * math.cos(math.radians(lat0))

    def to_xy(self, lat, lon):
        return (lon - self.lon0) * self.kx, (lat - self.lat0) * EARTH_M_PER_DEG

    def to_latlon(self, x, y):
        return self.lat0 + y / EARTH_M_PER_DEG, self.lon0 + x / self.kx


# ── Point-in-polygon (vectorized ray casting) ──────────────
def points_in_polygon(px, py, poly):
    inside = np.zeros(px.shape, dtype=bool)
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        crosses = (yi > py) != (yj > py)
        x_at_y = (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi
        inside ^= crosses & (px < x_at_y)
        j = i
    return inside


# ── Coverage grid over a search area ───────────────────────
class CoverageGrid:
    """Counts how many frames have seen each cell. Cells are marked by the true image footprint,
    not its bounding box, so coverage is not overstated at the edges."""

    def __init__(self, frame: LocalFrame, width_m: float, height_m: float, cell_m: float = 5.0):
        self.frame = frame
        self.cell = cell_m
        self.cols = int(math.ceil(width_m / cell_m))
        self.rows = int(math.ceil(height_m / cell_m))
        self.counts = np.zeros((self.rows, self.cols), dtype=np.uint32)

    def _cell_centers(self):
        xs = (np.arange(self.cols) + 0.5) * self.cell
        ys = (np.arange(self.rows) + 0.5) * self.cell
        return np.meshgrid(xs, ys)  # shape (rows, cols)

    def mark_footprint(self, corners_latlon):
        """corners_latlon: four ground points (lat, lon) from the image corners, in order."""
        poly = [self.frame.to_xy(la, lo) for la, lo in corners_latlon]
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        c0 = max(0, int(min(xs) // self.cell))
        c1 = min(self.cols - 1, int(max(xs) // self.cell))
        r0 = max(0, int(min(ys) // self.cell))
        r1 = min(self.rows - 1, int(max(ys) // self.cell))
        if c0 > c1 or r0 > r1:
            return
        cx, cy = np.meshgrid((np.arange(c0, c1 + 1) + 0.5) * self.cell,
                             (np.arange(r0, r1 + 1) + 0.5) * self.cell)
        inside = points_in_polygon(cx, cy, poly)
        self.counts[r0:r1 + 1, c0:c1 + 1] += inside.astype(np.uint32)

    def mark_from_frame(self, frame_shape, tel: Telemetry, cfg: Config):
        """Project the four image corners and mark the footprint. Skips frames whose corners hit the horizon."""
        H, W = frame_shape[:2]
        corners = []
        for px, py in [(0, 0), (W, 0), (W, H), (0, H)]:
            g = project_to_ground(px, py, W, H, tel, cfg)
            if g is None:
                return
            corners.append((g[0], g[1]))
        self.mark_footprint(corners)

    def nearest_unseen(self, drone_lat, drone_lon):
        """Global search for the closest unseen cell. Returns (lat, lon) or None when fully covered."""
        unseen = np.argwhere(self.counts == 0)
        if unseen.size == 0:
            return None
        x, y = self.frame.to_xy(drone_lat, drone_lon)
        dr = unseen[:, 0] * self.cell + self.cell / 2 - y
        dc = unseen[:, 1] * self.cell + self.cell / 2 - x
        i = int(np.argmin(dr * dr + dc * dc))
        r, c = unseen[i]
        return self.frame.to_latlon((c + 0.5) * self.cell, (r + 0.5) * self.cell)

    def coverage_fraction(self):
        return float((self.counts > 0).mean())


# ── Lawnmower waypoints over a search polygon ──────────────
def lawnmower(polygon_latlon, frame: LocalFrame, swath_m: float, heading_deg: float = 0.0):
    """Boustrophedon waypoints covering a polygon. swath_m = ground width seen per pass.
    Returns a list of (lat, lon). The caller must still check these against the geofence."""
    poly = [frame.to_xy(la, lo) for la, lo in polygon_latlon]

    def rotate(pts, deg):
        t = math.radians(deg)
        c, s = math.cos(t), math.sin(t)
        return [(x * c - y * s, x * s + y * c) for x, y in pts]

    rot = rotate(poly, -heading_deg)         # rows run along the local x axis
    ys = [p[1] for p in rot]
    n = len(rot)

    rows = []
    y = min(ys) + swath_m / 2
    while y <= max(ys) - swath_m / 2 + 1e-9:
        xs = []
        for i in range(n):
            x1, y1 = rot[i]
            x2, y2 = rot[(i + 1) % n]
            if (y1 > y) != (y2 > y):
                xs.append(x1 + (y - y1) * (x2 - x1) / (y2 - y1))
        xs.sort()
        segs = list(zip(xs[0::2], xs[1::2]))
        if segs:
            rows.append((y, segs))
        y += swath_m

    waypoints = []
    for row_idx, (y, segs) in enumerate(rows):
        if row_idx % 2 == 1:                 # alternate direction to avoid long transits
            segs = [(b, a) for a, b in reversed(segs)]
        for a, b in segs:
            waypoints.append((a, y))
            waypoints.append((b, y))

    back = rotate(waypoints, heading_deg)
    return [frame.to_latlon(x, y) for x, y in back]


# ── Fusing repeat sightings of the same person ─────────────
def fuse_sightings(observations, base_sigma_m=3.0, sigma_per_m_range=0.03):
    """observations: list of (lat, lon, range_m) from different passes.
    Weights each by the expected error, which grows with range. Returns (lat, lon, sigma_m)."""
    if not observations:
        return None
    sw = swx = swy = 0.0
    lat0 = observations[0][0]
    kx = EARTH_M_PER_DEG * math.cos(math.radians(lat0))
    for lat, lon, rng in observations:
        sigma = base_sigma_m + sigma_per_m_range * rng
        w = 1.0 / (sigma * sigma)
        sw += w
        swx += w * (lon * kx)
        swy += w * (lat * EARTH_M_PER_DEG)
    lat = swy / sw / EARTH_M_PER_DEG
    lon = swx / sw / kx
    return lat, lon, math.sqrt(1.0 / sw)


# ── Example ────────────────────────────────────────────────
if __name__ == "__main__":
    origin = (13.0800, 80.2700)
    frame = LocalFrame(*origin)

    area = [(13.0800, 80.2700), (13.0800, 80.2750),
            (13.0830, 80.2750), (13.0830, 80.2700)]
    waypoints = lawnmower(area, frame, swath_m=20.0, heading_deg=0.0)
    print(f"{len(waypoints)} waypoints. First: {waypoints[0]}")

    grid = CoverageGrid(frame, width_m=600, height_m=400, cell_m=5.0)
    cfg = Config()
    tel = Telemetry(13.0800, 80.2700, 30.0, 0.0, -75.0, 0.0)
    grid.mark_from_frame((480, 640), tel, cfg)
    print(f"Coverage after one frame: {grid.coverage_fraction():.2%}")

    fused = fuse_sightings([(13.08010, 80.27010, 40.0), (13.08012, 80.27008, 55.0)])
    print("Fused sighting:", fused)
