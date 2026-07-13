# Kalman Filter - By: Shannon Pitman - Fri Jun 26 2026
# just track, no window
import csi
import time
import math

cam = csi.CSI()
cam.reset()
cam.pixformat(csi.GRAYSCALE)
cam.framesize(csi.QVGA)
cam.auto_gain(False, gain_db=1)
cam.auto_exposure(False, exposure_us=4000)
cam.brightness(-3)
cam.snapshot(time=2000)

full_W = cam.width()
full_H = cam.height()

thresh = [(50, 255)]  # tune with Jordan's results

N_MARKERS = 1

clock = time.clock()
tracks = []
last_t = time.ticks_ms()

# Tuning params
P0_POS = 25.0  # px^2
P0_VEL = 500.0  # px^2/ s^2
q = 50.0  # px^2/s^3
R = 1.0  # centroid measurement variance px^2
ROImargin = 4  # pixels
gate_min = 10.0  # pixels
gate_nSig = 5.0
ROW_TOL = 30  # px band to group markers into rows


class pixelAxis:
    def __init__(self, p0):
        self.p, self.v = p0, 0.0
        self.P00, self.P01, self.P11 = P0_POS, 0.0, P0_VEL  # State Error Covariances

    def predict(self, dt):
        self.p += dt*self.v  # constant velocity
        P00n = self.P00 + 2.0 * dt * self.P01 + dt * dt * self.P11
        P01n = self.P01 + dt * self.P11
        P11n = self.P11
        P00n += q*dt*dt*dt/3.0
        P01n += q*dt*dt/2.0
        P11n += q*dt
        self.P00, self.P01, self.P11 = P00n, P01n, P11n
        return self.P00 + R  # innovation covariance for ROI

    def update(self, z):
        y = z - self.p  # innovation
        S = self.P00 + R
        K0 = self.P00 / S  # KG for position
        K1 = self.P01 / S  # KG for velocity
        self.p += K0 * y
        self.v += K1 * y
        P00_old = self.P00
        P01_old = self.P01
        P11_old = self.P11
        self.P00 = (1-K0) * P00_old
        self.P01 = (1-K0) * P01_old
        self.P11 = P11_old - K1 * P01_old


class MarkerTrack:
    def __init__(self, mid, u0, v0, bw=4, bh=4):
        self.id = mid
        self.u = pixelAxis(u0)
        self.v = pixelAxis(v0)
        self.Su = R
        self.Sv = R
        self.bw = bw  # blob width
        self.bh = bh
        self.misses = 0  # track consecutive frames with no detection

    def predict(self, dt):
        self.Su = self.u.predict(dt)
        self.Sv = self.v.predict(dt)

    def correct(self, u, v, bw, bh):
        self.u.update(u)
        self.v.update(v)
        self.bw = bw
        self.bh = bh
        self.misses = 0

    def gate(self):
        return max(gate_min, gate_nSig*math.sqrt(self.Su + self.Sv))  # search radius for matching markers


def init_tracks():
    while True:
        img = cam.snapshot()
        blobs = img.find_blobs(thresh, merge=False)
        if len(blobs) >= N_MARKERS:
            blobs = sorted(blobs, key=lambda b: b.pixels(), reverse=True)[:N_MARKERS]  # sort 4 biggest blobs
            blobs.sort(key=lambda b: (int(b.cyf() // ROW_TOL), b.cxf()))  # sort top to bottom then left to right
            tracks = []
            for i, b in enumerate(blobs):
                tracks.append(MarkerTrack(i, b.cxf(), b.cyf(), b.w(), b.h()))
                print("Initialised %d markers:" % len(tracks))
            for t in tracks:
                print("id %d  u=%.1f v=%.1f" % (t.id, t.u.p, t.v.p))
            return tracks


def associate(tracks, detections):
    used = [False] * len(detections)
    for t in tracks:
        g2 = t.gate()**2  # gate squared to compare squared distances later
        best = -1  # no match found
        best_d2 = g2
        for i, det in enumerate(detections):
            if used[i]:
                continue
            du = det[0] - t.u.p  # offset between prediction and detection
            dv = det[1] - t.v.p
            d2 = du*du + dv*dv
            if d2 < best_d2:
                best_d2 = d2
                best = i
        if best >= 0:
            used[best] = True
            u, v, bw, bh = detections[best]
            t.correct(u, v, bw, bh)
        else:
            t.misses += 1  # no detection, relies on prediction


clock = time.clock()
tracks = init_tracks()
last_t = time.ticks_ms()


while True:
    clock.tick()
    now = time.ticks_ms()
    dt = time.ticks_diff(now, last_t) / 1000.0
    last_t = now
    for t in tracks:
        t.predict(dt)
    img = cam.snapshot()
    detections = []

    for b in img.find_blobs(thresh, merge=False):
        detections.append((b.cxf(), b.cyf(), b.w(), b.h()))
    associate(tracks, detections)

    for t in tracks:
        lx = int(t.u.p)
        ly = int(t.v.p)
        img.draw_cross(lx, ly, color=255)
        img.draw_string(lx + 3, ly+3, str(t.id), color=255)

    print("fps=%.1f | " % clock.fps()+" ".join("id%d(%.1f,%.1f)m%d" % (t.id, t.u.p, t.v.p, t.misses) for t in tracks))
