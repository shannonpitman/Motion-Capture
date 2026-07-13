# Kalman Filter - By: Shannon Pitman - Fri Jun 26 2026
# just track, no window
import csi
import time
import math

cam = csi.CSI()

SEARCHING_RESOLUTION = csi.VGA
SEARCHING_AREA_THRESHOLD = 16
SEARCHING_PIXEL_THRESHOLD = SEARCHING_AREA_THRESHOLD

TRACKING_RESOLUTION = csi.QQVGA
TRACKING_AREA_THRESHOLD = 256
TRACKING_PIXEL_THRESHOLD = TRACKING_AREA_THRESHOLD
TRACKING_EDGE_TOLERANCE = 0.05  # Blob can move 5% away from the center -> otherwise recenter

cam.reset()
cam.pixformat(csi.GRAYSCALE)
cam.framesize(SEARCHING_RESOLUTION)
cam.snapshot(time=1000)
cam.auto_gain(False, gain_db=1)
cam.auto_exposure(False, exposure_us=5000)

cam.snapshot(time=3000)

# sensor_w and sensor_h are the image sensor raw pixels w/h (x/y are 0 initially).
x, y, sensor_w, sensor_h = cam.ioctl(csi.IOCTL_GET_READOUT_WINDOW)


thresh = [(120, 255)]  # tune with Jordan's results
N_MARKERS = 1
ROW_TOL = 30  # px band to group markers into rows

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
MAX_MISSES = 10  # detection free frames - full frame search

current_res = SEARCHING_RESOLUTION


def image_to_sensor(cx, cy):
    # Map an image-frame point to absolute sensor-array coordinates.
    x, y, w, h = cam.ioctl(csi.IOCTL_GET_READOUT_WINDOW)
    W = cam.width()
    H = cam.height()
    ratio = min(w / float(W), h / float(H))
    mx = (cx - (W / 2.0)) * ratio
    mx += (w - (W * ratio)) / 2.0  # keep aspect-ratio
    mx += x + (sensor_w / 2.0)  # displacement from sensor center
    my = (cy - (H / 2.0)) * ratio
    my += (h - (H * ratio)) / 2.0
    my += y + (sensor_h / 2.0)
    return (mx, my)


def sensor_to_image(mx, my):
    # Inverse of image_to_sensor: sensor coords -> current image frame.
    x, y, w, h = cam.ioctl(csi.IOCTL_GET_READOUT_WINDOW)
    W = cam.width()
    H = cam.height()
    ratio = min(w / float(W), h / float(H))
    cx = (W / 2.0) + (mx - x - (sensor_w / 2.0) - (w - W * ratio) / 2.0) / ratio
    cy = (H / 2.0) + (my - y - (sensor_h / 2.0) - (h - H * ratio) / 2.0) / ratio
    return (cx, cy)


def mapped_blob(b):
    #  (sensor_cx, sensor_cy, w, h) for a blob, centroid in sensor coords.
    mx, my = image_to_sensor(b.cxf(), b.cyf())
    return (mx, my, b.w(), b.h())


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


def reset_to_search():
    #  Full-frame search view: Startup or when tracking is lost
    global current_res
    cam.framesize(SEARCHING_RESOLUTION)
    cam.ioctl(csi.IOCTL_SET_READOUT_WINDOW, (0, 0, sensor_w, sensor_h))
    current_res = SEARCHING_RESOLUTION


def center_window(mapped_cx, mapped_cy, res):
    #  Center the readout window on a point given in sensor coordinates
    global current_res
    if res != current_res:
        cam.framesize(res)  # keeps FPS high
        current_res = res
    x = int(mapped_cx - (sensor_w / 2.0))
    y = int(mapped_cy - (sensor_h / 2.0))
    w = cam.width()
    h = cam.height()
    cam.ioctl(csi.IOCTL_SET_READOUT_WINDOW, (x, y, w, h))
    #  Detect edge clamping
    new_x, new_y, _, _ = cam.ioctl(csi.IOCTL_GET_READOUT_WINDOW)
    if x - new_x < 0:
        print("-X Limit ", end="")
    if x - new_x > 0:
        print("+X Limit ", end="")
    if y - new_y < 0:
        print("-Y Limit ", end="")
    if y - new_y > 0:
        print("+Y Limit ", end="")


def window_center_sensor():
    # Sensor-coord point the current readout window is centered on.
    return image_to_sensor(cam.width() / 2.0, cam.height() / 2.0)


def init_tracks():
    # Blocking search at full frame until N_MARKERS blobs are seen
    reset_to_search()
    while True:
        img = cam.snapshot()
        blobs = img.find_blobs(thresh, area_threshold=SEARCHING_AREA_THRESHOLD,
                               pixels_threshold=SEARCHING_PIXEL_THRESHOLD, merge=False)
        if len(blobs) >= N_MARKERS:
            blobs = sorted(blobs, key=lambda b: b.pixels(), reverse=True)[:N_MARKERS]
            # order top->bottom then left->right
            blobs.sort(key=lambda b: (int(b.cyf() // ROW_TOL), b.cxf()))
            tracks = []
            for i, b in enumerate(blobs):
                mx, my = image_to_sensor(b.cxf(), b.cyf())
                tracks.append(MarkerTrack(i, mx, my, b.w(), b.h()))
            print("Initialised %d markers:" % len(tracks))
            for t in tracks:
                print("id %d  u=%.1f v=%.1f" % (t.id, t.u.p, t.v.p))
            return tracks


def find_detections(img):
    if current_res == TRACKING_RESOLUTION:
        at, pt = TRACKING_AREA_THRESHOLD, TRACKING_PIXEL_THRESHOLD
    else:
        at, pt = SEARCHING_AREA_THRESHOLD, SEARCHING_PIXEL_THRESHOLD
    dets = []
    for b in img.find_blobs(thresh, area_threshold=at, pixels_threshold=pt, merge=False):
        dets.append(mapped_blob(b))  # already in sensor coords
    return dets


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


def target_point(tracks):
    #  Sensor-coord point to center the window on -> FIX when multiple blobs
    n = len(tracks)
    return (sum(t.u.p for t in tracks) / n,
            sum(t.v.p for t in tracks) / n)


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
    detections = find_detections(img)
    associate(tracks, detections)

    if all(t.misses > MAX_MISSES for t in tracks):
        print("Lost all tracks -> re-init")
        tracks = init_tracks()
        last_t = time.ticks_ms()
        continue

    tx, ty = target_point(tracks)
    wc_x, wc_y = window_center_sensor()
    _, _, ww, wh = cam.ioctl(csi.IOCTL_GET_READOUT_WINDOW)
    if (abs(tx - wc_x) > TRACKING_EDGE_TOLERANCE * ww or
            abs(ty - wc_y) > TRACKING_EDGE_TOLERANCE * wh or
            current_res != TRACKING_RESOLUTION):
        center_window(tx, ty, TRACKING_RESOLUTION)

    # map estimate back into the current image frame
    for t in tracks:
        ix, iy = sensor_to_image(t.u.p, t.v.p)
        lx, ly = int(ix), int(iy)
        if 0 <= lx < cam.width() and 0 <= ly < cam.height():
            img.draw_cross(lx, ly, color=255)
            img.draw_string(lx + 3, ly + 3, str(t.id), color=255)

    print("fps=%.1f | " % clock.fps() +
          " ".join("id%d(%.1f,%.1f)m%d" % (t.id, t.u.p, t.v.p, t.misses) for t in tracks))
