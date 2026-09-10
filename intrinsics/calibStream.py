#!/usr/bin/env python3
"""calibStream.py - hands-free intrinsic calibration capture, OpenMV RT1062.

Run it, open http://localhost:8420, press BEGIN, and work the board.
Frames land in intrinsics/images/camN/ as full-res .pgm, already where
calibrateIntrinsics.m looks for them. No SD card needed.

Read PROTOCOL.md section 7a first - especially BRACE THE BOARD. The sensor
is rolling shutter with a 74 ms full-res readout, and motion during readout
shears the target. That was the dominant error source in the first run
(r = +0.84 against per-image reprojection error).

The camera stays in ONE configuration (GRAYSCALE / WQXGA2 2592x1944). The
preview is a 4x downscale of the same frame the capture saves. The running
loop polls stdin, so a capture needs no reprogramming - press nothing, just
hold the board still where the box is green.

  http://localhost:8420/     Begin calibration / Stop, live view, coverage
"""
import serial, glob, time, base64, threading, json, os
import numpy as np, cv2
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

SP = os.path.dirname(os.path.abspath(__file__))
CAM_ID = int(os.environ.get("CAM_ID", "2"))
OUT = os.path.join(SP, "images", "cam%d" % CAM_ID)
PORT = 8420
SENSOR_W, SENSOR_H = 2592, 1944
# measured by calibration on 2026-09-10 (was a 1215 guess from an assumed
# board distance - the fit says 2323, so the distance readout was 1.9x out)
F_SENSOR_PX = 2323.0
PREVIEW_DIV = 4
SQUARE_M = 0.040
GRID = (9, 6)

# auto-shoot gates
REACH_MIN = 70          # % of radial field the outermost corner must reach
STABLE_FRAMES = 3       # consecutive steady frames = held still
# Two motion budgets, and the tighter one wins.
#  BLUR  - smear within one exposure (8-40 ms). Softens corners.
#  SHEAR - the sensor is ROLLING SHUTTER and a full-res readout takes 74 ms, so
#          a board spanning 1000 rows is scanned over ~38 ms. Motion during that
#          shears the board. Measured r = +0.84 against per-image reprojection
#          error - this is the dominant error source, and it is ~10x tighter
#          than the blur budget. It is why the board must be BRACED.
MAX_BLUR_PX = 0.7       # preview px of smear during the exposure
MAX_SHEAR_PX = 0.5      # SENSOR px of shear across the board during readout
READOUT_MS = 74.0       # full-res frame readout time
SENSOR_ROWS = 1944
MIN_STABLE_PX = 0.6     # detection jitter floor - do not demand better
EDGE_MARGIN_FRAC = 0.04 # board must sit this far inside the frame
NOVEL_PX = 55.0         # board centre must move this far from the last shot
NOVEL_SCALE = 0.15      # ...or corner spacing change by this fraction
COOLDOWN_S = 1.5
TARGET_SHOTS = 40
TILT_MIN_DEG = 15       # a shot counts as "tilted" above this
TRIGGER_MODE = os.environ.get("TRIGGER_MODE", "stdin")  # stdin | reprogram
WEDGE_S = 12.0          # no preview frames for this long = camera wedged

CAM = open(os.path.join(SP, "calibStreamCam.py.tmpl")).read()
# with the safe trigger we never poll stdin at all
CAM_SAFE = CAM.replace("poll = select.poll(); poll.register(sys.stdin, select.POLLIN)",
                       "poll = None").replace("while poll.poll(0):", "while False:")
health = {"last_frame": 0.0, "wedges": 0}
SHOOT_ONE = """
import csi, binascii
c = csi.CSI(); c.reset(); c.pixformat(csi.GRAYSCALE)
c.framesize(csi.WQXGA2); c.snapshot(time=2000)
c.auto_gain(False, gain_db=%d)
c.auto_exposure(False, exposure_us=%d)
for _ in range(3): c.snapshot()
img = c.snapshot()
n = img.width()*img.height(); b = img.bytearray()
print("#SHOT %%d %%d %%d" %% (img.width(), img.height(), n))
i = 0; step = 4800
while i < n:
    e = i + step
    if e > n: e = n
    print(binascii.b2a_base64(b[i:e]).decode().strip())
    i = e
print("#SHOTEND")
"""

state = {"jpeg": None, "stats": {}, "cells": {}, "n": 0, "shots": 0, "tilted": 0,
         "rejected": 0, "pending": 0, "last_shot": "", "busy": False,
         "running": False, "msg": "idle"}
import queue
verify_q = queue.Queue()
params = {"exp": 8000, "gain": 12, "ae": 1, "dirty": True, "trigger": False}
track = {"prev": None, "still": 0, "last_pose": None, "t_last": 0.0,
         "t_frame": 0.0, "dt": 0.2, "drift": 0.0, "allow": 0.0, "bind": "-"}
PREV_W = SENSOR_W // PREVIEW_DIV      # 648
PREV_H = SENSOR_H // PREVIEW_DIV      # 486
cam = {"exp_us": 8000}
lock = threading.Lock()

OBJP = np.zeros((GRID[0] * GRID[1], 3), np.float32)
OBJP[:, :2] = np.mgrid[0:GRID[0], 0:GRID[1]].T.reshape(-1, 2) * (SQUARE_M * 1000.0)


def tilt_deg(pts, W, H, size):
    """Angle between the board normal and the camera axis, via solvePnP."""
    try:
        f = F_SENSOR_PX / PREVIEW_DIV
        K = np.array([[f, 0, W / 2.0], [0, f, H / 2.0], [0, 0, 1]], np.float64)
        objp = np.zeros((size[0] * size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:size[0], 0:size[1]].T.reshape(-1, 2) * (SQUARE_M * 1000.0)
        ok, rvec, _ = cv2.solvePnP(objp, pts.astype(np.float32), K, None,
                                   flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok: return 0.0
        R, _ = cv2.Rodrigues(rvec)
        n = R @ np.array([0.0, 0.0, 1.0])
        return float(np.degrees(np.arccos(min(1.0, abs(n[2])))))
    except Exception:
        return 0.0


def analyse(img):
    H, W = img.shape[:2]
    st = {"found": False, "frame_max": int(img.max())}
    pts = None; size = None
    for sz in [GRID, (GRID[1], GRID[0])]:
        ok, c = cv2.findChessboardCorners(img, sz,
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
        if ok:
            pts = c.reshape(-1, 2); size = sz; break
    if pts is None:
        return st, None, None
    x0, y0 = pts.min(0); x1, y1 = pts.max(0); cx, cy = pts.mean(0)
    r = np.hypot(pts[:, 0] - W / 2, pts[:, 1] - H / 2); rf = np.hypot(W / 2, H / 2)
    g = pts.reshape(size[1], size[0], 2)
    d = [np.linalg.norm(g[a, b + 1] - g[a, b])
         for a in range(size[1]) for b in range(size[0] - 1)]
    sp = float(np.mean(d))
    patch = img[int(y0):int(y1) + 1, int(x0):int(x1) + 1]
    st.update({"found": True, "dist_m": round((F_SENSOR_PX / PREVIEW_DIV) * SQUARE_M / sp, 2),
               "fill_pct": int(round(100 * (y1 - y0) / H)),
               "reach_pct": int(round(100 * r.max() / rf)),
               "cell": "%d%d" % (min(2, int(cy / (H / 3.0))), min(2, int(cx / (W / 3.0)))),
               "board_max": int(patch.max()), "spacing_px": round(sp, 1),
               "clipping": bool((patch >= 250).mean() > 0.01),
               "tilt_deg": int(round(tilt_deg(pts, W, H, size)))})
    return st, pts, size


def gate(st, pts):
    """Should we fire? Returns (fire, reason-for-the-overlay)."""
    if not state["running"]:
        return False, "press BEGIN"
    if state["shots"] >= TARGET_SHOTS:
        return False, "target reached - press STOP"
    if not st.get("found"):
        track["prev"] = None; track["still"] = 0
        return False, "no board"
    if st["clipping"]:
        return False, "clipping - angle away from the light"
    if st["reach_pct"] < REACH_MIN:
        return False, "move toward a corner (reach %d%%)" % st["reach_pct"]
    prev = track["prev"]
    track["prev"] = pts.copy()
    if prev is None or prev.shape != pts.shape:
        track["still"] = 0
        return False, "hold still"
    x0, y0 = pts.min(0); x1, y1 = pts.max(0)
    exp_s = max(cam["exp_us"], 1) / 1e6
    allow_blur = (MAX_BLUR_PX / exp_s) * track["dt"]
    rows_sensor = (y1 - y0) * PREVIEW_DIV
    t_read = (READOUT_MS / 1000.0) * max(rows_sensor, 1) / SENSOR_ROWS
    allow_shear = ((MAX_SHEAR_PX / t_read) / PREVIEW_DIV) * track["dt"]
    allow = float(max(MIN_STABLE_PX, min(allow_blur, allow_shear)))
    # The capture lands ~2 preview frames after this decision, so keep the board
    # far enough inside the frame that drift cannot clip it. A clipped board
    # fails full-res detection outright - that cost 5 of 9 losses last run.
    mx = max(EDGE_MARGIN_FRAC * PREV_W, 2.0 * allow)
    my = max(EDGE_MARGIN_FRAC * PREV_H, 2.0 * allow)
    if x0 < mx or y0 < my or x1 > PREV_W - mx or y1 > PREV_H - my:
        track["still"] = 0
        return False, "too close to the frame edge - bring it in slightly"
    drift = float(np.median(np.hypot(*(pts - prev).T)))
    track["drift"] = drift; track["allow"] = allow
    track["bind"] = "shear" if allow_shear < allow_blur else "blur"
    track["still"] = track["still"] + 1 if drift < allow else 0
    if track["still"] < STABLE_FRAMES:
        return False, "hold still  %.2f/%.2f px (%s)  [%s]" % (
            drift, allow, track["bind"],
            "#" * track["still"] + "." * (STABLE_FRAMES - track["still"]))
    if time.time() - track["t_last"] < COOLDOWN_S:
        return False, "..."
    lp = track["last_pose"]
    if lp is not None:
        moved = float(np.hypot(*(pts.mean(0) - lp[0])))
        scale = abs(st["spacing_px"] - lp[1]) / max(lp[1], 1e-6)
        if moved < NOVEL_PX and scale < NOVEL_SCALE:
            return False, "move to a new position"
    return True, "CAPTURING"


def annotate(img, st, pts, size, reason):
    H, W = img.shape[:2]
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    for fx in (W // 3, 2 * W // 3): cv2.line(vis, (fx, 0), (fx, H), (70, 70, 70), 1)
    for fy in (H // 3, 2 * H // 3): cv2.line(vis, (0, fy), (W, fy), (70, 70, 70), 1)
    if pts is not None:
        cv2.drawChessboardCorners(vis, size, pts.reshape(-1, 1, 2).astype(np.float32), True)
        x0, y0 = pts.min(0); x1, y1 = pts.max(0)
        col = ((0, 235, 0) if st["reach_pct"] >= REACH_MIN and not st["clipping"]
               else ((0, 200, 255) if st["reach_pct"] >= 50 else (60, 60, 255)))
        cv2.rectangle(vis, (int(x0), int(y0)), (int(x1), int(y1)), col, 3)
        lines = ["%.2f m   reach %d%%   tilt %d deg" % (st["dist_m"], st["reach_pct"], st["tilt_deg"]),
                 "board max %d   exp %d us%s" % (st["board_max"], cam["exp_us"],
                                                 "   CLIPPING" if st["clipping"] else "")]
    else:
        lines = ["no board detected"]
    with lock:
        cells = dict(state["cells"]); shots = state["shots"]; tl = state["tilted"]
        busy = state["busy"]; last = state["last_shot"]
    with lock:
        rej = state["rejected"]; pend_n = state["pending"]
    lines.append("%d/%d verified   %d tilted   %d rejected%s" % (
        shots, TARGET_SHOTS, tl, rej, "   (%d verifying)" % pend_n if pend_n else ""))
    lines.append(last)
    lines.append("SAVING..." if busy else reason)
    y = 20
    for i, t in enumerate(lines):
        sc = 0.62 if i == len(lines) - 1 else 0.5
        cv2.putText(vis, t, (8, y), cv2.FONT_HERSHEY_SIMPLEX, sc, (0, 0, 0), 4)
        cv2.putText(vis, t, (8, y), cv2.FONT_HERSHEY_SIMPLEX, sc,
                    (0, 255, 0) if t == "CAPTURING" else (255, 255, 255), 1)
        y += 22
    bx, by, s = W - 78, 8, 22
    cv2.putText(vis, "coverage", (bx - 2, by - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
    for ri in range(3):
        for ci in range(3):
            x = bx + ci * s; yy = by + ri * s + 4
            k = cells.get("%d%d" % (ri, ci), 0)
            cv2.rectangle(vis, (x, yy), (x + s - 3, yy + s - 3),
                          (0, 160, 0) if k else (60, 60, 60), -1 if k else 1)
            if k:
                cv2.putText(vis, str(k), (x + 6, yy + s - 8), cv2.FONT_HERSHEY_SIMPLEX,
                            0.4, (255, 255, 255), 1)
    return vis


def verifier():
    """A frame that will not detect at full resolution is worthless, and the
    downscaled preview is more forgiving than the real thing - so check the
    saved file and delete it if it fails. The set is then good by construction."""
    while True:
        path, cell, tilt = verify_q.get()
        ok = False; why = ""
        try:
            im = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if im is None:
                why = "unreadable"
            else:
                for sz in [GRID, (GRID[1], GRID[0])]:
                    found_ok, c = cv2.findChessboardCorners(im, sz,
                        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
                    if found_ok:
                        p = c.reshape(-1, 2)
                        h, w = im.shape
                        m = EDGE_MARGIN_FRAC * min(w, h) * 0.5
                        if (p[:, 0].min() < m or p[:, 1].min() < m or
                                p[:, 0].max() > w - m or p[:, 1].max() > h - m):
                            why = "board touches the frame edge"
                        else:
                            ok = True
                        break
                else:
                    why = "no board at full res (blur / shear / clipped)"
        except Exception as e:
            why = repr(e)
        name = os.path.basename(path)
        with lock:
            state["pending"] = max(0, state["pending"] - 1)
            if ok:
                state["shots"] += 1
                state["cells"][cell] = state["cells"].get(cell, 0) + 1
                if tilt >= TILT_MIN_DEG:
                    state["tilted"] += 1
                state["last_shot"] = "%s ok" % name
            else:
                state["rejected"] += 1
                state["last_shot"] = "REDO - %s: %s" % (name, why)
        if not ok:
            try: os.remove(path)
            except Exception: pass
        print(("verified %s" % name) if ok else ("REJECTED %s - %s" % (name, why)), flush=True)


def next_index():
    n = 0
    for f in glob.glob(os.path.join(OUT, "img_*.pgm")):
        try: n = max(n, int(os.path.basename(f)[4:7]))
        except Exception: pass
    return n + 1


def save_pgm(path, w, h, raw):
    with open(path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (w, h)); f.write(raw)


def program(ser):
    ser.write(b"\r\x03\x03"); time.sleep(0.5); ser.reset_input_buffer()
    ser.write(b"\x01"); time.sleep(0.4); ser.read(1 << 16)
    tmpl = CAM if TRIGGER_MODE == "stdin" else CAM_SAFE
    code = tmpl % {"ae": params["ae"], "gain": params["gain"], "exp": params["exp"]}
    data = code.encode()
    for i in range(0, len(data), 256):
        ser.write(data[i:i + 256]); ser.flush(); time.sleep(0.02)
    ser.write(b"\x04"); time.sleep(0.4); ser.reset_input_buffer()
    params["dirty"] = False
    print("camera programmed exp=%d gain=%d ae=%d" % (params["exp"], params["gain"], params["ae"]), flush=True)


def clean_start(ser):
    """Reconnecting to a camera that is mid-dump is what wedges the USB VCP.
    Drain whatever is in flight, interrupt, soft-reboot, then start clean."""
    ser.write(b"\r\x03\x03"); time.sleep(0.4)
    t = time.time(); n = 0
    while time.time() - t < 3.0:
        c = ser.read(1 << 16)
        if c: n += len(c); t = time.time()
    if n: print("drained %d bytes of stale output" % n, flush=True)
    ser.write(b"\x03\x03"); time.sleep(0.3); ser.reset_input_buffer()
    ser.write(b"\x04"); time.sleep(2.5)          # soft reboot
    ser.reset_input_buffer()
    print("camera soft-rebooted", flush=True)


def reader():
    ser = serial.Serial(sorted(glob.glob("/dev/cu.usbmodem*"))[0], 115200, timeout=0.3)
    try: ser.set_buffer_size(rx_size=1 << 22)
    except Exception: pass
    clean_start(ser)
    program(ser)
    buf = b""; cur = None; want = -1; shot = None; sw = sh = sn = 0; t0 = 0.0
    pend = None
    while True:
        if params["dirty"]:
            program(ser); buf = b""; cur = None; shot = None
        if params["trigger"] and shot is None:
            params["trigger"] = False
            t0 = time.time()
            with lock: state["busy"] = True
            if TRIGGER_MODE == "stdin":
                ser.write(b"S"); ser.flush()
            else:
                code = (CAM_SAFE % {"ae": 0, "gain": params["gain"],
                                    "exp": params["exp"]}).replace("while True:", "if True:")
                ser.write(b"\r\x03\x03"); time.sleep(0.4); ser.reset_input_buffer()
                ser.write(b"\x01"); time.sleep(0.3); ser.read(1 << 16)
                one = SHOOT_ONE % (params["gain"], params["exp"])
                d = one.encode()
                for i in range(0, len(d), 256):
                    ser.write(d[i:i+256]); ser.flush(); time.sleep(0.02)
                ser.write(b"\x04"); time.sleep(0.3)
                params["dirty"] = True  # back to preview once the dump completes
        chunk = ser.read(1 << 16)
        if not chunk: continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            line = line.strip()
            if not line.startswith(b"#"):
                if shot is not None: shot.append(line)
                elif cur is not None: cur.append(line)
                continue
            f = line.split(); tag = f[0]
            if tag == b"#FS":
                cur = []
                try:
                    want = int(f[1])
                    if len(f) > 2: cam["exp_us"] = int(f[2])
                except Exception: want = -1
            elif tag == b"#SHOT":
                try: sw, sh, sn = int(f[1]), int(f[2]), int(f[3]); shot = []
                except Exception:
                    shot = None
                    with lock: state["busy"] = False
            elif tag == b"#SHOTEND":
                if shot is None:
                    with lock: state["busy"] = False
                    continue
                raw = base64.b64decode(b"".join(shot)); shot = None
                if len(raw) == sn:
                    idx = next_index()
                    path = os.path.join(OUT, "img_%03d.pgm" % idx)
                    save_pgm(path, sw, sh, raw)
                    pend = pend or globals().get("_pend_manual") or ("11", 0)
                    with lock: state["pending"] += 1
                    verify_q.put((path, pend[0], pend[1]))
                    msg = "img_%03d.pgm saved (%.1fs) - verifying" % (idx, time.time() - t0)
                else:
                    msg = "TRUNCATED %d/%d - not saved" % (len(raw), sn)
                print(msg, flush=True)
                with lock: state["last_shot"] = msg; state["busy"] = False
                track["t_last"] = time.time()
            elif tag == b"#FE":
                if cur is None: continue
                try:
                    raw = base64.b64decode(b"".join(cur)); cur = None
                    if want > 0 and len(raw) != want: continue
                    img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_GRAYSCALE)
                    if img is None: continue
                    st, pts, size = analyse(img)
                    fire, reason = (False, "SAVING...") if state["busy"] else gate(st, pts)
                    if fire:
                        pend = (st["cell"], st["tilt_deg"])
                        track["last_pose"] = (pts.mean(0).copy(), st["spacing_px"])
                        track["still"] = 0
                        params["trigger"] = True
                    vis = annotate(img, st, pts, size, reason)
                    ok, enc = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if ok:
                        with lock:
                            state["jpeg"] = enc.tobytes(); state["stats"] = st
                            state["n"] += 1; state["msg"] = reason
                        now = time.time()
                        if track["t_frame"]:
                            track["dt"] = 0.7 * track["dt"] + 0.3 * (now - track["t_frame"])
                        track["t_frame"] = now
                        health["last_frame"] = now
                except Exception:
                    cur = None
            else:
                cur = None


def watchdog():
    """A wedged camera stops sending frames. Reprogram; if that fails twice,
    the USB VCP itself is stuck and only a physical replug clears it."""
    global TRIGGER_MODE
    while True:
        time.sleep(3.0)
        if health["last_frame"] == 0.0 or state["busy"] or params["dirty"]:
            continue
        idle = time.time() - health["last_frame"]
        if idle < WEDGE_S:
            continue
        health["wedges"] += 1
        health["last_frame"] = time.time()
        print("WATCHDOG: no frames for %.0fs (wedge #%d) - reprogramming"
              % (idle, health["wedges"]), flush=True)
        if health["wedges"] >= 2 and TRIGGER_MODE == "stdin":
            TRIGGER_MODE = "reprogram"
            print("WATCHDOG: falling back to the safe (reprogram) trigger", flush=True)
        with lock:
            state["msg"] = "camera stalled - recovering"
        params["dirty"] = True


def supervised():
    while True:
        try: reader()
        except Exception as e:
            import traceback; print("reader crashed: %r" % (e,), flush=True)
            traceback.print_exc()
            with lock: state["busy"] = False
            params["dirty"] = True; time.sleep(2.0)


PAGE = b"""<!doctype html><meta charset=utf-8><title>cam2 calibration</title>
<style>html,body{height:100%;margin:0}
body{background:#111;color:#eee;font:13px system-ui;display:flex;flex-direction:column;
align-items:center;justify-content:flex-start;overflow:hidden}
img{flex:1 1 auto;min-height:0;max-width:100vw;object-fit:contain;image-rendering:pixelated}
.bar{flex:0 0 auto;display:flex;align-items:center;gap:14px;padding:6px 10px}
button{font:600 18px system-ui;padding:10px 30px;border:0;border-radius:9px;
background:#2b7;color:#022;cursor:pointer}button.stop{background:#c44;color:#fff}
p{margin:0;color:#aaa;font-size:12px;text-align:left;max-width:60ch}</style>
<img src="/stream">
<div class=bar><button id=b onclick="go()">BEGIN CALIBRATION</button>
<p>Hold the board where the box is green - it shoots by itself and beeps.
Then move somewhere new. The overlay shows drift vs the allowed blur budget.</p></div>
<script>
let on=false, ac=null, last=0, rej=0;
function go(){ac=ac||new (window.AudioContext||window.webkitAudioContext)();ac.resume();
 on=!on;fetch('/run?on='+(on?1:0));
 const b=document.getElementById('b');
 b.textContent=on?'STOP':'BEGIN CALIBRATION';b.className=on?'stop':'';}
function beep(f,d){if(!ac)return;const o=ac.createOscillator(),g=ac.createGain();
 o.connect(g);g.connect(ac.destination);o.frequency.value=f;g.gain.value=0.15;
 o.start();o.stop(ac.currentTime+d);}
setInterval(async()=>{try{const s=await(await fetch('/stats')).json();
 if(s.rejected>rej){rej=s.rejected;beep(200,0.35);}
 if(s.shots>last){last=s.shots;beep(880,0.12);
   if(s.shots>=s.target){setTimeout(()=>{beep(1320,0.5)},200);}}
 }catch(e){}},400);
</script>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def _json(self, o):
        b = json.dumps(o).encode()
        self.send_response(200); self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)

    def do_GET(self):
        p = self.path
        if p.startswith("/run"):
            import urllib.parse as up
            q = up.parse_qs(up.urlparse(p).query)
            state["running"] = q.get("on", ["1"])[0] not in ("0", "false", "off")
            track["last_pose"] = None; track["still"] = 0
            return self._json({"running": state["running"]})
        if p.startswith("/shoot"):
            if state["busy"]: return self._json({"error": "busy"})
            with lock:
                st = dict(state["stats"])
            if not st.get("found"):
                return self._json({"error": "no board in view"})
            pend_cell = st.get("cell", "--"); pend_tilt = st.get("tilt_deg", 0)
            globals()["_pend_manual"] = (pend_cell, pend_tilt)
            params["trigger"] = True
            return self._json({"ok": True, "shot": state["shots"] + 1})
        if p.startswith("/exp"):
            import urllib.parse as up
            q = up.parse_qs(up.urlparse(p).query)
            if "us" in q: params["exp"] = max(50, min(200000, int(q["us"][0]))); params["ae"] = 0
            if "gain" in q: params["gain"] = max(0, min(24, int(q["gain"][0])))
            if "ae" in q: params["ae"] = 1 if q["ae"][0] not in ("0","false","off") else 0
            params["dirty"] = True
            return self._json({"exp": params["exp"], "gain": params["gain"], "ae": params["ae"]})
        if p.startswith("/reset"):
            with lock: state["cells"] = {}; state["shots"] = 0; state["tilted"] = 0; state["rejected"] = 0
            track["last_pose"] = None
            return self._json({"ok": True})
        if p.startswith("/stats"):
            with lock:
                return self._json({"stats": state["stats"], "cells": state["cells"],
                                   "frames": state["n"], "shots": state["shots"],
                                   "tilted": state["tilted"], "target": TARGET_SHOTS,
                                   "rejected": state["rejected"],
                                   "pending": state["pending"],
                                   "running": state["running"], "busy": state["busy"],
                                   "msg": state["msg"], "last_shot": state["last_shot"],
                                   "exp": params["exp"], "gain": params["gain"],
                                   "trigger_mode": TRIGGER_MODE, "wedges": health["wedges"]})
        if p.startswith("/stream"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
            self.end_headers()
            try:
                while True:
                    with lock: j = state["jpeg"]
                    if j:
                        self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(j)).encode() + b"\r\n\r\n" + j + b"\r\n")
                    time.sleep(0.05)
            except Exception:
                return
        self.send_response(200); self.send_header("Content-Type", "text/html")
        self.send_header("Content-Length", str(len(PAGE))); self.end_headers()
        self.wfile.write(PAGE)


def selftest():
    """Camera scripts are format templates; an unescaped %d inside them is a
    silent landmine that only fires at capture time. Check at startup."""
    a = CAM % {"ae": 1, "gain": 12, "exp": 8000}
    b = CAM_SAFE % {"ae": 1, "gain": 12, "exp": 8000}
    c = SHOOT_ONE % (12, 8000)
    for name, code in (("CAM", a), ("CAM_SAFE", b), ("SHOOT_ONE", c)):
        for marker in ("#SHOT ", "#FS ") if name != "SHOOT_ONE" else ("#SHOT ",):
            if marker not in code:
                raise SystemExit("selftest: %s lost marker %r" % (name, marker))
        if "%d" in code.split("print(")[0]:
            raise SystemExit("selftest: %s has an unsubstituted %%d" % name)
    print("selftest ok: all camera templates render", flush=True)


if __name__ == "__main__":
    selftest()
    os.makedirs(OUT, exist_ok=True)
    threading.Thread(target=supervised, daemon=True).start()
    threading.Thread(target=watchdog, daemon=True).start()
    threading.Thread(target=verifier, daemon=True).start()
    time.sleep(3)
    ThreadingHTTPServer(("127.0.0.1", PORT), H).serve_forever()
