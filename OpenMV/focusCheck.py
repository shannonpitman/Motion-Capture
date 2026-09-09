# Focus readout with BRIGHTNESS-INDEPENDENT spot-size metrics.
#   ew    = flux / peak          (equivalent width, px^2)  -> MINIMISE
#   rms_r = sqrt(sum w r^2/sum w) (RMS radius, px)         -> MINIMISE
# Both are ratios of the profile to itself, so they do not move when the
# source dims. `area` (pixels over a fixed threshold) DOES, so it is only
# reported for continuity - do not focus on it.
import csi, time
EXP = 10000        # fixed exposure, us. Lower it if peak pins at 255.
DUR = 8            # seconds per burst
SEARCH_EXP = 60000  # exposure for the initial locate only
SW, SH = 2592, 1944
THRESH_LO = 80; FLOOR = 50; PAD = 4
c = csi.CSI(); c.reset(); c.pixformat(csi.GRAYSCALE)
c.framesize(csi.VGA); c.snapshot(time=1000)
c.auto_gain(False, gain_db=0)
c.auto_exposure(False, exposure_us=SEARCH_EXP)
for _ in range(15): c.snapshot()
img = c.snapshot()
bs = img.find_blobs([(50,255)], area_threshold=2, pixels_threshold=2, merge=True)
if not bs:
    print("FOCUS error=no_marker"); raise SystemExit
b = max(bs, key=lambda z: z.pixels); W, H = img.width(), img.height()
r = min(SW/float(W), SH/float(H))
cx = (b.cxf-W*0.5)*r + SW/2.0; cy = (b.cyf-H*0.5)*r + SH/2.0
c.framesize(csi.VGA); c.snapshot()
c.ioctl(csi.IOCTL_SET_READOUT_WINDOW, (int(cx-SW/2.0), int(cy-SH/2.0), 640, 480))
for _ in range(10): c.snapshot()
c.auto_exposure(False, exposure_us=EXP)   # re-apply AFTER the window change
for _ in range(15): c.snapshot()
th = [(THRESH_LO,255)]
print("located sensor (%.0f, %.0f) | exposure %d us" % (cx, cy, c.exposure_us()))
print("%6s %7s %8s %8s %7s"%("t_s","peak","ew_px2","rms_r","area"))
t0=time.ticks_ms(); best=None
while True:
    el=time.ticks_diff(time.ticks_ms(),t0)/1000.0
    if el>DUR: break
    W2=c.width(); H2=c.height()
    pk=0.0; ew=0.0; rr=0.0; ar=0.0; n=0
    for _ in range(6):
        im=c.snapshot(); bf=im.bytearray()
        bb=im.find_blobs(th, area_threshold=2, pixels_threshold=2, merge=True)
        if not bb: continue
        z=max(bb,key=lambda q:q.pixels)
        x0=max(0,z.x-PAD); y0=max(0,z.y-PAD)
        x1=min(W2,z.x+z.w+PAD); y1=min(H2,z.y+z.h+PAD)
        fp=0; sw=0.0; sx=0.0; sy=0.0
        y=y0
        while y<y1:
            base=y*W2; x=x0
            while x<x1:
                p=bf[base+x]
                if p>fp: fp=p
                if p>FLOOR:
                    wgt=p-FLOOR; sw+=wgt; sx+=wgt*x; sy+=wgt*y
                x+=1
            y+=1
        if sw<=0 or fp<=FLOOR: continue
        ccx=sx/sw; ccy=sy/sw
        s2=0.0; y=y0
        while y<y1:
            base=y*W2; x=x0
            while x<x1:
                p=bf[base+x]
                if p>FLOOR:
                    wgt=p-FLOOR; dx=x-ccx; dy=y-ccy
                    s2+=wgt*(dx*dx+dy*dy)
                x+=1
            y+=1
        pk+=fp; ew+=sw/float(fp-FLOOR); rr+=(s2/sw)**0.5; ar+=z.pixels; n+=1
    if not n: continue
    pk/=n; ew/=n; rr/=n; ar/=n
    if best is None or ew<best[1]: best=(pk,ew,rr,el)
    print("%6.1f %7.1f %8.1f %8.2f %7.0f"%(el,pk,ew,rr,ar))
if best: print("FOCUS BEST ew=%.1f px2  rms_r=%.2f px  peak=%.1f at t=%.1fs"%(best[1],best[2],best[0],best[3]))
