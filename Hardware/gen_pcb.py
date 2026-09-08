#!/usr/bin/env python3
"""gen_pcb.py - PCB layout for the wand strobe driver.

Two layers, 80 x 60 mm. Nets come from the schematic netlist, so the board
cannot silently disagree with the schematic; DRC is the check that the routing
below actually implements them.

The layout follows the rule from the README: the switching current and the
high-impedance timing network live in separate regions and meet only at the
bulk capacitor's ground. B.Cu is a GND pour, so GND is not routed by hand -
through-hole pads reach it directly and SMD grounds get a via.

Shannon Pitman
"""

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from kipcb import Pcb                                            # noqa: E402

NET = os.path.join(HERE, "WandStrobeDriver", "WandStrobeDriver.net")
OUT = os.path.join(HERE, "WandStrobeDriver", "WandStrobeDriver.kicad_pcb")
KCLI = "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli"

# board edges
X0, Y0, X1, Y1 = 100.0, 55.0, 180.0, 115.0

FP = dict(
    term="TerminalBlock_Phoenix:TerminalBlock_Phoenix_MPT-0,5-2-2.54_1x02_P2.54mm_Horizontal",
    sw="Button_Switch_THT:SW_Slide_SPDT_Angled_CK_OS102011MA1Q",
    fuse="Fuse:Fuse_Bourns_MF-RG300",
    sma="Diode_SMD:D_SMA",
    cel10="Capacitor_THT:CP_Radial_D10.0mm_P5.00mm",
    cel5="Capacitor_THT:CP_Radial_D5.0mm_P2.00mm",
    cfilm="Capacitor_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm",
    c805="Capacitor_SMD:C_0805_2012Metric",
    r805="Resistor_SMD:R_0805_2012Metric",
    rtht="Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal",
    dip8="Package_DIP:DIP-8_W7.62mm_Socket",
    sot23="Package_TO_SOT_SMD:SOT-23",
    jst="Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical",
    tp="TestPoint:TestPoint_Pad_D1.5mm",
    led805="LED_SMD:LED_0805_2012Metric",
)


def build():
    b = Pcb("WandStrobeDriver", netlist=NET,
            title="Wand strobe driver rev B - 9 V")

    # ---- placement ---------------------------------------------------------
    J1 = b.place(FP["term"], "J1", "9V IN", (105, 68), 90, ref_at=(9, 0))
    SW1 = b.place(FP["sw"], "SW1", "POWER", (116, 62))
    F1 = b.place(FP["fuse"], "F1", "500mA", (127, 62))
    D1 = b.place(FP["sma"], "D1", "SS34", (139, 63.2), 180)

    C1 = b.place(FP["cel10"], "C1", "1000u", (108, 78), ref_at=(2.5, -6.5))
    C2 = b.place(FP["c805"], "C2", "100n", (120, 76), 270)
    R5 = b.place(FP["r805"], "R5", "22k", (126, 76), 270)
    D2 = b.place(FP["led805"], "D2", "PWR", (126, 81), 90)

    R10 = b.place(FP["r805"], "R10", "100", (144, 66), 270)
    C3 = b.place(FP["cel5"], "C3", "10u", (144, 76))
    C4 = b.place(FP["c805"], "C4", "100n", (146, 70), 270)

    R4 = b.place(FP["r805"], "R4", "100k", (154, 66), 270)
    R3 = b.place(FP["r805"], "R3", "220", (154, 86))
    Q1 = b.place(FP["sot23"], "Q1", "DMP3098L", (162, 80), 90)

    U1 = b.place(FP["dip8"], "U1", "TLC555", (114, 90))
    R1 = b.place(FP["rtht"], "R1", "1M", (130, 86))
    R2 = b.place(FP["rtht"], "R2", "3k3", (136, 94), 180)
    C6 = b.place(FP["cfilm"], "C6", "470n", (130, 108))
    C5 = b.place(FP["c805"], "C5", "10n", (121.62, 102), 90)

    J2 = b.place(FP["jst"], "J2", "MARKER A", (172, 66), 90)
    J3 = b.place(FP["jst"], "J3", "MARKER B", (172, 78), 90)
    J4 = b.place(FP["jst"], "J4", "MARKER C", (172, 90), 90)
    J5 = b.place(FP["jst"], "J5", "MARKER D", (172, 102), 90)

    TP1 = b.place(FP["tp"], "TP1", "GATE", (142, 108))
    TP2 = b.place(FP["tp"], "TP2", "SW_LED", (168, 108))
    TP3 = b.place(FP["tp"], "TP3", "GND", (158, 108))

    # ---- input chain -------------------------------------------------------
    b.track("Net-(J1-Pin_1)", J1("1"), (105, 71), (118, 71), SW1("2"))
    b.track("Net-(SW1-A)", SW1("1"), (116, 58), (127, 58), F1("1"))
    b.track("Net-(D1-A)", F1("2"), D1("2"))

    # ---- +9 V --------------------------------------------------------------
    P9 = "+9V"
    b.track(P9, D1("1"), (141, 73), (108, 73))
    b.track(P9, (108, 73), C1("1"))
    b.track(P9, (120, 73), C2("1"))
    b.track(P9, (126, 73), R5("1"))
    b.track(P9, D1("1"), (144, 63.2), R10("1"))
    b.track(P9, (144, 63.2), (154, 63.2), R4("1"))
    b.track(P9, (154, 63.2), (149, 63.2), (149, 88), (162.95, 88), Q1("2"))

    # ---- filtered VCC ------------------------------------------------------
    V = "VCC"
    b.track(V, R10("2"), (144, 82), (130, 82), R1("1"))     # C3 pad1 sits on it
    b.track(V, (144, 68), (146, 68), C4("1"))
    b.track(V, (134, 82), (134, 88), (124, 88), (124, 90), U1("8"))
    b.track(V, U1("4"), (114, 100), (129, 100), (129, 90), U1("8"), layer="B.Cu")

    # ---- indicator ---------------------------------------------------------
    b.track("Net-(D2-A)", R5("2"), D2("2"))

    # ---- timing ------------------------------------------------------------
    b.track("Net-(U1-DISCH)", R1("2"), (140.16, 90), (136, 90), R2("1"))
    b.track("Net-(U1-DISCH)", (136, 92.54), U1("7"))
    T = "Net-(U1-THRES)"
    b.track(T, R2("2"), (125.84, 105), (112.5, 105), (112.5, 108), C6("1"))
    b.track(T, (125.84, 95.08), U1("6"))
    b.track(T, U1("2"), (110, 92.54), (110, 108), (112.5, 108))
    b.track("Net-(U1-CONT)", C5("2"), U1("5"))

    # ---- gate / output -----------------------------------------------------
    G = "/GATE"
    b.track(G, U1("3"), (117.8, 95.08), (117.8, 83), (150, 83), layer="B.Cu")
    b.via(G, (150, 83))
    b.track(G, (150, 83), (150, 86), R3("1"))
    b.track(G, (142, 83), (142, 106), layer="B.Cu")
    b.via(G, (142, 106))
    b.track(G, (142, 106), TP1("1"))

    QG = "Net-(Q1-G)"
    b.track(QG, R3("2"), (157, 86), (157, 80.94), Q1("1"))
    b.track(QG, R4("2"), (154, 70), (157, 70), (157, 80.94))

    S = "/SW_LED"
    b.track(S, Q1("3"), (162, 76), (168, 76), (168, 66), J2("1"))
    b.track(S, (168, 78), J3("1"))
    b.track(S, (168, 76), (168, 108), TP2("1"))
    b.track(S, (168, 90), J4("1"))
    b.track(S, (168, 102), J5("1"))

    # ---- GND ---------------------------------------------------------------
    # B.Cu carries the whole return. Only two signals use B.Cu - the gate
    # escape along y=83 (x 117.8..150) and its test-point stub at x=142 - so
    # every GND crossing of y=83 is placed outside that span.
    N = "GND"
    Bk = dict(layer="B.Cu", width=0.6)
    b.track(N, (102.5, 81), (155, 81), **Bk)
    b.track(N, (102.5, 81), (102.5, 65.46), J1("2"), **Bk)
    b.track(N, (113, 81), C1("2"), **Bk)
    b.track(N, (146, 81), C3("2"), **Bk)
    b.track(N, (120, 81), (120, 79), **Bk)
    b.track(N, (146, 81), (146, 73), **Bk)
    b.track(N, (124, 102.95), (124, 101), **Bk)
    b.track(N, (129, 81), (129, 81.94), **Bk)
    b.track(N, (112, 81), (112, 101), (137, 101), (137, 108), C6("2"), **Bk)
    b.track(N, (112, 90), U1("1"), **Bk)
    b.track(N, (155, 81), (155, 111), **Bk)
    b.track(N, (155, 111), (158, 111), **Bk)
    b.track(N, (155, 81), (176, 81), **Bk)
    b.track(N, (176, 63.5), (176, 99.5), **Bk)
    for j in (J2, J3, J4, J5):
        b.track(N, (176, j("2")[1]), j("2"), **Bk)

    for pad, v in ((C2("2"), (120, 79)), (C4("2"), (146, 73)),
                   (C5("1"), (124, 102.95)), (D2("1"), (129, 81.94)),
                   (TP3("1"), (158, 111))):
        b.track(N, pad, v)
        b.via(N, v)

    # ---- board -------------------------------------------------------------
    b.outline(X0, Y0, X1, Y1)
    b.zone("GND", X0 + 0.3, Y0 + 0.3, X1 - 0.3, Y1 - 0.3, layers=("B.Cu",))

    b.text("WAND STROBE DRIVER  rev B  9V", (103, 112.5), size=1.4)

    b.write(OUT)
    print("wrote", OUT)


def drc():
    r = subprocess.run([KCLI, "pcb", "drc", "--severity-error", "--severity-warning",
                        "--exit-code-violations", "-o", "/tmp/pcb.rpt", OUT],
                       capture_output=True, text=True)
    txt = open("/tmp/pcb.rpt").read() if os.path.exists("/tmp/pcb.rpt") else r.stdout
    print(txt[-4000:] if len(txt) > 4000 else txt)


if __name__ == "__main__":
    build()
    drc()
