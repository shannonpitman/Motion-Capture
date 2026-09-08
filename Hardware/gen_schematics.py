#!/usr/bin/env python3
"""gen_schematics.py - build the two KiCad schematics for the strobed wand.

    WandStrobeDriver  one TLC555 + one P-FET, at the handle
    WandMarker        x4, one per marker (A, B, C, D), 4 IR LEDs + one resistor

Rev B. Rev A ran 12 V and put a constant-current sink on every marker board.
The sink cost 1.6 V of headroom, which is the only reason 12 V was needed, and
it bought marker matching worth about 5% - less than the measured run-to-run
repeatability of the characterisation itself. Deleted. The marker board is now
the board that was already characterised, plus a connector.

Run it and open the .kicad_pro files. Change a value here and re-run rather
than editing the .kicad_sch by hand, so the design notes and the schematic
cannot drift apart.

Shannon Pitman
"""

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from kisch import Sch  # noqa: E402

FP = {  # footprints; passives 0805, transistors SOT-23, timer socketed DIP-8
    "R":     "Resistor_SMD:R_0805_2012Metric",
    "Rsense": "Resistor_SMD:R_1206_3216Metric",
    "C":     "Capacitor_SMD:C_0805_2012Metric",
    "Cfilm": "Capacitor_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm",
    "Cel":   "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm",
    "Cel_s": "Capacitor_THT:CP_Radial_D5.0mm_P2.00mm",
    "SOT23": "Package_TO_SOT_SMD:SOT-23",
    "DIP8":  "Package_DIP:DIP-8_W7.62mm_Socket",
    "SMA":   "Diode_SMD:D_SMA",
    "LEDv":  "LED_SMD:LED_0805_2012Metric",
    "pot":   "Potentiometer_THT:Potentiometer_Bourns_3296W_Vertical",
    "sw":    "Button_Switch_THT:SW_Slide_SPDT_Angled_CK_OS102011MA1Q",
    "conn2": "Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical",
    "screw2": "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MPT-0,5-2-2.54_1x02_P2.54mm_Horizontal",
    "tp":    "TestPoint:TestPoint_Pad_D1.5mm",
    "fuse":  "Fuse:Fuse_Bourns_MF-RG300",
    "Rt":    "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal",
}


# ===========================================================================
#  Board 1 - driver
# ===========================================================================
def driver(outdir):
    s = Sch("WandStrobeDriver",
            "Wand strobe driver - TLC555 + high-side P-FET, 9 V",
            rev="B", company="UCT MSc - Shannon Pitman", paper="A3")

    LEFT = dict(ref_off=(-2.54, -1.27), val_off=(-2.54, 1.27), just="right")
    TPT = dict(ref_off=(2.54, -2.54), val_off=(2.54, 0), just="left")
    RAIL = 38.1

    # ---- input protection --------------------------------------------------
    j1 = s.sym("Connector_Generic:Conn_01x02", "J1", "9V IN", (30.48, RAIL),
               rot=180, mirror="x", footprint=FP["screw2"],
               ref_off=(0, -10.16), val_off=(0, -7.62), just=None)
    sw1 = s.sym("Switch:SW_SPDT", "SW1", "POWER", (48.26, RAIL), footprint=FP["sw"],
                ref_off=(0, -7.62), val_off=(0, 6.35), just=None)
    f1 = s.sym("Device:Polyfuse", "F1", "500mA", (63.5, RAIL), rot=90,
               footprint=FP["fuse"])
    d1 = s.sym("Device:D_Schottky", "D1", "SS34", (80.01, RAIL), rot=180,
               footprint=FP["SMA"])
    s.wire(j1("1"), sw1("2"))                 # 2 = common
    s.wire(sw1("1"), (59.69, 35.56), f1("1"))  # 1 = the throw we use
    s.no_connect(sw1("3"))                     # 3 = unused throw
    s.wire(f1("2"), d1("2"))

    s.wire(j1("2"), (35.56, 58.42))
    s.power("GND", (35.56, 58.42))
    s.wire((35.56, 53.34), (22.86, 53.34))
    s.sym("power:PWR_FLAG", "#FLG02", "PWR_FLAG", (22.86, 53.34), hide_ref=True,
          val_off=(0, -6.35), just=None)
    s.junction((35.56, 53.34))

    # ---- +9 V rail ---------------------------------------------------------
    s.wire(d1("1"), (212.09, RAIL))
    c1 = s.sym("Device:C_Polarized", "C1", "1000u/16V", (106.68, 50.8),
               footprint=FP["Cel"])
    c2 = s.sym("Device:C", "C2", "100n", (129.54, 50.8), footprint=FP["C"])
    r5 = s.sym("Device:R", "R5", "22k", (152.4, 50.8), footprint=FP["R"])
    d2 = s.sym("Device:LED", "D2", "PWR green", (152.4, 64.77), rot=90,
               footprint=FP["LEDv"], ref_off=(11.43, -1.27),
               val_off=(11.43, 1.27), just=None)
    for x, top, bot in ((106.68, c1("1"), c1("2")), (129.54, c2("1"), c2("2"))):
        s.wire((x, RAIL), top)
        s.wire(bot, (x, 60.96))
        s.power("GND", (x, 60.96))
    s.wire((152.4, RAIL), r5("1"))
    s.wire(r5("2"), d2("2"))
    s.wire(d2("1"), (152.4, 74.93))
    s.power("GND", (152.4, 74.93))

    s.wire((175.26, RAIL), (175.26, 30.48))
    s.sym("power:PWR_FLAG", "#FLG01", "PWR_FLAG", (175.26, 30.48), hide_ref=True,
          val_off=(0, -3.81), just=None)
    s.wire((190.5, RAIL), (190.5, 30.48))
    s.power("+9V", (190.5, 30.48))
    s.junction((106.68, RAIL), (129.54, RAIL), (152.4, RAIL),
               (175.26, RAIL), (190.5, RAIL))

    # ---- filtered timer rail ----------------------------------------------
    r10 = s.sym("Device:R", "R10", "100", (215.9, RAIL), rot=90, footprint=FP["R"])
    s.wire(r10("2"), (295.28, RAIL))
    c3 = s.sym("Device:C_Polarized", "C3", "10u", (241.3, 50.8), footprint=FP["Cel_s"])
    c4 = s.sym("Device:C", "C4", "100n", (264.16, 50.8), footprint=FP["C"])
    for x, top, bot in ((241.3, c3("1"), c3("2")), (264.16, c4("1"), c4("2"))):
        s.wire((x, RAIL), top)
        s.wire(bot, (x, 60.96))
        s.power("GND", (x, 60.96))
    s.wire((275.59, RAIL), (275.59, 30.48))
    s.sym("power:PWR_FLAG", "#FLG03", "PWR_FLAG", (275.59, 30.48), hide_ref=True,
          val_off=(0, -3.81), just=None)
    s.wire((295.28, RAIL), (295.28, 30.48))
    s.power("VCC", (295.28, 30.48))
    s.junction((241.3, RAIL), (264.16, RAIL), (275.59, RAIL))

    # ---- timing chain: three fixed parts, nothing to turn ------------------
    XT = 60.96
    r1 = s.sym("Device:R", "R1", "1M", (XT, 95.25), footprint=FP["Rt"], **LEFT)
    r2 = s.sym("Device:R", "R2", "3k3", (XT, 113.03), footprint=FP["Rt"], **LEFT)
    c6 = s.sym("Device:C", "C6", "470n film", (XT, 129.54), footprint=FP["Cfilm"],
               **LEFT)

    s.wire(r1("1"), (XT, 85.09))
    s.power("VCC", (XT, 85.09))
    s.wire(r1("2"), r2("1"))
    s.wire(r2("2"), c6("1"))
    s.wire(c6("2"), (XT, 140.97))
    s.power("GND", (XT, 140.97))

    # ---- TLC555 ------------------------------------------------------------
    u1 = s.sym("Timer:TLC555xP", "U1", "TLC555CP", (109.22, 121.92),
               footprint=FP["DIP8"],
               datasheet="https://www.ti.com/lit/ds/symlink/tlc555.pdf",
               ref_off=(12.7, -8.89), val_off=(12.7, -6.35), just="left")

    s.wire((XT, 104.14), (82.55, 104.14), (82.55, 119.38), u1("7"))
    s.junction((XT, 104.14))
    s.wire((XT, 121.92), (88.9, 121.92), (88.9, 127), u1("2"))
    s.wire((88.9, 124.46), u1("6"))
    s.junction((XT, 121.92), (88.9, 124.46))
    s.wire(u1("4"), (93.98, 116.84), (93.98, 105.41))
    s.power("VCC", (93.98, 105.41))

    c5 = s.sym("Device:C", "C5", "10n", (104.14, 96.52), footprint=FP["C"],
               ref_off=(-2.54, -1.27), val_off=(-2.54, 1.27), just="right")
    s.wire(u1("5"), (109.22, 100.33), c5("2"))
    s.wire(c5("1"), (104.14, 85.09))
    s.power("GND", (104.14, 85.09))

    s.wire(u1("8"), (111.76, 99.06))
    s.power("VCC", (111.76, 99.06))
    s.wire(u1("1"), (109.22, 138.43))
    s.power("GND", (109.22, 138.43))

    s.wire(u1("3"), (142.24, 121.92))
    s.label("GATE", (142.24, 121.92))
    s.wire((133.35, 121.92), (133.35, 114.3))
    s.sym("Connector:TestPoint", "TP1", "GATE", (133.35, 114.3),
          footprint=FP["tp"], **TPT)
    s.junction((133.35, 121.92))

    # ---- high-side switch --------------------------------------------------
    r3 = s.sym("Device:R", "R3", "220", (186.69, 121.92), rot=90, footprint=FP["R"])
    q1 = s.sym("Transistor_FET:Q_PMOS_GSD", "Q1", "DMP3098L-7", (201.93, 121.92),
               mirror="x", footprint=FP["SOT23"],
               datasheet="https://www.diodes.com/assets/Datasheets/DMP3098L.pdf",
               ref_off=(7.62, -5.08), val_off=(7.62, -2.54), just="left")
    r4 = s.sym("Device:R", "R4", "100k", (196.85, 106.68), footprint=FP["R"], **LEFT)

    s.label("GATE", (176.53, 121.92))
    s.wire((176.53, 121.92), r3("1"))
    s.wire(r3("2"), q1("1"))
    s.wire(r4("2"), q1("1"))
    s.junction(q1("1"))
    s.wire(r4("1"), (196.85, 99.06))
    s.power("+9V", (196.85, 99.06))
    s.wire(q1("2"), (204.47, 109.22))
    s.power("+9V", (204.47, 109.22))
    s.wire(q1("3"), (204.47, 135.89), (219.71, 135.89))
    s.label("SW_LED", (219.71, 135.89))
    s.wire((212.09, 135.89), (212.09, 143.51))
    s.sym("Connector:TestPoint", "TP2", "SW_LED", (212.09, 143.51),
          footprint=FP["tp"], **TPT)
    s.junction((212.09, 135.89))
    s.sym("Connector:TestPoint", "TP3", "GND", (243.84, 140.97),
          footprint=FP["tp"], **TPT)
    s.wire((243.84, 140.97), (243.84, 147.32))
    s.power("GND", (243.84, 147.32))

    # ---- marker outputs ----------------------------------------------------
    for ref, tag, y in [("J2", "MARKER A (origin)", 100.33),
                        ("J3", "MARKER B", 118.11),
                        ("J4", "MARKER C (+X)", 135.89),
                        ("J5", "MARKER D (+Y)", 153.67)]:
        j = s.sym("Connector_Generic:Conn_01x02", ref, tag, (302.26, y),
                  footprint=FP["conn2"], ref_off=(7.62, -2.54),
                  val_off=(7.62, 0), just="left")
        s.label("SW_LED", (283.21, y))
        s.wire((283.21, y), j("1"))
        s.wire(j("2"), (283.21, y + 2.54), (283.21, y + 7.62))
        s.power("GND", (283.21, y + 7.62))

    # ---- notes -------------------------------------------------------------
    s.text("STROBE DRIVER rev B - one timer, nothing to adjust", (30.48, 168), 2.0, True)
    s.text(
        "Regime B (long camera exposure, short flash). The flash - not the shutter -\n"
        "defines the instant, so every camera whose shutter is open records the same\n"
        "~1 ms. One timer for the whole wand, so \"dark\" always means occluded and\n"
        "never out of phase.\n"
        "\n"
        "NO DIODE ACROSS R2, AND NONE IS NEEDED. The usual trick for sub-50% duty\n"
        "assumes the LEDs are on when OUT is HIGH. They are not: Q1 is P-channel on\n"
        "the high side and conducts when OUT is LOW, so the SHORT state is already\n"
        "the one we want. A plain astable with R1 >> R2 gives 0.36% duty directly.\n"
        "Adding the diode would force R1 small, and R1 then sinks Vcc/R1 through\n"
        "DISCH for the whole 327 ms off-time - about 20 mA on a battery. As built,\n"
        "that current is 9 uA.\n"
        "\n"
        "High side also means every marker's return is permanently grounded, so a\n"
        "chafed wire against the frame is a non-event rather than a short.\n"
        "\n"
        "TIMING   t_on  = 0.693 * R2 * C6            1173 us\n"
        "         t_off = 0.693 * (R1+R2) * C6       327 ms      -> 3.05 Hz\n"
        "         1 M is legal ONLY because the TLC555 is CMOS (pA input current).\n"
        "         A bipolar NE555 would not hold this operating point.\n"
        "\n"
        "R2 SETS PULSE WIDTH, WHICH IS THE BRIGHTNESS CONTROL. Peak current is fixed\n"
        "by the ballast resistor on each marker board, so the light delivered into\n"
        "one exposure is I x t_on. Fit R2 THT and clip a 5k pot across its pads for\n"
        "bring-up, then solder the nearest E24 value. Rate barely moves with R2:\n"
        "         1k8  -> 586 us      3k3 -> 1075 us   (fitted, E12)\n"
        "         2k2  -> 717 us      3k9 -> 1270 us\n"
        "         2k7  -> 879 us      4k7 -> 1531 us\n"
        "         Every resistor on this board is an E12 value. The exact number does\n"
        "         not matter - the 470n film cap is +/-5% on its own, which is +/-54 us,\n"
        "         and R2 is set empirically against a camera reading anyway.\n"
        "\n"
        "SUPPLY   9 V. 6xAA alkaline, or a PP3 for bench work. The 12 V of rev A was\n"
        "         only ever needed by the constant-current sinks, which are gone.\n"
        "         Total average draw ~2.2 mA, dominated by the TLC555, not the LEDs.\n"
        "\n"
        "R10/C3/C4 keep the 555 off the LED rail so the switching edge cannot\n"
        "modulate its Vcc-referenced thresholds. Two parts against a fault that is\n"
        "very hard to diagnose later - worth keeping even in a minimal build.",
        (30.48, 174), 1.4)
    s.text(
        "Q1 sees Vgs = -9 V, so a +/-12 V part (AO3401A) is now legal too.\n"
        "DMP3098L-7 is +/-20 V and costs the same. Do not fit a +/-8 V part.",
        (176.53, 158), 1.4)

    _write(s, outdir, "WandStrobeDriver")


# ===========================================================================
#  Board 2 - marker (x4)
# ===========================================================================
def marker(outdir):
    s = Sch("WandMarker",
            "Wand marker - 4x Kingbright L-7113SF7C + one ballast resistor",
            rev="B", company="UCT MSc - Shannon Pitman", paper="A3")

    TPT = dict(ref_off=(2.54, -2.54), val_off=(2.54, 0), just="left")
    RAIL = 49.53
    XL = 76.2

    j1 = s.sym("Connector_Generic:Conn_01x02", "J1", "FROM DRIVER", (30.48, RAIL),
               rot=180, mirror="x", footprint=FP["conn2"],
               ref_off=(0, -10.16), val_off=(0, -7.62), just=None)
    s.wire(j1("1"), (XL, RAIL))
    s.label("SW_LED", (60.96, RAIL))
    s.wire(j1("2"), (35.56, 68.58))
    s.power("GND", (35.56, 68.58))
    s.wire((35.56, 63.5), (22.86, 63.5))
    s.sym("power:PWR_FLAG", "#FLG01", "PWR_FLAG", (22.86, 63.5), hide_ref=True,
          val_off=(0, -6.35), just=None)
    s.junction((35.56, 63.5))

    c1 = s.sym("Device:C", "C1", "100n", (48.26, 60.96), footprint=FP["C"])
    s.wire((48.26, RAIL), c1("1"))
    s.wire(c1("2"), (48.26, 71.12))
    s.power("GND", (48.26, 71.12))
    s.junction((48.26, RAIL))

    leds = []
    for i, y in enumerate((58.42, 71.12, 83.82, 96.52)):
        leds.append(s.sym("Device:LED", "D%d" % (i + 1), "L-7113SF7C", (XL, y),
                          rot=90, footprint="LED_THT:LED_D5.0mm",
                          ref_off=(12.7, -1.27), val_off=(12.7, 1.27),
                          just=None, dnp=True,
                          datasheet="https://docs.rs-online.com/36ff/0900766b8139a40d.pdf",
                          fields={"MPN": "Kingbright L-7113SF7C"}))
    s.wire((XL, RAIL), leds[0]("2"))
    for a, b in zip(leds, leds[1:]):
        s.wire(a("1"), b("2"))

    r1 = s.sym("Device:R", "R1", "68R", (XL, 109.22), footprint=FP["Rt"])
    s.wire(leds[3]("1"), r1("1"))
    s.wire(r1("2"), (XL, 119.38))
    s.power("GND", (XL, 119.38))

    s.wire((XL, 102.87), (88.9, 102.87))
    s.sym("Connector:TestPoint", "TP1", "I_SENSE", (88.9, 102.87),
          footprint=FP["tp"], **TPT)
    s.junction((XL, 102.87))
    s.sym("Connector:TestPoint", "TP2", "GND", (99.06, 111.76),
          footprint=FP["tp"], **TPT)
    s.wire((99.06, 111.76), (99.06, 119.38))
    s.power("GND", (99.06, 119.38))

    s.text("MARKER BOARD rev B - build 4 (A, B, C, D), all identical",
           (30.48, 140), 2.0, True)
    s.text(
        "Eight parts. This is the board that was already characterised, plus a\n"
        "connector - the constant-current sink of rev A is deleted. It bought about\n"
        "5% marker-to-marker matching, which is smaller than the 3% run-to-run\n"
        "repeatability measured on the bench, and it cost 1.6 V of headroom, which\n"
        "was the entire reason rev A needed 12 V.\n"
        "\n"
        "EMITTER  Kingbright L-7113SF7C, 5 mm THT, GaAlAs, 850 nm (DSAB9772 V.16B).\n"
        "   Vf          1.4 typ / 1.6 max  AT If = 20 mA ONLY. Vf rises with current;\n"
        "               ~1.5 V each at 36 mA, so the string is ~5.9 V not 6.4 V.\n"
        "   If (DC)     50 mA absolute max\n"
        "   iFS (peak)  1 A, but ONLY at 1/100 duty and 10 us pulse width\n"
        "   Pd          80 mW.  At 36 mA / 0.36% duty each LED averages 0.19 mW.\n"
        "   2 theta 1/2 20 deg - narrow. This is why the marker rings, and why\n"
        "               tangential aiming matters more than shell diameter.\n"
        "\n"
        "R1 SETS THE CURRENT. Same value on all four boards. Computed at 9 V less\n"
        "the driver's Schottky and bulk-cap droop, with the real Vf curve:\n"
        "         33R -> 61 mA      68R -> 36 mA   (fitted, matches the bench build)\n"
        "         47R -> 48 mA     100R -> 26 mA\n"
        "Measure across TP1-TP2 with a scope during a pulse: I = V(TP1) / R1.\n"
        "\n"
        "SUPPLY SENSITIVITY is the price of dropping the CC sink: 0.3 V of battery\n"
        "sag moves the current 9%. Peak pixel moves with it, so re-target the\n"
        "exposure occasionally. It is a slow drift, not noise, and the bench data\n"
        "shows sigma changing only ~3% over a 20% exposure range - so it costs\n"
        "well under 1% of centroid accuracy. That is the trade, stated plainly.\n"
        "\n"
        "CHECK THE WORKBOOK NUMBER. LED_characterisation.xlsx lists 38.2 mA for this\n"
        "build, computed from an assumed Vf of 1.6 V - which is the datasheet MAX at\n"
        "20 mA, not the value at 38 mA. With the real curve, 9 V through 68R gives\n"
        "about 43 mA. Worth measuring once and correcting in the sheet.\n"
        "\n"
        "D1..D4 are DNP so an assembler skips them - they are hand-soldered so the\n"
        "standoff to the diffuser can be set by hand. The pads are still required.\n"
        "Space them as widely as the shell allows and aim them TANGENTIALLY.",
        (30.48, 146), 1.4)

    _write(s, outdir, "WandMarker")


def _write(s, outdir, name):
    d = os.path.join(outdir, name)
    os.makedirs(d, exist_ok=True)
    s.write(os.path.join(d, name + ".kicad_sch"))
    pro = os.path.join(d, name + ".kicad_pro")
    if not os.path.exists(pro):
        with open(pro, "w") as f:
            json.dump({"board": {}, "boards": [], "cvpcb": {"equivalence_files": []},
                       "libraries": {"pinned_footprint_libs": [],
                                     "pinned_symbol_libs": []},
                       "meta": {"filename": name + ".kicad_pro", "version": 3},
                       "net_settings": {}, "pcbnew": {},
                       "schematic": {}, "sheets": [[s.root, "Root"]],
                       "text_variables": {}}, f, indent=2)
    print("wrote", os.path.join(d, name + ".kicad_sch"))


if __name__ == "__main__":
    out = HERE
    driver(out)
    marker(out)
