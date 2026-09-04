#!/usr/bin/env python3
"""gen_schematics.py - build the two KiCad schematics for the strobed wand.

    WandStrobeDriver  one TLC555 + one P-FET, at the handle
    WandMarker        x4, one per marker (A, B, C, D), 4 IR LEDs + a CC sink

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
    "sw":    "Button_Switch_THT:SW_CK_JS202011AQN_DPDT_Angled",
    "conn2": "Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical",
    "screw2": "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MPT-0,5-2-2.54_1x02_P2.54mm_Horizontal",
    "tp":    "TestPoint:TestPoint_Pad_D1.5mm",
    "fuse":  "Fuse:Fuse_Bourns_MF-RG300",
}


# ===========================================================================
#  Board 1 - driver
# ===========================================================================
def driver(outdir):
    s = Sch("WandStrobeDriver",
            "Wand strobe driver - TLC555 + high-side P-FET, 12 V",
            rev="A", company="UCT MSc - Shannon Pitman", paper="A3")

    LEFT = dict(ref_off=(-2.54, -1.27), val_off=(-2.54, 1.27), just="right")
    TPT = dict(ref_off=(2.54, -2.54), val_off=(2.54, 0), just="left")
    RAIL = 38.1

    # ---- input protection --------------------------------------------------
    j1 = s.sym("Connector_Generic:Conn_01x02", "J1", "12V IN", (30.48, RAIL),
               rot=180, mirror="x", footprint=FP["screw2"],
               ref_off=(0, -10.16), val_off=(0, -7.62), just=None)
    sw1 = s.sym("Switch:SW_SPST", "SW1", "POWER", (48.26, RAIL), footprint=FP["sw"])
    f1 = s.sym("Device:Polyfuse", "F1", "1A", (63.5, RAIL), rot=90, footprint=FP["fuse"])
    d1 = s.sym("Device:D_Schottky", "D1", "SS34", (80.01, RAIL), rot=180,
               footprint=FP["SMA"])
    s.wire(j1("1"), sw1("1"))
    s.wire(sw1("2"), f1("1"))
    s.wire(f1("2"), d1("2"))

    s.wire(j1("2"), (35.56, 58.42))
    s.power("GND", (35.56, 58.42))
    s.wire((35.56, 53.34), (22.86, 53.34))
    s.sym("power:PWR_FLAG", "#FLG02", "PWR_FLAG", (22.86, 53.34), hide_ref=True,
          val_off=(0, -6.35), just=None)
    s.junction((35.56, 53.34))

    # ---- +12 V rail --------------------------------------------------------
    s.wire(d1("1"), (212.09, RAIL))
    c1 = s.sym("Device:C_Polarized", "C1", "1000u/25V", (106.68, 50.8),
               footprint=FP["Cel"])
    c2 = s.sym("Device:C", "C2", "100n", (129.54, 50.8), footprint=FP["C"])
    r5 = s.sym("Device:R", "R5", "10k", (152.4, 50.8), footprint=FP["R"])
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
    s.power("+12V", (190.5, 30.48))
    s.junction((106.68, RAIL), (129.54, RAIL), (152.4, RAIL),
               (175.26, RAIL), (190.5, RAIL))

    # ---- filtered timer rail (VCC) ----------------------------------------
    r10 = s.sym("Device:R", "R10", "100", (215.9, RAIL), rot=90, footprint=FP["R"])
    s.wire(r10("2"), (304.8, RAIL))
    c3 = s.sym("Device:C_Polarized", "C3", "10u", (241.3, 50.8), footprint=FP["Cel_s"])
    c4 = s.sym("Device:C", "C4", "100n", (264.16, 50.8), footprint=FP["C"])
    for x, top, bot in ((241.3, c3("1"), c3("2")), (264.16, c4("1"), c4("2"))):
        s.wire((x, RAIL), top)
        s.wire(bot, (x, 60.96))
        s.power("GND", (x, 60.96))
    s.wire((287.02, RAIL), (287.02, 30.48))
    s.sym("power:PWR_FLAG", "#FLG03", "PWR_FLAG", (287.02, 30.48), hide_ref=True,
          val_off=(0, -3.81), just=None)
    s.wire((304.8, RAIL), (304.8, 30.48))
    s.power("VCC", (304.8, 30.48))
    s.junction((241.3, RAIL), (264.16, RAIL), (287.02, RAIL))

    # ---- timing chain ------------------------------------------------------
    XT = 60.96
    r1 = s.sym("Device:R", "R1", "470k", (XT, 95.25), footprint=FP["R"], **LEFT)
    rv1 = s.sym("Device:R_Potentiometer_Trim", "RV1", "1M  RATE", (XT, 106.68),
                footprint=FP["pot"], **LEFT)
    r2 = s.sym("Device:R", "R2", "470", (XT, 118.11), footprint=FP["R"], **LEFT)
    rv2 = s.sym("Device:R_Potentiometer_Trim", "RV2", "2k  WIDTH", (XT, 129.54),
                footprint=FP["pot"], **LEFT)
    c6 = s.sym("Device:C", "C6", "470n film", (XT, 140.97), footprint=FP["Cfilm"],
               **LEFT)

    s.wire(r1("1"), (XT, 85.09))
    s.power("VCC", (XT, 85.09))
    s.wire(r1("2"), rv1("1"))
    s.wire(rv1("3"), r2("1"))
    s.wire(rv1("2"), (68.58, 106.68), (68.58, 110.49), rv1("3"))
    s.wire(r2("2"), rv2("1"))
    s.wire(rv2("2"), (68.58, 129.54), (68.58, 133.35), rv2("3"))
    s.wire(rv2("3"), c6("1"))
    s.wire(c6("2"), (XT, 149.86))
    s.power("GND", (XT, 149.86))
    s.junction(rv1("3"), rv2("3"))

    # ---- TLC555 ------------------------------------------------------------
    u1 = s.sym("Timer:TLC555xP", "U1", "TLC555CP", (109.22, 121.92),
               footprint=FP["DIP8"],
               datasheet="https://www.ti.com/lit/ds/symlink/tlc555.pdf",
               ref_off=(12.7, -8.89), val_off=(12.7, -6.35), just="left")

    s.wire((XT, 111.76), (82.55, 111.76), (82.55, 119.38), u1("7"))
    s.junction((XT, 111.76))
    s.wire((XT, 135.89), (88.9, 135.89), (88.9, 124.46), u1("6"))
    s.wire((88.9, 127), u1("2"))
    s.junction((XT, 135.89), (88.9, 127))
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
    s.power("+12V", (196.85, 99.06))
    s.wire(q1("2"), (204.47, 109.22))
    s.power("+12V", (204.47, 109.22))
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
    s.text("STROBE DRIVER - one timer for the whole wand", (30.48, 168), 2.0, True)
    s.text(
        "Regime B (long camera exposure, short flash). The flash - not the shutter -\n"
        "defines the instant, so every camera whose shutter is open sees the same\n"
        "300 us. Duty ~0.09%, so pulsed overdrive is available.\n"
        "\n"
        "TIMING   plain astable, no diode. R1+RV1 >> R2+RV2 puts the SHORT state on\n"
        "         OUT LOW, and Q1 (P-channel, high side) conducts when OUT is low.\n"
        "         The diode-across-R2 trick is NOT used: it would burn Vcc/R1 through\n"
        "         DISCH for the whole 333 ms off-time (~20 mA) on a battery.\n"
        "           t_on  = 0.693 * (R2+RV2) * C6     153 us .. 804 us   (300 us nominal)\n"
        "           t_off = 0.693 * (R1+RV1+R2+RV2) * C6\n"
        "           f     = 6.5 Hz .. 2.1 Hz          3 Hz at RV1 mid-travel\n"
        "         1 M timing resistors are fine ONLY because the TLC555 is CMOS\n"
        "         (pA input current). A bipolar NE555 would not hold this.\n"
        "\n"
        "RV2 IS THE BRIGHTNESS KNOB. The marker boards are constant-current, so peak\n"
        "current is fixed and pulse WIDTH sets how much light lands in the exposure.\n"
        "One trimpot at the handle moves all four markers together and cannot\n"
        "unmatch them - unlike a per-marker current pot.\n"
        "  -> ExtrinsicsCalib/README.md step 2 says \"turn the LED trimpot\": that is RV2.\n"
        "\n"
        "R10/C3/C4 keep the 555 off the LED rail. The 600 mA switching edge would\n"
        "otherwise modulate its Vcc-referenced thresholds and add timing jitter.\n"
        "\n"
        "AVERAGE CURRENT   TLC555 ~1 mA, timing ~12 uA, D2 ~1.2 mA,\n"
        "                  LEDs 4 x 96 mA x 0.09% = 0.35 mA.  Total ~2.6 mA:\n"
        "                  the timer, not the LEDs, dominates. 8xAA lasts ~900 h.\n"
        "\n"
        "C1 SIZING   worst case 4 x 96 mA x 804 us = 309 uC; 1000 uF -> 0.31 V droop.\n"
        "            The CC sinks reject that entirely until they hit dropout.\n"
        "\n"
        "HEADROOM    the sinks drop out below about 9.3 V at the connector.\n"
        "            8xAA alkaline or 3S LiPo; do not substitute a 9 V PP3.",
        (30.48, 174), 1.4)
    s.text(
        "Q1 must be rated Vgs +/-20 V: the gate swings the full 12 V rail.\n"
        "Common SOT-23 P-FETs (AO3401A, Si2301) are only +/-8..12 V - do not fit one.",
        (176.53, 158), 1.4)

    _write(s, outdir, "WandStrobeDriver")


# ===========================================================================
#  Board 2 - marker (x4)
# ===========================================================================
def marker(outdir):
    s = Sch("WandMarker",
            "Wand marker - 4x 850 nm IR LED + constant-current sink",
            rev="A", company="UCT MSc - Shannon Pitman", paper="A3")

    TPT = dict(ref_off=(2.54, -2.54), val_off=(2.54, 0), just="left")
    LEFT = dict(ref_off=(-2.54, -1.27), val_off=(-2.54, 1.27), just="right")
    RAIL = 49.53
    XL = 96.52          # LED string

    j1 = s.sym("Connector_Generic:Conn_01x02", "J1", "FROM DRIVER", (30.48, RAIL),
               rot=180, mirror="x", footprint=FP["conn2"],
               ref_off=(0, -10.16), val_off=(0, -7.62), just=None)
    s.wire(j1("1"), (XL, RAIL))
    s.label("SW_LED", (78.74, RAIL))
    s.wire(j1("2"), (35.56, 68.58))
    s.power("GND", (35.56, 68.58))
    s.wire((35.56, 63.5), (22.86, 63.5))
    s.sym("power:PWR_FLAG", "#FLG01", "PWR_FLAG", (22.86, 63.5), hide_ref=True,
          val_off=(0, -6.35), just=None)
    s.junction((35.56, 63.5))

    c1 = s.sym("Device:C_Polarized", "C1", "10u", (48.26, 60.96), footprint=FP["Cel_s"])
    c2 = s.sym("Device:C", "C2", "100n", (63.5, 60.96), footprint=FP["C"])
    for x, top, bot in ((48.26, c1("1"), c1("2")), (63.5, c2("1"), c2("2"))):
        s.wire((x, RAIL), top)
        s.wire(bot, (x, 71.12))
        s.power("GND", (x, 71.12))
    s.junction((48.26, RAIL), (63.5, RAIL), (82.55, RAIL))

    leds = []
    for i, y in enumerate((58.42, 71.12, 83.82, 96.52)):
        leds.append(s.sym("Device:LED", "D%d" % (i + 1), "IR 850nm", (XL, y),
                          rot=90, footprint="", ref_off=(12.7, -1.27),
                          val_off=(12.7, 1.27), just=None, dnp=True,
                          fields={"MPN": "Kingbright 850 nm - fill in exact MPN"}))
    s.wire((XL, RAIL), leds[0]("2"))
    for a, b in zip(leds, leds[1:]):
        s.wire(a("1"), b("2"))

    q1 = s.sym("Transistor_BJT:Q_NPN_BEC", "Q1", "MMBT2222A", (XL, 113.03),
               footprint=FP["SOT23"], ref_off=(7.62, -2.54),
               val_off=(7.62, 0), just="left")
    q2 = s.sym("Transistor_BJT:Q_NPN_BEC", "Q2", "MMBT3904", (76.2, 113.03),
               mirror="y", footprint=FP["SOT23"], ref_off=(-17.78, -2.54),
               val_off=(-17.78, 0), just="left")
    r1 = s.sym("Device:R", "R1", "680", (82.55, 73.66), footprint=FP["R"], **LEFT)
    r2 = s.sym("Device:R", "R2", "6R8 1%", (99.06, 125.73), footprint=FP["Rsense"])
    r3 = s.sym("Device:R", "R3", "DNP (trim)", (129.54, 125.73),
               footprint=FP["Rsense"], dnp=True)

    s.wire(leds[3]("1"), (XL, 107.95), q1("3"))
    s.wire((82.55, RAIL), r1("1"))
    s.wire(r1("2"), (82.55, 105.41), (91.44, 105.41), q1("1"))
    s.wire(q2("3"), (73.66, 105.41), (82.55, 105.41))
    s.junction((82.55, 105.41))
    s.wire(q1("2"), r2("1"))
    s.wire((99.06, 120.65), (81.28, 120.65), q2("1"))
    s.wire((99.06, 120.65), (129.54, 120.65), r3("1"))
    s.junction((99.06, 120.65))
    s.label("I_SENSE", (86.36, 120.65))
    s.wire(r2("2"), (99.06, 135.89))
    s.power("GND", (99.06, 135.89))
    s.wire(q2("2"), (73.66, 135.89))
    s.power("GND", (73.66, 135.89))
    s.wire(r3("2"), (129.54, 135.89))
    s.power("GND", (129.54, 135.89))

    s.wire((114.3, 120.65), (114.3, 127))
    s.sym("Connector:TestPoint", "TP1", "I_SENSE", (114.3, 127),
          footprint=FP["tp"], **TPT)
    s.junction((114.3, 120.65))
    s.sym("Connector:TestPoint", "TP2", "GND", (147.32, 127),
          footprint=FP["tp"], **TPT)
    s.wire((147.32, 127), (147.32, 133.35))
    s.power("GND", (147.32, 133.35))

    # ---- notes -------------------------------------------------------------
    s.text("MARKER BOARD - build 4 (A, B, C, D), all identical", (30.48, 152), 2.0, True)
    s.text(
        "Q1/Q2/R2 is a constant-current sink: Q2 steals Q1's base drive as soon as\n"
        "R2 reaches a Vbe, so\n"
        "        I_LED = Vbe(Q2) / R2   ~= 0.65 / R2\n"
        "independent of the supply, of C1's droop during the pulse, and of Q1's beta.\n"
        "That is the point: all four markers land on the same current without\n"
        "matching LEDs, so ONE camera exposure targets every marker at once, and a\n"
        "sagging battery dims nothing until the sink drops out.\n"
        "\n"
        "R2 SELECTION      4R3 -> 151 mA     6R8 -> 96 mA (fitted)\n"
        "                  8R2 ->  79 mA     13R ->  50 mA\n"
        "R3 is a DNP footprint in parallel with R2 for fine trim upward.\n"
        "Measure across TP1-TP2 with a scope during a pulse: I = V / R2.\n"
        "\n"
        "96 mA is 2.5x the 38.2 mA DC point from LED_characterisation.xlsx, at 0.09%\n"
        "duty - far inside any datasheet pulse rating. Q1 dissipates ~0.33 W DURING\n"
        "the pulse and 0.3 mW average.\n"
        "\n"
        "HEADROOM BUDGET at 96 mA (this is what sets the 12 V rail):\n"
        "   4 x Vf(850nm @ 96 mA)  ~7.4      D1 Schottky on driver   0.45\n"
        "   R2 sense                0.65      Q1 Vce, min to regulate 0.50\n"
        "   Q1 Vds + wiring         0.06      C1 droop                0.31\n"
        "   ------------------------------------------------------------\n"
        "   minimum pack voltage   ~9.3 V     <- 8xAA or 3S LiPo, NOT a 9 V PP3\n"
        "\n"
        "D1..D4 FOOTPRINT IS STILL BLANK. DNP excludes them from assembly, NOT from\n"
        "the PCB - the pads must exist, so the package is needed before layout.\n"
        "LED_characterisation.xlsx ranks Kingbright 25 mm best (sigma 0.015 px at\n"
        "1186 us, vs XL's 6057 us) but records no MPN - fill in the MPN field.\n"
        "Its only failing was fill 0.82 against the 0.85 gate, which Test G\n"
        "(thicker wall) is meant to fix.\n"
        "\n"
        "LED RING: space D1..D4 as widely as the 20/25 mm shell allows and aim them\n"
        "TANGENTIALLY, not radially - NOTES.md attributes the ring artefact to beam\n"
        "angle, not diffuser diameter.\n"
        "\n"
        "TWO DIFFERENT MEANINGS OF DNP ON THIS SHEET - do not conflate them:\n"
        "  D1..D4  ARE FITTED. Marked DNP only so a turnkey assembler skips them;\n"
        "          the Kingbright emitters are hand-soldered after assembly so the\n"
        "          diffuser standoff can be set by hand. The board is useless\n"
        "          without them. The FOOTPRINT is still required for layout.\n"
        "  R3      IS NOT FITTED. Bare pads, an optional trim in parallel with R2.",
        (30.48, 158), 1.4)

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
