"""kisch.py - the smallest schematic writer that produces a file KiCad will
open, edit and net-list correctly.

Deliberately not a general KiCad library. It knows how to place a symbol, run
an orthogonal wire, drop a junction, a net label, a power symbol and a text
block - which is all a two-sheet analog design needs. Everything else (fields,
sim models, buses) is left to KiCad once the file is open.

Pin coordinates come from kisym, so wire endpoints are derived from the real
symbol geometry rather than typed in by hand.
"""

import hashlib
import kisym

SCH_VERSION = "20250114"   # KiCad 9 format; KiCad 10 reads and upgrades it
GRID = 1.27                # KiCad's default 50 mil connection grid


def snap(p):
    """Round a point onto the connection grid.

    Every stock symbol places its pins on multiples of 1.27 mm, so snapping the
    placement origin puts the pins on grid too. Snapping is a pure function of
    the coordinate, so two points that shared an x or a y before still share it
    after - orthogonal wires stay orthogonal.
    """
    return (round(p[0] / GRID) * GRID, round(p[1] / GRID) * GRID)


def _uuid(seed):
    h = hashlib.md5(seed.encode()).hexdigest()
    return "%s-%s-%s-%s-%s" % (h[0:8], h[8:12], h[12:16], h[16:20], h[20:32])


def _n(v):
    return ("%.4f" % v).rstrip("0").rstrip(".") or "0"


class Sch:
    def __init__(self, project, title, rev="A", company="", paper="A3"):
        self.project = project
        self.paper = paper
        self.title = title
        self.rev = rev
        self.company = company
        self.root = _uuid(project + "root")
        self.libs = {}       # lib_id -> raw block
        self.items = []      # rendered s-expressions
        self._k = 0
        self._pwr = 0

    def _uid(self, tag):
        self._k += 1
        return _uuid("%s:%s:%d" % (self.project, tag, self._k))

    # ---- placement ---------------------------------------------------------
    def sym(self, lib_id, ref, value, at, rot=0, mirror=None,
            footprint="", datasheet="~", dnp=False, fields=None,
            ref_off=None, val_off=None, just=None, hide_ref=False):
        """Place one symbol. Returns a callable p(pin_number) -> (x, y) so
        wires can be attached without repeating the geometry.

        Reference and Value are placed automatically for two-pin parts: beside
        a symbol that ends up drawn vertically, above and below one drawn
        horizontally. Pass ref_off/val_off/just to override.
        """
        if lib_id not in self.libs:
            self.libs[lib_id] = kisym.raw_symbol_flat(lib_id)
        at = snap(at)
        x, y = at
        u = self._uid("sym" + ref)

        def pin(n):
            return snap(kisym.pin_xy(lib_id, n, at, rot, mirror))

        # KiCad adds the symbol's rotation to the field angle, so a field on a
        # 90-degree symbol has to be counter-rotated to stay readable.
        pang = 90 if rot in (90, 270) else 0

        if ref_off is None or val_off is None:
            pn = sorted(kisym.pins_flat(lib_id))
            vertical = True
            if len(pn) == 2:
                a, b = pin(pn[0]), pin(pn[1])
                vertical = abs(b[1] - a[1]) >= abs(b[0] - a[0])
            if vertical:
                d_ref, d_val, d_just = (2.54, -1.27), (2.54, 1.27), "left"
            else:
                d_ref, d_val, d_just = (0.0, -3.81), (0.0, 3.81), None
            ref_off = ref_off if ref_off is not None else d_ref
            val_off = val_off if val_off is not None else d_val
            just = just if just is not None else d_just

        jtok = "\n\t\t\t\t(justify %s)" % just if just else ""

        props = [("Reference", ref, ref_off, hide_ref),
                 ("Value", value, val_off, False),
                 ("Footprint", footprint, (0.0, 0.0), True),
                 ("Datasheet", datasheet, (0.0, 0.0), True),
                 ("Description", "", (0.0, 0.0), True)]
        for k, v in (fields or {}).items():
            props.append((k, v, (0.0, 0.0), True))

        pblocks = []
        for name, val, off, hide in props:
            px, py = snap((x + off[0], y + off[1]))
            pblocks.append(
                '\t\t(property "%s" "%s"\n'
                '\t\t\t(at %s %s %d)\n'
                '\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t)'
                '%s%s\n\t\t\t)\n\t\t)'
                % (name, val, _n(px), _n(py), pang, jtok,
                   "\n\t\t\t\t(hide yes)" if hide else ""))

        pinblocks = ['\t\t(pin "%s"\n\t\t\t(uuid "%s")\n\t\t)'
                     % (pnum, _uuid(u + "pin" + pnum))
                     for pnum in sorted(kisym.pins_flat(lib_id))]

        mir = "\n\t\t(mirror %s)" % mirror if mirror else ""
        self.items.append(
            '\t(symbol\n\t\t(lib_id "%s")\n\t\t(at %s %s %d)%s\n\t\t(unit 1)\n'
            '\t\t(exclude_from_sim no)\n\t\t(in_bom yes)\n\t\t(on_board yes)\n'
            '\t\t(dnp %s)\n\t\t(uuid "%s")\n%s\n%s\n'
            '\t\t(instances\n\t\t\t(project "%s"\n\t\t\t\t(path "/%s"\n'
            '\t\t\t\t\t(reference "%s")\n\t\t\t\t\t(unit 1)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n\t)'
            % (lib_id, _n(x), _n(y), rot, mir, "yes" if dnp else "no", u,
               "\n".join(pblocks), "\n".join(pinblocks),
               self.project, self.root, ref))

        return pin

    def power(self, kind, at, rot=0):
        """A power port (power:GND, power:+12V, ...). Its pin sits on `at`."""
        lib_id = "power:" + kind
        if lib_id not in self.libs:
            self.libs[lib_id] = kisym.raw_symbol_flat(lib_id)
        x, y = snap(at)
        u = self._uid("pwr" + kind)
        self._pwr += 1
        ref = "#PWR%02d" % self._pwr
        off = 6.35 if kind == "GND" else -6.35
        self.items.append(
            '\t(symbol\n\t\t(lib_id "%s")\n\t\t(at %s %s %d)\n\t\t(unit 1)\n'
            '\t\t(exclude_from_sim no)\n\t\t(in_bom yes)\n\t\t(on_board yes)\n'
            '\t\t(dnp no)\n\t\t(uuid "%s")\n'
            '\t\t(property "Reference" "%s"\n\t\t\t(at %s %s 0)\n'
            '\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t)\n'
            '\t\t\t\t(hide yes)\n\t\t\t)\n\t\t)\n'
            '\t\t(property "Value" "%s"\n\t\t\t(at %s %s 0)\n'
            '\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n'
            '\t\t(pin "1"\n\t\t\t(uuid "%s")\n\t\t)\n'
            '\t\t(instances\n\t\t\t(project "%s"\n\t\t\t\t(path "/%s"\n'
            '\t\t\t\t\t(reference "%s")\n\t\t\t\t\t(unit 1)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n\t)'
            % (lib_id, _n(x), _n(y), rot, u, ref, _n(x), _n(y),
               kind, _n(x), _n(y + off),
               _uuid(u + "p1"), self.project, self.root, ref))

    # ---- connectivity ------------------------------------------------------
    def wire(self, *pts):
        """Orthogonal polyline. Consecutive points must share x or y."""
        pts = [snap(p) for p in pts]
        for a, b in zip(pts, pts[1:]):
            if abs(a[0] - b[0]) > 1e-6 and abs(a[1] - b[1]) > 1e-6:
                raise ValueError("wire segment %s-%s is not orthogonal" % (a, b))
            if a == b:
                continue
            self.items.append(
                '\t(wire\n\t\t(pts\n\t\t\t(xy %s %s) (xy %s %s)\n\t\t)\n'
                '\t\t(stroke\n\t\t\t(width 0)\n\t\t\t(type default)\n\t\t)\n'
                '\t\t(uuid "%s")\n\t)'
                % (_n(a[0]), _n(a[1]), _n(b[0]), _n(b[1]), self._uid("w")))

    def junction(self, *pts):
        for p in (snap(q) for q in pts):
            self.items.append(
                '\t(junction\n\t\t(at %s %s)\n\t\t(diameter 0)\n'
                '\t\t(color 0 0 0 0)\n\t\t(uuid "%s")\n\t)'
                % (_n(p[0]), _n(p[1]), self._uid("j")))

    def label(self, name, at, rot=0, just="left bottom"):
        at = snap(at)
        self.items.append(
            '\t(label "%s"\n\t\t(at %s %s %d)\n'
            '\t\t(effects\n\t\t\t(font\n\t\t\t\t(size 1.27 1.27)\n\t\t\t)\n'
            '\t\t\t(justify %s)\n\t\t)\n\t\t(uuid "%s")\n\t)'
            % (name, _n(at[0]), _n(at[1]), rot, just, self._uid("lbl")))

    def text(self, body, at, size=1.27, bold=False):
        self.items.append(
            '\t(text "%s"\n\t\t(exclude_from_sim no)\n\t\t(at %s %s 0)\n'
            '\t\t(effects\n\t\t\t(font\n\t\t\t\t(size %s %s)%s\n\t\t\t)\n'
            '\t\t\t(justify left top)\n\t\t)\n\t\t(uuid "%s")\n\t)'
            % (body.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n"),
               _n(at[0]), _n(at[1]),
               _n(size), _n(size), "\n\t\t\t\t(bold yes)" if bold else "",
               self._uid("txt")))

    # ---- output ------------------------------------------------------------
    def render(self):
        libs = "\n".join(self.libs[k] for k in sorted(self.libs))
        return (
            "(kicad_sch\n\t(version %s)\n\t(generator \"eeschema\")\n"
            "\t(generator_version \"9.0\")\n\t(uuid \"%s\")\n\t(paper \"%s\")\n"
            "\t(title_block\n\t\t(title \"%s\")\n\t\t(rev \"%s\")\n\t\t(company \"%s\")\n\t)\n"
            "\t(lib_symbols\n%s\n\t)\n%s\n"
            "\t(sheet_instances\n\t\t(path \"/\"\n\t\t\t(page \"1\")\n\t\t)\n\t)\n"
            "\t(embedded_fonts no)\n)\n"
            % (SCH_VERSION, self.root, self.paper, self.title, self.rev,
               self.company, libs, "\n".join(self.items)))

    def write(self, path):
        with open(path, "w") as f:
            f.write(self.render())
