"""kipcb.py - minimal .kicad_pcb writer.

Same idea as kisch.py: place real footprints, take pad coordinates from the
real .kicad_mod files, and let the caller route with points derived from those
pads rather than typed in by hand. Nets come straight from the schematic
netlist, so the board cannot disagree with the schematic about connectivity.

Not a general PCB library. It knows footprints, tracks, vias, a board outline
and one copper zone, which is all a two-layer board of this size needs.
"""

import hashlib
import math
import os
import re

FPDIR = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints"
PCB_VERSION = "20241229"      # KiCad 9 format; KiCad 10 reads and upgrades it


def _uuid(seed):
    h = hashlib.md5(seed.encode()).hexdigest()
    return "%s-%s-%s-%s-%s" % (h[0:8], h[8:12], h[12:16], h[16:20], h[20:32])


def _n(v):
    return ("%.4f" % v).rstrip("0").rstrip(".") or "0"


_MODCACHE = {}


def _mod(fp):
    if fp not in _MODCACHE:
        lib, name = fp.split(":")
        with open(os.path.join(FPDIR, lib + ".pretty", name + ".kicad_mod")) as f:
            _MODCACHE[fp] = f.read()
    return _MODCACHE[fp]


def pads(fp):
    """{pad_number: (x, y)} in the footprint's own unrotated frame."""
    out = {}
    for m in re.finditer(r'\(pad "([^"]*)" \w+ \w+\s*\n\s*\(at ([\d.eE+-]+) ([\d.eE+-]+)', _mod(fp)):
        if m.group(1):
            out[m.group(1)] = (float(m.group(2)), float(m.group(3)))
    return out


def xy(fp, num, at, rot=0):
    """Pad `num` in board coordinates for a footprint placed at `at`/`rot`.

    Footprint and board both use +Y down, so unlike the schematic there is no
    flip - only KiCad's counter-clockwise rotation.
    """
    px, py = pads(fp)[str(num)]
    t = math.radians(rot)
    c, s = math.cos(t), math.sin(t)
    return (round(at[0] + px * c + py * s, 4), round(at[1] - px * s + py * c, 4))


class Pcb:
    def __init__(self, name, netlist=None, title=""):
        self.name = name
        self.title = title
        self.items = []
        self.nets = {"": 0}
        self.pin2net = {}          # (ref, pad) -> net name
        self._k = 0
        if netlist:
            self.load_netlist(netlist)

    def _uid(self, tag):
        self._k += 1
        return _uuid("%s:%s:%d" % (self.name, tag, self._k))

    # ---- nets --------------------------------------------------------------
    def load_netlist(self, path):
        txt = open(path).read()
        blk = txt[txt.index("\t(nets"):]
        for m in re.finditer(r'\(net\n\t+\(code "\d+"\)\n\t+\(name "([^"]*)"\)(.*?)\n\t\t\)\n',
                             blk, re.S):
            net = m.group(1)
            self.net(net)
            for ref, pad in re.findall(r'\(ref "([^"]+)"\)\n\t+\(pin "([^"]+)"\)', m.group(2)):
                self.pin2net[(ref, pad)] = net

    def net(self, name):
        if name not in self.nets:
            self.nets[name] = len(self.nets)
        return self.nets[name]

    # ---- placement ---------------------------------------------------------
    def place(self, fp, ref, value, at, rot=0, back=False, ref_at=(0, -3.7)):
        body = _mod(fp)
        i = body.index("\n", body.index('(footprint "'))
        body = body[i + 1:body.rindex(")")]
        # drop the file-level header lines the board does not want
        body = re.sub(r'^\t\((?:version|generator|generator_version|layer)[^\n]*\n',
                      '', body, flags=re.M)
        body = re.sub(r'^\t\(property "(?:Reference|Value)"[\s\S]*?\n\t\)\n', '', body, flags=re.M)

        # fresh uuids so repeated footprints stay unique
        seen = {}
        def fresh(m):
            seen.setdefault(m.group(1), len(seen))
            return '(uuid "%s")' % _uuid("%s%s%d" % (ref, m.group(1), seen[m.group(1)]))
        body = re.sub(r'\(uuid "([^"]*)"\)', fresh, body)

        # pads: add the footprint rotation to the pad angle, and assign the net
        def padfix(m):
            num, head, ax, ay, aa = m.group(1), m.group(0), m.group(2), m.group(3), m.group(4)
            ang = (float(aa or 0) + rot) % 360
            at_new = '(at %s %s%s)' % (ax, ay, '' if ang == 0 else ' ' + _n(ang))
            net = self.pin2net.get((ref, num))
            nt = '\n\t\t\t(net %d "%s")' % (self.net(net), net) if net else ''
            return head[:head.index('(at ')] + at_new + nt
        body = re.sub(r'\(pad "([^"]*)" \w+ \w+\s*\n\s*\(at ([\d.eE+-]+) ([\d.eE+-]+)(?: ([\d.eE+-]+))?\)',
                      padfix, body)

        rt = '' if rot == 0 else ' ' + _n(rot)
        self.items.append(
            '\t(footprint "%s"\n\t\t(layer "%s")\n\t\t(uuid "%s")\n\t\t(at %s %s%s)\n'
            '\t\t(property "Reference" "%s"\n\t\t\t(at %s %s %s)\n\t\t\t(layer "F.SilkS")\n'
            '\t\t\t(uuid "%s")\n\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 0.8 0.8)\n'
            '\t\t\t\t\t(thickness 0.15)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n'
            '\t\t(property "Value" "%s"\n\t\t\t(at 0 1.5 %s)\n\t\t\t(layer "F.Fab")\n'
            '\t\t\t(uuid "%s")\n\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 0.8 0.8)\n'
            '\t\t\t\t\t(thickness 0.15)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n%s)'
            % (fp, "B.Cu" if back else "F.Cu", self._uid("fp" + ref),
               _n(at[0]), _n(at[1]), rt,
               ref, _n(ref_at[0]), _n(ref_at[1]), _n(rot), _uuid(ref + "reftext"),
               value, _n(rot), _uuid(ref + "valtext"), body))

        return lambda num: xy(fp, num, at, rot)

    # ---- copper ------------------------------------------------------------
    def track(self, net, *pts, layer="F.Cu", width=0.4):
        n = self.net(net)
        for a, b in zip(pts, pts[1:]):
            if a == b:
                continue
            self.items.append(
                '\t(segment\n\t\t(start %s %s)\n\t\t(end %s %s)\n\t\t(width %s)\n'
                '\t\t(layer "%s")\n\t\t(net %d)\n\t\t(uuid "%s")\n\t)'
                % (_n(a[0]), _n(a[1]), _n(b[0]), _n(b[1]), _n(width), layer, n,
                   self._uid("t")))

    def via(self, net, at, size=0.8, drill=0.4):
        self.items.append(
            '\t(via\n\t\t(at %s %s)\n\t\t(size %s)\n\t\t(drill %s)\n'
            '\t\t(layers "F.Cu" "B.Cu")\n\t\t(net %d)\n\t\t(uuid "%s")\n\t)'
            % (_n(at[0]), _n(at[1]), _n(size), _n(drill), self.net(net), self._uid("v")))

    def outline(self, x0, y0, x1, y1, r=2.0):
        """Rounded rectangle board edge."""
        seg = [((x0 + r, y0), (x1 - r, y0)), ((x1, y0 + r), (x1, y1 - r)),
               ((x1 - r, y1), (x0 + r, y1)), ((x0, y1 - r), (x0, y0 + r))]
        for a, b in seg:
            self.items.append(
                '\t(gr_line\n\t\t(start %s %s)\n\t\t(end %s %s)\n'
                '\t\t(stroke\n\t\t\t(width 0.1)\n\t\t\t(type default)\n\t\t)\n'
                '\t\t(layer "Edge.Cuts")\n\t\t(uuid "%s")\n\t)'
                % (_n(a[0]), _n(a[1]), _n(b[0]), _n(b[1]), self._uid("edge")))
        for cx, cy, sa in ((x0 + r, y0 + r, 180), (x1 - r, y0 + r, 270),
                           (x1 - r, y1 - r, 0), (x0 + r, y1 - r, 90)):
            p = [(cx + r * math.cos(math.radians(sa + t)),
                  cy + r * math.sin(math.radians(sa + t))) for t in (0, 45, 90)]
            self.items.append(
                '\t(gr_arc\n\t\t(start %s %s)\n\t\t(mid %s %s)\n\t\t(end %s %s)\n'
                '\t\t(stroke\n\t\t\t(width 0.1)\n\t\t\t(type default)\n\t\t)\n'
                '\t\t(layer "Edge.Cuts")\n\t\t(uuid "%s")\n\t)'
                % (_n(p[0][0]), _n(p[0][1]), _n(p[1][0]), _n(p[1][1]),
                   _n(p[2][0]), _n(p[2][1]), self._uid("arc")))

    def zone(self, net, x0, y0, x1, y1, layers=("B.Cu",)):
        pts = "".join("(xy %s %s) " % (_n(a), _n(b))
                      for a, b in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)))
        lay = "".join('"%s" ' % l for l in layers).strip()
        self.items.append(
            '\t(zone\n\t\t(net %d)\n\t\t(net_name "%s")\n\t\t(layers %s)\n\t\t(uuid "%s")\n'
            '\t\t(name "%s")\n\t\t(hatch edge 0.5)\n\t\t(connect_pads\n\t\t\t(clearance 0.3)\n\t\t)\n'
            '\t\t(min_thickness 0.25)\n\t\t(filled_areas_thickness no)\n'
            '\t\t(fill yes\n\t\t\t(thermal_gap 0.4)\n\t\t\t(thermal_bridge_width 0.5)\n\t\t)\n'
            '\t\t(polygon\n\t\t\t(pts\n\t\t\t\t%s\n\t\t\t)\n\t\t)\n\t)'
            % (self.net(net), net, lay, self._uid("zone"), net, pts))

    def text(self, body, at, layer="F.SilkS", size=1.0, rot=0):
        self.items.append(
            '\t(gr_text "%s"\n\t\t(at %s %s %s)\n\t\t(layer "%s")\n\t\t(uuid "%s")\n'
            '\t\t(effects\n\t\t\t(font\n\t\t\t\t(size %s %s)\n\t\t\t\t(thickness 0.15)\n\t\t\t)\n'
            '\t\t\t(justify left)\n\t\t)\n\t)'
            % (body, _n(at[0]), _n(at[1]), _n(rot), layer, self._uid("txt"),
               _n(size), _n(size)))

    # ---- output ------------------------------------------------------------
    def render(self):
        nets = "\n".join('\t(net %d "%s")' % (i, n)
                         for n, i in sorted(self.nets.items(), key=lambda kv: kv[1]))
        layers = "\n".join('\t\t%s' % l for l in [
            '(0 "F.Cu" signal)', '(2 "B.Cu" signal)',
            '(9 "F.Adhes" user "F.Adhesive")', '(11 "B.Adhes" user "B.Adhesive")',
            '(13 "F.Paste" user)', '(15 "B.Paste" user)',
            '(5 "F.SilkS" user "F.Silkscreen")', '(7 "B.SilkS" user "B.Silkscreen")',
            '(1 "F.Mask" user)', '(3 "B.Mask" user)',
            '(17 "Dwgs.User" user "User.Drawings")', '(19 "Cmts.User" user "User.Comments")',
            '(21 "Eco1.User" user "User.Eco1")', '(23 "Eco2.User" user "User.Eco2")',
            '(25 "Edge.Cuts" user)', '(27 "Margin" user)',
            '(31 "F.CrtYd" user "F.Courtyard")', '(29 "B.CrtYd" user "B.Courtyard")',
            '(35 "F.Fab" user)', '(33 "B.Fab" user)'])
        return ("(kicad_pcb\n\t(version %s)\n\t(generator \"pcbnew\")\n"
                "\t(generator_version \"9.0\")\n"
                "\t(general\n\t\t(thickness 1.6)\n\t\t(legacy_teardrops no)\n\t)\n"
                "\t(paper \"A4\")\n\t(title_block\n\t\t(title \"%s\")\n\t)\n"
                "\t(layers\n%s\n\t)\n"
                "\t(setup\n\t\t(pad_to_mask_clearance 0)\n"
                "\t\t(allow_soldermask_bridges_in_footprints no)\n\t\t(tenting front back)\n\t)\n"
                "%s\n%s\n)\n"
                % (PCB_VERSION, self.title, layers, nets, "\n".join(self.items)))

    def write(self, path):
        with open(path, "w") as f:
            f.write(self.render())
