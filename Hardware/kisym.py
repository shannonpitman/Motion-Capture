"""kisym.py - pull symbol definitions and pin geometry out of the stock KiCad
symbol libraries so gen_schematics.py can embed them and wire to them.

Nothing here is specific to the wand. It is a small S-expression reader plus
the one coordinate transform that matters: symbol libraries use +Y up, the
schematic canvas uses +Y down, so every pin has to be flipped before it can be
compared with a wire endpoint.
"""

import os
import re

SYMDIR = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols"


def _tokenize(s):
    out, i, n = [], 0, len(s)
    while i < n:
        c = s[i]
        if c in "()":
            out.append(c); i += 1
        elif c == '"':
            j = i + 1; buf = []
            while j < n:
                if s[j] == "\\":
                    buf.append(s[j + 1]); j += 2
                elif s[j] == '"':
                    break
                else:
                    buf.append(s[j]); j += 1
            out.append(('"', "".join(buf))); i = j + 1
        elif c.isspace():
            i += 1
        else:
            j = i
            while j < n and not s[j].isspace() and s[j] not in '()"':
                j += 1
            out.append(s[i:j]); i = j
    return out


def _parse(tokens, i=0):
    # tokens[i] must be "("
    assert tokens[i] == "(", tokens[i]
    i += 1
    node = []
    while tokens[i] != ")":
        t = tokens[i]
        if t == "(":
            sub, i = _parse(tokens, i)
            node.append(sub)
        else:
            node.append(t); i += 1
    return node, i + 1


def _atom(x):
    return x[1] if isinstance(x, tuple) else x


def _find_all(node, head):
    return [c for c in node if isinstance(c, list) and c and _atom(c[0]) == head]


def _find(node, head):
    r = _find_all(node, head)
    return r[0] if r else None


_LIBCACHE = {}


def _load_lib(libname):
    if libname in _LIBCACHE:
        return _LIBCACHE[libname]
    path = os.path.join(SYMDIR, libname + ".kicad_sym")
    with open(path) as f:
        text = f.read()
    toks = _tokenize(text)
    tree, _ = _parse(toks, 0)
    by_name = {}
    for sym in _find_all(tree, "symbol"):
        by_name[_atom(sym[1])] = sym
    _LIBCACHE[libname] = (text, by_name)
    return _LIBCACHE[libname]


def raw_symbol(lib_id):
    """The literal source text of a top-level symbol block, re-headed as
    `(symbol "Lib:Name"` so it can be dropped straight into lib_symbols."""
    libname, name = lib_id.split(":")
    text, _ = _load_lib(libname)
    start = text.index('\n\t(symbol "%s"\n' % name) + 1
    # walk to the matching close paren, ignoring parens inside strings
    depth, i, n, instr = 0, start, len(text), False
    while i < n:
        c = text[i]
        if instr:
            if c == "\\":
                i += 2; continue
            if c == '"':
                instr = False
        elif c == '"':
            instr = True
        elif c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                i += 1
                break
        i += 1
    block = text[start:i]
    return block.replace('(symbol "%s"' % name, '(symbol "%s"' % lib_id, 1)


def pins(lib_id):
    """{pin_number: (x, y)} in LIBRARY coordinates (+Y up).

    A pin's (at x y angle) is its electrical connection point, so the tip is
    what we return - no length offset needed.
    """
    libname, name = lib_id.split(":")
    _, by_name = _load_lib(libname)
    sym = by_name[name]
    out = {}
    for sub in _find_all(sym, "symbol"):
        for p in _find_all(sub, "pin"):
            at = _find(p, "at")
            num = _find(p, "number")
            if at is None or num is None:
                continue
            out[_atom(num[1])] = (float(_atom(at[1])), float(_atom(at[2])))
    return out


def pin_xy(lib_id, number, at, rot=0, mirror=None):
    """Pin `number` in SCHEMATIC coordinates for an instance placed at `at`
    with `rot` degrees counter-clockwise.

    Library +Y is up and schematic +Y is down, so the pin offset is flipped to
    (px, -py) first; the rotation is then applied in screen space.
    """
    import math
    px, py = pins_flat(lib_id)[str(number)]
    dx, dy = px, -py
    if mirror == "x":       # mirror about the horizontal axis
        dy = -dy
    elif mirror == "y":     # mirror about the vertical axis
        dx = -dx
    t = math.radians(rot)
    c, s = math.cos(t), math.sin(t)
    gx = at[0] + dx * c + dy * s
    gy = at[1] - dx * s + dy * c
    return (round(gx, 4), round(gy, 4))

def _extends_of(lib_id):
    libname, name = lib_id.split(":")
    _, by_name = _load_lib(libname)
    e = _find(by_name[name], "extends")
    return _atom(e[1]) if e else None


def raw_symbol_flat(lib_id):
    """raw_symbol(), but with any `(extends "Parent")` resolved in place.

    KiCad derives a symbol's graphics from its parent, so a derived symbol on
    its own has no pins. Embedding one in lib_symbols without its parent gives
    a schematic with an empty box, so splice the parent's drawing units in and
    drop the extends line - the properties on the derived symbol already
    override the parent's.
    """
    parent = _extends_of(lib_id)
    block = raw_symbol(lib_id)
    if parent is None:
        return block
    libname, name = lib_id.split(":")
    pblock = raw_symbol("%s:%s" % (libname, parent))
    units = re.findall(r"\n\t\t\(symbol \"%s_\d+_\d+\".*?\n\t\t\)" % re.escape(parent),
                       pblock, re.S)
    units = [u.replace('"%s_' % parent, '"%s_' % name) for u in units]
    block = re.sub(r"\n\t\t\(extends \"[^\"]*\"\)", "", block)
    return block[:block.rindex(")")].rstrip() + "".join(units) + "\n\t)"


def pins_flat(lib_id):
    parent = _extends_of(lib_id)
    if parent is None:
        return pins(lib_id)
    libname, _ = lib_id.split(":")
    return pins("%s:%s" % (libname, parent))
