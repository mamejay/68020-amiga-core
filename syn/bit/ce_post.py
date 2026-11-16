#!/usr/bin/env python3
"""Apply the m68020 core's clock enable after synthesis.

usage: ce_post.py in.json out.json

The core is synthesised with M68020_CE_POSTMAP: its main `if (ce)` is tied true and the module is
kept as its own netlist module (keep_hierarchy). That netlist is therefore the core running on
every clock. This pass gates every register and memory port in that module with the module's `ce`
port:

  TRELLIS_FF        CE  := ce & CE        LSR := ce & LSR    (sync set/reset only)
  TRELLIS_DPR16X4   WRE := ce & WRE
  DP16KD            CEA := ce & CEA       CEB := ce & CEB    (no output registers)

so the whole module advances only on ce edges: exactly the RTL's `if (ce)`, with `ce` one LUT from
every enable pin, which is what timing needs. (Synthesis left to itself folded `ce` into the
next-state logic up to 7 LUTs deep, and `ce` is a single-cycle signal.)

Anything this pass does not understand (another clocked cell type, an asynchronous reset, an
inverted or second clock) stops the build.
"""
import json, re, sys

src, dst = sys.argv[1], sys.argv[2]
d = json.load(open(src))
mods = [n for n in d["modules"] if re.search(r"(^|\\)m68020_core(\\|$)", n)]
if len(mods) != 1:
    sys.exit(f"ce_post: expected one m68020_core module, found {mods} (keep_hierarchy / M68020_CE_POSTMAP missing?)")
M = d["modules"][mods[0]]

maxbit = 1
for m in d["modules"].values():
    for nn in m["netnames"].values():
        maxbit = max([maxbit] + [b for b in nn["bits"] if isinstance(b, int)])
for m in d["modules"].values():
    for c in m["cells"].values():
        for bits in c["connections"].values():
            maxbit = max([maxbit] + [b for b in bits if isinstance(b, int)])

ce = M["ports"]["ce"]["bits"][0]
clk = M["ports"]["clk"]["bits"][0]
assert isinstance(ce, int) and isinstance(clk, int)

gates = {}            # (bit, inverted) -> gated bit
new_cells = {}


def gated(bit, inv):
    """bit of (ce & bit) or (ce & ~bit); bit may be a constant string."""
    global maxbit
    if bit in ("1", 1) and not inv or bit == "0" and inv:
        return ce
    if bit in ("0", "x") and not inv or bit == "1" and inv:
        return None       # never enabled: leave the pin alone
    key = (bit, inv)
    if key not in gates:
        maxbit += 1
        g = maxbit
        name = f"ce_gate_{len(gates)}"
        new_cells[name] = {
            "hide_name": 0, "type": "LUT4",
            # Z = A & B (A = ce, B = enable) or A & ~B, independent of C/D
            "parameters": {"INIT": "0010001000100010" if inv else "1000100010001000"},
            "attributes": {"src": "syn/bit/ce_post.py"},
            "port_directions": {"A": "input", "B": "input", "C": "input", "D": "input", "Z": "output"},
            "connections": {"A": [ce], "B": [bit], "C": ["0"], "D": ["0"], "Z": [g]},
        }
        M["netnames"][f"ce_gate_{len(gates)}_Z"] = {"hide_name": 0, "bits": [g], "attributes": {}}
        gates[key] = g
    return gates[key]


def p(cell, k, default):
    v = cell["parameters"].get(k, default)
    return v.strip() if isinstance(v, str) else v


counts = {}
for name, c in M["cells"].items():
    t = c["type"]
    cn = c["connections"]
    if t == "TRELLIS_FF":
        if p(c, "CLKMUX", "CLK") != "CLK" or cn["CLK"] != [clk]:
            sys.exit(f"ce_post: {name}: unexpected clock {cn['CLK']} / {p(c, 'CLKMUX', 'CLK')}")
        # clock enable
        mux = p(c, "CEMUX", "1")
        bit = cn.get("CE", ["1"])[0]
        if mux in ("1", "0"):
            bit, mux = mux, "CE"
        g = gated(bit, mux == "INV")
        if g is None:
            sys.exit(f"ce_post: {name}: CE never enabled")
        cn["CE"] = [g]; c["parameters"]["CEMUX"] = "CE"
        # synchronous set/reset (LSR_OVER_CE: applies regardless of CE, so it is gated too)
        lmux = p(c, "LSRMUX", "LSR")
        lbit = cn.get("LSR", ["0"])[0]
        if lmux in ("1", "0"):
            lbit, lmux = lmux, "LSR"
        if not (lbit == "0" and lmux == "LSR"):
            if p(c, "SRMODE", "LSR_OVER_CE") == "ASYNC":
                sys.exit(f"ce_post: {name}: asynchronous reset in the core")
            g = gated(lbit, lmux == "INV")
            cn["LSR"] = [g] if g is not None else ["0"]
            c["parameters"]["LSRMUX"] = "LSR"
        counts[t] = counts.get(t, 0) + 1
    elif t == "TRELLIS_DPR16X4":
        if p(c, "WCKMUX", "WCK") != "WCK" or cn["WCK"] != [clk]:
            sys.exit(f"ce_post: {name}: unexpected write clock")
        mux = p(c, "WREMUX", "WRE")
        bit = cn.get("WRE", ["1"])[0]
        if mux in ("1", "0"):
            bit, mux = mux, "WRE"
        g = gated(bit, mux == "INV")
        cn["WRE"] = [g] if g is not None else ["0"]
        c["parameters"]["WREMUX"] = "WRE"
        counts[t] = counts.get(t, 0) + 1
    elif t == "DP16KD":
        for k in ("REGMODE_A", "REGMODE_B"):
            if p(c, k, "NOREG") != "NOREG":
                sys.exit(f"ce_post: {name}: output register ({k}) not handled")
        for port in "AB":
            if p(c, f"CLK{port}MUX", f"CLK{port}") != f"CLK{port}" or cn[f"CLK{port}"] != [clk]:
                sys.exit(f"ce_post: {name}: unexpected CLK{port}")
            for k in (f"CE{port}MUX",):
                if p(c, k, f"CE{port}") != f"CE{port}":
                    sys.exit(f"ce_post: {name}: {k} not handled")
            g = gated(cn[f"CE{port}"][0], False)
            cn[f"CE{port}"] = [g] if g is not None else ["0"]
        counts[t] = counts.get(t, 0) + 1
    elif t in ("LUT4", "PFUMX", "L6MUX21", "CCU2C", "MULT18X18D"):
        if t == "MULT18X18D" and any(k.startswith("CLK") and v not in (["0"], ["x"]) for k, v in cn.items()):
            sys.exit(f"ce_post: {name}: registered multiplier not handled")
        pass
    else:
        sys.exit(f"ce_post: {name}: cell type {t} not handled")

# pins this pass connected that the synthesised cell had left out
for c in M["cells"].values():
    for k in c["connections"]:
        c["port_directions"].setdefault(k, "input")
M["cells"].update(new_cells)
json.dump(d, open(dst, "w"))
print(f"ce_post: {mods[0]}: gated {counts} with ce through {len(gates)} LUTs")
