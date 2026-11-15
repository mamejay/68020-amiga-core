# nextpnr-ecp5 --pre-place hook: placement/routing aid for the m68020 multicycle paths.
#
# nextpnr has no multicycle constraint and normalises timing criticality per clock-domain pair.
# With one clock, the core's 3-cycle paths (~27 ns) set the scale, and the Base64 bridge/SDRAM
# paths (12-19 ns, really single-cycle) look non-critical, so the placer ignores them.
#
# This hook gives the core's sequential cells their own clock net for PLACEMENT only: the clk_12x
# input net (pin -> global buffer), constrained to 86/CE_DIV MHz. core_clk_join.py (pre-route)
# moves every pin back onto the single global clock before anything is routed, so routing and
# the bitstream see exactly the RTL's one clock. Sign-off is sta_sdf.py on the final SDF.
import json, os

CE_DIV = int(os.environ.get("CE_DIV", "3"))
MAP = os.environ.get("CORE_CLK_MAP", "core_clk_pins.json")

gbuf = None
for cname, cell in ctx.cells:
    if cell.type == "DCCA" and "clk_12x" in cname:
        gbuf = cell
assert gbuf is not None, "clk_12x DCCA not found (global promotion changed?)"
G = gbuf.ports["CLKO"].net
clk_in = gbuf.ports["CLKI"].net


def is_core(cname, cell):
    if cname.startswith("cpu.core."):
        return True
    for k, v in cell.attrs:
        if k == "src":
            return "m68020_core.sv" in str(v)
    return False


moves = []
for cname, cell in ctx.cells:
    if not is_core(cname, cell):
        continue
    for pname, port in cell.ports:
        if port.net is not None and port.net.name == G.name:
            moves.append((cname, pname))
assert len(moves) > 1000, f"only {len(moves)} core clock pins found"

# The core's clock pins go onto the clock INPUT net (pin -> DCCA), constrained to 86/CE_DIV MHz.
# No helper cell is needed, so core_clk_join.py only has to move the pins back.
for cname, pname in moves:
    ctx.disconnectPort(cname, pname)
    ctx.connectPort(clk_in.name, cname, pname)
ctx.addClock(clk_in.name, 86.0 / CE_DIV)
json.dump({"global": G.name, "moves": moves}, open(MAP, "w"))
print(f"core_clk_split: {len(moves)} core clock pins moved to {clk_in.name} ({86.0 / CE_DIV:.2f} MHz), "
      f"main clock {G.name}")
