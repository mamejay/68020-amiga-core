#!/usr/bin/env python3
"""Convert base64_gateware's Diamond LPF to one nextpnr-ecp5 accepts, keeping every IO setting.

usage: mklpf.py <diamond.lpf> <synth.json> <out.lpf>
Drops: rvl_alias (Reveal), MULTICYCLE (fx68k cells, nextpnr ignores them), BLOCK JTAGPATHS.
Expands DEFINE PORT GROUP + IOBUF GROUP into per-port IOBUF lines (nextpnr has no groups).
Adds FREQUENCY PORT clk_12x 86 MHZ (Diamond takes it from the .fdc).
"""
import fnmatch, json, re, sys

src, js, out = sys.argv[1:4]
mod = json.load(open(js))["modules"]["base64_top"]
ports = []
for name, p in mod["ports"].items():
    n = len(p["bits"])
    off = p.get("offset", 0)
    ports += [name] if n == 1 and off == 0 else [f"{name}[{i}]" for i in range(off, off + n)]

groups, lines = {}, []
for raw in open(src):
    s = raw.strip()
    if not s or s.startswith("#"):
        lines.append(raw.rstrip("\n")); continue
    for stmt in [x.strip() for x in s.split(";") if x.strip()]:
        w = stmt.split()
        verb = w[0].upper()
        if verb == "RVL_ALIAS" or verb == "MULTICYCLE":
            lines.append(f"# (dropped) {stmt};"); continue
        if verb == "BLOCK" and w[1].upper() not in ("ASYNCPATHS", "RESETPATHS"):
            lines.append(f"# (dropped) {stmt};"); continue
        m = re.match(r'DEFINE\s+PORT\s+GROUP\s+"([^"]+)"\s+(.*)', stmt, re.I)
        if m:
            pats = re.findall(r'"([^"]+)"', m.group(2))
            groups[m.group(1)] = [p for p in ports if any(fnmatch.fnmatchcase(p, q) for q in pats)]
            lines.append(f"# (expanded) {stmt};"); continue
        m = re.match(r'IOBUF\s+GROUP\s+"([^"]+)"\s+(.*)', stmt, re.I)
        if m:
            members = groups[m.group(1)]
            if not members: sys.exit(f"group {m.group(1)} matched no ports")
            lines.append(f"# (expanded) {stmt};")
            lines += [f'IOBUF PORT "{p}" {m.group(2)};' for p in members]
            continue
        lines.append(stmt + ";")
lines.append('FREQUENCY PORT "clk_12x" 86 MHZ;')
open(out, "w").write("\n".join(lines) + "\n")

# every top-level port must be LOCATEd
loc = set(re.findall(r'LOCATE\s+COMP\s+"([^"]+)"', "\n".join(lines)))
missing = [p for p in ports if p not in loc]
print(f"{len(ports)} ports, {len(loc)} located, groups: " + ", ".join(f"{k}={len(v)}" for k, v in groups.items()))
if missing: sys.exit("unlocated ports: " + " ".join(missing))
