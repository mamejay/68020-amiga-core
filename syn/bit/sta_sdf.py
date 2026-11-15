#!/usr/bin/env python3
"""Multicycle-aware setup check of a routed nextpnr-ecp5 design, from its SDF.

nextpnr has no set_multicycle_path, so its own report always shows the m68020 core's
clock-enabled paths as failing at 86 MHz and hides everything behind them. This walks the
SDF that nextpnr wrote for the routed design (cell arcs + interconnect delays, max corner)
and checks every flop-to-flop path against the budget its two ends allow:

  B = sequential cells of m68020_core.sv (all under `if (ce)`, ce every CE_DIV clocks)
  C = the wrapper's registers written only on core_ce edges: c_ack, c_berr, c_rdata, c_rst_n
      (into the core), q_* (the core's request and RESET/HALTED outputs, re-registered for the
      68000 bus engine and the board)
  A = everything else
  B/C -> B/C: CE_DIV periods.  Every other pair: one period.
  Paths from input pins are not checked (the gateware false-paths them: all asynchronous
  motherboard inputs land in synchronisers); output pins have no setup check.

usage: sta_sdf.py routed.json routed.sdf CE_DIV [period_ns=11.628]
"""
import json, re, sys, collections

js, sdf, ce_div = sys.argv[1], sys.argv[2], int(sys.argv[3])
_pos = [a for a in sys.argv[1:] if not a.startswith("--")]
T = float(_pos[3]) if len(_pos) > 3 else 1000.0 / 86.0
CLOCK_PINS = {"CLK", "CLKA", "CLKB", "WCK"}

# ---- classify sequential cells from the routed netlist ------------------------------
mod = json.load(open(js))["modules"]["top"]
bitnames = collections.defaultdict(set)
for name, nn in mod["netnames"].items():
    for i, b in enumerate(nn["bits"]):
        bitnames[b].add(name if len(nn["bits"]) == 1 else f"{name}[{i}]")
C_NETS = re.compile(r"^cpu\.(c_ack|c_berr|c_rst_n|c_rdata|q_reset_out|q_halted|q_req|q_we|q_addr|q_wdata|q_size|q_fc)(\[\d+\])?$")

dom = {}
for n, c in mod["cells"].items():
    src = c.get("attributes", {}).get("src", "")
    if "m68020_core.sv" in src or n.startswith("cpu.core."):
        dom[n] = "B"
    elif c["type"] == "TRELLIS_FF" and "m68020_fx68k.sv" in src and \
            any(C_NETS.match(x) for x in bitnames[c["connections"]["Q"][0]]):
        dom[n] = "C"
    elif c["type"] == "TRELLIS_IO":
        dom[n] = "IO"
    else:
        dom[n] = "A"
ncls = collections.Counter(dom[n] for n, c in mod["cells"].items() if c["type"] in ("TRELLIS_FF", "DP16KD", "TRELLIS_RAMW"))
cc = collections.Counter(re.sub(r"_TRELLIS_FF.*", "", n) for n in dom if dom[n] == "C")
print(f"sequential cells: {dict(ncls)}   C: {dict(cc)}")


def budget(s, d):
    # both ends change / sample only on core_ce edges -> CE_DIV periods
    return ce_div * T if s in ("B", "C") and d in ("B", "C") else T


# ---- parse SDF ------------------------------------------------------------------------
unesc = lambda s: re.sub(r"\\(.)", r"\1", s)
def maxd(t):  # "(a:b:c)" -> c in ns
    return float(t.strip("()").split(":")[-1]) / 1000.0

edges = collections.defaultdict(list)   # node -> [(node, delay, kind)]
launch = []                             # (node, delay, domain)
negedge = set()                         # cells clocked on the falling edge (s_7m_fall)
setup = {}                              # node -> (setup_ns, domain)
indeg = collections.Counter()
inst = None
re_inst = re.compile(r"\(INSTANCE\s*(.*?)\)\s*$")
re_ic = re.compile(r"\(INTERCONNECT\s+(\S+)\s+(\S+)\s+(\([\d:.]+\))\s+(\([\d:.]+\))")
re_io = re.compile(r"\(IOPATH\s+(\S+)\s+(\S+)\s+(\([\d:.]+\))\s+(\([\d:.]+\))")
re_su = re.compile(r"\(SETUPHOLD\s+\((?:posedge|negedge)\s+(\S+)\)\s+\((posedge|negedge)\s+(\S+)\)\s+(\([\d:.]+\))\s+(\([\d:.]+\))")


def pin(s):
    s = unesc(s)
    i = s.rfind("/")
    return (s[:i], s[i + 1:])


for line in open(sdf):
    m = re_inst.search(line)
    if m:
        inst = unesc(m.group(1).strip()); continue
    m = re_ic.search(line)
    if m:
        a, b = pin(m.group(1)), pin(m.group(2))
        if b[1] in CLOCK_PINS:
            continue
        d = max(maxd(m.group(3)), maxd(m.group(4)))
        edges[a].append((b, d)); indeg[b] += 1; continue
    m = re_io.search(line)
    if m and inst:
        i, o = m.group(1), m.group(2)
        d = max(maxd(m.group(3)), maxd(m.group(4)))
        if i in CLOCK_PINS:
            launch.append(((inst, o), d, dom.get(inst, "A")))
        else:
            edges[(inst, i)].append(((inst, o), d)); indeg[(inst, o)] += 1
        continue
    m = re_su.search(line)
    if m and inst:
        if m.group(2) == "negedge":
            negedge.add(inst)
        # nextpnr writes (setup)(hold); take the larger as a conservative setup
        su = max(maxd(m.group(4)), maxd(m.group(5)))
        k = (inst, m.group(1))
        setup[k] = (max(su, setup.get(k, (0, 0))[0]), dom.get(inst, "A"))

# ---- longest paths, per launching domain ------------------------------------------------
arr = collections.defaultdict(dict)      # node -> {src_dom: (arrival, pred)}
for node, d, sd in launch:
    if node[0] in negedge:
        d += T / 2                       # launched half a period later
    if d > arr[node].get(sd, (-1,))[0]:
        arr[node][sd] = (d, None)
nodes = set(edges) | set(indeg) | set(arr)
q = collections.deque(n for n in nodes if indeg[n] == 0)
seen = 0
while q:
    n = q.popleft(); seen += 1
    for m_, d in edges.get(n, ()):
        for sd, (a, _) in arr[n].items():
            if sd == "IO":
                continue
            if a + d > arr[m_].get(sd, (-1,))[0]:
                arr[m_][sd] = (a + d, n)
        indeg[m_] -= 1
        if indeg[m_] == 0:
            q.append(m_)
if seen != len(nodes):
    print(f"WARNING: {len(nodes) - seen} nodes in combinational loops were not analysed")

# ---- check endpoints --------------------------------------------------------------------
worst = {}
fails = collections.Counter()
for k, (su, dd) in setup.items():
    for sd, (a, _) in arr.get(k, {}).items():
        need = a + su
        slack = budget(sd, dd) - need - (T / 2 if k[0] in negedge else 0)
        pair = (sd, dd)
        if slack < 0:
            fails[pair] += 1
        if pair not in worst or slack < worst[pair][0]:
            worst[pair] = (slack, need, k)


def trace(k, sd):
    p, out = k, []
    while p is not None:
        out.append(p)
        p = arr[p][sd][1]
    return out[::-1]


print(f"\nperiod {T:.3f} ns, CE_DIV {ce_div}: core budget {ce_div * T:.3f} ns\n")
print(f"{'launch->capture':16} {'budget':>8} {'worst path':>11} {'slack':>8}  failing endpoints")
ok = True
for pair in sorted(worst):
    slack, need, k = worst[pair]
    ok &= slack >= 0
    print(f"{pair[0] + ' -> ' + pair[1]:16} {budget(*pair):8.3f} {need:11.3f} {slack:8.3f}  {fails[pair]}")
for pair in sorted(worst):
    slack, need, k = worst[pair]
    path = trace(k, pair[0])
    print(f"\n{pair[0]}->{pair[1]} worst: {need:.3f} ns, {len(path)} pins")
    print(f"  from {path[0][0]}.{path[0][1]}")
    print(f"  to   {k[0]}.{k[1]}")
if "--detail" in sys.argv:
    grp = lambda c: re.sub(r"(_TRELLIS_FF.*|_LUT4.*|\$.*|\.\d+\.\d+.*)$", "", c)
    detail = collections.defaultdict(collections.Counter)
    for k, (su, dd) in setup.items():
        for sd, (a, _) in arr.get(k, {}).items():
            if budget(sd, dd) - a - su - (T / 2 if k[0] in negedge else 0) < 0:
                st = trace(k, sd)[0][0]
                detail[(sd, dd)][(grp(st), grp(k[0]) + "." + re.sub(r"\d+$", "", k[1]))] += 1
    for pair, c in sorted(detail.items()):
        print(f"\nfailing {pair[0]}->{pair[1]} by start -> end group:")
        for (a, b), n in c.most_common(25):
            print(f"  {n:5}  {a:40} -> {b}")
print("\nRESULT:", "PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
