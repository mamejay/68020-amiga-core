# nextpnr-ecp5 --pre-route hook: undo core_clk_split.py before anything is routed. Every core clock
# pin goes back onto the single clk_12x global, so routing and the bitstream see exactly the RTL's one clock. (The split only steers placement.)
import json, os

m = json.load(open(os.environ.get("CORE_CLK_MAP", "core_clk_pins.json")))
G = m["global"]
for cname, pname in m["moves"]:
    ctx.disconnectPort(cname, pname)
    ctx.connectPort(G, cname, pname)
n_users = None
for nname, net in ctx.nets:
    if nname == G:
        n_users = len(net.users)
print(f"core_clk_join: {len(m['moves'])} pins back on {G} ({n_users} users), before routing")
