#!/bin/bash
# Synthesise + place & route the fx68k-compatible CPU for the iCESugar-Pro (LFE5U-25F-6BG256C)
export PATH=$HOME/.local/opt/oss-cad-suite/bin:$PATH
cd "$(dirname "$0")"
yosys -q -l synth.log -p "read_verilog -sv ../rtl/m68020_core.sv ../rtl/m68020_fx68k.sv syn_top.sv; synth_ecp5 -top syn_top -json syn_top.json; stat" || exit 1
nextpnr-ecp5 --25k --package CABGA256 --speed 6 --json syn_top.json --lpf-allow-unconstrained \
    --freq 42.6 --timing-allow-fail --seed 1 --report pnr_report.json -l pnr.log > /dev/null 2>&1
grep -E "Info: Max frequency|Device utilisation" -A12 pnr.log | grep -E "MHz|LUT4|TRELLIS_FF|DP16KD|MULT18|TRELLIS_COMB|TRELLIS_RAMW" | head -20
