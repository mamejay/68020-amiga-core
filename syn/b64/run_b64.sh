#!/bin/bash
# Full base64_top (fx68k gateware) with m68020_fx68k in place of fx68k: synth + P&R fit check
export PATH=$HOME/.local/opt/oss-cad-suite/bin:$PATH
cd "$(dirname "$0")"
R=../../ref/base64_gateware/rtl/base64_fx68k
sed 's/^\(\s*\)fx68k cpu (/\1m68020_fx68k #(.CE_DIV(3)) cpu (/' $R/base64_top.sv > base64_top_020.sv
cp $R/*.mem .
SRCS="$R/autoconfig_zii.v $R/fastmem_zii.v $R/fifo.v $R/rx_cpu_buf.v $R/tx_cpu_buf.v $R/sd_subsystem.v $R/sdcard.v $R/sdram_ctrl.v $R/shifter.v $R/turbomem_zii.v"
yosys -q -l b64.log -p "read_verilog -sv ../../rtl/m68020_core.sv ../../rtl/m68020_fx68k.sv base64_top_020.sv; read_verilog $SRCS; synth_ecp5 -top base64_top -json b64.json; stat" || exit 1
nextpnr-ecp5 --25k --package CABGA256 --speed 6 --json b64.json --lpf base64_np.lpf --lpf-allow-unconstrained \
    --timing-allow-fail --seed 1 -l pnr.log > /dev/null 2>&1
grep -E "TRELLIS_COMB|TRELLIS_FF:|DP16KD:|Max frequency" pnr.log | tail -6
