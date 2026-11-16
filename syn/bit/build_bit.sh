#!/bin/bash
# Build a Base64 bitstream (base64_gateware fx68k design, fx68k swapped for m68020_fx68k)
# with the open-source toolchain: yosys -> nextpnr-ecp5 -> ecppack.
#   usage: ./build_bit.sh [CE_DIV=3] [SEED=1]
# Output: out/cediv<N>/base64_m68020.bit  (+ routed.json, pnr.log, report.json)
set -e
set -o pipefail
CE_DIV=${1:-3}; SEED=${2:-1}
export PATH=$HOME/.local/opt/oss-cad-suite/bin:$PATH
cd "$(dirname "$0")"
R=../../ref/base64_gateware/rtl/base64_fx68k
L=../../ref/base64_gateware/fx68k/constraints/base64.lpf
O=out/cediv$CE_DIV; mkdir -p $O
sed "s/^\(\s*\)fx68k cpu (/\1m68020_fx68k #(.CE_DIV($CE_DIV)) cpu (/" $R/base64_top.sv > $O/base64_top_020.sv
grep -q "m68020_fx68k #(.CE_DIV($CE_DIV)) cpu (" $O/base64_top_020.sv || { echo "instance patch failed"; exit 1; }
cp $R/*.mem $O/
SRCS="$R/autoconfig_zii.v $R/fastmem_zii.v $R/fifo.v $R/rx_cpu_buf.v $R/tx_cpu_buf.v $R/sd_subsystem.v $R/sdcard.v $R/sdram_ctrl.v $R/shifter.v $R/turbomem_zii.v"
cd $O
if [ ! -f synth.json ] || [ ../../../../rtl/m68020_core.sv -nt synth.json ] || [ ../../../../rtl/m68020_fx68k.sv -nt synth.json ]; then
  echo "== yosys"; yosys -q -l synth.log -p "read_verilog -sv -DM68020_CE_POSTMAP ../../../../rtl/m68020_core.sv ../../../../rtl/m68020_fx68k.sv base64_top_020.sv; read_verilog ${SRCS//$R/../../$R}; synth_ecp5 -top base64_top -json synth.json"
fi
# the core's clock enable goes onto every core register's enable pin (see ce_post.py)
python3 ../../ce_post.py synth.json synth_ce.json
python3 ../../mklpf.py ../../$L synth.json base64_nextpnr.lpf
echo "== nextpnr (seed $SEED)"
# core_clk_split/join: the core gets its own clock net during placement only (see the scripts)
CE_DIV=$CE_DIV nextpnr-ecp5 --25k --package CABGA256 --speed 6 --json synth_ce.json --lpf base64_nextpnr.lpf \
    --timing-allow-fail --seed $SEED --pre-place ../../core_clk_split.py --pre-route ../../core_clk_join.py \
    --textcfg base64.config --write routed.json --report report.json --sdf routed.sdf -l pnr.log > pnr.out 2>&1
grep -E "core_clk_(split|join)" pnr.out
grep -E "TRELLIS_COMB:|TRELLIS_FF:|DP16KD:|MULT18X18D:" pnr.log | tail -4
echo "== ecppack"
# No --spimode: Diamond's .bit (the working fx68k-v1.2.2 release) has no SPI-mode command, and
# the dual-spi one ecppack adds is the suspected reason earlier builds were dead on hardware.
ecppack --compress --freq 62.0 --input base64.config --bit base64_m68020.bit
ls -l base64_m68020.bit
echo "== timing sign-off (multicycle-aware, from the routed SDF)"
python3 ../../sta_sdf.py routed.json routed.sdf $CE_DIV | tee sta.txt | sed -n '/launch->capture/,/^$/p;/RESULT/p' || true
