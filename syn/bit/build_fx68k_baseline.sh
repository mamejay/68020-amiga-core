#!/bin/bash
# Reference: the ORIGINAL base64_gateware fx68k design through the same yosys/nextpnr flow,
# to tell timing problems of the open toolchain apart from problems of the m68020 core.
set -e
export PATH=$HOME/.local/opt/oss-cad-suite/bin:$PATH
cd "$(dirname "$0")"; O=out/fx68k_baseline; mkdir -p $O; cd $O
G=../../../../ref/base64_gateware; R=$G/rtl/base64_fx68k; F=$G/rtl/fx68k
cp $R/*.mem $F/*.mem .
SRCS="$R/autoconfig_zii.v $R/fastmem_zii.v $R/fifo.v $R/rx_cpu_buf.v $R/tx_cpu_buf.v $R/sd_subsystem.v $R/sdcard.v $R/sdram_ctrl.v $R/shifter.v $R/turbomem_zii.v"
sed "s/parameter string /parameter /; s/parameter bit /parameter /" $F/bram/fx68kRom_generic.sv > fx68kRom_generic.sv
[ -f synth.json ] || yosys -m slang -q -l synth.log -p "read_verilog -lib +/ecp5/cells_bb.v; read_slang --compat vcs --allow-use-before-declare --extern-modules --top base64_top $F/fx68k_pkg.sv $F/uaddrPla.sv $F/fx68kAlu.sv $F/bram/fx68kRom_generic.sv $F/bram/fx68kRegs_generic.sv $F/fx68k.sv $R/base64_top.sv $SRCS; synth_ecp5 -top base64_top -json synth.json"
python3 ../../mklpf.py $G/fx68k/constraints/base64.lpf synth.json base64_nextpnr.lpf
nextpnr-ecp5 --25k --package CABGA256 --speed 6 --json synth.json --lpf base64_nextpnr.lpf \
    --timing-allow-fail --seed 1 --textcfg base64.config --write routed.json --sdf routed.sdf -l pnr.log > /dev/null 2>&1
grep -E "TRELLIS_COMB:|TRELLIS_FF:|Max frequency" pnr.log | tail -4
# same packing options as build_bit.sh, so this bitstream differs from the m68020 one only in the CPU
ecppack --compress --freq 62.0 --spimode dual-spi --input base64.config --bit base64_fx68k_nextpnr.bit
ls -l base64_fx68k_nextpnr.bit
