#!/bin/bash
# Gate-level check of the FPGA netlist flow: m68020_fx68k (+core) through yosys synth_ecp5 with
# M68020_CE_POSTMAP, then ce_post.py, then the lockstep harness (tb_fx.sv, Musashi) on the
# resulting netlist simulated with the ECP5 cell models.
#   usage: ./run_gl.sh [CE_DIV=3] [random-count=3000]
set -e; set -o pipefail
CE_DIV=${1:-3}; COUNT=${2:-3000}
export PATH=$HOME/.local/opt/oss-cad-suite/bin:$PATH
cd "$(dirname "$0")"
LAT=$HOME/.local/opt/oss-cad-suite/share/yosys/lattice
O=out$CE_DIV; mkdir -p $O; cd $O
yosys -q -l synth.log -p "read_verilog -sv -DM68020_CE_POSTMAP ../../../rtl/m68020_core.sv ../../../rtl/m68020_fx68k.sv; chparam -set CE_DIV $CE_DIV -set DEBUG 1 m68020_fx68k; synth_ecp5 -top m68020_fx68k -json gl.json"
python3 ../../bit/ce_post.py gl.json gl_ce.json
yosys -q -p "read_json gl_ce.json; hierarchy -top m68020_fx68k; flatten; write_verilog -noattr gl.v"
sed -e 's/m68020_fx68k #(.DEBUG(1.b1), .CE_DIV(`CE_DIV)) u_cpu (/m68020_fx68k u_cpu (/' ../../../sim/tb_fx.sv > tb_gl.sv
grep -q "m68020_fx68k u_cpu (" tb_gl.sv
S=../../../sim
rm -f obj/Vtb
verilator --cc --exe --build -j 0 -Wno-fatal -Wno-lint -Wno-style -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-SPECIFYIGN -O3 --top-module tb \
  -I$LAT -CFLAGS "-O2 -I$(cd $S; pwd)/musashi" -LDFLAGS "$(cd $S; pwd)/obj_mus/*.o -lm" \
  --Mdir obj tb_gl.sv gl.v ../gl_models.v $S/lockstep.cpp -o Vtb > verilator.log 2>&1 || { tail -30 verilator.log; exit 1; }
fail=0
for t in $S/tests/*.bin; do ./obj/Vtb prog $t --max 200000 || fail=1; done
./obj/Vtb random --seed 7 --count $COUNT --show 5 || fail=1
echo "GL RESULT: $([ $fail = 0 ] && echo PASS || echo FAIL)"
exit $fail
