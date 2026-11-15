# 68020 Amiga Core

An original 68020-compatible CPU core in SystemVerilog, written for the
[Base64](https://github.com/jbilander/Base64) 68000-socket accelerator (iCESugar-Pro, Lattice ECP5
LFE5U-25F). It drops into the [base64_gateware](https://github.com/jbilander/base64_gateware)
fx68k design in place of the fx68k core, through a wrapper with fx68k's exact port list.

The core was written from the Motorola documentation (M68000PRM, MC68020UM). TG68K.C and
base64_gateware were read for reference only; no code was taken from them.

## Status (16 Nov 2026)

| | |
|---|---|
| ISA | full 68020 user + supervisor set except CALLM/RTM. No FPU/MMU: coprocessor opcodes take the F-line trap, as on a bare 68020 |
| Addressing | all 68020 modes, including full-format extension words (memory indirect pre/post-indexed, scale, suppressed base/index) |
| Exceptions | format $0/$1/$2/$A frames, VBR, MSP/ISP (M bit), interrupt throwaway frame, T1/T0 trace, address error on odd PC, bus error (restart model), double-fault halt |
| Cache | 64 x 32-bit direct-mapped instruction cache (CACR E/F/CE/C, CAAR), coherent with the core's own writes |
| Verification | lockstep against Musashi (68020): 9 directed programs + over 1.3 million random single-instruction tests, run both on the native bus and through the 68000-bus wrapper. 0 open mismatches |
| Size | core + wrapper ~14.3k LUT4 + 880 carry cells, 1 DSP, 2 EBR. The full Base64 design with this core fits the LFE5U-25F (see doc/integration.md) |
| Speed | nextpnr Fmax of the core ~31 MHz, so the wrapper runs it at 85 MHz / 3 = 28.4 MHz (`CE_DIV=3`). About 5.9 clocks per instruction on a cached integer loop |
| Hardware | **not yet run on a Base64.** Diamond build + board bring-up are the next step |

## Layout

```
rtl/m68020_core.sv     the CPU (native 32-bit request/ack bus, see doc/core_interface.md)
rtl/m68020_fx68k.sv    fx68k-compatible wrapper: 68000 bus cycles on enPhi1/enPhi2, CE_DIV clock enable
sim/                   Verilator + Musashi lockstep harness, directed tests (tests/*.S)
syn/                   yosys/nextpnr scripts (core-only P&R, full Base64 fit check in syn/b64)
doc/                   architecture, interface, Base64 integration, verification notes
```

## Quick start

Needs [oss-cad-suite](https://github.com/YosysHQ/oss-cad-suite-build) (Verilator 5, yosys, nextpnr-ecp5)
and `m68k-linux-gnu` binutils for the test programs.

```
cd sim
make                     # ./lockstep        core on its native bus
make lockstep_fx         # ./lockstep_fx     core inside the fx68k wrapper (68000 bus cycles)
make tests               # assemble tests/*.S
./lockstep prog tests/exceptions.bin            # directed test, lockstep with Musashi
./lockstep random --seed 1 --count 20000        # random instructions, lockstep with Musashi
./lockstep prog tests/bench.bin --nowait -v     # per-instruction cycle counts

cd ../syn && ./run_pnr.sh                        # core + wrapper P&R for LFE5U-25F (Fmax, utilisation)
cd b64 && ./run_b64.sh                           # whole Base64 design with this core (fit check)
```

## Documents

* [doc/architecture.md](doc/architecture.md): how the core works and where it differs from a real 68020
* [doc/core_interface.md](doc/core_interface.md): native bus and debug port contract
* [doc/integration.md](doc/integration.md): putting it into base64_gateware (Diamond), clocking and constraints
* [doc/verification.md](doc/verification.md): harness, test coverage, Musashi corrections, results
