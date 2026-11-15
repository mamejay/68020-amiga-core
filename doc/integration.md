# Integrating with base64_gateware (fx68k build)

`rtl/m68020_fx68k.sv` has exactly the port list of the fx68k instance in
`rtl/base64_fx68k/base64_top.sv` (fredrequin fork, `eab[31:1]`), so the swap is one line plus
project and constraint changes. Nothing else in `base64_top.sv` changes: the bus bridge, the
internal slaves (SDRAM fast RAM, turbomem ROM, SD card), autoconfig, IPL filtering, BG retiming
and the RESET stretch all keep working as they do now.

## 1. Sources

Copy `rtl/m68020_core.sv` and `rtl/m68020_fx68k.sv` into the gateware tree, e.g. `rtl/m68020/`.

In `prj/base64_fx68k/base64_fx68k.ldf` (Diamond), **remove** the fx68k sources

```
rtl/fx68k/fx68k_pkg.sv  rtl/fx68k/uaddrPla.sv  rtl/fx68k/fx68kAlu.sv
rtl/fx68k/bram/fx68kRom_generic.sv  rtl/fx68k/bram/fx68kRegs_generic.sv  rtl/fx68k/fx68k.sv
```

and **add** the two m68020 files as SystemVerilog.

## 2. base64_top.sv

Section 8, change the instance line only:

```systemverilog
-    fx68k cpu (
+    m68020_fx68k #(.CE_DIV(3)) cpu (
```

All the port connections stay as they are. `E` and `VMAn` are driven inactive (the top already
ignores them), and `VPAn` is ignored (the bridge runs all 6800 cycles).

## 3. Clocking and constraints

The wrapper's 68000 bus engine runs on the same `enPhi1`/`enPhi2` grid as fx68k did, so the
motherboard sees the same S0..S7 cycles. The core itself runs on a clock enable:

| `CE_DIV` | core clock | status |
|---|---|---|
| 2 | `enPhi1`, 42.6 MHz | target; does not close yet (nextpnr Fmax of the core ~31 MHz) |
| 3 | clk/3, 28.4 MHz | **use this**; meets timing in nextpnr with margin |
| 4 | clk/4, 21.3 MHz | fallback if Diamond disagrees |

Every core flop is enabled by that clock enable, and the wrapper only changes the core's inputs
(`c_ack`, `c_berr`, `c_rdata`) on clock-enable edges. So **every path into and inside `cpu.core`
is a CE_DIV-cycle path**. In `fx68k/constraints/base64.fdc`:

1. Delete the fx68k-specific `set_multicycle_path` and `define_attribute` lines that name
   `cpu.excUnit.*` / `cpu.U_nanoRom_t3.*`. Those cells no longer exist, and each line would
   produce a BN105 warning.
2. Add (for `CE_DIV = 3`):

```tcl
# m68020 core: all flops are clock-enabled every 3rd clk_12x
set_multicycle_path -setup 3 -hold 2 -from [get_cells {cpu.core.*}] -to [get_cells {cpu.core.*}]
set_multicycle_path -setup 3 -hold 2 -from [get_cells {cpu.c_ack* cpu.c_berr* cpu.c_rdata*}] -to [get_cells {cpu.core.*}]
```

The fx68k notes warn against a blanket `cpu.*` multicycle. That warning is right for fx68k,
which has flops that run every clock, but doesn't apply here: everything inside `cpu.core` is
under `if (ce)`. Paths from the core to the wrapper (`c_req`, `c_addr`, ...) are plain
flop-to-flop and stay single-cycle. Check `grep -c BN105` = 0 after the build, as the existing
fdc comments describe.

## 4. Resources

The whole Base64 design with this core in place of fx68k (yosys + nextpnr, LFE5U-25F-6BG256C):
see `syn/b64/b64_result.txt` for the latest run. At v0.3 it was 17.7k of 24.3k logic cells (72%),
21 of 56 EBR, 1 DSP. Diamond's numbers will differ somewhat but there is headroom.

## 5. What changes for the Amiga

* **Memory map.** The core puts all 32 address bits on `eab[31:1]`. The fx68k build already
  decodes its internal slaves on the full width (Zorro II fast RAM, 32-bit space fast RAM,
  turbomem DiagArea ROM, SD). The motherboard still sees A23..A1, as with a real 68000 socket
  card.
* **68010+ privilege rules apply.** MOVE from SR is privileged (classic 68000-only software traps
  on any 68010/020 accelerator; Kickstart 2.0+ and Degrader-style patches deal with it).
  AmigaOS detects a 68020 (AttnFlags) and uses the 68020 stack frames, VBR and CACR.
* **Instruction cache.** Exec turns it on through CACR. Code loaded by DMA (trackloaders,
  blitter) needs a cache flush. AmigaOS does that; some games that only ever ran on a 68000 don't.
  `ICACHE=0` (parameter of `m68020_fx68k`) builds the core without it.
* **Not cycle exact.** Anything that depends on 68000 instruction timing will run differently,
  as on any 68020 accelerator.
* **TAS/CAS/CAS2** are not indivisible on the 68000 bus (AS is not held). Chip RAM doesn't
  support RMW anyway.

## 6. Performance (current)

On the direct bus with the cache on and no wait states, an integer loop averages ~5.9 core
clocks per instruction (`sim/tests/bench.S`). At 28.4 MHz that is roughly 4.8 MIPS on cached
code. Every data access and cache miss still goes through a full 68000-style cycle in the
wrapper (16 bits per 4 phase-clocks), so fast-RAM-heavy code is bus-bound.

Biggest wins still available, in order:
1. a 32-bit direct path from the core to the SDRAM fast-RAM slave, bypassing 68000-cycle
   emulation (long-word fetches and data in one access);
2. timing work to run the core at `enPhi1` (42.6 MHz): the critical paths are register-file
   read → adder → register-file write, and the state-dependent read-address multiplexers;
3. prefetching the next cache line into the instruction buffer (saves ~0.5 clock/instruction).

## 7. Bring-up checklist

1. Build in Diamond 3.11 (as the fx68k README recommends). Confirm BN105 = 0 and
   "Pin Constraint Summary" = all pins locked.
2. Timing report: no unconstrained or failing paths outside `cpu.core` at 85.9 MHz, and the
   core paths within 3 cycles.
3. Program the module out of the Amiga (see the gateware README: USB 5 V back-feed).
4. First boot: Kickstart 1.3 and 3.1 with no expansion, then with fast RAM. The turbomem status
   registers ($F000..) show boot_ok / core_halted / boot_ms as for the fx68k build.
5. If it doesn't boot: `oHALTEDn` low means a double bus fault. Try `CE_DIV=4`, then `ICACHE=0`,
   to separate timing from logic problems.
