# Lockstep verification harness

`m68020_core` (Verilator) runs side by side with Musashi (68020 mode). After every retired
instruction the harness compares D0-D7, A0-A7, PC, SR, USP/ISP/MSP, VBR and the bytes written to
memory during that instruction.

## Build / run
    make                      # builds ./lockstep (= obj_dir/Vtb); uses ../rtl/m68020_core.sv (+ other ../rtl/*.sv) if present, else stub_core.sv
    make TRACE=1              # enables --vcd
    make tests                # tests/*.S -> tests/*.bin (m68k-linux-gnu-as -m68020, linked at 0)
    make check                # all directed tests + 2000 random tests

    ./lockstep random --seed 1 --count 10000 [--post 1] [--show 20] [--nowait] [--timeout 20000] [--vcd x.vcd] [-v]
    ./lockstep prog tests/arith.bin [--max 200000] [--nowait] [-v]
    ./lockstep --musashi-only [-v]                      # Musashi self-check (preamble + exception frame)
    ./lockstep prog tests/x.bin --musashi-only [-v]     # run a test program on Musashi alone

## Memory / bus model
* Sparse byte memory. Unwritten bytes read as `hash(address, test seed)` (random mode) or 0 (prog mode).
  RTL and Musashi have separate copies preloaded identically; per-instruction write logs are compared
  as byte images (last write wins), so how the core splits a misaligned write does not matter.
* RTL slave: 0-3 random wait states per transfer (`--nowait` = 0, ack in the same cycle bus_req is first seen),
  any alignment, sizes byte/word/long, data right-aligned. Reads with fc=7 return `bus_avec=1`.
  Protocol errors flagged: bus outputs changing while a transfer is pending, bus_req dropped before ack, size 3.

## Random mode
Per test: random D0-D7/A0-A6 (mix of small values, sign-extended bytes, addresses, full random),
USP random, ISP in 0x200000.., MSP in 0x300000.., SR = S | IPL7 | random M | random XNZVC, VBR = 0.
Memory: vectors 2..255 -> handler at `0xF00000 + v*16` (NOPs, so the exception taken is visible from the PC),
reset vector -> preamble at 0x400 that loads all registers with MOVE/MOVEA/MOVE USP/MOVEC MSP,ISP, then
`MOVE #sr,SR`, `JMP test_pc`. Test = random opcode word + 10 random extension words at a random even PC.
Every preamble instruction is also compared. Then the test instruction plus `--post` instructions.

Undefined flags masked: DIVx (V set: N,Z; divide-by-zero: N,Z,V, also in stacked SR), ABCD/SBCD/NBCD (N,V),
CHK (trap: Z,V,C incl. stacked SR; no trap: N,Z,V,C), CHK2/CMP2 (N,V). CCR is ignored until the preamble's
`MOVE #sr,SR` (prog mode: for the first instruction only - start programs with `move #0x2700,sr`).

Skipped (counted under "skipped"): STOP, RESET, BKPT, CALLM, RTM as test instruction; tests where Musashi lands
on an odd PC (Musashi does not model the 68020 odd-PC address error).

## Musashi changes (sim/musashi, generated m68kops.c)
* m68kconf.h: trace emulation ON, instruction hook -> `lockstep_instr_hook`, PMMU OFF.
* m68k_in.c: on 68020, cpGEN/cpBcc/cpDBcc/cpScc/cpTRAPcc and PMMU (F-line) opcodes take the F-line exception
  (stock Musashi silently treats them as NOPs). Models a bare 68020 with no 68881/68851.
* m68k_in.c: MOVEM -(An) with An in the register list stores initial An minus the operand size on 020+
  (M68000PRM); stock Musashi stores the initial value (68000 behaviour).

## Directed tests (tests/*.S)
arith, addrmodes (incl. full-format memory indirect, PC-relative, misaligned), movem (+movep), bitfield,
muldiv (.L 32/64, overflow, divide by zero), cas (CAS/CAS2/TAS), exceptions (TRAP, ILLEGAL, line A/F,
CHK/CHK2/CMP2, TRAPV/TRAPcc, RTR/RTD/LINK.L, MOVEC, MSP/M bit, privilege violation, trace via RTE,
format error, VBR relocation). A program ends when the next instruction is STOP (= PASS).
