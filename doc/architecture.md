# m68020_core architecture

A multi-cycle, microcode-free FSM implementation of the 68020 instruction set. Each instruction
passes through a short sequence of states. Shared subroutines handle effective addresses, bus
transfers and exceptions, and a few datapath blocks are shared by every instruction.

## Datapath

| Block | Notes |
|---|---|
| Register file | D0-D7, A0-A6, USP, ISP, MSP in distributed RAM (three byte-lane LUTRAMs, so sized writes need no read-modify-write). One write port, registered and applied on the next clock with per-lane bypass to the reads. Read ports: A, B, C (per-state address mux) and SP (follows SR.S/M). |
| ALU | ADD/SUB/CMP/NEG (+X forms), logic, MOVE/TST/CLR/TAS, ADDA/SUBA, Scc, ABCD/SBCD/NBCD. Operands are aligned to bit 31 so one 33-bit adder gives byte/word/long carries. Shared with CAS, CAS2, CHK and CHK2; their compare result is registered (`alu_q`) before it steers control flow. |
| Shifter | one 66-bit right funnel shifter: LSR/ASR directly, LSL/ASL via bit reversal, ROL/ROR via a replicated operand. ROXL/ROXR iterate one bit per clock. Also does the bit-field rotations. |
| Multiplier | one 16x16 DSP multiply. 32x32→64 takes four partial products plus two signed corrections on a shared 64-bit adder (7 clocks). 16x16 is 2-4 clocks. |
| Divider | restoring, 1 bit per clock on magnitudes (32 iterations). The overflow pre-check uses the same subtractor; sign handling uses the shared 64-bit adder. |
| Bit fields | 40-bit window (up to 5 bytes). Memory operands use the minimum byte/word/long accesses. Register operands are rotated through the shifter. |

## Control

* **Fetch.** The opcode and extension words come from a one-longword prefetch buffer (flops).
  On a buffer miss the 64-entry instruction cache refills it (1 clock); otherwise a long-word bus
  fetch is made. The cache is tagged with {S, A31..A8}. Data writes by the core invalidate the
  matching buffer and cache lines (self-modifying code safe, unlike a real 68020).
* **Decode.** One `casez` on the opcode sets up a generic operand flow or jumps to an
  instruction-specific sequence. Invalid EA modes and reserved encodings raise illegal
  instruction before the privilege check.
* **Generic flow.** `G_SRC → [EA] → G_SRC_RD → G_DST → [EA] → G_DST_RD → G_EXEC → G_WB`.
  Fast paths skip most of these: register operands are read in one clock (source and destination
  together), (An)/(An)+/-(An) start the bus read from the EA state, immediates hand over directly,
  and register or memory results are written from `G_EXEC`.
* **EA engine.** A subroutine (`ea_ret`) for all modes. The brief format includes the 68020 scale.
  The full format covers base/index suppress, word/long base displacement, and memory indirect
  pre/post-indexed with null/word/long outer displacement. An indexed address takes one extra
  clock (index latched first, for timing).
* **Bus subroutine.** `S_MEM` (`mem_ret`). One outstanding transfer, misaligned data allowed;
  the wrapper or slave splits it.
* **Completion.** An instruction ends straight into the next opcode fetch. Interrupts are
  recognised at that boundary; trace (T1, or T0 on change of flow) goes through `S_END`.

## Exceptions

| Kind | Vector | Frame | Stacked PC |
|---|---|---|---|
| Bus error | 2 | $A (16 words: SSW, fault address, data output buffer) | instruction address (RTE restarts the instruction) |
| Address error (odd PC on fetch) | 3 | $A | the odd PC |
| Illegal, BKPT, CALLM/RTM | 4 | $0 | instruction |
| Zero divide, CHK/CHK2, TRAPcc/TRAPV, trace | 5, 6, 7, 9 | $2 (+ instruction address) | next instruction |
| Privilege, line A, line F | 8, 10, 11 | $0 | instruction |
| Format error (bad RTE frame) | 14 | $0 | the RTE |
| TRAP #n | 32-47 | $0 | next instruction |
| Interrupt | autovector 25-31 or from IACK | $0 on the active stack, then $1 throwaway on ISP if M was set | next instruction |

* Trace after a trap-class exception (TRAP, CHK, CHK2, TRAPV, TRAPcc, DIV0) is taken before the
  handler's first instruction. Other exceptions cancel a pending trace.
* RTE accepts formats $0, $1 (throwaway: continue on the new stack), $2, $9, $A, $B. Any other
  format raises a format error.
* A bus error during exception processing halts the core (`halted`), as a double bus fault does.

## Differences from a real MC68020

* Not cycle accurate. Timing is whatever the FSM takes; no 68020 pipeline effects.
* CALLM/RTM take the illegal-instruction trap (they were dropped from the 68030 onwards and
  AmigaOS does not use them). BKPT does not run a breakpoint-acknowledge cycle.
* Bus error and address error use a simplified format $A frame. RTE of $A/$B frames **restarts**
  the instruction rather than resuming the faulted bus cycle, so a faulting (An)+ / -(An)
  instruction would apply its register update twice. Acceptable for an Amiga, where bus errors
  are fatal anyway.
* Coprocessor instructions don't run the coprocessor protocol: they take the F-line trap
  directly (what software sees on a 68020 without an FPU).
* The instruction cache is invalidated by the core's own writes, and the CACR freeze/clear bits
  work, but DMA writes are invisible to it. As on a real 68020, software must flush after
  loading code by DMA. AmigaOS does this.
* RMW cycles (TAS, CAS, CAS2) raise `bus_lock`, but the 68000-bus wrapper does not hold AS
  across the read and write. The Amiga chipset doesn't support RMW on chip RAM anyway.
