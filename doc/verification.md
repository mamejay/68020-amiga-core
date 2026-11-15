# Verification

## Method

`sim/lockstep` runs the RTL (Verilator) and Musashi in 68020 mode side by side. After every
instruction it compares D0-D7, A0-A7, PC, SR, USP/ISP/MSP, VBR and the bytes each model wrote
during that instruction (compared as byte images, so how a transfer is split doesn't matter).
Exception processing, including trace and interrupts, counts as part of the instruction that
caused it.

* **random mode.** Each test runs a preamble that loads every register (random data, addresses,
  SR with random M bit and flags), then one random 16-bit opcode with random extension words at a
  random PC, plus N more instructions. Memory is sparse: unwritten bytes read as a hash of the
  address. Vectors point to distinct handlers, so the exception taken is visible from the PC.
* **prog mode.** Runs an assembled program (`sim/tests/*.S`) to its STOP, comparing every
  instruction. A byte write to `$DFF0F0` sets the interrupt level for both models.
* **bus model.** 0-3 random wait states per transfer (or `--nowait`). Protocol checks catch bus
  outputs changing while a transfer is pending, and requests dropped early.
* **lockstep_fx.** The same harness with the core inside `m68020_fx68k`. A small 68000-bus slave
  converts every AS/UDS/LDS cycle back into harness transfers, so the 68000 cycle engine, the
  splitting of long/misaligned accesses, IACK and the CE_DIV handshake are all in the loop.

Undefined flags are masked: DIVx on overflow and divide by zero (N, Z, V), ABCD/SBCD/NBCD (N, V),
CHK (Z, V, C), CHK2/CMP2 (N, V).

## Directed tests (`sim/tests`)

| test | covers |
|---|---|
| arith | ADD/SUB/X forms, logic, shifts/rotates, BCD, MUL/DIV basics |
| addrmodes | every EA mode incl. full-format memory indirect, PC-relative, scale, misaligned data |
| movem | MOVEM all directions/sizes incl. -(An) with An in the list, MOVEP |
| bitfield | BFTST/BFEXTU/BFEXTS/BFFFO/BFCHG/BFCLR/BFSET/BFINS, register + memory, 1-5 byte spans |
| muldiv | MULx.L 32/64, DIVx.L 32/64, DIVUL/DIVSL, overflow, divide by zero |
| cas | CAS, CAS2 (equal / not equal), TAS |
| exceptions | TRAP, ILLEGAL, line A/F, CHK/CHK2/CMP2, TRAPV/TRAPcc, RTR/RTD/LINK.L/BSR.L, MOVEC, MOVE USP, VBR, M bit, privilege, trace, format error |
| interrupts | masking, level 5 over mask 4, NMI edge trigger, M=1 master frame + throwaway frame, user-mode interrupt |
| bench | integer loop with the I-cache on (performance) |

## Results (v0.4)

* All directed programs pass on the native bus and through the wrapper (CE_DIV 2 and 3).
* Random: over 1.3 million single-instruction tests (plus 1-3 follow-on instructions each)
  across the development versions, the last 200k+ on the current RTL, both harness variants.
  No open mismatches.
* Each RTL bug found along the way had a matching random or directed test that now passes:
  NOT.B Z flag, NBCD zero result, ASR fill for counts > 32, ASL V for counts ≥ width,
  LINK A7, MOVES An,(An)+, BFFFO register offset, privilege-vs-illegal priority, T0 cases.

Musashi itself needed corrections where it disagrees with the Motorola manuals; see
`sim/musashi/MODIFICATIONS.md`. Where the manuals are silent, Musashi's choice was kept so the
RTL matches it (e.g. UNLK A7, N flag after NBCD of zero).

## Not covered yet

* Bus errors (`BERRn`) and the format $A frame contents. The logic is there, but there are no
  tests with a faulting slave.
* Odd-PC address errors (Musashi doesn't model them; those random tests are counted as skipped).
* STOP/RESET as random test instructions (directed only), BKPT/CALLM/RTM (illegal by design).
* Bus arbitration (BR/BG/BGACK) and HALT in the wrapper.
* Real hardware. The next step is a Diamond build and a Base64 board.
