# Changes to Musashi in this copy (verification reference model only)

Musashi (Karl Stenerud, MIT-style licence, see readme.txt) is used as the 68020 golden model in the
lockstep harness. These changes make it match documented 68020 behaviour; each was found by a
lockstep mismatch and checked against the M68000 Family Programmer's Reference Manual / MC68020 UM
(and WinUAE's hardware-validated behaviour where the manuals are vague):

| Area | Stock Musashi | Changed to |
|---|---|---|
| cpGEN/cpBcc/cpDBcc/cpScc/cpTRAPcc, PMMU ops | executed as NOPs | F-line exception (bare 68020) |
| MOVEM An,-(An) | stores initial An (68000) | stores An - size (68020) |
| PACK -(Ax),-(Ay) | first byte read = high byte | first byte read (higher address) = low byte |
| UNPK -(Ax),-(Ay) | high byte written first | low byte written first (big-endian word) |
| CHK2/CMP2 .B/.W | bounds zero-extended, An compared on low byte | bounds sign-extended, An compared full 32 bits |
| DIVU/DIVS overflow, divide by zero | C flag not cleared | C cleared |
| DIVS.W / DIVS.L 0x80000000 / -1 | host-crash workaround, no overflow | overflow (V=1, registers unchanged) |
| DIVS.L 32-bit form (64-bit host path) | no overflow check | overflow checked |
| CAS2.W failure | compare register sign-extended when Rn is An; Dc2 wins | low word only; Dc1 wins (PRM) |
| Trace after TRAP/CHK/CHK2/TRAPV/TRAPcc/DIV0 | pending trace cancelled | trace taken after the trap (68020 UM) |
| RTE frame formats $9/$A/$B | format error | restore SR/PC, pop frame (restart model) |
| Format error | stacks PC after the RTE | stacks PC of the RTE |
| NBCD <ea> with result 0 | no write | writes 0 (read-modify-write) |
| MOVE Dn,SR | no T0 (change-of-flow) trace | traced like MOVE <ea>,SR |
| Bit field EA with offset -2^31 | integer overflow in address | arithmetic shift |
| m68k_set_irq | recognised at next step | recognised at this instruction boundary (harness hook) |
