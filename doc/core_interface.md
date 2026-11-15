# m68020_core — port interface (contract between RTL and testbench)

```systemverilog
module m68020_core (
  input  logic        clk,
  input  logic        ce,           // clock enable: the core only advances on clk edges with ce=1
  input  logic        rst_n,        // synchronous active-low reset; core starts reset vector fetch after release

  // ---- unified bus (instruction + data), one outstanding transfer ----
  output logic        bus_req,      // held high, with all bus_* outputs stable, until a clock edge where bus_ack or bus_berr is high
  output logic        bus_we,       // 1 = write
  output logic [31:0] bus_addr,     // byte address. Data accesses MAY be misaligned (68020 semantics) - slave/adapter must handle
  output logic [1:0]  bus_size,     // 0 = byte, 1 = word, 2 = long
  output logic [31:0] bus_wdata,    // right-aligned: byte in [7:0], word in [15:0]
  output logic [2:0]  bus_fc,       // function code (1 user data, 2 user prog, 5 super data, 6 super prog, 7 CPU space)
  output logic        bus_lock,     // read-modify-write sequence in progress (TAS/CAS/CAS2)
  input  logic        bus_ack,      // completes the transfer at this clock edge (read data valid in bus_rdata)
  input  logic        bus_berr,     // terminates the transfer with a bus error
  input  logic        bus_avec,     // with bus_ack during interrupt-acknowledge (fc=7): use autovector
  input  logic [31:0] bus_rdata,    // right-aligned like bus_wdata

  input  logic [2:0]  ipl,          // interrupt priority level, active high (0 = none, 7 = NMI), already synchronised

  output logic        reset_out,    // high while RESET instruction drives the external reset
  output logic        halted,       // double bus fault

  // ---- debug / verification ----
  output logic        dbg_retire,   // pulse (one ce period): an instruction, plus any exception/interrupt/trace taken after it, completed
  output logic [31:0] dbg_pc,       // address of the next instruction (its opcode has just been fetched); valid with dbg_retire
  output logic [15:0] dbg_sr,
  output logic [255:0] dbg_d,       // D7..D0, D0 in [31:0]
  output logic [255:0] dbg_a,       // A7..A0, A0 in [31:0]; A7 = currently active stack pointer
  output logic [31:0] dbg_usp, dbg_isp, dbg_msp, dbg_vbr
);
```

Rules
* A transfer completes on a rising edge where `bus_req && (bus_ack || bus_berr)`. The core may present the next
  request in the following cycle (bus_req can stay high across back-to-back transfers). Slaves must not ack when bus_req is low.
* Instruction fetches are long-word (size 2) reads at 4-byte-aligned addresses, fc = 2/6.
* Interrupt acknowledge: read, size byte, fc = 7, addr = 32'hFFFFFFF1 | (level << 1). Slave returns vector in rdata[7:0], or asserts bus_avec with bus_ack.
* Reset sequence: long read of SSP at 0, long read of PC at 4 (fc = 6 for both). D0-D7, A0-A6, USP and MSP are cleared
  at reset (the real 68020 leaves them undefined; clearing makes simulation deterministic).
* dbg_d/dbg_a/dbg_usp/dbg_isp/dbg_msp come from a shadow copy of the register file that only exists with DEBUG=1.
* All inputs (bus_ack, bus_berr, bus_avec, bus_rdata, ipl) are sampled only on clock edges where ce=1.
  A slave must therefore hold bus_ack/bus_berr until an edge with ce=1 (or run with ce tied high).
