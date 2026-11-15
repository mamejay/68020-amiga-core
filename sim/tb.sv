// Simulation top: passes the core ports straight through; the bus slave and memory live in lockstep.cpp.
module tb (
  input  logic        clk,
  input  logic        rst_n,
  output logic        bus_req,
  output logic        bus_we,
  output logic [31:0] bus_addr,
  output logic [1:0]  bus_size,
  output logic [31:0] bus_wdata,
  output logic [2:0]  bus_fc,
  output logic        bus_lock,
  input  logic        bus_ack,
  input  logic        bus_berr,
  input  logic        bus_avec,
  input  logic [31:0] bus_rdata,
  input  logic [2:0]  ipl,
  output logic        reset_out,
  output logic        halted,
  output logic        dbg_retire,
  output logic [31:0] dbg_pc,
  output logic [15:0] dbg_sr,
  output logic [255:0] dbg_d,
  output logic [255:0] dbg_a,
  output logic [31:0] dbg_usp, dbg_isp, dbg_msp, dbg_vbr
);
  m68020_core u_core (.*);
endmodule
