// Stub with the m68020_core port list (doc/core_interface.md). Does nothing useful;
// only exists so the lockstep harness can be compiled before the real RTL exists.
module m68020_core (
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
  assign bus_req = 1'b0; assign bus_we = 1'b0; assign bus_addr = '0; assign bus_size = 2'd2;
  assign bus_wdata = '0; assign bus_fc = 3'd6; assign bus_lock = 1'b0;
  assign reset_out = 1'b0; assign halted = 1'b0;
  assign dbg_retire = 1'b0; assign dbg_pc = '0; assign dbg_sr = 16'h2700;
  assign dbg_d = '0; assign dbg_a = '0; assign dbg_usp = '0; assign dbg_isp = '0; assign dbg_msp = '0; assign dbg_vbr = '0;
  wire _unused = &{1'b0, clk, rst_n, bus_ack, bus_berr, bus_avec, bus_rdata, ipl};
endmodule
