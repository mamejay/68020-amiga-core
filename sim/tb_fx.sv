`ifndef CE_DIV
`define CE_DIV 2
`endif
// Simulation top for the fx68k-compatible wrapper: m68020_fx68k drives a 68000 bus, and a small
// 68000-bus slave adapter turns every byte/word cycle back into the harness's req/ack transfer.
// Same port list as tb.sv, so lockstep.cpp is unchanged.
/* verilator lint_off UNUSEDSIGNAL */
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
  // phase enables: phi1 / phi2 alternate every clk (as in base64_top)
  logic ph = 1'b0;
  always_ff @(posedge clk) ph <= ~ph;
  wire enPhi1 = ~ph, enPhi2 = ph;

  wire        eRWn, ASn, LDSn, UDSn, FC0, FC1, FC2, BGn, oRESETn, oHALTEDn, E, VMAn;
  wire [15:0] oEdb;
  wire [31:1] eab;
  logic       DTACKn = 1'b1, BERRn = 1'b1;
  logic [15:0] iEdb = 16'hFFFF;

  m68020_fx68k #(.DEBUG(1'b1), .CE_DIV(`CE_DIV)) u_cpu (
    .clk(clk), .HALTn(1'b1), .extReset(!rst_n), .pwrUp(1'b0), .enPhi1(enPhi1), .enPhi2(enPhi2),
    .eRWn(eRWn), .ASn(ASn), .LDSn(LDSn), .UDSn(UDSn), .E(E), .VMAn(VMAn),
    .FC0(FC0), .FC1(FC1), .FC2(FC2), .BGn(BGn), .oRESETn(oRESETn), .oHALTEDn(oHALTEDn),
    .DTACKn(DTACKn), .VPAn(1'b1), .BERRn(BERRn), .BRn(1'b1), .BGACKn(1'b1),
    .IPL0n(~ipl[0]), .IPL1n(~ipl[1]), .IPL2n(~ipl[2]),
    .iEdb(iEdb), .oEdb(oEdb), .eab(eab),
    .dbg_retire(dbg_retire), .dbg_pc(dbg_pc), .dbg_sr(dbg_sr), .dbg_d(dbg_d), .dbg_a(dbg_a),
    .dbg_usp(dbg_usp), .dbg_isp(dbg_isp), .dbg_msp(dbg_msp), .dbg_vbr(dbg_vbr)
  );

  assign reset_out = ~oRESETn;
  assign halted    = ~oHALTEDn;
  assign bus_lock  = 1'b0;

  // 68000 bus slave adapter
  logic [1:0] ad = 2'd0;
  wire        ds   = !UDSn || !LDSn;
  wire        word = !UDSn && !LDSn;
  wire [2:0]  fc   = {FC2, FC1, FC0};
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      ad <= 2'd0; bus_req <= 1'b0; DTACKn <= 1'b1; BERRn <= 1'b1;
    end else case (ad)
      2'd0: if (!ASn && ds) begin
        if (fc == 3'd7) begin                       // interrupt acknowledge: autovector number
          iEdb   <= {8'h00, 8'd24 + {5'd0, eab[3:1]}};
          DTACKn <= 1'b0;
          ad     <= 2'd2;
        end else begin
          bus_req   <= 1'b1;
          bus_we    <= !eRWn;
          bus_addr  <= {eab, UDSn};                 // LDS only -> odd byte
          bus_size  <= word ? 2'd1 : 2'd0;
          bus_wdata <= word ? {16'd0, oEdb} : {24'd0, UDSn ? oEdb[7:0] : oEdb[15:8]};
          bus_fc    <= fc;
          ad        <= 2'd1;
        end
      end
      2'd1: if (bus_ack || bus_berr) begin
        bus_req <= 1'b0;
        iEdb    <= word ? bus_rdata[15:0] : {bus_rdata[7:0], bus_rdata[7:0]};
        if (bus_berr) BERRn <= 1'b0; else DTACKn <= 1'b0;
        ad      <= 2'd2;
      end
      default: if (ASn) begin DTACKn <= 1'b1; BERRn <= 1'b1; ad <= 2'd0; end
    endcase
  end
endmodule
