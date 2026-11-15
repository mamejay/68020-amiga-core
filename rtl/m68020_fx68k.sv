// =============================================================================
// m68020_fx68k.sv - m68020_core behind an fx68k-compatible port list
//
// Drop-in replacement for the fx68k instance in jbilander/base64_gateware
// (rtl/base64_fx68k/base64_top.sv).  The port names, directions and phase
// enables match fx68k (fredrequin fork, eab[31:1]); the 68000 bus cycles are
// produced on the enPhi1/enPhi2 grid exactly like a real 68000:
//
//   S0 (phi1)  FC, R/W=1, address
//   S2 (phi1)  AS low; read: UDS/LDS low; write: R/W low
//   S3 (phi2)  write data driven
//   S4 (phi1)  write: UDS/LDS low
//   S4->S5     DTACK/BERR sampled on every phi2 (wait states until seen)
//   S6->S7     read data latched, AS/UDS/LDS negated
//
// The core's 32-bit (possibly misaligned) transfers are split into 68000
// byte/word cycles here.  Interrupt acknowledge is a normal FC=7 byte read;
// the vector is taken from D7..D0 (base64 returns 24+level).
// The core runs on a clock enable: CE_DIV = 2 uses enPhi1 (one core clock per
// 68000 clock, 42.6 MHz on base64); CE_DIV = 3 / 4 run it at clk/3 or clk/4 for
// timing closure.  Results are handed to the core on core_ce edges only, so all
// paths into and inside the core are multicycle CE_DIV paths (see doc/integration.md).
//
// Copyright (c) 2026 Jason (ITMD).  Licence: see LICENSE in the project root.
// =============================================================================
/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */

module m68020_fx68k #(
    parameter bit ICACHE = 1'b1,
    parameter bit DEBUG  = 1'b0,
    parameter int CE_DIV = 3
) (
    input  wire        clk,
    input  wire        HALTn,
    input  wire        extReset,
    input  wire        pwrUp,
    input  wire        enPhi1,
    input  wire        enPhi2,

    output reg         eRWn,
    output reg         ASn,
    output reg         LDSn,
    output reg         UDSn,
    output wire        E,
    output wire        VMAn,
    output reg         FC0,
    output reg         FC1,
    output reg         FC2,
    output reg         BGn,
    output wire        oRESETn,
    output wire        oHALTEDn,
    input  wire        DTACKn,
    input  wire        VPAn,
    input  wire        BERRn,
    input  wire        BRn,
    input  wire        BGACKn,
    input  wire        IPL0n,
    input  wire        IPL1n,
    input  wire        IPL2n,
    input  wire [15:0] iEdb,
    output reg  [15:0] oEdb,
    output reg  [31:1] eab,

    // debug / verification (only meaningful with DEBUG=1)
    output wire        dbg_retire,
    output wire [31:0] dbg_pc,
    output wire [15:0] dbg_sr,
    output wire [255:0] dbg_d,
    output wire [255:0] dbg_a,
    output wire [31:0] dbg_usp,
    output wire [31:0] dbg_isp,
    output wire [31:0] dbg_msp,
    output wire [31:0] dbg_vbr
);

assign E    = 1'b0;     // base64 generates E itself
assign VMAn = 1'b1;     // base64 runs all 6800 cycles in its bridge

// -----------------------------------------------------------------------------
// core
// -----------------------------------------------------------------------------
wire        c_req, c_we, c_lock;
wire [31:0] c_addr, c_wdata;
wire [1:0]  c_size;
wire [2:0]  c_fc;
reg         c_ack, c_berr;
reg  [31:0] c_rdata;
wire        c_reset_out, c_halted;

// core clock enable
reg  [2:0]  cediv = 3'd0;
always @(posedge clk) cediv <= (cediv == CE_DIV - 1) ? 3'd0 : cediv + 3'd1;
wire        core_ce = (CE_DIV == 2) ? enPhi1 : (cediv == 3'd0);

reg  [2:0]  ipl_s;
always @(posedge clk) if (enPhi2) ipl_s <= ~{IPL2n, IPL1n, IPL0n};

m68020_core #(.ICACHE(ICACHE), .DEBUG(DEBUG)) core (
    .clk       (clk),
    .ce        (core_ce),
    .rst_n     (!(extReset || pwrUp)),
    .bus_req   (c_req),
    .bus_we    (c_we),
    .bus_addr  (c_addr),
    .bus_size  (c_size),
    .bus_wdata (c_wdata),
    .bus_fc    (c_fc),
    .bus_lock  (c_lock),
    .bus_ack   (c_ack),
    .bus_berr  (c_berr),
    .bus_avec  (1'b0),
    .bus_rdata (c_rdata),
    .ipl       (ipl_s),
    .reset_out (c_reset_out),
    .halted    (c_halted),
    .dbg_retire(dbg_retire),
    .dbg_pc    (dbg_pc),
    .dbg_sr    (dbg_sr),
    .dbg_d     (dbg_d),
    .dbg_a     (dbg_a),
    .dbg_usp   (dbg_usp),
    .dbg_isp   (dbg_isp),
    .dbg_msp   (dbg_msp),
    .dbg_vbr   (dbg_vbr)
);

assign oRESETn  = ~c_reset_out;
assign oHALTEDn = ~c_halted;

// -----------------------------------------------------------------------------
// 68000 bus cycle engine
// -----------------------------------------------------------------------------
localparam [3:0] B_IDLE = 4'd0, B_S0 = 4'd1, B_S1 = 4'd2, B_S2 = 4'd3, B_S3 = 4'd4,
                 B_S4 = 4'd5, B_S5 = 4'd6, B_S6 = 4'd7, B_S7 = 4'd8, B_GRANT = 4'd9, B_EXT = 4'd10;
reg  [3:0]  bst;

reg  [31:0] p_addr;          // address of the current part
reg  [2:0]  p_rem;           // bytes still to transfer (incl. current part)
reg  [31:0] p_wbuf;          // write bytes, next byte in [31:24]
reg  [31:0] p_rbuf;          // read bytes shifted in
reg         p_word;          // current part is a word
reg         p_we;
reg  [1:0]  p_size;          // original size (for read result alignment)
reg         p_berr;

wire [2:0]  sz_bytes  = (c_size == 2'd0) ? 3'd1 : (c_size == 2'd1) ? 3'd2 : 3'd4;
// part type for the current address / remaining count
wire        nxt_word  = !p_addr[0] && (p_rem >= 3'd2);

reg  [1:0]  brs;             // synchronised BR / BGACK
reg  [1:0]  bgacks;
always @(posedge clk) if (enPhi2) begin brs <= {brs[0], BRn}; bgacks <= {bgacks[0], BGACKn}; end
wire br_lo    = !brs[1];
wire bgack_lo = !bgacks[1];

// read data of the finished transfer, right-aligned
function automatic [31:0] ralign(input [31:0] b, input [1:0] s);
    ralign = (s == 2'd0) ? {24'd0, b[7:0]} : (s == 2'd1) ? {16'd0, b[15:0]} : b;
endfunction

// result of the finished transfer, handed to the core only on a core_ce edge so
// that every path into the core is a clean CE_DIV-cycle (multicycle) path
reg         r_done, r_berr;
reg  [31:0] r_data;

always @(posedge clk) begin
    if (core_ce) begin
        if (c_ack || c_berr) begin            // consumed by the core on this edge
            c_ack  <= 1'b0;
            c_berr <= 1'b0;
        end else if (r_done) begin
            c_ack   <= !r_berr;
            c_berr  <= r_berr;
            c_rdata <= r_data;
            r_done  <= 1'b0;
        end
    end

    if (extReset || pwrUp) begin
        bst   <= B_IDLE;
        ASn   <= 1'b1; UDSn <= 1'b1; LDSn <= 1'b1; eRWn <= 1'b1;
        BGn   <= 1'b1;
        {FC2, FC1, FC0} <= 3'b110;
        c_ack <= 1'b0; c_berr <= 1'b0; r_done <= 1'b0;
    end else begin
        case (bst)
        B_IDLE: if (enPhi1) begin
            eRWn <= 1'b1;
            if (br_lo) begin
                BGn <= 1'b0;
                bst <= B_GRANT;
            end else if (c_req && !c_ack && !c_berr && !r_done && HALTn) begin
                // start a transfer: first part
                p_addr <= c_addr;
                p_rem  <= sz_bytes;
                p_we   <= c_we;
                p_size <= c_size;
                p_berr <= 1'b0;
                p_wbuf <= (c_size == 2'd0) ? {c_wdata[7:0], 24'd0} :
                          (c_size == 2'd1) ? {c_wdata[15:0], 16'd0} : c_wdata;
                p_rbuf <= 32'd0;
                {FC2, FC1, FC0} <= c_fc;
                eab  <= c_addr[31:1];
                p_word <= !c_addr[0] && (sz_bytes >= 3'd2);
                bst  <= B_S0;
            end
        end
        B_S0: if (enPhi2) bst <= B_S1;
        B_S1: if (enPhi1) begin
            ASn <= 1'b0;
            if (!p_we) begin
                UDSn <= !(p_word || !p_addr[0]);
                LDSn <= !(p_word ||  p_addr[0]);
            end else
                eRWn <= 1'b0;
            bst <= B_S2;
        end
        B_S2: if (enPhi2) begin
            if (p_we)
                oEdb <= p_word ? p_wbuf[31:16] : {p_wbuf[31:24], p_wbuf[31:24]};
            bst <= B_S3;
        end
        B_S3: if (enPhi1) begin
            if (p_we) begin
                UDSn <= !(p_word || !p_addr[0]);
                LDSn <= !(p_word ||  p_addr[0]);
            end
            bst <= B_S4;
        end
        B_S4: if (enPhi2) begin
            if (!BERRn) begin p_berr <= 1'b1; bst <= B_S5; end
            else if (!DTACKn) bst <= B_S5;
        end
        B_S5: if (enPhi1) bst <= B_S6;
        B_S6: if (enPhi2) begin
            // latch data, end the cycle
            if (!p_we) begin
                if (p_word) p_rbuf <= {p_rbuf[15:0], iEdb};
                else        p_rbuf <= {p_rbuf[23:0], p_addr[0] ? iEdb[7:0] : iEdb[15:8]};
            end
            ASn <= 1'b1; UDSn <= 1'b1; LDSn <= 1'b1;
            if (p_berr || p_rem == (p_word ? 3'd2 : 3'd1)) begin
                // transfer complete: queue the result for the core
                r_done <= 1'b1;
                r_berr <= p_berr;
                r_data <= ralign(p_word ? {p_rbuf[15:0], iEdb}
                                        : {p_rbuf[23:0], p_addr[0] ? iEdb[7:0] : iEdb[15:8]}, p_size);
                bst <= B_S7;
                p_rem <= 3'd0;
            end else begin
                p_rem  <= p_rem - (p_word ? 3'd2 : 3'd1);
                p_addr <= p_addr + (p_word ? 32'd2 : 32'd1);
                p_wbuf <= p_word ? {p_wbuf[15:0], 16'd0} : {p_wbuf[23:0], 8'd0};
                bst <= B_S7;
            end
        end
        B_S7: if (enPhi1) begin
            eRWn <= 1'b1;
            if (p_rem == 3'd0) bst <= B_IDLE;
            else begin
                // next part of a split transfer starts right away (S0)
                eab    <= p_addr[31:1];
                p_word <= nxt_word;
                bst    <= B_S0;
            end
        end
        B_GRANT: if (enPhi1) begin
            // bus granted to another master: wait for BGACK, or BR withdrawn
            if (bgack_lo) begin BGn <= 1'b1; bst <= B_EXT; end
            else if (!br_lo) begin BGn <= 1'b1; bst <= B_IDLE; end
        end
        B_EXT: if (enPhi1) begin
            if (!bgack_lo) bst <= B_IDLE;
        end
        default: bst <= B_IDLE;
        endcase
    end
end

endmodule
