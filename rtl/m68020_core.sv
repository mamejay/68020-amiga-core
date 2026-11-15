// =============================================================================
// m68020_core.sv - 68020-compatible CPU core (ISA level, not cycle accurate)
//
// Written for the Base64 68000-socket accelerator (jbilander/Base64) as an
// original implementation. Architecture:
//   * multi-cycle FSM with shared effective-address and bus subroutines
//   * unified 32-bit request/ack bus, misaligned data allowed (see doc/core_interface.md)
//   * 64-entry x 32-bit direct-mapped instruction cache (68020 compatible CACR)
//     plus a one-longword prefetch buffer
//   * full 68020 user + supervisor ISA except CALLM/RTM and coprocessor
//     instructions (F-line trap), bus errors restart the instruction
//
// Copyright (c) 2026 Jason (ITMD).  Licence: see LICENSE in the project root.
// =============================================================================
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
/* verilator lint_off CASEINCOMPLETE */
/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off BLKSEQ */
/* verilator lint_off CASEOVERLAP */
/* verilator lint_off UNUSEDPARAM */

module m68020_core #(
    parameter int RESET_CYCLES = 1024,   // length of RESET instruction pulse
    parameter bit ICACHE       = 1'b1    // implement 64-entry instruction cache
) (
    input  logic         clk,
    input  logic         rst_n,

    output logic         bus_req,
    output logic         bus_we,
    output logic [31:0]  bus_addr,
    output logic [1:0]   bus_size,
    output logic [31:0]  bus_wdata,
    output logic [2:0]   bus_fc,
    output logic         bus_lock,
    input  logic         bus_ack,
    input  logic         bus_berr,
    input  logic         bus_avec,
    input  logic [31:0]  bus_rdata,

    input  logic [2:0]   ipl,

    output logic         reset_out,
    output logic         halted,

    output logic         dbg_retire,
    output logic [31:0]  dbg_pc,
    output logic [15:0]  dbg_sr,
    output logic [255:0] dbg_d,
    output logic [255:0] dbg_a,
    output logic [31:0]  dbg_usp,
    output logic [31:0]  dbg_isp,
    output logic [31:0]  dbg_msp,
    output logic [31:0]  dbg_vbr
);

// -----------------------------------------------------------------------------
// constants
// -----------------------------------------------------------------------------
localparam [1:0] SZ_B = 2'd0, SZ_W = 2'd1, SZ_L = 2'd2;
localparam [1:0] K_D = 2'd0, K_A = 2'd1, K_M = 2'd2, K_I = 2'd3;   // EA kinds
localparam [15:0] SR_MASK = 16'hF71F;

// EA class bits (one-hot index)
localparam int C_DN=0, C_AN=1, C_IND=2, C_PI=3, C_PD=4, C_D16=5, C_IDX=6,
               C_ABSW=7, C_ABSL=8, C_PCD=9, C_PCX=10, C_IMM=11;
localparam [11:0] M_ALL      = 12'hFFF;
localparam [11:0] M_DATA     = 12'hFFD;
localparam [11:0] M_MEM      = 12'hFFC;
localparam [11:0] M_CTRL     = 12'h7E4;
localparam [11:0] M_ALT      = 12'h1FF;
localparam [11:0] M_DALT     = 12'h1FD;
localparam [11:0] M_MALT     = 12'h1FC;
localparam [11:0] M_CALT     = 12'h1E4;
localparam [11:0] M_DATA_NI  = 12'h7FD;   // data, no immediate (CMPI, BTST #)

// ALU ops
localparam [4:0] A_ADD=0, A_ADDX=1, A_SUB=2, A_SUBX=3, A_CMP=4, A_NEG=5, A_NEGX=6,
                 A_AND=7, A_OR=8, A_EOR=9, A_NOT=10, A_MOVE=11, A_CLR=12, A_TST=13,
                 A_ADDA=14, A_SUBA=15, A_PASS=16, A_SCC=17, A_ABCD=18, A_SBCD=19,
                 A_NBCD=20, A_TAS=21, A_FROMSR=22, A_FROMCCR=23, A_TOCCR=24, A_TOSR=25;

// -----------------------------------------------------------------------------
// FSM states
// -----------------------------------------------------------------------------
typedef enum logic [7:0] {
    S_RESET, S_RST1, S_RST2, S_RST3,
    S_FETCH, S_DECODE, S_END, S_RETIRE, S_STOPPED, S_HALT,
    S_MEM, S_IFETCH,
    // effective address engine
    E_START, E_D16, E_ABSW, E_ABSL1, E_ABSL2, E_IMM1, E_IMM2, E_IDX, E_BDW, E_BDL1, E_BDL2,
    E_FULL, E_ODW, E_ODL1, E_ODL2, E_MI, E_MI2,
    // generic ALU flow
    G_SRC, G_SRC_RD, G_SRC_RD2, G_DST, G_DST_RD, G_DST_RD2, G_EXEC, G_WB,
    // misc
    S_IMMSR, S_IMMSR2, S_TOSR, S_MOVEM0, S_MOVEM1, S_MOVEM_L, S_MOVEM_LD, S_MOVEM_PD,
    S_MOVEP0, S_MOVEP1, S_MOVEP2,
    S_BCC0, S_BCCW, S_BCCL1, S_BCCL2, S_BCCGO,
    S_DBCC, S_TRAPCC, S_TRAPCC2,
    S_JEA, S_JSR2, S_LEA, S_PEA,
    S_LINK0, S_LINKL, S_LINK1, S_LINK2, S_UNLK1,
    S_RTS1, S_RTD0, S_RTD1, S_RTR1, S_RTR2,
    S_RTE0, S_RTE1, S_RTE2, S_RTE3, S_RTE4,
    S_CHK, S_CHK2X, S_CHK2E, S_CHK2R1, S_CHK2R2,
    S_MULW, S_MULLX, S_MULL, S_MUL_LOOP, S_MUL_FIN,
    S_DIVW, S_DIVLX, S_DIVL, S_DIV_LOOP, S_DIV_FIN,
    S_SHIFT, S_ROX, S_SHIFTM,
    S_BITX, S_BITD, S_BIT,
    S_BFX, S_BFE, S_BF0, S_BF1, S_BF2, S_BF3, S_BF4,
    S_CASX, S_CASE, S_CAS, S_CAS2X1, S_CAS2X2, S_CAS2R1, S_CAS2R2, S_CAS2C, S_CAS2W1, S_CAS2W2,
    S_PACKX, S_PACKM1, S_PACKM2, S_PACKM3, S_UNPKM1, S_UNPKM2, S_UNPKM3, S_UNPKM4,
    S_MOVECX, S_MOVESX, S_MOVESE, S_MOVES1, S_MOVES2,
    S_STOP0, S_RESETI,
    S_INT0, S_INT1,
    X_START, X_F2, X_FA, X_FV, X_PC, X_SR, X_VEC, X_VEC2, X_T1, X_T2, X_T3, X_DONE
} state_t;

state_t state, ea_ret, mem_ret, if_ret, g_after_src, g_after_dst;

// -----------------------------------------------------------------------------
// architectural registers
// -----------------------------------------------------------------------------
logic [31:0] D [0:7];
logic [31:0] A [0:7];   // A[7] unused (A7 = active stack pointer)
logic [31:0] usp, isp, msp, vbr, cacr, caar;
logic [2:0]  sfc, dfc;
logic [15:0] sr;
logic [31:0] PC, PCi;

wire [31:0] sp = !sr[13] ? usp : (sr[12] ? msp : isp);

wire [31:0] R [0:15];
genvar gi;
generate
    for (gi = 0; gi < 8; gi++) begin : g_r
        assign R[gi] = D[gi];
        if (gi < 7) begin : g_a
            assign R[gi+8] = A[gi];
        end
    end
endgenerate
assign R[15] = sp;

// -----------------------------------------------------------------------------
// helper functions
// -----------------------------------------------------------------------------
function automatic [31:0] sx8(input [7:0] v);   sx8 = {{24{v[7]}}, v};   endfunction
function automatic [31:0] sx16(input [15:0] v); sx16 = {{16{v[15]}}, v}; endfunction
function automatic [31:0] sxs(input [31:0] v, input [1:0] s);
    sxs = s == SZ_B ? sx8(v[7:0]) : s == SZ_W ? sx16(v[15:0]) : v;
endfunction
function automatic [31:0] zxs(input [31:0] v, input [1:0] s);
    zxs = s == SZ_B ? {24'd0, v[7:0]} : s == SZ_W ? {16'd0, v[15:0]} : v;
endfunction
function automatic [31:0] merge(input [31:0] old, input [31:0] nv, input [1:0] s);
    merge = s == SZ_B ? {old[31:8], nv[7:0]} : s == SZ_W ? {old[31:16], nv[15:0]} : nv;
endfunction
function automatic msbs(input [31:0] v, input [1:0] s);
    msbs = s == SZ_B ? v[7] : s == SZ_W ? v[15] : v[31];
endfunction
function automatic [2:0] szbytes(input [1:0] s);
    szbytes = s == SZ_B ? 3'd1 : s == SZ_W ? 3'd2 : 3'd4;
endfunction

function automatic cond(input [3:0] cc, input [4:0] f);
    logic n, z, v, c;
    n = f[3]; z = f[2]; v = f[1]; c = f[0];
    case (cc)
        4'h0: cond = 1'b1;
        4'h1: cond = 1'b0;
        4'h2: cond = !c && !z;
        4'h3: cond = c || z;
        4'h4: cond = !c;
        4'h5: cond = c;
        4'h6: cond = !z;
        4'h7: cond = z;
        4'h8: cond = !v;
        4'h9: cond = v;
        4'hA: cond = !n;
        4'hB: cond = n;
        4'hC: cond = n == v;
        4'hD: cond = n != v;
        4'hE: cond = !z && (n == v);
        default: cond = z || (n != v);
    endcase
endfunction

// one-hot EA class of (mode, reg)
function automatic [11:0] eacls(input [2:0] m, input [2:0] r);
    eacls = 12'd0;
    case (m)
        3'd0: eacls[C_DN] = 1'b1;
        3'd1: eacls[C_AN] = 1'b1;
        3'd2: eacls[C_IND] = 1'b1;
        3'd3: eacls[C_PI] = 1'b1;
        3'd4: eacls[C_PD] = 1'b1;
        3'd5: eacls[C_D16] = 1'b1;
        3'd6: eacls[C_IDX] = 1'b1;
        default: case (r)
            3'd0: eacls[C_ABSW] = 1'b1;
            3'd1: eacls[C_ABSL] = 1'b1;
            3'd2: eacls[C_PCD] = 1'b1;
            3'd3: eacls[C_PCX] = 1'b1;
            3'd4: eacls[C_IMM] = 1'b1;
            default: ;
        endcase
    endcase
endfunction
function automatic eaok(input [2:0] m, input [2:0] r, input [11:0] msk);
    eaok = |(eacls(m, r) & msk);
endfunction

// lowest set bit index of 16-bit mask
function automatic [3:0] lsb16(input [15:0] m);
    lsb16 = 4'd0;
    for (int i = 15; i >= 0; i--) if (m[i]) lsb16 = i[3:0];
endfunction

// -----------------------------------------------------------------------------
// ALU:  returns {flags_changed, ccr_new[4:0], result[31:0]}
// -----------------------------------------------------------------------------
function automatic [37:0] alu(input [4:0] op, input [1:0] s, input [31:0] sv, input [31:0] dv,
                              input [4:0] ccr, input cc_true);
    logic [4:0]  sh;
    logic [31:0] sa, da, ra, r;
    logic [32:0] t;
    logic        x, n, z, v, c, chg;
    logic [9:0]  b;     // BCD intermediate
    logic        bc;
    sh = (s == SZ_B) ? 5'd24 : (s == SZ_W) ? 5'd16 : 5'd0;
    sa = sv << sh;
    da = dv << sh;
    x = ccr[4]; n = ccr[3]; z = ccr[2]; v = ccr[1]; c = ccr[0];
    chg = 1'b1;
    ra = 32'd0; r = 32'd0;
    case (op)
        A_ADD, A_ADDX: begin
            t = {1'b0, da} + {1'b0, sa} + ((op == A_ADDX && x) ? (33'd1 << sh) : 33'd0);
            ra = t[31:0];
            c = t[32]; x = c;
            v = (sa[31] == da[31]) && (ra[31] != da[31]);
            n = ra[31];
            z = (op == A_ADDX) ? (z && ra == 32'd0) : (ra == 32'd0);
            r = ra >> sh;
        end
        A_SUB, A_SUBX, A_CMP, A_NEG, A_NEGX: begin
            if (op == A_NEG || op == A_NEGX) begin sa = da; da = 32'd0; end
            t = {1'b0, da} - {1'b0, sa} - (((op == A_SUBX || op == A_NEGX) && x) ? (33'd1 << sh) : 33'd0);
            ra = t[31:0];
            c = t[32];
            if (op != A_CMP) x = c;
            v = (sa[31] != da[31]) && (ra[31] != da[31]);
            n = ra[31];
            z = (op == A_SUBX || op == A_NEGX) ? (z && ra == 32'd0) : (ra == 32'd0);
            r = ra >> sh;
        end
        A_AND, A_OR, A_EOR, A_NOT, A_MOVE, A_CLR, A_TST, A_TAS: begin
            case (op)
                A_AND:  ra = da & sa;
                A_OR:   ra = da | sa;
                A_EOR:  ra = da ^ sa;
                A_NOT:  ra = ~da;
                A_MOVE: ra = sa;
                A_CLR:  ra = 32'd0;
                default: ra = da;   // TST, TAS (flags from original)
            endcase
            n = ra[31]; z = (ra == 32'd0); v = 1'b0; c = 1'b0;
            r = ra >> sh;
            if (op == A_TAS) r = dv | 32'h80;
        end
        A_ADDA:   begin r = dv + sv; chg = 1'b0; end
        A_SUBA:   begin r = dv - sv; chg = 1'b0; end
        A_PASS:   begin r = sv; chg = 1'b0; end
        A_SCC:    begin r = cc_true ? 32'hFF : 32'h00; chg = 1'b0; end
        A_FROMSR: begin r = sv; chg = 1'b0; end
        A_FROMCCR:begin r = {27'd0, ccr}; chg = 1'b0; end
        A_ABCD: begin
            b = {6'd0, dv[3:0]} + {6'd0, sv[3:0]} + {9'd0, x};
            v = ~b[7];
            if (b > 10'd9) b = b + 10'd6;
            b = b + {6'd0, dv[7:4], 4'd0} + {6'd0, sv[7:4], 4'd0};
            bc = b > 10'h99;
            if (bc) b = b - 10'hA0;
            v = v & b[7];
            n = b[7]; c = bc; x = bc;
            if (b[7:0] != 8'd0) z = 1'b0;
            r = {24'd0, b[7:0]};
        end
        A_SBCD: begin
            b = {6'd0, dv[3:0]} - {6'd0, sv[3:0]} - {9'd0, x};
            v = ~b[7];
            if (b > 10'd9) b = b - 10'd6;
            b = b + {6'd0, dv[7:4], 4'd0} - {6'd0, sv[7:4], 4'd0};
            bc = b > 10'h99;
            if (bc) b = b + 10'hA0;
            v = v & b[7];
            n = b[7]; c = bc; x = bc;
            if (b[7:0] != 8'd0) z = 1'b0;
            r = {24'd0, b[7:0]};
        end
        A_NBCD: begin
            b = 10'h9A - {2'd0, dv[7:0]} - {9'd0, x};
            b = {2'd0, b[7:0]};
            if (b[7:0] != 8'h9A) begin
                v = ~b[7];
                if (b[3:0] == 4'hA) b = {2'd0, b[7:4], 4'd0} + 10'h10;
                b = {2'd0, b[7:0]};
                v = v & b[7];
                if (b[7:0] != 8'd0) z = 1'b0;
                c = 1'b1; x = 1'b1;
            end else begin
                v = 1'b0; c = 1'b0; x = 1'b0;
            end
            n = b[7];
            r = {24'd0, b[7:0]};
        end
        default: begin r = sv; chg = 1'b0; end
    endcase
    alu = {chg, x, n, z, v, c, r};
endfunction

// -----------------------------------------------------------------------------
// shifter (AS/LS/RO, not ROX): returns {ccr_new, result}
// -----------------------------------------------------------------------------
function automatic [36:0] shifter(input [1:0] typ, input left, input [1:0] s,
                                  input [5:0] cnt, input [31:0] vin, input [4:0] ccr);
    logic [31:0] v, va, r, m, bits;
    logic [63:0] e, rep;
    logic [5:0]  w, n;
    logic        x, c, vf;
    w  = s == SZ_B ? 6'd8 : s == SZ_W ? 6'd16 : 6'd32;
    v  = zxs(vin, s);
    x  = ccr[4]; c = 1'b0; vf = 1'b0; r = 32'd0;
    if (typ[1] == 1'b0) begin
        // arithmetic / logical
        if (left) begin
            va = v << (6'd32 - w);
            e  = {32'd0, va} << cnt;
            r  = e[31:0] >> (6'd32 - w);
            c  = (cnt == 0) ? 1'b0 : e[32];
            if (typ == 2'b00) begin  // ASL overflow
                m    = (cnt >= 6'd31) ? 32'hFFFF_FFFF : ~(32'hFFFF_FFFF >> (cnt + 6'd1));
                bits = va & m;
                vf   = !(bits == 32'd0 || bits == m);
            end
        end else begin
            e = (typ == 2'b00) ? {{32{msbs(v, s)}}, sxs(v, s)} : {32'd0, v};
            r = $signed(e) >>> cnt;
            if (cnt == 0) c = 1'b0;
            else c = e[cnt - 6'd1];
            r = zxs(r, s);
        end
        if (cnt != 0) x = c;
    end else begin
        // rotate (typ 11)
        n   = cnt & (w - 6'd1);
        rep = s == SZ_B ? {8{v[7:0]}} : s == SZ_W ? {4{v[15:0]}} : {2{v}};
        if (left) begin
            e = rep << n;
            r = s == SZ_B ? {24'd0, e[63:56]} : s == SZ_W ? {16'd0, e[63:48]} : e[63:32];
            c = (cnt == 0) ? 1'b0 : r[0];
        end else begin
            e = rep >> n;
            r = zxs(e[31:0], s);
            c = (cnt == 0) ? 1'b0 : msbs(r, s);
        end
    end
    shifter = {x, msbs(r, s), zxs(r, s) == 32'd0, vf, c, r};
endfunction

// -----------------------------------------------------------------------------
// instruction prefetch buffer + instruction cache
// -----------------------------------------------------------------------------
logic [29:0] ib_tag;
logic [31:0] ib_data;
logic        ib_valid;

logic [63:0] ic_valid;
logic [31:0] ic_data [0:63];
logic [24:0] ic_tag  [0:63];
wire  [5:0]  ic_idx = PC[7:2];
wire  [24:0] ic_curtag = {sr[13], PC[31:8]};
wire         ic_hit = ICACHE && cacr[0] && ic_valid[ic_idx] && (ic_tag[ic_idx] == ic_curtag);
wire         ib_hit = ib_valid && (ib_tag == PC[31:2]);
wire         iw_hit = ib_hit || ic_hit;
wire  [31:0] iw_long = ib_hit ? ib_data : ic_data[ic_idx];
wire  [15:0] iw = PC[1] ? iw_long[15:0] : iw_long[31:16];

wire ic_we = ICACHE && (state == S_IFETCH) && bus_ack && cacr[0] && !cacr[1];
always_ff @(posedge clk) begin
    if (ic_we) begin
        ic_data[bus_addr[7:2]] <= bus_rdata;
        ic_tag[bus_addr[7:2]]  <= {bus_fc[2], bus_addr[31:8]};
    end
end

// -----------------------------------------------------------------------------
// working registers
// -----------------------------------------------------------------------------
logic [15:0] IR, ext, ext2;
logic [1:0]  sz, ssz;                // op size, source size
logic [2:0]  ea_mode, ea_reg;
logic [1:0]  ea_sz;
logic [1:0]  ea_kind;
logic [31:0] ea_addr, ea_imm, ea_base, ea_bd, ea_od, ea_idx;
logic [15:0] ea_ext;
logic        ea_pc;

logic [2:0]  src_mode, src_reg, dst_mode, dst_reg;
logic        g_src_ea;               // src from EA (else preset)
logic [1:0]  g_dsel;                 // 0 = EA, 1 = Dn, 2 = An
logic        g_rd, g_wr, sext_src;
logic [4:0]  aop;
logic [31:0] src, dst, res;
logic [1:0]  dst_kind;
logic [31:0] dst_addr;
logic [2:0]  dst_rn;

logic [31:0] mem_rd;
logic        mem_avec;
logic        in_exc, in_iack;

logic        trace_t1, trace_t0, flow, stop_pending;

// exceptions
logic [7:0]  exc_vec;
logic [3:0]  exc_fmt;
logic [31:0] exc_pc, exc_addr;
logic        exc_int;
logic [2:0]  exc_lvl;
logic [15:0] x_sr;
logic        x_m;
logic [3:0]  x_cnt;
logic [15:0] x_ssw;
logic [31:0] x_dob;

// multi-purpose
logic [31:0] tmp, tmp2, bdisp;
logic [15:0] mvm_mask;
logic [31:0] mvm_addr;
logic [2:0]  cnt3;
logic [5:0]  cnt6;
logic [31:0] reset_cnt;

// multiplier / divider
logic [31:0] md_a, md_b;
logic [63:0] md_acc;
logic [1:0]  md_step;
logic [15:0] mul_x, mul_y;
wire  [31:0] mul_p = mul_x * mul_y;
logic        md_neg, md_rneg, md_signed, md_long64, md_word;
logic [31:0] md_q, md_r;
logic [32:0] md_rem;
logic [5:0]  md_cnt;

// bitfield
logic [31:0] bf_off;
logic [5:0]  bf_w;
logic [39:0] bf_buf;
logic [2:0]  bf_bc;
wire  [6:0]  bf_bsum   = {4'd0, bf_off[2:0]} + {1'b0, bf_w} + 7'd7;
wire  [2:0]  bf_bcount = bf_bsum[5:3];
logic [31:0] bf_addr;

// interrupts
logic [2:0]  ipl_q;
logic        nmi_prev, nmi_latch;
wire         int_pending = (ipl_q == 3'd7) ? (nmi_latch || sr[10:8] != 3'd7) : (ipl_q > sr[10:8]);

// -----------------------------------------------------------------------------
// macros for register / bus access (statements, used inside the FSM)
// -----------------------------------------------------------------------------
`define WR_A(N, V) \
    if ((N) == 3'd7) begin \
        if (!sr[13]) usp <= (V); else if (sr[12]) msp <= (V); else isp <= (V); \
    end else A[(N)] <= (V);

`define WR_R(I, V) \
    if ((I) >= 4'd8) begin `WR_A(((I) - 4'd8), V) end else D[(I)] <= (V);

`define MEMX(ADDR, SIZE, WE, DATA, FC, RET) begin \
    bus_req <= 1'b1; bus_we <= (WE); bus_addr <= (ADDR); bus_size <= (SIZE); \
    bus_wdata <= (DATA); bus_fc <= (FC); mem_ret <= (RET); state <= S_MEM; end

`define DFC (sr[13] ? 3'd5 : 3'd1)
`define MEMRD(ADDR, SIZE, RET)       `MEMX(ADDR, SIZE, 1'b0, 32'd0, `DFC, RET)
`define MEMWR(ADDR, SIZE, DATA, RET) `MEMX(ADDR, SIZE, 1'b1, DATA, `DFC, RET)

`define NEED_IW(RET) begin \
    bus_req <= 1'b1; bus_we <= 1'b0; bus_addr <= {PC[31:2], 2'b00}; bus_size <= SZ_L; \
    bus_fc <= sr[13] ? 3'd6 : 3'd2; if_ret <= (RET); state <= S_IFETCH; end

`define PUSHL(V, RET) begin `WR_A(3'd7, sp - 32'd4) `MEMWR(sp - 32'd4, SZ_L, V, RET) end
`define PUSHW(V, RET) begin `WR_A(3'd7, sp - 32'd2) `MEMWR(sp - 32'd2, SZ_W, V, RET) end

// exception: vector, format, stacked PC, address field.  Cancels trace.
`define EXC(V, F, P, AD) begin \
    exc_vec <= (V); exc_fmt <= (F); exc_pc <= (P); exc_addr <= (AD); exc_int <= 1'b0; \
    trace_t1 <= 1'b0; trace_t0 <= 1'b0; state <= X_START; end
// trap-type exception: trace (if pending) is taken after it
`define EXCT(V, F, P, AD) begin \
    exc_vec <= (V); exc_fmt <= (F); exc_pc <= (P); exc_addr <= (AD); exc_int <= 1'b0; \
    state <= X_START; end

`define ILLEGAL   `EXC(8'd4, 4'h0, PCi, PCi)
`define PRIV      `EXC(8'd8, 4'h0, PCi, PCi)

// call EA engine
`define CALL_EA(M, Rg, S, RET) begin \
    ea_mode <= (M); ea_reg <= (Rg); ea_sz <= (S); ea_ret <= (RET); state <= E_START; end

// -----------------------------------------------------------------------------
// combinational helpers for decode / EA
// -----------------------------------------------------------------------------
wire [1:0]  ir_sz  = IR[7:6];
wire [2:0]  ir_m   = IR[5:3];
wire [2:0]  ir_r   = IR[2:0];
wire [2:0]  ir_x   = IR[11:9];
wire [11:0] ir_cls = eacls(ir_m, ir_r);

// EA increment for (An)+ / -(An)
wire [31:0] ea_inc = (ea_sz == SZ_B) ? ((ea_reg == 3'd7) ? 32'd2 : 32'd1) :
                     (ea_sz == SZ_W) ? 32'd2 : 32'd4;
wire [31:0] ea_an  = R[{1'b1, ea_reg}];

// index register from extension word currently at iw
wire [31:0] idx_raw = R[iw[15:12]];
wire [31:0] idx_val = (iw[11] ? idx_raw : sx16(idx_raw[15:0])) << iw[10:9];

// ALU instance (combinational) for G_EXEC
wire [31:0] alu_s = sext_src ? sxs(src, ssz) : src;
wire [37:0] alu_o = alu(aop, sz, alu_s, dst, sr[4:0], cond(IR[11:8], sr[4:0]));

// shifter
wire [5:0]  sh_cnt = IR[5] ? D[IR[11:9]][5:0] : (IR[11:9] == 3'd0 ? 6'd8 : {3'd0, IR[11:9]});

// bitfield helpers
wire [31:0] bf_w32   = (bf_w == 6'd32) ? 32'hFFFF_FFFF : ~(32'hFFFF_FFFF >> bf_w);  // top-aligned mask
wire [4:0]  bf_ro    = bf_off[4:0];
wire [63:0] bf_rot64 = {D[IR[2:0]], D[IR[2:0]]} << bf_ro;

function automatic [7:0] pack_b(input [15:0] v);
    pack_b = {v[11:8], v[3:0]};
endfunction

// end address of the current bus transfer (for write snooping)
wire [31:0] bus_addr_end = bus_addr + ((bus_size == SZ_L) ? 32'd3 : (bus_size == SZ_W) ? 32'd1 : 32'd0);

// MOVEC read mux (control register selected by the extension word at iw)
logic [31:0] movec_rd;
always_comb begin
    case (iw[11:0])
        12'h000: movec_rd = {29'd0, sfc};
        12'h001: movec_rd = {29'd0, dfc};
        12'h002: movec_rd = cacr;
        12'h800: movec_rd = usp;
        12'h801: movec_rd = vbr;
        12'h802: movec_rd = caar;
        12'h803: movec_rd = msp;
        12'h804: movec_rd = isp;
        default: movec_rd = 32'd0;
    endcase
end

// format $A frame filler words (offset 8 + 2*x_cnt)
logic [15:0] xfa_word;
always_comb begin
    case (x_cnt)
        4'd1: xfa_word = x_ssw;              // offset 10: SSW
        4'd4: xfa_word = exc_addr[31:16];    // offset 16: fault address
        4'd5: xfa_word = exc_addr[15:0];
        4'd8: xfa_word = x_dob[31:16];       // offset 24: data output buffer
        4'd9: xfa_word = x_dob[15:0];
        default: xfa_word = 16'd0;
    endcase
end

// -----------------------------------------------------------------------------
// debug outputs
// -----------------------------------------------------------------------------
assign dbg_retire = (state == S_RETIRE);
assign dbg_pc  = PC;
assign dbg_sr  = sr;
assign dbg_usp = usp;
assign dbg_isp = isp;
assign dbg_msp = msp;
assign dbg_vbr = vbr;
generate
    for (gi = 0; gi < 8; gi++) begin : g_dbg
        assign dbg_d[gi*32 +: 32] = D[gi];
        assign dbg_a[gi*32 +: 32] = R[gi+8];
    end
endgenerate
assign halted = (state == S_HALT);

// -----------------------------------------------------------------------------
// main FSM
// -----------------------------------------------------------------------------
always_ff @(posedge clk) begin
    // interrupt input sampling
    ipl_q    <= ipl;
    nmi_prev <= (ipl_q == 3'd7);
    if (ipl_q == 3'd7 && !nmi_prev) nmi_latch <= 1'b1;

    if (!rst_n) begin
        state     <= S_RESET;
        bus_req   <= 1'b0;
        bus_we    <= 1'b0;
        bus_lock  <= 1'b0;
        bus_size  <= SZ_L;
        bus_fc    <= 3'd6;
        bus_addr  <= 32'd0;
        bus_wdata <= 32'd0;
        sr        <= 16'h2700;
        vbr       <= 32'd0;
        usp       <= 32'd0;
        msp       <= 32'd0;
        for (int k = 0; k < 8; k++) begin D[k] <= 32'd0; A[k] <= 32'd0; end
        cacr      <= 32'd0;
        caar      <= 32'd0;
        sfc       <= 3'd0;
        dfc       <= 3'd0;
        ib_valid  <= 1'b0;
        ic_valid  <= 64'd0;
        reset_out <= 1'b0;
        in_exc    <= 1'b1;
        in_iack   <= 1'b0;
        trace_t1  <= 1'b0;
        trace_t0  <= 1'b0;
        nmi_latch <= 1'b0;
        stop_pending <= 1'b0;
    end else begin
    case (state)

    // =========================================================================
    // reset
    // =========================================================================
    S_RESET: begin
        `MEMX(32'd0, SZ_L, 1'b0, 32'd0, 3'd6, S_RST1)
    end
    S_RST1: begin
        isp <= mem_rd;
        `MEMX(32'd4, SZ_L, 1'b0, 32'd0, 3'd6, S_RST2)
    end
    S_RST2: begin
        PC <= mem_rd;
        in_exc <= 1'b0;
        state <= S_FETCH;
    end

    // =========================================================================
    // bus subroutines
    // =========================================================================
    S_MEM: begin
        if (bus_ack) begin
            bus_req  <= 1'b0;
            mem_rd   <= zxs(bus_rdata, bus_size);
            mem_avec <= bus_avec;
            state    <= mem_ret;
            if (bus_we) begin
                // keep instruction stream coherent with our own writes
                if (ib_tag == bus_addr[31:2] || ib_tag == bus_addr_end[31:2]) ib_valid <= 1'b0;
                ic_valid[bus_addr[7:2]]     <= 1'b0;
                ic_valid[bus_addr_end[7:2]] <= 1'b0;
            end
        end else if (bus_berr) begin
            bus_req  <= 1'b0;
            bus_lock <= 1'b0;
            if (in_iack) begin
                mem_avec <= 1'b0;
                mem_rd   <= 32'd24;      // spurious interrupt
                state    <= mem_ret;
            end else if (in_exc) begin
                state <= S_HALT;         // double bus fault
            end else begin
                x_ssw <= {7'd0, 1'b1, 1'b0, ~bus_we, bus_size == SZ_B ? 2'b01 : bus_size == SZ_W ? 2'b10 : 2'b00, 1'b0, bus_fc};
                x_dob <= bus_wdata;
                `EXC(8'd2, 4'hA, PCi, bus_addr)
            end
        end
    end

    S_IFETCH: begin
        if (bus_ack) begin
            bus_req  <= 1'b0;
            ib_data  <= bus_rdata;
            ib_tag   <= bus_addr[31:2];
            ib_valid <= 1'b1;
            if (ic_we) ic_valid[bus_addr[7:2]] <= 1'b1;
            state    <= if_ret;
        end else if (bus_berr) begin
            bus_req <= 1'b0;
            if (in_exc) state <= S_HALT;
            else begin
                x_ssw <= {16'h0006 | {13'd0, bus_fc}} | 16'h4000;
                x_dob <= 32'd0;
                `EXC(8'd2, 4'hA, PCi, bus_addr)
            end
        end
    end

    // =========================================================================
    // instruction fetch / completion
    // =========================================================================
    S_FETCH: begin
        if (int_pending) begin
            state <= S_INT0;
        end else if (PC[0]) begin
            x_ssw <= 16'h4006 | {13'd0, sr[13], 2'b10};
            x_dob <= 32'd0;
            `EXC(8'd3, 4'hA, PC, PC)
        end else if (!iw_hit) begin
            `NEED_IW(S_FETCH)
        end else begin
            IR       <= iw;
            PCi      <= PC;
            PC       <= PC + 32'd2;
            trace_t1 <= sr[15];
            trace_t0 <= sr[14];
            flow     <= 1'b0;
            state    <= S_DECODE;
        end
    end

    S_END: begin
        bus_lock <= 1'b0;
        if (trace_t1 || (trace_t0 && flow)) begin
            trace_t1 <= 1'b0;
            trace_t0 <= 1'b0;
            `EXCT(8'd9, 4'h2, PC, PCi)
        end else begin
            state <= S_RETIRE;
        end
    end

    S_RETIRE: begin
        if (stop_pending) begin
            stop_pending <= 1'b0;
            state <= S_STOPPED;
        end else
            state <= S_FETCH;
    end

    S_STOPPED: begin
        if (int_pending) state <= S_FETCH;
    end

    S_HALT: ;

    // =========================================================================
    // effective address engine
    // =========================================================================
    E_START: begin
        ea_pc <= 1'b0;
        case (ea_mode)
            3'd0: begin ea_kind <= K_D; state <= ea_ret; end
            3'd1: begin ea_kind <= K_A; state <= ea_ret; end
            3'd2: begin ea_kind <= K_M; ea_addr <= ea_an; state <= ea_ret; end
            3'd3: begin ea_kind <= K_M; ea_addr <= ea_an; `WR_A(ea_reg, ea_an + ea_inc) state <= ea_ret; end
            3'd4: begin ea_kind <= K_M; ea_addr <= ea_an - ea_inc; `WR_A(ea_reg, ea_an - ea_inc) state <= ea_ret; end
            3'd5: begin ea_base <= ea_an; state <= E_D16; end
            3'd6: begin ea_base <= ea_an; state <= E_IDX; end
            default: case (ea_reg)
                3'd0: state <= E_ABSW;
                3'd1: state <= E_ABSL1;
                3'd2: begin ea_base <= PC; ea_pc <= 1'b1; state <= E_D16; end
                3'd3: begin ea_base <= PC; ea_pc <= 1'b1; state <= E_IDX; end
                3'd4: state <= E_IMM1;
                default: `ILLEGAL
            endcase
        endcase
    end
    E_D16: begin
        if (!iw_hit) `NEED_IW(E_D16)
        else begin
            PC <= PC + 32'd2;
            ea_addr <= ea_base + sx16(iw);
            ea_kind <= K_M;
            state <= ea_ret;
        end
    end
    E_ABSW: begin
        if (!iw_hit) `NEED_IW(E_ABSW)
        else begin
            PC <= PC + 32'd2;
            ea_addr <= sx16(iw);
            ea_kind <= K_M;
            state <= ea_ret;
        end
    end
    E_ABSL1: begin
        if (!iw_hit) `NEED_IW(E_ABSL1)
        else begin PC <= PC + 32'd2; ea_addr[31:16] <= iw; state <= E_ABSL2; end
    end
    E_ABSL2: begin
        if (!iw_hit) `NEED_IW(E_ABSL2)
        else begin PC <= PC + 32'd2; ea_addr[15:0] <= iw; ea_kind <= K_M; state <= ea_ret; end
    end
    E_IMM1: begin
        if (!iw_hit) `NEED_IW(E_IMM1)
        else begin
            PC <= PC + 32'd2;
            ea_kind <= K_I;
            if (ea_sz == SZ_L) begin ea_imm[31:16] <= iw; state <= E_IMM2; end
            else begin
                ea_imm <= (ea_sz == SZ_B) ? {24'd0, iw[7:0]} : {16'd0, iw};
                state <= ea_ret;
            end
        end
    end
    E_IMM2: begin
        if (!iw_hit) `NEED_IW(E_IMM2)
        else begin PC <= PC + 32'd2; ea_imm[15:0] <= iw; state <= ea_ret; end
    end
    E_IDX: begin
        if (!iw_hit) `NEED_IW(E_IDX)
        else begin
            PC <= PC + 32'd2;
            ea_ext <= iw;
            if (!iw[8]) begin
                // brief format (scale allowed on 68020)
                ea_addr <= ea_base + sx8(iw[7:0]) + idx_val;
                ea_kind <= K_M;
                state <= ea_ret;
            end else begin
                ea_idx <= iw[6] ? 32'd0 : idx_val;
                if (iw[7]) ea_base <= 32'd0;
                ea_bd <= 32'd0;
                ea_od <= 32'd0;
                if (iw[5]) state <= iw[4] ? E_BDL1 : E_BDW;
                else       state <= E_FULL;
            end
        end
    end
    E_BDW: begin
        if (!iw_hit) `NEED_IW(E_BDW)
        else begin PC <= PC + 32'd2; ea_bd <= sx16(iw); state <= E_FULL; end
    end
    E_BDL1: begin
        if (!iw_hit) `NEED_IW(E_BDL1)
        else begin PC <= PC + 32'd2; ea_bd[31:16] <= iw; state <= E_BDL2; end
    end
    E_BDL2: begin
        if (!iw_hit) `NEED_IW(E_BDL2)
        else begin PC <= PC + 32'd2; ea_bd[15:0] <= iw; state <= E_FULL; end
    end
    E_FULL: begin
        if (ea_ext[2:0] == 3'd0) begin
            ea_addr <= ea_base + ea_bd + ea_idx;
            ea_kind <= K_M;
            state <= ea_ret;
        end else if (ea_ext[1]) begin
            state <= ea_ext[0] ? E_ODL1 : E_ODW;
        end else begin
            state <= E_MI;
        end
    end
    E_ODW: begin
        if (!iw_hit) `NEED_IW(E_ODW)
        else begin PC <= PC + 32'd2; ea_od <= sx16(iw); state <= E_MI; end
    end
    E_ODL1: begin
        if (!iw_hit) `NEED_IW(E_ODL1)
        else begin PC <= PC + 32'd2; ea_od[31:16] <= iw; state <= E_ODL2; end
    end
    E_ODL2: begin
        if (!iw_hit) `NEED_IW(E_ODL2)
        else begin PC <= PC + 32'd2; ea_od[15:0] <= iw; state <= E_MI; end
    end
    E_MI: begin
        if (ea_ext[2]) `MEMRD(ea_base + ea_bd, SZ_L, E_MI2)
        else           `MEMRD(ea_base + ea_bd + ea_idx, SZ_L, E_MI2)
    end
    E_MI2: begin
        ea_addr <= mem_rd + (ea_ext[2] ? ea_idx : 32'd0) + ea_od;
        ea_kind <= K_M;
        state <= ea_ret;
    end

    // =========================================================================
    // generic ALU flow:  [src EA read] -> [dst EA (read)] -> exec -> [write]
    // =========================================================================
    G_SRC: begin
        if (g_src_ea) `CALL_EA(src_mode, src_reg, ssz, G_SRC_RD)
        else state <= g_after_src;
    end
    G_SRC_RD: begin
        case (ea_kind)
            K_D: begin src <= D[ea_reg]; state <= g_after_src; end
            K_A: begin src <= R[{1'b1, ea_reg}]; state <= g_after_src; end
            K_I: begin src <= ea_imm; state <= g_after_src; end
            default: `MEMRD(ea_addr, ssz, G_SRC_RD2)
        endcase
    end
    G_SRC_RD2: begin
        src <= mem_rd;
        state <= g_after_src;
    end
    G_DST: begin
        case (g_dsel)
            2'd0: `CALL_EA(dst_mode, dst_reg, sz, G_DST_RD)
            2'd1: begin dst <= D[dst_reg]; dst_kind <= K_D; dst_rn <= dst_reg; state <= g_after_dst; end
            default: begin dst <= R[{1'b1, dst_reg}]; dst_kind <= K_A; dst_rn <= dst_reg; state <= g_after_dst; end
        endcase
    end
    G_DST_RD: begin
        dst_kind <= ea_kind;
        dst_addr <= ea_addr;
        dst_rn   <= ea_reg;
        case (ea_kind)
            K_D: begin dst <= D[ea_reg]; state <= g_after_dst; end
            K_A: begin dst <= R[{1'b1, ea_reg}]; state <= g_after_dst; end
            K_I: begin dst <= ea_imm; state <= g_after_dst; end
            default: if (g_rd) `MEMRD(ea_addr, sz, G_DST_RD2)
                     else state <= g_after_dst;
        endcase
    end
    G_DST_RD2: begin
        dst <= mem_rd;
        state <= g_after_dst;
    end
    G_EXEC: begin
        res <= alu_o[31:0];
        if (alu_o[37]) sr[4:0] <= alu_o[36:32];
        case (aop)
            A_TOCCR: sr[4:0] <= src[4:0];
            A_TOSR:  sr <= src[15:0] & SR_MASK;
            default: ;
        endcase
        state <= g_wr ? G_WB : S_END;
    end
    G_WB: begin
        case (dst_kind)
            K_D: begin D[dst_rn] <= merge(D[dst_rn], res, sz); state <= S_END; end
            K_A: begin `WR_A(dst_rn, res) state <= S_END; end
            default: `MEMWR(dst_addr, sz, res, S_END)
        endcase
    end

    // =========================================================================
    // decode
    // =========================================================================
    S_DECODE: begin
        // defaults for the generic flow
        g_src_ea    <= 1'b1;
        g_dsel      <= 2'd0;
        g_rd        <= 1'b1;
        g_wr        <= 1'b1;
        sext_src    <= 1'b0;
        g_after_src <= G_DST;
        g_after_dst <= G_EXEC;
        src_mode    <= ir_m;
        src_reg     <= ir_r;
        dst_mode    <= ir_m;
        dst_reg     <= ir_r;
        sz          <= ir_sz;
        ssz         <= ir_sz;
        cnt3        <= 3'd0;

        casez (IR)
        // ------------------------------------------------------------- line 0
        16'b0000_0000_0011_1100, 16'b0000_0010_0011_1100, 16'b0000_1010_0011_1100: begin
            state <= S_IMMSR;                        // ORI/ANDI/EORI to CCR
        end
        16'b0000_0000_0111_1100, 16'b0000_0010_0111_1100, 16'b0000_1010_0111_1100: begin
            if (!sr[13]) `PRIV else state <= S_IMMSR; // ... to SR
        end
        16'b0000_1100_1111_1100, 16'b0000_1110_1111_1100: begin   // CAS2
            sz <= IR[9] ? SZ_L : SZ_W;
            bus_lock <= 1'b1;
            state <= S_CAS2X1;
        end
        16'b0000_0??0_11??_????: begin                             // CMP2/CHK2 (or CALLM)
            if (IR[10:9] == 2'b11 || !eaok(ir_m, ir_r, M_CTRL)) `ILLEGAL
            else begin sz <= IR[10:9]; state <= S_CHK2X; end
        end
        16'b0000_1??0_11??_????: begin                             // CAS (or BSET #)
            if (IR[10:9] == 2'b00) begin
                // static BSET
                if (!eaok(ir_m, ir_r, M_DALT)) `ILLEGAL
                else state <= S_BITX;
            end else if (!eaok(ir_m, ir_r, M_MALT)) `ILLEGAL
            else begin
                sz <= IR[10:9] - 2'd1;
                state <= S_CASX;
            end
        end
        16'b0000_???1_??00_1???: begin                             // MOVEP
            state <= S_MOVEP0;
        end
        16'b0000_???1_????_????: begin                             // dynamic bit op
            if (!eaok(ir_m, ir_r, (IR[7:6] == 2'b00) ? M_DATA : M_DALT)) `ILLEGAL
            else begin src <= D[ir_x]; state <= S_BITD; end
        end
        16'b0000_1000_????_????: begin                             // static bit op
            if (!eaok(ir_m, ir_r, (IR[7:6] == 2'b00) ? M_DATA_NI : M_DALT)) `ILLEGAL
            else state <= S_BITX;
        end
        16'b0000_0000_????_????, 16'b0000_0010_????_????, 16'b0000_0100_????_????,
        16'b0000_0110_????_????, 16'b0000_1010_????_????, 16'b0000_1100_????_????: begin
            // ORI ANDI SUBI ADDI EORI CMPI
            if (ir_sz == 2'b11 || !eaok(ir_m, ir_r, (IR[11:9] == 3'b110) ? M_DATA_NI : M_DALT)) `ILLEGAL
            else begin
                src_mode <= 3'd7; src_reg <= 3'd4;
                case (IR[11:9])
                    3'b000: aop <= A_OR;
                    3'b001: aop <= A_AND;
                    3'b010: aop <= A_SUB;
                    3'b011: aop <= A_ADD;
                    3'b101: aop <= A_EOR;
                    default: begin aop <= A_CMP; g_wr <= 1'b0; end
                endcase
                state <= G_SRC;
            end
        end
        16'b0000_1110_????_????: begin                             // MOVES
            if (!sr[13]) `PRIV
            else if (ir_sz == 2'b11 || !eaok(ir_m, ir_r, M_MALT)) `ILLEGAL
            else state <= S_MOVESX;
        end

        // ------------------------------------------------------- MOVE / MOVEA
        16'b0001_????_????_????, 16'b0010_????_????_????, 16'b0011_????_????_????: begin
            sz  <= (IR[13:12] == 2'b01) ? SZ_B : (IR[13:12] == 2'b11) ? SZ_W : SZ_L;
            ssz <= (IR[13:12] == 2'b01) ? SZ_B : (IR[13:12] == 2'b11) ? SZ_W : SZ_L;
            dst_mode <= IR[8:6];
            dst_reg  <= IR[11:9];
            g_rd <= 1'b0;
            if (!eaok(ir_m, ir_r, (IR[13:12] == 2'b01) ? M_DATA : M_ALL)) `ILLEGAL
            else if (IR[8:6] == 3'd1) begin
                // MOVEA
                if (IR[13:12] == 2'b01) `ILLEGAL
                else begin
                    aop <= A_PASS; sext_src <= 1'b1; sz <= SZ_L; g_dsel <= 2'd2;
                    state <= G_SRC;
                end
            end else if (!eaok(IR[8:6], IR[11:9], M_DALT)) `ILLEGAL
            else begin
                aop <= A_MOVE;
                state <= G_SRC;
            end
        end

        // -------------------------------------------------------------- line 4
        16'h4AFC: `ILLEGAL
        16'h4E70: begin                                            // RESET
            if (!sr[13]) `PRIV else begin reset_cnt <= 32'd0; state <= S_RESETI; end
        end
        16'h4E71: state <= S_END;                                  // NOP
        16'h4E72: begin                                            // STOP
            if (!sr[13]) `PRIV else state <= S_STOP0;
        end
        16'h4E73: begin                                            // RTE
            if (!sr[13]) `PRIV else state <= S_RTE0;
        end
        16'h4E74: state <= S_RTD0;                                 // RTD
        16'h4E75: begin                                            // RTS
            `MEMRD(sp, SZ_L, S_RTS1)
        end
        16'h4E76: begin                                            // TRAPV
            if (sr[1]) `EXCT(8'd7, 4'h2, PC, PCi)
            else state <= S_END;
        end
        16'h4E77: begin                                            // RTR
            `MEMRD(sp, SZ_W, S_RTR1)
        end
        16'b0100_1110_0111_101?: begin                             // MOVEC
            if (!sr[13]) `PRIV else state <= S_MOVECX;
        end
        16'b0100_1110_0100_????: begin                             // TRAP #n
            `EXCT({4'd2, IR[3:0]}, 4'h0, PC, PCi)
        end
        16'b0100_1110_0101_0???: state <= S_LINK0;                 // LINK.W
        16'b0100_1000_0000_1???: state <= S_LINKL;                 // LINK.L
        16'b0100_1110_0101_1???: begin                             // UNLK
            `WR_A(3'd7, R[{1'b1, ir_r}])
            `MEMRD(R[{1'b1, ir_r}], SZ_L, S_UNLK1)
        end
        16'b0100_1110_0110_????: begin                             // MOVE USP
            if (!sr[13]) `PRIV
            else begin
                if (IR[3]) begin `WR_A(ir_r, usp) end
                else usp <= R[{1'b1, ir_r}];
                state <= S_END;
            end
        end
        16'b0100_1110_10??_????: begin                             // JSR
            if (!eaok(ir_m, ir_r, M_CTRL)) `ILLEGAL
            else `CALL_EA(ir_m, ir_r, SZ_L, S_JSR2)
        end
        16'b0100_1110_11??_????: begin                             // JMP
            if (!eaok(ir_m, ir_r, M_CTRL)) `ILLEGAL
            else `CALL_EA(ir_m, ir_r, SZ_L, S_JEA)
        end
        16'b0100_1000_0100_0???: begin                             // SWAP
            D[ir_r] <= {D[ir_r][15:0], D[ir_r][31:16]};
            sr[3] <= D[ir_r][15];
            sr[2] <= (D[ir_r] == 32'd0);
            sr[1:0] <= 2'b00;
            state <= S_END;
        end
        16'b0100_1000_0100_1???: `ILLEGAL                          // BKPT (no debugger)
        16'b0100_1000_1000_0???: begin                             // EXT.W
            D[ir_r][15:0] <= {{8{D[ir_r][7]}}, D[ir_r][7:0]};
            sr[3] <= D[ir_r][7];
            sr[2] <= (D[ir_r][7:0] == 8'd0);
            sr[1:0] <= 2'b00;
            state <= S_END;
        end
        16'b0100_1000_1100_0???: begin                             // EXT.L
            D[ir_r] <= sx16(D[ir_r][15:0]);
            sr[3] <= D[ir_r][15];
            sr[2] <= (D[ir_r][15:0] == 16'd0);
            sr[1:0] <= 2'b00;
            state <= S_END;
        end
        16'b0100_1001_1100_0???: begin                             // EXTB.L
            D[ir_r] <= sx8(D[ir_r][7:0]);
            sr[3] <= D[ir_r][7];
            sr[2] <= (D[ir_r][7:0] == 8'd0);
            sr[1:0] <= 2'b00;
            state <= S_END;
        end
        16'b0100_1000_00??_????: begin                             // NBCD
            if (!eaok(ir_m, ir_r, M_DALT)) `ILLEGAL
            else begin sz <= SZ_B; aop <= A_NBCD; g_src_ea <= 1'b0; state <= G_DST; end
        end
        16'b0100_1000_01??_????: begin                             // PEA
            if (!eaok(ir_m, ir_r, M_CTRL)) `ILLEGAL
            else `CALL_EA(ir_m, ir_r, SZ_L, S_PEA)
        end
        16'b0100_1?00_1???_????: begin                             // MOVEM
            if (!eaok(ir_m, ir_r, IR[10] ? (M_CTRL | 12'h008) : (M_CALT | 12'h010))) `ILLEGAL
            else begin sz <= IR[6] ? SZ_L : SZ_W; state <= S_MOVEM0; end
        end
        16'b0100_0000_11??_????: begin                             // MOVE from SR
            if (!sr[13]) `PRIV
            else if (!eaok(ir_m, ir_r, M_DALT)) `ILLEGAL
            else begin sz <= SZ_W; g_src_ea <= 1'b0; src <= {16'd0, sr}; aop <= A_FROMSR; g_rd <= 1'b0; state <= G_DST; end
        end
        16'b0100_0010_11??_????: begin                             // MOVE from CCR
            if (!eaok(ir_m, ir_r, M_DALT)) `ILLEGAL
            else begin sz <= SZ_W; g_src_ea <= 1'b0; aop <= A_FROMCCR; g_rd <= 1'b0; state <= G_DST; end
        end
        16'b0100_0100_11??_????: begin                             // MOVE to CCR
            if (!eaok(ir_m, ir_r, M_DATA)) `ILLEGAL
            else begin ssz <= SZ_W; sz <= SZ_W; aop <= A_TOCCR; g_after_src <= G_EXEC; g_wr <= 1'b0; state <= G_SRC; end
        end
        16'b0100_0110_11??_????: begin                             // MOVE to SR
            if (!sr[13]) `PRIV
            else if (!eaok(ir_m, ir_r, M_DATA)) `ILLEGAL
            else begin ssz <= SZ_W; sz <= SZ_W; aop <= A_TOSR; g_after_src <= G_EXEC; g_wr <= 1'b0; state <= G_SRC; end
        end
        16'b0100_0000_????_????, 16'b0100_0010_????_????,
        16'b0100_0100_????_????, 16'b0100_0110_????_????: begin    // NEGX CLR NEG NOT
            if (!eaok(ir_m, ir_r, M_DALT)) `ILLEGAL
            else begin
                g_src_ea <= 1'b0;
                case (IR[10:9])
                    2'b00: aop <= A_NEGX;
                    2'b01: begin aop <= A_CLR; g_rd <= 1'b0; end
                    2'b10: aop <= A_NEG;
                    default: aop <= A_NOT;
                endcase
                state <= G_DST;
            end
        end
        16'b0100_1010_11??_????: begin                             // TAS
            if (!eaok(ir_m, ir_r, M_DALT)) `ILLEGAL
            else begin sz <= SZ_B; g_src_ea <= 1'b0; aop <= A_TAS; bus_lock <= (ir_m != 3'd0); state <= G_DST; end
        end
        16'b0100_1010_????_????: begin                             // TST
            if (!eaok(ir_m, ir_r, (ir_sz == SZ_B) ? M_DATA : M_ALL)) `ILLEGAL
            else begin g_src_ea <= 1'b0; aop <= A_TST; g_wr <= 1'b0; state <= G_DST; end
        end
        16'b0100_1100_00??_????: begin                             // MULU.L / MULS.L
            if (!eaok(ir_m, ir_r, M_DATA)) `ILLEGAL
            else begin ssz <= SZ_L; sz <= SZ_L; state <= S_MULLX; end
        end
        16'b0100_1100_01??_????: begin                             // DIVU.L / DIVS.L
            if (!eaok(ir_m, ir_r, M_DATA)) `ILLEGAL
            else begin ssz <= SZ_L; sz <= SZ_L; state <= S_DIVLX; end
        end
        16'b0100_???1_11??_????: begin                             // LEA
            if (!eaok(ir_m, ir_r, M_CTRL)) `ILLEGAL
            else `CALL_EA(ir_m, ir_r, SZ_L, S_LEA)
        end
        16'b0100_???1_?0??_????: begin                             // CHK.W / CHK.L
            if (!eaok(ir_m, ir_r, M_DATA)) `ILLEGAL
            else begin
                ssz <= IR[7] ? SZ_W : SZ_L;
                sz  <= IR[7] ? SZ_W : SZ_L;
                g_after_src <= S_CHK;
                state <= G_SRC;
            end
        end

        // -------------------------------------------------------------- line 5
        16'b0101_????_1100_1???: state <= S_DBCC;                  // DBcc
        16'b0101_????_1111_1???: begin                             // TRAPcc
            if (ir_r == 3'd2 || ir_r == 3'd3 || ir_r == 3'd4) state <= S_TRAPCC;
            else if (eaok(ir_m, ir_r, M_DALT)) begin               // Scc abs
                sz <= SZ_B; g_src_ea <= 1'b0; g_rd <= 1'b0; aop <= A_SCC; state <= G_DST;
            end else `ILLEGAL
        end
        16'b0101_????_11??_????: begin                             // Scc
            if (!eaok(ir_m, ir_r, M_DALT)) `ILLEGAL
            else begin sz <= SZ_B; g_src_ea <= 1'b0; g_rd <= 1'b0; aop <= A_SCC; state <= G_DST; end
        end
        16'b0101_????_????_????: begin                             // ADDQ / SUBQ
            if (!eaok(ir_m, ir_r, (ir_sz == SZ_B) ? M_DALT : M_ALT)) `ILLEGAL
            else begin
                g_src_ea <= 1'b0;
                src <= (IR[11:9] == 3'd0) ? 32'd8 : {29'd0, IR[11:9]};
                if (ir_m == 3'd1) begin
                    sz <= SZ_L;
                    aop <= IR[8] ? A_SUBA : A_ADDA;
                end else
                    aop <= IR[8] ? A_SUB : A_ADD;
                state <= G_DST;
            end
        end

        // -------------------------------------------------------------- line 6
        16'b0110_????_????_????: state <= S_BCC0;

        // -------------------------------------------------------------- line 7
        16'b0111_???0_????_????: begin                             // MOVEQ
            D[ir_x] <= sx8(IR[7:0]);
            sr[3] <= IR[7];
            sr[2] <= (IR[7:0] == 8'd0);
            sr[1:0] <= 2'b00;
            state <= S_END;
        end

        // -------------------------------------------------------------- line 8
        16'b1000_????_11??_????: begin                             // DIVU.W / DIVS.W
            if (!eaok(ir_m, ir_r, M_DATA)) `ILLEGAL
            else begin ssz <= SZ_W; sz <= SZ_W; g_after_src <= S_DIVW; state <= G_SRC; end
        end
        16'b1000_???1_0000_????: begin                             // SBCD
            sz <= SZ_B; ssz <= SZ_B; aop <= A_SBCD;
            src_mode <= IR[3] ? 3'd4 : 3'd0; dst_mode <= IR[3] ? 3'd4 : 3'd0;
            src_reg <= ir_r; dst_reg <= ir_x;
            state <= G_SRC;
        end
        16'b1000_???1_0100_????: state <= S_PACKX;                 // PACK
        16'b1000_???1_1000_????: state <= S_PACKX;                 // UNPK
        16'b1000_????_????_????: begin                             // OR
            if (!IR[8]) begin
                if (!eaok(ir_m, ir_r, M_DATA)) `ILLEGAL
                else begin aop <= A_OR; g_dsel <= 2'd1; dst_reg <= ir_x; state <= G_SRC; end
            end else begin
                if (!eaok(ir_m, ir_r, M_MALT)) `ILLEGAL
                else begin aop <= A_OR; g_src_ea <= 1'b0; src <= D[ir_x]; state <= G_DST; end
            end
        end

        // -------------------------------------------------------- line 9 / D
        16'b1?01_????_11??_????: begin                             // SUBA / ADDA
            ssz <= IR[8] ? SZ_L : SZ_W;
            sz  <= SZ_L;
            sext_src <= 1'b1;
            aop <= IR[14] ? A_ADDA : A_SUBA;
            g_dsel <= 2'd2; dst_reg <= ir_x;
            state <= G_SRC;
        end
        16'b1?01_???1_??00_????: begin                             // SUBX / ADDX
            aop <= IR[14] ? A_ADDX : A_SUBX;
            src_mode <= IR[3] ? 3'd4 : 3'd0; dst_mode <= IR[3] ? 3'd4 : 3'd0;
            src_reg <= ir_r; dst_reg <= ir_x;
            state <= G_SRC;
        end
        16'b1?01_????_????_????: begin                             // SUB / ADD
            if (!IR[8]) begin
                if (!eaok(ir_m, ir_r, (ir_sz == SZ_B) ? M_DATA : M_ALL)) `ILLEGAL
                else begin aop <= IR[14] ? A_ADD : A_SUB; g_dsel <= 2'd1; dst_reg <= ir_x; state <= G_SRC; end
            end else begin
                if (!eaok(ir_m, ir_r, M_MALT)) `ILLEGAL
                else begin aop <= IR[14] ? A_ADD : A_SUB; g_src_ea <= 1'b0; src <= D[ir_x]; state <= G_DST; end
            end
        end

        // -------------------------------------------------------------- line B
        16'b1011_????_11??_????: begin                             // CMPA
            ssz <= IR[8] ? SZ_L : SZ_W; sz <= SZ_L; sext_src <= 1'b1;
            aop <= A_CMP; g_wr <= 1'b0;
            g_dsel <= 2'd2; dst_reg <= ir_x;
            state <= G_SRC;
        end
        16'b1011_???1_??00_1???: begin                             // CMPM
            aop <= A_CMP; g_wr <= 1'b0;
            src_mode <= 3'd3; dst_mode <= 3'd3; src_reg <= ir_r; dst_reg <= ir_x;
            state <= G_SRC;
        end
        16'b1011_???1_????_????: begin                             // EOR
            if (!eaok(ir_m, ir_r, M_DALT)) `ILLEGAL
            else begin aop <= A_EOR; g_src_ea <= 1'b0; src <= D[ir_x]; state <= G_DST; end
        end
        16'b1011_???0_????_????: begin                             // CMP
            if (!eaok(ir_m, ir_r, (ir_sz == SZ_B) ? M_DATA : M_ALL)) `ILLEGAL
            else begin aop <= A_CMP; g_wr <= 1'b0; g_dsel <= 2'd1; dst_reg <= ir_x; state <= G_SRC; end
        end

        // -------------------------------------------------------------- line C
        16'b1100_????_11??_????: begin                             // MULU.W / MULS.W
            if (!eaok(ir_m, ir_r, M_DATA)) `ILLEGAL
            else begin ssz <= SZ_W; sz <= SZ_W; g_after_src <= S_MULW; state <= G_SRC; end
        end
        16'b1100_???1_0000_????: begin                             // ABCD
            sz <= SZ_B; ssz <= SZ_B; aop <= A_ABCD;
            src_mode <= IR[3] ? 3'd4 : 3'd0; dst_mode <= IR[3] ? 3'd4 : 3'd0;
            src_reg <= ir_r; dst_reg <= ir_x;
            state <= G_SRC;
        end
        16'b1100_???1_0100_0???: begin                             // EXG Dx,Dy
            D[ir_x] <= D[ir_r]; D[ir_r] <= D[ir_x]; state <= S_END;
        end
        16'b1100_???1_0100_1???: begin                             // EXG Ax,Ay
            `WR_A(ir_x, R[{1'b1, ir_r}])
            `WR_A(ir_r, R[{1'b1, ir_x}])
            state <= S_END;
        end
        16'b1100_???1_1000_1???: begin                             // EXG Dx,Ay
            D[ir_x] <= R[{1'b1, ir_r}];
            `WR_A(ir_r, D[ir_x])
            state <= S_END;
        end
        16'b1100_????_????_????: begin                             // AND
            if (!IR[8]) begin
                if (!eaok(ir_m, ir_r, M_DATA)) `ILLEGAL
                else begin aop <= A_AND; g_dsel <= 2'd1; dst_reg <= ir_x; state <= G_SRC; end
            end else begin
                if (!eaok(ir_m, ir_r, M_MALT)) `ILLEGAL
                else begin aop <= A_AND; g_src_ea <= 1'b0; src <= D[ir_x]; state <= G_DST; end
            end
        end

        // -------------------------------------------------------------- line E
        16'b1110_1???_11??_????: begin                             // bit field
            if (!eaok(ir_m, ir_r, (IR[10:8] == 3'b000 || IR[10:8] == 3'b001 || IR[10:8] == 3'b011 || IR[10:8] == 3'b101)
                                   ? (M_CTRL | 12'h001) : (M_CALT | 12'h001))) `ILLEGAL
            else state <= S_BFX;
        end
        16'b1110_0???_11??_????: begin                             // memory shift
            if (!eaok(ir_m, ir_r, M_MALT)) `ILLEGAL
            else begin sz <= SZ_W; g_src_ea <= 1'b0; g_after_dst <= S_SHIFTM; state <= G_DST; end
        end
        16'b1110_????_????_????: begin                             // register shift
            tmp <= D[ir_r];
            cnt6 <= sh_cnt;
            state <= S_SHIFT;
        end

        // ------------------------------------------------------------ A-line/F
        16'b1010_????_????_????: `EXC(8'd10, 4'h0, PCi, PCi)
        16'b1111_????_????_????: `EXC(8'd11, 4'h0, PCi, PCi)

        default: `ILLEGAL
        endcase
    end

    // =========================================================================
    // SR / CCR immediate ops
    // =========================================================================
    S_IMMSR: begin
        if (!iw_hit) `NEED_IW(S_IMMSR)
        else begin
            PC <= PC + 32'd2;
            case (IR[11:9])
                3'b000: tmp <= {16'd0, sr | (IR[6] ? iw : {8'd0, iw[7:0]})};
                3'b001: tmp <= {16'd0, sr & (IR[6] ? iw : {8'hFF, iw[7:0]})};
                default: tmp <= {16'd0, sr ^ (IR[6] ? iw : {8'd0, iw[7:0]})};
            endcase
            state <= S_IMMSR2;
        end
    end
    S_IMMSR2: begin
        if (IR[6]) sr <= tmp[15:0] & SR_MASK;
        else       sr[4:0] <= tmp[4:0];
        state <= S_END;
    end

    // =========================================================================
    // MOVEM
    // =========================================================================
    S_MOVEM0: begin
        if (!iw_hit) `NEED_IW(S_MOVEM0)
        else begin
            PC <= PC + 32'd2;
            mvm_mask <= iw;
            if (ir_m == 3'd4) begin               // -(An), reg -> mem
                mvm_addr <= R[{1'b1, ir_r}];
                tmp      <= R[{1'b1, ir_r}] - szbytes(sz);  // value stored for An itself (68020)
                state    <= S_MOVEM_PD;
            end else if (ir_m == 3'd3) begin      // (An)+, mem -> reg
                mvm_addr <= R[{1'b1, ir_r}];
                state    <= S_MOVEM_L;
            end else
                `CALL_EA(ir_m, ir_r, sz, S_MOVEM1)
        end
    end
    S_MOVEM1: begin
        mvm_addr <= ea_addr;
        state <= S_MOVEM_L;
    end
    S_MOVEM_L: begin
        if (mvm_mask == 16'd0) begin
            if (ir_m == 3'd3) `WR_A(ir_r, mvm_addr)
            state <= S_END;
        end else begin
            cnt6[3:0] <= lsb16(mvm_mask);
            mvm_mask[lsb16(mvm_mask)] <= 1'b0;
            mvm_addr <= mvm_addr + szbytes(sz);
            if (IR[10]) `MEMRD(mvm_addr, sz, S_MOVEM_LD)
            else        `MEMWR(mvm_addr, sz, R[lsb16(mvm_mask)], S_MOVEM_L)
        end
    end
    S_MOVEM_LD: begin
        `WR_R(cnt6[3:0], sxs(mem_rd, sz))
        state <= S_MOVEM_L;
    end
    S_MOVEM_PD: begin
        if (mvm_mask == 16'd0) begin
            `WR_A(ir_r, mvm_addr)
            state <= S_END;
        end else begin
            mvm_mask[lsb16(mvm_mask)] <= 1'b0;
            mvm_addr <= mvm_addr - szbytes(sz);
            if ((4'd15 - lsb16(mvm_mask)) == {1'b1, ir_r})
                `MEMWR(mvm_addr - szbytes(sz), sz, tmp, S_MOVEM_PD)
            else
                `MEMWR(mvm_addr - szbytes(sz), sz, R[4'd15 - lsb16(mvm_mask)], S_MOVEM_PD)
        end
    end

    // =========================================================================
    // MOVEP
    // =========================================================================
    S_MOVEP0: begin
        if (!iw_hit) `NEED_IW(S_MOVEP0)
        else begin
            PC <= PC + 32'd2;
            mvm_addr <= R[{1'b1, ir_r}] + sx16(iw);
            cnt3 <= IR[6] ? 3'd4 : 3'd2;
            tmp  <= IR[6] ? D[ir_x] : {D[ir_x][15:0], 16'd0};
            state <= S_MOVEP1;
        end
    end
    S_MOVEP1: begin
        if (cnt3 == 3'd0) begin
            if (!IR[7]) begin
                if (IR[6]) D[ir_x] <= tmp;
                else       D[ir_x][15:0] <= tmp[15:0];
            end
            state <= S_END;
        end else begin
            cnt3 <= cnt3 - 3'd1;
            mvm_addr <= mvm_addr + 32'd2;
            if (IR[7]) begin
                tmp <= {tmp[23:0], 8'd0};
                `MEMWR(mvm_addr, SZ_B, {24'd0, tmp[31:24]}, S_MOVEP1)
            end else
                `MEMRD(mvm_addr, SZ_B, S_MOVEP2)
        end
    end
    S_MOVEP2: begin
        tmp <= {tmp[23:0], mem_rd[7:0]};
        state <= S_MOVEP1;
    end

    // =========================================================================
    // Bcc / BRA / BSR
    // =========================================================================
    S_BCC0: begin
        if (IR[7:0] == 8'h00) state <= S_BCCW;
        else if (IR[7:0] == 8'hFF) state <= S_BCCL1;
        else begin bdisp <= sx8(IR[7:0]); state <= S_BCCGO; end
    end
    S_BCCW: begin
        if (!iw_hit) `NEED_IW(S_BCCW)
        else begin PC <= PC + 32'd2; bdisp <= sx16(iw); state <= S_BCCGO; end
    end
    S_BCCL1: begin
        if (!iw_hit) `NEED_IW(S_BCCL1)
        else begin PC <= PC + 32'd2; bdisp[31:16] <= iw; state <= S_BCCL2; end
    end
    S_BCCL2: begin
        if (!iw_hit) `NEED_IW(S_BCCL2)
        else begin PC <= PC + 32'd2; bdisp[15:0] <= iw; state <= S_BCCGO; end
    end
    S_BCCGO: begin
        if (IR[11:8] == 4'h1) begin           // BSR
            PC <= PCi + 32'd2 + bdisp;
            flow <= 1'b1;
            `PUSHL(PC, S_END)
        end else begin
            if (cond(IR[11:8], sr[4:0])) begin
                PC <= PCi + 32'd2 + bdisp;
                flow <= 1'b1;
            end
            state <= S_END;
        end
    end

    // =========================================================================
    // DBcc / TRAPcc
    // =========================================================================
    S_DBCC: begin
        if (!iw_hit) `NEED_IW(S_DBCC)
        else begin
            PC <= PC + 32'd2;
            if (!cond(IR[11:8], sr[4:0])) begin
                D[ir_r][15:0] <= D[ir_r][15:0] - 16'd1;
                if (D[ir_r][15:0] != 16'd0) begin
                    PC <= PCi + 32'd2 + sx16(iw);
                    flow <= 1'b1;
                end
            end
            state <= S_END;
        end
    end
    S_TRAPCC: begin
        if (ir_r == 3'd4) state <= S_TRAPCC2;
        else if (!iw_hit) `NEED_IW(S_TRAPCC)
        else begin
            PC <= PC + 32'd2;
            if (ir_r == 3'd3) IR[2:0] <= 3'd2;  // one more word to consume
            else state <= S_TRAPCC2;
        end
    end
    S_TRAPCC2: begin
        if (cond(IR[11:8], sr[4:0])) `EXCT(8'd7, 4'h2, PC, PCi)
        else state <= S_END;
    end

    // =========================================================================
    // JMP / JSR / LEA / PEA
    // =========================================================================
    S_JEA: begin
        PC <= ea_addr;
        flow <= 1'b1;
        state <= S_END;
    end
    S_JSR2: begin
        PC <= ea_addr;
        flow <= 1'b1;
        `PUSHL(PC, S_END)
    end
    S_LEA: begin
        `WR_A(ir_x, ea_addr)
        state <= S_END;
    end
    S_PEA: begin
        `PUSHL(ea_addr, S_END)
    end

    // =========================================================================
    // LINK / UNLK
    // =========================================================================
    S_LINK0: begin
        if (!iw_hit) `NEED_IW(S_LINK0)
        else begin PC <= PC + 32'd2; bdisp <= sx16(iw); state <= S_LINK1; end
    end
    S_LINKL: begin
        if (!iw_hit) `NEED_IW(S_LINKL)
        else begin
            PC <= PC + 32'd2;
            if (cnt3 != 3'd5) begin bdisp[31:16] <= iw; cnt3 <= 3'd5; end
            else begin bdisp[15:0] <= iw; cnt3 <= 3'd0; state <= S_LINK1; end
        end
    end
    S_LINK1: begin
        // LINK A7 stores the already-decremented stack pointer
        `PUSHL((ir_r == 3'd7) ? sp - 32'd4 : R[{1'b1, ir_r}], S_LINK2)
    end
    S_LINK2: begin
        if (ir_r != 3'd7) A[ir_r] <= sp;
        `WR_A(3'd7, sp + bdisp)
        state <= S_END;
    end
    S_UNLK1: begin
        if (ir_r == 3'd7) begin
            `WR_A(3'd7, mem_rd)
        end else begin
            A[ir_r] <= mem_rd;
            `WR_A(3'd7, sp + 32'd4)
        end
        state <= S_END;
    end

    // =========================================================================
    // RTS / RTD / RTR
    // =========================================================================
    S_RTS1: begin
        PC <= mem_rd;
        `WR_A(3'd7, sp + 32'd4)
        flow <= 1'b1;
        state <= S_END;
    end
    S_RTD0: begin
        if (!iw_hit) `NEED_IW(S_RTD0)
        else begin
            PC <= PC + 32'd2;
            bdisp <= sx16(iw);
            `MEMRD(sp, SZ_L, S_RTD1)
        end
    end
    S_RTD1: begin
        PC <= mem_rd;
        `WR_A(3'd7, sp + 32'd4 + bdisp)
        flow <= 1'b1;
        state <= S_END;
    end
    S_RTR1: begin
        sr[4:0] <= mem_rd[4:0];
        `MEMRD(sp + 32'd2, SZ_L, S_RTR2)
    end
    S_RTR2: begin
        PC <= mem_rd;
        `WR_A(3'd7, sp + 32'd6)
        flow <= 1'b1;
        state <= S_END;
    end

    // =========================================================================
    // RTE
    // =========================================================================
    S_RTE0: begin
        `MEMRD(sp, SZ_W, S_RTE1)
    end
    S_RTE1: begin
        tmp2[15:0] <= mem_rd[15:0];               // new SR
        `MEMRD(sp + 32'd2, SZ_L, S_RTE2)
    end
    S_RTE2: begin
        tmp <= mem_rd;                             // new PC
        `MEMRD(sp + 32'd6, SZ_W, S_RTE3)
    end
    S_RTE3: begin
        case (mem_rd[15:12])
            4'h0, 4'h1: begin `WR_A(3'd7, sp + 32'd8)  state <= S_RTE4; end
            4'h2:       begin `WR_A(3'd7, sp + 32'd12) state <= S_RTE4; end
            4'h9:       begin `WR_A(3'd7, sp + 32'd20) state <= S_RTE4; end
            4'hA:       begin `WR_A(3'd7, sp + 32'd32) state <= S_RTE4; end
            4'hB:       begin `WR_A(3'd7, sp + 32'd92) state <= S_RTE4; end
            default:    `EXC(8'd14, 4'h0, PCi, PCi)
        endcase
        cnt3 <= {2'd0, mem_rd[15:12] == 4'h1};
    end
    S_RTE4: begin
        sr <= tmp2[15:0] & SR_MASK;
        if (cnt3[0]) state <= S_RTE0;              // throwaway frame: continue on new stack
        else begin
            PC <= tmp;
            flow <= 1'b1;
            state <= S_END;
        end
    end

    // =========================================================================
    // CHK / CHK2 / CMP2
    // =========================================================================
    S_CHK: begin
        // src = upper bound, compare Dn (sized, signed)
        if ($signed(sxs(D[ir_x], sz)) < 0) begin
            sr[3] <= 1'b1;
            `EXCT(8'd6, 4'h2, PC, PCi)
        end else if ($signed(sxs(D[ir_x], sz)) > $signed(sxs(src, sz))) begin
            sr[3] <= 1'b0;
            `EXCT(8'd6, 4'h2, PC, PCi)
        end else
            state <= S_END;
        sr[2] <= (zxs(D[ir_x], sz) == 32'd0);
        sr[1:0] <= 2'b00;
    end
    S_CHK2X: begin
        if (!iw_hit) `NEED_IW(S_CHK2X)
        else begin
            PC <= PC + 32'd2;
            ext <= iw;
            `CALL_EA(ir_m, ir_r, sz, S_CHK2E)
        end
    end
    S_CHK2E: begin
        `MEMRD(ea_addr, sz, S_CHK2R1)
    end
    S_CHK2R1: begin
        src <= sxs(mem_rd, sz);                    // lower bound
        `MEMRD(ea_addr + szbytes(sz), sz, S_CHK2R2)
    end
    S_CHK2R2: begin : chk2_blk
        logic [31:0] lo, hi, rv;
        logic        out;
        lo = src;
        hi = sxs(mem_rd, sz);
        rv = ext[15] ? R[ext[15:12]] : sxs(R[ext[15:12]], sz);
        out = ($signed(lo) <= $signed(hi)) ? ($signed(rv) < $signed(lo) || $signed(rv) > $signed(hi))
                                           : ($signed(rv) > $signed(hi) && $signed(rv) < $signed(lo));
        sr[2] <= (rv == lo) || (rv == hi);
        sr[0] <= out;
        if (out && ext[11]) `EXCT(8'd6, 4'h2, PC, PCi)
        else state <= S_END;
    end

    // =========================================================================
    // MUL
    // =========================================================================
    S_MULW: begin
        // 16x16 -> 32 (single multiplier pass)
        md_signed <= IR[8];
        md_a <= IR[8] ? (src[15] ? {16'd0, -src[15:0]} : {16'd0, src[15:0]}) : {16'd0, src[15:0]};
        md_b <= IR[8] ? (D[ir_x][15] ? {16'd0, -D[ir_x][15:0]} : {16'd0, D[ir_x][15:0]}) : {16'd0, D[ir_x][15:0]};
        md_neg <= IR[8] && (src[15] ^ D[ir_x][15]);
        md_word <= 1'b1;
        md_long64 <= 1'b0;
        md_acc <= 64'd0;
        md_step <= 2'd0;
        state <= S_MUL_LOOP;
    end
    S_MULLX: begin
        if (!iw_hit) `NEED_IW(S_MULLX)
        else begin
            PC <= PC + 32'd2;
            ext <= iw;
            `CALL_EA(ir_m, ir_r, SZ_L, G_SRC_RD)
            g_after_src <= S_MULL;
        end
    end
    S_MULL: begin
        md_signed <= ext[11];
        md_a <= (ext[11] && src[31]) ? -src : src;
        md_b <= (ext[11] && D[ext[14:12]][31]) ? -D[ext[14:12]] : D[ext[14:12]];
        md_neg <= ext[11] && (src[31] ^ D[ext[14:12]][31]);
        md_word <= 1'b0;
        md_long64 <= ext[10];
        md_acc <= 64'd0;
        md_step <= 2'd0;
        state <= S_MUL_LOOP;
    end
    S_MUL_LOOP: begin
        // one 16x16 partial product per pass (product registered through mul_x/mul_y)
        case (md_step)
            2'd0: begin mul_x <= md_a[15:0];  mul_y <= md_b[15:0];  end
            2'd1: begin mul_x <= md_a[31:16]; mul_y <= md_b[15:0];  end
            2'd2: begin mul_x <= md_a[15:0];  mul_y <= md_b[31:16]; end
            default: begin mul_x <= md_a[31:16]; mul_y <= md_b[31:16]; end
        endcase
        cnt3 <= {1'b0, md_step};
        md_step <= md_step + 2'd1;
        if (md_step != 2'd0) begin
            // accumulate previous product
            case (cnt3[1:0])
                2'd0: md_acc <= md_acc + {32'd0, mul_p};
                2'd1, 2'd2: md_acc <= md_acc + {16'd0, mul_p, 16'd0};
                default: md_acc <= md_acc + {mul_p, 32'd0};
            endcase
        end
        if (md_word || md_step == 2'd3) state <= S_MUL_FIN;
    end
    S_MUL_FIN: begin : mulfin
        logic [63:0] p, acc;
        // add the final outstanding product
        case (cnt3[1:0])
            2'd0: acc = md_acc + {32'd0, mul_p};
            2'd1, 2'd2: acc = md_acc + {16'd0, mul_p, 16'd0};
            default: acc = md_acc + {mul_p, 32'd0};
        endcase
        p = md_neg ? -acc : acc;
        if (md_word) begin
            D[ir_x] <= p[31:0];
            sr[3] <= p[31];
            sr[2] <= (p[31:0] == 32'd0);
            sr[1:0] <= 2'b00;
        end else if (md_long64) begin
            D[ext[2:0]] <= p[63:32];
            D[ext[14:12]] <= p[31:0];
            sr[3] <= p[63];
            sr[2] <= (p == 64'd0);
            sr[1:0] <= 2'b00;
        end else begin
            D[ext[14:12]] <= p[31:0];
            sr[3] <= p[31];
            sr[2] <= (p[31:0] == 32'd0);
            sr[1] <= md_signed ? (p[63:32] != {32{p[31]}}) : (p[63:32] != 32'd0);
            sr[0] <= 1'b0;
        end
        state <= S_END;
    end

    // =========================================================================
    // DIV
    // =========================================================================
    S_DIVW: begin
        if (src[15:0] == 16'd0) begin
            sr[0] <= 1'b0;
            `EXCT(8'd5, 4'h2, PC, PCi)
        end else begin
            md_signed <= IR[8];
            md_word   <= 1'b1;
            md_long64 <= 1'b0;
            // dividend magnitude in md_acc, divisor magnitude in md_b
            md_acc <= {32'd0, (IR[8] && D[ir_x][31]) ? -D[ir_x] : D[ir_x]};
            md_b   <= IR[8] ? (src[15] ? {16'd0, -src[15:0]} : {16'd0, src[15:0]}) : {16'd0, src[15:0]};
            md_neg <= IR[8] && (D[ir_x][31] ^ src[15]);
            md_rneg <= IR[8] && D[ir_x][31];
            state <= S_DIV_LOOP;
            md_cnt <= 6'd0;
        end
    end
    S_DIVLX: begin
        if (!iw_hit) `NEED_IW(S_DIVLX)
        else begin
            PC <= PC + 32'd2;
            ext <= iw;
            `CALL_EA(ir_m, ir_r, SZ_L, G_SRC_RD)
            g_after_src <= S_DIVL;
        end
    end
    S_DIVL: begin : divl_blk
        logic [63:0] dvd;
        logic        dneg;
        if (src == 32'd0) begin
            sr[0] <= 1'b0;
            `EXCT(8'd5, 4'h2, PC, PCi)
        end else begin
            dvd  = ext[10] ? {D[ext[2:0]], D[ext[14:12]]} : (ext[11] ? {{32{D[ext[14:12]][31]}}, D[ext[14:12]]} : {32'd0, D[ext[14:12]]});
            dneg = ext[11] && dvd[63];
            md_signed <= ext[11];
            md_word   <= 1'b0;
            md_long64 <= ext[10];
            md_acc <= dneg ? -dvd : dvd;
            md_b   <= (ext[11] && src[31]) ? -src : src;
            md_neg <= ext[11] && (dneg ^ src[31]);
            md_rneg <= dneg;
            md_cnt <= 6'd0;
            state <= S_DIV_LOOP;
        end
    end
    S_DIV_LOOP: begin : divloop
        logic [32:0] r2;
        if (md_cnt == 6'd0) begin
            // overflow pre-check: high half must be < divisor
            if (md_acc[63:32] >= md_b) begin
                sr[1] <= 1'b1; sr[0] <= 1'b0;
                state <= S_END;
            end else begin
                md_rem <= {1'b0, md_acc[63:32]};
                md_q   <= md_acc[31:0];
                md_cnt <= 6'd1;
            end
        end else begin
            // restoring division step: shift in next dividend bit (held in md_q MSB)
            r2 = {md_rem[31:0], md_q[31]};
            if (r2 >= {1'b0, md_b}) begin
                md_rem <= r2 - {1'b0, md_b};
                md_q   <= {md_q[30:0], 1'b1};
            end else begin
                md_rem <= r2;
                md_q   <= {md_q[30:0], 1'b0};
            end
            md_cnt <= md_cnt + 6'd1;
            if (md_cnt == 6'd32) state <= S_DIV_FIN;
        end
    end
    S_DIV_FIN: begin : divfin
        logic [31:0] q, r;
        logic        ovf;
        q = md_neg ? -md_q : md_q;
        r = md_rneg ? -md_rem[31:0] : md_rem[31:0];
        if (md_word)
            ovf = md_signed ? (md_neg ? (md_q > 32'h8000) : (md_q > 32'h7FFF)) : (md_q > 32'hFFFF);
        else
            ovf = md_signed ? (md_neg ? (md_q > 32'h8000_0000) : (md_q > 32'h7FFF_FFFF)) : 1'b0;
        sr[0] <= 1'b0;
        if (ovf) begin
            sr[1] <= 1'b1;
        end else begin
            sr[1] <= 1'b0;
            if (md_word) begin
                D[ir_x] <= {r[15:0], q[15:0]};
                sr[3] <= q[15];
                sr[2] <= (q[15:0] == 16'd0);
            end else begin
                if (md_long64 || ext[2:0] != ext[14:12]) D[ext[2:0]] <= r;
                D[ext[14:12]] <= q;
                sr[3] <= q[31];
                sr[2] <= (q == 32'd0);
            end
        end
        state <= S_END;
    end

    // =========================================================================
    // shifts / rotates
    // =========================================================================
    S_SHIFT: begin : shift_blk
        logic [36:0] so;
        if (IR[4:3] == 2'b10) begin
            // ROXL / ROXR : iterative
            case (ir_sz)
                SZ_B: cnt6 <= cnt6 % 6'd9;
                SZ_W: cnt6 <= cnt6 % 6'd17;
                default: cnt6 <= cnt6 % 6'd33;
            endcase
            sz <= ir_sz;
            sr[0] <= sr[4];
            state <= S_ROX;
        end else begin
            so = shifter(IR[4:3], IR[8], ir_sz, cnt6, tmp, sr[4:0]);
            D[ir_r] <= merge(D[ir_r], so[31:0], ir_sz);
            sr[4:0] <= so[36:32];
            state <= S_END;
        end
    end
    S_ROX: begin
        if (cnt6 == 6'd0) begin
            D[ir_r] <= merge(D[ir_r], tmp, sz);
            sr[3] <= msbs(tmp, sz);
            sr[2] <= (zxs(tmp, sz) == 32'd0);
            sr[1] <= 1'b0;
            sr[0] <= sr[4];
            state <= S_END;
        end else begin
            cnt6 <= cnt6 - 6'd1;
            if (IR[8]) begin   // left
                sr[4] <= msbs(tmp, sz);
                tmp <= {tmp[30:0], sr[4]};
            end else begin
                sr[4] <= tmp[0];
                case (sz)
                    SZ_B: tmp <= {24'd0, sr[4], tmp[7:1]};
                    SZ_W: tmp <= {16'd0, sr[4], tmp[15:1]};
                    default: tmp <= {sr[4], tmp[31:1]};
                endcase
            end
        end
    end
    S_SHIFTM: begin : shiftm_blk
        logic [36:0] so;
        logic [31:0] v;
        if (IR[10:9] == 2'b10) begin
            // ROXL/ROXR #1 word
            v = IR[8] ? {16'd0, dst[14:0], sr[4]} : {16'd0, sr[4], dst[15:1]};
            res <= v;
            sr[4] <= IR[8] ? dst[15] : dst[0];
            sr[0] <= IR[8] ? dst[15] : dst[0];
            sr[3] <= v[15];
            sr[2] <= (v[15:0] == 16'd0);
            sr[1] <= 1'b0;
        end else begin
            so = shifter(IR[10:9], IR[8], SZ_W, 6'd1, dst, sr[4:0]);
            res <= so[31:0];
            sr[4:0] <= so[36:32];
        end
        state <= G_WB;
    end

    // =========================================================================
    // bit operations
    // =========================================================================
    S_BITX: begin   // static: fetch bit number
        if (!iw_hit) `NEED_IW(S_BITX)
        else begin
            PC <= PC + 32'd2;
            src <= {24'd0, iw[7:0]};
            state <= S_BITD;
        end
    end
    S_BITD: begin
        sz <= (ir_m == 3'd0) ? SZ_L : SZ_B;
        `CALL_EA(ir_m, ir_r, (ir_m == 3'd0) ? SZ_L : SZ_B, G_DST_RD)
        g_rd <= 1'b1;
        g_after_dst <= S_BIT;
    end
    S_BIT: begin : bit_blk
        logic [4:0]  bn;
        logic [31:0] m;
        bn = (dst_kind == K_D) ? src[4:0] : {2'b00, src[2:0]};
        m  = 32'd1 << bn;
        sr[2] <= !(|(dst & m));
        case (IR[7:6])
            2'b01: res <= dst ^ m;
            2'b10: res <= dst & ~m;
            2'b11: res <= dst | m;
            default: res <= dst;
        endcase
        state <= (IR[7:6] == 2'b00) ? S_END : G_WB;
    end

    // =========================================================================
    // bit fields
    // =========================================================================
    S_BFX: begin
        if (!iw_hit) `NEED_IW(S_BFX)
        else begin
            PC <= PC + 32'd2;
            ext <= iw;
            `CALL_EA(ir_m, ir_r, SZ_L, S_BFE)
        end
    end
    S_BFE: begin
        bf_off <= ext[11] ? D[ext[8:6]] : {27'd0, ext[10:6]};
        bf_w   <= ext[5] ? ((D[ext[2:0]][4:0] == 5'd0) ? 6'd32 : {1'b0, D[ext[2:0]][4:0]})
                         : ((ext[4:0] == 5'd0) ? 6'd32 : {1'b0, ext[4:0]});
        state  <= S_BF0;
    end
    S_BF0: begin : bf0_blk
        logic [31:0] rot, nrot, ins, fld;
        logic [5:0]  i;
        logic        found;
        if (ea_kind == K_D) begin
            rot = bf_rot64[63:32];
            fld = rot & bf_w32;
            ins = D[ext[14:12]] << (6'd32 - bf_w);
            sr[3] <= rot[31];
            sr[2] <= (fld == 32'd0);
            sr[1:0] <= 2'b00;
            case (IR[10:8])
                3'b010: nrot = rot ^ bf_w32;                       // BFCHG
                3'b100: nrot = rot & ~bf_w32;                      // BFCLR
                3'b110: nrot = rot | bf_w32;                       // BFSET
                3'b111: begin                                      // BFINS
                    nrot = (rot & ~bf_w32) | (ins & bf_w32);
                    sr[3] <= ins[31];
                    sr[2] <= ((ins & bf_w32) == 32'd0);
                end
                default: nrot = rot;
            endcase
            if (IR[10:8] == 3'b010 || IR[10:8] == 3'b100 || IR[10:8] == 3'b110 || IR[10:8] == 3'b111)
                D[ir_r] <= ({nrot, nrot} >> bf_ro) ;   // rotate right back
            // extract / find first one
            case (IR[10:8])
                3'b001: D[ext[14:12]] <= fld >> (6'd32 - bf_w);                       // BFEXTU
                3'b011: D[ext[14:12]] <= $signed(fld) >>> (6'd32 - bf_w);             // BFEXTS
                3'b101: begin                                                           // BFFFO
                    found = 1'b0; i = bf_w;
                    for (int k = 31; k >= 0; k--)
                        if (!found && fld[k]) begin found = 1'b1; i = 6'd31 - k[5:0]; end
                    D[ext[14:12]] <= {27'd0, bf_off[4:0]} + {26'd0, i};
                end
                default: ;
            endcase
            state <= S_END;
        end else begin
            // bytes touched: 1 -> byte, 2 -> word, 3 -> word+byte, 4 -> long, 5 -> long+byte
            bf_addr <= ea_addr + {{3{bf_off[31]}}, bf_off[31:3]};
            bf_bc   <= bf_bcount;
            bf_buf  <= 40'd0;
            `MEMRD(ea_addr + {{3{bf_off[31]}}, bf_off[31:3]},
                   (bf_bcount == 3'd1) ? SZ_B : (bf_bcount <= 3'd3) ? SZ_W : SZ_L, S_BF1)
        end
    end
    S_BF1: begin
        case (bf_bc)
            3'd1: bf_buf[39:32] <= mem_rd[7:0];
            3'd2, 3'd3: bf_buf[39:24] <= mem_rd[15:0];
            default: bf_buf[39:8] <= mem_rd;
        endcase
        if (bf_bc == 3'd3)      `MEMRD(bf_addr + 32'd2, SZ_B, S_BF2)
        else if (bf_bc == 3'd5) `MEMRD(bf_addr + 32'd4, SZ_B, S_BF2)
        else state <= S_BF3;
    end
    S_BF2: begin
        if (bf_bc == 3'd3) bf_buf[23:16] <= mem_rd[7:0];
        else               bf_buf[7:0]   <= mem_rd[7:0];
        state <= S_BF3;
    end
    S_BF3: begin : bf3_blk
        logic [39:0] sh, msk, nb, insv;
        logic [31:0] fld, ins;
        logic [5:0]  i;
        logic        found;
        sh  = bf_buf << bf_off[2:0];
        fld = sh[39:8] & bf_w32;
        msk = {bf_w32, 8'd0} >> bf_off[2:0];
        ins = D[ext[14:12]] << (6'd32 - bf_w);
        insv = {ins & bf_w32, 8'd0} >> bf_off[2:0];
        sr[3] <= sh[39];
        sr[2] <= (fld == 32'd0);
        sr[1:0] <= 2'b00;
        case (IR[10:8])
            3'b010: nb = bf_buf ^ msk;
            3'b100: nb = bf_buf & ~msk;
            3'b110: nb = bf_buf | msk;
            3'b111: begin
                nb = (bf_buf & ~msk) | insv;
                sr[3] <= ins[31];
                sr[2] <= ((ins & bf_w32) == 32'd0);
            end
            default: nb = bf_buf;
        endcase
        bf_buf <= nb;
        case (IR[10:8])
            3'b001: D[ext[14:12]] <= fld >> (6'd32 - bf_w);
            3'b011: D[ext[14:12]] <= $signed(fld) >>> (6'd32 - bf_w);
            3'b101: begin
                found = 1'b0; i = bf_w;
                for (int k = 31; k >= 0; k--)
                    if (!found && fld[k]) begin found = 1'b1; i = 6'd31 - k[5:0]; end
                D[ext[14:12]] <= bf_off + {26'd0, i};
            end
            default: ;
        endcase
        if (IR[10:8] == 3'b010 || IR[10:8] == 3'b100 || IR[10:8] == 3'b110 || IR[10:8] == 3'b111) begin
            case (bf_bc)
                3'd1:       `MEMWR(bf_addr, SZ_B, {24'd0, nb[39:32]}, S_END)
                3'd2:       `MEMWR(bf_addr, SZ_W, {16'd0, nb[39:24]}, S_END)
                3'd3:       `MEMWR(bf_addr, SZ_W, {16'd0, nb[39:24]}, S_BF4)
                3'd4:       `MEMWR(bf_addr, SZ_L, nb[39:8], S_END)
                default:    `MEMWR(bf_addr, SZ_L, nb[39:8], S_BF4)
            endcase
        end else
            state <= S_END;
    end
    S_BF4: begin
        if (bf_bc == 3'd3) `MEMWR(bf_addr + 32'd2, SZ_B, {24'd0, bf_buf[23:16]}, S_END)
        else               `MEMWR(bf_addr + 32'd4, SZ_B, {24'd0, bf_buf[7:0]}, S_END)
    end

    // =========================================================================
    // CAS / CAS2
    // =========================================================================
    S_CASX: begin
        if (!iw_hit) `NEED_IW(S_CASX)
        else begin
            PC <= PC + 32'd2;
            ext <= iw;
            bus_lock <= 1'b1;
            `CALL_EA(ir_m, ir_r, sz, S_CASE)
        end
    end
    S_CASE: begin
        `MEMRD(ea_addr, sz, S_CAS)
    end
    S_CAS: begin : cas_blk
        logic [37:0] o;
        o = alu(A_CMP, sz, D[ext[2:0]], mem_rd, sr[4:0], 1'b0);
        sr[4:0] <= o[36:32];
        if (o[34]) `MEMWR(ea_addr, sz, D[ext[8:6]], S_END)
        else begin
            D[ext[2:0]] <= merge(D[ext[2:0]], mem_rd, sz);
            state <= S_END;
        end
    end
    S_CAS2X1: begin
        if (!iw_hit) `NEED_IW(S_CAS2X1)
        else begin PC <= PC + 32'd2; ext <= iw; state <= S_CAS2X2; end
    end
    S_CAS2X2: begin
        if (!iw_hit) `NEED_IW(S_CAS2X2)
        else begin
            PC <= PC + 32'd2;
            ext2 <= iw;
            `MEMRD(R[ext[15:12]], sz, S_CAS2R1)
        end
    end
    S_CAS2R1: begin
        tmp <= mem_rd;
        `MEMRD(R[ext2[15:12]], sz, S_CAS2R2)
    end
    S_CAS2R2: begin
        tmp2 <= mem_rd;
        state <= S_CAS2C;
    end
    S_CAS2C: begin : cas2_blk
        logic [37:0] o1, o2;
        o1 = alu(A_CMP, sz, D[ext[2:0]], tmp, sr[4:0], 1'b0);
        o2 = alu(A_CMP, sz, D[ext2[2:0]], tmp2, sr[4:0], 1'b0);
        if (!o1[34]) begin
            sr[4:0] <= o1[36:32];
            D[ext2[2:0]] <= merge(D[ext2[2:0]], tmp2, sz);
            state <= S_CAS2W2;      // second register write (Dc1 wins)
        end else if (!o2[34]) begin
            sr[4:0] <= o2[36:32];
            D[ext2[2:0]] <= merge(D[ext2[2:0]], tmp2, sz);
            state <= S_CAS2W2;
        end else begin
            sr[4:0] <= o2[36:32];
            `MEMWR(R[ext[15:12]], sz, D[ext[8:6]], S_CAS2W1)
        end
    end
    S_CAS2W1: begin
        `MEMWR(R[ext2[15:12]], sz, D[ext2[8:6]], S_END)
    end
    S_CAS2W2: begin
        D[ext[2:0]] <= merge(D[ext[2:0]], tmp, sz);
        state <= S_END;
    end

    // =========================================================================
    // PACK / UNPK  (also: EXG Ax,Ay second half)
    // =========================================================================
    S_PACKX: begin
        if (!iw_hit) `NEED_IW(S_PACKX)
        else begin
            PC <= PC + 32'd2;
            bdisp <= {16'd0, iw};
            if (!IR[3]) begin
                if (!IR[7]) begin   // PACK Dx,Dy
                    D[ir_x][7:0] <= pack_b(D[ir_r][15:0] + iw);
                end else begin      // UNPK Dx,Dy
                    D[ir_x][15:0] <= {4'd0, D[ir_r][7:4], 4'd0, D[ir_r][3:0]} + iw;
                end
                state <= S_END;
            end else begin
                `CALL_EA(3'd4, ir_r, SZ_B, IR[7] ? S_UNPKM1 : S_PACKM1)
            end
        end
    end
    S_PACKM1: begin
        `MEMRD(ea_addr, SZ_B, S_PACKM2)          // low byte (higher address)
    end
    S_PACKM2: begin
        // first byte read (higher address) is the low byte of the unpacked word
        if (cnt3 == 3'd0) begin
            tmp2[7:0] <= mem_rd[7:0];
            cnt3 <= 3'd1;
            `CALL_EA(3'd4, ir_r, SZ_B, S_PACKM1)
        end else begin
            tmp2[15:8] <= mem_rd[7:0];
            cnt3 <= 3'd0;
            state <= S_PACKM3;
        end
    end
    S_PACKM3: begin
        res <= {24'd0, pack_b(tmp2[15:0] + bdisp[15:0])};
        sz <= SZ_B;
        g_rd <= 1'b0;
        g_after_dst <= G_WB;
        `CALL_EA(3'd4, ir_x, SZ_B, G_DST_RD)
    end
    S_UNPKM1: begin
        `MEMRD(ea_addr, SZ_B, S_UNPKM2)
    end
    S_UNPKM2: begin
        tmp2 <= {16'd0, ({4'd0, mem_rd[7:4], 4'd0, mem_rd[3:0]} + bdisp[15:0])};
        `CALL_EA(3'd4, ir_x, SZ_B, S_UNPKM3)
    end
    S_UNPKM3: begin
        // low byte first at -(Ay), then high byte at -(Ay)
        if (cnt3 == 3'd0) begin
            cnt3 <= 3'd1;
            `MEMWR(ea_addr, SZ_B, {24'd0, tmp2[7:0]}, S_UNPKM4)
        end else begin
            cnt3 <= 3'd0;
            `MEMWR(ea_addr, SZ_B, {24'd0, tmp2[15:8]}, S_END)
        end
    end
    S_UNPKM4: begin
        `CALL_EA(3'd4, ir_x, SZ_B, S_UNPKM3)
    end

    // =========================================================================
    // MOVEC / MOVES
    // =========================================================================
    S_MOVECX: begin
        if (!iw_hit) `NEED_IW(S_MOVECX)
        else begin
            PC <= PC + 32'd2;
            state <= S_END;
            if (IR[0]) begin
                // Rn -> Rc
                case (iw[11:0])
                    12'h000: sfc <= R[iw[15:12]][2:0];
                    12'h001: dfc <= R[iw[15:12]][2:0];
                    12'h002: begin
                        cacr <= {30'd0, R[iw[15:12]][1:0]};
                        if (R[iw[15:12]][3]) ic_valid <= 64'd0;
                        else if (R[iw[15:12]][2]) ic_valid[caar[7:2]] <= 1'b0;
                    end
                    12'h800: usp <= R[iw[15:12]];
                    12'h801: vbr <= R[iw[15:12]];
                    12'h802: caar <= R[iw[15:12]];
                    12'h803: msp <= R[iw[15:12]];
                    12'h804: isp <= R[iw[15:12]];
                    default: `ILLEGAL
                endcase
            end else begin
                case (iw[11:0])
                    12'h000, 12'h001, 12'h002, 12'h800, 12'h801, 12'h802, 12'h803, 12'h804: begin
                        `WR_R(iw[15:12], movec_rd)
                    end
                    default: `ILLEGAL
                endcase
            end
        end
    end
    S_MOVESX: begin
        if (!iw_hit) `NEED_IW(S_MOVESX)
        else begin
            PC <= PC + 32'd2;
            ext <= iw;
            `CALL_EA(ir_m, ir_r, sz, S_MOVESE)
        end
    end
    S_MOVESE: begin
        if (ext[11]) `MEMX(ea_addr, sz, 1'b1, R[ext[15:12]], dfc, S_END)   // 68020 stores post-EA An
        else         `MEMX(ea_addr, sz, 1'b0, 32'd0, sfc, S_MOVES1)
    end
    S_MOVES1: begin
        if (ext[15]) `WR_A(ext[14:12], sxs(mem_rd, sz))
        else D[ext[14:12]] <= merge(D[ext[14:12]], mem_rd, sz);
        state <= S_END;
    end

    // =========================================================================
    // STOP / RESET
    // =========================================================================
    S_STOP0: begin
        if (!iw_hit) `NEED_IW(S_STOP0)
        else begin
            PC <= PC + 32'd2;
            sr <= iw & SR_MASK;
            stop_pending <= 1'b1;
            state <= S_END;
        end
    end
    S_RESETI: begin
        reset_out <= 1'b1;
        reset_cnt <= reset_cnt + 32'd1;
        if (reset_cnt == RESET_CYCLES) begin
            reset_out <= 1'b0;
            state <= S_END;
        end
    end

    // =========================================================================
    // interrupt acknowledge
    // =========================================================================
    S_INT0: begin
        exc_lvl  <= ipl_q;
        in_iack  <= 1'b1;
        if (ipl_q == 3'd7) nmi_latch <= 1'b0;
        PCi      <= PC;
        trace_t1 <= 1'b0;
        trace_t0 <= 1'b0;
        `MEMX(32'hFFFF_FFF1 | {28'd0, ipl_q, 1'b0}, SZ_B, 1'b0, 32'd0, 3'd7, S_INT1)
    end
    S_INT1: begin
        in_iack <= 1'b0;
        exc_vec <= mem_avec ? (8'd24 + {5'd0, exc_lvl}) : mem_rd[7:0];
        exc_fmt <= 4'h0;
        exc_pc  <= PC;
        exc_addr <= PC;
        exc_int <= 1'b1;
        state   <= X_START;
    end

    // =========================================================================
    // exception processing
    // =========================================================================
    X_START: begin
        x_sr <= sr;
        x_m  <= sr[12];
        sr[15:14] <= 2'b00;
        sr[13] <= 1'b1;
        if (exc_int) sr[10:8] <= exc_lvl;
        in_exc <= 1'b1;
        bus_lock <= 1'b0;
        x_cnt <= 4'd11;
        case (exc_fmt)
            4'h2: state <= X_F2;
            4'hA: state <= X_FA;
            default: state <= X_FV;
        endcase
    end
    X_F2: begin
        `PUSHL(exc_addr, X_FV)
    end
    X_FA: begin
        // words at frame offsets 30 down to 8 (x_cnt 11..0 -> offset 8 + 2*x_cnt)
        x_cnt <= x_cnt - 4'd1;
        if (x_cnt == 4'd0) `PUSHW({16'd0, xfa_word}, X_FV)
        else               `PUSHW({16'd0, xfa_word}, X_FA)
    end
    X_FV: begin
        `PUSHW({exc_fmt, 2'b00, exc_vec, 2'b00}, X_PC)
    end
    X_PC: begin
        `PUSHL(exc_pc, X_SR)
    end
    X_SR: begin
        `PUSHW({16'd0, x_sr}, X_VEC)
    end
    X_VEC: begin
        `MEMX(vbr + {22'd0, exc_vec, 2'b00}, SZ_L, 1'b0, 32'd0, 3'd5, X_VEC2)
    end
    X_VEC2: begin
        PC <= mem_rd;
        if (exc_int && x_m) begin
            sr[12] <= 1'b0;
            state <= X_T1;
        end else
            state <= X_DONE;
    end
    X_T1: begin `PUSHW({4'h1, 2'b00, exc_vec, 2'b00}, X_T2) end
    X_T2: begin `PUSHL(exc_pc, X_T3) end
    X_T3: begin `PUSHW({16'd0, x_sr | 16'h2000}, X_DONE) end
    X_DONE: begin
        in_exc <= 1'b0;
        state  <= S_END;
    end

    default: state <= S_HALT;
    endcase
    end
end

endmodule
