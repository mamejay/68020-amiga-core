// =============================================================================
// m68020_core.sv - 68020-compatible CPU core (ISA level, not cycle accurate)
//
// Written for the Base64 68000-socket accelerator (jbilander/Base64) as an
// original implementation. Architecture:
//   * multi-cycle FSM with shared effective-address and bus subroutines
//   * register file in distributed RAM: one (registered) write port with
//     bypass, three general read ports + stack-pointer port
//   * one ALU, one funnel shifter, one 16x16 multiplier, one 64-bit
//     multiply/divide accumulator adder, all shared between instructions
//   * unified 32-bit request/ack bus, misaligned data allowed (doc/core_interface.md)
//   * 64-entry x 32-bit direct-mapped instruction cache (68020 compatible CACR)
//     plus a one-longword prefetch buffer
//   * full 68020 user + supervisor ISA except CALLM/RTM and coprocessor
//     instructions (F-line trap); bus errors restart the instruction
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
    parameter bit ICACHE       = 1'b1,   // implement 64-entry instruction cache
    parameter bit DEBUG        = 1'b1    // drive the dbg_* register outputs (shadow copy)
) (
    input  logic         clk,
    input  logic         ce,          // clock enable (tie 1 to run every clk)
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
localparam [4:0] P_USP = 5'd15, P_ISP = 5'd16, P_MSP = 5'd17;      // physical RF slots

// EA class bits (one-hot index)
localparam int C_DN=0, C_AN=1, C_IND=2, C_PI=3, C_PD=4, C_D16=5, C_IDX=6,
               C_ABSW=7, C_ABSL=8, C_PCD=9, C_PCX=10, C_IMM=11;
localparam [11:0] M_ALL      = 12'hFFF;
localparam [11:0] M_DATA     = 12'hFFD;
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
    S_CLR, S_RESET, S_RST1, S_RST2,
    S_FETCH, S_DECODE, S_END, S_STOPPED, S_HALT,
    S_MEM, S_IFETCH,
    // effective address engine
    E_START, E_D16, E_ABSW, E_ABSL1, E_ABSL2, E_IMM1, E_IMM2, E_IDX, E_BDW, E_BDL1, E_BDL2,
    E_IDX2, E_FULL, E_ODW, E_ODL1, E_ODL2, E_MI, E_MI2,
    // generic ALU flow
    G_SRC, G_SRC_RD, G_SRC_RD2, G_DST, G_DST_RD, G_DST_RD2, G_EXEC, G_WB,
    // misc
    S_EXG2, S_IMMSR, S_IMMSR2, S_MOVEM0, S_MOVEM1, S_MOVEM_L, S_MOVEM_LD, S_MOVEM_PD,
    S_MOVEP0, S_MOVEP1, S_MOVEP2,
    S_BCC0, S_BCCW, S_BCCL1, S_BCCL2, S_BCCGO,
    S_DBCC, S_TRAPCC, S_TRAPCC2,
    S_JEA, S_JSR2, S_LEA, S_PEA,
    S_LINK0, S_LINKL, S_LINK1, S_LINK2, S_LINK3, S_UNLK1, S_UNLK2,
    S_RTS1, S_RTD0, S_RTD1, S_RTR1, S_RTR2,
    S_RTE0, S_RTE1, S_RTE2, S_RTE3, S_RTE4,
    S_CHK, S_CHK1W, S_CHK1, S_CHK2X, S_CHK2E, S_CHK2R1, S_CHK2R2, S_CHK2C1, S_CHK2C2, S_CHK2C3, S_CHK2C4,
    S_MULW, S_MULLX, S_MULL, S_MUL_LOOP, S_MUL_FIN, S_MUL_FIN2,
    S_DIVW, S_DIVLX, S_DIVL, S_DIV_ABS1, S_DIV_ABS2, S_DIV_LOOP, S_DIV_FIN, S_DIV_FIN2,
    S_SHIFT, S_ROX, S_SHIFTM,
    S_BITX, S_BITD, S_BIT,
    S_BFX, S_BFE, S_BF0, S_BF1, S_BF2, S_BF3, S_BF4, S_BF5, S_BF6,
    S_CASX, S_CASE, S_CAS, S_CAS1W, S_CAS1,
    S_CAS2X1, S_CAS2X2, S_CAS2R1, S_CAS2R2, S_CAS2C1, S_CAS2C1W, S_CAS2C2, S_CAS2C2W, S_CAS2C3,
    S_CAS2W1, S_CAS2W2, S_CAS2F1, S_CAS2F2,
    S_PACKX, S_PACKM1, S_PACKM2, S_PACKM3, S_UNPKM1, S_UNPKM2, S_UNPKM3, S_UNPKM4,
    S_MOVECX, S_MOVESX, S_MOVESE, S_MOVES1,
    S_STOP0, S_RESETI,
    S_INT0, S_INT1,
    X_START, X_F2, X_FA, X_FV, X_PC, X_SR, X_VEC, X_VEC2, X_T1, X_T2, X_T3, X_DONE
} state_t;

state_t state, ea_ret, mem_ret, if_ret, g_after_src, g_after_dst;
state_t END_ST;      // where an instruction goes when it is done: S_FETCH, or S_END to take a trace

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
function automatic [31:0] bitrev(input [31:0] v);
    for (int i = 0; i < 32; i++) bitrev[i] = v[31-i];
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

function automatic [7:0] pack_b(input [15:0] v);
    pack_b = {v[11:8], v[3:0]};
endfunction

// -----------------------------------------------------------------------------
// ALU:  returns {flags_changed, ccr_new[4:0], result[31:0]}
// -----------------------------------------------------------------------------
function automatic [37:0] alu(input [4:0] op, input [1:0] s, input [31:0] sv, input [31:0] dv,
                              input [4:0] ccr, input cc_true);
    logic [4:0]  sh;
    logic [31:0] sa, da, ra, r, ai, bi;
    logic [32:0] t;
    logic        x, n, z, v, c, chg, sub, cin;
    logic [9:0]  b;     // BCD intermediate
    logic        bc;
    sh = (s == SZ_B) ? 5'd24 : (s == SZ_W) ? 5'd16 : 5'd0;
    sa = sv << sh;
    da = dv << sh;
    x = ccr[4]; n = ccr[3]; z = ccr[2]; v = ccr[1]; c = ccr[0];
    chg = 1'b1;
    ra = 32'd0; r = 32'd0;
    // one shared adder for ADD/ADDX/SUB/SUBX/CMP/NEG/NEGX (operands aligned to bit 31)
    sub = !(op == A_ADD || op == A_ADDX);
    ai  = (op == A_NEG || op == A_NEGX) ? 32'd0 : da;
    bi  = (op == A_NEG || op == A_NEGX) ? da : sa;
    cin = (op == A_ADDX || op == A_SUBX || op == A_NEGX) && x;
    t   = sub ? ({1'b0, ai} - {1'b0, bi} - ({32'd0, cin} << sh))
              : ({1'b0, ai} + {1'b0, bi} + ({32'd0, cin} << sh));
    case (op)
        A_ADD, A_ADDX, A_SUB, A_SUBX, A_CMP, A_NEG, A_NEGX: begin
            ra = t[31:0];
            c = t[32];
            if (op != A_CMP) x = c;
            v = sub ? ((bi[31] != ai[31]) && (ra[31] != ai[31]))
                    : ((bi[31] == ai[31]) && (ra[31] != ai[31]));
            n = ra[31];
            z = (op == A_ADDX || op == A_SUBX || op == A_NEGX) ? (z && ra == 32'd0) : (ra == 32'd0);
            r = ra >> sh;
        end
        A_AND, A_OR, A_EOR, A_NOT, A_MOVE, A_CLR, A_TST, A_TAS: begin
            case (op)
                A_AND:  ra = da & sa;
                A_OR:   ra = da | sa;
                A_EOR:  ra = da ^ sa;
                A_NOT:  ra = ~da & (32'hFFFF_FFFF << sh);
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
        A_SCC:    begin r = cc_true ? 32'hFF : 32'h00; chg = 1'b0; end
        A_FROMCCR:begin r = {27'd0, ccr}; chg = 1'b0; end
        A_ABCD: begin
            b = {6'd0, dv[3:0]} + {6'd0, sv[3:0]} + {9'd0, x};
            v = ~b[7];
            if (b > 10'd9) b = b + 10'd6;
            b = b + {2'd0, dv[7:4], 4'd0} + {2'd0, sv[7:4], 4'd0};
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
            b = b + {2'd0, dv[7:4], 4'd0} - {2'd0, sv[7:4], 4'd0};
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
            n = b[7];                    // undefined; matches Musashi (0x9A -> N=1)
            if (b[7:0] == 8'h9A) b = 10'd0;   // 0 - 0 - 0 = 0
            r = {24'd0, b[7:0]};
        end
        default: begin r = sv; chg = 1'b0; end   // PASS, FROMSR, TOSR, TOCCR
    endcase
    alu = {chg, x, n, z, v, c, r};
endfunction

// -----------------------------------------------------------------------------
// shifter (AS/LS/RO, not ROX) built around one 65-bit right funnel shift:
// returns {ccr_new, result}
// -----------------------------------------------------------------------------
function automatic [36:0] shifter(input [1:0] typ, input left, input [1:0] s,
                                  input [5:0] cnt, input [31:0] vin, input [4:0] ccr);
    logic [31:0] v, va, r, m, bits, rep;
    logic [65:0] xsrc, y;
    logic [5:0]  w, n, amt;
    logic        x, c, vf, msb;
    w   = s == SZ_B ? 6'd8 : s == SZ_W ? 6'd16 : 6'd32;
    v   = zxs(vin, s);
    msb = msbs(v, s);
    va  = v << (6'd32 - w);                                 // operand aligned to bit 31
    rep = s == SZ_B ? {4{v[7:0]}} : s == SZ_W ? {2{v[15:0]}} : v;
    n   = cnt & (w - 6'd1);
    x = ccr[4]; vf = 1'b0;
    if (typ == 2'b11) begin
        xsrc = {1'b0, rep, rep, 1'b0};
        amt  = left ? ((w - n) & (w - 6'd1)) : n;
    end else if (left) begin
        xsrc = {33'd0, bitrev(va), 1'b0};
        amt  = (cnt > 6'd33) ? 6'd33 : cnt;          // >= 33 gives the same result
    end else begin
        xsrc = {{33{(typ == 2'b00) && msb}}, (typ == 2'b00) ? sxs(v, s) : v, 1'b0};
        amt  = (cnt > 6'd33) ? 6'd33 : cnt;
    end
    y = xsrc >> amt;
    if (typ == 2'b11) begin
        r = zxs(y[32:1], s);
        c = (cnt == 0) ? 1'b0 : (left ? r[0] : msbs(r, s));
    end else begin
        r = left ? (bitrev(y[32:1]) >> (6'd32 - w)) : zxs(y[32:1], s);
        c = (cnt == 0) ? 1'b0 : y[0];
        if (cnt != 0) x = c;
        if (typ == 2'b00 && left) begin   // ASL overflow: top cnt+1 bits not all equal
            m    = (cnt >= 6'd31) ? 32'hFFFF_FFFF : ~(32'hFFFF_FFFF >> (cnt + 6'd1));
            bits = va & m;
            vf   = (cnt >= w) ? (v != 32'd0) : !(bits == 32'd0 || bits == m);
        end
    end
    shifter = {x, msbs(r, s), r == 32'd0, vf, c, r};
endfunction

// -----------------------------------------------------------------------------
// working registers
// -----------------------------------------------------------------------------
logic [15:0] sr;
logic [31:0] PC, PCi, vbr, cacr, caar;
logic [2:0]  sfc, dfc;

logic [15:0] IR, ext, ext2;
logic [1:0]  sz, ssz;                // op size, source size
logic [2:0]  ea_mode, ea_reg;
logic [1:0]  ea_sz;
logic [1:0]  ea_kind;
logic [31:0] ea_addr, ea_imm, ea_base, ea_bd, ea_od, ea_idx;
logic [15:0] ea_ext;

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
logic        retire_q, first_fetch;
assign END_ST = (trace_t1 || trace_t0) ? S_END : S_FETCH;

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
logic [1:0]  flg;                    // CHK2 comparison results

// multiplier / divider
logic [31:0] md_a, md_b;
logic [63:0] md_acc;
logic [2:0]  md_step;
logic [15:0] mul_x, mul_y;
wire  [31:0] mul_p = mul_x * mul_y;
logic        md_neg, md_rneg, md_signed, md_long64, md_word, md_bneg;
logic [31:0] md_q;
logic [32:0] md_rem;
logic [5:0]  md_cnt;

// bitfield
logic [31:0] bf_off;
logic [5:0]  bf_w;
logic [39:0] bf_buf;
logic [2:0]  bf_bc;
logic [31:0] bf_addr;
logic        bf_reg;
wire  [6:0]  bf_bsum   = {4'd0, bf_off[2:0]} + {1'b0, bf_w} + 7'd7;
wire  [2:0]  bf_bcount = bf_bsum[5:3];

// interrupts
logic [2:0]  ipl_q;
logic        nmi_prev, nmi_latch;
wire         nmi_pend    = nmi_latch || (ipl_q == 3'd7 && !nmi_prev);      // level 7 is edge triggered
wire         int_pending = (ipl_q == 3'd7) ? (nmi_pend || sr[10:8] != 3'd7) : (ipl_q > sr[10:8]);

// -----------------------------------------------------------------------------
// register file: D0-D7 = 0..7, A0-A6 = 8..14, USP 15, ISP 16, MSP 17
// one registered write request (applied on the next edge, bypassed to reads)
// -----------------------------------------------------------------------------
// register-file write intent of the current cycle (blocking, set by `RFW in the FSM)
logic        w_en;
logic [4:0]  w_a;
logic [31:0] w_d;
logic [1:0]  w_sz;
// three byte-lane memories so sized writes need no read-modify-write;
// written directly by the FSM (visible to reads on the next clock)
logic [7:0]  rf_b0 [0:31];
logic [7:0]  rf_b1 [0:31];
logic [15:0] rf_hi [0:31];

wire  [4:0]  sp_ph = !sr[13] ? P_USP : (sr[12] ? P_MSP : P_ISP);
function automatic [4:0] ph(input [3:0] l, input [4:0] spp);
    ph = (l == 4'd15) ? spp : {1'b0, l};
endfunction

logic [4:0]  raA, raB, raC;
// registered write request (applied one clock later) + per-lane bypass
logic        wq_en;
logic [4:0]  wq_a;
logic [31:0] wq_d;
logic [1:0]  wq_sz;
function automatic [31:0] rfrd(input [4:0] a, input [7:0] b0, input [7:0] b1, input [15:0] hi,
                               input en, input [4:0] wa, input [31:0] wd, input [1:0] wsz);
    logic hit;
    hit = en && (wa == a);
    rfrd = { (hit && wsz == SZ_L) ? wd[31:16] : hi,
             (hit && wsz != SZ_B) ? wd[15:8]  : b1,
             hit                  ? wd[7:0]   : b0 };
endfunction
wire  [31:0] rA = rfrd(raA,   rf_b0[raA],   rf_b1[raA],   rf_hi[raA],   wq_en, wq_a, wq_d, wq_sz);
wire  [31:0] rB = rfrd(raB,   rf_b0[raB],   rf_b1[raB],   rf_hi[raB],   wq_en, wq_a, wq_d, wq_sz);
wire  [31:0] rC = rfrd(raC,   rf_b0[raC],   rf_b1[raC],   rf_hi[raC],   wq_en, wq_a, wq_d, wq_sz);
wire  [31:0] sp = rfrd(sp_ph, rf_b0[sp_ph], rf_b1[sp_ph], rf_hi[sp_ph], wq_en, wq_a, wq_d, wq_sz);

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
// instruction words only ever come from the prefetch buffer (flops); the cache
// (block RAM) refills the buffer, costing one cycle per longword on a hit
wire         iw_hit = ib_hit;
wire  [15:0] iw = PC[1] ? ib_data[15:0] : ib_data[31:16];

wire ic_we = ICACHE && ce && (state == S_IFETCH) && bus_ack && cacr[0] && !cacr[1];
always_ff @(posedge clk) begin
    if (ic_we) begin
        ic_data[bus_addr[7:2]] <= bus_rdata;
        ic_tag[bus_addr[7:2]]  <= {bus_fc[2], bus_addr[31:8]};
    end
end

// -----------------------------------------------------------------------------
// macros (statements, used inside the FSM)
// -----------------------------------------------------------------------------
`define RFW(PA, V, SZ) begin w_en = 1'b1; w_a = (PA); w_d = (V); w_sz = (SZ); end
`define WR_A(N, V)     `RFW(ph({1'b1, (N)}, sp_ph), V, SZ_L)
`define WR_R(I, V)     `RFW(ph((I), sp_ph), V, SZ_L)
`define WR_D(N, V, S)  `RFW({2'b00, (N)}, V, S)
`define WR_SP(V)       `RFW(sp_ph, V, SZ_L)

`define MEMX(ADDR, SIZE, WE, DATA, FC, RET) begin \
    bus_req <= 1'b1; bus_we <= (WE); bus_addr <= (ADDR); bus_size <= (SIZE); \
    bus_wdata <= (DATA); bus_fc <= (FC); mem_ret <= (RET); state <= S_MEM; end

`define DFC (sr[13] ? 3'd5 : 3'd1)
`define MEMRD(ADDR, SIZE, RET)       `MEMX(ADDR, SIZE, 1'b0, 32'd0, `DFC, RET)
`define MEMWR(ADDR, SIZE, DATA, RET) `MEMX(ADDR, SIZE, 1'b1, DATA, `DFC, RET)

`define NEED_IW(RET) begin \
    if (ic_hit) begin ib_data <= ic_data[ic_idx]; ib_tag <= PC[31:2]; ib_valid <= 1'b1; end \
    else begin \
    bus_req <= 1'b1; bus_we <= 1'b0; bus_addr <= {PC[31:2], 2'b00}; bus_size <= SZ_L; \
    bus_fc <= sr[13] ? 3'd6 : 3'd2; if_ret <= (RET); state <= S_IFETCH; end end

`define PUSHL(V, RET) begin `WR_SP(sp - 32'd4) `MEMWR(sp - 32'd4, SZ_L, V, RET) end
`define PUSHW(V, RET) begin `WR_SP(sp - 32'd2) `MEMWR(sp - 32'd2, SZ_W, V, RET) end

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

// source operand done: a register destination is read right away (port B)
`define SRC_DONE begin \
    if (g_after_src == G_DST && (g_dsel != 2'd0 || dst_mode[2:1] == 2'b00)) begin \
        dst <= rB; dst_kind <= (g_dsel == 2'd2 || (g_dsel == 2'd0 && dst_mode[0])) ? K_A : K_D; \
        dst_rn <= dst_reg; state <= g_after_dst; \
    end else state <= g_after_src; end

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

// EA increment for (An)+ / -(An)
wire [31:0] ea_inc = (ea_sz == SZ_B) ? ((ea_reg == 3'd7) ? 32'd2 : 32'd1) :
                     (ea_sz == SZ_W) ? 32'd2 : 32'd4;
wire [31:0] ea_dec = rA - ea_inc;                                  // port A = An in E_START
wire [31:0] ea_ind = (ea_mode == 3'd4) ? ea_dec : rA;              // operand address
wire [31:0] ea_upd = (ea_mode == 3'd4) ? ea_dec : rA + ea_inc;     // new An

// index register (port B in E_IDX)
wire [31:0] idx_val = (iw[11] ? rB : sx16(rB[15:0])) << iw[10:9];

// shared ALU
wire [31:0] alu_s = sext_src ? sxs(src, ssz) : src;
wire [37:0] alu_o = alu(aop, sz, alu_s, dst, sr[4:0], cond(IR[11:8], sr[4:0]));
wire        alu_z = alu_o[34];
wire        alu_n = alu_o[35];
wire        alu_v = alu_o[33];
// registered ALU output, for states that branch on a comparison result
logic [37:0] alu_q;
wire        aq_z = alu_q[34];
wire        aq_n = alu_q[35];
wire        aq_v = alu_q[33];

// register shift count (decode, port A = D[ir_x])
wire [5:0]  sh_cnt_dec = IR[5] ? rA[5:0] : (IR[11:9] == 3'd0 ? 6'd8 : {3'd0, IR[11:9]});

// shared shifter input selection
logic [1:0]  sh_typ;
logic        sh_left;
logic [1:0]  sh_sz;
logic [5:0]  sh_n;
logic [31:0] sh_v;
always_comb begin
    sh_typ = IR[4:3]; sh_left = IR[8]; sh_sz = ir_sz; sh_n = cnt6; sh_v = tmp;
    case (state)
        S_SHIFTM: begin sh_typ = IR[10:9]; sh_sz = SZ_W; sh_n = 6'd1; sh_v = dst; end
        S_BF0:    begin sh_typ = 2'b11; sh_left = 1'b1; sh_sz = SZ_L; sh_n = {1'b0, bf_off[4:0]}; sh_v = tmp; end
        S_BF5:    begin sh_typ = 2'b11; sh_left = 1'b0; sh_sz = SZ_L; sh_n = {1'b0, bf_off[4:0]}; sh_v = bf_buf[39:8]; end
        default: ;
    endcase
end
wire [36:0] sh_o = shifter(sh_typ, sh_left, sh_sz, sh_n, sh_v, sr[4:0]);

// bitfield datapath (works on a 40-bit window, field starts at bit 39 - bf_sh)
wire [31:0] bf_w32  = (bf_w == 6'd32) ? 32'hFFFF_FFFF : ~(32'hFFFF_FFFF >> bf_w);   // top-aligned mask
wire [2:0]  bf_sh   = bf_reg ? 3'd0 : bf_off[2:0];
wire [39:0] bf_al   = bf_buf << bf_sh;
wire [31:0] bf_fld  = bf_al[39:8] & bf_w32;
wire [39:0] bf_msk  = {bf_w32, 8'd0} >> bf_sh;
wire [31:0] bf_ins  = tmp2 << (6'd32 - bf_w);                                      // latched D[ext[14:12]]
wire [39:0] bf_insv = {bf_ins & bf_w32, 8'd0} >> bf_sh;
logic [31:0] bf_fr;                                                                  // registered field
wire [32:0] bf_extv = $signed({(IR[10:8] == 3'b011) && bf_fr[31], bf_fr}) >>> (6'd32 - bf_w);
logic [5:0] bf_ffo;
always_comb begin
    bf_ffo = bf_w;
    for (int k = 0; k < 32; k++)
        if (bf_fr[k]) bf_ffo = 6'd31 - k[5:0];
end
logic [39:0] bf_nb;
always_comb begin
    case (IR[10:8])
        3'b010:  bf_nb = bf_buf ^ bf_msk;                     // BFCHG
        3'b100:  bf_nb = bf_buf & ~bf_msk;                    // BFCLR
        3'b110:  bf_nb = bf_buf | bf_msk;                     // BFSET
        3'b111:  bf_nb = (bf_buf & ~bf_msk) | bf_insv;        // BFINS
        default: bf_nb = bf_buf;
    endcase
end
wire bf_modify = IR[10:8] == 3'b010 || IR[10:8] == 3'b100 || IR[10:8] == 3'b110 || IR[10:8] == 3'b111;

// multiply / divide shared 64-bit adder
logic [63:0] md_x, md_y;
logic        md_cin;
always_comb begin
    md_x = md_acc; md_y = 64'd0; md_cin = 1'b0;
    case (state)
        S_MUL_LOOP: begin
            case (md_step)
                3'd1: md_y = {32'd0, mul_p};
                3'd2, 3'd3: md_y = {16'd0, mul_p, 16'd0};
                3'd4: md_y = {mul_p, 32'd0};
                // signed corrections: 32-bit form subtracts b<<32 / a<<32,
                // 16-bit form (MULS.W, zero-extended operands) subtracts b<<16 / a<<16
                3'd5: if (md_signed && (md_word ? md_a[15] : md_a[31])) begin
                          md_y = md_word ? ~{32'd0, md_b[15:0], 16'd0} : {~md_b, 32'hFFFF_FFFF}; md_cin = 1'b1; end
                3'd6: if (md_signed && (md_word ? md_b[15] : md_b[31])) begin
                          md_y = md_word ? ~{32'd0, md_a[15:0], 16'd0} : {~md_a, 32'hFFFF_FFFF}; md_cin = 1'b1; end
                default: ;
            endcase
        end
        S_DIV_ABS1: begin md_x = ~md_acc; md_cin = 1'b1; end                     // -dividend
        S_DIV_ABS2: begin md_x = {32'hFFFF_FFFF, ~md_b}; md_cin = 1'b1; end       // -divisor
        S_DIV_FIN:  begin md_x = {32'd0, md_rneg ? ~md_rem[31:0] : md_rem[31:0]}; md_cin = md_rneg; end
        S_DIV_FIN2: begin md_x = {32'd0, md_neg ? ~md_q : md_q}; md_cin = md_neg; end
        default: ;
    endcase
end
wire [63:0] md_sum = md_x + md_y + {63'd0, md_cin};

// divider step: shift in next dividend bit, trial subtract (first cycle: overflow check)
wire [32:0] div_r2   = (md_cnt == 6'd0) ? {1'b0, md_acc[63:32]} : {md_rem[31:0], md_q[31]};
wire [33:0] div_diff = {1'b0, div_r2} - {2'b0, md_b};

// -----------------------------------------------------------------------------
// register file read port addressing (per state)
// -----------------------------------------------------------------------------
wire xa_dec = (IR[15:12] == 4'hC) && (IR[8:3] == 6'b101001);                       // EXG Ax,Ay
wire ya_dec = (IR[15:8] == 8'h4E) || ((IR[15:12] == 4'hC) && IR[8] && IR[3]);      // UNLK/MOVE USP/EXG
always_comb begin
    raA = {2'b00, ir_x};
    raB = {2'b00, ir_r};
    raC = P_USP;
    case (state)
        S_DECODE: begin
            raA = ph({xa_dec, ir_x}, sp_ph);
            raB = ph({ya_dec, ir_r}, sp_ph);
        end
        E_START:                 raA = ph({1'b1, ea_reg}, sp_ph);
        E_IDX:                   raB = ph(iw[15:12], sp_ph);
        G_SRC_RD, G_SRC_RD2, E_IMM1, E_IMM2: begin
            raA = ph({ea_kind == K_A, ea_reg}, sp_ph);
            raB = (g_dsel == 2'd0) ? ph({dst_mode[0], dst_reg}, sp_ph) : ph({g_dsel == 2'd2, dst_reg}, sp_ph);
        end
        G_DST_RD:                raA = ph({ea_kind == K_A, ea_reg}, sp_ph);
        G_SRC: begin
            raA = ph({src_mode[0], src_reg}, sp_ph);
            raB = (g_dsel == 2'd0) ? ph({dst_mode[0], dst_reg}, sp_ph) : ph({g_dsel == 2'd2, dst_reg}, sp_ph);
        end
        G_DST:                   raA = (g_dsel == 2'd0) ? ph({dst_mode[0], dst_reg}, sp_ph)
                                                        : ph({g_dsel == 2'd2, dst_reg}, sp_ph);
        S_MOVEM0, S_MOVEP0, S_LINK1: raA = ph({1'b1, ir_r}, sp_ph);
        S_MOVEM_L:               raA = ph(lsb16(mvm_mask), sp_ph);
        S_MOVEM_PD:              raA = ph(4'd15 - lsb16(mvm_mask), sp_ph);
        S_DBCC, S_PACKX:         raA = {2'b00, ir_r};
        S_CHK, S_MULW, S_DIVW:   raA = {2'b00, ir_x};
        S_CHK2R2:                raA = ph(ext[15:12], sp_ph);
        S_MULL:                  raA = {2'b00, ext[14:12]};
        S_DIVL: begin            raA = {2'b00, ext[14:12]}; raB = {2'b00, ext[2:0]}; end
        S_BFE: begin             raA = {2'b00, ext[8:6]};   raB = {2'b00, ext[2:0]}; raC = {2'b00, ir_r}; end
        S_BF0:                   raB = {2'b00, ext[14:12]};
        S_CAS: begin             raA = {2'b00, ext[2:0]};   raB = {2'b00, ext[8:6]}; end
        S_CAS1:                  raB = {2'b00, ext[8:6]};
        S_CAS2X2:                raA = ph(ext[15:12], sp_ph);
        S_CAS2R1:                raA = ph(ext2[15:12], sp_ph);
        S_CAS2C1:                raA = {2'b00, ext[2:0]};
        S_CAS2C2:                raB = {2'b00, ext2[2:0]};
        S_CAS2W1: begin          raA = ph(ext[15:12], sp_ph);  raB = {2'b00, ext[8:6]}; end
        S_CAS2W2: begin          raA = ph(ext2[15:12], sp_ph); raB = {2'b00, ext2[8:6]}; end
        S_MOVECX: begin
            raA = ph(iw[15:12], sp_ph);
            raB = (iw[11:0] == 12'h803) ? P_MSP : (iw[11:0] == 12'h804) ? P_ISP : P_USP;
        end
        S_MOVESE:                raA = ph(ext[15:12], sp_ph);
        default: ;   // default: port A = D[ir_x], port B = D[ir_r]
    endcase
end

// MOVEC read mux (control register selected by the extension word at iw)
logic [31:0] movec_rd;
always_comb begin
    case (iw[11:0])
        12'h000: movec_rd = {29'd0, sfc};
        12'h001: movec_rd = {29'd0, dfc};
        12'h002: movec_rd = cacr;
        12'h801: movec_rd = vbr;
        12'h802: movec_rd = caar;
        default: movec_rd = rB;          // USP / MSP / ISP via port B
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

// end address of the current bus transfer (for write snooping)
wire [31:0] bus_addr_end = bus_addr + ((bus_size == SZ_L) ? 32'd3 : (bus_size == SZ_W) ? 32'd1 : 32'd0);

// -----------------------------------------------------------------------------
// debug outputs (shadow copy of the register file)
// -----------------------------------------------------------------------------
assign dbg_retire = retire_q && ce;      // pulse in the cycle after the next opcode was fetched
assign dbg_pc  = PCi;                    // = address of that next instruction
assign dbg_sr  = sr;
assign dbg_vbr = vbr;
assign halted  = (state == S_HALT);
logic [31:0] shadow [0:31];      // debug copy of the register file (written with it)
generate
    if (DEBUG) begin : g_dbg
        for (genvar gi = 0; gi < 8; gi++) begin : g_d
            assign dbg_d[gi*32 +: 32] = shadow[gi];
            if (gi < 7) begin : g_a
                assign dbg_a[gi*32 +: 32] = shadow[gi + 8];
            end
        end
        assign dbg_a[255:224] = shadow[sp_ph];
        assign dbg_usp = shadow[P_USP];
        assign dbg_isp = shadow[P_ISP];
        assign dbg_msp = shadow[P_MSP];
    end else begin : g_nodbg
        assign dbg_d = '0;
        assign dbg_a = '0;
        assign dbg_usp = '0;
        assign dbg_isp = '0;
        assign dbg_msp = '0;
    end
endgenerate

// -----------------------------------------------------------------------------
// main FSM
// -----------------------------------------------------------------------------

always_ff @(posedge clk) if (ce) begin
    w_en = 1'b0; w_a = 5'd0; w_d = 32'd0; w_sz = SZ_L;
    // interrupt input sampling
    ipl_q    <= ipl;
    nmi_prev <= (ipl_q == 3'd7);
    if (ipl_q == 3'd7 && !nmi_prev) nmi_latch <= 1'b1;

    alu_q <= alu_o;
    retire_q <= 1'b0;

    if (!rst_n) begin
        state     <= S_CLR;
        cnt6      <= 6'd0;
        bus_req   <= 1'b0;
        bus_we    <= 1'b0;
        bus_lock  <= 1'b0;
        bus_size  <= SZ_L;
        bus_fc    <= 3'd6;
        bus_addr  <= 32'd0;
        bus_wdata <= 32'd0;
        sr        <= 16'h2700;
        vbr       <= 32'd0;
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
        first_fetch <= 1'b1;
    end else begin
    case (state)

    // =========================================================================
    // reset: clear the register file (D, A, USP, MSP defined as 0), then
    // fetch SSP / PC
    // =========================================================================
    S_CLR: begin
        `RFW(cnt6[4:0], 32'd0, SZ_L)
        cnt6 <= cnt6 + 6'd1;
        if (cnt6 == 6'd17) state <= S_RESET;
    end
    S_RESET: begin
        `MEMX(32'd0, SZ_L, 1'b0, 32'd0, 3'd6, S_RST1)
    end
    S_RST1: begin
        `RFW(P_ISP, mem_rd, SZ_L)
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
                // keep the instruction stream coherent with our own writes
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
                x_ssw <= 16'h4006 | {13'd0, bus_fc};
                x_dob <= 32'd0;
                `EXC(8'd2, 4'hA, PCi, bus_addr)
            end
        end
    end

    // =========================================================================
    // instruction fetch / completion
    // =========================================================================
    S_FETCH: begin
        bus_lock <= 1'b0;
        if (stop_pending) begin
            stop_pending <= 1'b0;
            state <= S_STOPPED;
        end else if (int_pending) begin
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
            // the previous instruction (and any exception / interrupt taken after it)
            // is complete: its register writes are visible from the next cycle on
            retire_q    <= !first_fetch;
            first_fetch <= 1'b0;
        end
    end

    S_END: begin
        // only reached when tracing: take the trace exception, then fetch
        if (trace_t1 || (trace_t0 && flow)) begin
            trace_t1 <= 1'b0;
            trace_t0 <= 1'b0;
            `EXCT(8'd9, 4'h2, PC, PCi)
        end else begin
            trace_t1 <= 1'b0;
            trace_t0 <= 1'b0;
            state <= S_FETCH;
        end
    end

    S_STOPPED: begin
        if (int_pending) state <= S_FETCH;
    end

    S_HALT: ;

    // =========================================================================
    // effective address engine (port A = An, port B = index register)
    // =========================================================================
    E_START: begin
        case (ea_mode)
            3'd0: begin ea_kind <= K_D; state <= ea_ret; end
            3'd1: begin ea_kind <= K_A; state <= ea_ret; end
            3'd2, 3'd3, 3'd4: begin
                ea_kind <= K_M;
                ea_addr <= ea_ind;
                if (ea_mode != 3'd2) `WR_A(ea_reg, ea_upd)
                // generic flow: issue the operand read right away
                if (ea_ret == G_SRC_RD)
                    `MEMRD(ea_ind, ssz, G_SRC_RD2)
                else if (ea_ret == G_DST_RD) begin
                    dst_kind <= K_M; dst_addr <= ea_ind; dst_rn <= ea_reg;
                    if (g_rd) `MEMRD(ea_ind, sz, G_DST_RD2)
                    else      state <= g_after_dst;
                end else
                    state <= ea_ret;
            end
            3'd5: begin ea_base <= rA; state <= E_D16; end
            3'd6: begin ea_base <= rA; state <= E_IDX; end
            default: case (ea_reg)
                3'd0: state <= E_ABSW;
                3'd1: state <= E_ABSL1;
                3'd2: begin ea_base <= PC; state <= E_D16; end
                3'd3: begin ea_base <= PC; state <= E_IDX; end
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
                if (ea_ret == G_SRC_RD) begin
                    src <= (ea_sz == SZ_B) ? {24'd0, iw[7:0]} : {16'd0, iw};
                    `SRC_DONE
                end else
                    state <= ea_ret;
            end
        end
    end
    E_IMM2: begin
        if (!iw_hit) `NEED_IW(E_IMM2)
        else begin
            PC <= PC + 32'd2;
            ea_imm[15:0] <= iw;
            if (ea_ret == G_SRC_RD) begin
                src <= {ea_imm[31:16], iw};
                `SRC_DONE
            end else
                state <= ea_ret;
        end
    end
    E_IDX: begin
        if (!iw_hit) `NEED_IW(E_IDX)
        else begin
            PC <= PC + 32'd2;
            ea_ext <= iw;
            if (!iw[8]) begin
                // brief format (scale allowed on 68020): base + d8 now, + index next cycle
                ea_base <= ea_base + sx8(iw[7:0]);
                ea_idx  <= idx_val;
                state   <= E_IDX2;
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
    E_IDX2: begin
        ea_addr <= ea_base + ea_idx;
        ea_kind <= K_M;
        state <= ea_ret;
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
        ea_base <= ea_base + ea_bd;
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
        // ea_base already holds base + bd
        `MEMRD(ea_base + (ea_ext[2] ? 32'd0 : ea_idx), SZ_L, E_MI2)
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
        if (g_src_ea && src_mode[2:1] == 2'b00) begin
            // register source (port A); a register destination is read in the same cycle (port B)
            src <= rA;
            `SRC_DONE
        end else if (g_src_ea) `CALL_EA(src_mode, src_reg, ssz, G_SRC_RD)
        else state <= g_after_src;
    end
    G_SRC_RD: begin
        case (ea_kind)
            K_D, K_A: begin src <= rA; `SRC_DONE end
            K_I:      begin src <= ea_imm; `SRC_DONE end
            default:  `MEMRD(ea_addr, ssz, G_SRC_RD2)
        endcase
    end
    G_SRC_RD2: begin
        src <= mem_rd;
        `SRC_DONE
    end
    G_DST: begin
        case (g_dsel)
            2'd0: if (dst_mode[2:1] == 2'b00) begin       // register destination: read directly
                      dst <= rA; dst_kind <= dst_mode[0] ? K_A : K_D; dst_rn <= dst_reg; state <= g_after_dst;
                  end else `CALL_EA(dst_mode, dst_reg, sz, G_DST_RD)
            2'd1: begin dst <= rA; dst_kind <= K_D; dst_rn <= dst_reg; state <= g_after_dst; end
            default: begin dst <= rA; dst_kind <= K_A; dst_rn <= dst_reg; state <= g_after_dst; end
        endcase
    end
    G_DST_RD: begin
        dst_kind <= ea_kind;
        dst_addr <= ea_addr;
        dst_rn   <= ea_reg;
        case (ea_kind)
            K_D, K_A: begin dst <= rA; state <= g_after_dst; end
            K_I:      begin dst <= ea_imm; state <= g_after_dst; end
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
        if (g_wr && dst_kind == K_D) begin           // register result: write it now
            `WR_D(dst_rn, alu_o[31:0], sz)
            state <= END_ST;
        end else if (g_wr && dst_kind == K_A) begin
            `WR_A(dst_rn, alu_o[31:0])
            state <= END_ST;
        end else if (g_wr && aop != A_TOSR && aop != A_TOCCR)
            `MEMWR(dst_addr, sz, alu_o[31:0], END_ST)
        else
            state <= END_ST;
    end
    G_WB: begin
        case (dst_kind)
            K_D: begin `WR_D(dst_rn, res, sz) state <= END_ST; end
            K_A: begin `WR_A(dst_rn, res) state <= END_ST; end
            default: `MEMWR(dst_addr, sz, res, END_ST)
        endcase
    end

    // =========================================================================
    // decode   (port A = Dx / Ax, port B = Dy / Ay, port C = USP)
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
        // T0 ("change of flow") also traces instructions that synchronise the pipeline:
        // MOVE/ANDI/EORI/ORI to SR, MOVE An,USP, MOVEC, MOVES, CAS, CAS2, NOP, STOP
        if (IR[15:8] == 8'h46 && IR[7:6] == 2'b11 ||                         // MOVE to SR
            IR == 16'h007C || IR == 16'h027C || IR == 16'h0A7C ||            // ORI/ANDI/EORI to SR
            IR[15:3] == 13'b0100_1110_0110_0 ||                              // MOVE An,USP
            IR[15:1] == 15'b0100_1110_0111_101 ||                            // MOVEC
            IR[15:8] == 8'h0E && IR[7:6] != 2'b11 ||                         // MOVES
            IR[15:11] == 5'b00001 && IR[8:6] == 3'b011 && IR[10:9] != 2'b00 || // CAS / CAS2
            IR == 16'h4E71 || IR == 16'h4E72)                                // NOP / STOP
            flow <= 1'b1;

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
            else begin src <= rA; state <= S_BITD; end
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
            if (ir_sz == 2'b11 || !eaok(ir_m, ir_r, M_MALT)) `ILLEGAL
            else if (!sr[13]) `PRIV
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
        16'h4E71: state <= END_ST;                                  // NOP
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
            else state <= END_ST;
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
            `WR_SP(rB)
            `MEMRD(rB, SZ_L, S_UNLK1)
        end
        16'b0100_1110_0110_????: begin                             // MOVE USP
            if (!sr[13]) `PRIV
            else begin
                if (IR[3]) `WR_A(ir_r, rC)
                else       `RFW(P_USP, rB, SZ_L)
                state <= END_ST;
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
            `WR_D(ir_r, {rB[15:0], rB[31:16]}, SZ_L)
            sr[3] <= rB[15];
            sr[2] <= (rB == 32'd0);
            sr[1:0] <= 2'b00;
            state <= END_ST;
        end
        16'b0100_1000_0100_1???: `ILLEGAL                          // BKPT (no debugger)
        16'b0100_1000_1000_0???: begin                             // EXT.W
            `WR_D(ir_r, sx8(rB[7:0]), SZ_W)
            sr[3] <= rB[7];
            sr[2] <= (rB[7:0] == 8'd0);
            sr[1:0] <= 2'b00;
            state <= END_ST;
        end
        16'b0100_1000_1100_0???: begin                             // EXT.L
            `WR_D(ir_r, sx16(rB[15:0]), SZ_L)
            sr[3] <= rB[15];
            sr[2] <= (rB[15:0] == 16'd0);
            sr[1:0] <= 2'b00;
            state <= END_ST;
        end
        16'b0100_1001_1100_0???: begin                             // EXTB.L
            `WR_D(ir_r, sx8(rB[7:0]), SZ_L)
            sr[3] <= rB[7];
            sr[2] <= (rB[7:0] == 8'd0);
            sr[1:0] <= 2'b00;
            state <= END_ST;
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
            if (!eaok(ir_m, ir_r, M_DALT)) `ILLEGAL
            else if (!sr[13]) `PRIV
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
            if (!eaok(ir_m, ir_r, M_DATA)) `ILLEGAL
            else if (!sr[13]) `PRIV
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
            `WR_D(ir_x, sx8(IR[7:0]), SZ_L)
            sr[3] <= IR[7];
            sr[2] <= (IR[7:0] == 8'd0);
            sr[1:0] <= 2'b00;
            state <= END_ST;
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
                else begin aop <= A_OR; g_src_ea <= 1'b0; src <= rA; state <= G_DST; end
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
                else begin aop <= IR[14] ? A_ADD : A_SUB; g_src_ea <= 1'b0; src <= rA; state <= G_DST; end
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
            else begin aop <= A_EOR; g_src_ea <= 1'b0; src <= rA; state <= G_DST; end
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
        16'b1100_???1_0100_0???, 16'b1100_???1_0100_1???, 16'b1100_???1_1000_1???: begin   // EXG
            // first half: Rx <= Ry, keep old Rx for the second half
            `RFW(ph({xa_dec, ir_x}, sp_ph), rB, SZ_L)
            tmp <= rA;
            state <= S_EXG2;
        end
        16'b1100_????_????_????: begin                             // AND
            if (!IR[8]) begin
                if (!eaok(ir_m, ir_r, M_DATA)) `ILLEGAL
                else begin aop <= A_AND; g_dsel <= 2'd1; dst_reg <= ir_x; state <= G_SRC; end
            end else begin
                if (!eaok(ir_m, ir_r, M_MALT)) `ILLEGAL
                else begin aop <= A_AND; g_src_ea <= 1'b0; src <= rA; state <= G_DST; end
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
            tmp  <= rB;
            cnt6 <= sh_cnt_dec;
            state <= S_SHIFT;
        end

        // ------------------------------------------------------------ A-line/F
        16'b1010_????_????_????: `EXC(8'd10, 4'h0, PCi, PCi)
        16'b1111_????_????_????: `EXC(8'd11, 4'h0, PCi, PCi)

        default: `ILLEGAL
        endcase
    end

    S_EXG2: begin
        `RFW(ph({ya_dec, ir_r}, sp_ph), tmp, SZ_L)
        state <= END_ST;
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
        state <= END_ST;
    end

    // =========================================================================
    // MOVEM   (port A = An / register being transferred)
    // =========================================================================
    S_MOVEM0: begin
        if (!iw_hit) `NEED_IW(S_MOVEM0)
        else begin
            PC <= PC + 32'd2;
            mvm_mask <= iw;
            if (ir_m == 3'd4) begin               // -(An), reg -> mem
                mvm_addr <= rA;
                tmp      <= rA - szbytes(sz);     // value stored for An itself (68020)
                state    <= S_MOVEM_PD;
            end else if (ir_m == 3'd3) begin      // (An)+, mem -> reg
                mvm_addr <= rA;
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
            state <= END_ST;
        end else begin
            cnt6[3:0] <= lsb16(mvm_mask);
            mvm_mask[lsb16(mvm_mask)] <= 1'b0;
            mvm_addr <= mvm_addr + szbytes(sz);
            if (IR[10]) `MEMRD(mvm_addr, sz, S_MOVEM_LD)
            else        `MEMWR(mvm_addr, sz, rA, S_MOVEM_L)
        end
    end
    S_MOVEM_LD: begin
        `WR_R(cnt6[3:0], sxs(mem_rd, sz))
        state <= S_MOVEM_L;
    end
    S_MOVEM_PD: begin
        if (mvm_mask == 16'd0) begin
            `WR_A(ir_r, mvm_addr)
            state <= END_ST;
        end else begin
            mvm_mask[lsb16(mvm_mask)] <= 1'b0;
            mvm_addr <= mvm_addr - szbytes(sz);
            if ((4'd15 - lsb16(mvm_mask)) == {1'b1, ir_r})
                `MEMWR(mvm_addr - szbytes(sz), sz, tmp, S_MOVEM_PD)
            else
                `MEMWR(mvm_addr - szbytes(sz), sz, rA, S_MOVEM_PD)
        end
    end

    // =========================================================================
    // MOVEP   (S_MOVEP0: port A = Ay; S_MOVEP2 first pass: port A = Dx)
    // =========================================================================
    S_MOVEP0: begin
        if (!iw_hit) `NEED_IW(S_MOVEP0)
        else begin
            PC <= PC + 32'd2;
            mvm_addr <= rA + sx16(iw);
            cnt3 <= IR[6] ? 3'd4 : 3'd2;
            tmp  <= 32'd0;
            cnt6 <= 6'd1;
            state <= S_MOVEP2;
        end
    end
    S_MOVEP2: begin
        if (cnt6 == 6'd1) begin
            cnt6 <= 6'd0;
            if (IR[7]) tmp <= IR[6] ? rA : {rA[15:0], 16'd0};
        end else begin
            tmp <= {tmp[23:0], mem_rd[7:0]};
        end
        state <= S_MOVEP1;
    end
    S_MOVEP1: begin
        if (cnt3 == 3'd0) begin
            if (!IR[7]) `WR_D(ir_x, tmp, IR[6] ? SZ_L : SZ_W)
            state <= END_ST;
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

    // =========================================================================
    // Bcc / BRA / BSR
    // =========================================================================
    S_BCC0: begin
        if (IR[7:0] == 8'h00) state <= S_BCCW;
        else if (IR[7:0] == 8'hFF) state <= S_BCCL1;
        else if (IR[11:8] == 4'h1) begin bdisp <= sx8(IR[7:0]); state <= S_BCCGO; end   // BSR.S
        else begin                                                                      // Bcc.S / BRA.S
            if (cond(IR[11:8], sr[4:0])) begin
                PC <= PC + sx8(IR[7:0]);       // PC = PCi + 2 here
                flow <= 1'b1;
            end
            state <= END_ST;
        end
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
            `PUSHL(PC, END_ST)
        end else begin
            if (cond(IR[11:8], sr[4:0])) begin
                PC <= PCi + 32'd2 + bdisp;
                flow <= 1'b1;
            end
            state <= END_ST;
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
                `WR_D(ir_r, {16'd0, rA[15:0] - 16'd1}, SZ_W)
                if (rA[15:0] != 16'd0) begin
                    PC <= PCi + 32'd2 + sx16(iw);
                    flow <= 1'b1;
                end
            end
            state <= END_ST;
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
        else state <= END_ST;
    end

    // =========================================================================
    // JMP / JSR / LEA / PEA
    // =========================================================================
    S_JEA: begin
        PC <= ea_addr;
        flow <= 1'b1;
        state <= END_ST;
    end
    S_JSR2: begin
        PC <= ea_addr;
        flow <= 1'b1;
        `PUSHL(PC, END_ST)
    end
    S_LEA: begin
        `WR_A(ir_x, ea_addr)
        state <= END_ST;
    end
    S_PEA: begin
        `PUSHL(ea_addr, END_ST)
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
        `PUSHL((ir_r == 3'd7) ? sp - 32'd4 : rA, (ir_r == 3'd7) ? S_LINK3 : S_LINK2)
    end
    S_LINK2: begin
        `WR_A(ir_r, sp)
        state <= S_LINK3;
    end
    S_LINK3: begin
        `WR_SP(sp + bdisp)
        state <= END_ST;
    end
    S_UNLK1: begin
        if (ir_r == 3'd7) begin
            `WR_SP(mem_rd)
            state <= END_ST;
        end else begin
            `WR_A(ir_r, mem_rd)
            state <= S_UNLK2;
        end
    end
    S_UNLK2: begin
        `WR_SP(sp + 32'd4)
        state <= END_ST;
    end

    // =========================================================================
    // RTS / RTD / RTR
    // =========================================================================
    S_RTS1: begin
        PC <= mem_rd;
        `WR_SP(sp + 32'd4)
        flow <= 1'b1;
        state <= END_ST;
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
        `WR_SP(sp + 32'd4 + bdisp)
        flow <= 1'b1;
        state <= END_ST;
    end
    S_RTR1: begin
        sr[4:0] <= mem_rd[4:0];
        `MEMRD(sp + 32'd2, SZ_L, S_RTR2)
    end
    S_RTR2: begin
        PC <= mem_rd;
        `WR_SP(sp + 32'd6)
        flow <= 1'b1;
        state <= END_ST;
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
            4'h0, 4'h1: begin `WR_SP(sp + 32'd8)  state <= S_RTE4; end
            4'h2:       begin `WR_SP(sp + 32'd12) state <= S_RTE4; end
            4'h9:       begin `WR_SP(sp + 32'd20) state <= S_RTE4; end
            4'hA:       begin `WR_SP(sp + 32'd32) state <= S_RTE4; end
            4'hB:       begin `WR_SP(sp + 32'd92) state <= S_RTE4; end
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
            state <= END_ST;
        end
    end

    // =========================================================================
    // CHK (src = upper bound, port A = Dn); compare through the shared ALU
    // =========================================================================
    S_CHK: begin
        dst  <= sxs(rA, sz);
        src  <= sxs(src, sz);
        sz   <= SZ_L;
        aop  <= A_CMP;
        sext_src <= 1'b0;
        state <= S_CHK1W;
    end
    S_CHK1W: state <= S_CHK1;                      // compare result -> alu_q
    S_CHK1: begin
        // dst = Dn, src = bound (both sign-extended): Dn > bound  <=>  GT
        sr[2] <= (dst == 32'd0);
        sr[1:0] <= 2'b00;
        if (dst[31]) begin
            sr[3] <= 1'b1;
            `EXCT(8'd6, 4'h2, PC, PCi)
        end else if (!aq_z && (aq_n == aq_v)) begin
            sr[3] <= 1'b0;
            `EXCT(8'd6, 4'h2, PC, PCi)
        end else
            state <= END_ST;
    end

    // =========================================================================
    // CHK2 / CMP2  (three compares through the shared ALU)
    // =========================================================================
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
    S_CHK2R2: begin
        // compare 1: upper - lower  (lower <= upper  <=> GE)
        dst <= sxs(mem_rd, sz);
        tmp <= sxs(mem_rd, sz);                    // keep upper bound
        tmp2 <= ext[15] ? rA : sxs(rA, sz);        // value being checked
        sz  <= SZ_L;
        aop <= A_CMP;
        sext_src <= 1'b0;
        state <= S_CHK2C1;
    end
    S_CHK2C1: begin
        flg[0] <= (alu_n == alu_v);                // lower <= upper
        dst <= tmp2;                               // compare 2: rv - lower
        state <= S_CHK2C2;
    end
    S_CHK2C2: begin
        flg[1] <= (alu_n != alu_v);                // rv < lower
        sr[2]  <= alu_z;                           // rv == lower
        src <= tmp;                                // compare 3: rv - upper
        state <= S_CHK2C3;
    end
    S_CHK2C3: state <= S_CHK2C4;                   // compare 3 -> alu_q
    S_CHK2C4: begin : chk2_blk
        logic gt, out;
        gt  = !aq_z && (aq_n == aq_v);             // rv > upper
        out = flg[0] ? (flg[1] || gt) : (flg[1] && gt);
        if (aq_z) sr[2] <= 1'b1;
        sr[0] <= out;
        if (out && ext[11]) `EXCT(8'd6, 4'h2, PC, PCi)
        else state <= END_ST;
    end

    // =========================================================================
    // MUL: 32x32 -> 64 from four 16x16 products (+ signed corrections)
    // =========================================================================
    S_MULW: begin
        md_signed <= IR[8];
        md_a <= {16'd0, src[15:0]};
        md_b <= {16'd0, rA[15:0]};
        md_word <= 1'b1;
        md_long64 <= 1'b0;
        md_acc <= 64'd0;
        md_step <= 3'd0;
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
        md_a <= src;
        md_b <= rA;
        md_word <= 1'b0;
        md_long64 <= ext[10];
        md_acc <= 64'd0;
        md_step <= 3'd0;
        state <= S_MUL_LOOP;
    end
    S_MUL_LOOP: begin
        case (md_step)
            3'd0: begin mul_x <= md_a[15:0];  mul_y <= md_b[15:0];  end
            3'd1: begin mul_x <= md_a[31:16]; mul_y <= md_b[15:0];  end
            3'd2: begin mul_x <= md_a[15:0];  mul_y <= md_b[31:16]; end
            3'd3: begin mul_x <= md_a[31:16]; mul_y <= md_b[31:16]; end
            default: ;
        endcase
        md_acc  <= md_sum;
        // 16x16: one partial product (+ two corrections when signed)
        md_step <= (md_word && md_step == 3'd1) ? 3'd5 : md_step + 3'd1;
        if (md_step == 3'd6 || (md_word && !md_signed && md_step == 3'd1)) state <= S_MUL_FIN;
    end
    S_MUL_FIN: begin
        if (md_word) begin
            `WR_D(ir_x, md_acc[31:0], SZ_L)
            sr[3] <= md_acc[31];
            sr[2] <= (md_acc[31:0] == 32'd0);
            sr[1:0] <= 2'b00;
            state <= END_ST;
        end else if (md_long64) begin
            `WR_D(ext[2:0], md_acc[63:32], SZ_L)
            sr[3] <= md_acc[63];
            sr[2] <= (md_acc == 64'd0);
            sr[1:0] <= 2'b00;
            state <= S_MUL_FIN2;
        end else begin
            sr[3] <= md_acc[31];
            sr[2] <= (md_acc[31:0] == 32'd0);
            sr[1] <= md_signed ? (md_acc[63:32] != {32{md_acc[31]}}) : (md_acc[63:32] != 32'd0);
            sr[0] <= 1'b0;
            state <= S_MUL_FIN2;
        end
    end
    S_MUL_FIN2: begin
        `WR_D(ext[14:12], md_acc[31:0], SZ_L)
        state <= END_ST;
    end

    // =========================================================================
    // DIV: restoring division on magnitudes
    // =========================================================================
    S_DIVW: begin
        if (src[15:0] == 16'd0) begin
            sr[0] <= 1'b0;
            `EXCT(8'd5, 4'h2, PC, PCi)
        end else begin
            md_signed <= IR[8];
            md_word   <= 1'b1;
            md_long64 <= 1'b0;
            md_acc  <= IR[8] ? {{32{rA[31]}}, rA} : {32'd0, rA};
            md_b    <= IR[8] ? sx16(src[15:0]) : {16'd0, src[15:0]};
            md_neg  <= IR[8] && (rA[31] ^ src[15]);
            md_rneg <= IR[8] && rA[31];
            md_bneg <= IR[8] && src[15];
            state <= S_DIV_ABS1;
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
        if (src == 32'd0) begin
            sr[0] <= 1'b0;
            `EXCT(8'd5, 4'h2, PC, PCi)
        end else begin
            // port A = Dq, port B = Dr
            dvd = ext[10] ? {rB, rA} : (ext[11] ? {{32{rA[31]}}, rA} : {32'd0, rA});
            md_signed <= ext[11];
            md_word   <= 1'b0;
            md_long64 <= ext[10];
            md_acc  <= dvd;
            md_b    <= src;
            md_neg  <= ext[11] && (dvd[63] ^ src[31]);
            md_rneg <= ext[11] && dvd[63];
            md_bneg <= ext[11] && src[31];
            state <= S_DIV_ABS1;
        end
    end
    S_DIV_ABS1: begin
        if (md_rneg) md_acc <= md_sum;                       // |dividend|
        state <= S_DIV_ABS2;
    end
    S_DIV_ABS2: begin
        if (md_bneg) md_b <= md_sum[31:0];                   // |divisor|
        md_cnt <= 6'd0;
        state <= S_DIV_LOOP;
    end
    S_DIV_LOOP: begin
        if (md_cnt == 6'd0) begin
            // overflow pre-check: high half must be < divisor
            if (!div_diff[33]) begin
                sr[1] <= 1'b1; sr[0] <= 1'b0;
                state <= END_ST;
            end else begin
                md_rem <= {1'b0, md_acc[63:32]};
                md_q   <= md_acc[31:0];
                md_cnt <= 6'd1;
            end
        end else begin
            if (!div_diff[33]) begin
                md_rem <= div_diff[32:0];
                md_q   <= {md_q[30:0], 1'b1};
            end else begin
                md_rem <= div_r2;
                md_q   <= {md_q[30:0], 1'b0};
            end
            md_cnt <= md_cnt + 6'd1;
            if (md_cnt == 6'd32) state <= S_DIV_FIN;
        end
    end
    S_DIV_FIN: begin : divfin
        logic ovf;
        if (md_word)
            ovf = md_signed ? (md_neg ? (md_q > 32'h8000) : (md_q > 32'h7FFF)) : (md_q > 32'hFFFF);
        else
            ovf = md_signed ? (md_neg ? (md_q > 32'h8000_0000) : (md_q > 32'h7FFF_FFFF)) : 1'b0;
        sr[0] <= 1'b0;
        tmp <= md_sum[31:0];                       // signed remainder
        if (ovf) begin
            sr[1] <= 1'b1;
            state <= END_ST;
        end else begin
            sr[1] <= 1'b0;
            if (!md_word && (md_long64 || ext[2:0] != ext[14:12]))
                `WR_D(ext[2:0], md_sum[31:0], SZ_L)
            state <= S_DIV_FIN2;
        end
    end
    S_DIV_FIN2: begin
        if (md_word) begin
            `WR_D(ir_x, {tmp[15:0], md_sum[15:0]}, SZ_L)
            sr[3] <= md_sum[15];
            sr[2] <= (md_sum[15:0] == 16'd0);
        end else begin
            `WR_D(ext[14:12], md_sum[31:0], SZ_L)
            sr[3] <= md_sum[31];
            sr[2] <= (md_sum[31:0] == 32'd0);
        end
        state <= END_ST;
    end

    // =========================================================================
    // shifts / rotates
    // =========================================================================
    S_SHIFT: begin
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
            `WR_D(ir_r, sh_o[31:0], ir_sz)
            sr[4:0] <= sh_o[36:32];
            state <= END_ST;
        end
    end
    S_ROX: begin
        if (cnt6 == 6'd0) begin
            `WR_D(ir_r, tmp, sz)
            sr[3] <= msbs(tmp, sz);
            sr[2] <= (zxs(tmp, sz) == 32'd0);
            sr[1] <= 1'b0;
            sr[0] <= sr[4];
            state <= END_ST;
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
            res <= sh_o[31:0];
            sr[4:0] <= sh_o[36:32];
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
        state <= (IR[7:6] == 2'b00) ? END_ST : G_WB;
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
        // port A = D[ext[8:6]] (offset), port B = D[ext[2:0]] (width)
        bf_off <= ext[11] ? rA : {27'd0, ext[10:6]};
        bf_w   <= ext[5] ? ((rB[4:0] == 5'd0) ? 6'd32 : {1'b0, rB[4:0]})
                         : ((ext[4:0] == 5'd0) ? 6'd32 : {1'b0, ext[4:0]});
        bf_reg <= (ea_kind == K_D);
        tmp    <= rC;                              // register operand D[ir_r]
        state  <= S_BF0;
    end
    S_BF0: begin
        tmp2 <= rB;                                // BFINS source D[ext[14:12]]
        if (bf_reg) begin
            // register: rotate the field to the top (shifter: ROL Dn, offset)
            bf_buf <= {sh_o[31:0], 8'd0};
            state  <= S_BF3;
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
    S_BF3: begin
        sr[3] <= bf_al[39];
        sr[2] <= (bf_fld == 32'd0);
        sr[1:0] <= 2'b00;
        if (IR[10:8] == 3'b111) begin
            sr[3] <= bf_ins[31];
            sr[2] <= ((bf_ins & bf_w32) == 32'd0);
        end
        bf_buf <= bf_nb;
        bf_fr  <= bf_fld;
        if (!bf_modify) state <= (IR[10:8] == 3'b000) ? END_ST : S_BF6;
        else if (bf_reg) state <= S_BF5;
        else begin
            case (bf_bc)
                3'd1:       `MEMWR(bf_addr, SZ_B, {24'd0, bf_nb[39:32]}, END_ST)
                3'd2:       `MEMWR(bf_addr, SZ_W, {16'd0, bf_nb[39:24]}, END_ST)
                3'd3:       `MEMWR(bf_addr, SZ_W, {16'd0, bf_nb[39:24]}, S_BF4)
                3'd4:       `MEMWR(bf_addr, SZ_L, bf_nb[39:8], END_ST)
                default:    `MEMWR(bf_addr, SZ_L, bf_nb[39:8], S_BF4)
            endcase
        end
    end
    S_BF4: begin
        if (bf_bc == 3'd3) `MEMWR(bf_addr + 32'd2, SZ_B, {24'd0, bf_buf[23:16]}, END_ST)
        else               `MEMWR(bf_addr + 32'd4, SZ_B, {24'd0, bf_buf[7:0]}, END_ST)
    end
    S_BF6: begin
        // BFEXTU / BFEXTS / BFFFO result from the registered field
        if (IR[10:8] == 3'b101)
            `WR_D(ext[14:12], (bf_reg ? {27'd0, bf_off[4:0]} : bf_off) + {26'd0, bf_ffo}, SZ_L)
        else
            `WR_D(ext[14:12], bf_extv[31:0], SZ_L)
        state <= END_ST;
    end
    S_BF5: begin
        // register: rotate back (shifter: ROR field, offset)
        `WR_D(ir_r, sh_o[31:0], SZ_L)
        state <= END_ST;
    end

    // =========================================================================
    // CAS / CAS2  (compares through the shared ALU)
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
    S_CAS: begin
        dst <= mem_rd;
        src <= rA;                                   // Dc
        aop <= A_CMP;
        sext_src <= 1'b0;
        state <= S_CAS1W;
    end
    S_CAS1W: state <= S_CAS1;                      // compare result -> alu_q
    S_CAS1: begin
        sr[4:0] <= alu_q[36:32];
        if (aq_z) `MEMWR(ea_addr, sz, rB, END_ST)     // Du
        else begin
            `WR_D(ext[2:0], dst, sz)
            state <= END_ST;
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
            `MEMRD(rA, sz, S_CAS2R1)                 // (Rn1)
        end
    end
    S_CAS2R1: begin
        tmp <= mem_rd;
        `MEMRD(rA, sz, S_CAS2R2)                     // (Rn2)
    end
    S_CAS2R2: begin
        tmp2 <= mem_rd;
        aop <= A_CMP;
        sext_src <= 1'b0;
        state <= S_CAS2C1;
    end
    S_CAS2C1: begin
        src <= rA;                                   // Dc1
        dst <= tmp;
        state <= S_CAS2C1W;
    end
    S_CAS2C1W: state <= S_CAS2C2;                  // compare 1 -> alu_q
    S_CAS2C2: begin
        sr[4:0] <= alu_q[36:32];
        if (!aq_z) state <= S_CAS2F1;
        else begin
            src <= rB;                               // Dc2
            dst <= tmp2;
            state <= S_CAS2C2W;
        end
    end
    S_CAS2C2W: state <= S_CAS2C3;                  // compare 2 -> alu_q
    S_CAS2C3: begin
        sr[4:0] <= alu_q[36:32];
        state <= aq_z ? S_CAS2W1 : S_CAS2F1;
    end
    S_CAS2W1: begin
        `MEMWR(rA, sz, rB, S_CAS2W2)                 // Du1 -> (Rn1)
    end
    S_CAS2W2: begin
        `MEMWR(rA, sz, rB, END_ST)                    // Du2 -> (Rn2)
    end
    S_CAS2F1: begin
        `WR_D(ext2[2:0], tmp2, sz)
        state <= S_CAS2F2;
    end
    S_CAS2F2: begin
        `WR_D(ext[2:0], tmp, sz)                     // Dc1 wins if Dc1 == Dc2
        state <= END_ST;
    end

    // =========================================================================
    // PACK / UNPK
    // =========================================================================
    S_PACKX: begin
        if (!iw_hit) `NEED_IW(S_PACKX)
        else begin
            PC <= PC + 32'd2;
            bdisp <= {16'd0, iw};
            if (!IR[3]) begin
                if (!IR[7]) `WR_D(ir_x, {24'd0, pack_b(rA[15:0] + iw)}, SZ_B)                          // PACK Dx,Dy
                else        `WR_D(ir_x, {16'd0, {4'd0, rA[7:4], 4'd0, rA[3:0]} + iw}, SZ_W)            // UNPK Dx,Dy
                state <= END_ST;
            end else begin
                `CALL_EA(3'd4, ir_r, SZ_B, IR[7] ? S_UNPKM1 : S_PACKM1)
            end
        end
    end
    S_PACKM1: begin
        `MEMRD(ea_addr, SZ_B, S_PACKM2)
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
            `MEMWR(ea_addr, SZ_B, {24'd0, tmp2[15:8]}, END_ST)
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
            state <= END_ST;
            if (IR[0]) begin
                // Rn -> Rc  (port A = Rn)
                case (iw[11:0])
                    12'h000: sfc <= rA[2:0];
                    12'h001: dfc <= rA[2:0];
                    12'h002: begin
                        cacr <= {30'd0, rA[1:0]};
                        if (rA[3]) ic_valid <= 64'd0;
                        else if (rA[2]) ic_valid[caar[7:2]] <= 1'b0;
                    end
                    12'h800: `RFW(P_USP, rA, SZ_L)
                    12'h801: vbr <= rA;
                    12'h802: caar <= rA;
                    12'h803: `RFW(P_MSP, rA, SZ_L)
                    12'h804: `RFW(P_ISP, rA, SZ_L)
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
        if (ext[11]) `MEMX(ea_addr, sz, 1'b1, rA, dfc, END_ST)   // 68020 stores post-EA An
        else         `MEMX(ea_addr, sz, 1'b0, 32'd0, sfc, S_MOVES1)
    end
    S_MOVES1: begin
        if (ext[15]) `WR_R(ext[15:12], sxs(mem_rd, sz))
        else         `WR_D(ext[14:12], mem_rd, sz)
        state <= END_ST;
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
            state <= END_ST;
        end
    end
    S_RESETI: begin
        reset_out <= 1'b1;
        reset_cnt <= reset_cnt + 32'd1;
        if (reset_cnt == RESET_CYCLES) begin
            reset_out <= 1'b0;
            state <= END_ST;
        end
    end

    // =========================================================================
    // interrupt acknowledge
    // =========================================================================
    S_INT0: begin
        exc_lvl  <= ipl_q;
        in_iack  <= 1'b1;
        if (ipl_q == 3'd7) nmi_latch <= 1'b0;
        stop_pending <= 1'b0;                    // an interrupt ends STOP
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

    // register file write: request registered here, written on the next clock
    wq_en <= w_en; wq_a <= w_a; wq_d <= w_d; wq_sz <= w_sz;
    if (wq_en) begin
        rf_b0[wq_a] <= wq_d[7:0];
        if (wq_sz != SZ_B) rf_b1[wq_a] <= wq_d[15:8];
        if (wq_sz == SZ_L) rf_hi[wq_a] <= wq_d[31:16];
        if (DEBUG) shadow[wq_a] <= merge(shadow[wq_a], wq_d, wq_sz);
    end
end

endmodule
