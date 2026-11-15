// Synthesis-only top: the fx68k-compatible CPU with just its 68000 bus pins (no debug ports)
module syn_top (
    input  wire clk, input wire HALTn, input wire extReset, input wire pwrUp, input wire enPhi1, input wire enPhi2,
    output wire eRWn, ASn, LDSn, UDSn, FC0, FC1, FC2, BGn, oRESETn, oHALTEDn,
    input  wire DTACKn, VPAn, BERRn, BRn, BGACKn, IPL0n, IPL1n, IPL2n,
    input  wire [15:0] iEdb, output wire [15:0] oEdb, output wire [23:1] eab
);
    wire [31:1] eab_full;
    wire E, VMAn, r; wire [31:0] p, u, i, m, v; wire [15:0] s; wire [255:0] d, a;
    m68020_fx68k #(.DEBUG(1'b0)) cpu (
        .clk(clk), .HALTn(HALTn), .extReset(extReset), .pwrUp(pwrUp), .enPhi1(enPhi1), .enPhi2(enPhi2),
        .eRWn(eRWn), .ASn(ASn), .LDSn(LDSn), .UDSn(UDSn), .E(E), .VMAn(VMAn), .FC0(FC0), .FC1(FC1), .FC2(FC2),
        .BGn(BGn), .oRESETn(oRESETn), .oHALTEDn(oHALTEDn), .DTACKn(DTACKn), .VPAn(VPAn), .BERRn(BERRn),
        .BRn(BRn), .BGACKn(BGACKn), .IPL0n(IPL0n), .IPL1n(IPL1n), .IPL2n(IPL2n),
        .iEdb(iEdb), .oEdb(oEdb), .eab(eab_full),
        .dbg_retire(r), .dbg_pc(p), .dbg_sr(s), .dbg_d(d), .dbg_a(a), .dbg_usp(u), .dbg_isp(i), .dbg_msp(m), .dbg_vbr(v));
    assign eab = eab_full[23:1];
endmodule
