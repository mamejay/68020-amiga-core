`timescale 1ns/100ps
// SD-card path test: tb_top plus an SPI slave model on the iCESugar-Pro micro-SD pins.
// MISO sends an incrementing byte counter; MOSI bytes are logged. The test program
// (sim/b64/sdtest.S) replays spisd.device's own register routines and self-checks.
module tb;
  reg clk12 = 0;
  always #5.874 clk12 = ~clk12;
  reg ph7 = 0; integer c = 0;
  always @(posedge clk12) begin
    c <= (c==11) ? 0 : c + 1;
    if (c==11) ph7 <= 1; else if (c==5) ph7 <= 0;
  end
  wire #6.0 clk_7m = ph7;

  integer dtack_ns, dvalid_ns, run_us, trace;
  reg [1023:0] binf;
  wire [23:1] cpu_a;  wire [15:0] cpu_d;  wire [2:0] cpu_fc;
  wire cpu_rw, cpu_as_n, cpu_uds_n, cpu_lds_n, cpu_vma_n, cpu_e, cpu_bg_n;
  wire cpu_reset_n, cpu_halt_n;
  reg  dtack_n = 1, vpa_n = 1, berr_n = 1, br_n = 1, bgack_n = 1;
  reg  [2:0] ipl_n = 3'b111;
  pullup(cpu_reset_n); pullup(cpu_halt_n);

  reg [7:0] mem [0:524287];
  wire [18:0] ba = {cpu_a[18:1], 1'b0};
  reg d_valid = 0;
  always @(negedge cpu_as_n) begin #(dvalid_ns) d_valid = 1; end
  always @(posedge cpu_as_n) begin #2 d_valid = 0; end
  assign cpu_d = (!cpu_as_n && cpu_rw && d_valid) ? {mem[ba], mem[ba|1]} : 16'bz;
  always @(negedge cpu_as_n) begin #(dtack_ns) dtack_n <= 0; end
  always @(posedge cpu_as_n) dtack_n <= 1;
  // latch write data while each strobe is asserted (Agnus/Gary act on DS, which can rise before AS)
  always @(posedge clk12) if (!cpu_as_n && !cpu_rw) begin
    if (!cpu_uds_n) mem[ba]   <= cpu_d[15:8];
    if (!cpu_lds_n) mem[ba|1] <= cpu_d[7:0];
  end

  // physical-bus write monitor: DS state over the cycle
  integer ds_seen_u = 0, ds_seen_l = 0;
  always @(negedge cpu_as_n) begin ds_seen_u = 0; ds_seen_l = 0; end
  always @(posedge clk12) if (!cpu_as_n && !cpu_rw) begin
    if (!cpu_uds_n) ds_seen_u = ds_seen_u + 1;
    if (!cpu_lds_n) ds_seen_l = ds_seen_l + 1;
  end
  always @(posedge cpu_as_n) if (!cpu_rw && trace > 0)
    $display("%9.2f us  BUS W A=%06x uds_n=%b lds_n=%b at AS rise, DS low for U%0d/L%0d clocks, D=%04x",
             $realtime/1000.0, ba, cpu_uds_n, cpu_lds_n, ds_seen_u, ds_seen_l, cpu_d);

  // ---- SPI slave (mode 0): MISO = byte counter, changes on SCLK fall; MOSI sampled on rise
  wire sd_clk, sd_mosi; wire [3:0] sd_d;
  reg  [31:0] miso_bits = 0;
  wire [7:0]  miso_byte = miso_bits[10:3];
  assign sd_d[0] = miso_byte[3'd7 - miso_bits[2:0]];
  reg  [7:0]  mosi_sr = 0; integer mosi_n = 0, mosi_nonff = 0, mosi_bad = 0, mosi_exp_i = 0;
  reg  [7:0]  mosi_log [0:4095];
  always @(negedge sd_clk) if (!sd_d[3]) miso_bits <= miso_bits + 1;
  always @(posedge sd_clk) if (!sd_d[3]) begin
    mosi_sr = {mosi_sr[6:0], sd_cmd_w};
    mosi_n = mosi_n + 1;
    if (mosi_n[2:0] == 0) begin
      if (mosi_sr != 8'hFF) begin
        if (mosi_nonff < 4096) mosi_log[mosi_nonff] = mosi_sr;
        mosi_nonff = mosi_nonff + 1;
      end
    end
  end
  wire sd_cmd_w = sd_mosi;
  wire [15:0] sdq; wire [12:0] sda; wire [1:0] sdba;
  wire led_r, led_g, led_b;
  base64_top #(.PWRUP_BIT(9)) dut (
    .clk_7m(clk_7m), .clk_12x(clk12),
    .cpu_a(cpu_a), .cpu_d(cpu_d), .cpu_fc(cpu_fc),
    .cpu_rw(cpu_rw), .cpu_as_n(cpu_as_n),
    .cpu_uds_n(cpu_uds_n), .cpu_lds_n(cpu_lds_n),
    .cpu_dtack_n(dtack_n), .cpu_vpa_n(vpa_n), .cpu_vma_n(cpu_vma_n),
    .cpu_e(cpu_e), .cpu_berr_n(berr_n),
    .cpu_br_n(br_n), .cpu_bg_n(cpu_bg_n), .cpu_bgack_n(bgack_n),
    .cpu_ipl_n(ipl_n), .cpu_reset_n(cpu_reset_n), .cpu_halt_n(cpu_halt_n),
    .cfgin_n(1'b0), .cfgout_n(),
    .cbt_oe_d0_7_n(), .cbt_oe_d8_15_n(), .cbt_oe_ctl_hi_n(),
    .cbt_oe_a8_17_n(), .cbt_oe_a1_7_n(),
    .sdram_dq(sdq), .sdram_a(sda), .sdram_ba(sdba),
    .sdram_clk(), .sdram_cke(), .sdram_cs_n(), .sdram_ras_n(),
    .sdram_cas_n(), .sdram_we_n(), .sdram_dqm(),
    .sd_clk(sd_clk), .sd_cmd(sd_mosi), .sd_d(sd_d),
    .led_r(led_r), .led_g(led_g), .led_b(led_b)
  );
  sdram_model_checked #(.CAS_LAT(2)) u_mem (
    .clk(dut.sdram_clk), .a(sda), .ba(sdba), .dq(sdq), .dqm(dut.sdram_dqm),
    .cke(dut.sdram_cke), .cs_n(dut.sdram_cs_n), .ras_n(dut.sdram_ras_n),
    .cas_n(dut.sdram_cas_n), .we_n(dut.sdram_we_n));

  integer fd, n, i, ncyc = 0;
  initial begin
    if (!$value$plusargs("dtack=%d", dtack_ns)) dtack_ns = 294;
    if (!$value$plusargs("dvalid=%d", dvalid_ns)) dvalid_ns = 400;
    if (!$value$plusargs("us=%d", run_us)) run_us = 3000;
    if (!$value$plusargs("trace=%d", trace)) trace = 40;
    if (!$value$plusargs("bin=%s", binf)) binf = "../tests/arith.bin";
    for (i = 0; i < 524288; i = i + 1) mem[i] = 8'h4E;
    fd = $fopen(binf, "rb");
    if (fd == 0) begin $display("cannot open bin"); $finish; end
    n = $fread(mem, fd); $fclose(fd);
    $display("loaded %0d bytes", n);
  end

  // core-side bus trace (internal address, before the bridge)
  reg as_d = 1;
  always @(posedge clk12) begin
    as_d <= dut.core_as_n;
    if (as_d && !dut.core_as_n) ncyc = ncyc + 1;
    if (!as_d && dut.core_as_n && ncyc <= trace)
      $display("%9.2f us  cyc %4d  %s A=%06x FC=%0d %s%s D=%04x", $realtime/1000.0, ncyc,
               dut.core_rw ? "R" : "W", {dut.core_a[23:1], 1'b0}, {dut.core_fc2, dut.core_fc1, dut.core_fc0},
               dut.core_uds_n ? "-" : "U", dut.core_lds_n ? "-" : "L",
               dut.core_rw ? dut.core_iedb : dut.core_dout);
  end
  reg h_d = 0, r_d = 1, er_d = 1;
  always @(posedge clk12) begin
    h_d <= ~dut.core_ohalted_n; er_d <= dut.ext_reset;
    if (~dut.core_ohalted_n && !h_d) $display("%9.2f us  *** core HALTED (double bus fault) after %0d bus cycles", $realtime/1000.0, ncyc);
    if (er_d && !dut.ext_reset) $display("%9.2f us  core released from reset", $realtime/1000.0);
  end
  integer k, ok;
  initial begin
    #(run_us * 1000.0);
    // expected MOSI (non-FF) stream: 16 single TX bytes (7i+3), then 512 bulk bytes (i & $7F)
    ok = 1;
    for (k = 0; k < 16; k = k + 1) if (mosi_log[k] != ((k*7+3) & 8'hFF)) begin ok = 0; if (mosi_bad < 8) $display("mosi[%0d]=%02x exp %02x", k, mosi_log[k], (k*7+3)&8'hFF); mosi_bad = mosi_bad + 1; end
    if (mosi_nonff != 16 + 512) ok = 0;
    for (k = 0; k < 512; k = k + 1) if (mosi_log[16+k] != (k & 8'h7F)) begin ok = 0; if (mosi_bad < 8) $display("mosi bulk[%0d]=%02x exp %02x", k, mosi_log[16+k], k&8'h7F); mosi_bad = mosi_bad + 1; end
    $display("END %0.1f us: bus cycles %0d, halted=%b sd_configured=%b miso bytes=%0d mosi bytes=%0d non-FF=%0d",
      $realtime/1000.0, ncyc, ~dut.core_ohalted_n, dut.sd_configured, miso_bits/8, mosi_n/8, mosi_nonff);
    $display("RESULT word @0x100 = %02x%02x%02x%02x  (600D0000 = all pass, BAD0xxyy = test xx failed at index yy)  MOSI stream %s",
      mem[19'h100], mem[19'h101], mem[19'h102], mem[19'h103], ok ? "OK" : "BAD");
    $finish;
  end
endmodule
