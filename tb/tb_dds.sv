// tb_dds.sv - self-checking testbench for the SPI-configured DDS core
//
//   iverilog -g2012 -o tb_dds.vvp tb_dds.sv ../rtl/*.v
//   vvp tb_dds.vvp
//
// The SPI master model is bit-accurate mode 0 (CPOL=0, CPHA=0, MSB first,
// 16-bit words, CS framing) and clocks at 9 MHz against a 100 MHz dds_clk -
// exactly what the STM32F103 firmware in MCU/STM32F1 produces.

`timescale 1ns / 1ps

module tb_dds;

localparam real CLK_MHZ  = 100.0;                  // dds_clk
localparam real CLK_NS   = 1000.0 / CLK_MHZ;       // 10 ns
`ifdef SCK_FAST
// the documented limit of the oversampled slave: f_sck = dds_clk / 6
localparam real SCK_MHZ  = 16.0;
`else
localparam real SCK_MHZ  = 9.0;                    // STM32 SPI1 at 72/8 MHz
`endif
localparam real SCK_HALF = 500.0 / SCK_MHZ;

// register word addresses
localparam ID=6'h00, VER=6'h01, CTRL=6'h02, STATUS=6'h03,
           FWORD_L=6'h04, FWORD_H=6'h05, POW=6'h06, AMP=6'h07, DUTY=6'h08,
           OFFSET=6'h09, UPDATE=6'h0A, SWCTRL=6'h0B,
           FSTART_L=6'h0C, FSTART_H=6'h0D, FSTOP_L=6'h0E, FSTOP_H=6'h0F,
           FDELTA_L=6'h10, FDELTA_H=6'h11, RATE_L=6'h12, RATE_H=6'h13,
           PROF_CTRL=6'h14, P0F_L=6'h15, P0F_H=6'h16, P1F_L=6'h17, P1F_H=6'h18,
           P2F_L=6'h19, P2F_H=6'h1A, P3F_L=6'h1B, P3F_H=6'h1C,
           P0POW=6'h1D, P1POW=6'h1E, P2POW=6'h1F, P3POW=6'h20,
           IRQ_EN=6'h21, IRQ_STAT=6'h22;
localparam [13:0] RAM_BASE = 14'h2000;

localparam W_SIN=3'd0, W_COS=3'd1, W_SQ=3'd2, W_TRI=3'd3, W_RMP=3'd4, W_USR=3'd5;
localparam M_FIXED=2'd0, M_SWEEP=2'd1, M_PROF=2'd2;
localparam SW_SINGLE=2'd0, SW_SAW=2'd1, SW_UPDOWN=2'd2;

function [15:0] ctrl_word(input en, input [2:0] wave, input [1:0] fmode,
                          input w12, input fmt2c, input srst);
    ctrl_word = {1'b0, 1'b0, fmt2c, w12, 2'b00, fmode, 1'b0, wave, 2'b00, srst, en};
endfunction

// ----------------------------------------------------------------- DUT
reg         clk = 0;
reg         rstn = 0;
reg         sck = 0, cs_n = 1, mosi = 0;
wire        miso, miso_oe;
reg         ch0_trig = 0, ch1_trig = 0;
reg  [1:0]  ch0_hop = 2'd0, ch1_hop = 2'd0;
wire [13:0] dac0, dac1;
wire        dv0, dv1;
wire [1:0]  irq;

always #(CLK_NS/2.0) clk = ~clk;

dds_top #(
    .NUM_CH(2), .SIN_LUT_FILE("../rtl/dds_sin_lut.mem")
) dut (
    .dds_clk(clk), .dds_rstn(rstn),
    .spi_sck(sck), .spi_cs_n(cs_n), .spi_mosi(mosi),
    .spi_miso(miso), .spi_miso_oe(miso_oe),
    .ch0_sweep_trig(ch0_trig), .ch0_hop_sel(ch0_hop),
    .ch1_sweep_trig(ch1_trig), .ch1_hop_sel(ch1_hop),
    .ch0_dac_data(dac0), .ch0_dac_valid(dv0),
    .ch1_dac_data(dac1), .ch1_dac_valid(dv1),
    .dds_irq(irq)
);

// signed views (most tests select two's complement output)
wire signed [13:0] s0 = dac0;
wire signed [13:0] s1 = dac1;

// ------------------------------------------------------- SPI master model
// Mode 0: MOSI changes on the falling edge, both ends sample on the rising.
task spi_word(input [15:0] tx, output [15:0] rx);
    integer i;
    begin
        for (i = 15; i >= 0; i = i - 1) begin
            mosi = tx[i];
            #(SCK_HALF);
            sck = 1;
            rx[i] = miso;
            #(SCK_HALF);
            sck = 0;
        end
    end
endtask

task spi_write(input ch, input [13:0] addr, input integer n,
               input [15:0] d0, input [15:0] d1);
    reg [15:0] junk;
    begin
        cs_n = 0;  #(SCK_HALF);
        spi_word({1'b1, ch, addr}, junk);
        spi_word(d0, junk);
        if (n > 1) spi_word(d1, junk);
        #(SCK_HALF);
        cs_n = 1;  #(SCK_HALF * 4);        // the commit happens on this edge
    end
endtask

task wr(input ch, input [5:0] addr, input [15:0] data);
    begin spi_write(ch, {8'h00, addr}, 1, data, 16'h0); end
endtask

// a 32-bit value in one frame -> commits atomically, no intermediate frequency
task wr32(input ch, input [5:0] addr_lo, input [31:0] data);
    begin spi_write(ch, {8'h00, addr_lo}, 2, data[15:0], data[31:16]); end
endtask

task wr_ram(input ch, input [11:0] addr, input [15:0] data);
    begin spi_write(ch, RAM_BASE | {2'b00, addr}, 1, data, 16'h0); end
endtask

// read frame: CMD, turnaround word, then data
task rd(input ch, input [13:0] addr, output [15:0] data);
    reg [15:0] junk;
    begin
        cs_n = 0;  #(SCK_HALF);
        spi_word({1'b0, ch, addr}, junk);
        spi_word(16'h0000, junk);
        spi_word(16'h0000, data);
        #(SCK_HALF);
        cs_n = 1;  #(SCK_HALF * 4);
    end
endtask

task rdr(input ch, input [5:0] addr, output [15:0] data);
    begin rd(ch, {8'h00, addr}, data); end
endtask

// -------------------------------------------------------------- checking
integer errors = 0;
integer checks = 0;

task check(input [8*40:1] name, input integer got, input integer exp);
    begin
        checks = checks + 1;
        if (got !== exp) begin
            errors = errors + 1;
            $display("  FAIL %0s: got %0d, expected %0d", name, got, exp);
        end else
            $display("  ok   %0s = %0d", name, got);
    end
endtask

task check_near(input [8*40:1] name, input integer got, input integer exp,
                input integer tol);
    begin
        checks = checks + 1;
        if ((got > exp + tol) || (got < exp - tol)) begin
            errors = errors + 1;
            $display("  FAIL %0s: got %0d, expected %0d +/- %0d", name, got, exp, tol);
        end else
            $display("  ok   %0s = %0d (exp %0d +/- %0d)", name, got, exp, tol);
    end
endtask

// period of channel 0 in dds_clk cycles, timed over nper rising zero crossings
task measure_period(input integer nper, output integer cycles);
    integer t0, t1, seen;
    reg prev;
    begin
        seen = 0; t0 = 0; t1 = 0;
        @(posedge clk);
        prev = s0[13];
        while (seen <= nper) begin
            @(posedge clk);
            if (prev && !s0[13]) begin              // negative -> positive
                if (seen == 0) t0 = $time;
                t1 = $time;
                seen = seen + 1;
            end
            prev = s0[13];
        end
        cycles = ((t1 - t0) / CLK_NS) / nper;
    end
endtask

task measure_peaks(input integer n, output integer pmax, output integer pmin);
    integer i;
    begin
        pmax = -9000; pmin = 9000;
        for (i = 0; i < n; i = i + 1) begin
            @(posedge clk);
            if (s0 > pmax) pmax = s0;
            if (s0 < pmin) pmin = s0;
        end
    end
endtask

task measure_duty(input integer n, output integer pct);
    integer i, hi;
    begin
        hi = 0;
        for (i = 0; i < n; i = i + 1) begin
            @(posedge clk);
            if (s0 > 0) hi = hi + 1;
        end
        pct = (hi * 100) / n;
    end
endtask

// ------------------------------------------------------------------ tests
reg [15:0] v;
integer    per, pmax, pmin, pct, i, f_lo, f_hi;

initial begin
    $dumpfile("tb_dds.vcd");
    $dumpvars(0, tb_dds);

    repeat (10) @(posedge clk);
    rstn = 1;
    repeat (10) @(posedge clk);

    $display("\n=== 1. identification ===");
    rdr(0, ID,  v);  check("ch0 ID",      v, 16'h4453);
    rdr(0, VER, v);  check("ch0 VERSION", v, 16'h0200);
    rdr(1, ID,  v);  check("ch1 ID",      v, 16'h4453);

    $display("\n=== 2. reset defaults ===");
    rdr(0, AMP,    v);  check("AMP default 1.0",   v, 16'h8000);
    rdr(0, DUTY,   v);  check("DUTY default 50pc", v, 16'h8000);
    rdr(0, UPDATE, v);  check("AUTO default",      v[1], 1);
    check("idle output = mid-scale", dac0, 14'h2000);   // offset binary

    $display("\n=== 3. register access ===");
    wr(0, POW, 16'h1234);  rdr(0, POW, v);  check("POW readback", v, 16'h1234);
    wr(0, AMP, 16'hFFFF);  rdr(0, AMP, v);  check("AMP clamps to 1.0", v, 16'h8000);
    wr32(0, FSTART_L, 32'hDEAD_BEEF);
    rdr(0, FSTART_L, v);  check("burst word 0 -> FSTART_L", v, 16'hBEEF);
    rdr(0, FSTART_H, v);  check("burst word 1 -> FSTART_H", v, 16'hDEAD);
    wr(0, POW, 16'h0000);
    wr(0, AMP, 16'h8000);

    $display("\n=== 4. fixed-frequency sine ===");
    // FWORD = 2^28 -> f = dds_clk / 16 -> 16 samples per period.
    // SRST goes last so the phase grid restarts at 0 and one sample lands
    // exactly on the crest; otherwise the peak we see is 8191*cos(phase error)
    // and depends on wherever the accumulator happened to be.
    wr32(0, FWORD_L, 32'h1000_0000);
    wr(0, CTRL, ctrl_word(1'b1, W_SIN, M_FIXED, 1'b0, 1'b1, 1'b1));
    repeat (50) @(posedge clk);
    measure_period(8, per);
    check("sine period (clk cycles)", per, 16);
    measure_peaks(64, pmax, pmin);
    check_near("sine peak",   pmax,  8191, 60);
    check_near("sine trough", pmin, -8191, 60);

    $display("\n=== 5. retune ===");
    wr32(0, FWORD_L, 32'h0800_0000);          // dds_clk / 32
    repeat (50) @(posedge clk);
    measure_period(8, per);
    check("retuned period", per, 32);

    $display("\n=== 6. deferred commit (AUTO = 0) ===");
    wr(0, UPDATE, 16'h0000);                  // AUTO = 0
    wr32(0, FWORD_L, 32'h1000_0000);          // shadow only
    repeat (50) @(posedge clk);
    measure_period(8, per);
    check("frequency unchanged", per, 32);
    wr(0, UPDATE, 16'h0001);                  // UPD -> commit
    repeat (50) @(posedge clk);
    measure_period(8, per);
    check("committed by UPDATE.UPD", per, 16);
    wr(0, UPDATE, 16'h0002);                  // AUTO = 1

    $display("\n=== 7. amplitude and DC offset ===");
    wr(0, AMP, 16'h4000);                     // 0.5
    wr(0, CTRL, ctrl_word(1'b1, W_SIN, M_FIXED, 1'b0, 1'b1, 1'b1));  // realign
    repeat (50) @(posedge clk);
    measure_peaks(64, pmax, pmin);
    check_near("half-scale peak", pmax, 4096, 4);
    wr(0, AMP, 16'h8000);
    // pure DC: freeze the accumulator, sine(0) = 0, so output = OFFSET
    wr32(0, FWORD_L, 32'h0000_0000);
    wr(0, CTRL, ctrl_word(1'b1, W_SIN, M_FIXED, 1'b0, 1'b1, 1'b1));   // + SRST
    wr(0, OFFSET, 16'd2000);
    repeat (50) @(posedge clk);
    check_near("DC output = OFFSET", s0, 2000, 4);
    wr(0, OFFSET, 16'h0000);

    $display("\n=== 8. square duty cycle ===");
    wr32(0, FWORD_L, 32'h0010_0000);
    wr(0, DUTY, 16'h4000);                    // 25 %
    wr(0, CTRL, ctrl_word(1'b1, W_SQ, M_FIXED, 1'b0, 1'b1, 1'b0));
    repeat (200) @(posedge clk);
    measure_duty(4096 * 4, pct);              // 4 periods
    check_near("square duty pct", pct, 25, 2);
    wr(0, DUTY, 16'h8000);

    $display("\n=== 9. triangle and ramp ===");
    wr(0, CTRL, ctrl_word(1'b1, W_TRI, M_FIXED, 1'b0, 1'b1, 1'b0));
    repeat (200) @(posedge clk);
    measure_peaks(4096 * 2, pmax, pmin);
    check_near("triangle peak",   pmax,  8191, 60);
    check_near("triangle trough", pmin, -8192, 60);
    wr(0, CTRL, ctrl_word(1'b1, W_RMP, M_FIXED, 1'b0, 1'b1, 1'b0));
    repeat (200) @(posedge clk);
    measure_peaks(4096 * 2, pmax, pmin);
    check_near("ramp peak",   pmax,  8191, 60);
    check_near("ramp trough", pmin, -8192, 60);

    $display("\n=== 10. user waveform RAM ===");
    // +FS over the first half of the table, -FS over the second
    for (i = 0; i < 8; i = i + 1)
        wr_ram(0, i[11:0], (i < 4) ? 16'h1FFF : 16'h2000);   // +8191 / -8192
    wr(0, CTRL, ctrl_word(1'b1, W_USR, M_FIXED, 1'b0, 1'b1, 1'b1));  // + SRST
    wr32(0, FWORD_L, 32'h0010_0000);          // playback addr = phase[31:20]
    repeat (100) @(posedge clk);
    measure_peaks(4096 * 2, pmax, pmin);
    check("user RAM +FS sample", pmax,  8191);
    check("user RAM -FS sample", pmin, -8192);

    $display("\n=== 11. 12-bit output ===");
    wr32(0, FWORD_L, 32'h1000_0000);
    wr(0, CTRL, ctrl_word(1'b1, W_SIN, M_FIXED, 1'b1, 1'b1, 1'b1));  // OUT12 + SRST
    repeat (50) @(posedge clk);
    measure_peaks(64, pmax, pmin);
    // 8191 -> round to 12 bits = 2047 (saturated) -> MSB-aligned = 8188
    check_near("12-bit peak, MSB-aligned", pmax, 8188, 4);
    check("12-bit LSBs zero", dac0[1:0], 2'b00);

    $display("\n=== 12. offset-binary coding ===");
    wr32(0, FWORD_L, 32'h0000_0000);
    wr(0, CTRL, ctrl_word(1'b1, W_SIN, M_FIXED, 1'b0, 1'b0, 1'b1));  // + SRST
    repeat (50) @(posedge clk);
    check_near("mid-scale at phase 0", dac0, 14'h2000, 4);

    $display("\n=== 13. linear sweep ===");
    wr(0, CTRL, ctrl_word(1'b0, W_SIN, M_FIXED, 1'b0, 1'b1, 1'b1));  // off + SRST
    wr(0, IRQ_EN, 16'h0003);
    wr32(0, FSTART_L, 32'h0400_0000);         // dds_clk / 64
    wr32(0, FSTOP_L,  32'h1000_0000);         // dds_clk / 16
    wr32(0, FDELTA_L, 32'h0000_8000);
    wr32(0, RATE_L,   32'd1);                 // one step per dds_clk
    wr(0, SWCTRL, {14'h0, SW_SINGLE});
    wr(0, CTRL, ctrl_word(1'b1, W_SIN, M_SWEEP, 1'b0, 1'b1, 1'b0));
    repeat (20) @(posedge clk);
    measure_period(4, f_lo);                  // sitting at FSTART
    wr(0, SWCTRL, 16'h0100 | {14'h0, SW_SINGLE});   // START
    rdr(0, STATUS, v);
    check("sweep active", v[0], 1);
    // (0x10000000 - 0x04000000) / 0x8000 = 6144 steps at 1 clk each
    repeat (8000) @(posedge clk);
    rdr(0, STATUS, v);
    check("sweep finished", v[0], 0);
    rdr(0, IRQ_STAT, v);
    check("DONE irq flag", v[0], 1);
    check("irq line", irq[0], 1);
    measure_period(4, f_hi);
    check("ends at FSTOP", f_hi, 16);
    check("started below FSTOP", (f_lo > f_hi) ? 1 : 0, 1);
    wr(0, IRQ_STAT, 16'h0007);                // W1C
    rdr(0, IRQ_STAT, v);
    check("irq flags cleared", v, 16'h0000);
    wr(0, IRQ_EN, 16'h0000);

    $display("\n=== 14. profile hopping (FSK) ===");
    wr(0, CTRL, ctrl_word(1'b0, W_SIN, M_FIXED, 1'b0, 1'b1, 1'b1));  // off + SRST
    wr32(0, P0F_L, 32'h1000_0000);            // dds_clk / 16
    wr32(0, P1F_L, 32'h0800_0000);            // dds_clk / 32
    wr32(0, P2F_L, 32'h0400_0000);            // dds_clk / 64
    wr32(0, P3F_L, 32'h0200_0000);            // dds_clk / 128
    wr(0, PROF_CTRL, 16'h0000);               // profile 0, register-selected
    wr(0, CTRL, ctrl_word(1'b1, W_SIN, M_PROF, 1'b0, 1'b1, 1'b0));
    repeat (50) @(posedge clk);
    measure_period(4, per);  check("profile 0", per, 16);
    wr(0, PROF_CTRL, 16'h0002);               // profile 2
    repeat (50) @(posedge clk);
    measure_period(4, per);  check("profile 2", per, 64);
    wr(0, PROF_CTRL, 16'h0010);               // EXT_SEL: follow hop_sel pins
    ch0_hop = 2'd1;
    repeat (60) @(posedge clk);
    measure_period(4, per);  check("hop pin -> profile 1", per, 32);
    rdr(0, STATUS, v);
    check("STATUS.ACTIVE_PROF", v[3:2], 1);
    ch0_hop = 2'd0;
    repeat (60) @(posedge clk);
    measure_period(4, per);  check("hop pin -> profile 0", per, 16);

    $display("\n=== 15. channel independence ===");
    wr(0, CTRL, ctrl_word(1'b1, W_SIN, M_FIXED, 1'b0, 1'b1, 1'b1));  // + SRST
    wr32(0, FWORD_L, 32'h1000_0000);
    wr32(1, FWORD_L, 32'h0800_0000);
    wr(1, CTRL, ctrl_word(1'b1, W_SQ, M_FIXED, 1'b0, 1'b1, 1'b0));
    repeat (100) @(posedge clk);
    rdr(0, FWORD_H, v);  check("ch0 FWORD_H intact", v, 16'h1000);
    rdr(1, FWORD_H, v);  check("ch1 FWORD_H", v, 16'h0800);
    measure_period(8, per);
    check("ch0 still dds_clk/16 sine", per, 16);
    check("ch1 square sits on a rail", ((s1 == 8191) || (s1 == -8191)) ? 1 : 0, 1);
    wr(1, CTRL, ctrl_word(1'b0, W_SQ, M_FIXED, 1'b0, 1'b1, 1'b0));
    repeat (20) @(posedge clk);
    check("ch1 disabled -> mid-scale", dac1, 14'h0000);   // two's complement mid
    check("ch1 dac_valid low", dv1, 0);

    $display("\n=========================================");
    if (errors == 0)
        $display("ALL TESTS PASSED (%0d checks)", checks);
    else
        $display("%0d of %0d CHECKS FAILED", errors, checks);
    $display("=========================================\n");
    $finish;
end

initial begin
    #10_000_000;
    $display("TIMEOUT");
    $finish;
end

endmodule
