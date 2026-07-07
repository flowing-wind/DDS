// tb_dds.v - self-checking testbench for dds_top
//
// Runs PCLK = dds_clk = 50 MHz (they may also be made asynchronous below).
// Tests: ID read, sine amplitude/period, amplitude scaling, square duty
// cycle, ramp/triangle monotonicity, sweep with IRQ, profile hopping,
// wave RAM readback + user waveform playback, 12-bit mode, offset binary.

`timescale 1ns/1ps

module tb_dds;

reg         PCLK = 0, dds_clk = 0;
reg         PRESETn = 0, dds_rstn = 0;
reg  [14:0] PADDR = 0;
reg         PSEL = 0, PENABLE = 0, PWRITE = 0;
reg  [31:0] PWDATA = 0;
wire [31:0] PRDATA;
wire        PREADY;
reg         sweep_trig = 0;
reg  [1:0]  hop_sel = 0;
wire [13:0] dac_data;
wire        dac_valid;
wire        dds_irq;

integer errors = 0;
integer i;

always #10 PCLK = ~PCLK;        // 50 MHz
`ifdef ASYNC_CLKS
always #4 dds_clk = ~dds_clk;   // 125 MHz, asynchronous to PCLK
`else
always #10 dds_clk = ~dds_clk;  // 50 MHz (same source as PCLK)
`endif

dds_top #(.SIN_LUT_FILE("../rtl/dds_sin_lut.mem")) dut (
    .PCLK(PCLK), .PRESETn(PRESETn),
    .PADDR(PADDR), .PSEL(PSEL), .PENABLE(PENABLE), .PWRITE(PWRITE),
    .PWDATA(PWDATA), .PRDATA(PRDATA), .PREADY(PREADY),
    .dds_clk(dds_clk), .dds_rstn(dds_rstn),
    .sweep_trig(sweep_trig), .hop_sel(hop_sel),
    .dac_data(dac_data), .dac_valid(dac_valid),
    .dds_irq(dds_irq)
);

// register offsets
localparam R_ID     = 15'h00, R_CTRL   = 15'h04, R_STATUS = 15'h08,
           R_FWORD  = 15'h0C, R_POW    = 15'h10, R_AMP    = 15'h14,
           R_DUTY   = 15'h18, R_UPDATE = 15'h1C, R_SWCTRL = 15'h20,
           R_FSTART = 15'h24, R_FSTOP  = 15'h28, R_FDELTA = 15'h2C,
           R_RATE   = 15'h30, R_PROFC  = 15'h34, R_PF0    = 15'h38,
           R_PF1    = 15'h3C, R_PF2    = 15'h40, R_PF3    = 15'h44,
           R_IRQEN  = 15'h50, R_IRQST  = 15'h54;
localparam RAM_BASE = 15'h4000;

task apb_write(input [14:0] addr, input [31:0] data);
begin
    @(posedge PCLK);
    PADDR <= addr; PWDATA <= data; PWRITE <= 1; PSEL <= 1; PENABLE <= 0;
    @(posedge PCLK);
    PENABLE <= 1;
    @(posedge PCLK);
    while (!PREADY) @(posedge PCLK);
    PSEL <= 0; PENABLE <= 0;
end
endtask

task apb_read(input [14:0] addr, output [31:0] data);
begin
    @(posedge PCLK);
    PADDR <= addr; PWRITE <= 0; PSEL <= 1; PENABLE <= 0;
    @(posedge PCLK);
    PENABLE <= 1;
    #1;
    while (!PREADY) begin @(posedge PCLK); #1; end
    data = PRDATA;
    @(posedge PCLK);
    PSEL <= 0; PENABLE <= 0;
end
endtask

task check(input [255:0] name, input cond);
begin
    if (cond)
        $display("PASS: %0s", name);
    else begin
        $display("FAIL: %0s", name);
        errors = errors + 1;
    end
end
endtask

// wait until the shadow->active transfer has been applied (STATUS.UPD_PENDING=0)
task wait_applied;
reg [31:0] st;
begin
    st = 32'h10;
    while (st[4]) apb_read(R_STATUS, st);
    repeat (12) @(posedge dds_clk);   // pipeline flush
end
endtask

// signed view of dac_data assuming two's-complement format
wire signed [13:0] dac_s = $signed(dac_data);

integer smin, smax, hi_cnt, tot_cnt;
reg [31:0] rd;
reg signed [13:0] prev_s;
integer rising, k;
integer per_meas, t_edge1, t_edge2;

initial begin
    $dumpfile("tb_dds.vcd");
    $dumpvars(0, tb_dds);

    repeat (5) @(posedge PCLK);
    PRESETn = 1; dds_rstn = 1;
    repeat (5) @(posedge PCLK);

    // ---- ID -------------------------------------------------------------
    apb_read(R_ID, rd);
    check("ID register", rd == 32'h4453_0100);

    // ---- sine, two's complement, full amplitude --------------------------
    // FWORD = 2^26 -> 64 samples per period
    apb_write(R_FWORD, 32'h0400_0000);
    apb_write(R_CTRL,  32'h0000_2001);   // EN, sine, FIXED, 14b, two's comp
    wait_applied;

    smin = 0; smax = 0;
    for (i = 0; i < 200; i = i + 1) begin
        @(posedge dds_clk);
        if (dac_valid) begin
            if (dac_s > smax) smax = dac_s;
            if (dac_s < smin) smin = dac_s;
        end
    end
    check("sine max ~ +8191", (smax >= 8180) && (smax <= 8191));
    check("sine min ~ -8191", (smin <= -8180) && (smin >= -8191));

    // ---- amplitude 0.5 ----------------------------------------------------
    apb_write(R_AMP, 32'h0000_8000);
    wait_applied;
    smin = 0; smax = 0;
    for (i = 0; i < 200; i = i + 1) begin
        @(posedge dds_clk);
        if (dac_s > smax) smax = dac_s;
        if (dac_s < smin) smin = dac_s;
    end
    check("amp 0.5 max ~ +4096", (smax >= 4088) && (smax <= 4100));
    check("amp 0.5 min ~ -4096", (smin <= -4088) && (smin >= -4100));
    apb_write(R_AMP, 32'h0001_0000);     // back to 1.0
    wait_applied;

    // ---- square wave, 25% duty -------------------------------------------
    apb_write(R_DUTY, 32'h0000_4000);
    apb_write(R_CTRL, 32'h0000_2021);    // EN, square
    wait_applied;
    hi_cnt = 0; tot_cnt = 0;
    for (i = 0; i < 256; i = i + 1) begin   // 4 periods of 64
        @(posedge dds_clk);
        tot_cnt = tot_cnt + 1;
        if (dac_s > 0) hi_cnt = hi_cnt + 1;
    end
    check("square 25% duty", (hi_cnt >= 56) && (hi_cnt <= 72)); // 64 +/- tol
    check("square levels +/-8191", (dac_s == 8191) || (dac_s == -8191));

    // ---- ramp: monotonic rising with one wrap per period ------------------
    apb_write(R_CTRL, 32'h0000_2041);    // EN, ramp
    wait_applied;
    rising = 0; k = 0; prev_s = dac_s;
    for (i = 0; i < 128; i = i + 1) begin
        @(posedge dds_clk);
        if (dac_s > prev_s) rising = rising + 1;
        if (dac_s < prev_s) k = k + 1;   // falls (wraps)
        prev_s = dac_s;
    end
    check("ramp mostly rising", rising > 120);
    check("ramp wraps ~2x in 128", (k >= 1) && (k <= 3));

    // ---- sweep: single mode, IRQ ------------------------------------------
    apb_write(R_UPDATE, 32'h0000_0000);  // AUTO off: program set atomically
    apb_write(R_FSTART, 32'h0100_0000);
    apb_write(R_FSTOP,  32'h0200_0000);
    apb_write(R_FDELTA, 32'h0010_0000);  // 16 steps
    apb_write(R_RATE,   32'd4);
    apb_write(R_IRQEN,  32'h0000_0007);
    apb_write(R_CTRL,   32'h0000_2101);  // EN, sine, SWEEP mode
    apb_write(R_UPDATE, 32'h0000_0001);  // commit
    wait_applied;
    apb_write(R_IRQST,  32'h0000_0007);  // clear stale flags (incl UPD_DONE)
    apb_write(R_SWCTRL, 32'h0000_0100);  // START, single mode
    // sweep takes ~16*4 dds_clk; poll IRQ_STAT
    rd = 0;
    for (i = 0; (i < 100) && !rd[0]; i = i + 1)
        apb_read(R_IRQST, rd);
    check("sweep SWEEP_DONE flag", rd[0] === 1'b1);
    check("irq line asserted", dds_irq === 1'b1);
    apb_read(R_STATUS, rd);
    check("sweep no longer active", rd[0] === 1'b0);
    apb_write(R_IRQST, 32'h0000_0007);   // W1C
    apb_read(R_IRQST, rd);
    check("IRQ_STAT W1C", rd[2:0] === 3'b000);
    apb_write(R_UPDATE, 32'h0000_0002);  // AUTO back on

    // ---- profile hopping: measure period via square wave -------------------
    apb_write(R_PF0, 32'h0200_0000);     // 128 samples/period
    apb_write(R_PF1, 32'h0800_0000);     // 32 samples/period
    apb_write(R_DUTY, 32'h0000_8000);
    apb_write(R_PROFC, 32'h0000_0000);   // profile 0, register select
    apb_write(R_CTRL, 32'h0000_2221);    // EN, square, PROFILE mode
    wait_applied;
    // measure period between rising edges
    @(posedge dds_clk); prev_s = dac_s;
    t_edge1 = -1; t_edge2 = -1;
    for (i = 0; (i < 400) && (t_edge2 < 0); i = i + 1) begin
        @(posedge dds_clk);
        if ((prev_s < 0) && (dac_s > 0)) begin
            if (t_edge1 < 0) t_edge1 = i; else t_edge2 = i;
        end
        prev_s = dac_s;
    end
    per_meas = t_edge2 - t_edge1;
    check("profile 0 period = 128", (per_meas >= 127) && (per_meas <= 129));

    apb_write(R_PROFC, 32'h0000_0001);   // hop to profile 1
    wait_applied;
    @(posedge dds_clk); prev_s = dac_s;
    t_edge1 = -1; t_edge2 = -1;
    for (i = 0; (i < 400) && (t_edge2 < 0); i = i + 1) begin
        @(posedge dds_clk);
        if ((prev_s < 0) && (dac_s > 0)) begin
            if (t_edge1 < 0) t_edge1 = i; else t_edge2 = i;
        end
        prev_s = dac_s;
    end
    per_meas = t_edge2 - t_edge1;
    check("profile 1 period = 32", (per_meas >= 31) && (per_meas <= 33));

    // ---- wave RAM: write, readback, playback -------------------------------
    apb_write(R_CTRL, 32'h0000_2000);    // disable while loading
    for (i = 0; i < 4096; i = i + 1)
        apb_write(RAM_BASE + 4*i, (i * 4) % 16384); // raw 14-bit pattern
    apb_read(RAM_BASE + 4*100, rd);
    check("wave RAM readback", rd[13:0] == 14'd400);

    apb_write(R_FWORD, 32'h0010_0000);   // addr += 1 per dds_clk
    apb_write(R_CTRL,  32'h0000_2051);   // EN, user wave, FIXED
    wait_applied;
    // output must step by 4 per sample (as two's complement of the pattern)
    @(posedge dds_clk); prev_s = dac_s;
    k = 0;
    for (i = 0; i < 64; i = i + 1) begin
        @(posedge dds_clk);
        if ((dac_s - prev_s) == 4) k = k + 1;
        prev_s = dac_s;
    end
    check("user waveform steps of 4", k >= 60);

    // ---- 12-bit mode + offset binary ---------------------------------------
    apb_write(R_FWORD, 32'h0400_0000);
    apb_write(R_CTRL,  32'h0000_1001);   // EN, sine, 12-bit, offset binary
    wait_applied;
    smin = 16383; smax = 0;
    for (i = 0; i < 200; i = i + 1) begin
        @(posedge dds_clk);
        if (dac_data > smax) smax = dac_data;
        if (dac_data < smin) smin = dac_data;
        if (dac_data[1:0] !== 2'b00) errors = errors + 1;
    end
    check("12b offset binary max ~0x3FFC", (smax >= 14'h3FF0) && (smax <= 14'h3FFC));
    check("12b offset binary min ~0x0000", smin <= 14'h000C);

    // ---- disable: mid-scale -------------------------------------------------
    apb_write(R_CTRL, 32'h0000_1000);    // EN=0
    repeat (20) @(posedge dds_clk);
    check("disabled -> mid-scale (offset binary)", dac_data == 14'h2000);
    check("dac_valid low when disabled", dac_valid === 1'b0);

    // ---- summary ------------------------------------------------------------
    if (errors == 0) $display("ALL TESTS PASSED");
    else             $display("TESTS FAILED: %0d error(s)", errors);
    $finish;
end

initial begin
    #20_000_000;
    $display("TIMEOUT");
    $finish;
end

endmodule
