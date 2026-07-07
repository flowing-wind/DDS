// dds_top.v - DDS IP core, top level
//
// APB-configured direct digital synthesizer with 12/14-bit parallel DAC
// output. See Doc/DDS_Datasheet.md for the register map and usage.
//
// Clocking: PCLK is the APB bus clock, dds_clk is the DAC sample clock.
// They may be the same PLL output (tie both to it) or fully asynchronous;
// all crossings are handled internally.

module dds_top #(
    parameter SIN_LUT_FILE = "dds_sin_lut.mem"
) (
    // APB
    input  wire        PCLK,
    input  wire        PRESETn,

    input  wire [14:0] PADDR,
    input  wire        PSEL,
    input  wire        PENABLE,
    input  wire        PWRITE,
    input  wire [31:0] PWDATA,
    output wire [31:0] PRDATA,
    output wire        PREADY,

    // DDS sample clock domain
    input  wire        dds_clk,
    input  wire        dds_rstn,

    // hardware pacing hooks for other IP (SPI, user logic, pins)
    input  wire        sweep_trig,   // sweep start, rising edge
    input  wire [1:0]  hop_sel,      // profile select (FSK/PSK data)

    // DAC interface (dds_clk domain)
    output wire [13:0] dac_data,
    output wire        dac_valid,

    // Interrupt Request
    output wire        dds_irq
);

// regs <-> core
wire [31:0] t_fword, t_sweep_fstart, t_sweep_fstop, t_sweep_fdelta;
wire [31:0] t_prof_fword0, t_prof_fword1, t_prof_fword2, t_prof_fword3;
wire [16:0] t_amp;
wire [15:0] t_pow, t_duty;
wire [13:0] t_offset;
wire [15:0] t_prof_pow0, t_prof_pow1, t_prof_pow2, t_prof_pow3;
wire [23:0] t_sweep_rate;
wire [2:0]  t_wave_sel;
wire [1:0]  t_freq_mode, t_sweep_mode, t_prof_sel;
wire        t_out_width, t_out_fmt, t_out_inv, t_sweep_ext_trig, t_prof_ext_sel;
wire        cfg_en, xfer_tgl, srst_tgl, start_tgl, abort_tgl;
wire        upd_ack_tgl, evt_done_tgl, evt_wrap_tgl;
wire        stat_sweep_active, stat_sweep_dir;
wire [1:0]  stat_active_prof;

// regs <-> wave RAM port A
wire        ram_en, ram_we;
wire [11:0] ram_addr;
wire [13:0] ram_wdata, ram_rdata;

// core <-> wave RAM port B
wire [11:0] wave_addr;
wire [13:0] wave_rdata;

dds_regs u_regs (
    .PCLK              (PCLK),
    .PRESETn           (PRESETn),
    .PADDR             (PADDR),
    .PSEL              (PSEL),
    .PENABLE           (PENABLE),
    .PWRITE            (PWRITE),
    .PWDATA            (PWDATA),
    .PRDATA            (PRDATA),
    .PREADY            (PREADY),

    .t_fword           (t_fword),
    .t_pow             (t_pow),
    .t_amp             (t_amp),
    .t_duty            (t_duty),
    .t_offset          (t_offset),
    .t_wave_sel        (t_wave_sel),
    .t_freq_mode       (t_freq_mode),
    .t_out_width       (t_out_width),
    .t_out_fmt         (t_out_fmt),
    .t_out_inv         (t_out_inv),
    .t_sweep_mode      (t_sweep_mode),
    .t_sweep_ext_trig  (t_sweep_ext_trig),
    .t_sweep_fstart    (t_sweep_fstart),
    .t_sweep_fstop     (t_sweep_fstop),
    .t_sweep_fdelta    (t_sweep_fdelta),
    .t_sweep_rate      (t_sweep_rate),
    .t_prof_sel        (t_prof_sel),
    .t_prof_ext_sel    (t_prof_ext_sel),
    .t_prof_fword0     (t_prof_fword0),
    .t_prof_fword1     (t_prof_fword1),
    .t_prof_fword2     (t_prof_fword2),
    .t_prof_fword3     (t_prof_fword3),
    .t_prof_pow0       (t_prof_pow0),
    .t_prof_pow1       (t_prof_pow1),
    .t_prof_pow2       (t_prof_pow2),
    .t_prof_pow3       (t_prof_pow3),

    .cfg_en            (cfg_en),
    .xfer_tgl          (xfer_tgl),
    .srst_tgl          (srst_tgl),
    .start_tgl         (start_tgl),
    .abort_tgl         (abort_tgl),

    .upd_ack_tgl       (upd_ack_tgl),
    .evt_done_tgl      (evt_done_tgl),
    .evt_wrap_tgl      (evt_wrap_tgl),
    .stat_sweep_active (stat_sweep_active),
    .stat_sweep_dir    (stat_sweep_dir),
    .stat_active_prof  (stat_active_prof),

    .ram_en            (ram_en),
    .ram_we            (ram_we),
    .ram_addr          (ram_addr),
    .ram_wdata         (ram_wdata),
    .ram_rdata         (ram_rdata),

    .dds_irq           (dds_irq)
);

dds_core #(
    .SIN_LUT_FILE      (SIN_LUT_FILE)
) u_core (
    .dds_clk           (dds_clk),
    .dds_rstn          (dds_rstn),

    .t_fword           (t_fword),
    .t_pow             (t_pow),
    .t_amp             (t_amp),
    .t_duty            (t_duty),
    .t_offset          (t_offset),
    .t_wave_sel        (t_wave_sel),
    .t_freq_mode       (t_freq_mode),
    .t_out_width       (t_out_width),
    .t_out_fmt         (t_out_fmt),
    .t_out_inv         (t_out_inv),
    .t_sweep_mode      (t_sweep_mode),
    .t_sweep_ext_trig  (t_sweep_ext_trig),
    .t_sweep_fstart    (t_sweep_fstart),
    .t_sweep_fstop     (t_sweep_fstop),
    .t_sweep_fdelta    (t_sweep_fdelta),
    .t_sweep_rate      (t_sweep_rate),
    .t_prof_sel        (t_prof_sel),
    .t_prof_ext_sel    (t_prof_ext_sel),
    .t_prof_fword0     (t_prof_fword0),
    .t_prof_fword1     (t_prof_fword1),
    .t_prof_fword2     (t_prof_fword2),
    .t_prof_fword3     (t_prof_fword3),
    .t_prof_pow0       (t_prof_pow0),
    .t_prof_pow1       (t_prof_pow1),
    .t_prof_pow2       (t_prof_pow2),
    .t_prof_pow3       (t_prof_pow3),

    .cfg_en            (cfg_en),
    .xfer_tgl          (xfer_tgl),
    .srst_tgl          (srst_tgl),
    .start_tgl         (start_tgl),
    .abort_tgl         (abort_tgl),
    .upd_ack_tgl       (upd_ack_tgl),
    .evt_done_tgl      (evt_done_tgl),
    .evt_wrap_tgl      (evt_wrap_tgl),
    .stat_sweep_active (stat_sweep_active),
    .stat_sweep_dir    (stat_sweep_dir),
    .stat_active_prof  (stat_active_prof),

    .sweep_trig        (sweep_trig),
    .hop_sel           (hop_sel),

    .wave_addr         (wave_addr),
    .wave_rdata        (wave_rdata),

    .dac_data          (dac_data),
    .dac_valid         (dac_valid)
);

dds_wave_ram u_wave_ram (
    .clka  (PCLK),
    .ena   (ram_en),
    .wea   (ram_we),
    .addra (ram_addr),
    .dina  (ram_wdata),
    .douta (ram_rdata),
    .clkb  (dds_clk),
    .enb   (1'b1),
    .addrb (wave_addr),
    .doutb (wave_rdata)
);

endmodule // dds_top
