// dds_channel.v - one complete DDS channel
//
// Register file + datapath + user waveform RAM, all in the dds_clk domain,
// hanging off the internal bus that dds_spi_slave drives. dds_top wires one
// of these per DAC channel.

module dds_channel #(
    parameter SIN_LUT_FILE = "dds_sin_lut.mem"
) (
    input  wire        clk,
    input  wire        rstn,

    // internal register bus (already decoded for this channel)
    input  wire [13:0] bus_addr,
    input  wire [15:0] bus_wdata,
    input  wire        bus_we,
    input  wire        bus_frame_end,
    output wire [15:0] bus_rdata,

    // hardware pacing pins (asynchronous)
    input  wire        sweep_trig,
    input  wire [1:0]  hop_sel,

    // DAC interface
    output wire [13:0] dac_data,
    output wire        dac_valid,

    output wire        dds_irq
);

wire [31:0] cfg_fword, cfg_sweep_fstart, cfg_sweep_fstop, cfg_sweep_fdelta;
wire [31:0] cfg_sweep_rate;
wire [31:0] cfg_prof_fword0, cfg_prof_fword1, cfg_prof_fword2, cfg_prof_fword3;
wire [15:0] cfg_pow, cfg_amp, cfg_duty;
wire [15:0] cfg_prof_pow0, cfg_prof_pow1, cfg_prof_pow2, cfg_prof_pow3;
wire [13:0] cfg_offset;
wire [2:0]  cfg_wave_sel;
wire [1:0]  cfg_freq_mode, cfg_sweep_mode, cfg_prof_sel;
wire        cfg_out_width, cfg_out_fmt, cfg_out_inv;
wire        cfg_sweep_ext_trig, cfg_prof_ext_sel;
wire        cfg_en, cfg_apply, srst_p, start_p, abort_p;
wire        upd_done_p, evt_done_p, evt_wrap_p;
wire        stat_sweep_active, stat_sweep_dir;
wire [1:0]  stat_active_prof;

wire        ram_we;
wire [11:0] ram_addr;
wire [13:0] ram_wdata, ram_rdata;
wire [11:0] wave_addr;
wire [13:0] wave_rdata;

dds_regs u_regs (
    .clk                (clk),
    .rstn               (rstn),

    .bus_addr           (bus_addr),
    .bus_wdata          (bus_wdata),
    .bus_we             (bus_we),
    .bus_frame_end      (bus_frame_end),
    .bus_rdata          (bus_rdata),

    .cfg_fword          (cfg_fword),
    .cfg_pow            (cfg_pow),
    .cfg_amp            (cfg_amp),
    .cfg_duty           (cfg_duty),
    .cfg_offset         (cfg_offset),
    .cfg_wave_sel       (cfg_wave_sel),
    .cfg_freq_mode      (cfg_freq_mode),
    .cfg_out_width      (cfg_out_width),
    .cfg_out_fmt        (cfg_out_fmt),
    .cfg_out_inv        (cfg_out_inv),
    .cfg_sweep_mode     (cfg_sweep_mode),
    .cfg_sweep_ext_trig (cfg_sweep_ext_trig),
    .cfg_sweep_fstart   (cfg_sweep_fstart),
    .cfg_sweep_fstop    (cfg_sweep_fstop),
    .cfg_sweep_fdelta   (cfg_sweep_fdelta),
    .cfg_sweep_rate     (cfg_sweep_rate),
    .cfg_prof_sel       (cfg_prof_sel),
    .cfg_prof_ext_sel   (cfg_prof_ext_sel),
    .cfg_prof_fword0    (cfg_prof_fword0),
    .cfg_prof_fword1    (cfg_prof_fword1),
    .cfg_prof_fword2    (cfg_prof_fword2),
    .cfg_prof_fword3    (cfg_prof_fword3),
    .cfg_prof_pow0      (cfg_prof_pow0),
    .cfg_prof_pow1      (cfg_prof_pow1),
    .cfg_prof_pow2      (cfg_prof_pow2),
    .cfg_prof_pow3      (cfg_prof_pow3),

    .cfg_en             (cfg_en),
    .cfg_apply          (cfg_apply),
    .srst_p             (srst_p),
    .start_p            (start_p),
    .abort_p            (abort_p),

    .upd_done_p         (upd_done_p),
    .evt_done_p         (evt_done_p),
    .evt_wrap_p         (evt_wrap_p),
    .stat_sweep_active  (stat_sweep_active),
    .stat_sweep_dir     (stat_sweep_dir),
    .stat_active_prof   (stat_active_prof),

    .ram_we             (ram_we),
    .ram_addr           (ram_addr),
    .ram_wdata          (ram_wdata),
    .ram_rdata          (ram_rdata),

    .dds_irq            (dds_irq)
);

dds_core #(
    .SIN_LUT_FILE       (SIN_LUT_FILE)
) u_core (
    .dds_clk            (clk),
    .dds_rstn           (rstn),

    .cfg_fword          (cfg_fword),
    .cfg_pow            (cfg_pow),
    .cfg_amp            (cfg_amp),
    .cfg_duty           (cfg_duty),
    .cfg_offset         (cfg_offset),
    .cfg_wave_sel       (cfg_wave_sel),
    .cfg_freq_mode      (cfg_freq_mode),
    .cfg_out_width      (cfg_out_width),
    .cfg_out_fmt        (cfg_out_fmt),
    .cfg_out_inv        (cfg_out_inv),
    .cfg_sweep_mode     (cfg_sweep_mode),
    .cfg_sweep_ext_trig (cfg_sweep_ext_trig),
    .cfg_sweep_fstart   (cfg_sweep_fstart),
    .cfg_sweep_fstop    (cfg_sweep_fstop),
    .cfg_sweep_fdelta   (cfg_sweep_fdelta),
    .cfg_sweep_rate     (cfg_sweep_rate),
    .cfg_prof_sel       (cfg_prof_sel),
    .cfg_prof_ext_sel   (cfg_prof_ext_sel),
    .cfg_prof_fword0    (cfg_prof_fword0),
    .cfg_prof_fword1    (cfg_prof_fword1),
    .cfg_prof_fword2    (cfg_prof_fword2),
    .cfg_prof_fword3    (cfg_prof_fword3),
    .cfg_prof_pow0      (cfg_prof_pow0),
    .cfg_prof_pow1      (cfg_prof_pow1),
    .cfg_prof_pow2      (cfg_prof_pow2),
    .cfg_prof_pow3      (cfg_prof_pow3),

    .cfg_en             (cfg_en),
    .cfg_apply          (cfg_apply),
    .srst_p             (srst_p),
    .start_p            (start_p),
    .abort_p            (abort_p),
    .upd_done_p         (upd_done_p),
    .evt_done_p         (evt_done_p),
    .evt_wrap_p         (evt_wrap_p),
    .stat_sweep_active  (stat_sweep_active),
    .stat_sweep_dir     (stat_sweep_dir),
    .stat_active_prof   (stat_active_prof),

    .sweep_trig         (sweep_trig),
    .hop_sel            (hop_sel),

    .wave_addr          (wave_addr),
    .wave_rdata         (wave_rdata),

    .dac_data           (dac_data),
    .dac_valid          (dac_valid)
);

dds_wave_ram u_wave_ram (
    .clk   (clk),
    .wea   (ram_we),
    .addra (ram_addr),
    .dina  (ram_wdata),
    .douta (ram_rdata),
    .enb   (1'b1),
    .addrb (wave_addr),
    .doutb (wave_rdata)
);

endmodule // dds_channel
