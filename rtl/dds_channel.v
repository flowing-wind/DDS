// dds_channel.v - one complete DDS channel
//
// Register file + datapath + user waveform RAM, all in the dds_clk domain,
// hanging off the internal bus that dds_spi_slave drives. dds_top wires one
// of these per DAC channel.
//
// With HAS_AGC = 1 the channel also carries its own amplitude loop: a
// measurement unit (dds_meas) eating one of the two ADC streams, a controller
// (dds_agc) driving the core's amplitude override, and a register block at
// word addresses 0x40..0x7F. Which ADC feeds the loop is a register field
// (AGC_CTRL.SRC), not a wire - any DDS channel can watch any ADC input, both
// channels can watch the same one, and a channel whose loop is off ignores the
// ADCs entirely. Each channel owning a full meas+agc pair (rather than sharing
// one) is what makes the two loops independent: different windows, different
// metrics, different targets, no arbitration.
//
// The channel's 14-bit word-address space:
//   0x0000..0x003F  block 0: dds_regs      (core registers)
//   0x0040..0x007F  block 1: dds_agc_regs  (amplitude loop), if HAS_AGC
//   0x2000..0x2FFF  user waveform RAM      (decoded inside dds_regs)

module dds_channel #(
    parameter SIN_LUT_FILE = "dds_sin_lut.mem",
    parameter HAS_AGC      = 1
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

    // captured ADC streams (dds_clk domain, from dds_adc_if via dds_top)
    input  wire signed [11:0] adc0_smp,
    input  wire               adc0_otr,
    input  wire signed [11:0] adc1_smp,
    input  wire               adc1_otr,
    input  wire               adc_smp_valid,

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

wire        amp_ovr_en;
wire [15:0] amp_ovr, amp_cur;

// ------------------------------------------------------------- block decode
// Block 0 (dds_regs) also owns the RAM half of the address space; block 1 is
// the loop. dds_regs only responds to writes with addr[12:6] == 0, so block-1
// writes need no extra gating on its side.
wire blk1_sel = HAS_AGC && !bus_addr[13] && (bus_addr[12:6] == 7'h01);

wire [15:0] rdata_regs, rdata_agc;

// Both blocks register their read data one cycle after the address, so the
// mux select needs the same delay.
reg blk1_sel_d;
always @(posedge clk or negedge rstn) begin
    if (!rstn) blk1_sel_d <= 1'b0;
    else       blk1_sel_d <= blk1_sel;
end

assign bus_rdata = blk1_sel_d ? rdata_agc : rdata_regs;

dds_regs u_regs (
    .clk                (clk),
    .rstn               (rstn),

    .bus_addr           (bus_addr),
    .bus_wdata          (bus_wdata),
    .bus_we             (bus_we),
    .bus_frame_end      (bus_frame_end),
    .bus_rdata          (rdata_regs),

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

    .amp_ovr_en         (amp_ovr_en),
    .amp_ovr            (amp_ovr),
    .amp_cur            (amp_cur),

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

// ------------------------------------------------------- the amplitude loop
generate
if (HAS_AGC) begin : g_agc

    wire        agc_en, agc_hold, agc_metric, agc_clr;
    wire [1:0]  agc_src;
    wire [11:0] agc_target, agc_tol;
    wire [4:0]  agc_win;
    wire [15:0] agc_ki, agc_amin, agc_amax;
    wire        agc_locked, agc_sat_hi, agc_sat_lo;

    // measurement source select; SRC = 1x parks the loop on "no input"
    wire signed [11:0] meas_smp   = agc_src[0] ? adc1_smp : adc0_smp;
    wire               meas_otr   = agc_src[0] ? adc1_otr : adc0_otr;
    wire               meas_vld   = adc_smp_valid && !agc_src[1];

    wire [11:0] m_vpp, m_rms;
    wire signed [11:0] m_mean, m_vmin, m_vmax;
    wire        m_otr, m_valid;

    dds_meas u_meas (
        .clk          (clk),
        .rstn         (rstn),
        .cfg_win_log2 (agc_win),
        .clr          (agc_clr),
        .smp          (meas_smp),
        .smp_valid    (meas_vld),
        .otr          (meas_otr),
        .vpp          (m_vpp),
        .rms          (m_rms),
        .mean         (m_mean),
        .vmin         (m_vmin),
        .vmax         (m_vmax),
        .otr_win      (m_otr),
        .result_valid (m_valid)
    );

    dds_agc u_agc (
        .clk         (clk),
        .rstn        (rstn),
        .cfg_en      (agc_en),
        .cfg_hold    (agc_hold),
        .clr         (agc_clr),
        .cfg_target  (agc_target),
        .cfg_ki      (agc_ki),
        .cfg_tol     (agc_tol),
        .cfg_amp_min (agc_amin),
        .cfg_amp_max (agc_amax),
        .init_amp    (amp_cur),
        .meas        (agc_metric ? m_rms : m_vpp),
        .meas_valid  (m_valid),
        .amp         (amp_ovr),
        .amp_ovr_en  (amp_ovr_en),
        .locked      (agc_locked),
        .sat_hi      (agc_sat_hi),
        .sat_lo      (agc_sat_lo)
    );

    dds_agc_regs u_agc_regs (
        .clk           (clk),
        .rstn          (rstn),
        .bus_idx       (bus_addr[5:0]),
        .bus_wdata     (bus_wdata),
        .bus_we        (bus_we && blk1_sel),
        .bus_frame_end (bus_frame_end),
        .bus_rdata     (rdata_agc),

        .cfg_en        (agc_en),
        .cfg_hold      (agc_hold),
        .cfg_src       (agc_src),
        .cfg_metric    (agc_metric),
        .cfg_target    (agc_target),
        .cfg_ki        (agc_ki),
        .cfg_tol       (agc_tol),
        .cfg_win_log2  (agc_win),
        .cfg_amp_min   (agc_amin),
        .cfg_amp_max   (agc_amax),
        .clr_p         (agc_clr),

        .st_amp        (amp_cur),
        .st_locked     (agc_locked),
        .st_sat_hi     (agc_sat_hi),
        .st_sat_lo     (agc_sat_lo),
        .st_vpp        (m_vpp),
        .st_rms        (m_rms),
        .st_mean       (m_mean),
        .st_vmin       (m_vmin),
        .st_vmax       (m_vmax),
        .st_otr        (m_otr)
    );

end else begin : g_no_agc
    assign amp_ovr_en = 1'b0;
    assign amp_ovr    = 16'h8000;
    assign rdata_agc  = 16'h0;
end
endgenerate

endmodule // dds_channel
