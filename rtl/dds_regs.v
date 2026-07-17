// dds_regs.v - 16-bit register file for one DDS channel
//
// Sits on the internal bus produced by dds_spi_slave, in the dds_clk domain
// (the SPI pins are oversampled, so the whole IP is single-clock).
//
// Word address map (see Doc/DDS_Datasheet.md for the bit fields):
//   0x00 ID (RO)      0x08 DUTY          0x10 SWEEP_FDELTA_L  0x18 PROF2_FWORD_L
//   0x01 VERSION (RO) 0x09 OFFSET        0x11 SWEEP_FDELTA_H  0x19 PROF2_FWORD_H
//   0x02 CTRL         0x0A UPDATE        0x12 SWEEP_RATE_L    0x1A PROF3_FWORD_L
//   0x03 STATUS (RO)  0x0B SWEEP_CTRL    0x13 SWEEP_RATE_H    0x1B PROF3_FWORD_H
//   0x04 FWORD_L      0x0C SWEEP_FSTART_L 0x14 PROF_CTRL      0x1C PROF0_POW
//   0x05 FWORD_H      0x0D SWEEP_FSTART_H 0x15 PROF0_FWORD_L  0x1D PROF1_POW
//   0x06 POW          0x0E SWEEP_FSTOP_L  0x16 PROF0_FWORD_H  0x1E PROF2_POW
//   0x07 AMP          0x0F SWEEP_FSTOP_H  0x17 PROF1_FWORD_L  0x1F PROF3_POW
//   0x20 IRQ_EN       0x21 IRQ_STAT
//   0x2000..0x2FFF    user waveform RAM (14-bit samples)
//
// Everything a frame writes takes effect together: config writes land in
// shadow registers, and the CS rising edge (bus_frame_end) commits them to
// the datapath with a single cfg_apply pulse. A 32-bit frequency word written
// as two 16-bit halves therefore never produces an intermediate frequency.
// UPDATE.AUTO = 1 (reset default) commits every frame that wrote config;
// with AUTO = 0 the shadows accumulate across frames until a frame writes
// UPDATE.UPD = 1. Sweep start/abort and soft reset are likewise deferred to
// the commit point, so "write sweep config + START" in one frame always
// starts the sweep with the config from that same frame.

module dds_regs (
    input  wire        clk,
    input  wire        rstn,

    // internal bus (from dds_spi_slave, already channel-decoded)
    input  wire [13:0] bus_addr,
    input  wire [15:0] bus_wdata,
    input  wire        bus_we,
    input  wire        bus_frame_end,
    output reg  [15:0] bus_rdata,

    // configuration -> dds_core (quasi-static, sampled on cfg_apply)
    output wire [31:0] cfg_fword,
    output wire [15:0] cfg_pow,
    output wire [15:0] cfg_amp,          // Q1.15, 0x8000 = 1.0
    output wire [15:0] cfg_duty,
    output wire [13:0] cfg_offset,       // signed
    output wire [2:0]  cfg_wave_sel,
    output wire [1:0]  cfg_freq_mode,
    output wire        cfg_out_width,    // 1 = 12-bit MSB-aligned
    output wire        cfg_out_fmt,      // 1 = two's complement, 0 = offset binary
    output wire        cfg_out_inv,
    output wire [1:0]  cfg_sweep_mode,
    output wire        cfg_sweep_ext_trig,
    output wire [31:0] cfg_sweep_fstart,
    output wire [31:0] cfg_sweep_fstop,
    output wire [31:0] cfg_sweep_fdelta,
    output wire [31:0] cfg_sweep_rate,
    output wire [1:0]  cfg_prof_sel,
    output wire        cfg_prof_ext_sel,
    output wire [31:0] cfg_prof_fword0,
    output wire [31:0] cfg_prof_fword1,
    output wire [31:0] cfg_prof_fword2,
    output wire [31:0] cfg_prof_fword3,
    output wire [15:0] cfg_prof_pow0,
    output wire [15:0] cfg_prof_pow1,
    output wire [15:0] cfg_prof_pow2,
    output wire [15:0] cfg_prof_pow3,

    output wire        cfg_en,           // CTRL.EN, immediate level
    output reg         cfg_apply,        // 1-cycle: load the config into the core
    output reg         srst_p,           // 1-cycle: soft reset
    output reg         start_p,          // 1-cycle: sweep start
    output reg         abort_p,          // 1-cycle: sweep abort

    // from dds_core (same clock domain)
    input  wire        upd_done_p,
    input  wire        evt_done_p,
    input  wire        evt_wrap_p,
    input  wire        stat_sweep_active,
    input  wire        stat_sweep_dir,
    input  wire [1:0]  stat_active_prof,

    // waveform RAM port A
    output wire        ram_we,
    output wire [11:0] ram_addr,
    output wire [13:0] ram_wdata,
    input  wire [13:0] ram_rdata,

    output wire        dds_irq
);

localparam [15:0] ID_CODE = 16'h4453;   // "DS"
localparam [15:0] VERSION = 16'h0210;   // v2.1 - SPI + ADC feedback / AGC

localparam A_ID        = 6'h00, A_VER       = 6'h01, A_CTRL      = 6'h02,
           A_STATUS    = 6'h03, A_FWORD_L   = 6'h04, A_FWORD_H   = 6'h05,
           A_POW       = 6'h06, A_AMP       = 6'h07, A_DUTY      = 6'h08,
           A_OFFSET    = 6'h09, A_UPDATE    = 6'h0A, A_SWCTRL    = 6'h0B,
           A_FSTART_L  = 6'h0C, A_FSTART_H  = 6'h0D, A_FSTOP_L   = 6'h0E,
           A_FSTOP_H   = 6'h0F, A_FDELTA_L  = 6'h10, A_FDELTA_H  = 6'h11,
           A_RATE_L    = 6'h12, A_RATE_H    = 6'h13, A_PROF_CTRL = 6'h14,
           A_P0F_L     = 6'h15, A_P0F_H     = 6'h16, A_P1F_L     = 6'h17,
           A_P1F_H     = 6'h18, A_P2F_L     = 6'h19, A_P2F_H     = 6'h1A,
           A_P3F_L     = 6'h1B, A_P3F_H     = 6'h1C, A_P0POW     = 6'h1D,
           A_P1POW     = 6'h1E, A_P2POW     = 6'h1F, A_P3POW     = 6'h20,
           A_IRQ_EN    = 6'h21, A_IRQ_STAT  = 6'h22;

wire       ram_sel = bus_addr[13];
wire [5:0] widx    = bus_addr[5:0];
wire       reg_we  = bus_we && !ram_sel;
wire       reg_hit = reg_we && (bus_addr[12:6] == 7'h0);   // no aliasing

// ---------------------------------------------------------- shadow registers
reg        s_en;
reg [2:0]  s_wave;
reg [1:0]  s_fmode;
reg        s_w12, s_fmt2c, s_inv;
reg [31:0] s_fword;
reg [15:0] s_pow, s_amp, s_duty;
reg [13:0] s_offset;
reg        s_auto;
reg [1:0]  s_swmode;
reg        s_sw_ext;
reg [31:0] s_fstart, s_fstop, s_fdelta, s_rate;
reg [1:0]  s_psel;
reg        s_pext;
reg [31:0] s_pfw  [0:3];
reg [15:0] s_ppow [0:3];
reg [2:0]  s_irq_en, s_irq_stat;

assign cfg_en             = s_en;
assign cfg_fword          = s_fword;
assign cfg_pow            = s_pow;
assign cfg_amp            = s_amp;
assign cfg_duty           = s_duty;
assign cfg_offset         = s_offset;
assign cfg_wave_sel       = s_wave;
assign cfg_freq_mode      = s_fmode;
assign cfg_out_width      = s_w12;
assign cfg_out_fmt        = s_fmt2c;
assign cfg_out_inv        = s_inv;
assign cfg_sweep_mode     = s_swmode;
assign cfg_sweep_ext_trig = s_sw_ext;
assign cfg_sweep_fstart   = s_fstart;
assign cfg_sweep_fstop    = s_fstop;
assign cfg_sweep_fdelta   = s_fdelta;
assign cfg_sweep_rate     = s_rate;
assign cfg_prof_sel       = s_psel;
assign cfg_prof_ext_sel   = s_pext;
assign cfg_prof_fword0    = s_pfw[0];
assign cfg_prof_fword1    = s_pfw[1];
assign cfg_prof_fword2    = s_pfw[2];
assign cfg_prof_fword3    = s_pfw[3];
assign cfg_prof_pow0      = s_ppow[0];
assign cfg_prof_pow1      = s_ppow[1];
assign cfg_prof_pow2      = s_ppow[2];
assign cfg_prof_pow3      = s_ppow[3];

// a write to any register whose content the core latches on cfg_apply
wire cfg_write = reg_hit &&
                 ((widx == A_CTRL) || (widx == A_FWORD_L) || (widx == A_FWORD_H) ||
                  (widx == A_POW)  || (widx == A_AMP)     || (widx == A_DUTY)    ||
                  (widx == A_OFFSET) ||
                  ((widx >= A_SWCTRL) && (widx <= A_P3POW)));

wire upd_write   = reg_hit && (widx == A_UPDATE) && bus_wdata[0];
wire srst_write  = reg_hit && (widx == A_CTRL)   && bus_wdata[1];
wire start_write = reg_hit && (widx == A_SWCTRL) && bus_wdata[8];
wire abort_write = reg_hit && (widx == A_SWCTRL) && bus_wdata[9];

integer i;
always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        s_en     <= 1'b0;      s_wave   <= 3'd0;      s_fmode  <= 2'd0;
        s_w12    <= 1'b0;      s_fmt2c  <= 1'b0;      s_inv    <= 1'b0;
        s_fword  <= 32'h0;     s_pow    <= 16'h0;
        s_amp    <= 16'h8000;  s_duty   <= 16'h8000;  s_offset <= 14'h0;
        s_auto   <= 1'b1;
        s_swmode <= 2'd0;      s_sw_ext <= 1'b0;
        s_fstart <= 32'h0;     s_fstop  <= 32'h0;
        s_fdelta <= 32'h0;     s_rate   <= 32'd1;
        s_psel   <= 2'd0;      s_pext   <= 1'b0;
        s_irq_en <= 3'b000;
        for (i = 0; i < 4; i = i + 1) begin
            s_pfw[i]  <= 32'h0;
            s_ppow[i] <= 16'h0;
        end
    end else if (reg_hit) begin
        case (widx)
            A_CTRL: begin
                s_en    <= bus_wdata[0];
                s_wave  <= bus_wdata[6:4];
                s_fmode <= bus_wdata[9:8];
                s_w12   <= bus_wdata[12];
                s_fmt2c <= bus_wdata[13];
                s_inv   <= bus_wdata[14];
            end
            A_FWORD_L:  s_fword[15:0]  <= bus_wdata;
            A_FWORD_H:  s_fword[31:16] <= bus_wdata;
            A_POW:      s_pow    <= bus_wdata;
            // gain is clamped to 1.0; anything above full scale would only clip
            A_AMP:      s_amp    <= (bus_wdata > 16'h8000) ? 16'h8000 : bus_wdata;
            A_DUTY:     s_duty   <= bus_wdata;
            A_OFFSET:   s_offset <= bus_wdata[13:0];
            A_UPDATE:   s_auto   <= bus_wdata[1];
            A_SWCTRL: begin
                s_swmode <= bus_wdata[1:0];
                s_sw_ext <= bus_wdata[2];
            end
            A_FSTART_L: s_fstart[15:0]  <= bus_wdata;
            A_FSTART_H: s_fstart[31:16] <= bus_wdata;
            A_FSTOP_L:  s_fstop[15:0]   <= bus_wdata;
            A_FSTOP_H:  s_fstop[31:16]  <= bus_wdata;
            A_FDELTA_L: s_fdelta[15:0]  <= bus_wdata;
            A_FDELTA_H: s_fdelta[31:16] <= bus_wdata;
            A_RATE_L:   s_rate[15:0]    <= bus_wdata;
            A_RATE_H:   s_rate[31:16]   <= bus_wdata;
            A_PROF_CTRL: begin
                s_psel <= bus_wdata[1:0];
                s_pext <= bus_wdata[4];
            end
            A_P0F_L: s_pfw[0][15:0]  <= bus_wdata;
            A_P0F_H: s_pfw[0][31:16] <= bus_wdata;
            A_P1F_L: s_pfw[1][15:0]  <= bus_wdata;
            A_P1F_H: s_pfw[1][31:16] <= bus_wdata;
            A_P2F_L: s_pfw[2][15:0]  <= bus_wdata;
            A_P2F_H: s_pfw[2][31:16] <= bus_wdata;
            A_P3F_L: s_pfw[3][15:0]  <= bus_wdata;
            A_P3F_H: s_pfw[3][31:16] <= bus_wdata;
            A_P0POW: s_ppow[0] <= bus_wdata;
            A_P1POW: s_ppow[1] <= bus_wdata;
            A_P2POW: s_ppow[2] <= bus_wdata;
            A_P3POW: s_ppow[3] <= bus_wdata;
            A_IRQ_EN: s_irq_en <= bus_wdata[2:0];
            default: ;
        endcase
    end
end

// ------------------------------------------------- deferred commit at CS rise
// Everything the frame asked for fires together on bus_frame_end: the config
// commit first (cfg_apply), and the sweep events in the same cycle - the core
// latches the new config on that edge and the sweep engine starts from it.
reg pend_cfg, pend_srst, pend_start, pend_abort;

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        pend_cfg   <= 1'b0;  pend_srst  <= 1'b0;
        pend_start <= 1'b0;  pend_abort <= 1'b0;
        cfg_apply  <= 1'b0;  srst_p     <= 1'b0;
        start_p    <= 1'b0;  abort_p    <= 1'b0;
    end else begin
        cfg_apply <= 1'b0;  srst_p  <= 1'b0;
        start_p   <= 1'b0;  abort_p <= 1'b0;

        if (upd_write || (s_auto && cfg_write)) pend_cfg   <= 1'b1;
        if (srst_write)                         pend_srst  <= 1'b1;
        if (start_write)                        pend_start <= 1'b1;
        if (abort_write)                        pend_abort <= 1'b1;

        if (bus_frame_end) begin
            cfg_apply  <= pend_cfg;    pend_cfg   <= 1'b0;
            srst_p     <= pend_srst;   pend_srst  <= 1'b0;
            start_p    <= pend_start;  pend_start <= 1'b0;
            abort_p    <= pend_abort;  pend_abort <= 1'b0;
        end
    end
end

// ------------------------------------------------------------ interrupt flags
// bit0 sweep done, bit1 sweep wrap, bit2 config update done. W1C.
wire irq_stat_write = reg_hit && (widx == A_IRQ_STAT);

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        s_irq_stat <= 3'b000;
    end else if (srst_p) begin
        s_irq_stat <= 3'b000;
    end else begin
        // a new event wins over a concurrent write-1-to-clear
        s_irq_stat[0] <= evt_done_p ? 1'b1 :
                         (irq_stat_write && bus_wdata[0]) ? 1'b0 : s_irq_stat[0];
        s_irq_stat[1] <= evt_wrap_p ? 1'b1 :
                         (irq_stat_write && bus_wdata[1]) ? 1'b0 : s_irq_stat[1];
        s_irq_stat[2] <= upd_done_p ? 1'b1 :
                         (irq_stat_write && bus_wdata[2]) ? 1'b0 : s_irq_stat[2];
    end
end

assign dds_irq = |(s_irq_stat & s_irq_en);

// ---------------------------------------------------------- waveform RAM port
assign ram_we    = bus_we && ram_sel;
assign ram_addr  = bus_addr[11:0];
assign ram_wdata = bus_wdata[13:0];

// --------------------------------------------------------------- read channel
// One pipeline stage, matching the RAM read latency, so the SPI slave sees a
// uniform 2-cycle bus read.
reg [15:0] reg_rdata;
reg        ram_sel_d;

always @(*) begin
    case (widx)
        A_ID:        reg_rdata = ID_CODE;
        A_VER:       reg_rdata = VERSION;
        A_CTRL:      reg_rdata = {1'b0, s_inv, s_fmt2c, s_w12, 2'b00,
                                  s_fmode, 1'b0, s_wave, 3'b000, s_en};
        A_STATUS:    reg_rdata = {12'h0, stat_active_prof, stat_sweep_dir,
                                  stat_sweep_active};
        A_FWORD_L:   reg_rdata = s_fword[15:0];
        A_FWORD_H:   reg_rdata = s_fword[31:16];
        A_POW:       reg_rdata = s_pow;
        A_AMP:       reg_rdata = s_amp;
        A_DUTY:      reg_rdata = s_duty;
        A_OFFSET:    reg_rdata = {2'b00, s_offset};
        A_UPDATE:    reg_rdata = {14'h0, s_auto, 1'b0};
        A_SWCTRL:    reg_rdata = {13'h0, s_sw_ext, s_swmode};
        A_FSTART_L:  reg_rdata = s_fstart[15:0];
        A_FSTART_H:  reg_rdata = s_fstart[31:16];
        A_FSTOP_L:   reg_rdata = s_fstop[15:0];
        A_FSTOP_H:   reg_rdata = s_fstop[31:16];
        A_FDELTA_L:  reg_rdata = s_fdelta[15:0];
        A_FDELTA_H:  reg_rdata = s_fdelta[31:16];
        A_RATE_L:    reg_rdata = s_rate[15:0];
        A_RATE_H:    reg_rdata = s_rate[31:16];
        A_PROF_CTRL: reg_rdata = {11'h0, s_pext, 2'b00, s_psel};
        A_P0F_L:     reg_rdata = s_pfw[0][15:0];
        A_P0F_H:     reg_rdata = s_pfw[0][31:16];
        A_P1F_L:     reg_rdata = s_pfw[1][15:0];
        A_P1F_H:     reg_rdata = s_pfw[1][31:16];
        A_P2F_L:     reg_rdata = s_pfw[2][15:0];
        A_P2F_H:     reg_rdata = s_pfw[2][31:16];
        A_P3F_L:     reg_rdata = s_pfw[3][15:0];
        A_P3F_H:     reg_rdata = s_pfw[3][31:16];
        A_P0POW:     reg_rdata = s_ppow[0];
        A_P1POW:     reg_rdata = s_ppow[1];
        A_P2POW:     reg_rdata = s_ppow[2];
        A_P3POW:     reg_rdata = s_ppow[3];
        A_IRQ_EN:    reg_rdata = {13'h0, s_irq_en};
        A_IRQ_STAT:  reg_rdata = {13'h0, s_irq_stat};
        default:     reg_rdata = 16'h0;
    endcase
end

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        bus_rdata <= 16'h0;
        ram_sel_d <= 1'b0;
    end else begin
        ram_sel_d <= ram_sel;
        bus_rdata <= ram_sel_d ? {2'b00, ram_rdata} : reg_rdata;
    end
end

endmodule // dds_regs
