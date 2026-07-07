// dds_regs.v - APB register file for the DDS core (PCLK domain)
//
// APB style aligned with spi_regs.v (PSEL/PENABLE/PWRITE, PREADY tied high),
// widened to 32-bit data. PADDR is a byte address; registers decode on
// PADDR[7:2], the waveform RAM occupies the 16 KB window selected by
// PADDR[14] (0x4000..0x7FFF).
//
// All datapath parameters are double-buffered:
//   shadow regs (APB writes)  --capture-->  t_* transfer regs  --xfer_tgl-->
//   active regs in dds_core (dds_clk domain), loaded atomically.
// With UPDATE.AUTO=1 (reset default) every config write triggers a capture;
// with AUTO=0 writes accumulate in the shadows until UPDATE.UPD is written.

module dds_regs (
    // APB
    input  wire        PCLK,
    input  wire        PRESETn,

    input  wire [14:0] PADDR,
    input  wire        PSEL,
    input  wire        PENABLE,
    input  wire        PWRITE,
    input  wire [31:0] PWDATA,
    output reg  [31:0] PRDATA,
    output wire        PREADY,

    // captured configuration -> dds_core (quasi-static across xfer_tgl)
    output reg  [31:0] t_fword,
    output reg  [15:0] t_pow,
    output reg  [16:0] t_amp,
    output reg  [15:0] t_duty,
    output reg  [13:0] t_offset,
    output reg  [2:0]  t_wave_sel,
    output reg  [1:0]  t_freq_mode,
    output reg         t_out_width,
    output reg         t_out_fmt,
    output reg         t_out_inv,
    output reg  [1:0]  t_sweep_mode,
    output reg         t_sweep_ext_trig,
    output reg  [31:0] t_sweep_fstart,
    output reg  [31:0] t_sweep_fstop,
    output reg  [31:0] t_sweep_fdelta,
    output reg  [23:0] t_sweep_rate,
    output reg  [1:0]  t_prof_sel,
    output reg         t_prof_ext_sel,
    output reg  [31:0] t_prof_fword0,
    output reg  [31:0] t_prof_fword1,
    output reg  [31:0] t_prof_fword2,
    output reg  [31:0] t_prof_fword3,
    output reg  [15:0] t_prof_pow0,
    output reg  [15:0] t_prof_pow1,
    output reg  [15:0] t_prof_pow2,
    output reg  [15:0] t_prof_pow3,

    output wire        cfg_en,       // CTRL.EN, immediate level
    output reg         xfer_tgl,     // flips when t_* set is valid
    output reg         srst_tgl,     // soft reset event
    output reg         start_tgl,    // sweep start event
    output reg         abort_tgl,    // sweep abort event

    // from dds_core (dds_clk domain)
    input  wire        upd_ack_tgl,      // flips when core loaded the t_* set
    input  wire        evt_done_tgl,     // flips on single-sweep completion
    input  wire        evt_wrap_tgl,     // flips on sweep wrap / reversal
    input  wire        stat_sweep_active,
    input  wire        stat_sweep_dir,
    input  wire [1:0]  stat_active_prof,

    // waveform RAM port A
    output wire        ram_en,
    output wire        ram_we,
    output wire [11:0] ram_addr,
    output wire [13:0] ram_wdata,
    input  wire [13:0] ram_rdata,

    // Interrupt Request
    output wire        dds_irq
);

localparam VERSION = 32'h4453_0110;   // "DS" + v1.1.0

// word index of each register (byte offset / 4)
localparam A_ID           = 6'd0,   // 0x00
           A_CTRL         = 6'd1,   // 0x04
           A_STATUS       = 6'd2,   // 0x08
           A_FWORD        = 6'd3,   // 0x0C
           A_POW          = 6'd4,   // 0x10
           A_AMP          = 6'd5,   // 0x14
           A_DUTY         = 6'd6,   // 0x18
           A_UPDATE       = 6'd7,   // 0x1C
           A_SWEEP_CTRL   = 6'd8,   // 0x20
           A_SWEEP_FSTART = 6'd9,   // 0x24
           A_SWEEP_FSTOP  = 6'd10,  // 0x28
           A_SWEEP_FDELTA = 6'd11,  // 0x2C
           A_SWEEP_RATE   = 6'd12,  // 0x30
           A_PROF_CTRL    = 6'd13,  // 0x34
           A_PROF0_FWORD  = 6'd14,  // 0x38
           A_PROF1_FWORD  = 6'd15,  // 0x3C
           A_PROF2_FWORD  = 6'd16,  // 0x40
           A_PROF3_FWORD  = 6'd17,  // 0x44
           A_PROF_POW10   = 6'd18,  // 0x48
           A_PROF_POW32   = 6'd19,  // 0x4C
           A_IRQ_EN       = 6'd20,  // 0x50
           A_IRQ_STAT     = 6'd21,  // 0x54
           A_OFFSET       = 6'd22;  // 0x58

// APB Signals
wire        APB_access   = PSEL && PENABLE;
wire        APB_write_en = APB_access && PWRITE;
wire        APB_read_en  = APB_access && !PWRITE;
wire        ram_sel      = PADDR[14];
wire [5:0]  widx         = PADDR[7:2];
assign      PREADY       = 1'b1;

// Shadow Regs
reg         reg_en;
reg  [2:0]  reg_wave_sel;
reg  [1:0]  reg_freq_mode;
reg         reg_out_width, reg_out_fmt, reg_out_inv;
reg  [31:0] reg_fword;
reg  [15:0] reg_pow;
reg  [16:0] reg_amp;
reg  [15:0] reg_duty;
reg  [13:0] reg_offset;
reg         reg_auto;
reg  [1:0]  reg_sweep_mode;
reg         reg_sweep_ext_trig;
reg  [31:0] reg_sweep_fstart, reg_sweep_fstop, reg_sweep_fdelta;
reg  [23:0] reg_sweep_rate;
reg  [1:0]  reg_prof_sel;
reg         reg_prof_ext_sel;
reg  [31:0] reg_prof_fword [0:3];
reg  [15:0] reg_prof_pow   [0:3];
reg  [2:0]  reg_irq_en;
reg  [2:0]  reg_irq_stat;

assign cfg_en = reg_en;

// a write to any register whose content travels through the t_* capture
wire cfg_write = APB_write_en && !ram_sel &&
                 ((widx == A_CTRL)  || (widx == A_FWORD) || (widx == A_POW)  ||
                  (widx == A_AMP)   || (widx == A_DUTY)  || (widx == A_OFFSET) ||
                  ((widx >= A_SWEEP_CTRL) && (widx <= A_PROF_POW32)));

wire upd_write   = APB_write_en && !ram_sel && (widx == A_UPDATE) && PWDATA[0];
wire srst_write  = APB_write_en && !ram_sel && (widx == A_CTRL)       && PWDATA[1];
wire start_write = APB_write_en && !ram_sel && (widx == A_SWEEP_CTRL) && PWDATA[8];
wire abort_write = APB_write_en && !ram_sel && (widx == A_SWEEP_CTRL) && PWDATA[9];

// APB Write Data (shadow registers)
always @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn) begin
        reg_en             <= 1'b0;
        reg_wave_sel       <= 3'd0;
        reg_freq_mode      <= 2'd0;
        reg_out_width      <= 1'b0;
        reg_out_fmt        <= 1'b0;
        reg_out_inv        <= 1'b0;
        reg_fword          <= 32'h0;
        reg_pow            <= 16'h0;
        reg_amp            <= 17'h10000;    // 1.0
        reg_duty           <= 16'h8000;     // 50%
        reg_offset         <= 14'h0;
        reg_auto           <= 1'b1;
        reg_sweep_mode     <= 2'd0;
        reg_sweep_ext_trig <= 1'b0;
        reg_sweep_fstart   <= 32'h0;
        reg_sweep_fstop    <= 32'h0;
        reg_sweep_fdelta   <= 32'h0;
        reg_sweep_rate     <= 24'd1;
        reg_prof_sel       <= 2'd0;
        reg_prof_ext_sel   <= 1'b0;
        reg_prof_fword[0]  <= 32'h0;
        reg_prof_fword[1]  <= 32'h0;
        reg_prof_fword[2]  <= 32'h0;
        reg_prof_fword[3]  <= 32'h0;
        reg_prof_pow[0]    <= 16'h0;
        reg_prof_pow[1]    <= 16'h0;
        reg_prof_pow[2]    <= 16'h0;
        reg_prof_pow[3]    <= 16'h0;
        reg_irq_en         <= 3'b000;
    end else begin
        if (APB_write_en && !ram_sel) begin
            case (widx)
                A_CTRL: begin
                    reg_en        <= PWDATA[0];
                    reg_wave_sel  <= PWDATA[6:4];
                    reg_freq_mode <= PWDATA[9:8];
                    reg_out_width <= PWDATA[12];
                    reg_out_fmt   <= PWDATA[13];
                    reg_out_inv   <= PWDATA[14];
                end
                A_FWORD:        reg_fword <= PWDATA;
                A_POW:          reg_pow   <= PWDATA[15:0];
                A_AMP:          reg_amp   <= PWDATA[16] ? 17'h10000 : PWDATA[16:0];
                A_DUTY:         reg_duty  <= PWDATA[15:0];
                A_OFFSET:       reg_offset <= PWDATA[13:0];
                A_UPDATE:       reg_auto  <= PWDATA[1];
                A_SWEEP_CTRL: begin
                    reg_sweep_mode     <= PWDATA[1:0];
                    reg_sweep_ext_trig <= PWDATA[2];
                end
                A_SWEEP_FSTART: reg_sweep_fstart <= PWDATA;
                A_SWEEP_FSTOP:  reg_sweep_fstop  <= PWDATA;
                A_SWEEP_FDELTA: reg_sweep_fdelta <= PWDATA;
                A_SWEEP_RATE:   reg_sweep_rate   <= PWDATA[23:0];
                A_PROF_CTRL: begin
                    reg_prof_sel     <= PWDATA[1:0];
                    reg_prof_ext_sel <= PWDATA[4];
                end
                A_PROF0_FWORD:  reg_prof_fword[0] <= PWDATA;
                A_PROF1_FWORD:  reg_prof_fword[1] <= PWDATA;
                A_PROF2_FWORD:  reg_prof_fword[2] <= PWDATA;
                A_PROF3_FWORD:  reg_prof_fword[3] <= PWDATA;
                A_PROF_POW10: begin
                    reg_prof_pow[0] <= PWDATA[15:0];
                    reg_prof_pow[1] <= PWDATA[31:16];
                end
                A_PROF_POW32: begin
                    reg_prof_pow[2] <= PWDATA[15:0];
                    reg_prof_pow[3] <= PWDATA[31:16];
                end
                A_IRQ_EN:       reg_irq_en <= PWDATA[2:0];
                default: ;
            endcase
        end
    end
end

// Update / capture handshake
//   upd_req -> capture one cycle later (so a config write in the same cycle
//   is included) -> xfer_tgl flips -> core loads and flips upd_ack_tgl back.
//   A request while busy is remembered (rerun) and served after the ack.
wire upd_req = upd_write || (reg_auto && cfg_write);
wire ack_p;
reg  busy, rerun, cap_now;

dds_cdc_pulse u_sync_ack (.clk(PCLK), .rstn(PRESETn), .tgl_in(upd_ack_tgl), .pulse(ack_p));

always @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn) begin
        busy    <= 1'b0;
        rerun   <= 1'b0;
        cap_now <= 1'b0;
        xfer_tgl <= 1'b0;
        t_fword <= 32'h0;         t_pow  <= 16'h0;
        t_amp   <= 17'h10000;     t_duty <= 16'h8000;
        t_offset <= 14'h0;
        t_wave_sel <= 3'd0;       t_freq_mode <= 2'd0;
        t_out_width <= 1'b0;      t_out_fmt <= 1'b0;      t_out_inv <= 1'b0;
        t_sweep_mode <= 2'd0;     t_sweep_ext_trig <= 1'b0;
        t_sweep_fstart <= 32'h0;  t_sweep_fstop <= 32'h0;
        t_sweep_fdelta <= 32'h0;  t_sweep_rate <= 24'd1;
        t_prof_sel <= 2'd0;       t_prof_ext_sel <= 1'b0;
        t_prof_fword0 <= 32'h0;   t_prof_fword1 <= 32'h0;
        t_prof_fword2 <= 32'h0;   t_prof_fword3 <= 32'h0;
        t_prof_pow0 <= 16'h0;     t_prof_pow1 <= 16'h0;
        t_prof_pow2 <= 16'h0;     t_prof_pow3 <= 16'h0;
    end else begin
        cap_now <= 1'b0;

        if (upd_req) begin
            if (!busy) begin
                busy    <= 1'b1;
                cap_now <= 1'b1;
            end else begin
                rerun <= 1'b1;
            end
        end

        if (ack_p) begin
            if (rerun) begin
                rerun   <= 1'b0;
                cap_now <= 1'b1;
            end else begin
                busy <= 1'b0;
            end
        end

        if (cap_now) begin
            t_fword          <= reg_fword;
            t_pow            <= reg_pow;
            t_amp            <= reg_amp;
            t_duty           <= reg_duty;
            t_offset         <= reg_offset;
            t_wave_sel       <= reg_wave_sel;
            t_freq_mode      <= reg_freq_mode;
            t_out_width      <= reg_out_width;
            t_out_fmt        <= reg_out_fmt;
            t_out_inv        <= reg_out_inv;
            t_sweep_mode     <= reg_sweep_mode;
            t_sweep_ext_trig <= reg_sweep_ext_trig;
            t_sweep_fstart   <= reg_sweep_fstart;
            t_sweep_fstop    <= reg_sweep_fstop;
            t_sweep_fdelta   <= reg_sweep_fdelta;
            t_sweep_rate     <= reg_sweep_rate;
            t_prof_sel       <= reg_prof_sel;
            t_prof_ext_sel   <= reg_prof_ext_sel;
            t_prof_fword0    <= reg_prof_fword[0];
            t_prof_fword1    <= reg_prof_fword[1];
            t_prof_fword2    <= reg_prof_fword[2];
            t_prof_fword3    <= reg_prof_fword[3];
            t_prof_pow0      <= reg_prof_pow[0];
            t_prof_pow1      <= reg_prof_pow[1];
            t_prof_pow2      <= reg_prof_pow[2];
            t_prof_pow3      <= reg_prof_pow[3];
            xfer_tgl         <= ~xfer_tgl;
        end
    end
end

// Event toggles toward the core. Delayed two cycles behind the write so the
// capture toggle (one cycle delay) always crosses first; both cross through
// equal-depth synchronizers, so e.g. "write sweep config + START with AUTO"
// loads the config before the start pulse fires.
reg srst_p1, srst_p2, start_p1, start_p2, abort_p1, abort_p2;

always @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn) begin
        {srst_p1, srst_p2, start_p1, start_p2, abort_p1, abort_p2} <= 6'b0;
        srst_tgl  <= 1'b0;
        start_tgl <= 1'b0;
        abort_tgl <= 1'b0;
    end else begin
        srst_p1  <= srst_write;   srst_p2  <= srst_p1;
        start_p1 <= start_write;  start_p2 <= start_p1;
        abort_p1 <= abort_write;  abort_p2 <= abort_p1;
        if (srst_p2)  srst_tgl  <= ~srst_tgl;
        if (start_p2) start_tgl <= ~start_tgl;
        if (abort_p2) abort_tgl <= ~abort_tgl;
    end
end

// Interrupt flags (set by core events, W1C, cleared by soft reset)
wire done_p, wrap_p;
dds_cdc_pulse u_sync_done (.clk(PCLK), .rstn(PRESETn), .tgl_in(evt_done_tgl), .pulse(done_p));
dds_cdc_pulse u_sync_wrap (.clk(PCLK), .rstn(PRESETn), .tgl_in(evt_wrap_tgl), .pulse(wrap_p));

wire irq_stat_write = APB_write_en && !ram_sel && (widx == A_IRQ_STAT);

always @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn) begin
        reg_irq_stat <= 3'b000;
    end else if (srst_write) begin
        reg_irq_stat <= 3'b000;
    end else begin
        // set has priority over W1C
        reg_irq_stat[0] <= done_p ? 1'b1 : (irq_stat_write && PWDATA[0] ? 1'b0 : reg_irq_stat[0]);
        reg_irq_stat[1] <= wrap_p ? 1'b1 : (irq_stat_write && PWDATA[1] ? 1'b0 : reg_irq_stat[1]);
        reg_irq_stat[2] <= ack_p  ? 1'b1 : (irq_stat_write && PWDATA[2] ? 1'b0 : reg_irq_stat[2]);
    end
end

assign dds_irq = |(reg_irq_stat & reg_irq_en);

// Status synchronizers (informational; ACTIVE_PROF bits sync independently)
wire stat_active_s, stat_dir_s, stat_prof0_s, stat_prof1_s;
dds_cdc_bit u_sync_act  (.clk(PCLK), .rstn(PRESETn), .d(stat_sweep_active),   .q(stat_active_s));
dds_cdc_bit u_sync_dir  (.clk(PCLK), .rstn(PRESETn), .d(stat_sweep_dir),      .q(stat_dir_s));
dds_cdc_bit u_sync_pr0  (.clk(PCLK), .rstn(PRESETn), .d(stat_active_prof[0]), .q(stat_prof0_s));
dds_cdc_bit u_sync_pr1  (.clk(PCLK), .rstn(PRESETn), .d(stat_active_prof[1]), .q(stat_prof1_s));

// Waveform RAM port A: read is fetched in the APB setup phase so data is
// ready in the access phase with zero wait states; write lands in the
// access phase.
assign ram_we    = APB_write_en && ram_sel;
assign ram_en    = (PSEL && !PENABLE && ram_sel) || ram_we;
assign ram_addr  = PADDR[13:2];
assign ram_wdata = PWDATA[13:0];

// APB Read Data
always @(*) begin
    PRDATA = 32'h0;
    if (APB_read_en) begin
        if (ram_sel) begin
            PRDATA = {18'h0, ram_rdata};
        end else begin
            case (widx)
                A_ID:           PRDATA = VERSION;
                A_CTRL:         PRDATA = {17'h0, reg_out_inv, reg_out_fmt, reg_out_width,
                                          2'b00, reg_freq_mode, 1'b0, reg_wave_sel,
                                          3'b000, reg_en};
                A_STATUS:       PRDATA = {27'h0, busy, stat_prof1_s, stat_prof0_s,
                                          stat_dir_s, stat_active_s};
                A_FWORD:        PRDATA = reg_fword;
                A_POW:          PRDATA = {16'h0, reg_pow};
                A_AMP:          PRDATA = {15'h0, reg_amp};
                A_DUTY:         PRDATA = {16'h0, reg_duty};
                A_UPDATE:       PRDATA = {30'h0, reg_auto, 1'b0};
                A_SWEEP_CTRL:   PRDATA = {29'h0, reg_sweep_ext_trig, reg_sweep_mode};
                A_SWEEP_FSTART: PRDATA = reg_sweep_fstart;
                A_SWEEP_FSTOP:  PRDATA = reg_sweep_fstop;
                A_SWEEP_FDELTA: PRDATA = reg_sweep_fdelta;
                A_SWEEP_RATE:   PRDATA = {8'h0, reg_sweep_rate};
                A_PROF_CTRL:    PRDATA = {27'h0, reg_prof_ext_sel, 2'b00, reg_prof_sel};
                A_PROF0_FWORD:  PRDATA = reg_prof_fword[0];
                A_PROF1_FWORD:  PRDATA = reg_prof_fword[1];
                A_PROF2_FWORD:  PRDATA = reg_prof_fword[2];
                A_PROF3_FWORD:  PRDATA = reg_prof_fword[3];
                A_PROF_POW10:   PRDATA = {reg_prof_pow[1], reg_prof_pow[0]};
                A_PROF_POW32:   PRDATA = {reg_prof_pow[3], reg_prof_pow[2]};
                A_IRQ_EN:       PRDATA = {29'h0, reg_irq_en};
                A_IRQ_STAT:     PRDATA = {29'h0, reg_irq_stat};
                A_OFFSET:       PRDATA = {18'h0, reg_offset};
                default:        PRDATA = 32'h0;
            endcase
        end
    end
end

endmodule // dds_regs
