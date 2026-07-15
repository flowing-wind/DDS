// dds_core.v - DDS datapath (dds_clk domain)
//
// Architecture per the ADI DDS tutorial (Doc/dds.pdf):
//   frequency source (fixed / sweep chirp / profile hop)
//     -> 32-bit phase accumulator (+ phase offset word)
//     -> waveform generation (1/4-wave sine LUT, square, triangle,
//        ramp, user RAM)
//     -> amplitude multiplier (Q1.15, 0..1.0) + DC offset (saturating)
//     -> output conditioning (14/12-bit, offset-binary / two's complement)
//
// Output pipeline latency: 6 dds_clk cycles from phase accumulator to
// dac_data. Configuration arrives from dds_regs in the same clock domain and
// is latched as one atomic set on cfg_apply.
//
// The only asynchronous inputs are the hardware pacing pins (sweep_trig,
// hop_sel); they are synchronized here.

module dds_core #(
    parameter SIN_LUT_FILE = "dds_sin_lut.mem"
) (
    input  wire        dds_clk,
    input  wire        dds_rstn,

    // configuration from dds_regs (latched on cfg_apply)
    input  wire [31:0] cfg_fword,
    input  wire [15:0] cfg_pow,
    input  wire [15:0] cfg_amp,           // Q1.15, 0x8000 = 1.0
    input  wire [15:0] cfg_duty,
    input  wire [13:0] cfg_offset,
    input  wire [2:0]  cfg_wave_sel,
    input  wire [1:0]  cfg_freq_mode,
    input  wire        cfg_out_width,
    input  wire        cfg_out_fmt,
    input  wire        cfg_out_inv,
    input  wire [1:0]  cfg_sweep_mode,
    input  wire        cfg_sweep_ext_trig,
    input  wire [31:0] cfg_sweep_fstart,
    input  wire [31:0] cfg_sweep_fstop,
    input  wire [31:0] cfg_sweep_fdelta,
    input  wire [31:0] cfg_sweep_rate,
    input  wire [1:0]  cfg_prof_sel,
    input  wire        cfg_prof_ext_sel,
    input  wire [31:0] cfg_prof_fword0,
    input  wire [31:0] cfg_prof_fword1,
    input  wire [31:0] cfg_prof_fword2,
    input  wire [31:0] cfg_prof_fword3,
    input  wire [15:0] cfg_prof_pow0,
    input  wire [15:0] cfg_prof_pow1,
    input  wire [15:0] cfg_prof_pow2,
    input  wire [15:0] cfg_prof_pow3,

    input  wire        cfg_en,            // CTRL.EN level, immediate
    input  wire        cfg_apply,         // 1-cycle: latch the config above
    input  wire        srst_p,            // 1-cycle: soft reset
    input  wire        start_p,           // 1-cycle: sweep start
    input  wire        abort_p,           // 1-cycle: sweep abort

    output reg         upd_done_p,        // 1-cycle: config latched
    output reg         evt_done_p,        // 1-cycle: single sweep finished
    output reg         evt_wrap_p,        // 1-cycle: sweep wrapped / reversed
    output wire        stat_sweep_active,
    output wire        stat_sweep_dir,
    output wire [1:0]  stat_active_prof,

    // hardware pacing inputs (asynchronous, synchronized here)
    input  wire        sweep_trig,        // rising edge starts a sweep
    input  wire [1:0]  hop_sel,           // profile select (FSK/PSK data)

    // waveform RAM playback port
    output wire [11:0] wave_addr,
    input  wire [13:0] wave_rdata,

    // DAC interface
    output reg  [13:0] dac_data,
    output reg         dac_valid
);

localparam [2:0] W_SIN = 3'd0, W_COS = 3'd1, W_SQ  = 3'd2,
                 W_TRI = 3'd3, W_RMP = 3'd4, W_USR = 3'd5;
localparam [1:0] M_FIXED = 2'd0, M_SWEEP = 2'd1, M_PROF = 2'd2;
localparam [1:0] SW_SINGLE = 2'd0, SW_SAW = 2'd1, SW_UPDOWN = 2'd2;

// ----------------------------------------------- async pacing pins -> sync
// sweep_trig: 2-FF sync + rising edge detect
reg [2:0] trig_s;
always @(posedge dds_clk or negedge dds_rstn) begin
    if (!dds_rstn) trig_s <= 3'b000;
    else           trig_s <= {trig_s[1:0], sweep_trig};
end
wire trig_edge = trig_s[1] & ~trig_s[2];

// hop_sel: 2-FF sync + 2-cycle agreement filter (the two bits may skew)
reg [1:0] hs_m, hs_s, hs_prev, hs_stable;
always @(posedge dds_clk or negedge dds_rstn) begin
    if (!dds_rstn) begin
        hs_m <= 2'b00; hs_s <= 2'b00; hs_prev <= 2'b00; hs_stable <= 2'b00;
    end else begin
        hs_m    <= hop_sel;
        hs_s    <= hs_m;
        hs_prev <= hs_s;
        if (hs_s == hs_prev)
            hs_stable <= hs_s;
    end
end

// ------------------------------------------------------------ active config
reg [31:0] a_fword, a_fstart, a_fdelta;
reg [31:0] a_limit;                       // fstop - fstart
reg [15:0] a_pow, a_duty, a_amp;
reg signed [13:0] a_offset;
reg [2:0]  a_wave;
reg [1:0]  a_mode, a_swmode, a_psel;
reg        a_ext_trig, a_ext_psel;
reg [31:0] a_rate;
reg        a_width12, a_fmt2c, a_inv;
reg [31:0] a_pfw  [0:3];
reg [15:0] a_ppow [0:3];

always @(posedge dds_clk or negedge dds_rstn) begin
    if (!dds_rstn) begin
        a_fword <= 32'h0;   a_fstart <= 32'h0;  a_fdelta <= 32'h0;
        a_limit <= 32'h0;   a_pow <= 16'h0;     a_duty <= 16'h8000;
        a_amp <= 16'h8000;  a_offset <= 14'sd0;
        a_wave <= W_SIN;    a_mode <= M_FIXED;
        a_swmode <= SW_SINGLE; a_psel <= 2'd0;
        a_ext_trig <= 1'b0; a_ext_psel <= 1'b0; a_rate <= 32'd1;
        a_width12 <= 1'b0;  a_fmt2c <= 1'b0;    a_inv <= 1'b0;
        a_pfw[0] <= 32'h0;  a_pfw[1] <= 32'h0;
        a_pfw[2] <= 32'h0;  a_pfw[3] <= 32'h0;
        a_ppow[0] <= 16'h0; a_ppow[1] <= 16'h0;
        a_ppow[2] <= 16'h0; a_ppow[3] <= 16'h0;
        upd_done_p <= 1'b0;
    end else begin
        upd_done_p <= cfg_apply;
        if (cfg_apply) begin
            a_fword    <= cfg_fword;
            a_fstart   <= cfg_sweep_fstart;
            a_fdelta   <= cfg_sweep_fdelta;
            a_limit    <= cfg_sweep_fstop - cfg_sweep_fstart;
            a_pow      <= cfg_pow;
            a_duty     <= cfg_duty;
            a_amp      <= cfg_amp;
            a_offset   <= $signed(cfg_offset);
            a_wave     <= cfg_wave_sel;
            a_mode     <= cfg_freq_mode;
            a_swmode   <= cfg_sweep_mode;
            a_psel     <= cfg_prof_sel;
            a_ext_trig <= cfg_sweep_ext_trig;
            a_ext_psel <= cfg_prof_ext_sel;
            a_rate     <= (cfg_sweep_rate == 32'd0) ? 32'd1 : cfg_sweep_rate;
            a_width12  <= cfg_out_width;
            a_fmt2c    <= cfg_out_fmt;
            a_inv      <= cfg_out_inv;
            a_pfw[0]   <= cfg_prof_fword0;  a_pfw[1] <= cfg_prof_fword1;
            a_pfw[2]   <= cfg_prof_fword2;  a_pfw[3] <= cfg_prof_fword3;
            a_ppow[0]  <= cfg_prof_pow0;    a_ppow[1] <= cfg_prof_pow1;
            a_ppow[2]  <= cfg_prof_pow2;    a_ppow[3] <= cfg_prof_pow3;
        end
    end
end

// ------------------------------------------------------------ sweep engine
// Frequency accumulator + ramp timing logic (tutorial Fig. 11.3): every
// a_rate cycles add a_fdelta to freq_off, clamped at a_limit; the phase
// accumulator input is a_fstart + freq_off.
reg         sweep_act, sweep_dir;      // dir: 0 = up, 1 = down
reg  [31:0] freq_off;
reg  [31:0] rate_cnt;

wire        rate_hit  = (rate_cnt + 32'd1 >= a_rate);
wire [32:0] off_up    = {1'b0, freq_off} + {1'b0, a_fdelta};
wire        hit_top   = (off_up >= {1'b0, a_limit});
wire        hit_bot   = (freq_off <= a_fdelta);
wire        start_evt = (a_ext_trig ? trig_edge : start_p) && (a_mode == M_SWEEP);

always @(posedge dds_clk or negedge dds_rstn) begin
    if (!dds_rstn) begin
        sweep_act <= 1'b0;  sweep_dir <= 1'b0;
        freq_off  <= 32'h0; rate_cnt  <= 32'd0;
        evt_done_p <= 1'b0; evt_wrap_p <= 1'b0;
    end else begin
        evt_done_p <= 1'b0;
        evt_wrap_p <= 1'b0;

        if (srst_p || (a_mode != M_SWEEP)) begin
            sweep_act <= 1'b0;  sweep_dir <= 1'b0;
            freq_off  <= 32'h0; rate_cnt  <= 32'd0;
        end else if (start_evt) begin
            sweep_act <= 1'b1;  sweep_dir <= 1'b0;
            freq_off  <= 32'h0; rate_cnt  <= 32'd0;
        end else if (abort_p) begin
            sweep_act <= 1'b0;  sweep_dir <= 1'b0;
            freq_off  <= 32'h0; rate_cnt  <= 32'd0;
        end else if (sweep_act && cfg_en) begin
            if (rate_hit) begin
                rate_cnt <= 32'd0;
                if (!sweep_dir) begin                       // sweeping up
                    if (hit_top) begin
                        case (a_swmode)
                            SW_SAW: begin
                                freq_off   <= 32'h0;        // restart at FSTART
                                evt_wrap_p <= 1'b1;
                            end
                            SW_UPDOWN: begin
                                freq_off   <= a_limit;
                                sweep_dir  <= 1'b1;
                                evt_wrap_p <= 1'b1;
                            end
                            default: begin                  // SW_SINGLE: hold FSTOP
                                freq_off   <= a_limit;
                                sweep_act  <= 1'b0;
                                evt_done_p <= 1'b1;
                            end
                        endcase
                    end else begin
                        freq_off <= off_up[31:0];
                    end
                end else begin                              // sweeping down
                    if (hit_bot) begin
                        freq_off   <= 32'h0;
                        sweep_dir  <= 1'b0;
                        evt_wrap_p <= 1'b1;
                    end else begin
                        freq_off <= freq_off - a_fdelta;
                    end
                end
            end else begin
                rate_cnt <= rate_cnt + 32'd1;
            end
        end
    end
end

assign stat_sweep_active = sweep_act;
assign stat_sweep_dir    = sweep_dir;

// ---------------------------------------------------- frequency / phase mux
reg [1:0]  act_prof;
reg [31:0] eff_fword;
reg [15:0] eff_pow;

always @(posedge dds_clk or negedge dds_rstn) begin
    if (!dds_rstn) begin
        act_prof  <= 2'd0;
        eff_fword <= 32'h0;
        eff_pow   <= 16'h0;
    end else begin
        act_prof <= a_ext_psel ? hs_stable : a_psel;

        case (a_mode)
            M_SWEEP:  eff_fword <= a_fstart + freq_off;
            M_PROF:   eff_fword <= a_pfw[act_prof];
            default:  eff_fword <= a_fword;
        endcase

        // cosine = sine advanced by 90 degrees (0x4000 in 16-bit phase units)
        eff_pow <= ((a_mode == M_PROF) ? a_ppow[act_prof] : a_pow)
                   + ((a_wave == W_COS) ? 16'h4000 : 16'h0000);
    end
end

assign stat_active_prof = act_prof;

// --------------------------------------------------------------- pipeline
// s0: phase accumulator
reg [31:0] phase;
always @(posedge dds_clk or negedge dds_rstn) begin
    if (!dds_rstn)     phase <= 32'h0;
    else if (srst_p)   phase <= 32'h0;
    else if (cfg_en)   phase <= phase + eff_fword;
end

// s1: apply phase offset
reg [31:0] ph1;
always @(posedge dds_clk or negedge dds_rstn) begin
    if (!dds_rstn)    ph1 <= 32'h0;
    else if (cfg_en)  ph1 <= phase + {eff_pow, 16'h0000};
end

// s2: sine LUT read (quarter-wave: quadrant = ph1[31:30], addr = ph1[29:18])
//     plus square/triangle/ramp computed arithmetically, user RAM read
wire [11:0] lut_addr = ph1[30] ? ~ph1[29:18] : ph1[29:18];
wire [12:0] lut_q;

dds_sin_lut #(.MEM_FILE(SIN_LUT_FILE)) u_sin_lut (
    .clk  (dds_clk),
    .en   (cfg_en),
    .addr (lut_addr),
    .dout (lut_q)
);

assign wave_addr = ph1[31:20];

wire [13:0] tri_fold = ph1[31] ? ~ph1[30:17] : ph1[30:17];   // 0..16383 up/down

reg               sign2;
reg signed [13:0] sq2, tri2, rmp2;
always @(posedge dds_clk or negedge dds_rstn) begin
    if (!dds_rstn) begin
        sign2 <= 1'b0;
        sq2 <= 14'sd0; tri2 <= 14'sd0; rmp2 <= 14'sd0;
    end else if (cfg_en) begin
        sign2 <= ph1[31];
        sq2   <= (ph1[31:16] < a_duty) ? 14'sd8191 : -14'sd8191;
        tri2  <= {~tri_fold[13], tri_fold[12:0]};            // unsigned - 8192
        rmp2  <= {~ph1[31], ph1[30:18]};                     // -FS rising to +FS
    end
end

// s3: waveform select, output inversion
reg signed [13:0] smp3;
reg signed [13:0] wave_mux;
always @(*) begin
    case (a_wave)
        W_SQ:    wave_mux = sq2;
        W_TRI:   wave_mux = tri2;
        W_RMP:   wave_mux = rmp2;
        W_USR:   wave_mux = $signed(wave_rdata);
        default: wave_mux = sign2 ? -$signed({1'b0, lut_q})  // sine / cosine
                                  :  $signed({1'b0, lut_q});
    endcase
    if (a_inv)
        wave_mux = (wave_mux == -14'sd8192) ? 14'sd8191 : -wave_mux;
end

always @(posedge dds_clk or negedge dds_rstn) begin
    if (!dds_rstn)    smp3 <= 14'sd0;
    else if (cfg_en)  smp3 <= wave_mux;
end

// s4: amplitude multiply, sample(14s) x amp(Q1.15) -> 31-bit product
reg signed [30:0] prod4;
always @(posedge dds_clk or negedge dds_rstn) begin
    if (!dds_rstn)    prod4 <= 31'sd0;
    else if (cfg_en)  prod4 <= smp3 * $signed({1'b0, a_amp});
end

// s5: round to nearest, add DC offset, saturate to 14-bit signed.
// Worst case |rnd5 + offset| = 8192 + 8192 fits in 16-bit signed.
wire signed [15:0] rnd5 = (prod4 + 31'sd16384) >>> 15;
wire signed [15:0] ofs5 = rnd5 + a_offset;
reg  signed [13:0] samp5;
always @(posedge dds_clk or negedge dds_rstn) begin
    if (!dds_rstn) samp5 <= 14'sd0;
    else if (cfg_en) begin
        if      (ofs5 >  16'sd8191) samp5 <=  14'sd8191;
        else if (ofs5 < -16'sd8192) samp5 <= -14'sd8192;
        else                        samp5 <= ofs5[13:0];
    end
end

// s6: width / coding conditioning, enable gating
wire signed [14:0] r12   = ($signed({samp5[13], samp5}) + 15'sd2) >>> 2; // round 14 -> 12
wire signed [11:0] r12s  = (r12 >  15'sd2047) ?  12'sd2047 :
                           (r12 < -15'sd2048) ? -12'sd2048 : r12[11:0];
wire        [13:0] w14   = a_width12 ? {r12s, 2'b00} : samp5;      // MSB-aligned
wire        [13:0] w14f  = a_fmt2c ? w14 : {~w14[13], w14[12:0]};  // offset binary
wire        [13:0] w_mid = a_fmt2c ? 14'h0000 : 14'h2000;          // mid-scale

// enable delayed to match pipeline depth so stale samples never reach the DAC
reg [5:0] en_dly;
always @(posedge dds_clk or negedge dds_rstn) begin
    if (!dds_rstn) en_dly <= 6'b0;
    else           en_dly <= {en_dly[4:0], cfg_en};
end

always @(posedge dds_clk or negedge dds_rstn) begin
    if (!dds_rstn) begin
        dac_data  <= 14'h0;
        dac_valid <= 1'b0;
    end else begin
        dac_data  <= en_dly[5] ? w14f : w_mid;
        dac_valid <= en_dly[5];
    end
end

endmodule // dds_core
