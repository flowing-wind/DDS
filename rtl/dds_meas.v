// dds_meas.v - amplitude measurement over a window of ADC samples
//
// Consumes one ADC stream and produces, once per window of 2^WIN_LOG2 samples:
//
//   vpp   peak-to-peak, max - min                        (ADC codes, 0..4095)
//   rms   RMS of the AC component                        (ADC codes, 0..2048)
//   mean  DC average                                     (ADC codes, signed)
//   vmin / vmax  the window's extremes                   (ADC codes, signed)
//   otr   1 if the ADC flagged over-range anywhere in the window
//
// Everything runs in the dds_clk domain off smp_valid, which pulses once per
// ADC period (every second dds_clk).
//
// ---------------------------------------------------- why RMS subtracts DC
//
// The AD9226 module's front end centres 0 V at code 2048 and dds_adc_if already
// removes that, but the residual - op-amp offset, reference tolerance, whatever
// DC the DDS itself is putting out via its OFFSET register - is real and can be
// hundreds of codes. sqrt(E[x^2]) would fold all of it into the "amplitude" and
// the loop would happily shrink a perfectly good sine to make room for a DC
// term it cannot control. So this computes the variance,
//
//     var = E[x^2] - E[x]^2
//
// and returns sqrt(var), which is the AC RMS and nothing else. mean is exposed
// separately because it is genuinely useful: it is how you check the analog
// front end is centred, and how you calibrate OFFSET.
//
// ------------------------------------------------------- Vpp versus RMS
//
// Both are computed every window; AGC_CTRL.METRIC picks which one drives the
// loop. They fail in different directions and it is worth knowing which:
//
//   Vpp  is waveform-agnostic and matches what a scope shows, but it is a
//        two-sample statistic. At 50 MSPS a 1 MHz output gives 50 samples per
//        period and min/max lands within ~0.2 % of the true peak; a 10 MHz
//        output gives 5, the samples sit at nearly fixed phases, and min/max
//        systematically reads LOW. Close the loop on Vpp up there and it will
//        push the real amplitude high to compensate.
//
//   RMS  averages every sample in the window, so undersampling costs it almost
//        nothing and it stays honest to the top of the band. The catch is that
//        it depends on the waveform: Vpp = 2*sqrt(2)*RMS for a sine, 2*RMS for
//        a square, 2*sqrt(3)*RMS for a triangle. Change WAVE and the target has
//        to change with it.
//
// Rule of thumb: Vpp below ~1 MHz, RMS above it.
//
// ------------------------------------------------------------ window length
//
// THE WINDOW MUST SPAN AT LEAST ONE FULL OUTPUT PERIOD, and several is better.
// Nothing here can enforce that - it depends on FWORD, which this module never
// sees - so it is on the driver. A window shorter than a period sees a slice of
// the waveform and reports whatever amplitude that slice happened to have; the
// loop then chases the slice. At 50 MSPS, 2^WIN_LOG2 samples covers
// 2^WIN_LOG2 / 50e6 seconds, so WIN_LOG2 = 16 (1.31 ms) is good down to about
// 5 kHz with 4 periods of margin, and WIN_LOG2 = 24 (335 ms) reaches ~10 Hz.

module dds_meas (
    input  wire        clk,
    input  wire        rstn,

    // configuration
    input  wire [4:0]  cfg_win_log2,   // clamped to 4..24, latched per window
    input  wire        clr,            // 1-cycle: abandon this window, restart

    // ADC stream
    input  wire signed [11:0] smp,
    input  wire               smp_valid,
    input  wire               otr,

    // results, all updated together on result_valid
    output reg  [11:0] vpp,
    output reg  [11:0] rms,
    output reg  signed [11:0] mean,
    output reg  signed [11:0] vmin,
    output reg  signed [11:0] vmax,
    output reg         otr_win,
    output reg         result_valid    // 1-cycle
);

localparam [4:0] W_MIN = 5'd4, W_MAX = 5'd24;

wire [4:0] win_req = (cfg_win_log2 < W_MIN) ? W_MIN :
                     (cfg_win_log2 > W_MAX) ? W_MAX : cfg_win_log2;

// ------------------------------------------------------------- accumulation
reg [4:0]  w;                 // this window's length, latched at its start
reg [24:0] cnt;
reg [24:0] win_last;          // (1 << w) - 1, registered at window start so
                              // the barrel shift is not in the per-cycle path
reg [47:0] acc_sq;            // 23-bit squares x 2^24 samples -> 47 bits used
reg signed [39:0] acc_s;      // 12-bit samples x 2^24 -> 36 bits used
reg signed [11:0] cur_min, cur_max;
reg        cur_otr;

// Stage A: register the per-sample products, so the 12x12 square never sits
// in series with the 48-bit accumulate in one clock. smp_valid comes at most
// every second cycle, so stage B below always lands in the gap.
reg               d_valid;
reg signed [11:0] d_smp;
reg [23:0]        d_sq;
reg               d_otr;

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        d_valid <= 1'b0;
        d_smp   <= 12'sd0;
        d_sq    <= 24'd0;
        d_otr   <= 1'b0;
    end else begin
        d_valid <= smp_valid && !clr;
        if (smp_valid) begin
            d_smp <= smp;
            d_sq  <= smp * smp;   // |smp| <= 2048 -> <= 4194304, always >= 0
            d_otr <= otr;
        end
    end
end

wire win_end = d_valid && (cnt == win_last);

// The final sample of a window has to land in both places at once: folded into
// the totals that get latched, and NOT into the accumulators, which restart on
// that same edge. These carry the folded-in version to the latch below.
wire [47:0] acc_sq_f = acc_sq + {24'd0, d_sq};
wire signed [39:0] acc_s_f = acc_s + d_smp;
wire signed [11:0] min_f = (d_smp < cur_min) ? d_smp : cur_min;
wire signed [11:0] max_f = (d_smp > cur_max) ? d_smp : cur_max;
wire        otr_f  = cur_otr | d_otr;

// Reset and roll-over are spelled out separately: the async-reset branch may
// only hold constants (win_req would break the reset inference), and folding
// !rstn into one condition stops the tool inferring the async reset at all.
always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        cnt     <= 25'd0;
        acc_sq  <= 48'd0;
        acc_s   <= 40'sd0;
        cur_min <= 12'sd2047;
        cur_max <= -12'sd2048;
        cur_otr <= 1'b0;
        w       <= 5'd16;
        win_last <= 25'd65535;
    end else if (clr || win_end) begin
        cnt     <= 25'd0;
        acc_sq  <= 48'd0;
        acc_s   <= 40'sd0;
        cur_min <= 12'sd2047;
        cur_max <= -12'sd2048;
        cur_otr <= 1'b0;
        w       <= win_req;          // a new WIN_LOG2 only takes effect here
        win_last <= (25'd1 << win_req) - 25'd1;
    end else if (d_valid) begin
        cnt    <= cnt + 25'd1;
        acc_sq <= acc_sq + {24'd0, d_sq};
        acc_s  <= acc_s + d_smp;
        if (d_smp < cur_min) cur_min <= d_smp;
        if (d_smp > cur_max) cur_max <= d_smp;
        if (d_otr)           cur_otr <= 1'b1;
    end
end

// ------------------------------------------------------------- the maths
// Runs once per window while the next one is already accumulating. The shortest
// legal window is 2^4 samples = 32 dds_clk cycles, and the longest this takes
// is 5 + 12 = 17, so the two never collide.
//
// The window division is a shift by l_w (0..24), split over two cycles - a
// coarse shift by {0,8,16,24} then a fine shift by 0..7 - because a 48-bit
// barrel shifter with a 5-bit select in one tick is exactly the kind of
// mux tree that fails timing for logic that runs once per millisecond.
localparam [2:0] S_IDLE = 3'd0, S_SH1 = 3'd1, S_MEAN = 3'd2,
                 S_VAR  = 3'd3, S_SQRT = 3'd4;

reg [2:0]  st;
reg [47:0] l_acc_sq;
reg signed [39:0] l_acc_s;
reg signed [11:0] l_min, l_max;
reg        l_otr;
reg [4:0]  l_w;
reg [47:0] sh_sq;
reg signed [39:0] sh_s;
reg [22:0] ms;
reg signed [11:0] mean_c;

wire [47:0] sh_sq_c   = l_acc_sq >> {l_w[4:3], 3'b000};   // coarse: 0/8/16/24
wire signed [39:0] sh_s_c = l_acc_s >>> {l_w[4:3], 3'b000};
wire [47:0] ms_full   = sh_sq >> l_w[2:0];                // fine: 0..7
wire signed [39:0] mean_full = sh_s >>> l_w[2:0];
wire [23:0] mean_sq  = mean_c * mean_c;         // signed 12 x 12, always >= 0
// 2047 - (-2048) = 4095 does not fit in 12-bit signed, so widen before
// subtracting and narrow after. Letting it wrap in 12 bits happens to give the
// right bit pattern, which is exactly the kind of accident that survives review
// and then breaks when someone widens the ADC.
wire signed [12:0] vpp_c = $signed({l_max[11], l_max}) - $signed({l_min[11], l_min});
// Truncation in mean_full biases mean_sq upward by a fraction of a code, so
// var can come out a hair negative on a pure-DC input. Floor it at zero rather
// than handing the square root a negative number.
wire [23:0] var_c    = ({1'b0, ms} > mean_sq) ? ({1'b0, ms} - mean_sq) : 24'd0;

reg         sq_start;
wire [11:0] sq_y;
wire        sq_done;

dds_isqrt u_isqrt (
    .clk   (clk),
    .rstn  (rstn),
    .start (sq_start),
    .x     (var_c),
    .y     (sq_y),
    .done  (sq_done),
    .busy  ()
);

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        st <= S_IDLE;
        vpp <= 12'd0;  rms <= 12'd0;  mean <= 12'sd0;
        vmin <= 12'sd0; vmax <= 12'sd0;
        otr_win <= 1'b0; result_valid <= 1'b0; sq_start <= 1'b0;
        l_acc_sq <= 48'd0; l_acc_s <= 40'sd0;
        l_min <= 12'sd0; l_max <= 12'sd0; l_otr <= 1'b0; l_w <= 5'd16;
        sh_sq <= 48'd0; sh_s <= 40'sd0;
        ms <= 23'd0; mean_c <= 12'sd0;
    end else begin
        result_valid <= 1'b0;
        sq_start     <= 1'b0;

        case (st)
            S_IDLE:
                if (win_end) begin
                    l_acc_sq <= acc_sq_f;
                    l_acc_s  <= acc_s_f;
                    l_min    <= min_f;
                    l_max    <= max_f;
                    l_otr    <= otr_f;
                    l_w      <= w;
                    st       <= S_SH1;
                end

            S_SH1: begin                       // coarse shift (0/8/16/24)
                sh_sq <= sh_sq_c;
                sh_s  <= sh_s_c;
                st    <= S_MEAN;
            end

            S_MEAN: begin                      // fine shift (0..7) -> mean, ms
                ms     <= ms_full[22:0];
                mean_c <= mean_full[11:0];
                st     <= S_VAR;
            end

            S_VAR: begin
                sq_start <= 1'b1;      // var_c is settled from mean_c / ms
                st       <= S_SQRT;
            end

            S_SQRT:
                if (sq_done) begin
                    vpp          <= vpp_c[11:0];
                    rms          <= sq_y;
                    mean         <= mean_c;
                    vmin         <= l_min;
                    vmax         <= l_max;
                    otr_win      <= l_otr;
                    result_valid <= 1'b1;
                    st           <= S_IDLE;
                end
        endcase
    end
end

endmodule // dds_meas
