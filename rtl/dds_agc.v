// dds_agc.v - amplitude control loop for one DDS channel
//
// One integrator, clocked once per measurement window:
//
//     err   = target - measured                      (ADC codes)
//     amp  += err * KI                               (Q1.15, clamped)
//
// and that is the whole controller. There is no proportional term because there
// is nothing for it to do: the plant is memoryless over a window (the DDS
// changes amplitude within a few clocks, the measurement takes a window), so
// the loop is a pure gain and an integrator alone gives zero steady-state error
// with a single tuning knob.
//
// ------------------------------------------------------------ loop gain
//
// The plant gain is fixed by hardware: full-scale AMP = 32768 produces some
// measured amplitude M_fs in ADC codes, so
//
//     k = d(measured) / d(AMP) = M_fs / 32768
//
// On the reference setup (DA 6 Vpp full scale, halved to 3 Vpp at the ADC,
// AD9226 at 2.441 mV/code) M_fs is about 1229 codes, so k ~= 1/26.7. Loop gain
// is KI * k, and KI is Q8.8, so KI = 6835 (26.7) is unity loop gain: it would
// try to erase the whole error in one window and ring. Back off to a quarter of
// that and the error decays ~0.78x per window, settling in ~20 windows with no
// overshoot. Hence the reset default of KI = 0x0600 (6.0).
//
// KI is signed only so that a chain with an inverting sense can be handled
// without recompiling. On this board the metrics are Vpp and RMS, which are
// magnitudes and cannot be inverted, so KI should be positive.
//
// -------------------------------------------------------- anti-windup
//
// The clamp is applied to the integrator STATE, not just to the output. That
// distinction is the whole of anti-windup: if the target is unreachable (cable
// unplugged, target above full scale) an unclamped state would keep integrating
// to infinity and then take just as long to unwind once the signal came back,
// leaving the output pinned at full scale for seconds. Clamped, the state
// parks at AMP_MAX and starts back down the moment the error changes sign.
// SAT_HI / SAT_LO report exactly that condition.

module dds_agc (
    input  wire        clk,
    input  wire        rstn,

    // configuration
    input  wire        cfg_en,          // 1 = loop drives AMP
    input  wire        cfg_hold,        // 1 = freeze the integrator, keep driving
    input  wire        clr,             // 1-cycle: reload the integrator from init_amp
    input  wire [11:0] cfg_target,      // ADC codes
    input  wire signed [15:0] cfg_ki,   // Q8.8
    input  wire [11:0] cfg_tol,         // dead band, ADC codes
    input  wire [15:0] cfg_amp_min,     // Q1.15
    input  wire [15:0] cfg_amp_max,     // Q1.15

    // the AMP register value; the integrator starts here so enabling the loop
    // does not jump the output
    input  wire [15:0] init_amp,

    // measurement
    input  wire [11:0] meas,
    input  wire        meas_valid,      // 1-cycle per window

    // to dds_core
    output reg  [15:0] amp,
    output wire        amp_ovr_en,

    // status
    output reg         locked,
    output wire        sat_hi,
    output wire        sat_lo
);

localparam [15:0] AMP_FS = 16'h8000;    // 1.0 in Q1.15

assign amp_ovr_en = cfg_en;

// AMP above full scale is meaningless - the datapath clamps it anyway - so the
// loop is never allowed to integrate up there and then claim it is not railed.
wire [15:0] amp_hi_c = (cfg_amp_max > AMP_FS) ? AMP_FS : cfg_amp_max;
wire [15:0] amp_hi   = (cfg_amp_min > amp_hi_c) ? cfg_amp_min : amp_hi_c;
wire [15:0] amp_lo   = cfg_amp_min;

// ---------------------------------------------------------------- the maths
// Pipelined over three cycles. Windows are tens of microseconds apart, so
// asking the tool to close error-subtract -> DSP multiply -> 30-bit add ->
// clamp in a single 10 ns tick buys nothing and costs timing: stage 0
// registers the error, stage 1 the product, stage 2 the clamped sum. The
// shortest legal window (16 samples = 32 clk) still dwarfs the 3 cycles.
wire signed [12:0] err = $signed({1'b0, cfg_target}) - $signed({1'b0, meas});
wire signed [12:0] err_abs = (err < 0) ? -err : err;
wire               in_band = (err_abs <= $signed({1'b0, cfg_tol}));

reg signed [12:0] p_err;
reg signed [28:0] p_prod;
reg [1:0]         seq;

wire signed [28:0] delta = p_prod >>> 8;                     // undo Q8.8

wire signed [29:0] amp_sum = $signed({14'd0, amp}) + {delta[28], delta};
wire signed [29:0] lo_ext  = $signed({14'd0, amp_lo});
wire signed [29:0] hi_ext  = $signed({14'd0, amp_hi});

wire [15:0] amp_next = (amp_sum > hi_ext) ? amp_hi :
                       (amp_sum < lo_ext) ? amp_lo : amp_sum[15:0];

assign sat_hi = cfg_en && (amp >= amp_hi);
assign sat_lo = cfg_en && (amp <= amp_lo);

// ------------------------------------------------------------- integrator
reg cfg_en_d;

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        amp      <= AMP_FS;
        locked   <= 1'b0;
        cfg_en_d <= 1'b0;
        p_err    <= 13'sd0;
        p_prod   <= 29'sd0;
        seq      <= 2'd0;
    end else begin
        cfg_en_d <= cfg_en;

        if (clr || (cfg_en && !cfg_en_d)) begin
            // Bumpless entry: pick up whatever the AMP register is driving right
            // now, which is exactly what the datapath is already using while the
            // loop is off, so closing the loop does not step the output.
            amp    <= init_amp;
            locked <= 1'b0;
            seq    <= 2'd0;
        end else if (!cfg_en) begin
            // Track the register while open-loop, so AGC_AMP always reads back
            // what the datapath is really using and the next enable is bumpless.
            amp    <= init_amp;
            locked <= 1'b0;
            seq    <= 2'd0;
        end else if (meas_valid && !cfg_hold) begin
            locked <= in_band;
            p_err  <= err;
            seq    <= in_band ? 2'd0 : 2'd1;
        end else if (seq == 2'd1) begin
            p_prod <= p_err * cfg_ki;
            seq    <= 2'd2;
        end else if (seq == 2'd2) begin
            amp    <= amp_next;
            seq    <= 2'd0;
        end
    end
end

endmodule // dds_agc
