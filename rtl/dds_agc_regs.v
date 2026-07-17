// dds_agc_regs.v - register block for one channel's amplitude loop
//
// Occupies bus block 1 (word addresses 0x40..0x7F) of the channel's space;
// dds_regs owns block 0 (0x00..0x3F) and dds_channel muxes the two read paths.
//
//   0x40 AGC_CTRL     0x44 AGC_TOL       0x48 AGC_AMP  (RO)  0x4C AGC_VMIN (RO)
//   0x41 AGC_STATUS   0x45 AGC_WIN       0x49 AGC_VPP  (RO)  0x4D AGC_VMAX (RO)
//   0x42 AGC_TARGET   0x46 AGC_AMP_MIN   0x4A AGC_RMS  (RO)
//   0x43 AGC_KI       0x47 AGC_AMP_MAX   0x4B AGC_MEAN (RO)
//
// Unlike dds_regs, these are plain registers with no shadow/commit pair, and
// they ignore UPDATE.AUTO. The atomic-frame machinery exists so that a 32-bit
// FWORD written as two 16-bit halves never puts an intermediate frequency on
// the DAC - a real hazard, because the datapath reads FWORD every clock. The
// loop reads these once per measurement window, milliseconds apart, and each
// register is a single 16-bit flop update that cannot tear. There is nothing
// for a shadow to protect here, so there isn't one.
//
// CLR is the exception: it is deferred to the CS rising edge like the sweep
// controls, so "clear + retarget + enable" written as one frame does what it
// reads like.

module dds_agc_regs (
    input  wire        clk,
    input  wire        rstn,

    // internal bus (already decoded for this channel AND this block)
    input  wire [5:0]  bus_idx,        // bus_addr[5:0]
    input  wire [15:0] bus_wdata,
    input  wire        bus_we,
    input  wire        bus_frame_end,
    output reg  [15:0] bus_rdata,

    // configuration out
    output wire        cfg_en,
    output wire        cfg_hold,
    output wire [1:0]  cfg_src,        // 00 = ADC0, 01 = ADC1, 1x = none
    output wire        cfg_metric,     // 0 = Vpp, 1 = RMS
    output wire [11:0] cfg_target,
    output wire [15:0] cfg_ki,
    output wire [11:0] cfg_tol,
    output wire [4:0]  cfg_win_log2,
    output wire [15:0] cfg_amp_min,
    output wire [15:0] cfg_amp_max,
    output reg         clr_p,          // 1-cycle at frame end

    // status in
    input  wire [15:0] st_amp,
    input  wire        st_locked,
    input  wire        st_sat_hi,
    input  wire        st_sat_lo,
    input  wire [11:0] st_vpp,
    input  wire [11:0] st_rms,
    input  wire signed [11:0] st_mean,
    input  wire signed [11:0] st_vmin,
    input  wire signed [11:0] st_vmax,
    input  wire        st_otr
);

localparam A_CTRL = 6'h00, A_STAT   = 6'h01, A_TARGET = 6'h02, A_KI   = 6'h03,
           A_TOL  = 6'h04, A_WIN    = 6'h05, A_AMIN   = 6'h06, A_AMAX = 6'h07,
           A_AMP  = 6'h08, A_VPP    = 6'h09, A_RMS    = 6'h0A, A_MEAN = 6'h0B,
           A_VMIN = 6'h0C, A_VMAX   = 6'h0D;

reg        r_en, r_hold, r_metric;
reg [1:0]  r_src;
reg [11:0] r_target, r_tol;
reg [15:0] r_ki, r_amin, r_amax;
reg [4:0]  r_win;

assign cfg_en       = r_en;
assign cfg_hold     = r_hold;
assign cfg_src      = r_src;
assign cfg_metric   = r_metric;
assign cfg_target   = r_target;
assign cfg_ki       = r_ki;
assign cfg_tol      = r_tol;
assign cfg_win_log2 = r_win;
assign cfg_amp_min  = r_amin;
assign cfg_amp_max  = r_amax;

wire hit = bus_we;

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        r_en     <= 1'b0;
        r_hold   <= 1'b0;
        r_src    <= 2'b00;        // ADC0
        r_metric <= 1'b0;         // Vpp
        r_target <= 12'd0;
        r_ki     <= 16'h0600;     // 6.0 in Q8.8 - see dds_agc.v for the sums
        r_tol    <= 12'd2;
        r_win    <= 5'd16;        // 65536 samples = 1.31 ms at 50 MSPS
        r_amin   <= 16'h0000;
        r_amax   <= 16'h8000;
    end else if (hit) begin
        case (bus_idx)
            A_CTRL: begin
                r_en     <= bus_wdata[0];
                r_hold   <= bus_wdata[1];
                r_src    <= bus_wdata[5:4];
                r_metric <= bus_wdata[8];
            end
            A_TARGET: r_target <= bus_wdata[11:0];
            A_KI:     r_ki     <= bus_wdata;
            A_TOL:    r_tol    <= bus_wdata[11:0];
            A_WIN:    r_win    <= bus_wdata[4:0];
            A_AMIN:   r_amin   <= bus_wdata;
            A_AMAX:   r_amax   <= bus_wdata;
            default: ;
        endcase
    end
end

// ------------------------------------------------------- deferred CLR pulse
reg pend_clr;
always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        pend_clr <= 1'b0;
        clr_p    <= 1'b0;
    end else begin
        clr_p <= 1'b0;
        if (hit && (bus_idx == A_CTRL) && bus_wdata[2])
            pend_clr <= 1'b1;
        if (bus_frame_end) begin
            clr_p    <= pend_clr;
            pend_clr <= 1'b0;
        end
    end
end

// --------------------------------------------------------------- read path
// One register stage, matching dds_regs, so dds_channel can mux the two blocks
// with a single delayed select.
reg [15:0] rd_c;

always @(*) begin
    case (bus_idx)
        A_CTRL:   rd_c = {7'h0, r_metric, 2'b00, r_src, 1'b0, r_hold, r_en};
        A_STAT:   rd_c = {12'h0, st_otr, st_sat_lo, st_sat_hi, st_locked};
        A_TARGET: rd_c = {4'h0, r_target};
        A_KI:     rd_c = r_ki;
        A_TOL:    rd_c = {4'h0, r_tol};
        A_WIN:    rd_c = {11'h0, r_win};
        A_AMIN:   rd_c = r_amin;
        A_AMAX:   rd_c = r_amax;
        A_AMP:    rd_c = st_amp;
        A_VPP:    rd_c = {4'h0, st_vpp};
        A_RMS:    rd_c = {4'h0, st_rms};
        // sign-extended so the host can just cast to int16_t
        A_MEAN:   rd_c = {{4{st_mean[11]}}, st_mean};
        A_VMIN:   rd_c = {{4{st_vmin[11]}}, st_vmin};
        A_VMAX:   rd_c = {{4{st_vmax[11]}}, st_vmax};
        default:  rd_c = 16'h0;
    endcase
end

always @(posedge clk or negedge rstn) begin
    if (!rstn) bus_rdata <= 16'h0;
    else       bus_rdata <= rd_c;
end

endmodule // dds_agc_regs
