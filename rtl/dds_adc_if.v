// dds_adc_if.v - AD9226 parallel ADC capture front end (dds_clk domain)
//
// Two 12-bit parallel ADCs sampled at dds_clk / 2 (50 MSPS from a 100 MHz
// dds_clk). This module owns the ADC *timing*: it produces the pattern that
// the board-level ODDR forwards as the ADC clock, and the clock enable that
// captures the returning data in the board-level IOB input flops. It contains
// no vendor primitives, so it still simulates under Icarus.
//
// WHY dds_clk/2 AND NOT 65 MSPS: the AD9226 tops out at 65 MSPS, so dds_clk
// (100 MHz) cannot drive it directly and something has to give. Dividing by two
// in the ODDR keeps every flop in the design on the single dds_clk - there is
// no second clock, no asynchronous FIFO and no CDC to constrain. Squeezing out
// the last 30 % of sample rate would cost exactly that, which is a bad trade
// for an amplitude loop that averages thousands of samples anyway.
//
// -------------------------------------------------------------- the timing
//
// sq toggles every dds_clk, so one ADC period spans two dds_clk cycles:
//
//   dds_clk    |‾|_|‾|_|‾|_|‾|_        edges at t = 0, 10, 20, 30 ns
//   sq          0   1   0   1
//   adc_clk    |‾‾‾‾‾‾‾|_______|       CLKPH = 0: rises at t = 0, 20
//   capture     ^       .       ^      always on the EVEN edges (sq == 0)
//
// The ADC samples on the rising edge of its clock and puts the result on the
// bus TOD = 3.5 .. 7 ns later (module manual section 2, item 5). Adding ~2 ns
// of ribbon delay each way, the data at the FPGA pin is guaranteed stable in
// roughly [t+11, t+27.5] ns. Capturing at t = 20 sits near the middle of that
// window with ~9 ns of setup and ~7.5 ns of hold margin - which is why the
// capture edge is the same one that drives the ADC clock high.
//
// CLKPH shifts the forwarded clock in 5 ns steps (half a dds_clk, which is the
// finest an ODDR can place an edge) WITHOUT moving the capture edge, so it
// walks the capture point across the full 20 ns ADC period:
//
//   CLKPH | adc_clk rises at | effective capture delay | verdict
//   ------|------------------|-------------------------|------------------
//     0   |  0 ns            |  20 ns                  | nominal, use this
//     1   |  5 ns            |  15 ns                  | ok, less hold margin
//     2   | 10 ns            |  10 ns                  | data transitions here
//     3   | 15 ns            |   5 ns                  | data transitions here
//
// If a long cable pushes the real window off all four settings, the escape
// hatch is a second clk_wiz output at 100 MHz with a tuned phase, used only for
// the capture flops. That is a phase relationship, not a clock domain crossing,
// so it stays timeable - but try CLKPH first.

module dds_adc_if (
    input  wire        clk,           // dds_clk, 100 MHz
    input  wire        rstn,

    // configuration (from dds_glob_regs)
    input  wire        cfg_en,        // 0 = park the ADC clock low, no capture
    input  wire [1:0]  cfg_clkph,     // forwarded-clock phase shift, x 5 ns
    input  wire        cfg_bitswap,   // 1 = reverse the data bus (see below)
    input  wire        cfg_inv,       // 1 = negate the sample (see below)

    // to the board-level primitives
    output wire        oddr_d1,       // -> ODDR .D1, clocked by clk
    output wire        oddr_d2,       // -> ODDR .D2
    output wire        capt_en,       // -> CE of the IOB input flops

    // from the board-level IOB input flops (already registered on clk)
    input  wire [11:0] adc0_pins,     // adc0_pins[11] must be the ADC's MSB
    input  wire        adc0_otr_pin,
    input  wire [11:0] adc1_pins,
    input  wire        adc1_otr_pin,

    // captured samples, signed and centred, one pulse per ADC period
    output reg  signed [11:0] adc0_smp,
    output reg                adc0_otr,
    output reg  signed [11:0] adc1_smp,
    output reg                adc1_otr,
    output reg                smp_valid      // common to both channels
);

// ------------------------------------------------------- clock / capture gen
reg sq;
always @(posedge clk or negedge rstn) begin
    if (!rstn)        sq <= 1'b0;
    else if (!cfg_en) sq <= 1'b0;
    else              sq <= ~sq;
end

// ODDR slot map over one ADC period (D1 drives the first 5 ns of a dds_clk
// cycle, D2 the second): even cycle = [0,5) [5,10), odd cycle = [10,15) [15,20).
// Each CLKPH column below is just "which slots are high" for a 50 % duty clock
// that rises at CLKPH x 5 ns.
//
//   CLKPH | [0,5) [5,10) [10,15) [15,20) | D1(even) D2(even) D1(odd) D2(odd)
//   ------|-----------------------------|----------------------------------
//     0   |   1     1      0       0     |    1        1        0       0
//     1   |   0     1      1       0     |    0        1        1       0
//     2   |   0     0      1       1     |    0        0        1       1
//     3   |   1     0      0       1     |    1        0        0       1
//
// which collapses to the two muxes below (sq == 0 on even cycles).
reg d1_r, d2_r;
always @(*) begin
    case (cfg_clkph)
        2'd0: begin d1_r = ~sq; d2_r = ~sq; end
        2'd1: begin d1_r =  sq; d2_r = ~sq; end
        2'd2: begin d1_r =  sq; d2_r =  sq; end
        2'd3: begin d1_r = ~sq; d2_r =  sq; end
    endcase
end

// Parked low rather than left floating: an ADC clock stuck high is a needless
// way to leave the part in an undefined pipeline state.
assign oddr_d1 = cfg_en ? d1_r : 1'b0;
assign oddr_d2 = cfg_en ? d2_r : 1'b0;

// One capture per ADC period, on the even edges (see the header).
assign capt_en = cfg_en & ~sq;

// The IOB flops load on the capt_en edge, so their output is settled and
// readable one dds_clk later - that is when smp_valid fires.
reg capt_d;
always @(posedge clk or negedge rstn) begin
    if (!rstn) capt_d <= 1'b0;
    else       capt_d <= capt_en;
end

// ------------------------------------------------------------ data mangling
//
// TWO HARDWARE FACTS, BOTH COUNTERINTUITIVE, BOTH FROM THE MODULE MANUAL:
//
// 1. The bus is numbered backwards. The AD9226 datasheet calls its MSB "BIT1"
//    and its LSB "BIT12", and the module keeps that convention on the header:
//    header AD1 (silk-screened D0) is the MSB, AD12 (silk D11) is the LSB.
//    Wire it per DDS_Constraints.xdc, which maps silk D0 -> adc0_pins[11], and
//    leave BITSWAP = 0. BITSWAP exists only to rescue a board already wired the
//    other way round without recutting traces.
//
// 2. The analog front end inverts. The manual gives D = 2048 - Vin/5 x 2048,
//    so +5 V in produces code 0. INV = 1 (the reset default) undoes that, so a
//    positive sample here means a positive volt at the SMA. Vpp and RMS do not
//    care about the sign, but anything that ever looks at DC or phase does.
function [11:0] bitrev12;
    input [11:0] v;
    integer k;
    begin
        for (k = 0; k < 12; k = k + 1)
            bitrev12[k] = v[11-k];
    end
endfunction

// straight binary -> two's complement: code 2048 (0 V) becomes 0
function signed [11:0] centre;
    input [11:0] v;
    begin
        centre = $signed({~v[11], v[10:0]});
    end
endfunction

// -(-2048) does not fit in 12 bits; saturate rather than wrap to itself
function signed [11:0] neg_sat;
    input signed [11:0] v;
    begin
        neg_sat = (v == -12'sd2048) ? 12'sd2047 : -v;
    end
endfunction

wire [11:0] raw0 = cfg_bitswap ? bitrev12(adc0_pins) : adc0_pins;
wire [11:0] raw1 = cfg_bitswap ? bitrev12(adc1_pins) : adc1_pins;

wire signed [11:0] c0 = centre(raw0);
wire signed [11:0] c1 = centre(raw1);

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        adc0_smp  <= 12'sd0;  adc0_otr <= 1'b0;
        adc1_smp  <= 12'sd0;  adc1_otr <= 1'b0;
        smp_valid <= 1'b0;
    end else begin
        smp_valid <= capt_d;
        if (capt_d) begin
            adc0_smp <= cfg_inv ? neg_sat(c0) : c0;
            adc1_smp <= cfg_inv ? neg_sat(c1) : c1;
            adc0_otr <= adc0_otr_pin;
            adc1_otr <= adc1_otr_pin;
        end
    end
end

endmodule // dds_adc_if
