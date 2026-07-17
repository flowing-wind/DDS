// dds_board_top.v - board-level top for the DDS on a Zynq-7020 (PL only)
//
// dds_top is the IP core; it has no PLL, no clock forwarding and no I/O
// primitives on purpose, so it stays board-independent and simulatable. This
// wrapper adds the three things a real board needs:
//
//   1. PLL  : board oscillator -> 100 MHz dds_clk   (Vivado clk_wiz IP)
//   2. reset: asynchronous assert, synchronous release, held until PLL lock
//   3. DAC  : output data registered in the IOBs, and the DAC clock forwarded
//             through an ODDR so the AD9764 sees its rising edge half a cycle
//             AFTER the data changes (~5 ns setup / 5 ns hold at 100 MHz).
//             See Doc/Application Note/AN01, section 4.
//
// This is a PL-only design: no PS7 block, no block design. Vivado will warn
// that the Zynq PS is unused - that is expected and harmless; the bitstream
// loads over JTAG and runs on its own.
//
// The clk_wiz_0 instance below matches the generated IP's port list. What the
// port names cannot tell you, and what you must get right in the IP config:
// clk_in1 = the board oscillator frequency, clk_out1 = 100.000 MHz.

module dds_board_top #(
    // The board's PL oscillator. Only used for documentation here - the actual
    // input frequency is baked into the clk_wiz IP when you generate it.
    parameter SYS_CLK_HZ  = 50_000_000,
    // 1 = the external reset button is active low (typical), 0 = active high.
    parameter EXT_RST_LOW = 1
) (
    // clock and reset from the board
    input  wire        sys_clk,        // PL-side oscillator
    input  wire        ext_rst,        // reset button; tie to the inactive level if unused

    // SPI slave, to the STM32F103 (mode 0, 16-bit words)
    input  wire        spi_sck,        // <- PA5
    input  wire        spi_cs_n,       // <- PA4 (GPIO software chip select)
    input  wire        spi_mosi,       // <- PA7
    output wire        spi_miso,       // -> PA6

    // AD9764 module, channel 0 (U3) and channel 1 (U6)
    output wire [13:0] dac0_data,      // -> DB13..DB0
    output wire        dac0_clk,       // -> CLKA
    output wire [13:0] dac1_data,      // -> DB13..DB0
    output wire        dac1_clk,       // -> CLKB

    // AD9226 module, both channels. BEWARE THE SILK SCREEN: the module numbers
    // its bus backwards, header silk D0 is the ADC's MSB. The XDC does the
    // un-reversing, so adc0_data[11] here = silk D0 = MSB. See AN02.
    // ACLK and BCLK are separate header pins, so each gets its own output
    // port and its own ODDR - both play the same dds_clk/2 pattern, so the
    // two ADCs sample on the same edge.
    output wire        adc0_clk,       // -> ACLK, 50 MHz
    output wire        adc1_clk,       // -> BCLK, 50 MHz
    input  wire [11:0] adc0_data,      // <- AD1 channel
    input  wire        adc0_otr,       // <- ATR (over-range; optional, tie 0)
    input  wire [11:0] adc1_data,      // <- AD2 channel
    input  wire        adc1_otr,       // <- BTR

    // optional: one LED per channel, lit while that channel's interrupt is set
    output wire [1:0]  led
);

// ---------------------------------------------------------------- clocking
wire dds_clk;       // 100 MHz, the only clock in the design
wire pll_locked;

wire ext_rst_n = EXT_RST_LOW ? ext_rst : ~ext_rst;

// Vivado clk_wiz: sys_clk -> 100 MHz. Configure it for your board's oscillator
// frequency, one output at 100.000 MHz, and an active-low "resetn" input.
clk_wiz_0 u_pll (
    .clk_in1  (sys_clk),
    .resetn   (ext_rst_n),
    .clk_out1 (dds_clk),
    .locked   (pll_locked)
);

// ------------------------------------------------------------------- reset
// Assert asynchronously (the moment the PLL drops lock or the button is
// pressed), release synchronously to dds_clk two flops later. Releasing a
// reset asynchronously is how you get one half of the fabric coming out of
// reset a cycle before the other.
wire rst_src_n = pll_locked & ext_rst_n;

(* ASYNC_REG = "TRUE" *) reg [1:0] rst_sync;
always @(posedge dds_clk or negedge rst_src_n) begin
    if (!rst_src_n) rst_sync <= 2'b00;
    else            rst_sync <= {rst_sync[0], 1'b1};
end

wire dds_rstn = rst_sync[1];

// --------------------------------------------------------------- the DDS IP
wire [13:0] ch0_dac_data, ch1_dac_data;
wire [1:0]  dds_irq;
wire        adc_clk_d1, adc_clk_d2, adc_capt_en;
wire [11:0] adc0_data_r, adc1_data_r;
wire        adc0_otr_r,  adc1_otr_r;

dds_top #(
    .NUM_CH         (2),
    .SIN_LUT_FILE   ("dds_sin_lut.mem"),
    .HAS_ADC        (1)
) u_dds (
    .dds_clk        (dds_clk),
    .dds_rstn       (dds_rstn),

    .spi_sck        (spi_sck),
    .spi_cs_n       (spi_cs_n),
    .spi_mosi       (spi_mosi),
    .spi_miso       (spi_miso),
    .spi_miso_oe    (),               // MISO is a dedicated pin, never tri-stated

    // hardware pacing pins are unused on this board; sweeps are started over
    // SPI and profiles are selected by register
    .ch0_sweep_trig (1'b0),
    .ch0_hop_sel    (2'b00),
    .ch1_sweep_trig (1'b0),
    .ch1_hop_sel    (2'b00),

    .ch0_dac_data   (ch0_dac_data),
    .ch0_dac_valid  (),
    .ch1_dac_data   (ch1_dac_data),
    .ch1_dac_valid  (),

    .adc_clk_d1     (adc_clk_d1),
    .adc_clk_d2     (adc_clk_d2),
    .adc_capt_en    (adc_capt_en),
    .adc0_data      (adc0_data_r),
    .adc0_otr       (adc0_otr_r),
    .adc1_data      (adc1_data_r),
    .adc1_otr       (adc1_otr_r),

    // the captured streams are also available here for other fabric modules
    .adc0_smp       (),
    .adc0_smp_otr   (),
    .adc1_smp       (),
    .adc1_smp_otr   (),
    .adc_smp_valid  (),

    .dds_irq        (dds_irq)
);

assign led = dds_irq;

// ------------------------------------------------------- DAC output timing
// Data leaves through flops packed into the IOBs, so every bit of the bus has
// the same clock-to-out delay. The IOB attribute is what forbids the tool from
// leaving these in the fabric (where routing skew between bits would eat the
// DAC's setup window).
(* IOB = "TRUE" *) reg [13:0] dac0_data_r;
(* IOB = "TRUE" *) reg [13:0] dac1_data_r;

always @(posedge dds_clk) begin
    dac0_data_r <= ch0_dac_data;
    dac1_data_r <= ch1_dac_data;
end

assign dac0_data = dac0_data_r;
assign dac1_data = dac1_data_r;

// The AD9764 latches on the RISING edge of its clock and needs >= 2 ns setup.
// Forwarding an INVERTED copy of dds_clk (D1 = 0, D2 = 1) puts that rising
// edge half a period (5 ns at 100 MHz) after the data changed - setup and hold
// both ~5 ns, with room for board and cable skew.
//
// Forwarding through an ODDR rather than routing dds_clk to a pin directly is
// not optional: a clock that leaves the fabric on ordinary logic routing
// arrives with an unpredictable delay relative to the data.
ODDR #(
    .DDR_CLK_EDGE ("SAME_EDGE"),
    .INIT         (1'b0),
    .SRTYPE       ("SYNC")
) u_oddr_dac0 (
    .Q  (dac0_clk),
    .C  (dds_clk),
    .CE (1'b1),
    .D1 (1'b0),        // driven on the rising edge of dds_clk
    .D2 (1'b1),        // driven on the falling edge
    .R  (1'b0),
    .S  (1'b0)
);

ODDR #(
    .DDR_CLK_EDGE ("SAME_EDGE"),
    .INIT         (1'b0),
    .SRTYPE       ("SYNC")
) u_oddr_dac1 (
    .Q  (dac1_clk),
    .C  (dds_clk),
    .CE (1'b1),
    .D1 (1'b0),
    .D2 (1'b1),
    .R  (1'b0),
    .S  (1'b0)
);

// -------------------------------------------------------- ADC clock and data
// The AD9226s run at 50 MSPS from a clock synthesized as an ODDR bit pattern:
// dds_adc_if computes which half-cycles are high (that is how its CLKPH phase
// knob works) and these ODDRs merely play the pattern out. Same reasoning as
// the DAC clocks - a clock must leave the fabric through a clock-capable
// output path, not ordinary routing. ACLK and BCLK are separate pins on the
// module, so each gets its own ODDR; both are fed the same D1/D2 pattern and
// launch from the same dds_clk edge, which is what keeps the two ADCs
// sampling in lockstep (and lets one CLKPH setting time both).
ODDR #(
    .DDR_CLK_EDGE ("SAME_EDGE"),
    .INIT         (1'b0),
    .SRTYPE       ("SYNC")
) u_oddr_adc0 (
    .Q  (adc0_clk),
    .C  (dds_clk),
    .CE (1'b1),
    .D1 (adc_clk_d1),
    .D2 (adc_clk_d2),
    .R  (1'b0),
    .S  (1'b0)
);

ODDR #(
    .DDR_CLK_EDGE ("SAME_EDGE"),
    .INIT         (1'b0),
    .SRTYPE       ("SYNC")
) u_oddr_adc1 (
    .Q  (adc1_clk),
    .C  (dds_clk),
    .CE (1'b1),
    .D1 (adc_clk_d1),
    .D2 (adc_clk_d2),
    .R  (1'b0),
    .S  (1'b0)
);

// Returning ADC data lands in IOB flops, same argument as the DAC outputs but
// in reverse: capturing at the pad gives every bit an identical, minimal input
// delay, so the 12 bits cannot skew apart in fabric routing. The clock enable
// makes them sample once per ADC period, on the edge dds_adc_if picked to sit
// mid-window (its header derives the margins).
(* IOB = "TRUE" *) reg [11:0] adc0_capt;
(* IOB = "TRUE" *) reg [11:0] adc1_capt;
(* IOB = "TRUE" *) reg        adc0_otr_capt, adc1_otr_capt;

always @(posedge dds_clk) begin
    if (adc_capt_en) begin
        adc0_capt     <= adc0_data;
        adc1_capt     <= adc1_data;
        adc0_otr_capt <= adc0_otr;
        adc1_otr_capt <= adc1_otr;
    end
end

assign adc0_data_r = adc0_capt;
assign adc1_data_r = adc1_capt;
assign adc0_otr_r  = adc0_otr_capt;
assign adc1_otr_r  = adc1_otr_capt;

endmodule // dds_board_top
