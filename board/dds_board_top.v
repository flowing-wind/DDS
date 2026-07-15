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

dds_top #(
    .NUM_CH         (2),
    .SIN_LUT_FILE   ("dds_sin_lut.mem")
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

endmodule // dds_board_top
