// dds_top.v - DDS IP core, top level
//
// One 16-bit SPI slave port (mode 0) driving one or two independent DDS
// channels, each with its own 14-bit parallel DAC output, plus an optional
// dual AD9226 capture front end and one amplitude control loop per channel.
// Instantiate this.
//
// Everything runs in the dds_clk domain, including the SPI front end (the SPI
// pins are oversampled) and the ADC capture (the ADC clock is dds_clk / 2,
// generated here as an ODDR bit pattern). There is exactly one clock and one
// reset - no CDC to constrain, no second bus clock. The only requirement is
//
//     f_sck <= dds_clk / 6
//
// which a 9 MHz STM32 SPI against a 100 MHz dds_clk satisfies with 11x margin.
//
// Channel select is CMD[14] of the SPI command word, so both channels share a
// single chip select. See Doc/DDS_Datasheet.md for the frame format and the
// register map.
//
// ---------------------------------------------------------- resource routing
//
// The DACs, the ADCs and the loops are deliberately NOT hard-wired to each
// other, because on a real bench they get mixed and matched:
//
//   which ADC feeds a channel's loop   -> AGC_CTRL.SRC register, per channel
//   loop on / off                      -> AGC_CTRL.EN register, per channel
//   ADC data for other fabric modules  -> adcN_smp / adc_smp_valid outputs
//                                         below, always live, loop or no loop
//   DAC driven by another module       -> mux chN_dac_data outside this core;
//                                         the DDS cannot see that, so leave
//                                         that channel's loop off
//
// HAS_ADC = 0 removes the ADC front end, both loops and their registers
// (GLOB_ID then reads 0), returning the core to its pre-ADC footprint.

module dds_top #(
    parameter NUM_CH       = 2,                  // 1 or 2
    parameter SIN_LUT_FILE = "dds_sin_lut.mem",
    parameter HAS_ADC      = 1
) (
    input  wire        dds_clk,      // DAC sample clock, e.g. 100 MHz
    input  wire        dds_rstn,     // active-low, synchronous release recommended

    // SPI slave (mode 0, 16-bit words, CS active low)
    input  wire        spi_sck,
    input  wire        spi_cs_n,
    input  wire        spi_mosi,
    output wire        spi_miso,
    output wire        spi_miso_oe,  // for a tri-stated / shared MISO net

    // hardware pacing hooks (asynchronous pins or on-chip logic)
    input  wire        ch0_sweep_trig,
    input  wire [1:0]  ch0_hop_sel,
    input  wire        ch1_sweep_trig,
    input  wire [1:0]  ch1_hop_sel,

    // DAC interfaces (dds_clk domain)
    output wire [13:0] ch0_dac_data,
    output wire        ch0_dac_valid,
    output wire [13:0] ch1_dac_data,
    output wire        ch1_dac_valid,

    // AD9226 front end: the board top forwards adc_clk_d1/d2 through an ODDR
    // as the shared ADC clock, and registers the returning pins in IOB flops
    // clock-enabled by adc_capt_en. See board/dds_board_top.v.
    output wire        adc_clk_d1,
    output wire        adc_clk_d2,
    output wire        adc_capt_en,
    input  wire [11:0] adc0_data,    // adc0_data[11] = ADC MSB (silk D0!)
    input  wire        adc0_otr,
    input  wire [11:0] adc1_data,
    input  wire        adc1_otr,

    // captured ADC streams, exported for any other fabric consumer (filters,
    // scope capture, a demodulator...). Signed, centred, sign-corrected;
    // one adc_smp_valid pulse per ADC sample period, common to both.
    output wire signed [11:0] adc0_smp,
    output wire               adc0_smp_otr,
    output wire signed [11:0] adc1_smp,
    output wire               adc1_smp_otr,
    output wire               adc_smp_valid,

    // interrupt request, one bit per channel
    output wire [1:0]  dds_irq
);

wire        bus_ch;
wire [13:0] bus_addr;
wire [15:0] bus_wdata;
wire        bus_we;
wire        bus_frame_end;
wire [15:0] bus_rdata;

wire [15:0] rdata_ch0, rdata_ch1, rdata_glob;

// ------------------------------------------------------------- block decode
// 0x80..0xBF is the global block. It is channel-agnostic: the ADC front end
// is one piece of hardware shared by both channels, so CMD.CH is ignored here
// and the same registers answer either way.
wire glob_sel = HAS_ADC && !bus_addr[13] && (bus_addr[12:6] == 7'h02);

reg glob_sel_d;
always @(posedge dds_clk or negedge dds_rstn) begin
    if (!dds_rstn) glob_sel_d <= 1'b0;
    else           glob_sel_d <= glob_sel;
end

assign bus_rdata = glob_sel_d ? rdata_glob :
                   (bus_ch && (NUM_CH > 1)) ? rdata_ch1 : rdata_ch0;

dds_spi_slave u_spi (
    .clk           (dds_clk),
    .rstn          (dds_rstn),

    .spi_sck       (spi_sck),
    .spi_cs_n      (spi_cs_n),
    .spi_mosi      (spi_mosi),
    .spi_miso      (spi_miso),
    .spi_miso_oe   (spi_miso_oe),

    .bus_ch        (bus_ch),
    .bus_addr      (bus_addr),
    .bus_wdata     (bus_wdata),
    .bus_we        (bus_we),
    .bus_frame_end (bus_frame_end),
    .bus_rdata     (bus_rdata)
);

// ---------------------------------------------------------- ADC front end
generate
if (HAS_ADC) begin : g_adc

    wire adc_en, adc_bitswap, adc_inv;
    wire [1:0] adc_clkph;

    dds_adc_if u_adc_if (
        .clk          (dds_clk),
        .rstn         (dds_rstn),
        .cfg_en       (adc_en),
        .cfg_clkph    (adc_clkph),
        .cfg_bitswap  (adc_bitswap),
        .cfg_inv      (adc_inv),
        .oddr_d1      (adc_clk_d1),
        .oddr_d2      (adc_clk_d2),
        .capt_en      (adc_capt_en),
        .adc0_pins    (adc0_data),
        .adc0_otr_pin (adc0_otr),
        .adc1_pins    (adc1_data),
        .adc1_otr_pin (adc1_otr),
        .adc0_smp     (adc0_smp),
        .adc0_otr     (adc0_smp_otr),
        .adc1_smp     (adc1_smp),
        .adc1_otr     (adc1_smp_otr),
        .smp_valid    (adc_smp_valid)
    );

    dds_glob_regs #(
        .HAS_ADC   (1)
    ) u_glob (
        .clk             (dds_clk),
        .rstn            (dds_rstn),
        .bus_idx         (bus_addr[5:0]),
        .bus_wdata       (bus_wdata),
        .bus_we          (bus_we && glob_sel),
        .bus_rdata       (rdata_glob),
        .cfg_adc_en      (adc_en),
        .cfg_adc_clkph   (adc_clkph),
        .cfg_adc_bitswap (adc_bitswap),
        .cfg_adc_inv     (adc_inv),
        .adc0_smp        (adc0_smp),
        .adc1_smp        (adc1_smp)
    );

end else begin : g_no_adc
    assign adc_clk_d1    = 1'b0;
    assign adc_clk_d2    = 1'b0;
    assign adc_capt_en   = 1'b0;
    assign adc0_smp      = 12'sd0;
    assign adc0_smp_otr  = 1'b0;
    assign adc1_smp      = 12'sd0;
    assign adc1_smp_otr  = 1'b0;
    assign adc_smp_valid = 1'b0;
    assign rdata_glob    = 16'h0;
end
endgenerate

// -------------------------------------------------------------- the channels
// Writes and the frame-end commit are steered by the channel bit; reads are
// muxed above. A frame never addresses both channels. The global block wins
// the write decode, so a global write must not also land in a channel - hence
// the !glob_sel below.
dds_channel #(
    .SIN_LUT_FILE  (SIN_LUT_FILE),
    .HAS_AGC       (HAS_ADC)
) u_ch0 (
    .clk           (dds_clk),
    .rstn          (dds_rstn),

    .bus_addr      (bus_addr),
    .bus_wdata     (bus_wdata),
    .bus_we        (bus_we        && !bus_ch && !glob_sel),
    .bus_frame_end (bus_frame_end && !bus_ch),
    .bus_rdata     (rdata_ch0),

    .sweep_trig    (ch0_sweep_trig),
    .hop_sel       (ch0_hop_sel),

    .adc0_smp      (adc0_smp),
    .adc0_otr      (adc0_smp_otr),
    .adc1_smp      (adc1_smp),
    .adc1_otr      (adc1_smp_otr),
    .adc_smp_valid (adc_smp_valid),

    .dac_data      (ch0_dac_data),
    .dac_valid     (ch0_dac_valid),
    .dds_irq       (dds_irq[0])
);

generate
if (NUM_CH > 1) begin : g_ch1
    dds_channel #(
        .SIN_LUT_FILE  (SIN_LUT_FILE),
        .HAS_AGC       (HAS_ADC)
    ) u_ch1 (
        .clk           (dds_clk),
        .rstn          (dds_rstn),

        .bus_addr      (bus_addr),
        .bus_wdata     (bus_wdata),
        .bus_we        (bus_we        && bus_ch && !glob_sel),
        .bus_frame_end (bus_frame_end && bus_ch),
        .bus_rdata     (rdata_ch1),

        .sweep_trig    (ch1_sweep_trig),
        .hop_sel       (ch1_hop_sel),

        .adc0_smp      (adc0_smp),
        .adc0_otr      (adc0_smp_otr),
        .adc1_smp      (adc1_smp),
        .adc1_otr      (adc1_smp_otr),
        .adc_smp_valid (adc_smp_valid),

        .dac_data      (ch1_dac_data),
        .dac_valid     (ch1_dac_valid),
        .dds_irq       (dds_irq[1])
    );
end else begin : g_no_ch1
    assign rdata_ch1     = 16'h0;
    assign ch1_dac_data  = 14'h0;
    assign ch1_dac_valid = 1'b0;
    assign dds_irq[1]    = 1'b0;
end
endgenerate

endmodule // dds_top
