// dds_top.v - DDS IP core, top level
//
// One 16-bit SPI slave port (mode 0) driving one or two independent DDS
// channels, each with its own 14-bit parallel DAC output. Instantiate this.
//
// Everything runs in the dds_clk domain, including the SPI front end (the SPI
// pins are oversampled). There is exactly one clock and one reset - no CDC to
// constrain, no second bus clock. The only requirement is
//
//     f_sck <= dds_clk / 6
//
// which a 9 MHz STM32 SPI against a 100 MHz dds_clk satisfies with 11x margin.
//
// Channel select is CMD[14] of the SPI command word, so both channels share a
// single chip select. See Doc/DDS_Datasheet.md for the frame format and the
// register map.

module dds_top #(
    parameter NUM_CH       = 2,                  // 1 or 2
    parameter SIN_LUT_FILE = "dds_sin_lut.mem"
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

    // interrupt request, one bit per channel
    output wire [1:0]  dds_irq
);

wire        bus_ch;
wire [13:0] bus_addr;
wire [15:0] bus_wdata;
wire        bus_we;
wire        bus_frame_end;
wire [15:0] bus_rdata;

wire [15:0] rdata_ch0, rdata_ch1;

assign bus_rdata = (bus_ch && (NUM_CH > 1)) ? rdata_ch1 : rdata_ch0;

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

// Writes and the frame-end commit are steered by the channel bit; reads are
// muxed above. A frame never addresses both channels.
dds_channel #(
    .SIN_LUT_FILE  (SIN_LUT_FILE)
) u_ch0 (
    .clk           (dds_clk),
    .rstn          (dds_rstn),

    .bus_addr      (bus_addr),
    .bus_wdata     (bus_wdata),
    .bus_we        (bus_we        && !bus_ch),
    .bus_frame_end (bus_frame_end && !bus_ch),
    .bus_rdata     (rdata_ch0),

    .sweep_trig    (ch0_sweep_trig),
    .hop_sel       (ch0_hop_sel),

    .dac_data      (ch0_dac_data),
    .dac_valid     (ch0_dac_valid),
    .dds_irq       (dds_irq[0])
);

generate
if (NUM_CH > 1) begin : g_ch1
    dds_channel #(
        .SIN_LUT_FILE  (SIN_LUT_FILE)
    ) u_ch1 (
        .clk           (dds_clk),
        .rstn          (dds_rstn),

        .bus_addr      (bus_addr),
        .bus_wdata     (bus_wdata),
        .bus_we        (bus_we        && bus_ch),
        .bus_frame_end (bus_frame_end && bus_ch),
        .bus_rdata     (rdata_ch1),

        .sweep_trig    (ch1_sweep_trig),
        .hop_sel       (ch1_hop_sel),

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
