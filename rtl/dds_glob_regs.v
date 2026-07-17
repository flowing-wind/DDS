// dds_glob_regs.v - channel-independent registers (bus block 2, 0x80..0xBF)
//
// Everything else in this IP is per-channel, and the SPI command word carries a
// channel bit to match. The ADC front end is the one thing that genuinely is
// not: there is a single capture timing generator feeding both ADCs, and asking
// "which DDS channel does the ADC clock phase belong to" has no answer. So this
// block ignores CMD[14] entirely - it reads and writes the same registers from
// either channel.
//
//   0x80 GLOB_ID  (RO)  0x4147 "AG" when the ADC/AGC option is built in,
//                       0x0000 when it is compiled out. Probe this to find out
//                       whether the bitstream in front of you has the loop.
//   0x81 ADC_CFG        bit0    EN       1 = run the ADC clock (reset: 1)
//                       bit5:4  CLKPH    forwarded-clock phase, x 5 ns (reset: 0)
//                       bit8    BITSWAP  1 = reverse the data bus (reset: 0)
//                       bit9    INV      1 = negate the sample (reset: 1)
//   0x82 ADC0_RAW (RO)  live ADC0 sample, sign-extended
//   0x83 ADC1_RAW (RO)  live ADC1 sample, sign-extended
//
// ADCn_RAW is the bring-up window into the analog chain: ground the input and
// it should read ~0, apply a known DC and it gives you the volts-per-code
// constant directly. It is a raw live sample with no averaging, so it will
// jitter by a few codes - that is the ADC's noise floor, not a fault.

module dds_glob_regs #(
    parameter HAS_ADC = 1
) (
    input  wire        clk,
    input  wire        rstn,

    // internal bus (already decoded for this block; NOT channel-decoded)
    input  wire [5:0]  bus_idx,       // bus_addr[5:0]
    input  wire [15:0] bus_wdata,
    input  wire        bus_we,
    output reg  [15:0] bus_rdata,

    // configuration out
    output wire        cfg_adc_en,
    output wire [1:0]  cfg_adc_clkph,
    output wire        cfg_adc_bitswap,
    output wire        cfg_adc_inv,

    // status in
    input  wire signed [11:0] adc0_smp,
    input  wire signed [11:0] adc1_smp
);

localparam A_ID = 6'h00, A_ADC_CFG = 6'h01, A_ADC0 = 6'h02, A_ADC1 = 6'h03;

localparam [15:0] GLOB_ID = HAS_ADC ? 16'h4147 : 16'h0000;

reg       r_en, r_bitswap, r_inv;
reg [1:0] r_clkph;

assign cfg_adc_en      = r_en;
assign cfg_adc_clkph   = r_clkph;
assign cfg_adc_bitswap = r_bitswap;
assign cfg_adc_inv     = r_inv;

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        r_en      <= 1'b1;
        r_clkph   <= 2'd0;      // 20 ns capture delay - see dds_adc_if.v
        r_bitswap <= 1'b0;      // the XDC already un-reverses the bus
        r_inv     <= 1'b1;      // the AD9226 module's front end inverts
    end else if (bus_we && (bus_idx == A_ADC_CFG)) begin
        r_en      <= bus_wdata[0];
        r_clkph   <= bus_wdata[5:4];
        r_bitswap <= bus_wdata[8];
        r_inv     <= bus_wdata[9];
    end
end

reg [15:0] rd_c;
always @(*) begin
    case (bus_idx)
        A_ID:      rd_c = GLOB_ID;
        A_ADC_CFG: rd_c = {6'h0, r_inv, r_bitswap, 2'b00, r_clkph, 3'b000, r_en};
        A_ADC0:    rd_c = {{4{adc0_smp[11]}}, adc0_smp};
        A_ADC1:    rd_c = {{4{adc1_smp[11]}}, adc1_smp};
        default:   rd_c = 16'h0;
    endcase
end

always @(posedge clk or negedge rstn) begin
    if (!rstn) bus_rdata <= 16'h0;
    else       bus_rdata <= rd_c;
end

endmodule // dds_glob_regs
