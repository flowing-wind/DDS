// dds_spi_slave.v - SPI slave front end for the DDS IP
//
// SPI mode 0 (CPOL=0, CPHA=0), MSB first, 16-bit words, CS active low.
// Matches an STM32 SPI master configured for DATASIZE_16BIT / POLARITY_LOW /
// PHASE_1EDGE with a GPIO-driven chip select.
//
// Frame format
//   CS low -> [CMD] [D0] [D1] ... -> CS high
//
//   CMD[15]    RW    1 = write, 0 = read
//   CMD[14]    CH    channel select (0 / 1)
//   CMD[13:0]  ADDR  word address, auto-increments after every data word
//
//   write:  D0 -> ADDR, D1 -> ADDR+1, ...
//   read:   the word right after CMD is a turnaround word (MISO = 0x0000);
//           read data appears from the third word onward. The address pointer
//           runs one word ahead of MISO, so the register / BRAM read always
//           has a full word time (16 SCK) to settle.
//
//   The rising edge of CS emits frame_end, which the register file uses as
//   the atomic commit point for the whole frame.
//
// SCK, CS and MOSI are oversampled in the clk domain - no SCK-driven flops,
// no BUFG on a slow external clock. This requires
//
//     f_sck <= f_clk / 6
//
// (9 MHz SCK against a 100 MHz clk gives 11x oversampling, plenty of margin).
// MISO is updated right after the rising edge that completes a word, i.e. a
// full half SCK period before the master samples it.

module dds_spi_slave (
    input  wire        clk,
    input  wire        rstn,

    // SPI pins (asynchronous)
    input  wire        spi_sck,
    input  wire        spi_cs_n,
    input  wire        spi_mosi,
    output wire        spi_miso,
    output wire        spi_miso_oe,   // 1 while this slave drives MISO

    // internal register bus (clk domain)
    output reg         bus_ch,        // channel select, stable during a frame
    output reg  [13:0] bus_addr,      // write address when bus_we, else read address
    output reg  [15:0] bus_wdata,
    output reg         bus_we,        // 1-cycle write strobe
    output reg         bus_frame_end, // 1-cycle pulse on CS rising edge
    input  wire [15:0] bus_rdata      // from the channel selected by bus_ch
);

// ------------------------------------------------------------ input sync
reg [2:0] sck_s, cs_s;
reg [1:0] mosi_s;

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        sck_s  <= 3'b000;
        cs_s   <= 3'b111;
        mosi_s <= 2'b00;
    end else begin
        sck_s  <= {sck_s[1:0], spi_sck};
        cs_s   <= {cs_s[1:0],  spi_cs_n};
        mosi_s <= {mosi_s[0],  spi_mosi};
    end
end

wire sck_rise = sck_s[1] & ~sck_s[2];
wire sck_fall = ~sck_s[1] & sck_s[2];
wire cs_act   = ~cs_s[1];                 // frame in progress
wire cs_rise  = cs_s[1] & ~cs_s[2];       // frame end
wire mosi     = mosi_s[1];

// ------------------------------------------------------------- shift regs
reg [15:0] sh_in, sh_out;
reg [3:0]  bitcnt;
reg [13:0] ptr;             // address pointer, one word ahead of MISO on reads
reg [15:0] rd_hold;         // registered bus_rdata
reg        have_cmd, rw;

wire [15:0] word_in  = {sh_in[14:0], mosi};       // word completing this cycle
wire        word_end = sck_rise && (bitcnt == 4'd15);

assign spi_miso    = sh_out[15];
assign spi_miso_oe = cs_act;

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        sh_in         <= 16'h0;
        sh_out        <= 16'h0;
        bitcnt        <= 4'd0;
        ptr           <= 14'h0;
        rd_hold       <= 16'h0;
        have_cmd      <= 1'b0;
        rw            <= 1'b0;
        bus_ch        <= 1'b0;
        bus_addr      <= 14'h0;
        bus_wdata     <= 16'h0;
        bus_we        <= 1'b0;
        bus_frame_end <= 1'b0;
    end else begin
        bus_we        <= 1'b0;
        bus_frame_end <= 1'b0;

        // bus_addr only moves on word boundaries, so the read data is stable
        // long before MISO needs it
        rd_hold <= bus_rdata;

        if (!cs_act) begin
            bitcnt   <= 4'd0;
            have_cmd <= 1'b0;
            sh_out   <= 16'h0;             // MISO reads 0 during the CMD word
            if (cs_rise)
                bus_frame_end <= 1'b1;
        end else begin
            if (sck_rise) begin
                sh_in  <= {sh_in[14:0], mosi};
                bitcnt <= bitcnt + 4'd1;   // wraps to 0 on the 16th bit
            end

            // MISO shifts on falling edges, but holds through the edge right
            // after a word boundary (bitcnt == 0) where it was just loaded
            if (sck_fall && (bitcnt != 4'd0))
                sh_out <= {sh_out[14:0], 1'b0};

            if (word_end) begin
                if (!have_cmd) begin
                    have_cmd <= 1'b1;
                    rw       <= word_in[15];
                    bus_ch   <= word_in[14];
                    ptr      <= word_in[13:0];
                    bus_addr <= word_in[13:0];   // start the read prefetch
                    sh_out   <= 16'h0;           // turnaround word
                end else if (rw) begin
                    bus_we    <= 1'b1;
                    bus_wdata <= word_in;
                    bus_addr  <= ptr;            // pre-increment value
                    ptr       <= ptr + 14'd1;
                end else begin
                    sh_out    <= rd_hold;        // data fetched a word ago
                    ptr       <= ptr + 14'd1;
                    bus_addr  <= ptr + 14'd1;    // prefetch the next one
                end
            end
        end
    end
end

endmodule // dds_spi_slave
