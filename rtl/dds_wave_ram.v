// dds_wave_ram.v - user-defined waveform memory
//
// 4096 x 14-bit true dual-port RAM, independent clocks. Infers block RAM.
//   Port A: APB side (PCLK)   - write samples, read back
//   Port B: DDS side (dds_clk) - playback, read only

module dds_wave_ram (
    // port A (APB / PCLK)
    input  wire        clka,
    input  wire        ena,
    input  wire        wea,
    input  wire [11:0] addra,
    input  wire [13:0] dina,
    output reg  [13:0] douta,
    // port B (DDS / dds_clk)
    input  wire        clkb,
    input  wire        enb,
    input  wire [11:0] addrb,
    output reg  [13:0] doutb
);

reg [13:0] mem [0:4095];

always @(posedge clka) begin
    if (ena) begin
        if (wea)
            mem[addra] <= dina;
        douta <= mem[addra];
    end
end

always @(posedge clkb) begin
    if (enb)
        doutb <= mem[addrb];
end

endmodule
