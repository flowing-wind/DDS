// dds_wave_ram.v - user-defined waveform memory
//
// 4096 x 14-bit simple dual-port RAM, single clock. Infers block RAM.
//   Port A: register bus - write samples, read back
//   Port B: datapath     - playback, read only

module dds_wave_ram (
    input  wire        clk,
    // port A (SPI register bus)
    input  wire        wea,
    input  wire [11:0] addra,
    input  wire [13:0] dina,
    output reg  [13:0] douta,
    // port B (playback)
    input  wire        enb,
    input  wire [11:0] addrb,
    output reg  [13:0] doutb
);

reg [13:0] mem [0:4095];

always @(posedge clk) begin
    if (wea)
        mem[addra] <= dina;
    douta <= mem[addra];
end

always @(posedge clk) begin
    if (enb)
        doutb <= mem[addrb];
end

endmodule
