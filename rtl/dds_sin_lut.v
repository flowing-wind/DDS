// dds_sin_lut.v - quarter-wave sine lookup table
//
// 4096 x 13-bit ROM holding sin((i+0.5)/4096 * 90deg) * 8191, i = 0..4095.
// Synchronous read; infers block RAM. The init file must be visible to the
// tool (add dds_sin_lut.mem to the Vivado project as a design source, or
// override MEM_FILE with an absolute path).

module dds_sin_lut #(
    parameter MEM_FILE = "dds_sin_lut.mem"
) (
    input  wire        clk,
    input  wire        en,
    input  wire [11:0] addr,
    output reg  [12:0] dout
);

reg [12:0] mem [0:4095];

initial $readmemh(MEM_FILE, mem);

always @(posedge clk) begin
    if (en)
        dout <= mem[addr];
end

endmodule
