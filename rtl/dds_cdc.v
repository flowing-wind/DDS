// dds_cdc.v - clock domain crossing helpers for the DDS core
//
// dds_cdc_pulse : toggle-based event crossing. Source domain flips a toggle
//                 line; this module produces a single-cycle pulse in the
//                 destination domain for every flip.
// dds_cdc_bit   : plain 2-FF synchronizer for a quasi-static level.

module dds_cdc_pulse (
    input  wire clk,
    input  wire rstn,
    input  wire tgl_in,     // toggle signal from the other clock domain
    output wire pulse       // one destination-clock pulse per toggle
);

reg [2:0] sync;

always @(posedge clk or negedge rstn) begin
    if (!rstn) sync <= 3'b000;
    else       sync <= {sync[1:0], tgl_in};
end

assign pulse = sync[2] ^ sync[1];

endmodule


module dds_cdc_bit (
    input  wire clk,
    input  wire rstn,
    input  wire d,          // level from the other clock domain
    output wire q
);

reg [1:0] sync;

always @(posedge clk or negedge rstn) begin
    if (!rstn) sync <= 2'b00;
    else       sync <= {sync[0], d};
end

assign q = sync[1];

endmodule
