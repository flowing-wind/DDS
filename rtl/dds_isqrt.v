// dds_isqrt.v - floor(sqrt(x)) for a 24-bit x, one bit per cycle
//
// Classic restoring bit-by-bit square root: 12 cycles, one comparison and one
// subtraction each, no multiplier and no divider. dds_meas needs one result per
// measurement window (256 samples at the very shortest), so 12 cycles of
// latency is free.
//
// The invariant at step i is rem <= 2 x root, which bounds every intermediate:
// with root < 2048 before the last step, rem_ext = 4 x rem + 3 <= 8 x 2047 + 3
// = 16379, so 15 bits of headroom is comfortably enough.

module dds_isqrt (
    input  wire        clk,
    input  wire        rstn,
    input  wire        start,       // 1-cycle; ignored while busy
    input  wire [23:0] x,
    output reg  [11:0] y,           // floor(sqrt(x))
    output reg         done,        // 1-cycle, y valid from this cycle on
    output wire        busy
);

reg [23:0] xs;
reg [12:0] rem;
reg [11:0] root;
reg [3:0]  it;
reg        run;

assign busy = run;

// bring down the next two bits of x, then try to subtract (4 x root + 1)
wire [14:0] rem_ext = {rem, xs[23:22]};
wire [14:0] trial   = {1'b0, root, 2'b01};
wire        fits    = (rem_ext >= trial);
// Subtract at full width and truncate the RESULT, never the operands: rem_ext
// reaches 16379 (14 bits) while both outcomes are bounded by 2 x root <= 8190
// (13 bits), so the narrowing is only safe on this side of the minus sign.
wire [14:0] rem_sub = rem_ext - trial;

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        xs   <= 24'd0;  rem  <= 13'd0;  root <= 12'd0;
        it   <= 4'd0;   run  <= 1'b0;   done <= 1'b0;  y <= 12'd0;
    end else begin
        done <= 1'b0;

        if (!run) begin
            if (start) begin
                xs   <= x;
                rem  <= 13'd0;
                root <= 12'd0;
                it   <= 4'd0;
                run  <= 1'b1;
            end
        end else begin
            xs   <= {xs[21:0], 2'b00};
            rem  <= fits ? rem_sub[12:0] : rem_ext[12:0];
            root <= {root[10:0], fits};

            if (it == 4'd11) begin
                run  <= 1'b0;
                done <= 1'b1;
                y    <= {root[10:0], fits};
            end else begin
                it <= it + 4'd1;
            end
        end
    end
end

endmodule // dds_isqrt
