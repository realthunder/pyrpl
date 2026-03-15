// Clock Domain Crossing syncrhonization using gray code
module cdc_sync #(
    parameter WIDTH = 8
)(
    input  logic             clk_in,
    input  logic             rstn_in,
    input  logic             clear_in = 1'b0,
    input  logic [WIDTH-1:0] bin_in,

    input  logic             clk_out,
    input  logic             rstn_out,
    input  logic             clear_out = 1'b0,
    output logic [WIDTH-1:0] bin_out
);

logic [WIDTH-1:0] gray_in;
logic [WIDTH-1:0] gray_sync1;
logic [WIDTH-1:0] gray_sync2;

// Binary to Gray conversion & Register on Source Clock
always_ff @(posedge clk_in) begin
    if (!rstn_in || clear_in) begin
        gray_in <= '0;
    end else begin
        gray_in <= (bin_in >> 1) ^ bin_in;
    end
end

// Double-flop synchronizer on Destination Clock
always_ff @(posedge clk_out) begin
    if (!rstn_out || clear_out) begin
        gray_sync1 <= '0;
        gray_sync2 <= '0;
    end else begin
        gray_sync1 <= gray_in;
        gray_sync2 <= gray_sync1;
    end
end

// Gray to Binary conversion on Destination Clock
always_comb begin
    bin_out[WIDTH-1] = gray_sync2[WIDTH-1];
    for (int i = WIDTH-2; i >= 0; i--) begin
        bin_out[i] = bin_out[i+1] ^ gray_sync2[i];
    end
end

endmodule
