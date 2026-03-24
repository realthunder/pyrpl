module peak_detector #(
    // --- PARAMETERS ---
    parameter SSZ, // maximum stream length in bit size (2^14 = 16k)
    parameter DSZ  // data bit size
)(
    input logic           clk,
    input logic           resetn,
    input logic           data_valid,
    input logic [DSZ-1:0] data_in,
    input logic [SSZ-1:0] data_index,

    input logic [16-1: 0] threshold_k_sq,

    output logic          maxi_rdy,
    input logic           maxi_valid,
    input logic           maxi_last,
    
    output logic [SSZ-1:0] peak_idx,
    output logic [DSZ-1:0] peak,
    output logic [SSZ-1:0] peak2_idx,
    output logic [DSZ-1:0] peak2,
    output logic [SSZ+DSZ-1:0] sum_o, 
    output logic [SSZ:0]   count_o,
    output logic           ready,
    output logic [8-1:0]   state
);

// This module detects two peaks (two maximum values) of a given stream of data. 
// The module outputs the index of the two highest peaks in the stream.
//
// The module try to use the mean and standard deviation and a given constant
// k to form a threshold to validate the peak. That is,
//
//      peak - mean > k * stdev
//
// The algorithm is optimized to NOT using any division and square root
// operation to use less clock cycles.
//
//                peak - mean  >  k * stdev
//            (peak - mean)^2  >  k^2 * variance
//      N^2 * (peak - mean)^2  >  k^2 * N^2 * (mean_of_square - square_of_mean)
//      (peak * N - mean*N)^2  >  k^2 * (N * N * mean_of_square - square_of_mean * N * N)
//         (peak * N - sum)^2  >  k^2 * ( N * sum_of_square - square_of_sum)

localparam S_IDLE = 0;
localparam S_STREAM = 1;
localparam S_DETECT1 = 2;
localparam S_DETECT2 = S_DETECT1+1;
localparam S_DETECT3 = S_DETECT2+1;
localparam S_DETECT4 = S_DETECT3+1;

logic [8-1: 0] current_state;

logic [SSZ+DSZ-1:0] sum; // sum of all data. Max value: 2^SSZ * 2^DSZ
logic [SSZ:0] count;

logic [DSZ-1:0] data_reg;
logic [DSZ*2-1:0] data_sq_reg;
logic [SSZ-1:0] data_index_reg;
logic data_valid_reg;

// sum of square of each data
logic [SSZ+DSZ*2-1:0] sum_sq;      // Max value: 2^SSZ * 2^DSZ * 2^DSZ

// square of sum
logic [(SSZ+DSZ)*2-1:0] S_sq;
// N * sum_of_square
logic [(SSZ+DSZ)*2-1:0] N_S2;

// Scaled variance, i.e. N^2 * Variance = (N * sum_of_squrae - square_of_sum)
logic [(SSZ+DSZ)*2-1:0] V_scaled;

// peak * N
logic [SSZ+DSZ-1:0] scaled_peak;
// peak2 * N
logic [SSZ+DSZ-1:0] scaled_peak2; 

// scaled_peak - sum
logic [SSZ+DSZ-1:0] scaled_diff;
// scaled_peak2 - sum
logic [SSZ+DSZ-1:0] scaled_diff2;; 

logic [(SSZ+DSZ)*2-1:0] scaled_diff_sq; // scaled_diff ^ 2
logic [(SSZ+DSZ)*2-1:0] scaled_diff2_sq; // scaled_diff2 ^2

logic [(SSZ+DSZ)*2-1:0] threshold;

assign ready = current_state==S_IDLE;

assign state = current_state;

assign maxi_rdy = current_state==S_STREAM;

always @(posedge clk)
if (resetn == 0) begin
    current_state <= S_IDLE;
end else begin
    if (current_state == S_IDLE) begin
        if (maxi_valid) begin
            data_reg <= data_in;
            data_sq_reg <= data_in * data_in;
            data_index_reg <= data_index;
            data_valid_reg <= data_valid;

            peak_idx <= 0;
            peak2_idx <= 0;
            peak <= 0;
            peak2 <= 0;
            count <= 0;
            sum <= 0;
            sum_sq <= 0;
            current_state <= S_STREAM;
        end
    end else if (current_state == S_STREAM) begin
        data_valid_reg <= data_valid & maxi_valid;
        if (maxi_valid) begin
            data_reg <= data_in;
            data_sq_reg <= data_in * data_in;
            data_index_reg <= data_index;

            if (maxi_last)
                current_state <= S_DETECT1;
        end
        if (data_valid_reg) begin
            if (peak <= data_reg) begin
                peak2 <= peak;
                peak2_idx <= peak_idx;
                peak <= data_reg;
                peak_idx <= data_index_reg;
            end else if (peak2 <= data_reg) begin
                peak2 <= data_reg;
                peak2_idx <= data_index_reg;
            end
            sum <= sum + data_reg;
            sum_sq <= sum_sq + data_sq_reg;
            count <= count + 1;
        end

    end else if (current_state >= S_DETECT1 && current_state < S_DETECT2) begin
        S_sq <= sum * sum; // S^2
        N_S2 <= count * sum_sq; // N * S2

        scaled_peak <= peak * count;
        scaled_peak2 <= peak2 * count;

        current_state <= current_state + 1;

    end else if (current_state >= S_DETECT2 && current_state < S_DETECT3) begin

        // TODO: do we need to worry about signess?
        scaled_diff <= scaled_peak - sum;
        scaled_diff2 <= scaled_peak2 - sum;
        V_scaled <= N_S2 - S_sq;

        current_state <= current_state + 1;

    end else if (current_state >= S_DETECT3 && current_state < S_DETECT4) begin
        threshold <= threshold_k_sq * V_scaled;
        scaled_diff_sq <= scaled_diff * scaled_diff;
        scaled_diff2_sq <= scaled_diff2 * scaled_diff2;

        current_state <= current_state + 1;

    end else if (current_state == S_DETECT4) begin
        if (scaled_diff_sq < threshold) begin
            peak_idx <= 0;
            peak2_idx <= 0;
        end else if (scaled_diff2_sq < threshold)
            peak2_idx <= 0;
        sum_o <= sum;
        count_o <= count;
        current_state <= S_IDLE;
    end
end

endmodule
