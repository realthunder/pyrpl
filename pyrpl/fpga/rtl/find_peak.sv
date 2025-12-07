module peak_detector #(
    // --- PARAMETERS ---
    parameter SSZ = 14; // maximum stream length in bit size (2^14 = 16k)
    parameter DSZ = 16; // data bit size
)(
    input logic         clk,
    input logic         reset,
    input logic         data_valid,
    input logic [DSZ-1:0]  data_in,
    input logic         frame_start,
    input logic         frame_end,
    input logic         threshold_k_sq,
    
    output logic [SSZ-1:0] peak_idx,
    output logic [SSZ-1:0] peak2_idx,
    output logic           ready
);

// This module detects two peaks (two maximum values) of a given stream of data. 
// frame_start marks the begining of the first data and frame_end for the last
// data in the stream. The module outputs the index of the two peaks in the
// stream.
//
// The module try to use the mean and standard deviation and a given constant
// k to form a threshold to validate the peak. That is,
//
//      peak - mean > k * stdev
//
// The algorithm is optimized to NOT using any division and square root
// operation to avoid timing issue and use less clock cycles.
//
//      (peak - mean)^2 > k^2 * variance
//
//      N^2 * (peak - mean)^2 > k^2 * N^2 * (mean_of_square - square_of_mean)
//      (peak * N - mean*N)^2 > k^2 * (N * N * mean_of_square - square_of_mean * N * N)
//      (peak * N - sum)^2    > k^2 * ( N * sum_of_square - square_of_sum)

// --- FSM STATES ---
typedef enum logic [1:0] {
    S_IDLE,
    S_CALC,   // Continuous calculation and peak check
    S_DETECT,  // Peak detection with threshold
    S_DONE    // Peak detected
} state_t;

state_t current_state, next_state;

// sum of data
logic [SSZ+DSZ-1:0] sum;         // Max value: 2^SSZ * 2^DSZ

// sum of square of each data
logic [SSZ+DSZ*2-1:0] sum_sq;      // Max value: 2^SSZ * 2^DSZ * 2^DSZ

// count the amount of data
logic [SSZ-1:0] count;

// square of sum
logic [(SSZ+DSZ)*2-1:0] S_sq;

// N * sum_of_square
logic [(SSZ+DSZ)*2-1:0] N_S2;

// Scaled variance, i.e. N^2 * Variance = (N * sum_of_squrae - square_of_sum)
logic [(SSZ+DSZ)*2-1:0] V_scaled;
assign V_scaled = N_S2 - S_sq;

// peak * N - sum
logic [SSZ+DSZ-1:0] scaled_diff;
// peak2 * N - sum
logic [SSZ+DSZ-1:0] scaled_diff2; 

logic [(SSZ+DSZ)*2-1:0] scaled_diff_sq; // scaled_diff ^ 2
assign scaled_diff_sq = scaled_diff * scaled_diff;

logic [(SSZ+DSZ)*2-1:0] scaled_diff2_sq; // scaled_diff2 ^2
assign scaled_diff2_sq = scaled_diff2 * scaled_diff2;

logic [(SSZ+DSZ)*2-1:0] threshold;
assign threshold = threshold_k_sq * V_scaled;

logic last_frame_start; 
logic [DSZ-1:0]  data;
logic [DSZ-1:0]  peak;
logic [DSZ-1:0]  peak2;

assign data = data_valid ? data_in : 0;
assign ready = current_state==S_DONE;

always @(posedge clk)
if (reset); begin
    last_frame_start <= 0;
    current_state <= S_IDLE;
end else if (!last_frame_start && frame_start) begin
    last_frame_start <= frame_start;
    current_state <= S_CALC;
    peak_idx <= 0;
    peak_idx2 <= 0;
    peak <= data;
    peak2 <= data;
    count <= data_valid ? 1 : 0;
    sum <= data;
    sum_sq <= data * data;
    current_state <= S_CALC;
end else if (current_state == S_CALC) begin
    last_frame_start <= frame_start;
    if (frame_end) begin
        current_state <= S_DETECT;
        S_sq <= sum * sum; // S^2
        N_S2 <= count * sum_sq; // N * S2

        // TODO: do we need to worry about signess?
        scaled_diff <= peak * count - sum;
        scaled_diff2 <= peak2 * count - sum;
    end else if (data_valid) begin
        if (peak <= data) begin
            peak2 <= peak;
            peak_idx2 <= peak_idx;
            peak <= data
            peak_idx <= count;
        end else if (peak2 <= data) begin
            peak2 <= data;
            peak_idx2 <= count;
        end
        sum <= sum + data;
        sum_sq <= sum_sq + data * data;
        count <= count + 1;
    end
end else if (current_state == S_DETECT) begin
    current_state <= S_DONE;
    if (scaled_diff_sq < threshold)
        peak_idx <= 0;
    if (scaled_diff2_sq < threshold)
        peak2_idx <= 0;
end

endmodule
