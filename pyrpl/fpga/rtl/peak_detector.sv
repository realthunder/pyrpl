module peak_detector #(
    // --- PARAMETERS ---
    parameter SSZ = 14, // maximum stream length in bit size (2^14 = 16k)
    parameter DSZ = 28  // data bit size
)(
    input logic           clk,
    input logic           resetn,
    input logic           data_valid,
    input logic [DSZ-1:0] data_in,
    input logic [SSZ-1:0] data_index,
    input logic           frame_start,
    input logic           frame_end,
    input logic [16-1: 0] threshold_k_sq,
    
    output logic [SSZ-1:0] peak_idx,
    output logic [DSZ-1:0] peak,
    output logic [SSZ-1:0] peak2_idx,
    output logic [DSZ-1:0] peak2,
    output logic [SSZ+DSZ-1:0] sum_o, 
    output logic [SSZ:0]   count_o,
    output logic           ready,
    output logic [5-1:0]   state
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
// operation to use less clock cycles.
//
//                peak - mean  >  k * stdev
//            (peak - mean)^2  >  k^2 * variance
//      N^2 * (peak - mean)^2  >  k^2 * N^2 * (mean_of_square - square_of_mean)
//      (peak * N - mean*N)^2  >  k^2 * (N * N * mean_of_square - square_of_mean * N * N)
//         (peak * N - sum)^2  >  k^2 * ( N * sum_of_square - square_of_sum)

localparam PIPLINE = 8-1;
localparam S_IDLE = 0;
localparam S_STREAM = 1;
localparam S_DETECT = 2;
localparam S_DETECT2 = S_DETECT+PIPLINE;
localparam S_DETECT3 = S_DETECT2+1;
localparam S_DETECT4 = S_DETECT3+PIPLINE;

logic [5-1: 0] current_state;

logic [SSZ+DSZ-1:0] sum; // sum of all data. Max value: 2^SSZ * 2^DSZ
logic [SSZ:0] count;

// sum of square of each data
logic [SSZ+DSZ*2-1:0] sum_sq;      // Max value: 2^SSZ * 2^DSZ * 2^DSZ

// square of sum
logic [(SSZ+DSZ)*2-1:0] S_sq [PIPLINE: 0]; // n-stage piplining
// N * sum_of_square
logic [(SSZ+DSZ)*2-1:0] N_S2 [PIPLINE: 0]; // n-stage piplining

// Scaled variance, i.e. N^2 * Variance = (N * sum_of_squrae - square_of_sum)
logic [(SSZ+DSZ)*2-1:0] V_scaled;
assign V_scaled = N_S2[PIPLINE] - S_sq[PIPLINE];

// peak * N - sum
logic [SSZ+DSZ-1:0] scaled_diff;
// peak2 * N - sum
logic [SSZ+DSZ-1:0] scaled_diff2; 

logic [(SSZ+DSZ)*2-1:0] scaled_diff_sq  [PIPLINE: 0]; // scaled_diff ^ 2, n-stage piplining
logic [(SSZ+DSZ)*2-1:0] scaled_diff2_sq [PIPLINE: 0]; // scaled_diff2 ^2, n-stage piplining

logic [(SSZ+DSZ)*2-1:0] threshold;

assign ready = current_state==S_IDLE;

assign state = current_state;

integer i;

always @(posedge clk)
if (resetn == 0) begin
    current_state <= S_IDLE;
end else begin

    for (i=0; i<PIPLINE; i=i+1) begin
        S_sq[i+1] <= S_sq[i];
        N_S2[i+1] <= N_S2[i];
        scaled_diff_sq[i+1] <= scaled_diff_sq[i];
        scaled_diff2_sq[i+1] <= scaled_diff2_sq[i];
    end

    if (frame_start) begin
        // Yes, we may be discarding the first incoming data, because data_valid might be on.
        // But that's okay.
        peak_idx <= 0;
        peak2_idx <= 0;
        peak <= 0;
        peak2 <= 0;
        count <= 0;
        sum <= 0;
        sum_sq <= 0;
        current_state <= S_STREAM;
    end else if (current_state == S_STREAM) begin
        if (frame_end) begin
            // Yes, we may be discarding the last data.
            // But that's okay.
            current_state <= S_DETECT;
            S_sq[0] <= sum * sum; // S^2
            N_S2[0] <= count * sum_sq; // N * S2

            // TODO: do we need to worry about signess?
            scaled_diff <= peak * count - sum;
            scaled_diff2 <= peak2 * count - sum;
        end else if (data_valid) begin
            if (peak <= data_in) begin
                peak2 <= peak;
                peak2_idx <= peak_idx;
                peak <= data_in;
                peak_idx <= data_index;
            end else if (peak2 <= data_in) begin
                peak2 <= data_in;
                peak2_idx <= data_index;
            end
            sum <= sum + data_in;
            sum_sq <= sum_sq + data_in * data_in;
            count <= count + 1;
        end
    end else if (current_state >= S_DETECT && current_state < S_DETECT2) begin
        current_state <= current_state + 1;
    end else if (current_state == S_DETECT2) begin
        threshold <= threshold_k_sq * V_scaled;
        scaled_diff_sq[0] <= scaled_diff * scaled_diff;
        scaled_diff2_sq[0] <= scaled_diff2 * scaled_diff2;
        current_state <= current_state + 1;
    end else if (current_state >= S_DETECT3 && current_state < S_DETECT4) begin
        current_state <= current_state + 1;
    end else if (current_state == S_DETECT4) begin
        if (scaled_diff_sq[PIPLINE] < threshold) begin
            peak_idx <= 0;
            peak2_idx <= 0;
        else if (scaled_diff2_sq[PIPLINE] < threshold)
            peak2_idx <= 0;
        sum_o <= sum;
        count_o <= count;
        current_state <= S_IDLE;
    end
end

endmodule
