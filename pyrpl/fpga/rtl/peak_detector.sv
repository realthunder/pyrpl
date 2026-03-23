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

localparam PIPELINE = 4-1;
localparam S_IDLE = 0;
localparam S_STREAM = 1;
localparam S_DETECT = 2;
localparam S_DETECT1 = S_DETECT +PIPELINE+1;
localparam S_DETECT2 = S_DETECT1+1;
localparam S_DETECT3 = S_DETECT2+PIPELINE+1;
localparam S_DETECT4 = S_DETECT3+PIPELINE+1;
localparam S_DETECT5 = S_DETECT4+PIPELINE+1;

logic [8-1: 0] current_state;

logic [SSZ+DSZ-1:0] sum; // sum of all data. Max value: 2^SSZ * 2^DSZ
logic [SSZ:0] count;

logic [DSZ-1:0] data_reg;
logic [SSZ-1:0] data_sq_reg;
logic [SSZ-1:0] data_index_reg;

// sum of square of each data
logic [SSZ+DSZ*2-1:0] sum_sq[PIPELINE: 0];      // Max value: 2^SSZ * 2^DSZ * 2^DSZ

// square of sum
logic [(SSZ+DSZ)*2-1:0] sum_sq_reg;
logic [(SSZ+DSZ)*2-1:0] scaled_sum_sq;
logic [(SSZ+DSZ)*2-1:0] S_sq [PIPELINE: 0]; // n-stage piplining
// N * sum_of_square
logic [(SSZ+DSZ)*2-1:0] N_S2 [PIPELINE: 0]; // n-stage piplining

// Scaled variance, i.e. N^2 * Variance = (N * sum_of_squrae - square_of_sum)
logic [(SSZ+DSZ)*2-1:0] V_scaled [PIPELINE: 0];

// peak * N
logic [SSZ+DSZ-1:0] scaled_peak [PIPELINE: 0];
logic [SSZ+DSZ-1:0] scaled_peak_reg;
// peak2 * N
logic [SSZ+DSZ-1:0] scaled_peak2 [PIPELINE: 0]; 
logic [SSZ+DSZ-1:0] scaled_peak2_reg; 

// scaled_peak - sum
logic [SSZ+DSZ-1:0] scaled_diff [PIPELINE: 0];
// scaled_peak2 - sum
logic [SSZ+DSZ-1:0] scaled_diff2 [PIPELINE: 0];; 

logic [(SSZ+DSZ)*2-1:0] scaled_diff_sq  [PIPELINE: 0]; // scaled_diff ^ 2, n-stage piplining
logic [(SSZ+DSZ)*2-1:0] scaled_diff2_sq [PIPELINE: 0]; // scaled_diff2 ^2, n-stage piplining

logic [(SSZ+DSZ)*2-1:0] threshold [PIPELINE: 0];

assign ready = current_state==S_IDLE;

assign state = current_state;

assign maxi_rdy = current_state==S_STREAM;

integer i;

always @(posedge clk)
if (resetn == 0) begin
    current_state <= S_IDLE;
end else begin
    if (current_state == S_IDLE) begin
        if (maxi_valid) begin
            data_reg <= data_in;
            data_sq_reg <= data_in * data_in;
            data_index_reg <= data_index;

            peak_idx <= 0;
            peak2_idx <= 0;
            peak <= 0;
            peak2 <= 0;
            count <= 0;
            sum <= 0;
            for (i=0; i<=PIPELINE; i=i+1) begin
                sum_sq[i] <= 0;
            end
            current_state <= S_STREAM;
        end
    end else if (current_state == S_STREAM) begin
        if (maxi_valid) begin
            data_reg <= data_in;
            data_sq_reg <= data_in * data_in;
            data_index_reg <= data_index;

            if (maxi_last)
                current_state <= S_DETECT;
            if (data_valid) begin
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
                sum_sq[0] <= data_sq_reg;
                for (i=0; i<PIPELINE-1; i=i+1) begin
                    sum_sq[i+1] <= sum_sq[i];
                end
                count <= count + 1;
            end
        end

    end else if (current_state >= S_DETECT && current_state < S_DETECT1) begin
        for (i=0; i<PIPELINE-1; i=i+1) begin
            sum_sq[i+1] <= sum_sq[i+1] + sum_sq[i];
        end
        sum_sq[PIPELINE] <= sum_sq[PIPELINE] + sum_sq[PIPELINE-1];

        current_state <= current_state + 1;

    end else if (current_state == S_DETECT1) begin
        sum_sq_reg <= sum_sq[PIPELINE];
        scaled_peak_reg <= peak * count;
        scaled_peak2_reg <= peak2 * count;

        current_state <= current_state + 1;

    end else if (current_state >= S_DETECT2 && current_state < S_DETECT3) begin
        S_sq[0] <= sum * sum; // S^2
        N_S2[0] <= count * sum_sq_reg; // N * S2

        scaled_peak[0] <= scaled_peak_reg;
        scaled_peak2[0] <= scaled_peak2_reg;

        for (i=0; i<PIPELINE; i=i+1) begin
            S_sq[i+1] <= S_sq[i];
            N_S2[i+1] <= N_S2[i];
            scaled_peak[i+1] <= scaled_peak[i];
            scaled_peak2[i+1] <= scaled_peak2[i];
        end

        current_state <= current_state + 1;

    end else if (current_state >= S_DETECT3 && current_state < S_DETECT4) begin

        // TODO: do we need to worry about signess?
        scaled_diff[0] <= scaled_peak[PIPELINE] - sum;
        scaled_diff2[0] <= scaled_peak2[PIPELINE] - sum;
        V_scaled[0] <= N_S2[PIPELINE] - S_sq[PIPELINE];

        for (i=0; i<PIPELINE; i=i+1) begin
            scaled_diff[i+1] <= scaled_diff[i];
            scaled_diff2[i+1] <= scaled_diff2[i];
            V_scaled[i+1] <= V_scaled[i];
        end

        current_state <= current_state + 1;

    end else if (current_state >= S_DETECT4 && current_state < S_DETECT5) begin
        threshold[0] <= threshold_k_sq * V_scaled[PIPELINE];
        scaled_diff_sq[0] <= scaled_diff[PIPELINE] * scaled_diff[PIPELINE];
        scaled_diff2_sq[0] <= scaled_diff2[PIPELINE] * scaled_diff2[PIPELINE];

        for (i=0; i<PIPELINE; i=i+1) begin
            threshold[i+1] <= threshold[i];
            scaled_diff_sq[i+1] <= scaled_diff_sq[i];
            scaled_diff2_sq[i+1] <= scaled_diff2_sq[i];
        end

        current_state <= current_state + 1;

    end else if (current_state == S_DETECT5) begin
        if (scaled_diff_sq[PIPELINE] < threshold[PIPELINE]) begin
            peak_idx <= 0;
            peak2_idx <= 0;
        end else if (scaled_diff2_sq[PIPELINE] < threshold[PIPELINE])
            peak2_idx <= 0;
        sum_o <= sum;
        count_o <= count;
        current_state <= S_IDLE;
    end
end

endmodule
