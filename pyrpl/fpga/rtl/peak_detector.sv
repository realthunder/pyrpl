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
    output logic           ready
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

typedef enum {
    S_IDLE,
    S_STREAM,
    S_STEP1,
    S_STEP2,
    S_STEP3,
    S_STEP4,
    S_STEP5,
    S_STEP6,
    S_STEP7,
    S_STEP8
} peak_state_t;

(* fsm_encoding = "one_hot" *) peak_state_t current_state;

logic [SSZ+DSZ-1:0] sum, sum1; // sum of all data. Max value: 2^SSZ * 2^DSZ
logic [SSZ:0] count;

localparam PIPELINE = 2; // to meet the timing of data_sq
logic [DSZ-1:0]     data_r      [0:PIPELINE];
logic [DSZ*2-1:0]   data_sq     [0:PIPELINE];
logic [SSZ-1:0]     data_index_r[0:PIPELINE];
logic               data_valid_r[0:PIPELINE];
logic               data_last   [0:PIPELINE];

// sum of square of each data
logic [SSZ+DSZ*2-1:0] sum_sq, sum_sq1;      // Max value: 2^SSZ * 2^DSZ * 2^DSZ

// square of sum
logic [(SSZ+DSZ)*2-1:0] S_sq, S_sq_tmp;
// N * sum_of_square
logic [(SSZ+DSZ)*2-1:0] N_S2, N_S2_tmp;

// Scaled variance, i.e. N^2 * Variance = (N * sum_of_squrae - square_of_sum)
logic [(SSZ+DSZ)*2-1:0] V_scaled, V_scaled_tmp;

// peak * N
logic [SSZ+DSZ-1:0] scaled_peak, scaled_peak_tmp;
// peak2 * N
logic [SSZ+DSZ-1:0] scaled_peak2, scaled_peak2_tmp; 

// scaled_peak - sum
logic [SSZ+DSZ-1:0] scaled_diff, scaled_diff_tmp;
// scaled_peak2 - sum
logic [SSZ+DSZ-1:0] scaled_diff2, scaled_diff2_tmp; 

logic [(SSZ+DSZ)*2-1:0] scaled_diff_sq, scaled_diff_sq_tmp; // scaled_diff ^ 2
logic [(SSZ+DSZ)*2-1:0] scaled_diff2_sq, scaled_diff2_sq_tmp; // scaled_diff2 ^2

logic [16-1:0] k_sq;
logic [(SSZ+DSZ)*2-1:0] threshold, threshold_tmp;

assign ready = current_state==S_IDLE;

assign state = current_state;

logic [1:0] rst;
always @(posedge clk)
    if (!resetn)
        rst <= 0;
    else
        rst <= {rst[0], 1'b1};

integer i;

always @(posedge clk)
if (!rst[1]) begin
    current_state <= S_IDLE;
    maxi_rdy <= 0;

end else begin

    data_sq[0] <= data_r[0] * data_r[0];
    data_r[0] <= data_in;
    data_valid_r[0] <= data_valid & maxi_valid;
    data_index_r[0] <= data_index;
    data_last[0] <= maxi_last;

    case (current_state)
    S_IDLE:
        if (maxi_valid) begin
            for (i=0;i<PIPELINE;i+=1) begin
                data_valid_r[i+1] <= 0;
                data_last[i+1] <= 0;
            end
            peak_idx <= 0;
            peak2_idx <= 0;
            peak <= 0;
            peak2 <= 0;
            count <= 0;
            sum <= 0;
            sum_sq <= 0;
            current_state <= S_STREAM;
            maxi_rdy <= 1;
        end

    S_STREAM: begin
        for (i=0;i<PIPELINE;i+=1) begin
            data_r[i+1] <= data_r[i];
            data_sq[i+1] <= data_sq[i];
            data_valid_r[i+1] <= data_valid_r[i];
            data_index_r[i+1] <= data_index_r[i];
            data_last[i+1] <= data_last[i];
        end

        if (data_valid_r[PIPELINE]) begin
            if (peak <= data_r[PIPELINE]) begin
                peak2 <= peak;
                peak2_idx <= peak_idx;
                peak <= data_r[PIPELINE];
                peak_idx <= data_index_r[PIPELINE];
            end else if (peak2 <= data_r[PIPELINE]) begin
                peak2 <= data_r[PIPELINE];
                peak2_idx <= data_index_r[PIPELINE];
            end
            sum <= sum + data_r[PIPELINE];
            sum_sq <= sum_sq + data_sq[PIPELINE-1];
            count <= count + 1;
        end

        if (data_last[PIPELINE]) begin
            current_state <= S_STEP1;
            maxi_rdy <= 0;
        end

    end S_STEP1: begin
        sum1 <= sum;
        sum_sq1 <= sum_sq;

        current_state <= S_STEP2;
    end S_STEP2: begin
        S_sq_tmp <= sum1 * sum1; // S^2
        N_S2_tmp <= count * sum_sq1; // N * S2
        scaled_peak_tmp <= peak * count;
        scaled_peak2_tmp <= peak2 * count;

        current_state <= S_STEP3;
    end S_STEP3: begin
        S_sq <= S_sq_tmp;
        N_S2 <= N_S2_tmp;
        scaled_peak <= scaled_peak_tmp;
        scaled_peak2 <= scaled_peak2_tmp;

        current_state <= S_STEP4;
    end S_STEP4: begin

        // TODO: do we need to worry about signess?
        scaled_diff_tmp <= scaled_peak - sum1;
        scaled_diff2_tmp <= scaled_peak2 - sum1;
        V_scaled_tmp <= N_S2 - S_sq;

        current_state <= S_STEP5;
    end S_STEP5: begin
        scaled_diff <= scaled_diff_tmp;
        scaled_diff2 <= scaled_diff2_tmp;
        V_scaled <= V_scaled_tmp;
        k_sq <= threshold_k_sq;

        current_state <= S_STEP6;
    end S_STEP6: begin
        threshold_tmp <= k_sq * V_scaled;
        scaled_diff_sq_tmp <= scaled_diff * scaled_diff;
        scaled_diff2_sq_tmp <= scaled_diff2 * scaled_diff2;

        current_state <= S_STEP7;
    end S_STEP7: begin
        threshold <= threshold_tmp;
        scaled_diff_sq <= scaled_diff_sq_tmp;
        scaled_diff2_sq <= scaled_diff2_sq_tmp;

        current_state <= S_STEP8;
    end S_STEP8: begin
        if (scaled_diff_sq < threshold) begin
            peak_idx <= 0;
            peak2_idx <= 0;
        end else if (scaled_diff2_sq < threshold)
            peak2_idx <= 0;
        sum_o <= sum1;
        count_o <= count;
        current_state <= S_IDLE;
    end
    endcase
end

endmodule
