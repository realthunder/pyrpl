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

    output logic          saxi_rdy,
    input logic           saxi_valid,
    input logic           saxi_last,
    
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
    S_COMPUTE
} peak_state_t;

(* fsm_encoding = "one_hot" *) peak_state_t current_state;

localparam PL1 = 4; // to meet the timing of data_sq

logic [DSZ-1:0]     data_r      [0:PL1];
logic [DSZ*2-1:0]   data_sq     [0:PL1];
logic [SSZ-1:0]     data_index_r[0:PL1];
logic               data_valid_r[0:PL1];
logic               data_last   [0:PL1];

localparam PL2 = 4; // to meet the timing of routing to DSP input register

logic [DSZ-1:0] peak_r[0:PL2];
assign peak = peak_r[0];

logic [SSZ+DSZ-1:0] sum[0:PL2]; // sum of all data. Max value: 2^SSZ * 2^DSZ
logic [SSZ:0] count[0:PL2];

assign sum_o = sum[PL2];
assign count_o = count[PL2];

// sum of square of each data
logic [SSZ+DSZ*2-1:0] sum_sq[0:PL2];      // Max value: 2^SSZ * 2^DSZ * 2^DSZ

// square of sum
logic [(SSZ+DSZ)*2-1:0] S_sq[0:PL2];
// N * sum_of_square
logic [(SSZ+DSZ)*2-1:0] N_S2[0:PL2];

// Scaled variance, i.e. N^2 * Variance = (N * sum_of_squrae - square_of_sum)
logic [(SSZ+DSZ)*2-1:0] V_scaled[0:PL2];

// peak * N
logic [SSZ+DSZ-1:0] scaled_peak[0:PL2];

// scaled_peak - sum
logic [SSZ+DSZ-1:0] scaled_diff[0:PL2];

logic [(SSZ+DSZ)*2-1:0] scaled_diff_sq[0:PL2]; // scaled_diff ^ 2

logic [16-1:0] k_sq = threshold_k_sq;
logic [(SSZ+DSZ)*2-1:0] threshold[0:PL2];

assign ready = current_state==S_IDLE;

assign state = current_state;

// logic [1:0] rstn_reg;
// logic       rstn = rstn[1]
// always @(posedge clk) begin
//     if (!resetn)
//         rstn_reg <= 0;
//     else
//         rstn_reg <= {rstn_reg[0], 1'b1};
// end

logic rstn = resetn;

integer i;

logic [8-1:0] step_cnt;

always @(posedge clk)
if (!rstn) begin
    current_state <= S_IDLE;
    saxi_rdy <= 0;

end else begin

    data_sq[0] <= data_r[0] * data_r[0];
    data_r[0] <= data_in;
    data_valid_r[0] <= data_valid & saxi_valid;
    data_index_r[0] <= data_index;
    data_last[0] <= saxi_last;

    for (i=0; i<PL2; i+=1) begin
        count[i+1] <= count[i];
        sum[i+1] <= sum[i];
        sum_sq[i+1] <= sum_sq[i];
        S_sq[i+1] <= S_sq[i];
        N_S2[i+1] <= N_S2[i];
        V_scaled[i+1] <= V_scaled[i];
        scaled_peak[i+1] <= scaled_peak[i];
        scaled_diff[i+1] <= scaled_diff[i];
        scaled_diff_sq[i+1] <= scaled_diff_sq[i];
        threshold[i+1] <= threshold[i];
        peak_r[i+1] <= peak_r[i];
    end

    // PL2 cycles for inputs to propagate (to meet routing timing)

    // 1 cycle computation
    S_sq[0] <= sum[PL2] * sum[PL2]; // S^2
    N_S2[0] <= count[PL2] * sum_sq[PL2]; // N * S2
    scaled_peak[0] <= peak_r[PL2] * count[PL2];

    // PL2 cycles for results to propagate

    // 1 cycle
    // TODO: do we need to worry about signess?
    scaled_diff[0] <= scaled_peak[PL2] - sum[PL2];
    V_scaled[0] <= N_S2[PL2] - S_sq[PL2];

    // PL2 cycles for results

    // 1 cycle
    threshold[0] <= k_sq * V_scaled[PL2];
    scaled_diff_sq[0] <= scaled_diff[PL2] * scaled_diff[PL2];

    // PL2 cycles for results
    //
    // So total = 4*PL2 + 3

    case (current_state)
    S_IDLE:
        if (saxi_valid) begin
            for (i=0;i<PL1;i+=1) begin
                data_valid_r[i+1] <= 0;
                data_last[i+1] <= 0;
            end
            peak_idx <= 0;
            peak_r[0] <= 0;
            count[0] <= 0;
            sum[0] <= 0;
            sum_sq[0] <= 0;
            current_state <= S_STREAM;
            saxi_rdy <= 1;
        end

    S_STREAM: begin
        for (i=0;i<PL1;i+=1) begin
            data_r[i+1] <= data_r[i];
            data_sq[i+1] <= data_sq[i];
            data_valid_r[i+1] <= data_valid_r[i];
            data_index_r[i+1] <= data_index_r[i];
            data_last[i+1] <= data_last[i];
        end

        if (data_valid_r[PL1]) begin
            if (peak_r[0] <= data_r[PL1]) begin
                peak_r[0] <= data_r[PL1];
                peak_idx <= data_index_r[PL1];
            end
            sum[0] <= sum[0] + data_r[PL1];
            sum_sq[0] <= sum_sq[0] + data_sq[PL1];
            count[0] <= count[0] + 1;
        end

        if (saxi_last)
            saxi_rdy <= 0;

        if (data_last[PL1]) begin
            current_state <= S_COMPUTE;
            step_cnt <= 0;
        end

    end S_COMPUTE:
        if (step_cnt != 4*PL2+3) 
            step_cnt <= step_cnt + 1;
        else begin
            if (scaled_diff_sq[PL2] < threshold[PL2])
                peak_idx <= 0;
            current_state <= S_IDLE;
        end
    endcase
end

endmodule
