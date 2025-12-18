module fft_proc #(
  parameter DSZ = 28,  // FFT_output width
  parameter FSZ = 13,  // FFT transform length 2^FSZ

  parameter QSZ = 12,  // FFT queue size 2^QSZ
  parameter RSZ = 14,  // RAM size 2^RSZ
  parameter HSZ = 12  // fft history buffer size 2^HSZ (Note: consider word size of 32bit, better not exceed 64KBytes in total)
)(
  input logic             clk_i,
  input logic             rstn_i,
  input logic  [ 14-1: 0] data_i,
  input logic             enable_i,
  input logic             dvalid_i, 
  input logic             trig_i,
  input logic             wrap_i,
  input logic  [ 32-1: 0] set_dly,

  input logic  [ HSZ-1:0] fft_hist_length,
  input logic  [ 16-1: 0] fft_threshold_k,
  input logic  [ FSZ-1:0] fft_peak_start,
  input logic  [ DSZ-1:0] fft_peak_minimum,


  input logic  [FSZ-1: 0] fft_raddr_i,
  output logic [DSZ-1: 0] fft_rdata_o,
  output logic [FSZ-1: 0] fft_wp_last,

  input logic  [ HSZ-1:0] fft_hist_raddr_i,
  output logic [ FSZ-1:0] fft_hist_rdata_o,

  output logic [  6-1: 0] status_o,
  output logic            fft_done,
  output logic [ FSZ: 0]  fft_count,
  output logic [ FSZ+DSZ-1: 0]fft_sum,
  output logic [  4-1: 0] fft_peak_state,
  output logic [ FSZ-1:0] fft_peak_index,
  output logic [ DSZ-1:0] fft_peak_value,
  output logic [ 16-1: 0] fft_frame_cnt,
  output logic [ 32-1: 0] fft_we_cnt,
  output logic [ HSZ-1:0] fft_hist_wp,
  output logic [ 16-1: 0] fft_skip_cnt
);

logic [ 16-1: 0] skip_cnt;
logic [ 16-1: 0] frame_cnt;
logic [ 32-1: 0] clk_cnt;

logic [ 14-1: 0] fft_queue[0:(1<<QSZ)-1];
logic [ 14-1: 0] fft_last_data;
logic [ 14-1: 0] fft_data;

logic [ QSZ-1:0] fft_q_wp;
logic [ QSZ-1:0] fft_q_rp;
logic [ QSZ-1:0] fft_q_size = fft_q_wp - fft_q_rp;

logic [ 14-1: 0] fft_data_i;
logic [ 16-14-1:0] fft_data_ext;
logic            fft_saxi_last;
logic            fft_saxi_rdy;
logic            fft_saxi_valid;

logic [ FSZ-1: 0]   fft_hist[0:(1<<HSZ)-1];
logic [ DSZ-1: 0]   fft_buf [0:(1<<FSZ)-1];

logic [ HSZ-1: 0]   fft_hist_wp_next;
logic [ 32-1:  0]   fft_peak_last_indices;
logic [ FSZ-1: 0]   fft_peak_idx;
logic [ DSZ-1: 0]   fft_peak;
logic [ FSZ-1: 0]   fft_peak2_idx;
logic [ DSZ-1: 0]   fft_peak2;
logic [ FSZ+DSZ-1:0]_fft_sum;
logic [ FSZ: 0]     _fft_count;
logic               fft_peak_ready_last;
logic [ FSZ-1: 0]   fft_peak_rp;
logic               fft_peak_data_valid;
logic [ DSZ-1: 0]   fft_peak_data;
logic [ DSZ-1: 0]   fft_peak_data_abs;

logic [ 32-1:  0]   fft_maxi_phase;
logic [ 32-1:  0]   fft_maxi_data;
logic               fft_maxi_valid;
logic               fft_maxi_last;
logic [ FSZ-1: 0]   fft_wp;

logic [ HSZ-1: 0]   fft_hist_raddr1;
logic [ HSZ-1: 0]   fft_hist_raddr2;
logic [ FSZ-1: 0]   fft_raddr1;
logic [ FSZ-1: 0]   fft_raddr2;

// sign extend the data for padding according to xfft requirement
assign fft_data_ext = {16-14{fft_data_i[14-1]}};
assign fft_saxi_last = fft_we_cnt == 1;
assign fft_saxi_valid = fft_q_size > 0 && fft_we_cnt > 0 && fft_we_cnt <= 2**FSZ;
assign fft_done = fft_peak_ready_last && fft_we_cnt==0;
assign fft_data = enable_i ? data_i : fft_last_data;

always @(posedge clk_i) begin
   fft_raddr1   <= fft_raddr_i;
   fft_raddr2   <= fft_raddr1;
   fft_rdata_o <= fft_buf[fft_raddr2];

   fft_hist_raddr1 <= fft_hist_raddr_i;
   fft_hist_raddr2 <= fft_hist_raddr1;
   fft_hist_rdata_o <= fft_hist[fft_hist_raddr2] ;
end

always @(posedge clk_i)
if (clk_cnt >= 125000000) begin
    clk_cnt <= 0;
    fft_skip_cnt <= skip_cnt;
    fft_frame_cnt <= frame_cnt;
    if (trig_i && !fft_done)
        skip_cnt <= 1;
    else
        skip_cnt <= 0;
    if (fft_frame_start)
        frame_cnt <= 1;
    else
        frame_cnt <= 0;
end else begin
    clk_cnt <= clk_cnt + 1;
    if (trig_i && !fft_done && ~&skip_cnt)
        skip_cnt <= skip_cnt + 1;
    if (fft_frame_start && ~&frame_cnt)
        frame_cnt <= frame_cnt + 1;
end

always @(posedge clk_i)
if (rstn_i == 1'b0) begin
    fft_we_cnt <= 0;
    fft_q_wp <= 0;
    fft_q_rp <= 0;
    fft_last_data <= 0 ;

end else begin

    if (dvalid_i) begin
        fft_queue[fft_q_wp] <= fft_data ;
        if (fft_q_wp + 1 == fft_q_rp)
            fft_q_rp <= fft_q_rp + 1;
        fft_q_wp <= fft_q_wp + 1;
        fft_last_data <= fft_data;
    end

    if (trig_i && fft_done) begin
        if (set_dly >= 2**FSZ) begin
            fft_q_rp <= fft_q_wp;
            fft_we_cnt <= set_dly;
        end else begin
            fft_we_cnt <= 2**FSZ;
            fft_q_rp <= fft_q_wp - (2**FSZ - set_dly);
            fft_data_i <= fft_queue[fft_q_wp - (2**FSZ - set_dly)];
        end
    end else if (fft_q_size > 0 && fft_we_cnt > 0 && (fft_we_cnt > 2**FSZ || fft_saxi_rdy)) begin
        // NOTE: there might be buffer overrun if sizeof(fft_queue) < sizeof(adc_buf)
        // The overrun is less likely the larger of fft_queue
        if (fft_q_rp == fft_q_wp)
            fft_data_i <= fft_data;
        else
            fft_data_i <= fft_queue[fft_q_rp];
        fft_q_rp <= fft_q_rp + 1;
        fft_we_cnt <= fft_we_cnt - 1;
    end
end

always @(posedge clk_i)
if (rstn_i == 1'b0) begin
    fft_wp <= 0;
    fft_wp_last <= 0;
end else if (fft_maxi_valid) begin
    fft_buf[fft_wp] <= fft_maxi_data[DSZ-1:0];
    if (fft_maxi_last) begin
        fft_wp_last <= fft_wp;
        fft_wp <= 0;
    end else
        fft_wp = fft_wp + 1;
end

assign fft_peak_rp = fft_wp;
assign fft_peak_data = fft_maxi_data[DSZ-1:0];
assign fft_peak_data_abs = fft_peak_data[DSZ-1] ? -fft_peak_data : fft_peak_data;
assign fft_peak_data_valid = fft_maxi_valid && fft_peak_rp>=fft_peak_start && fft_peak_rp[FSZ-1]==0 && fft_peak_data_abs>fft_peak_minimum;

always @(posedge clk_i) begin
    if (rstn_i == 1'b0) begin
        fft_hist_wp <= 0;
        fft_hist_wp_next <= 1;
        fft_peak_last_indices <= 0;
        fft_peak_ready_last <= 1;
    end else begin
        fft_peak_ready_last <= fft_peak_ready;
        if (!fft_peak_ready_last && fft_peak_ready) begin
            fft_hist[fft_hist_wp] <= fft_peak_idx;
            fft_peak_index <= fft_peak_idx;
            fft_peak_value <= fft_peak;
            fft_sum <= _fft_sum;
            fft_count <= _fft_count;
            fft_hist_wp <= fft_hist_wp_next;

            if (fft_hist_length == 0 && wrap_i || fft_hist_length > 0 && fft_hist_wp_next >= fft_hist_length-1)
                fft_hist_wp_next <= 0;
            else
                fft_hist_wp_next <= fft_hist_wp_next + 1;

        end else if (fft_hist_length == 0 && wrap_i)
            fft_hist_wp_next <= 0;
    end
end

peak_detector #(.SSZ(FSZ), .DSZ(DSZ)) peak_detector_i (
    .clk            (clk_i),
    .resetn         (rstn_i),
    .data_valid     (fft_peak_data_valid),
    .data_in        (fft_peak_data_abs),
    .data_index     (fft_peak_rp),
    .frame_start    (fft_frame_start),
    .frame_end      (fft_maxi_last),
    .threshold_k_sq (fft_threshold_k),
    .peak_idx       (fft_peak_idx),
    .peak           (fft_peak),
    .peak2_idx      (fft_peak2_idx),
    .peak2          (fft_peak2),
    .sum_o          (_fft_sum),
    .count_o        (_fft_count),
    .ready          (fft_peak_ready),
    .state          (fft_peak_state)
);

fft_wrapper fft_i (
   .M_AXIS_DOUT_0_tdata         ({fft_maxi_phase, fft_maxi_data}),
   .M_AXIS_DOUT_0_tlast         (fft_maxi_last    ),
   .M_AXIS_DOUT_0_tvalid        (fft_maxi_valid   ),
   .S_AXIS_CONFIG_0_tdata       ( ),
   .S_AXIS_CONFIG_0_tready      ( ),
   .S_AXIS_CONFIG_0_tvalid      ( ),
   .S_AXIS_DATA_0_tdata         ({16'b0, fft_data_ext, fft_data_i}),
   .S_AXIS_DATA_0_tlast         (fft_saxi_last    ),
   .S_AXIS_DATA_0_tready        (fft_saxi_rdy     ),
   .S_AXIS_DATA_0_tvalid        (fft_saxi_valid   ),
   .aclk_0                      (clk_i            ),
   .aresetn_0                   (rstn_i           ),
   .event_data_in_channel_halt_0(fft_in_halt      ),
   .event_data_out_channel_halt_0(fft_out_halt    ),
   .event_status_channel_halt_0 (fft_status_halt  ),
   .event_frame_started_0       (fft_frame_start  ),
   .event_tlast_missing_0       (fft_tlast_missing),
   .event_tlast_unexpected_0    (fft_tlast_unexp  )
);

//---------------------------------------------------------------------------------
//  System bus connection

assign status_o = {fft_in_halt
                   , fft_out_halt
                   , fft_status_halt
                   , fft_frame_start
                   , fft_tlast_missing
                   , fft_tlast_unexp};

endmodule
