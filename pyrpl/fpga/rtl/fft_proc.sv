module fft_proc #(
  parameter ASZ,        // ADC input sample width
  parameter DSZ,        // FFT_output width
  parameter FSZ,        // FFT transform length 2^FSZ
  parameter RSZ,        // RAM size 2^RSZ
  parameter HSZ,        // fft history buffer size 2^HSZ (Note: consider word size of 32bit, better not exceed 64KBytes in total)
  parameter QSZ         // FFT queue size 2^QSZ
)(
  input logic             clk_i,
  input logic             rstn_i,
  input logic  [ ASZ-1:0] data_i,
  input logic             enable_i,
  input logic             dvalid_i, 
  input logic             trig_i,
  input logic  [ 32-1: 0] set_dly,

  input logic  [ 16-1: 0] fft_threshold_k,
  input logic  [ FSZ-1:0] fft_peak_start,
  input logic  [ DSZ-1:0] fft_peak_minimum,


  input logic  [ 32-1: 0] sys_addr,

  output logic [DSZ-1: 0] fft_rdata_up_o,
  output logic [DSZ-1: 0] fft_rdata_down_o,
  output logic [FSZ-1: 0] fft_wp_last,

  input logic             fft_index_flush_i,
  input logic             fft_index_valid_i,
  input logic  [ HSZ-1:0] fft_hist_index_i,

  output logic [ FSZ-1:0] fft_hist_rdata_up_o,
  output logic [ FSZ-1:0] fft_hist_rdata_down_o,

  output logic [  6-1: 0] status_o,
  output logic            fft_done,
  output logic [ 2-1 : 0] fft_peak_ready,
  output logic [ FSZ : 0] fft_count,
  output logic [ FSZ+DSZ-1: 0]fft_sum,
  output logic [  8-1: 0] fft_peak_state,
  output logic [ FSZ-1:0] fft_peak_index_up,
  output logic [ FSZ-1:0] fft_peak_index_down,
  output logic [ DSZ-1:0] fft_peak_value_up,
  output logic [ DSZ-1:0] fft_peak_value_down,
  output logic [ 32-1: 0] fft_frame_cnt,
  output logic [ 32-1: 0] fft_we_cnt,
  output logic [ 16-1: 0] fft_skip_cnt,

  output logic [ QSZ-1:0] fft_q_wp,
  output logic [ QSZ-1:0] fft_q_rp,
  output logic [ QSZ-1:0] fft_q_rp_save,
  output logic [ ASZ-1:0] fft_q_rdata_o
);

logic [ 16-1: 0] skip_cnt;
logic [ 32-1: 0] frame_cnt;
logic [ 32-1: 0] clk_cnt;

logic [ HSZ-1:0] fft_index_q[0:(1<<QSZ)-1];
logic [ HSZ-1:0] fft_hist_index;
logic [ QSZ-1:0] index_wp;
logic [ QSZ-1:0] index_wp_plus_one = index_wp + 1;
logic [ QSZ-1:0] index_rp;

logic [ASZ-1: 0] fft_queue[0:(1<<QSZ)-1];
// logic [ASZ-1: 0] fft_queue2[0:(1<<QSZ)-1];
logic [ASZ-1: 0] fft_last_data;
logic            fft_inited;
logic [ASZ-1: 0] fft_data;
logic [QSZ-1: 0] fft_q_wp_plus_one = fft_q_wp + 1;
logic [QSZ-1: 0] fft_q_size = fft_q_wp - fft_q_rp;

logic [ASZ-1: 0]    fft_data_i;
logic [16-ASZ-1:0]  fft_data_ext;
logic               fft_saxi_last;
logic               fft_saxi_rdy;
logic               fft_saxi_valid;

logic [ FSZ-1: 0]   fft_hist_up[0:(1<<HSZ)-1];
logic [ FSZ-1: 0]   fft_hist_down[0:(1<<HSZ)-1];

logic [ DSZ-1: 0]   fft_buf_up[0:(1<<FSZ)-1];
logic [ DSZ-1: 0]   fft_buf_down[0:(1<<FSZ)-1];

logic [ FSZ-1: 0]   fft_peak_idx;
logic [ DSZ-1: 0]   fft_peak;
logic [ FSZ-1: 0]   fft_peak2_idx;
logic [ DSZ-1: 0]   fft_peak2;
logic [ FSZ+DSZ-1:0]_fft_sum;
logic [ FSZ: 0]     _fft_count;
logic [ FSZ-1: 0]   fft_peak_rp;
logic               fft_peak_data_valid;
logic [ DSZ-1: 0]   fft_peak_data;
logic [ DSZ-1: 0]   fft_peak_data_abs;

logic [ 32-1:  0]   fft_maxi_phase;
logic [ 32-1:  0]   fft_maxi_data;
logic               fft_maxi_valid;
logic               fft_maxi_rdy;
logic               fft_maxi_last;
logic [ FSZ-1: 0]   fft_wp;

logic [ HSZ-1: 0]   fft_hist_raddr1;
logic [ HSZ-1: 0]   fft_hist_raddr2;
logic [ FSZ-1: 0]   fft_raddr1;
logic [ FSZ-1: 0]   fft_raddr2;
logic [ HSZ-1: 0]   fft_q_raddr1;
logic [ HSZ-1: 0]   fft_q_raddr2;

// sign extend the data for padding according to xfft requirement
assign fft_data_ext = {16-ASZ{fft_data_i[ASZ-1]}};
assign fft_data = (enable_i || !fft_inited) ? data_i : fft_last_data;

always @(posedge clk_i) begin
   fft_raddr1 <= sys_addr[FSZ-1+3:3] ;
   fft_raddr2 <= fft_raddr1;
   fft_rdata_up_o <= fft_buf_up[fft_raddr2];
   fft_rdata_down_o <= fft_buf_down[fft_raddr2];

   fft_hist_raddr1 <= sys_addr[HSZ-1+2:2]  ;
   fft_hist_raddr2 <= fft_hist_raddr1;
   fft_hist_rdata_down_o <= fft_hist_down[fft_hist_raddr2];
   fft_hist_rdata_up_o <= fft_hist_up[fft_hist_raddr2];

   // fft_q_raddr1 <= sys_addr[QSZ-1+2:2]  ;
   // fft_q_raddr2 <= fft_q_raddr1;
   // fft_q_rdata_o <= fft_queue2[fft_q_rp_save + fft_q_raddr2];
end

always @(posedge clk_i)
if (clk_cnt >= 125000000) begin
    clk_cnt <= 0;
    fft_skip_cnt <= skip_cnt;
    fft_frame_cnt <= {1'b0, frame_cnt[32-1:1]};
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

logic pre_size = 2**(FSZ+1) - set_dly;

logic up_in;

always @(posedge clk_i)
if (rstn_i == 1'b0) begin
    fft_we_cnt <= 0;
    fft_q_wp <= 0;
    fft_q_rp <= 0;
    fft_done <= 1;
    up_in <= 1;
end else begin
    if (dvalid_i) begin
        fft_queue[fft_q_wp] <= fft_data;
        // fft_queue2[fft_q_wp] <= fft_data;
        fft_q_wp <= fft_q_wp_plus_one;
        fft_last_data <= fft_data;
        fft_inited <= 1;
    end

    if (trig_i && fft_done && up_in == 1) begin
        // FSZ+1 because we need 2x amount of samples, one for Fup and one for Fdown
        if (set_dly >= 2**(FSZ+1)) begin
            fft_q_rp <= fft_q_wp;
            fft_q_rp_save <= fft_q_wp;
            fft_we_cnt <= set_dly;
        end else begin
            fft_we_cnt <= 2**(FSZ+1);
            if (fft_q_size > pre_size) begin
                fft_q_rp <= fft_q_wp - pre_size;
                fft_q_rp_save <= fft_q_wp - pre_size;
            end else
                fft_q_rp_save <= fft_q_rp;
        end
    end else if (fft_q_size > 0 && fft_we_cnt > 0 && (fft_we_cnt > 2**(FSZ+1) || fft_saxi_rdy)) begin
        fft_data_i <= fft_queue[fft_q_rp];
        fft_q_rp <= fft_q_rp + 1;
        if (fft_we_cnt == 1 || fft_we_cnt == (2**FSZ)+1)
            up_in <= !up_in;
        fft_we_cnt <= fft_we_cnt - 1;
    end else if (dvalid_i && fft_q_wp_plus_one == fft_q_rp)
        fft_q_rp <= fft_q_rp + 1;

    fft_done <= fft_we_cnt==0;
    fft_saxi_last <= fft_we_cnt == 1 || fft_we_cnt == (2**FSZ)+1;
    fft_saxi_valid <= fft_q_size > 0 && fft_we_cnt > 0 && fft_we_cnt <= 2**(FSZ+1);
end

logic up_out;

always @(posedge clk_i)
if (rstn_i == 1'b0) begin
    fft_wp <= 0;
    fft_wp_last <= 0;
    up_out <= 1;
end else if (fft_maxi_valid && fft_maxi_rdy) begin
    if (up_out)
        fft_buf_up[fft_wp] <= fft_maxi_data[DSZ-1:0];
    else
        fft_buf_down[fft_wp] <= fft_maxi_data[DSZ-1:0];
    if (fft_maxi_last) begin
        fft_wp_last <= fft_wp;
        fft_wp <= 0;
        up_out <= !up_out;
    end else
        fft_wp = fft_wp + 1;
end

assign fft_peak_rp = fft_wp;
assign fft_peak_data = fft_maxi_data[DSZ-1:0];
assign fft_peak_data_abs = fft_peak_data[DSZ-1] ? -fft_peak_data : fft_peak_data;
assign fft_peak_data_valid = fft_maxi_valid && fft_peak_rp>=fft_peak_start && fft_peak_rp[FSZ-1]==0 && fft_peak_data_abs>fft_peak_minimum;

logic peak_up;
logic peak_ready;

always @(posedge clk_i)
if (rstn_i == 1'b0) begin
    fft_peak_ready <= 2'b11;
    index_wp <= 0;
    index_rp <= 0;
    peak_up <= 1;
end else begin
    if (fft_index_flush_i) begin
        index_wp <= 0;
        index_rp <= 0;
        peak_up <= 1;
    end else begin
        if (fft_index_valid_i) begin
            fft_index_q[index_wp] <= fft_hist_index_i;
            index_wp <= index_wp_plus_one;
        end else if ({fft_peak_ready[0], peak_ready} == 2'b01) begin
            if (index_rp == index_wp)
                fft_hist_index <= fft_hist_index_i;
            else
                fft_hist_index <= fft_index_q[index_rp];

            if (peak_up == 0)
                index_rp <= index_rp + 1;
            peak_up <= !peak_up;

            if (peak_up) begin
                fft_peak_index_up <= fft_peak_idx;
                fft_peak_value_up <= fft_peak;
            end else begin
                fft_peak_index_down <= fft_peak_idx;
                fft_peak_value_down <= fft_peak;
            end
            fft_sum <= _fft_sum;
            fft_count <= _fft_count;

        end else if (peak_up == 0 && index_wp_plus_one == index_rp)
            index_rp <= index_rp + 1;

        if (fft_peak_ready == 2'b01) begin
            if (peak_up)
                fft_hist_up[fft_hist_index] <= fft_peak_index_up;
            else
                fft_hist_down[fft_hist_index] <= fft_peak_index_down;
        end
    end

    fft_peak_ready = {fft_peak_ready[0], peak_ready};
end

peak_detector #(.SSZ(FSZ), .DSZ(DSZ)) peak_detector_i (
    .clk            (clk_i),
    .resetn         (rstn_i),
    .data_valid     (fft_peak_data_valid),
    .data_in        (fft_peak_data_abs),
    .data_index     (fft_peak_rp),
    .maxi_rdy       (fft_maxi_rdy),
    .maxi_valid     (fft_maxi_valid),
    .maxi_last      (fft_maxi_last),
    .threshold_k_sq (fft_threshold_k),
    .peak_idx       (fft_peak_idx),
    .peak           (fft_peak),
    .peak2_idx      (fft_peak2_idx),
    .peak2          (fft_peak2),
    .sum_o          (_fft_sum),
    .count_o        (_fft_count),
    .ready          (peak_ready),
    .state          (fft_peak_state)
);

fft_wrapper fft_i (
   .M_AXIS_DOUT_0_tdata         ({fft_maxi_phase, fft_maxi_data}),
   .M_AXIS_DOUT_0_tlast         (fft_maxi_last    ),
   .M_AXIS_DOUT_0_tvalid        (fft_maxi_valid   ),
   .M_AXIS_DOUT_0_tready        (fft_maxi_rdy     ),
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
