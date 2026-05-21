module fft_proc #(
  parameter ASZ,        // ADC input sample width
  parameter DSZ,        // FFT_output width
  parameter FSZ,        // FFT transform length 2^FSZ
  parameter RSZ,        // RAM size 2^RSZ
  parameter HSZ,        // fft history buffer size 2^HSZ (Note: consider word size of 32bit, better not exceed 64KBytes in total)
  parameter QSZ         // FFT queue size 2^QSZ
)(
  input logic             clk_i,
  input logic             fft_rstn_i,
  input logic  [ ASZ-1:0] data_i,
  input logic             enable_i,
  input logic             dvalid_i, 
  input logic             trig_i,
  input logic  [ 32-1: 0] set_dly,

  input logic  [ 16-1: 0] fft_threshold_k,
  input logic  [ FSZ-1:0] fft_peak_start,
  input logic  [ DSZ-1:0] fft_peak_minimum,

  input logic  [ FSZ-1:0] fft_acq_up,
  input logic  [ FSZ-1:0] fft_acq_down,

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
  output logic [ FSZ-1:0] fft_peak_index_up,
  output logic [ FSZ-1:0] fft_peak_index_down,
  output logic [ DSZ-1:0] fft_peak_value_up,
  output logic [ DSZ-1:0] fft_peak_value_down,
  output logic [ 32-1: 0] fft_frame_cnt,
  output logic [ 32-1: 0] fft_scan_frame_cnt,
  output logic [ 32-1: 0] fft_we_cnt,
  output logic [ FSZ-1:0] padding_cnt,

  output logic [ QSZ-1:0] fft_q_wp,
  output logic [ QSZ-1:0] fft_q_rp,

  input logic  [  16-1:0] fft_conf_data_i,
  output logic [  32-1:0] fft_length,

  output logic [  32-1:0] overflow_cnt_o
);

logic [ 16-1:0] overflow_cnt;
logic [ 16-1:0] input_cnt;
assign overflow_cnt_o = {input_cnt, overflow_cnt};

logic [ 32-1: 0] frame_cnt;
logic [ 32-1: 0] scan_frame_cnt;
logic [ 32-1: 0] clk_cnt;

logic [ HSZ-1:0] fft_hist_index;
logic [ HSZ-1:0] prev_hist_index;

logic [DSZ-1: 0] fft_data;
logic [DSZ-1: 0] fft_data_abs;

logic [ASZ-1: 0]    fft_data_i;
logic [16-ASZ-1:0]  fft_data_ext;
logic               fft_saxi_last;
logic               fft_saxi_rdy;
logic               fft_saxi_valid;

logic [ FSZ-1: 0]   fft_hist_up[0:(1<<HSZ)-1];
logic [ FSZ-1: 0]   fft_hist_down[0:(1<<HSZ)-1];

logic [ FSZ-1: 0]   fft_peak_idx;
logic [ DSZ-1: 0]   fft_peak;
logic [ FSZ-1: 0]   fft_peak2_idx;
logic [ DSZ-1: 0]   fft_peak2;
logic [ FSZ+DSZ-1:0]_fft_sum;
logic [ FSZ: 0]     _fft_count;
logic [ FSZ-1: 0]   fft_peak_data_index;
logic               fft_peak_data_valid;
logic [ 32-1: 0]    fft_peak_data;
logic [ DSZ-1: 0]   fft_peak_data_abs;
logic               fft_peak_maxi_last;

logic [ 32-1:  0]   fft_maxi_phase;
logic [ 32-1:  0]   fft_maxi_data;
logic               fft_maxi_valid;
logic               fft_maxi_rdy;
logic               fft_maxi_last;
logic [ FSZ-1: 0]   fft_wp;
// bit reverse fft_wp, because we are using FFT ip core bit reversed option to
// save memory resource
logic [ FSZ-1: 0]   fft_wp_plus_one = fft_wp+1;
logic [ FSZ-1: 0]   fft_wp_reversed = {<<{fft_wp_plus_one}};
logic [ FSZ-1: 0]   fft_wp_index;

logic [ HSZ-1: 0]   fft_hist_raddr1;
logic [ HSZ-1: 0]   fft_hist_raddr2;

// sign extend the data for padding according to xfft requirement
assign fft_data_ext = {16-ASZ{fft_data_i[ASZ-1]}};

logic [1:0] fft_rstn;
always @(posedge clk_i) begin
    if (!fft_rstn_i)
        fft_rstn <= 0;
    else
        fft_rstn <= {fft_rstn[0], 1'b1};
end

logic       fft_conf_dvalid;
logic [1:0] rstn;

// always @(posedge clk_i) begin
//     if (!fft_rstn_i || fft_conf_dvalid)
//         rstn <= 0;
//     else
//         rstn <= {rstn[0], 1'b1};
// end

logic          rstn_i = fft_rstn[1];
logic [16-1:0] fft_conf_data;
logic [ 5-1:0] fft_nfft_i = fft_conf_data_i[5-1:0];
logic [ 5-1:0] fft_nfft = fft_conf_data[5-1:0];
logic [32-1:0] fft_length2, fft_length_plus_one;
logic          up_out;
logic          up_toggle = fft_length2 > fft_length;
logic [ FSZ-1: 0]   buf_raddr = sys_addr[FSZ-1+3:3];

xpm_memory_sdpram #(
    .MEMORY_SIZE            ((1<<FSZ)*DSZ),
    .ADDR_WIDTH_A           (FSZ),
    .ADDR_WIDTH_B           (FSZ),
    .CLOCKING_MODE          ("independent_clock"),
    .READ_LATENCY_B         (3),
    .READ_RESET_VALUE_B     ("100000"),
    .READ_DATA_WIDTH_B      (DSZ),
    .WRITE_DATA_WIDTH_A     (DSZ),
    .BYTE_WRITE_WIDTH_A     (DSZ)
) fft_buf_up (
    .addra  (fft_wp_index),
    .addrb  (buf_raddr),
    .clka   (clk_i),
    .clkb   (clk_i),
    .dina   (fft_maxi_data[DSZ-1:0]),
    .doutb  (fft_rdata_up_o),
    .ena    (1'b1),
    .wea    ({up_out && fft_maxi_valid && fft_maxi_rdy}),
    .rstb   (!rstn_i),
    .regceb (1'b1),
    .sleep  (1'b0),
    .enb    (1'b1)
);

xpm_memory_sdpram #(
    .MEMORY_SIZE            ((1<<FSZ)*DSZ),
    .ADDR_WIDTH_A           (FSZ),
    .ADDR_WIDTH_B           (FSZ),
    .CLOCKING_MODE          ("independent_clock"),
    .READ_LATENCY_B         (3),
    .READ_DATA_WIDTH_B      (DSZ),
    .WRITE_DATA_WIDTH_A     (DSZ),
    .BYTE_WRITE_WIDTH_A     (DSZ)
) fft_buf_down (
    .addra  (fft_wp_index),
    .addrb  (buf_raddr),
    .clka   (clk_i),
    .clkb   (clk_i),
    .dina   (fft_maxi_data[DSZ-1:0]),
    .doutb  (fft_rdata_down_o),
    .ena    (1'b1),
    .wea    ({!up_out && fft_maxi_valid && fft_maxi_rdy}),
    .rstb   (!rstn_i),
    .regceb (1'b1),
    .sleep  (1'b0),
    .enb    (1'b1)
);

always @(posedge clk_i) begin
   fft_hist_raddr1 <= sys_addr[HSZ-1+2:2]  ;
   fft_hist_raddr2 <= fft_hist_raddr1;
   fft_hist_rdata_down_o <= fft_hist_down[fft_hist_raddr2];
   fft_hist_rdata_up_o <= fft_hist_up[fft_hist_raddr2];
end

always @(posedge clk_i)
if (clk_cnt >= 125000000) begin
    clk_cnt <= 0;
    fft_frame_cnt <= {1'b0, frame_cnt[32-1:1]};
    fft_scan_frame_cnt <= scan_frame_cnt;
    if (fft_frame_start) begin
        frame_cnt <= 1;
        scan_frame_cnt <= 1;
        prev_hist_index <= fft_hist_index;
    end else begin
        frame_cnt <= 0;
        scan_frame_cnt <= 0;
    end
end else begin
    clk_cnt <= clk_cnt + 1;
    if (fft_frame_start) begin
        if (~&frame_cnt)
            frame_cnt <= frame_cnt + 1;
        if (~&scan_frame_cnt && prev_hist_index != fft_hist_index)
            scan_frame_cnt <= scan_frame_cnt + 1;
        prev_hist_index <= fft_hist_index;
    end
end

// Only allow one-time re-configuration after reset to avoid synchronization issue
always @(posedge clk_i)
if (fft_rstn_i == 1'b0) begin
    fft_conf_data <= fft_conf_data_i;
    fft_conf_dvalid <= 1;
    fft_length <= 2**fft_nfft_i;
    fft_length_plus_one = 2**fft_nfft_i + 1;
    // We need 2x amount of samples, one for Fup and one for Fdown
    if (fft_nfft_i < RSZ-1)
        fft_length2 <= 2**(fft_nfft_i+1);
    else
        fft_length2 <= 2**fft_nfft_i;
end else if (fft_conf_rdy) begin
    fft_conf_dvalid <= 0;
end

logic up_in;
logic [ FSZ-1:0]    padding_up;
logic [ FSZ-1:0]    padding_down;
assign  padding_cnt = up_in ? padding_up : padding_down;
logic triggered = trig_i && fft_done && up_in;

logic [ ASZ-1:0]    fin_dout;
logic fin_rd = fft_we_cnt > 0 && (fft_we_cnt > fft_length2 || fft_saxi_rdy);

xpm_fifo_async #(
    .FIFO_WRITE_DEPTH(1<<QSZ),
    .WRITE_DATA_WIDTH(ASZ),
    .READ_DATA_WIDTH (ASZ),
    // .RD_DATA_COUNT_WIDTH(QSZ),
    // .WR_DATA_COUNT_WIDTH(QSZ),
    .FIFO_READ_LATENCY(0),
    .READ_MODE       ("fwft")
) fifo_in (
    .rst             (!rstn_i || triggered),
    .wr_clk          (clk_i),
    .wr_en           (enable_i & dvalid_i),
    // .wr_data_count   (fft_q_wp),
    .din             (data_i),

    .rd_clk          (clk_i),
    .rd_en           (padding_cnt==0 && fin_rd),
    // .rd_data_count   (fft_q_rp),
    .dout            (fin_dout),

    .empty           (fin_empty),
    .full            (fin_full)
);


logic  [ HSZ-1:0] fft_hist_index_o;
logic             peak_ready_trig = {fft_peak_ready[0], peak_ready} == 2'b01;
logic             peak_up, peak_ready;

xpm_fifo_async #(
    .FIFO_WRITE_DEPTH(1<<(QSZ-1)),
    .WRITE_DATA_WIDTH(HSZ),
    .READ_DATA_WIDTH (HSZ),
    // .RD_DATA_COUNT_WIDTH(QSZ),
    // .WR_DATA_COUNT_WIDTH(QSZ),
    .FIFO_READ_LATENCY(0),
    .READ_MODE       ("fwft")
) fifo_index (
    .rst             (!rstn_i || fft_index_flush_i),
    .wr_clk          (clk_i),
    .wr_en           (fft_index_valid_i),
    .din             (fft_hist_index_i),

    // .empty           (findex_empty),
    // .full            (findex_full),

    .rd_clk          (clk_i),
    .rd_en           (peak_up != up_toggle && peak_ready_trig),
    .dout            (fft_hist_index_o)
);


always @(posedge clk_i)
if (rstn_i == 1'b0) begin
    fft_we_cnt <= 0;
    fft_done <= 1;
    up_in <= 1;
end else begin

    if (triggered) begin
        fft_we_cnt <= fft_length2;
        padding_up <= 2**fft_nfft - fft_acq_up;
        padding_down <= 2**fft_nfft - fft_acq_down;
    end else if ((padding_cnt > 0 || !fin_empty) && fin_rd) begin
        fft_data_i <= fin_dout;
        if (padding_cnt > 0) begin
            if (fin_full)
                overflow_cnt <= overflow_cnt + 1;
            else if (up_in)
                padding_up <= padding_up - 1;
            else
                padding_down <= padding_down - 1;
        end
        if (fft_we_cnt == 1 || fft_we_cnt == fft_length_plus_one)
            up_in <= up_in + up_toggle;
        fft_we_cnt <= fft_we_cnt - 1;
    end else if (fin_full) begin
        // overflow
        overflow_cnt <= overflow_cnt + 1;
        if (up_in)
            padding_up <= padding_up + 1;
        else
            padding_down <= padding_down + 1;
    end

    if (fft_saxi_rdy && fft_saxi_valid)
        input_cnt <= input_cnt + 1;

    fft_done <= fft_we_cnt==0;
    fft_saxi_last <= fft_we_cnt == 1 || fft_we_cnt == fft_length_plus_one;
    fft_saxi_valid <= (padding_cnt > 0 || !fin_empty) && fft_we_cnt > 0 && fft_we_cnt <= fft_length2;
end

always @(posedge clk_i)
if (rstn_i == 1'b0) begin
    fft_wp <= 0;
    fft_wp_index <= 0;
    fft_wp_last <= 0;
    up_out <= 1;
end else if (fft_maxi_valid && fft_maxi_rdy) begin
    if (fft_maxi_last) begin
        fft_wp_last <= fft_wp;
        fft_wp <= 0;
        fft_wp_index <= 0;
        up_out <= up_out + up_toggle;
    end else begin
        fft_wp <= fft_wp + 1;
        fft_wp_index <= fft_wp_reversed >> (FSZ-fft_nfft);
    end
end

assign fft_data = fft_maxi_data[DSZ-1:0];
assign fft_data_abs = fft_data[DSZ-1] ? -fft_data : fft_data;
assign fft_data_valid = fft_wp_index>=fft_peak_start && fft_wp_index<fft_length[FSZ:1] && fft_data_abs>fft_peak_minimum;

always @(posedge clk_i)
if (rstn_i == 1'b0) begin
    fft_peak_ready <= 2'b11;
    peak_up <= 1;
end else begin
    if (fft_index_flush_i) begin
        peak_up <= 1;
    end else begin
        if (peak_ready_trig) begin
            fft_hist_index <= fft_hist_index_o;
            peak_up <= peak_up + up_toggle;

            if (peak_up) begin
                fft_peak_index_up <= fft_peak_idx;
                fft_peak_value_up <= fft_peak;
            end else begin
                fft_peak_index_down <= fft_peak_idx;
                fft_peak_value_down <= fft_peak;
            end
            fft_sum <= _fft_sum;
            fft_count <= _fft_count;
        end

        if (fft_peak_ready == 2'b01) begin
            if (peak_up)
                fft_hist_up[fft_hist_index] <= fft_peak_index_up;
            else
                fft_hist_down[fft_hist_index] <= fft_peak_index_down;
        end
    end

    fft_peak_ready <= {fft_peak_ready[0], peak_ready};
end

xpm_fifo_axis #(
    .TDATA_WIDTH    (32),
    .TUSER_WIDTH    (FSZ+1),
    .FIFO_DEPTH     (16),
    .USE_ADV_FEATURES(16'h0000)   // Standard mode
) fifo_peak_in (
    .s_aclk         (clk_i),
    .s_aresetn      (rstn_i),
    
    // Slave side (From fft_i)
    .s_axis_tdata   ({{32-DSZ{1'b0}}, fft_data_abs}),
    .s_axis_tuser   ({fft_data_valid, fft_wp_index}),
    .s_axis_tvalid  (fft_maxi_valid),
    .s_axis_tready  (fft_maxi_rdy),
    .s_axis_tlast   (fft_maxi_last),

    // Master side (To Peak Detector)
    .m_axis_tdata   (fft_peak_data),
    .m_axis_tuser   ({fft_peak_data_valid, fft_peak_data_index}),
    .m_axis_tvalid  (fft_peak_maxi_valid),
    .m_axis_tready  (fft_peak_maxi_ready),
    .m_axis_tlast   (fft_peak_maxi_last)
);

peak_detector #(.SSZ(FSZ), .DSZ(DSZ)) peak_detector_i (
    .clk            (clk_i),
    .resetn         (rstn_i),
    .data_valid     (fft_peak_data_valid),
    .data_in        (fft_peak_data[DSZ-1:0]),
    .data_index     (fft_peak_data_index),
    .maxi_rdy       (fft_peak_maxi_ready),
    .maxi_valid     (fft_peak_maxi_valid),
    .maxi_last      (fft_peak_maxi_last),
    .threshold_k_sq (fft_threshold_k),
    .peak_idx       (fft_peak_idx),
    .peak           (fft_peak),
    .peak2_idx      (fft_peak2_idx),
    .peak2          (fft_peak2),
    .sum_o          (_fft_sum),
    .count_o        (_fft_count),
    .ready          (peak_ready)
);

fft_wrapper fft_i (
   .M_AXIS_DOUT_0_tdata         ({fft_maxi_phase, fft_maxi_data}),
   .M_AXIS_DOUT_0_tlast         (fft_maxi_last    ),
   .M_AXIS_DOUT_0_tvalid        (fft_maxi_valid   ),
   .M_AXIS_DOUT_0_tready        (fft_maxi_rdy     ),
   .S_AXIS_CONFIG_0_tdata       (fft_conf_data    ),
   .S_AXIS_CONFIG_0_tready      (fft_conf_rdy     ),
   .S_AXIS_CONFIG_0_tvalid      (fft_conf_dvalid  ),
   .S_AXIS_DATA_0_tdata         ({16'b0, fft_data_ext, fft_data_i}),
   .S_AXIS_DATA_0_tlast         (fft_saxi_last    ),
   .S_AXIS_DATA_0_tready        (fft_saxi_rdy     ),
   .S_AXIS_DATA_0_tvalid        (fft_saxi_valid   ),
   .aclk_0                      (clk_i            ),
   .aresetn_0                   (fft_rstn_i       ),
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
