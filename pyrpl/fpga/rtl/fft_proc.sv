module fft_proc #(
  parameter ASZ,        // ADC input sample width
  parameter DSZ,        // FFT_output width
  parameter FSZ,        // FFT transform length 2^FSZ
  parameter RSZ,        // RAM size 2^RSZ
  parameter HSZ,        // fft history buffer size 2^HSZ (Note: consider word size of 32bit, better not exceed 64KBytes in total)
  parameter QSZ,        // FFT queue size 2^QSZ
  parameter READ_DELAY  // memory output read delay
)(
  input logic             adc_clk_i,
  input logic             clk_i,
  input logic             adc_rstn_in,
  input logic  [ ASZ-1:0] data_in,
  input logic             enable_in,
  input logic             dvalid_in, 
  input logic             trig_in,

  input logic  [ 16-1: 0] fft_threshold_k_in,
  input logic  [ FSZ-1:0] fft_peak_start_in,
  input logic  [ DSZ-1:0] fft_peak_minimum_in,

  input logic  [ FSZ-1:0] fft_acq_up_in,
  input logic  [ FSZ-1:0] fft_acq_down_in,

  input logic  [ 32-1: 0] sys_addr_in,

  output logic [DSZ-1: 0] fft_rdata_up_o,
  output logic [DSZ-1: 0] fft_rdata_down_o,

  input logic             fft_index_flush_in,
  input logic             fft_index_valid_in,
  input logic  [ HSZ-1:0] fft_hist_index_in,

  output logic [ FSZ-1:0] fft_hist_rdata_up_o,
  output logic [ FSZ-1:0] fft_hist_rdata_down_o,

  output logic [  6-1: 0] status_o,
  output logic            fft_done_o,
  output logic            fft_peak_ready_o,

  output logic [ FSZ : 0] fft_count_o,
  output logic [ FSZ+DSZ-1: 0]fft_sum_o,
  output logic [ FSZ-1:0] fft_peak_index_up_o,
  output logic [ FSZ-1:0] fft_peak_index_down_o,
  output logic [ DSZ-1:0] fft_peak_value_up_o,
  output logic [ DSZ-1:0] fft_peak_value_down_o,

  output logic [ 32-1: 0] fft_we_cnt,

  output logic [ 32-1: 0] frame_cnt_o,
  output logic [ 32-1: 0] scan_frame_cnt_o,

  input logic  [  16-1:0] fft_conf_data_in,

  output logic [  32-1:0] overflow_cnt_o
);

logic [ 16-1:0] overflow_cnt;
logic [ 16-1:0] input_cnt;
assign overflow_cnt_o = {input_cnt, overflow_cnt};

logic [ 32-1: 0] frame_cnt;
logic [ 32-1: 0] scan_frame_cnt;

logic [ HSZ-1:0] fft_hist_index;
logic [ HSZ-1:0] prev_hist_index;

logic [DSZ-1: 0] fft_data;
logic [DSZ-1: 0] fft_data_abs;

logic [ASZ-1: 0]    fft_data_i;
logic [16-ASZ-1:0]  fft_data_ext;
logic               fft_saxi_last;
logic               fft_saxi_rdy;
logic               fft_saxi_valid;

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

// sign extend the data for padding according to xfft requirement
assign fft_data_ext = {16-ASZ{fft_data_i[ASZ-1]}};

logic             adc_rstn_i;

localparam READ_LATENCY = READ_DELAY-2;
localparam ADDR_A_DELAY = 2;
localparam ADDR_B_DELAY = 2;
localparam WRITE_DELAY = 3-1;

logic [ FSZ-1: 0]   buf_a_waddr  [0 : WRITE_DELAY];
logic [ FSZ-1: 0]   buf_b_waddr  [0 : WRITE_DELAY];
logic [ DSZ-1: 0]   buf_a_wdata  [0 : WRITE_DELAY];
logic [ DSZ-1: 0]   buf_b_wdata  [0 : WRITE_DELAY];
logic               buf_a_we     [0 : WRITE_DELAY];
logic               buf_b_we     [0 : WRITE_DELAY];
logic [ FSZ-1: 0]   buf_a_raddr  [0 :ADDR_A_DELAY];
logic [ FSZ-1: 0]   buf_b_raddr  [0 :ADDR_B_DELAY];

logic [ FSZ-1: 0]   hist_a_waddr [0 : WRITE_DELAY];
logic [ FSZ-1: 0]   hist_b_waddr [0 : WRITE_DELAY];
logic [ DSZ-1: 0]   hist_a_wdata [0 : WRITE_DELAY];
logic [ DSZ-1: 0]   hist_b_wdata [0 : WRITE_DELAY];
logic               hist_a_we    [0 : WRITE_DELAY];
logic               hist_b_we    [0 : WRITE_DELAY];
logic [ FSZ-1: 0]   hist_a_raddr [0 :ADDR_A_DELAY];
logic [ FSZ-1: 0]   hist_b_raddr [0 :ADDR_B_DELAY];

logic  [ ASZ-1:0] data_i;
logic             enable_i;
logic             dvalid_i; 
logic             trig_i;

logic  [ 16-1: 0] fft_threshold_k_arg;
logic  [ FSZ-1:0] fft_peak_start_arg;
logic  [ DSZ-1:0] fft_peak_minimum_arg;

logic  [ FSZ-1:0] fft_acq_up_i;
logic  [ FSZ-1:0] fft_acq_down_i;

logic  [ 32-1: 0] sys_addr_i;

logic             fft_index_flush_i;
logic             fft_index_valid_i;
logic  [ HSZ-1:0] fft_hist_index_i;

logic  [  16-1:0] fft_conf_data_i;

integer i;

always @(posedge adc_clk_i) begin
    hist_a_raddr[0] <= sys_addr_in[HSZ-1+2:2];
    hist_b_raddr[0] <= sys_addr_in[HSZ-1+2:2];
    buf_a_raddr[0] <= sys_addr_in[FSZ-1+3:3];
    buf_b_raddr[0] <= sys_addr_in[FSZ-1+3:3];
    for (i=0; i<ADDR_A_DELAY; i+=1) begin
        buf_a_raddr[i+1] <= buf_a_raddr[i];
        hist_a_raddr[i+1] <= hist_a_raddr[i];
    end
    for (i=0; i<ADDR_B_DELAY; i+=1) begin
        buf_b_raddr[i+1] <= buf_b_raddr[i];
        hist_b_raddr[i+1] <= hist_b_raddr[i];
    end
end

always @(posedge adc_clk_i) begin
    adc_rstn_i <= adc_rstn_in;
    data_i <= data_in;
    enable_i <= enable_in;
    dvalid_i <= dvalid_in; 
    trig_i <= trig_in;
    fft_threshold_k_arg <= fft_threshold_k_in;
    fft_peak_start_arg <= fft_peak_start_in;
    fft_peak_minimum_arg <= fft_peak_minimum_in;
    fft_acq_up_i <= fft_acq_up_in;
    fft_acq_down_i <= fft_acq_down_in;
    sys_addr_i <= sys_addr_in;
    fft_index_flush_i <= fft_index_flush_in;
    fft_index_valid_i <= fft_index_valid_in;
    fft_hist_index_i <= fft_hist_index_in;
    fft_conf_data_i <= fft_conf_data_in;
end

localparam SYNC_FF = 2;

xpm_cdc_sync_rst #(
    .DEST_SYNC_FF (SYNC_FF)
) rstn_sync (
    .src_rst    (adc_rstn_i),
    .dest_clk   (clk_i),
    .dest_rst   (rstn)
);

xpm_cdc_single #(
    .DEST_SYNC_FF (SYNC_FF)
) trig_sync (
    .src_clk   (adc_clk_i),
    .src_in    (trig_i),
    .dest_clk  (clk_i),
    .dest_out  (fft_trig)
);

logic            up_in, fft_done;

xpm_cdc_single #(
    .DEST_SYNC_FF (SYNC_FF)
) done_sync (
    .src_clk   (clk_i),
    .src_in    (fft_done && up_in),
    .dest_clk  (adc_clk_i),
    .dest_out  (fft_done_o)
);

logic [  2-1: 0] fft_peak_ready;
logic            peak_up, peak_ready;
logic            peak_ready_trig = {fft_peak_ready[0], peak_ready} == 2'b01;

xpm_cdc_single #(
    .DEST_SYNC_FF (SYNC_FF)
) peak_ready_sync (
    .src_clk   (clk_i),
    .src_in    (fft_peak_ready == 2'b01),
    .dest_clk  (adc_clk_i),
    .dest_out  (fft_peak_ready_o)
);

logic [ FSZ : 0]        fft_count, fft_count_;
logic [ FSZ+DSZ-1: 0]   fft_sum, fft_sum_;
logic [ FSZ-1:0]        fft_peak_index_up, fft_peak_index_up_;
logic [ FSZ-1:0]        fft_peak_index_down, fft_peak_index_down_;
logic [ DSZ-1:0]        fft_peak_value_up, fft_peak_value_up_;
logic [ DSZ-1:0]        fft_peak_value_down, fft_peak_value_down_;
logic                   out_send, out_send_;

xpm_cdc_handshake #(
    .WIDTH          (FSZ+1 + FSZ+DSZ + FSZ + FSZ + DSZ + DSZ),
    .DEST_EXT_HSK   (0),
    .SRC_SYNC_FF    (SYNC_FF),
    .DEST_SYNC_FF   (SYNC_FF)
) out_sync (
    .src_clk        (clk_i),
    .src_in         ({fft_count,
                      fft_sum,
                      fft_peak_index_up,
                      fft_peak_index_down,
                      fft_peak_value_up,
                      fft_peak_value_down
                     }),
    .src_rcv        (out_recv),
    .src_send       (out_send),
    .dest_clk       (adc_clk_i),
    .dest_out       ({fft_count_,
                      fft_sum_,
                      fft_peak_index_up_,
                      fft_peak_index_down_,
                      fft_peak_value_up_,
                      fft_peak_value_down_
                     }),
    .dest_req       (out_req)
);

logic [ 16-1:0] fft_conf_data, fft_conf_data_, conf_data_i;
logic [  5-1:0] fft_nfft, fft_nfft_, fft_shift, fft_shift_;
logic [ 32-1:0] fft_length, fft_length2, fft_length_plus_two;
logic [ 32-1:0] fft_length_, fft_length2_, fft_length_plus_two_;
logic           conf_send;
logic           up_out, up_toggle, up_toggle_;
logic [FSZ-1:0] acq_up, acq_up_, acq_down, acq_down_;

xpm_memory_sdpram #(
    // .MEMORY_PRIMITIVE       ("block"),
    .MEMORY_SIZE            ((1<<FSZ)*DSZ),
    .ADDR_WIDTH_A           (FSZ),
    .ADDR_WIDTH_B           (FSZ),
    .CLOCKING_MODE          ("independent_clock"),
    .READ_LATENCY_B         (READ_LATENCY-ADDR_A_DELAY),
    .WRITE_MODE_B           ("read_first"),
    .READ_DATA_WIDTH_B      (DSZ),
    .WRITE_DATA_WIDTH_A     (DSZ),
    .BYTE_WRITE_WIDTH_A     (DSZ)
) fft_buf_up (
    .clka   (clk_i),
    .addra  (buf_a_waddr[WRITE_DELAY]),
    .dina   (buf_a_wdata[WRITE_DELAY]),
    .wea    (buf_a_we[WRITE_DELAY]),
    .ena    (1'b1),
    .clkb   (adc_clk_i),
    .addrb  (buf_a_raddr[ADDR_A_DELAY]),
    .doutb  (fft_rdata_up_o),
    .rstb   (1'b0),
    .regceb (1'b1),
    .enb    (1'b1)
);

xpm_memory_sdpram #(
    // .MEMORY_PRIMITIVE       ("block"),
    .MEMORY_SIZE            ((1<<FSZ)*DSZ),
    .ADDR_WIDTH_A           (FSZ),
    .ADDR_WIDTH_B           (FSZ),
    .CLOCKING_MODE          ("independent_clock"),
    .READ_LATENCY_B         (READ_LATENCY-ADDR_B_DELAY),
    .WRITE_MODE_B           ("read_first"),
    .READ_DATA_WIDTH_B      (DSZ),
    .WRITE_DATA_WIDTH_A     (DSZ),
    .BYTE_WRITE_WIDTH_A     (DSZ)
) fft_buf_down (
    .clka   (clk_i),
    .addra  (buf_b_waddr[WRITE_DELAY]),
    .dina   (buf_b_wdata[WRITE_DELAY]),
    .wea    (buf_b_we[WRITE_DELAY]),
    .ena    (1'b1),
    .clkb   (adc_clk_i),
    .addrb  (buf_b_raddr[ADDR_B_DELAY]),
    .doutb  (fft_rdata_down_o),
    .rstb   (1'b0),
    .regceb (1'b1),
    .enb    (1'b1)
);

xpm_memory_sdpram #(
    // .MEMORY_PRIMITIVE       ("block"),
    .MEMORY_SIZE            ((1<<HSZ)*FSZ),
    .ADDR_WIDTH_A           (HSZ),
    .ADDR_WIDTH_B           (HSZ),
    .CLOCKING_MODE          ("independent_clock"),
    .READ_LATENCY_B         (READ_LATENCY-ADDR_A_DELAY),
    .WRITE_MODE_B           ("read_first"),
    .READ_DATA_WIDTH_B      (FSZ),
    .WRITE_DATA_WIDTH_A     (FSZ),
    .BYTE_WRITE_WIDTH_A     (FSZ)
) fft_hist_up (
    .clka   (clk_i),
    .addra  (hist_a_waddr[WRITE_DELAY]),
    .dina   (hist_a_wdata[WRITE_DELAY]),
    .wea    (hist_a_we[WRITE_DELAY]),
    .ena    (1'b1),
    .clkb   (adc_clk_i),
    .addrb  (hist_a_raddr[ADDR_A_DELAY]),
    .doutb  (fft_hist_rdata_up_o),
    .rstb   (1'b0),
    .regceb (1'b1),
    .enb    (1'b1)
);

xpm_memory_sdpram #(
    // .MEMORY_PRIMITIVE       ("block"),
    .MEMORY_SIZE            ((1<<HSZ)*FSZ),
    .ADDR_WIDTH_A           (HSZ),
    .ADDR_WIDTH_B           (HSZ),
    .CLOCKING_MODE          ("independent_clock"),
    .READ_LATENCY_B         (READ_LATENCY-ADDR_B_DELAY),
    .WRITE_MODE_B           ("read_first"),
    .READ_DATA_WIDTH_B      (FSZ),
    .WRITE_DATA_WIDTH_A     (FSZ),
    .BYTE_WRITE_WIDTH_A     (FSZ)
) fft_hist_down (
    .clka   (clk_i),
    .addra  (hist_b_waddr[WRITE_DELAY]),
    .dina   (hist_b_wdata[WRITE_DELAY]),
    .wea    (hist_b_we[WRITE_DELAY]),
    .ena    (1'b1),
    .clkb   (adc_clk_i),
    .addrb  (hist_b_raddr[ADDR_B_DELAY]),
    .doutb  (fft_hist_rdata_down_o),
    .rstb   (1'b0),
    .regceb (1'b1),
    .enb    (1'b1)
);

always @(posedge adc_clk_i)
if (out_req) begin
    fft_count_o <= fft_count_;
    fft_sum_o <= fft_sum_;
    fft_peak_index_up_o <= fft_peak_index_up_;
    fft_peak_index_down_o <= fft_peak_index_down_;
    fft_peak_value_up_o <= fft_peak_value_up_;
    fft_peak_value_down_o <= fft_peak_value_down_;
end

always @(posedge clk_i) begin
    if (fft_frame_start) begin
        // if (up_in)
        //     frame_cnt <= frame_cnt + 1;
        // if (prev_hist_index != fft_hist_index)
        //     scan_frame_cnt <= scan_frame_cnt + 1;
        // prev_hist_index <= fft_hist_index;
    end
end


xpm_cdc_gray #(
    .WIDTH        (32),
    .DEST_SYNC_FF (SYNC_FF)
) (
    .src_clk      (clk_i),
    .src_in_bin   (frame_cnt),
    .dest_clk     (adc_clk_i),
    .dest_out_bin (frame_cnt_o)
);

xpm_cdc_gray #(
    .WIDTH        (32),
    .DEST_SYNC_FF (SYNC_FF)
) (
    .src_clk      (clk_i),
    .src_in_bin   (scan_frame_cnt),
    .dest_clk     (adc_clk_i),
    .dest_out_bin (scan_frame_cnt_o)
);

logic           fft_conf_dvalid;

logic [16+FSZ+FSZ-1:0] fft_conf_input = {fft_acq_up_i, fft_acq_down_i, fft_conf_data_i};
logic [16+FSZ+FSZ-1:0] fft_conf_reg, conf_data;

xpm_cdc_handshake #(
    .WIDTH          (16 + FSZ + FSZ),
    .DEST_EXT_HSK   (0),
    .SRC_SYNC_FF    (SYNC_FF),
    .DEST_SYNC_FF   (SYNC_FF)
) (
    .src_clk        (adc_clk_i),
    .src_in         (fft_conf_reg),
    .src_rcv        (conf_recv),
    .src_send       (conf_send),
    .dest_clk       (clk_i),
    .dest_out       (conf_data),
    .dest_req       (conf_req)
);

always @(posedge adc_clk_i) begin
    if (conf_recv)
        conf_send <= 0;
    else if (!conf_send && fft_conf_input != fft_conf_reg) begin
        conf_send <= 1;
        fft_conf_reg <= fft_conf_input;
    end
end

localparam RESET_DELAY = 4-1;
logic [RESET_DELAY : 0]  fft_rstn;
logic                    fft_rstn_i = fft_rstn[RESET_DELAY];
logic                    rstn_i = fft_rstn_i;

logic [ FSZ-1:0] padding_up, padding_down;
logic [ FSZ-1:0] padding_up_, padding_down_;

// Only allow one-time re-configuration after reset to avoid synchronization issue
always @(posedge clk_i) begin
    if (conf_req) begin
        fft_nfft_ <= conf_data[5-1:0];
        acq_up_ <= conf_data[16+FSZ+FSZ-1:16+FSZ];
        acq_down_ <= conf_data[16+FSZ-1:16];
        fft_conf_data_ <= conf_data[16-1:0];
    end

    fft_length_ <= 1<<fft_nfft;
    fft_shift_ <= FSZ - fft_nfft;
    fft_length_plus_two_ <= (1<<fft_nfft) + 2;
    padding_up_ <= (1<<fft_nfft) - acq_up;
    padding_down_ <= (1<<fft_nfft) - acq_down;

    // We need 2x amount of samples, one for Fup and one for Fdown
    if (fft_nfft < RSZ-1) begin
        fft_length2_ <= 1<<(fft_nfft+1);
        up_toggle_ <= 1;
    end else begin
        fft_length2_ <= 1<<fft_nfft;
        up_toggle_ <= 0;
    end

    fft_nfft <= fft_nfft_;
    acq_up <= acq_up_;
    acq_down <= acq_down_;
    fft_conf_data <= fft_conf_data_;
    fft_length <= fft_length_;
    fft_length2 <= fft_length2_;
    fft_shift <= fft_shift_;
    fft_length_plus_two <= fft_length_plus_two_;
    padding_up <= padding_up_;
    padding_down <= padding_down_;
    up_toggle <= up_toggle_;

    if (!rstn || conf_req) begin
        fft_rstn <= 0;
    end else
        fft_rstn <= {fft_rstn[RESET_DELAY-1:0], 1'b1};

    if (fft_rstn_i == 1'b0) begin
        fft_conf_dvalid <= 1;
    end else if (fft_conf_rdy) begin
        fft_conf_dvalid <= 0;
    end
end

logic [ ASZ-1:0]    fin_dout;
logic               fin_rd, fin_empty;
logic               padding_done;
logic [ FSZ-1:0]    padding_cnt;
logic               fin_rst = !adc_rstn_i || (trig_i && fft_done_o);

xpm_fifo_async #(
    .FIFO_WRITE_DEPTH(1<<QSZ),
    .WRITE_DATA_WIDTH(ASZ),
    .READ_DATA_WIDTH (ASZ),
    .FIFO_READ_LATENCY(0),
    .USE_ADV_FEATURES("1001"), // enables data_valid and overflow
    .READ_MODE       ("fwft")
) fifo_in (
    .rst             (fin_rst),
    .wr_clk          (adc_clk_i),
    .wr_en           (enable_i & dvalid_i),
    .din             (data_i),

    .rd_clk          (clk_i),
    .rd_en           (padding_done & fft_saxi_rdy & fin_rd),
    .dout            (fin_dout),
    .data_valid      (fin_dvalid),

    // .empty           (fin_empty),
    // .full            (fin_full)
    .overflow        (fin_full)
);

logic  [ HSZ-1:0] fft_hist_index_o;

xpm_fifo_async #(
    // .FIFO_MEMORY_TYPE("block"),
    .FIFO_WRITE_DEPTH(128),
    .WRITE_DATA_WIDTH(HSZ),
    .READ_DATA_WIDTH (HSZ),
    .FIFO_READ_LATENCY(0),
    .READ_MODE       ("fwft")
) fifo_index (
    .rst             (!adc_rstn_i || fft_index_flush_i),
    .wr_clk          (adc_clk_i),
    .wr_en           (fft_index_valid_i),
    .din             (fft_hist_index_i),

    .rd_clk          (clk_i),
    .rd_en           (peak_up != up_toggle && peak_ready_trig),
    .dout            (fft_hist_index_o)
);

logic  fft_we_one;
logic  fft_we_length_plus_one;
assign fft_saxi_last = fft_we_one || fft_we_length_plus_one;
assign fft_saxi_valid = (!padding_done || fin_dvalid) && fin_rd;
assign fft_data_i = fin_dout;

always @(posedge clk_i)
if (rstn_i == 1'b0) begin
    fft_we_cnt <= 0;
    fft_we_one <= 0;
    fft_we_length_plus_one <= 0;
    fin_rd <= 0;
    fft_done <= 1;
    up_in <= 1;
    padding_cnt <= 0;
    padding_done <= 0;
    // fft_saxi_last <= 0;
    // fft_saxi_valid <= 0;

end else begin

    if (fft_frame_start)
        scan_frame_cnt <= scan_frame_cnt + 1;

    if (fft_saxi_last)
        input_cnt <= input_cnt + 1;

    if (fin_full) begin
        // overflow_cnt <= overflow_cnt + 1;
    end

    // fft_data_i <= fin_dout;

    if (fft_trig && fft_done && up_in) begin
        fft_we_cnt <= fft_length2;
        fft_we_one <= 0;
        fft_we_length_plus_one <= 0;
        fin_rd <= 1;
        padding_cnt <= padding_up;
        padding_done <= 0;
        fft_done <= 0;
        // fft_saxi_valid <= 1;
        // fft_saxi_last <= 0;
    end else begin
        if (!fft_done && fft_saxi_valid && fft_saxi_rdy) begin
            frame_cnt <= frame_cnt + 1;
            if (fft_we_one) begin
                up_in <= 1;
                fin_rd <= 0;
            end else if (fft_we_length_plus_one) begin
                up_in <= 0;
                padding_cnt <= padding_down;
                padding_done <= 0;
            end else if (!padding_done) begin
                padding_cnt <= padding_cnt - 1;
                padding_done <= padding_cnt == 1;
            end
            fft_we_cnt <= fft_we_cnt - 1;
            fft_done <= fft_we_one;
            fft_we_one <= fft_we_cnt == 2;
            fft_we_length_plus_one <= fft_we_cnt == fft_length_plus_two;
        end
        //
        // fft_saxi_last <= fft_we_one || fft_we_length_plus_one;
        // fft_saxi_valid <= (!padding_done || fin_dvalid) && fin_rd;
    end
end

always @(posedge clk_i) begin

    buf_a_waddr[0] <= fft_wp_index;
    buf_b_waddr[0] <= fft_wp_index;
    buf_a_wdata[0] <= fft_maxi_data[DSZ-1:0];
    buf_b_wdata[0] <= fft_maxi_data[DSZ-1:0];
    buf_a_we[0] <= up_out && fft_maxi_valid && fft_maxi_rdy;
    buf_b_we[0] <= !up_out && fft_maxi_valid && fft_maxi_rdy;
    for (i=0; i<WRITE_DELAY; i+=1) begin
        buf_a_waddr[i+1] <= buf_a_waddr[i];
        buf_b_waddr[i+1] <= buf_b_waddr[i];
        buf_a_wdata[i+1] <= buf_a_wdata[i];
        buf_b_wdata[i+1] <= buf_b_wdata[i];
        buf_a_we[i+1] <= buf_a_we[i];
        buf_b_we[i+1] <= buf_b_we[i];
    end
end

always @(posedge clk_i) begin
    hist_a_waddr[0] <= fft_hist_index;
    hist_b_waddr[0] <= fft_hist_index;
    hist_a_wdata[0] <= fft_peak_index_up;
    hist_b_wdata[0] <= fft_peak_index_down;
    hist_a_we[0] <= peak_up && fft_peak_ready == 2'b01;
    hist_b_we[0] <= !peak_up && fft_peak_ready == 2'b01;
    for (i=0; i<WRITE_DELAY; i+=1) begin
        hist_a_waddr[i+1] <= hist_a_waddr[i];
        hist_b_waddr[i+1] <= hist_b_waddr[i];
        hist_a_wdata[i+1] <= hist_a_wdata[i];
        hist_b_wdata[i+1] <= hist_b_wdata[i];
        hist_a_waddr[i+1] <= hist_a_waddr[i];
        hist_b_waddr[i+1] <= hist_b_waddr[i];
        hist_a_we[i+1] <= hist_a_we[i];
        hist_b_we[i+1] <= hist_b_we[i];
    end
end

always @(posedge clk_i)
if (rstn_i == 1'b0) begin
    fft_wp <= 0;
    fft_wp_index <= 0;
    up_out <= 1;
end else if (fft_maxi_valid && fft_maxi_rdy) begin
    overflow_cnt <= overflow_cnt + 1;
    if (fft_maxi_last) begin
        // input_cnt <= input_cnt + 1;
        fft_wp <= 0;
        fft_wp_index <= 0;
        up_out <= up_out + up_toggle;
    end else begin
        fft_wp <= fft_wp + 1;
        fft_wp_index <= fft_wp_reversed >> fft_shift;
    end
end

assign fft_data = fft_maxi_data[DSZ-1:0];
assign fft_data_abs = fft_data[DSZ-1] ? -fft_data : fft_data;
assign fft_data_valid = fft_wp_index>=fft_peak_start_arg && fft_wp_index<fft_length[FSZ:1] && fft_data_abs>fft_peak_minimum_arg;

always @(posedge clk_i)
if (rstn_i == 1'b0) begin
    fft_peak_ready <= 2'b11;
    peak_up <= 1;
end else begin

    if (out_recv)
        out_send <= 0;
    else if (out_send_)
        out_send <= 1;

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
            out_send_ <= 1;
        end else
            out_send_ <= 0;
    end

    fft_peak_ready <= {fft_peak_ready[0], peak_ready};
end

xpm_fifo_axis #(
    .TDATA_WIDTH        (32),
    .TUSER_WIDTH        (FSZ+1),
    .FIFO_DEPTH         (128),
    .USE_ADV_FEATURES   (16'h0000)   // Standard mode
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
    .maxi_last      (fft_peak_maxi_last & fft_peak_maxi_valid),
    .threshold_k_sq (fft_threshold_k_arg),
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

xpm_cdc_array_single #(
    .WIDTH          (6),
    .DEST_SYNC_FF   (SYNC_FF)
) status_sync (
    .src_clk    (clk_i),
    .src_in     ({fft_in_halt
                   , fft_out_halt
                   , fft_status_halt
                   , fft_frame_start
                   , fft_tlast_missing
                   , fft_tlast_unexp}),
    .dest_clk   (adc_clk_i),
    .dest_out   (status_o)
);

endmodule
