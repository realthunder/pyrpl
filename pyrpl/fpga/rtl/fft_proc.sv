module fft_proc #(
  parameter ASZ,        // ADC input sample width
  parameter DSZ,        // FFT_output width
  parameter FSZ,        // FFT transform length 2^FSZ
  parameter FRAC = 0,   // sub-bin interpolation fractional bits (peak index = Q(FSZ).FRAC); 0=off
  parameter FSSR,       // FFT super sample size
  parameter FFT_IMPL = 3, // 1=LogiCORE, 2=HLS SSR (DIT), 3=IP SSR (DIF)
  parameter RSZ,        // RAM size 2^RSZ
  parameter HSZ,        // fft history buffer size 2^HSZ (Note: consider word size of 32bit, better not exceed 64KBytes in total)
  parameter QSZ,        // FFT queue size 2^QSZ
  parameter READ_DELAY,  // memory output read delay
  parameter HIST_BLOCK_SIZE = 183,  // DMA packet size in detection words (183+1 hdr = 184×8 = 1472 B = one Ethernet MTU)
  parameter [3:0] CHANNEL_ID = 0   // 4-bit channel tag stamped into header bits [63:60]
)(
  input logic             adc_clk_i,
  input logic             clk_i,
  input logic             adc_rstn_in,
  input logic  [ ASZ-1:0] data_in,
  input logic             enable_in,
  input logic             dvalid_in, 
  input logic             trig_in,
  input logic             fft_parallel_in,

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

  output logic [ IDX-1:0] fft_hist_rdata_up_o,
  output logic [ IDX-1:0] fft_hist_rdata_down_o,

  output logic [ 63:0]    m_dma_tdata,
  output logic            m_dma_tvalid,
  input  logic            m_dma_tready,
  output logic            m_dma_tlast,

  output logic [  6-1: 0] status_o,
  output logic            fft_done_o,
  output logic            fft_peak_ready_o,

  output logic [ IDX-1:0] fft_peak_index_up_o,
  output logic [ IDX-1:0] fft_peak_index_down_o,
  output logic [ DSZ-1:0] fft_peak_value_up_o,
  output logic [ DSZ-1:0] fft_peak_value_down_o,

  output logic [ 32-1: 0] fft_we_cnt,

  output logic [ 32-1: 0] frame_cnt_o,
  output logic [ 32-1: 0] scan_frame_cnt_o,

  input logic  [  16-1:0] fft_conf_data_in,

  output logic [  32-1:0] overflow_cnt_o
);

localparam SSR_BITS = $clog2(FSSR);

// Output peak-index width: integer bin (FSZ) plus FRAC sub-bin fractional bits.
// The HLS peak detector emits the index as unsigned Q(FSZ).FRAC fixed-point
// (k_interp = peak_bin + parabolic offset). All downstream peak-INDEX carriers
// (inter-channel registers, history BRAM, point-cloud DMA word) are IDX-wide.
// Input/addressing indices (peak_start, FFT buffer addresses, acq/wait counts,
// scan-position hist_index) stay FSZ — only the detected bin value carries frac.
localparam IDX = FSZ + FRAC;

// Runtime FFT length is supported only by the LogiCORE IP (FFT_IMPL==1), or when
// explicitly opted in via the FFT_RUNTIME_NFFT define. Otherwise nfft and every
// derived size fold to compile-time constants via the real_nfft() function below:
// when RUNTIME_NFFT is 0 (a localparam) the function returns the constant NFFT_FIXED
// regardless of its argument, so all the `1<<real_nfft(...)` shifts and fft_shift
// collapse at elaboration and the runtime decode logic / fft_nfft register are pruned.
`ifdef FFT_RUNTIME_NFFT
localparam FFT_RT_DEF = 1;
`else
localparam FFT_RT_DEF = 0;
`endif
localparam RUNTIME_NFFT = (FFT_IMPL == 1) || FFT_RT_DEF;
localparam [5-1:0] NFFT_FIXED = FSZ - SSR_BITS;   // internal (sub-FFT) nfft at full size

// Effective internal (sub-FFT) nfft: the runtime value rt when runtime length is
// enabled, else the compile-time constant. Inlined + constant-folded by synthesis.
function automatic [5-1:0] real_nfft(input [5-1:0] rt);
    real_nfft = RUNTIME_NFFT ? rt : NFFT_FIXED;
endfunction

localparam READ_A_DELAY = READ_DELAY - 3;
localparam READ_B_DELAY = READ_DELAY - 3;

logic           fft_in_halt, fft_out_halt, fft_status_halt;
logic           fft_frame_start, fft_tlast_missing, fft_tlast_unexp;

logic [ 16-1:0] overflow_cnt;
logic [ 16-1:0] input_cnt;
assign overflow_cnt_o = {input_cnt, overflow_cnt};

logic [ 32-1: 0] frame_cnt;
logic [ 32-1: 0] scan_frame_cnt;

logic [ HSZ-1:0] fft_hist_index;
logic [ HSZ-1:0] prev_hist_index;

logic [ASZ*FSSR-1: 0]   fft_data_i;

logic                   fft_saxi_last;
logic                   fft_saxi_rdy;
logic                   fft_saxi_valid;

logic [ IDX-1: 0]       fft_peak_idx;
logic [ DSZ-1: 0]       fft_peak;

logic [ DSZ*FSSR-1:0]   fft_maxi_data;
logic                   fft_maxi_valid;
logic                   fft_maxi_rdy;
logic                   fft_maxi_last;
logic [ FSZ-1: 0]       fft_wp;

logic               adc_rstn_i;

logic [ 16-1:0]     fft_conf_data, fft_conf_data_, conf_data_i;
logic               fft_conf_rdy;
logic [  5-1:0]     fft_nfft, fft_nfft_, fft_shift, fft_shift_;
logic [ 32-1:0]     fft_length, fft_length2, fft_length_plus_two;
logic [ 32-1:0]     fft_length_, fft_length2_, fft_length_plus_two_;
logic               conf_send, conf_recv, conf_req;
logic               up_out, up_toggle, up_toggle_;
logic [FSZ-1:0]     acq_up, acq_up_, acq_down, acq_down_;

logic [ 2-1: 0]     fft_peak_ready;
logic               peak_up, peak_ready;
logic               peak_ready_pretrig = {fft_peak_ready[0], peak_ready} == 2'b01;
logic               peak_ready_trig = fft_peak_ready == 2'b01;

logic [ IDX-1:0]    fft_peak_index_up, fft_peak_index_up_;
logic [ IDX-1:0]    fft_peak_index_down, fft_peak_index_down_;
logic [ DSZ-1:0]    fft_peak_value_up, fft_peak_value_up_;
logic [ DSZ-1:0]    fft_peak_value_down, fft_peak_value_down_;
logic               out_recv, out_send, out_send_;

logic [ FSZ-1: 0]   buf_a_waddr;
logic [ FSZ-1: 0]   buf_b_waddr;
logic [ DSZ-1: 0]   buf_a_wdata [0:FSSR-1];
logic [ DSZ-1: 0]   buf_b_wdata [0:FSSR-1];
logic               buf_a_we   ;
logic               buf_b_we   ;
logic [ FSZ-1: 0]   buf_a_raddr;
logic [ FSZ-1: 0]   buf_b_raddr;

logic [ HSZ-1: 0]   hist_a_waddr;
logic [ HSZ-1: 0]   hist_b_waddr;
logic [ IDX-1: 0]   hist_a_wdata;
logic [ IDX-1: 0]   hist_b_wdata;
logic               hist_a_we   ;
logic               hist_b_we   ;
logic [ HSZ-1: 0]   hist_a_raddr;
logic [ HSZ-1: 0]   hist_b_raddr;


genvar s, k;
integer j;

// Bit-reversed read addresses: Vivado can't part-select the result of a
// streaming operator ({<<{}}[...]), so we materialise the reversal explicitly.
logic [FSZ-1:0] buf_a_raddr_bitrev;
logic [FSZ-1:0] buf_b_raddr_bitrev;
generate
for (k = 0; k < FSZ; k++) begin : gen_raddr_bitrev
    assign buf_a_raddr_bitrev[k] = buf_a_raddr[FSZ-1-k];
    assign buf_b_raddr_bitrev[k] = buf_b_raddr[FSZ-1-k];
end
endgenerate

// Vivado doesn't allow part-selects on shift expressions; use a wire.
logic [32-1:0] fft_end_idx;
assign fft_end_idx = fft_length << SSR_BITS;

always @(posedge adc_clk_i) begin
    buf_a_raddr  <= sys_addr_in[FSZ-1+3:3];
    buf_b_raddr  <= sys_addr_in[FSZ-1+3:3];
    hist_a_raddr <= sys_addr_in[HSZ-1+2:2];
    hist_b_raddr <= sys_addr_in[HSZ-1+2:2];
end

always @(posedge clk_i) begin
    buf_a_waddr <= fft_wp;
    buf_b_waddr <= fft_wp;
    buf_a_we    <=  up_out && fft_maxi_valid && fft_maxi_rdy;
    buf_b_we    <= !up_out && fft_maxi_valid && fft_maxi_rdy;
    for (j=0; j<FSSR; j+=1) begin
        buf_a_wdata[j] <= fft_maxi_data[j*DSZ +: DSZ];
        buf_b_wdata[j] <= fft_maxi_data[j*DSZ +: DSZ];
    end
end

always @(posedge clk_i) begin
    hist_a_waddr <=  fft_hist_index;
    hist_b_waddr <=  fft_hist_index;
    hist_a_wdata <=  fft_peak_index_up;
    hist_b_wdata <=  fft_peak_index_down;
    hist_a_we    <=  peak_up && peak_ready_trig;
    hist_b_we    <= !peak_up && peak_ready_trig;
end

logic  [ ASZ-1:0] data_i;
logic             enable_i;
logic             dvalid_i; 
logic             trig_i;

logic             fft_parallel_i, fft_parallel;

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

always @(posedge adc_clk_i) begin
    adc_rstn_i <= adc_rstn_in;
    data_i <= data_in;
    enable_i <= enable_in;
    dvalid_i <= dvalid_in; 
    trig_i <= trig_in;
    fft_parallel_i <= fft_parallel_in;
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
logic      rstn;

xpm_cdc_sync_rst #(
    .DEST_SYNC_FF (SYNC_FF)
) rstn_sync (
    .src_rst    (adc_rstn_i),
    .dest_clk   (clk_i),
    .dest_rst   (rstn)
);

logic     fft_trig;

xpm_cdc_array_single #(
    .WIDTH        (2),
    .DEST_SYNC_FF (SYNC_FF)
) input_arg_sync (
    .src_clk   (adc_clk_i),
    .src_in    ({fft_parallel_i, trig_i}),
    .dest_clk  (clk_i),
    .dest_out  ({fft_parallel, fft_trig})
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

logic rstn_i;

xpm_cdc_pulse #(
    .DEST_SYNC_FF (SYNC_FF)
) peak_ready_sync (
    .src_clk    (clk_i),
    .src_rst    (~rstn_i),
    .src_pulse  (fft_peak_ready[1]),
    .dest_clk   (adc_clk_i),
    .dest_rst   (~adc_rstn_i),
    .dest_pulse (fft_peak_ready_o)
);

xpm_cdc_handshake #(
    .WIDTH          (IDX + IDX + DSZ + DSZ),
    .DEST_EXT_HSK   (0),
    .SRC_SYNC_FF    (SYNC_FF),
    .DEST_SYNC_FF   (SYNC_FF)
) out_sync (
    .src_clk        (clk_i),
    .src_in         ({fft_peak_index_up,
                      fft_peak_index_down,
                      fft_peak_value_up,
                      fft_peak_value_down
                     }),
    .src_rcv        (out_recv),
    .src_send       (out_send),
    .dest_clk       (adc_clk_i),
    .dest_out       ({fft_peak_index_up_,
                      fft_peak_index_down_,
                      fft_peak_value_up_,
                      fft_peak_value_down_
                     }),
    .dest_req       (out_req)
);

logic [DSZ-1:0]             fft_rdata_up    [0:FSSR-1];
logic [DSZ-1:0]             fft_rdata_down  [0:FSSR-1];
logic [FSZ-SSR_BITS-1: 0]  buf_a_raddr_reversed;
logic [FSZ-SSR_BITS-1: 0]  buf_b_raddr_reversed;

// BRAM channel/address split from bin index k.
// buf_a_raddr_bitrev[j] = buf_a_raddr[FSZ-1-j], so:
//   bitrev[0]     = k[FSZ-1]  (MSB of k)
//   bitrev[FSZ-1] = k[0]      (LSB of k)
//
// FFT_IMPL==2 (HLS SSR, DIT): lanes hold lower/upper spectrum halves.
//   channel = k[FSZ-1]              → buf_a_raddr_bitrev[SSR_BITS-1:0]
//   address = bit_rev(k>>SSR_BITS)  → buf_a_raddr_bitrev[FSZ-1:SSR_BITS]
//
// FFT_IMPL==3 (IP SSR, DIF) and 5 (direct hls::fft): lanes hold bins grouped by
// k mod FSSR, with bit-reversed per-lane address (LogiCORE bit_reversed_order).
//   channel = k[SSR_BITS-1:0]       → buf_a_raddr[SSR_BITS-1:0]  (no bit-reversal)
//   address = bit_rev(k>>SSR_BITS)  → buf_a_raddr_bitrev[FSZ-SSR_BITS-1:0]
//
// FFT_IMPL==4 (native-SSR xfft): per PG109, SSR>1 fixed-point supports ONLY
// NATURAL output order (bit_reversed_order is silently unavailable). So bin k is at
// the natural position — lane = k mod FSSR, address = k>>SSR_BITS, NO bit-reversal.
//   channel = k[SSR_BITS-1:0]       → buf_a_raddr[SSR_BITS-1:0]
//   address = k>>SSR_BITS           → buf_a_raddr[FSZ-1:SSR_BITS]  (NOT reversed)
generate
if (FSSR == 1) begin
    assign fft_rdata_up_o       = fft_rdata_up  [0];
    assign fft_rdata_down_o     = fft_rdata_down[0];
    assign buf_a_raddr_reversed = buf_a_raddr_bitrev >> fft_shift;
    assign buf_b_raddr_reversed = buf_b_raddr_bitrev >> fft_shift;
end else if (FFT_IMPL == 4) begin
    // Native-SSR xfft: NATURAL output order (PG109: SSR>1 fixed-point is natural-only).
    // lane = k mod FSSR; address = k>>SSR_BITS with NO bit-reversal.
    assign fft_rdata_up_o       = fft_rdata_up  [buf_a_raddr[SSR_BITS-1:0]];
    assign fft_rdata_down_o     = fft_rdata_down[buf_b_raddr[SSR_BITS-1:0]];
    assign buf_a_raddr_reversed = buf_a_raddr[FSZ-1:SSR_BITS];
    assign buf_b_raddr_reversed = buf_b_raddr[FSZ-1:SSR_BITS];
end else if (FFT_IMPL == 3 || FFT_IMPL == 5) begin
    // DIF: channel = k mod FSSR — low SSR_BITS of the bin index, no reversal needed.
    assign fft_rdata_up_o       = fft_rdata_up  [buf_a_raddr[SSR_BITS-1:0]];
    assign fft_rdata_down_o     = fft_rdata_down[buf_b_raddr[SSR_BITS-1:0]];
    assign buf_a_raddr_reversed = buf_a_raddr_bitrev[FSZ-SSR_BITS-1:0];
    assign buf_b_raddr_reversed = buf_b_raddr_bitrev[FSZ-SSR_BITS-1:0];
end else begin
    // DIT: channel = upper SSR_BITS of k, accessed via its bit-reversed position.
    assign fft_rdata_up_o       = fft_rdata_up  [buf_a_raddr_bitrev[SSR_BITS-1:0]];
    assign fft_rdata_down_o     = fft_rdata_down[buf_b_raddr_bitrev[SSR_BITS-1:0]];
    assign buf_a_raddr_reversed = buf_a_raddr_bitrev[FSZ-1:SSR_BITS];
    assign buf_b_raddr_reversed = buf_b_raddr_bitrev[FSZ-1:SSR_BITS];
end
endgenerate

generate
for(s=0; s<FSSR; s+=1) begin
    xpm_memory_sdpram #(
        .MEMORY_SIZE            ((1<<(FSZ-SSR_BITS))*DSZ),
        .ADDR_WIDTH_A           (FSZ-SSR_BITS),
        .ADDR_WIDTH_B           (FSZ-SSR_BITS),
        .CLOCKING_MODE          ("independent_clock"),
        .READ_LATENCY_B         (READ_A_DELAY),
        .WRITE_MODE_B           ("read_first"),
        .READ_DATA_WIDTH_B      (DSZ),
        .WRITE_DATA_WIDTH_A     (DSZ),
        .BYTE_WRITE_WIDTH_A     (DSZ)
    ) fft_buf_up (
        .clka   (clk_i),
        .addra  (buf_a_waddr),
        .dina   (buf_a_wdata[s]),
        .wea    (buf_a_we),
        .ena    (1'b1),
        .clkb   (adc_clk_i),
        .addrb  (buf_a_raddr_reversed),
        .doutb  (fft_rdata_up[s]),
        .rstb   (1'b0),
        .regceb (1'b1),
        .enb    (1'b1)
    );

    xpm_memory_sdpram #(
        .MEMORY_SIZE            ((1<<(FSZ-SSR_BITS))*DSZ),
        .ADDR_WIDTH_A           (FSZ-SSR_BITS),
        .ADDR_WIDTH_B           (FSZ-SSR_BITS),
        .CLOCKING_MODE          ("independent_clock"),
        .READ_LATENCY_B         (READ_B_DELAY),
        .WRITE_MODE_B           ("read_first"),
        .READ_DATA_WIDTH_B      (DSZ),
        .WRITE_DATA_WIDTH_A     (DSZ),
        .BYTE_WRITE_WIDTH_A     (DSZ)
    ) fft_buf_down (
        .clka   (clk_i),
        .addra  (buf_b_waddr),
        .dina   (buf_b_wdata[s]),
        .wea    (buf_b_we),
        .ena    (1'b1),
        .clkb   (adc_clk_i),
        .addrb  (buf_b_raddr_reversed),
        .doutb  (fft_rdata_down[s]),
        .rstb   (1'b0),
        .regceb (1'b1),
        .enb    (1'b1)
    );
end
endgenerate


`ifdef STORE_HIST

xpm_memory_sdpram #(
    // .MEMORY_PRIMITIVE       ("block"),
    .MEMORY_SIZE            ((1<<HSZ)*IDX),
    .ADDR_WIDTH_A           (HSZ),
    .ADDR_WIDTH_B           (HSZ),
    .CLOCKING_MODE          ("independent_clock"),
    .READ_LATENCY_B         (READ_A_DELAY),
    .WRITE_MODE_B           ("read_first"),
    .READ_DATA_WIDTH_B      (IDX),
    .WRITE_DATA_WIDTH_A     (IDX),
    .BYTE_WRITE_WIDTH_A     (IDX)
) fft_hist_up (
    .clka   (clk_i),
    .addra  (hist_a_waddr),
    .dina   (hist_a_wdata),
    .wea    (hist_a_we),
    .ena    (1'b1),
    .clkb   (adc_clk_i),
    .addrb  (hist_a_raddr),
    .doutb  (fft_hist_rdata_up_o),
    .rstb   (1'b0),
    .regceb (1'b1),
    .enb    (1'b1)
);

xpm_memory_sdpram #(
    // .MEMORY_PRIMITIVE       ("block"),
    .MEMORY_SIZE            ((1<<HSZ)*IDX),
    .ADDR_WIDTH_A           (HSZ),
    .ADDR_WIDTH_B           (HSZ),
    .CLOCKING_MODE          ("independent_clock"),
    .READ_LATENCY_B         (READ_B_DELAY),
    .WRITE_MODE_B           ("read_first"),
    .READ_DATA_WIDTH_B      (IDX),
    .WRITE_DATA_WIDTH_A     (IDX),
    .BYTE_WRITE_WIDTH_A     (IDX)
) fft_hist_down (
    .clka   (clk_i),
    .addra  (hist_b_waddr),
    .dina   (hist_b_wdata),
    .wea    (hist_b_we),
    .ena    (1'b1),
    .clkb   (adc_clk_i),
    .addrb  (hist_b_raddr),
    .doutb  (fft_hist_rdata_down_o),
    .rstb   (1'b0),
    .regceb (1'b1),
    .enb    (1'b1)
);

`else
// History BRAM disabled: drive the readback outputs to 0 so they are not undriven
// nets feeding the scope sys_rdata mux (undriven X-prone bits corrupt the readback).
assign fft_hist_rdata_up_o   = '0;
assign fft_hist_rdata_down_o = '0;

`endif

always @(posedge adc_clk_i)
if (out_req) begin
    fft_peak_index_up_o <= fft_peak_index_up_;
    fft_peak_index_down_o <= fft_peak_index_down_;
    fft_peak_value_up_o <= fft_peak_value_up_;
    fft_peak_value_down_o <= fft_peak_value_down_;
end

// scan_frame_cnt: counts scan-position changes. fft_hist_index changes only at
// scan-step rate, so sampling it at fft_frame_start is fine. frame_cnt moved to the
// peak_ready_pretrig block below (gated by peak_up) — up_in is the INPUT-side phase
// and fft_frame_start fires at FFT-processing start, so the two desync once the FFT
// pipeline fills and frame_cnt froze.
always @(posedge clk_i) begin
    if (fft_frame_start) begin
        if (prev_hist_index != fft_hist_index)
            scan_frame_cnt <= scan_frame_cnt + 1;
        prev_hist_index <= fft_hist_index;
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

logic [16+FSZ+FSZ-1:0] fft_conf_input = {fft_acq_up_i>>SSR_BITS, fft_acq_down_i>>SSR_BITS, fft_conf_data_i};
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
assign                   rstn_i = fft_rstn_i;

logic [ FSZ-1:0] padding_up, padding_down;
logic [ FSZ-1:0] padding_up_, padding_down_;

// Only allow one-time re-configuration after reset to avoid synchronization issue
always @(posedge clk_i) begin
    if (conf_req) begin
        // real_nfft() folds to NFFT_FIXED when !RUNTIME_NFFT (the conf_data decode is
        // then dead and pruned); the LogiCORE/opt-in runtime path keeps the live value.
        fft_nfft_ <= real_nfft(conf_data[5-1:0] - SSR_BITS);
        acq_up_ <= conf_data[16+FSZ+FSZ-1:16+FSZ];
        acq_down_ <= conf_data[16+FSZ-1:16];
        fft_conf_data_ <= conf_data[16-1:0];
    end

    fft_length_ <= 1 << real_nfft(fft_nfft);
    fft_shift_ <= FSZ - real_nfft(fft_nfft);
    // Stage 1 does the shift only (same op as fft_length_); the +2 add moves
    // to stage 2 below, breaking the shift+adder carry chain from fft_nfft.
    fft_length_plus_two_ <= 1 << real_nfft(fft_nfft);
    padding_up_ <= (1 << real_nfft(fft_nfft)) - acq_up;
    padding_down_ <= (1 << real_nfft(fft_nfft)) - acq_down;

    // We need 2x amount of samples, one for Fup and one for Fdown
    if (!fft_parallel) begin
        fft_length2_ <= 1 << (real_nfft(fft_nfft) + 1'b1);
        up_toggle_ <= 1;
    end else begin
        fft_length2_ <= 1 << real_nfft(fft_nfft);
        up_toggle_ <= 0;
    end

    fft_nfft <= fft_nfft_;
    acq_up <= acq_up_;
    acq_down <= acq_down_;
    fft_conf_data <= fft_conf_data_;
    fft_length <= fft_length_;
    fft_length2 <= fft_length2_;
    fft_shift <= fft_shift_;
    fft_length_plus_two <= fft_length_plus_two_ + 2;
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

logic [ FSSR*ASZ-1:0]   fin_dout;
logic                   fin_rd, fin_dvalid;
logic                   padding_done;
logic [ FSZ-1:0]        padding_cnt;
logic                   fin_rst = !adc_rstn_i || (trig_i && fft_done_o);

xpm_fifo_async #(
    .FIFO_WRITE_DEPTH(1<<QSZ),
    .WRITE_DATA_WIDTH(ASZ),
    .READ_DATA_WIDTH (ASZ*FSSR),
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
    .rd_en           (peak_up != up_toggle && peak_ready_pretrig),
    .dout            (fft_hist_index_o)
);

logic  fft_we_one;
logic  fft_we_length_plus_one;
assign fft_saxi_last = fft_we_one || fft_we_length_plus_one;
assign fft_saxi_valid = (!padding_done || fin_dvalid) && fin_rd;
assign fft_data_i = padding_done ? fin_dout: '0;

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

end else begin

    if (fin_full) begin
        overflow_cnt <= overflow_cnt + 1;
    end

    if (fft_trig && fft_done && up_in) begin
        fft_we_cnt <= fft_length2;
        fft_we_one <= 0;
        fft_we_length_plus_one <= 0;
        fin_rd <= 1;
        padding_cnt <= padding_up;
        padding_done <= 0;
        fft_done <= 0;
    end else begin
        if (!fft_done && fft_saxi_valid && fft_saxi_rdy) begin
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
    end
end

always @(posedge clk_i)
if (rstn_i == 1'b0) begin
    fft_wp  <= 0;
    up_out  <= 1;
end else if (fft_maxi_valid && fft_maxi_rdy) begin
    if (fft_maxi_last) begin
        fft_wp    <= 0;
        up_out    <= up_out + up_toggle;
    end else begin
        fft_wp <= fft_wp + 1;
    end
end

always @(posedge clk_i)
if (rstn_i == 1'b0) begin
    fft_peak_ready <= 2'b11;
    peak_up <= 1;
end else begin

    if (out_recv)
        out_send <= 0;
    else if (out_send_)
        out_send <= 1;

    if (fft_index_flush_i)
        peak_up <= 1;
    else if (peak_ready_pretrig)
        peak_up <= peak_up + up_toggle;

    if (peak_ready_pretrig) begin

        fft_hist_index <= fft_hist_index_o;

        if (peak_up) begin
            frame_cnt <= frame_cnt + 1;   // coherent up-frame count (was up_in @ fft_frame_start)
            fft_peak_index_up <= fft_peak_idx;
            fft_peak_value_up <= fft_peak;
        end else begin
            fft_peak_index_down <= fft_peak_idx;
            fft_peak_value_down <= fft_peak;
        end
        out_send_ <= 1;
    end else
        out_send_ <= 0;

    fft_peak_ready <= {fft_peak_ready[0], peak_ready};
end

// FSSR*DSZ-wide FIFO between FFT output and HLS peak detector.
// FSSR*DSZ must be a multiple of 8; satisfied for even FSSR and DSZ=16.
localparam PEAK_IN_WIDTH = FSSR * DSZ;

logic [PEAK_IN_WIDTH-1:0] peak_in_data;
logic                     peak_in_valid;
logic                     peak_in_ready;
logic                     peak_in_last;

xpm_fifo_axis #(
    .TDATA_WIDTH      (PEAK_IN_WIDTH),
    .FIFO_DEPTH       (32),
    .USE_ADV_FEATURES (16'h0000)
) fifo_peak_in (
    .s_aclk          (clk_i),
    .s_aresetn       (rstn_i),

    .s_axis_tdata    (fft_maxi_data),
    .s_axis_tvalid   (fft_maxi_valid),
    .s_axis_tready   (fft_maxi_rdy),
    .s_axis_tlast    (fft_maxi_last),

    .m_axis_tdata    (peak_in_data),
    .m_axis_tvalid   (peak_in_valid),
    .m_axis_tready   (peak_in_ready),
    .m_axis_tlast    (peak_in_last)
);

logic [63:0] peak_out_data;
logic        peak_out_valid;

peak_detector_bd_wrapper pd_i (
    .aclk            (clk_i),
    .aresetn         (rstn_i),

    .s_axis_tdata    (peak_in_data),
    .s_axis_tvalid   (peak_in_valid),
    .s_axis_tready   (peak_in_ready),
    .s_axis_tlast    (peak_in_last),

    .m_axis_tdata    (peak_out_data),
    .m_axis_tvalid   (peak_out_valid),
    .m_axis_tready   (1'b1),
    .m_axis_tlast    (),
    .ap_done         (),

    .threshold_k_sq  (fft_threshold_k_arg),
    .start_index     (fft_peak_start_arg),
    .end_index       (fft_end_idx[FSZ:1]),
    .data_min        (fft_peak_minimum_arg),
    .nfft            (fft_nfft+SSR_BITS)
);

logic  fft_peak_valid = peak_out_data[DSZ];
// k_interp occupies [DSZ+IDX : DSZ+1] (IDX = FSZ+FRAC bits), value in [DSZ-1:0].
assign fft_peak_idx   = fft_peak_valid ? peak_out_data[DSZ + IDX : DSZ + 1] : 0;
assign fft_peak       = peak_out_data[DSZ-1 : 0];
assign peak_ready     = peak_out_valid;

// --- DMA point cloud output ---
// Packet = 1 header word + HIST_BLOCK_SIZE data words, tlast on last data word.
//
// Header word (64-bit):
//   [31:0]           frame_cnt       (cumulative frame counter)
//   [31+HSZ:32]      fft_hist_index  (scan-position tag for this block)
//   [59:32+HSZ]      reserved 0
//   [63:60]          CHANNEL_ID      (4-bit channel tag: 0=fft_a, 1=fft_b)
//
// Data word (64-bit):
//   [IDX-1:0]       peak_bin_up    (k_interp, Q(FSZ).FRAC fixed-point)
//   [2*IDX-1:IDX]   peak_bin_down  (k_interp, Q(FSZ).FRAC fixed-point)
//   [63:2*IDX]      reserved 0
//   IDX = FSZ+FRAC. Host recovers each bin = field / 2^FRAC. Requires 2*IDX <= 64.
//
// Emit condition: sequential (up_toggle=1) → after down chirp (both peaks fresh);
//                 parallel  (up_toggle=0) → every frame.
// On the first detection of a new block the header is sent this cycle and the
// paired data word is held one cycle (dma_data_pending).
// fft_index_flush_i closes any in-progress packet cleanly (emits tlast) so the
// downstream FIFO and dma_s2mm stay consistent with no PS intervention.
localparam DMA_HDR_RSVD = 64 - 4 - 32 - HSZ;  // [63:60]=channel_id [59:32+HSZ]=rsvd [31+HSZ:32]=hist_index [31:0]=frame_cnt
localparam DMA_DAT_RSVD = 64 - 2*IDX;

logic [15:0]    dma_data_sent;      // data words sent in current packet (0..HIST_BLOCK_SIZE)
logic           dma_data_pending;   // first data word buffered after header
logic [IDX-1:0] dma_saved_peak_up;
logic [IDX-1:0] dma_saved_peak_down;

logic [63:0]    dma_wr_data;
logic           dma_wr_en;
logic           dma_wr_tlast;

wire dma_emit = peak_ready_trig && (peak_up || !up_toggle);

always @(posedge clk_i) begin
    dma_wr_en <= 0;
    if (!rstn_i) begin
        dma_data_sent    <= 0;
        dma_data_pending <= 0;
    end else if (fft_index_flush_i) begin
        // Close any in-progress packet so the AXI-S FIFO receives a proper tlast.
        if (dma_data_pending) begin
            dma_wr_data  <= { {DMA_DAT_RSVD{1'b0}}, dma_saved_peak_down, dma_saved_peak_up };
            dma_wr_tlast <= 1;
            dma_wr_en    <= 1;
        end else if (dma_data_sent != 0) begin
            dma_wr_data  <= 64'h0;
            dma_wr_tlast <= 1;
            dma_wr_en    <= 1;
        end
        dma_data_sent    <= 0;
        dma_data_pending <= 0;
    end else if (dma_data_pending) begin
        // Emit buffered first data word (header was sent last cycle)
        dma_wr_data      <= { {DMA_DAT_RSVD{1'b0}}, dma_saved_peak_down, dma_saved_peak_up };
        dma_wr_tlast     <= (HIST_BLOCK_SIZE == 1);
        dma_wr_en        <= 1;
        dma_data_pending <= 0;
        dma_data_sent    <= (HIST_BLOCK_SIZE == 1) ? 16'd0 : 16'd1;
    end else if (dma_emit) begin
        if (dma_data_sent == 0) begin
            // First detection of new block: send header, buffer data for next cycle
            dma_wr_data         <= { CHANNEL_ID, {DMA_HDR_RSVD{1'b0}}, fft_hist_index, frame_cnt };
            dma_wr_tlast        <= 0;
            dma_wr_en           <= 1;
            dma_saved_peak_up   <= fft_peak_index_up;
            dma_saved_peak_down <= fft_peak_index_down;
            dma_data_pending    <= 1;
        end else begin
            // Subsequent detection: emit data word directly
            dma_wr_data  <= { {DMA_DAT_RSVD{1'b0}}, fft_peak_index_down, fft_peak_index_up };
            dma_wr_tlast <= (dma_data_sent == HIST_BLOCK_SIZE - 1);
            dma_wr_en    <= 1;
            dma_data_sent <= (dma_data_sent == HIST_BLOCK_SIZE - 1) ? 16'd0
                                                                     : (dma_data_sent + 16'd1);
        end
    end
end

// Independent-clock FIFO: written on the FFT/ser clock (clk_i, 250 MHz), read on
// the DMA/adc clock (adc_clk_i, 125 MHz). This bridges the FFT output to a
// 125 MHz DMA so the DMA's a_tready handshake is no longer a 250 MHz cross-chip
// path; the FIFO synchronisers handle the clock crossing. Point-cloud data is
// sparse, so 125 MHz read keeps up easily.
xpm_fifo_axis #(
    .TDATA_WIDTH      (64),
    .FIFO_DEPTH       (1 << $clog2(HIST_BLOCK_SIZE * 2 + 4)),
    .CLOCKING_MODE    ("independent_clock"),
    .RELATED_CLOCKS   (0),
    .CDC_SYNC_STAGES  (2),
    .USE_ADV_FEATURES (16'h0000)
) fifo_dma_out (
    .s_aclk          (clk_i),
    .m_aclk          (adc_clk_i),
    .s_aresetn       (rstn_i),
    .s_axis_tdata    (dma_wr_data),
    .s_axis_tvalid   (dma_wr_en),
    .s_axis_tready   (),
    .s_axis_tlast    (dma_wr_tlast),
    .m_axis_tdata    (m_dma_tdata),
    .m_axis_tvalid   (m_dma_tvalid),
    .m_axis_tready   (m_dma_tready),
    .m_axis_tlast    (m_dma_tlast)
);

generate
if (FFT_IMPL == 1) begin : gen_fft_single
    // Plain LogiCORE FFT IP via AXI-Stream config+data ports.

    logic [64-DSZ-1:0] fft_maxi_unused;
    logic [16-ASZ-1:0] fft_data_ext;
    // sign extend the data for padding according to xfft requirement
    assign             fft_data_ext = {16-ASZ{fft_data_i[ASZ-1]}};

    fft_wrapper fft_i (
       .M_AXIS_DOUT_0_tdata         ({fft_maxi_unused, fft_maxi_data}),
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

end else if (FFT_IMPL == 3) begin : gen_fft_ip_ssr
    // DIF SSR FFT: HLS top-level wrapping LogiCORE sub-FFTs.
    // Lane 0 = even bins, lane 1 = odd bins, both in bit-reversed beat order.

    fft_ip_ssr_bd_wrapper fft_i (
        .aclk                   (clk_i),
        .aresetn                (fft_rstn_i),

        .s_axis_tdata           (fft_data_i),
        .s_axis_tvalid          (fft_saxi_valid),
        .s_axis_tready          (fft_saxi_rdy),
        .s_axis_tlast           (fft_saxi_last),

        .m_axis_tdata           (fft_maxi_data),
        .m_axis_tvalid          (fft_maxi_valid),
        .m_axis_tready          (fft_maxi_rdy),
        .m_axis_tlast           (fft_maxi_last),

        .event_frame_started    (fft_frame_start)
    );

end else if (FFT_IMPL == 5) begin : gen_fft_hls_direct
    // Direct hls::fft: one HLS IP instantiates the LogiCORE sub-FFTs internally,
    // fusing pre/FFT/post — no xfft cell or separate post IP. DIF output ordering
    // (handled by the IMPL==3 lane-map branch above).

    fft_hls_direct_bd_wrapper fft_i (
        .aclk                   (clk_i),
        .aresetn                (fft_rstn_i),

        // Runtime sub-FFT length (per-lane log2 size). Stable after the one-time
        // post-reset reconfigure, so the hls::fft core's run-time configurable
        // transform length tracks fft_nfft just like the IMPL==1 xfft config word.
        // The nfft pin only exists when the IP is built with FFT_RUNTIME_NFFT
        // (build knob, off by default); the BD/wrapper and this define are driven
        // by the same env var so the port set stays consistent.
`ifdef FFT_RUNTIME_NFFT
        .nfft                   (fft_nfft),
`endif

        .s_axis_tdata           (fft_data_i),
        .s_axis_tvalid          (fft_saxi_valid),
        .s_axis_tready          (fft_saxi_rdy),
        .s_axis_tlast           (fft_saxi_last),

        .m_axis_tdata           (fft_maxi_data),
        .m_axis_tvalid          (fft_maxi_valid),
        .m_axis_tready          (fft_maxi_rdy),
        .m_axis_tlast           (fft_maxi_last),

        .event_frame_started    (fft_frame_start)
    );

end else if (FFT_IMPL == 4) begin : gen_fft_native
    // Native-SSR xfft (Vivado 2025.2 CONFIG.super_sample_rates): single SSR xfft
    // with thin HLS pre (real->complex+config) and mag (complex->magnitude) wrappers.
    // Output is NATURAL order (PG109: SSR>1 fixed-point is natural-only; handled by
    // the FFT_IMPL==4 lane-map branch above). Input is consecutive samples, sample 0
    // in the LSB lane (fifo_in delivers that) — confirmed correct by RTL sim of the BD.

    fft_ssr_native_bd_wrapper fft_i (
        .aclk                   (clk_i),
        .aresetn                (fft_rstn_i),

        .s_axis_tdata           (fft_data_i),
        .s_axis_tvalid          (fft_saxi_valid),
        .s_axis_tready          (fft_saxi_rdy),
        .s_axis_tlast           (fft_saxi_last),

        .m_axis_tdata           (fft_maxi_data),
        .m_axis_tvalid          (fft_maxi_valid),
        .m_axis_tready          (fft_maxi_rdy),
        .m_axis_tlast           (fft_maxi_last),

        .event_frame_started    (fft_frame_start)
    );

end else begin : gen_fft_ssr
    // DIT SSR FFT (Vitis xf::dsp::fft): lane 0 = lower-half bins, lane 1 = upper-half.

    fft_ssr_bd_wrapper fft_i (
        .aclk                   (clk_i),
        .aresetn                (fft_rstn_i),

        .s_axis_tdata           (fft_data_i),
        .s_axis_tvalid          (fft_saxi_valid),
        .s_axis_tready          (fft_saxi_rdy),
        .s_axis_tlast           (fft_saxi_last),

        .m_axis_tdata           (fft_maxi_data),
        .m_axis_tvalid          (fft_maxi_valid),
        .m_axis_tready          (fft_maxi_rdy),
        .m_axis_tlast           (fft_maxi_last),

        .event_frame_started    (fft_frame_start)
    );

end
endgenerate

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

