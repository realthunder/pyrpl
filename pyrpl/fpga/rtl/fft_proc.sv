module fft_proc #(
  parameter ASZ,        // ADC input sample width
  parameter DSZ,        // FFT_output width
  parameter FSZ,        // FFT transform length 2^FSZ
  parameter RSZ,        // RAM size 2^RSZ
  parameter HSZ,        // fft history buffer size 2^HSZ (Note: consider word size of 32bit, better not exceed 64KBytes in total)
  parameter QSZ,        // FFT queue size 2^QSZ
  parameter IQSZ = 6    // index queue size 2^IQSZ
)(
  input logic             clk_i,
  input logic             fft_clk_i,   // FAST CLOCK
  input logic             fft_rstn_i,
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
  
  // QUEUE MONITORING
  output logic [ 32-1: 0] fft_q_overflow_o, 

  output logic [  8-1: 0] fft_peak_state,
  output logic [ FSZ-1:0] fft_peak_index_up,
  output logic [ FSZ-1:0] fft_peak_index_down,
  output logic [ DSZ-1:0] fft_peak_value_up,
  output logic [ DSZ-1:0] fft_peak_value_down,
  output logic [ 32-1: 0] fft_frame_cnt,
  output logic [ 32-1: 0] fft_scan_frame_cnt,
  output logic [ 32-1: 0] fft_we_cnt,

  output logic [ QSZ-1:0] fft_q_wp,
  output logic [ QSZ-1:0] fft_q_rp,
  output logic [ QSZ-1:0] fft_q_rp_save,
  output logic [ ASZ-1:0] fft_q_rdata_o,

  input logic  [  16-1:0] fft_conf_data_i,
  output logic [  32-1:0] fft_length,
  output logic [  32-1:0] fft_length2
);

// -------------------------------------------------------------------------
// Reset Synchronization
// -------------------------------------------------------------------------
logic rstn_slow_sync1, rstn_slow;
logic rstn_fast_sync1, rstn_fast;

always @(posedge clk_i) {rstn_slow, rstn_slow_sync1} <= {rstn_slow_sync1, fft_rstn_i};
always @(posedge fft_clk_i) {rstn_fast, rstn_fast_sync1} <= {rstn_fast_sync1, fft_rstn_i};

// -------------------------------------------------------------------------
// Internal Signals
// -------------------------------------------------------------------------
logic [ 16-1: 0] fft_q_overflow;
logic [ 16-1: 0] index_q_overflow; 
assign fft_q_overflow_o = {index_q_overflow, fft_q_overflow};

logic [ 32-1: 0] frame_cnt;
logic [ 32-1: 0] scan_frame_cnt;
logic [ 32-1: 0] clk_cnt;

logic [ HSZ-1:0] fft_index_q[0:(1<<IQSZ)-1];
logic [ HSZ-1:0] fft_hist_index;
logic [ HSZ-1:0] fast_fft_hist_index;
logic [ HSZ-1:0] prev_hist_index;
logic [IQSZ-1:0] index_wp;
logic [IQSZ-1:0] index_rp;
logic [IQSZ-1:0] index_wp_fast_bin;
logic [IQSZ-1:0] fast_index_rp;

logic [ASZ-1: 0] fft_queue[0:(1<<QSZ)-1];
logic [ASZ-1: 0] fft_last_data;
logic            fft_inited;
logic [ASZ-1: 0] fft_data;
logic [QSZ-1: 0] fft_q_size;

logic [QSZ-1:0] fft_q_used;
logic [QSZ-1:0] wp_fast_bin;

logic index_flush_toggle;
logic index_flush_sync1, index_flush_sync2, index_flush_d, index_flush_pulse;

logic [ASZ-1: 0]    fft_data_i;
logic [16-ASZ-1:0]  fft_data_ext;
(* mark_debug = "true" *)
logic               fft_saxi_last;
(* mark_debug = "true" *)
logic               fft_saxi_rdy;
(* mark_debug = "true" *)
logic               fft_saxi_valid;
logic [ FSZ-1: 0]   fft_hist_up[0:(1<<HSZ)-1];
logic [ FSZ-1: 0]   fft_hist_down[0:(1<<HSZ)-1];
logic [ DSZ-1: 0]   fft_buf_up[0:(1<<FSZ)-1];
logic [ DSZ-1: 0]   fft_buf_down[0:(1<<FSZ)-1];

// Internal Fast Clock processing signals
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
// bit reverse fft_wp, because we are using FFT ip core bit reversed option to
// save memory resource
logic [ FSZ-1: 0]   fft_wp_reversed;
assign fft_wp_reversed = {<<{fft_wp+1}};
logic [ FSZ-1: 0]   fft_wp_index;
logic [ HSZ-1: 0]   fft_hist_raddr1, fft_hist_raddr2;
logic [ FSZ-1: 0]   fft_raddr1, fft_raddr2;

// Fast Domain Internal Equivalents for outputs
logic [ FSZ-1: 0]   fast_fft_wp_last;
logic [ 32-1: 0]    fast_fft_we_cnt;
logic [ QSZ-1: 0]   fast_fft_q_rp;
logic [ QSZ-1: 0]   fast_fft_q_rp_save;
(* mark_debug = "true" *)
logic               fast_fft_done;
logic [ 32-1: 0]    fast_fft_length;
logic [ 32-1: 0]    fast_fft_length2;
logic [  6-1: 0]    fast_status_o;
logic [  8-1: 0]    fast_fft_peak_state;
logic [ 2-1 : 0]    fast_fft_peak_ready;
logic [ FSZ : 0]    fast_fft_count;
logic [ FSZ+DSZ-1:0]fast_fft_sum;
logic [ FSZ-1:0]    fast_fft_peak_index_up, fast_fft_peak_index_down;
logic [ DSZ-1:0]    fast_fft_peak_value_up, fast_fft_peak_value_down;

// sign extend the data for padding according to xfft requirement
assign fft_data_ext = {16-ASZ{fft_data_i[ASZ-1]}};
assign fft_data = (enable_i || !fft_inited) ? data_i : fft_last_data;


// -------------------------------------------------------------------------
// POINTER CDC INSTANTIATIONS (Using our parameterized module)
// -------------------------------------------------------------------------

// Data Queue Write Pointer: Slow domain -> Fast domain
cdc_sync #(.WIDTH(QSZ)) sync_q_wp (
    .clk_in     (clk_i),
    .rstn_in    (rstn_slow),
    .bin_in     (fft_q_wp),
    .clk_out    (fft_clk_i),
    .rstn_out   (rstn_fast),
    .bin_out    (wp_fast_bin)
);

// Data Queue Read Pointer: Fast domain -> Slow domain (Drives fft_q_rp directly)
cdc_sync #(.WIDTH(QSZ)) sync_q_rp (
    .clk_in     (fft_clk_i),
    .rstn_in    (rstn_fast),
    .bin_in     (fast_fft_q_rp),
    .clk_out    (clk_i),
    .rstn_out   (rstn_slow),
    .bin_out    (fft_q_rp)
);

// Index Queue Write Pointer: Slow domain -> Fast domain
cdc_sync #(.WIDTH(IQSZ)) sync_index_wp (
    .clk_in     (clk_i),         
    .rstn_in    (rstn_slow), 
    .clear_in   (fft_index_flush_i), 
    .bin_in     (index_wp),
    .clk_out    (fft_clk_i),    
    .rstn_out   (rstn_fast),
    .clear_out  (index_flush_pulse),
    .bin_out    (index_wp_fast_bin)
);

// Index Queue Read Pointer: Fast domain -> Slow domain
cdc_sync #(.WIDTH(IQSZ)) sync_index_rp (
    .clk_in     (fft_clk_i),
    .rstn_in    (rstn_fast),
    .clear_in   (fft_index_flush_i), 
    .bin_in     (fast_index_rp),
    .clk_out    (clk_i),
    .rstn_out   (rstn_slow),
    .clear_out  (index_flush_pulse),
    .bin_out    (index_rp)
);

// Debug Counters: Fast domain -> Slow domain (Drives output ports directly)
cdc_sync #(.WIDTH(32)) sync_we_cnt (
    .clk_in     (fft_clk_i),
    .rstn_in    (rstn_fast),
    .bin_in     (fast_fft_we_cnt),
    .clk_out    (clk_i),
    .rstn_out   (rstn_slow),
    .bin_out    (fft_we_cnt)
);

cdc_sync #(.WIDTH(QSZ)) sync_rp_save (
    .clk_in     (fft_clk_i),
    .rstn_in    (rstn_fast),
    .bin_in     (fast_fft_q_rp_save),
    .clk_out    (clk_i),
    .rstn_out   (rstn_slow),
    .bin_out    (fft_q_rp_save)
);

cdc_sync #(.WIDTH(HSZ)) sync_hist_index (
    .clk_in     (fft_clk_i),
    .rstn_in    (rstn_fast),
    .bin_in     (fast_fft_hist_index),
    .clk_out    (clk_i),
    .rstn_out   (rstn_slow),
    .bin_out    (fft_hist_index)
);

// -------------------------------------------------------------------------
// SLOW CLOCK DOMAIN (clk_i) - Input Logic & OUTPUT REGISTRATIONS
// -------------------------------------------------------------------------

always @(posedge clk_i) begin
   fft_raddr1 <= sys_addr[FSZ-1+3:3] ;
   fft_raddr2 <= fft_raddr1;
   fft_rdata_up_o <= fft_buf_up[fft_raddr2];
   fft_rdata_down_o <= fft_buf_down[fft_raddr2];

   fft_hist_raddr1 <= sys_addr[HSZ-1+2:2]  ;
   fft_hist_raddr2 <= fft_hist_raddr1;
   fft_hist_rdata_down_o <= fft_hist_down[fft_hist_raddr2];
   fft_hist_rdata_up_o <= fft_hist_up[fft_hist_raddr2];
end

assign fft_q_used = fft_q_wp - fft_q_rp;
assign fft_q_full = (fft_q_used >= ((1<<QSZ) - 2));

// --- DATA QUEUE Write Pointer & Overflow Counter ---
always @(posedge clk_i) 
if (rstn_slow == 1'b0) begin
    fft_q_wp <= 0;
    fft_inited <= 0;
    fft_q_overflow <= 0;
end else if (dvalid_i) begin
    if (!fft_q_full) begin
        fft_queue[fft_q_wp] <= fft_data;
        fft_q_wp <= fft_q_wp + 1;
        fft_last_data <= fft_data;
        fft_inited <= 1;
    end else begin
        fft_q_overflow <= fft_q_overflow + 1; 
    end
end

assign index_q_used = index_wp - index_rp;
assign index_q_full = (index_q_used >= ((1<<IQSZ) - 2));

// --- INDEX QUEUE Write Pointer & Flush Generator ---
always @(posedge clk_i)
if (!rstn_slow) begin
    index_flush_toggle <= 0;
    index_wp <= 0;
    index_q_overflow <= 0;
end else if (fft_index_flush_i) begin
    index_flush_toggle <= ~index_flush_toggle;
    index_wp <= 0;
end else if (fft_index_valid_i) begin
    if (!index_q_full) begin
        fft_index_q[index_wp] <= fft_hist_index_i;
        index_wp <= index_wp + 1;
    end else begin
        index_q_overflow <= index_q_overflow + 1;
    end
end

// --- OUTPUT REGISTRATION BLOCK (ALL outputs forced to clk_i) ---
logic fast_wp_last_toggle, wp_last_sync1, wp_last_sync2, wp_last_d;
logic fast_peak_toggle, peak_toggle_sync1, peak_toggle_sync2, peak_toggle_d;
logic [6-1:0] status_sync1;
logic fft_done_sync1;
logic [8-1:0] peak_state_sync1;
logic [32-1:0] length_sync1;
logic [32-1:0] length2_sync1;
logic [ASZ-1:0] q_rdata_sync1;

always @(posedge clk_i)
if (!rstn_slow) begin
    fft_wp_last <= 0;
    status_o <= 0;
    fft_done <= 1;
    fft_peak_ready <= 0;
    fft_count <= 0;
    fft_sum <= 0;
    fft_peak_state <= 0;
    fft_peak_index_up <= 0;
    fft_peak_index_down <= 0;
    fft_peak_value_up <= 0;
    fft_peak_value_down <= 0;
    fft_q_rdata_o <= 0;
    fft_length <= 0;
    fft_length2 <= 0;
    {wp_last_sync2, wp_last_sync1, wp_last_d} <= 0;
    {peak_toggle_sync2, peak_toggle_sync1, peak_toggle_d} <= 0;
end else begin
    {status_o, status_sync1} <= {status_sync1, fast_status_o};
    {fft_done, fft_done_sync1} <= {fft_done_sync1, fast_fft_done};
    {fft_peak_state, peak_state_sync1} <= {peak_state_sync1, fast_fft_peak_state};
    {fft_length, length_sync1} <= {length_sync1, fast_fft_length};
    {fft_length2, length2_sync1} <= {length2_sync1, fast_fft_length2};
    {fft_q_rdata_o, q_rdata_sync1} <= {q_rdata_sync1, fft_data_i}; 

    {wp_last_sync2, wp_last_sync1} <= {wp_last_sync1, fast_wp_last_toggle};
    wp_last_d <= wp_last_sync2;
    if (wp_last_sync2 ^ wp_last_d) begin
        fft_wp_last <= fast_fft_wp_last;
    end

    {peak_toggle_sync2, peak_toggle_sync1} <= {peak_toggle_sync1, fast_peak_toggle};
    peak_toggle_d <= peak_toggle_sync2;
    if (peak_toggle_sync2 ^ peak_toggle_d) begin
        fft_peak_index_up   <= fast_fft_peak_index_up;
        fft_peak_index_down <= fast_fft_peak_index_down;
        fft_peak_value_up   <= fast_fft_peak_value_up;
        fft_peak_value_down <= fast_fft_peak_value_down;
        fft_sum             <= fast_fft_sum;
        fft_count           <= fast_fft_count;
    end
    fft_peak_ready <= {fft_peak_ready[0], (peak_toggle_sync2 ^ peak_toggle_d)};
end

logic fft_frame_start, fft_frame_start_toggle, toggle_sync1, toggle_sync2, toggle_d, fft_frame_start_pulse;
always @(posedge clk_i) begin
    {toggle_sync2, toggle_sync1} <= {toggle_sync1, fft_frame_start_toggle};
    toggle_d <= toggle_sync2;
end
assign fft_frame_start_pulse = toggle_sync2 ^ toggle_d;

always @(posedge clk_i)
if (clk_cnt >= 125000000) begin
    clk_cnt <= 0;
    fft_frame_cnt <= {1'b0, frame_cnt[32-1:1]};
    fft_scan_frame_cnt <= scan_frame_cnt;
    if (fft_frame_start_pulse) begin
        frame_cnt <= 1;
        scan_frame_cnt <= 1;
        prev_hist_index <= fft_hist_index;
    end else begin
        frame_cnt <= 0;
        scan_frame_cnt <= 0;
    end
end else begin
    clk_cnt <= clk_cnt + 1;
    if (fft_frame_start_pulse) begin
        if (~&frame_cnt)
            frame_cnt <= frame_cnt + 1;
        if (~&scan_frame_cnt && prev_hist_index != fft_hist_index)
            scan_frame_cnt <= scan_frame_cnt + 1;
        prev_hist_index <= fft_hist_index;
    end
end

// -------------------------------------------------------------------------
// FAST CLOCK DOMAIN (fft_clk_i) - INTERNAL PROCESSING ONLY
// -------------------------------------------------------------------------

always @(posedge fft_clk_i) begin
    if (!rstn_fast) fft_frame_start_toggle <= 0;
    else if (fft_frame_start) fft_frame_start_toggle <= ~fft_frame_start_toggle;
end

logic          fft_conf_dvalid;
logic          fft_conf_rdy;
logic          rstn_core;
assign rstn_core = rstn_fast && !fft_conf_dvalid;
logic [ 5-1:0] fft_nfft;
assign fft_nfft = fft_conf_data_i[5-1:0];
logic [32-1:0] fast_fft_length2;
logic          up_toggle;
assign up_toggle = fast_fft_length2 > fast_fft_length;

// Only allow one-time re-configuration after reset to avoid synchronization issue
always @(posedge fft_clk_i)
if (rstn_fast == 1'b0) begin
    fft_conf_dvalid <= 1;
    fast_fft_length <= 2**fft_nfft;
    // We need 2x amount of samples, one for Fup and one for Fdown
    if (fft_nfft < RSZ-1)
        fast_fft_length2 <= 2**(fft_nfft+1);
    else
        fast_fft_length2 <= 2**fft_nfft;
end else if (fft_conf_dvalid && fft_conf_rdy) begin
    fft_conf_dvalid <= 0;
end

logic [32-1:0] pre_size = 2**(RSZ-1) - set_dly;
logic up_in;

assign fft_q_size = wp_fast_bin - fast_fft_q_rp;

// Trigger sync and edge detection
logic trig_fast_sync1, trig_fast;

(* mark_debug = "true" *)
logic trig_fast_d;

always @(posedge fft_clk_i) begin
    {trig_fast, trig_fast_sync1} <= {trig_fast_sync1, trig_i};
    trig_fast_d <= trig_fast;
end
logic trig_pulse = trig_fast & ~trig_fast_d;

always @(posedge fft_clk_i)
if (rstn_core == 1'b0) begin
    fast_fft_we_cnt <= 0;
    fast_fft_q_rp <= 0;
    fast_fft_done <= 1;
    up_in <= 1;
end else begin
    if (trig_pulse && fast_fft_done && up_in == 1) begin
        fast_fft_q_rp <= wp_fast_bin;
        fast_fft_q_rp_save <= wp_fast_bin;
        fast_fft_we_cnt <= fast_fft_length2;

        // if (set_dly < 2**(RSZ-1)) begin
        //     fast_fft_we_cnt <= fast_fft_length2;
        //     if (pre_size < fft_q_size) begin
        //         fast_fft_q_rp <= wp_fast_bin - pre_size;
        //         fast_fft_q_rp_save <= wp_fast_bin - pre_size;
        //     end else begin
        //         fast_fft_q_rp_save <= fast_fft_q_rp;
        //     end
        // end else begin
        //     fast_fft_q_rp <= wp_fast_bin;
        //     fast_fft_q_rp_save <= wp_fast_bin;
        //     fast_fft_we_cnt <= set_dly - 2**(RSZ-1) + fast_fft_length2;
        // end
    end else if (fft_q_size > 0 && fast_fft_we_cnt > 0 && (fast_fft_we_cnt > fast_fft_length2 || fft_saxi_rdy)) begin
        fft_data_i <= fft_queue[fast_fft_q_rp];
        fast_fft_q_rp <= fast_fft_q_rp + 1;
        if (fast_fft_we_cnt == 1 || fast_fft_we_cnt == fast_fft_length+1)
            up_in <= up_in + up_toggle;
        fast_fft_we_cnt <= fast_fft_we_cnt - 1;
    end
        
    fast_fft_done <= (fast_fft_we_cnt == 0);
    fft_saxi_last <= (fast_fft_we_cnt == 1 || fast_fft_we_cnt == fast_fft_length+1);
    fft_saxi_valid <= (fft_q_size > 0 && fast_fft_we_cnt > 0 && fast_fft_we_cnt <= fast_fft_length2);
end

logic up_out;
always @(posedge fft_clk_i)
if (rstn_core == 1'b0) begin
    fft_wp <= 0;
    fft_wp_index <= 0;
    fast_fft_wp_last <= 0;
    fast_wp_last_toggle <= 0;
    up_out <= 1;
end else if (fft_maxi_valid && fft_maxi_rdy) begin
    if (up_out)
        fft_buf_up[fft_wp_index] <= fft_maxi_data[DSZ-1:0];
    else
        fft_buf_down[fft_wp_index] <= fft_maxi_data[DSZ-1:0];
        
    if (fft_maxi_last) begin
        fast_fft_wp_last <= fft_wp;
        fast_wp_last_toggle <= ~fast_wp_last_toggle;
        fft_wp <= 0;
        fft_wp_index <= 0;
        up_out <= up_out + up_toggle;
    end else begin
        fft_wp <= fft_wp + 1;
        fft_wp_index <= fft_wp_reversed >> (FSZ-fft_nfft);
    end
end

assign fft_peak_rp = fft_wp_index;
assign fft_peak_data = fft_maxi_data[DSZ-1:0];
assign fft_peak_data_abs = fft_peak_data[DSZ-1] ? -fft_peak_data : fft_peak_data;
assign fft_peak_data_valid = fft_maxi_valid && fft_peak_rp>=fft_peak_start && fft_peak_rp<fast_fft_length[FSZ:1] && fft_peak_data_abs>fft_peak_minimum;

logic peak_up;
logic peak_ready;

// Index Queue Flush logic in fast domain
always @(posedge fft_clk_i)
if (!rstn_fast) begin
    {index_flush_sync2, index_flush_sync1} <= 0;
    index_flush_d <= 0;
end else begin
    {index_flush_sync2, index_flush_sync1} <= {index_flush_sync1, index_flush_toggle};
    index_flush_d <= index_flush_sync2;
end
assign index_flush_pulse = index_flush_sync2 ^ index_flush_d;

always @(posedge fft_clk_i)
if (rstn_core == 1'b0 || index_flush_pulse) begin
    fast_fft_peak_ready <= 2'b11;
    fast_peak_toggle <= 0;
    fast_index_rp <= 0;
    peak_up <= 1;
    fast_fft_hist_index <= 0; 
end else begin
    if ({fast_fft_peak_ready[0], peak_ready} == 2'b01) begin
        if (fast_index_rp != index_wp_fast_bin) begin
            fast_fft_hist_index <= fft_index_q[fast_index_rp];
        end
            
        if (peak_up != up_toggle)
            fast_index_rp <= fast_index_rp + 1;
            
        peak_up <= peak_up + up_toggle;

        if (peak_up) begin
            fast_fft_peak_index_up <= fft_peak_idx;
            fast_fft_peak_value_up <= fft_peak;
        end else begin
            fast_fft_peak_index_down <= fft_peak_idx;
            fast_fft_peak_value_down <= fft_peak;
        end
        fast_fft_sum <= _fft_sum;
        fast_fft_count <= _fft_count;
        
        fast_peak_toggle <= ~fast_peak_toggle;
    end

    if (fast_fft_peak_ready == 2'b01) begin
        if (peak_up)
            fft_hist_up[fast_fft_hist_index] <= fast_fft_peak_index_up;
        else
            fft_hist_down[fast_fft_hist_index] <= fast_fft_peak_index_down;
    end

    fast_fft_peak_ready <= {fast_fft_peak_ready[0], peak_ready};
end

peak_detector #(.SSZ(FSZ), .DSZ(DSZ)) peak_detector_i (
    .clk            (fft_clk_i),
    .resetn         (rstn_core),
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
    .state          (fast_fft_peak_state)
);

logic fft_in_halt, fft_out_halt, fft_status_halt, fft_tlast_missing, fft_tlast_unexp;

fft_wrapper fft_i (
   .M_AXIS_DOUT_0_tdata         ({fft_maxi_phase, fft_maxi_data}),
   .M_AXIS_DOUT_0_tlast         (fft_maxi_last    ),
   .M_AXIS_DOUT_0_tvalid        (fft_maxi_valid   ),
   .M_AXIS_DOUT_0_tready        (fft_maxi_rdy     ),
   .S_AXIS_CONFIG_0_tdata       (fft_conf_data_i  ),
   .S_AXIS_CONFIG_0_tready      (fft_conf_rdy     ),
   .S_AXIS_CONFIG_0_tvalid      (fft_conf_dvalid  ),
   .S_AXIS_DATA_0_tdata         ({16'b0, fft_data_ext, fft_data_i}),
   .S_AXIS_DATA_0_tlast         (fft_saxi_last    ),
   .S_AXIS_DATA_0_tready        (fft_saxi_rdy     ),
   .S_AXIS_DATA_0_tvalid        (fft_saxi_valid   ),
   .aclk_0                      (fft_clk_i        ),
   .aresetn_0                   (rstn_core        ),
   .event_data_in_channel_halt_0(fft_in_halt      ),
   .event_data_out_channel_halt_0(fft_out_halt    ),
   .event_status_channel_halt_0 (fft_status_halt  ),
   .event_frame_started_0       (fft_frame_start  ),
   .event_tlast_missing_0       (fft_tlast_missing),
   .event_tlast_unexpected_0    (fft_tlast_unexp  )
);

//---------------------------------------------------------------------------------
//  System bus connection

assign fast_status_o = {fft_in_halt
                   , fft_out_halt
                   , fft_status_halt
                   , fft_frame_start
                   , fft_tlast_missing
                   , fft_tlast_unexp};

endmodule
