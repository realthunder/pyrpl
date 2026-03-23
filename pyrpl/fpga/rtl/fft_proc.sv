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

  input logic  [  16-1:0] fft_conf_data_i,
  output logic [  32-1:0] fft_length,
  output logic [  32-1:0] fft_dbg_cnt
);

localparam SYNC_FF = 4;

// -------------------------------------------------------------------------
// Fast Domain Configurations (Synchronized Inputs)
// -------------------------------------------------------------------------
logic            rstn_slow, rstn_slow_o, rstn_fast, rstn_fast_o;
assign rstn_slow = fft_rstn_i && rstn_slow_o;
assign rstn_fast = fft_rstn_i && rstn_fast_o;

logic            trig_fast, trig_fast_d, trig_pulse;
logic [ 32-1: 0] set_dly_fast;
logic [ 16-1: 0] fft_threshold_k_fast;
logic [ FSZ-1:0] fft_peak_start_fast;
logic [ DSZ-1:0] fft_peak_minimum_fast;
logic [ 16-1: 0] fft_conf_data_fast;

// -------------------------------------------------------------------------
// Internal Signals
// -------------------------------------------------------------------------
logic [ 16-1: 0] fft_q_overflow, fft_q_overflow_fast, index_q_overflow, index_q_overflow_fast;
assign fft_q_overflow_o = {index_q_overflow, fft_q_overflow};

logic [ 32-1: 0] frame_cnt, scan_frame_cnt, clk_cnt;
logic [ HSZ-1:0] fft_index_q[0:(1<<IQSZ)-1];
logic [ HSZ-1:0] fft_hist_index, fast_fft_hist_index, prev_hist_index;
logic [IQSZ-1:0] index_wp, index_rp, fast_index_rp, index_wp_fast_bin;
logic [ QSZ-1:0] wp_fast_bin;

logic [ASZ-1: 0] fft_queue[0:(1<<QSZ)-1];
logic [ASZ-1: 0] fft_last_data, fft_data;
logic            fft_inited;

logic [QSZ-1:0] fft_q_size, fft_q_used;

logic index_flush_toggle, index_flush_toggle_sync, index_flush_d, index_flush_pulse;

logic [ASZ-1: 0]    fft_data_i;
logic [16-ASZ-1:0]  fft_data_ext;
logic               fft_saxi_last, fft_saxi_rdy, fft_saxi_valid;
logic [ FSZ-1: 0]   fft_hist_up[0:(1<<HSZ)-1];
logic [ FSZ-1: 0]   fft_hist_down[0:(1<<HSZ)-1];
logic [ DSZ-1: 0]   fft_buf_up[0:(1<<FSZ)-1];
logic [ DSZ-1: 0]   fft_buf_down[0:(1<<FSZ)-1];

// Internal Fast Clock processing signals
logic [ FSZ-1: 0]   fft_peak_idx, fft_peak2_idx;
logic [ DSZ-1: 0]   fft_peak, fft_peak2;
logic [ FSZ+DSZ-1:0]_fft_sum;
logic [ FSZ: 0]     _fft_count;
logic [ FSZ-1: 0]   fft_peak_rp;
logic               fft_peak_data_valid;
logic [ DSZ-1: 0]   fft_peak_data, fft_peak_data_abs;
logic [ 32-1:  0]   fft_maxi_phase, fft_maxi_data;
logic               fft_maxi_valid, fft_maxi_rdy, fft_maxi_last;

logic [ FSZ-1: 0]   fft_wp, fft_wp_reversed, fft_wp_index;
logic [ FSZ-1: 0]   fft_wp_plus_one = fft_wp+1;
// bit reverse fft_wp, because we are using FFT ip core bit reversed option to
// save memory resource
assign fft_wp_reversed = {<<{fft_wp_plus_one}};

logic [ HSZ-1: 0]   fft_hist_raddr1, fft_hist_raddr2;
logic [ FSZ-1: 0]   fft_raddr1, fft_raddr2;

// Fast Domain Internal Equivalents for outputs
logic [ 32-1: 0]    fast_fft_we_cnt;
logic [ QSZ-1: 0]   fast_fft_q_rp;
logic [ QSZ-1: 0]   fast_fft_q_rp_save;
logic               fast_fft_done;
logic [ 32-1: 0]    fast_fft_length, fast_fft_length2;
logic [ 16-1: 0]    dbg_cnt1, dbg_cnt2;
logic [ 32-1: 0]    dbg_cnt = {dbg_cnt1, dbg_cnt2};
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
// XILINX XPM CDC INSTANTIATIONS
// -------------------------------------------------------------------------

// --- Resets ---
xpm_cdc_single #(.DEST_SYNC_FF(SYNC_FF), .SRC_INPUT_REG(0)) sync_rstn_slow (
    .dest_out(rstn_slow_o), .dest_clk(clk_i), .src_clk(clk_i), .src_in(fft_rstn_i)
);
xpm_cdc_single #(.DEST_SYNC_FF(SYNC_FF), .SRC_INPUT_REG(0)) sync_rstn_fast (
    .dest_out(rstn_fast_o), .dest_clk(fft_clk_i), .src_clk(clk_i), .src_in(fft_rstn_i)
);

// --- Input Configurations (clk_i -> fft_clk_i) ---
xpm_cdc_single #(.DEST_SYNC_FF(SYNC_FF), .SRC_INPUT_REG(0)) sync_trig (
    .dest_out(trig_fast), .dest_clk(fft_clk_i), .src_clk(clk_i), .src_in(trig_i)
);
xpm_cdc_array_single #(.WIDTH(32), .DEST_SYNC_FF(SYNC_FF), .SRC_INPUT_REG(0)) sync_dly (
    .dest_out(set_dly_fast), .dest_clk(fft_clk_i), .src_clk(clk_i), .src_in(set_dly)
);
xpm_cdc_array_single #(.WIDTH(16), .DEST_SYNC_FF(SYNC_FF), .SRC_INPUT_REG(0)) sync_th (
    .dest_out(fft_threshold_k_fast), .dest_clk(fft_clk_i), .src_clk(clk_i), .src_in(fft_threshold_k)
);
xpm_cdc_array_single #(.WIDTH(FSZ), .DEST_SYNC_FF(SYNC_FF), .SRC_INPUT_REG(0)) sync_pstrt (
    .dest_out(fft_peak_start_fast), .dest_clk(fft_clk_i), .src_clk(clk_i), .src_in(fft_peak_start)
);
xpm_cdc_array_single #(.WIDTH(DSZ), .DEST_SYNC_FF(SYNC_FF), .SRC_INPUT_REG(0)) sync_pmin (
    .dest_out(fft_peak_minimum_fast), .dest_clk(fft_clk_i), .src_clk(clk_i), .src_in(fft_peak_minimum)
);
xpm_cdc_array_single #(.WIDTH(16), .DEST_SYNC_FF(SYNC_FF), .SRC_INPUT_REG(0)) sync_conf (
    .dest_out(fft_conf_data_fast), .dest_clk(fft_clk_i), .src_clk(clk_i), .src_in(fft_conf_data_i)
);

// --- Static/Level Outputs (fft_clk_i -> clk_i) ---
xpm_cdc_array_single #(.WIDTH(6), .DEST_SYNC_FF(SYNC_FF), .SRC_INPUT_REG(0)) sync_stat (
    .dest_out(status_o), .dest_clk(clk_i), .src_clk(fft_clk_i), .src_in(fast_status_o)
);
xpm_cdc_single #(.DEST_SYNC_FF(SYNC_FF), .SRC_INPUT_REG(0)) sync_done (
    .dest_out(fft_done), .dest_clk(clk_i), .src_clk(fft_clk_i), .src_in(fast_fft_done)
);
xpm_cdc_array_single #(.WIDTH(8), .DEST_SYNC_FF(SYNC_FF), .SRC_INPUT_REG(0)) sync_pstate (
    .dest_out(fft_peak_state), .dest_clk(clk_i), .src_clk(fft_clk_i), .src_in(fast_fft_peak_state)
);
xpm_cdc_array_single #(.WIDTH(32), .DEST_SYNC_FF(SYNC_FF), .SRC_INPUT_REG(0)) sync_len (
    .dest_out(fft_length), .dest_clk(clk_i), .src_clk(fft_clk_i), .src_in(fast_fft_length)
);
xpm_cdc_array_single #(.WIDTH(32), .DEST_SYNC_FF(SYNC_FF), .SRC_INPUT_REG(0)) sync_len2 (
    .dest_out(fft_dbg_cnt), .dest_clk(clk_i), .src_clk(fft_clk_i), .src_in(dbg_cnt)
);

// --- Output Event Toggles (fft_clk_i -> clk_i) ---

logic fast_peak_toggle, peak_toggle_sync, peak_toggle_d;
xpm_cdc_single #(.DEST_SYNC_FF(SYNC_FF), .SRC_INPUT_REG(0)) sync_pk_tgl (
    .dest_out(peak_toggle_sync), .dest_clk(clk_i), .src_clk(fft_clk_i), .src_in(fast_peak_toggle)
);

logic fft_frame_start_toggle, frame_start_toggle_sync, frame_start_d;
xpm_cdc_single #(.DEST_SYNC_FF(SYNC_FF), .SRC_INPUT_REG(0)) sync_fs_tgl (
    .dest_out(frame_start_toggle_sync), .dest_clk(clk_i), .src_clk(fft_clk_i), .src_in(fft_frame_start_toggle)
);

// --- Index Flush Event Toggle (clk_i -> fft_clk_i) ---
xpm_cdc_single #(.DEST_SYNC_FF(SYNC_FF), .SRC_INPUT_REG(0)) sync_idx_flsh (
    .dest_out(index_flush_toggle_sync), .dest_clk(fft_clk_i), .src_clk(clk_i), .src_in(index_flush_toggle)
);

// --- Pointers (Using XPM Gray Logic) ---
xpm_cdc_gray #(.WIDTH(QSZ), .DEST_SYNC_FF(SYNC_FF), .REG_OUTPUT(0)) sync_q_wp (
    .dest_out_bin(wp_fast_bin), .dest_clk(fft_clk_i), .src_clk(clk_i), .src_in_bin(fft_q_wp)
);

xpm_cdc_gray #(.WIDTH(QSZ), .DEST_SYNC_FF(SYNC_FF), .REG_OUTPUT(0)) sync_q_rp (
    .dest_out_bin(fft_q_rp), .dest_clk(clk_i), .src_clk(fft_clk_i), .src_in_bin(fast_fft_q_rp)
);

xpm_cdc_gray #(.WIDTH(IQSZ), .DEST_SYNC_FF(SYNC_FF), .REG_OUTPUT(0)) sync_index_wp (
    .dest_out_bin(index_wp_fast_bin), .dest_clk(fft_clk_i), .src_clk(clk_i), .src_in_bin(index_wp)
);

xpm_cdc_gray #(.WIDTH(IQSZ), .DEST_SYNC_FF(SYNC_FF), .REG_OUTPUT(0)) sync_index_rp (
    .dest_out_bin(index_rp), .dest_clk(clk_i), .src_clk(fft_clk_i), .src_in_bin(fast_index_rp)
);

// --- Write Counters (Array Single) ---
xpm_cdc_array_single #(.WIDTH(32), .DEST_SYNC_FF(SYNC_FF), .SRC_INPUT_REG(0)) sync_we_cnt (
    .dest_out(fft_we_cnt), .dest_clk(clk_i), .src_clk(fft_clk_i), .src_in(fast_fft_we_cnt)
);

xpm_cdc_array_single #(.WIDTH(16), .DEST_SYNC_FF(SYNC_FF), .SRC_INPUT_REG(0)) sync_q_overflow (
    .dest_out(fft_q_overflow), .dest_clk(clk_i), .src_clk(fft_clk_i), .src_in(fft_q_overflow_fast)
);

xpm_cdc_array_single #(.WIDTH(16), .DEST_SYNC_FF(SYNC_FF), .SRC_INPUT_REG(0)) sync_index_overflow (
    .dest_out(index_q_overflow), .dest_clk(clk_i), .src_clk(fft_clk_i), .src_in(index_q_overflow_fast)
);

xpm_cdc_array_single #(.WIDTH(QSZ), .DEST_SYNC_FF(SYNC_FF), .SRC_INPUT_REG(0)) sync_rp_save (
    .dest_out(fft_q_rp_save), .dest_clk(clk_i), .src_clk(fft_clk_i), .src_in(fast_fft_q_rp_save)
);

xpm_cdc_array_single #(.WIDTH(HSZ), .DEST_SYNC_FF(SYNC_FF), .SRC_INPUT_REG(0)) sync_hist_index (
    .dest_out(fft_hist_index), .dest_clk(clk_i), .src_clk(fft_clk_i), .src_in(fast_fft_hist_index)
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

// --- DATA QUEUE Write Pointer & Overflow Counter ---
always @(posedge clk_i)
if (rstn_slow == 1'b0) begin
    fft_q_wp <= 0;
    fft_inited <= 0;
end else if (dvalid_i) begin
    fft_queue[fft_q_wp] <= fft_data;
    fft_q_wp <= fft_q_wp + 1;
    fft_last_data <= fft_data;
    fft_inited <= 1;
end

// --- INDEX QUEUE Write Pointer & Flush Generator ---
always @(posedge clk_i)
if (!rstn_slow) begin
    index_flush_toggle <= 0;
    index_wp <= 0;
end else if (fft_index_flush_i) begin
    index_flush_toggle <= ~index_flush_toggle;
    index_wp <= 0;
end else if (fft_index_valid_i) begin
    fft_index_q[index_wp] <= fft_hist_index_i;
    index_wp <= index_wp + 1;
end

// --- OUTPUT REGISTRATION BLOCK ---
always @(posedge clk_i)
if (!rstn_slow) begin
    fft_peak_ready <= 0;
    fft_count <= 0;
    fft_sum <= 0;
    fft_peak_index_up <= 0;
    fft_peak_index_down <= 0;
    fft_peak_value_up <= 0;
    fft_peak_value_down <= 0;
    
    peak_toggle_d <= 0;
    frame_start_d <= 0;
end else begin

    peak_toggle_d <= peak_toggle_sync;
    if (peak_toggle_sync ^ peak_toggle_d) begin
        fft_peak_index_up   <= fast_fft_peak_index_up;
        fft_peak_index_down <= fast_fft_peak_index_down;
        fft_peak_value_up   <= fast_fft_peak_value_up;
        fft_peak_value_down <= fast_fft_peak_value_down;
        fft_sum             <= fast_fft_sum;
        fft_count           <= fast_fft_count;
    end
    fft_peak_ready <= {fft_peak_ready[0], (peak_toggle_sync ^ peak_toggle_d)};
    
    frame_start_d <= frame_start_toggle_sync;
end
logic fft_frame_start_pulse;
assign fft_frame_start_pulse = frame_start_toggle_sync ^ frame_start_d;

always @(posedge clk_i)
if (clk_cnt >= 125000000) begin
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

logic rstn_delay;
logic fft_rstn = rstn_delay && rstn_fast;
always @(posedge fft_clk_i) begin
    rstn_delay <= rstn_fast;
end

logic fft_frame_start;
always @(posedge fft_clk_i) begin
    if (!rstn_fast) fft_frame_start_toggle <= 0;
    else if (fft_frame_start) fft_frame_start_toggle <= ~fft_frame_start_toggle;
end

logic          fft_conf_dvalid;
logic          fft_conf_rdy;
logic          rstn_core;
assign rstn_core = rstn_fast && !fft_conf_dvalid;
logic [ 5-1:0] fft_nfft;
assign fft_nfft = fft_conf_data_fast[5-1:0]; // Uses synchronized config!
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

logic [32-1:0] pre_size;
assign pre_size = 2**(RSZ-1) - set_dly_fast; // Uses synchronized delay
logic up_in;

assign fft_q_size = wp_fast_bin - fast_fft_q_rp;
assign fft_q_full = (fft_q_size >= ((1<<QSZ) - 2));

// Trigger sync edge detection
always @(posedge fft_clk_i) begin
    if (!rstn_fast) trig_fast_d <= 0;
    else trig_fast_d <= trig_fast;
end
assign trig_pulse = trig_fast & ~trig_fast_d;

always @(posedge fft_clk_i)
if (rstn_core == 1'b0) begin
    fast_fft_we_cnt <= 0;
    fast_fft_q_rp <= 0;
    fast_fft_done <= 1;
    up_in <= 1;
end else begin
    if (trig_pulse && fast_fft_done && up_in == 1) begin
        dbg_cnt1 <= 0;
        dbg_cnt2 <= 0;
        if (set_dly_fast < 2**(RSZ-1)) begin
            fast_fft_we_cnt <= fast_fft_length2;
            fast_fft_q_rp_save <= fast_fft_length2;
            if (pre_size < fft_q_size) begin
                fast_fft_q_rp <= wp_fast_bin - pre_size[QSZ-1:0];
                // fast_fft_q_rp_save <= wp_fast_bin - pre_size[QSZ-1:0];
            end else begin
                // fast_fft_q_rp_save <= fast_fft_q_rp;
            end
        end else begin
            fast_fft_q_rp <= wp_fast_bin;
            // fast_fft_q_rp_save <= wp_fast_bin;
            fast_fft_we_cnt <= set_dly_fast - 2**(RSZ-1) + fast_fft_length2;
            fast_fft_q_rp_save <= set_dly_fast - 2**(RSZ-1) + fast_fft_length2;
        end
    end else if (fft_q_size > 0 && fast_fft_we_cnt > 0 && (fast_fft_we_cnt > fast_fft_length2 || fft_saxi_rdy)) begin
        fft_data_i <= fft_queue[fast_fft_q_rp];
        fast_fft_q_rp <= fast_fft_q_rp + 1;
        if (fast_fft_we_cnt == 1 || fast_fft_we_cnt == fast_fft_length+1)
            up_in <= up_in + up_toggle;
        fast_fft_we_cnt <= fast_fft_we_cnt - 1;
        dbg_cnt1 <= dbg_cnt1 + 1;
    end else if (fft_q_full) begin
        dbg_cnt2 <= dbg_cnt2 + 1;
        fast_fft_q_rp <= fast_fft_q_rp + 1;
        fft_q_overflow_fast <= fft_q_overflow_fast + 1;
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
    up_out <= 1;
end else if (fft_maxi_valid && fft_maxi_rdy) begin
    if (up_out)
        fft_buf_up[fft_wp_index] <= fft_maxi_data[DSZ-1:0];
    else
        fft_buf_down[fft_wp_index] <= fft_maxi_data[DSZ-1:0];
        
    if (fft_maxi_last) begin
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

// Use synchronized peak configuration 
assign fft_peak_data_valid = fft_maxi_valid && 
                             fft_peak_rp >= fft_peak_start_fast && 
                             fft_peak_rp < fast_fft_length[FSZ:1] && 
                             fft_peak_data_abs > fft_peak_minimum_fast;

logic peak_up;
logic peak_ready;

// Index Queue Flush pulse logic
always @(posedge fft_clk_i) begin
    if (!rstn_fast) index_flush_d <= 0;
    else index_flush_d <= index_flush_toggle_sync;
end
assign index_flush_pulse = index_flush_toggle_sync ^ index_flush_d;

assign index_q_used = index_wp_fast_bin - fast_index_rp;
assign index_q_full = (index_q_used >= ((1<<IQSZ) - 2));

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
        else if (index_q_full) begin
            fast_index_rp <= fast_index_rp + 1;
            index_q_overflow_fast <= index_q_overflow_fast + 1;
        end
            
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
    end else if (index_q_full) begin
        fast_index_rp <= fast_index_rp + 1;
        index_q_overflow_fast <= index_q_overflow_fast + 1;
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
    .threshold_k_sq (fft_threshold_k_fast),
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
   .S_AXIS_CONFIG_0_tdata       (fft_conf_data_fast),
   .S_AXIS_CONFIG_0_tready      (fft_conf_rdy     ),
   .S_AXIS_CONFIG_0_tvalid      (fft_conf_dvalid  ),
   .S_AXIS_DATA_0_tdata         ({16'b0, fft_data_ext, fft_data_i}),
   .S_AXIS_DATA_0_tlast         (fft_saxi_last    ),
   .S_AXIS_DATA_0_tready        (fft_saxi_rdy     ),
   .S_AXIS_DATA_0_tvalid        (fft_saxi_valid   ),
   .aclk_0                      (fft_clk_i        ),
   .aresetn_0                   (fft_rstn         ),
   .event_data_in_channel_halt_0(fft_in_halt      ),
   .event_data_out_channel_halt_0(fft_out_halt    ),
   .event_status_channel_halt_0 (fft_status_halt  ),
   .event_frame_started_0       (fft_frame_start  ),
   .event_tlast_missing_0       (fft_tlast_missing),
   .event_tlast_unexpected_0    (fft_tlast_unexp  )
);

assign fast_status_o = {fft_in_halt, fft_out_halt, fft_status_halt, fft_frame_start, fft_tlast_missing, fft_tlast_unexp};

endmodule
