// tb_fft_chain — full FFT_IMPL=4 engine chain in simulation: the REAL fft_proc.sv
// (feed engine + input FIFO + fft_ssr_native_bd (pre -> xfft SSR -> mag) +
// fifo_peak_in + peak_detector_bd (CFAR)), driven by a model of the scope's
// acquisition FSM (trig -> WAIT1 -> FFT_UP(acq) -> WAIT2 -> FFT_DOWN(acq)) at
// FFT_CLK_SEL=0 (adc_clk == clk_i, the product configuration).
//
// PROVES, for the xfft REALTIME throttle scheme (ip/fft_ssr_native_bd.tcl):
//   1. s_axis_data_tvalid at the xfft core never drops between a frame's first
//      beat and its tlast (the feed's full-half buffering works);
//   2. the core never sees m_axis_data_tready low while it has data (mag/peak
//      sink never stalls);
//   3. no event_data_in/out_channel_halt / tlast_missing / tlast_unexpected
//      pulse, and the sticky status_o bits stay 0;
//   4. every frame's up AND down peak lands on the injected tone bin, for
//      NFRAMES back-to-back frames, at the selected acq/wait geometry.
// Knobs (-d): ACQ_UP ACQ_DOWN (samples, <= N), WAIT1 WAIT2 (cycles), NFRAMES,
//             TONE_K (up tone bin), TONE_K2 (down tone bin), GAP (idle cycles
//             between fft_done and the next trigger).
// Prints "RESULT ... : PASS" or "RESULT ... : FAIL <reasons>".
`timescale 1ns/1ps
`ifndef ACQ_UP
  `define ACQ_UP 1848
`endif
`ifndef ACQ_DOWN
  `define ACQ_DOWN 1848
`endif
`ifndef WAIT1
  `define WAIT1 100
`endif
`ifndef WAIT2
  `define WAIT2 200
`endif
`ifndef NFRAMES
  `define NFRAMES 6
`endif
`ifndef TONE_K
  `define TONE_K 100
`endif
`ifndef TONE_K2
  `define TONE_K2 300
`endif
`ifndef GAP
  `define GAP 0
`endif

module tb_fft_chain;
    localparam int ASZ = 14, DSZ = 24, FSZ = 11, FRAC = 8, FSSR = 4;
    localparam int RSZ = 14, HSZ = 12, IQSZ = 5, READ_DELAY = 4;
    localparam int QSZ = FSZ + 1;            // as red_pitaya_scope.sv
    localparam int N = 1 << FSZ, BEATS = N / FSSR, IDX = FSZ + FRAC;
    localparam int ACQ_UP = `ACQ_UP, ACQ_DOWN = `ACQ_DOWN;
    localparam int WAIT1 = `WAIT1, WAIT2 = `WAIT2, NFRAMES = `NFRAMES, GAP = `GAP;
    localparam int K1 = `TONE_K, K2 = `TONE_K2;
    localparam real AMP = 4000.0, PI = 3.14159265358979;

    logic clk = 0; always #4 clk = ~clk;      // 125 MHz, adc_clk == clk_i
    logic rstn = 0;

    // ---- DUT ports ----
    logic [ASZ-1:0] data_in = 0;
    logic           enable_in = 0, trig_in = 0;
    logic [FSZ-1:0] acq_up_i = ACQ_UP, acq_down_i = ACQ_DOWN;
    logic           acq_up_done = 0, acq_down_done = 0;
    logic           index_valid = 0;
    logic [HSZ-1:0] hist_index = 0;
    logic [DSZ-1:0] rdata_up, rdata_down;
    logic [IDX-1:0] hist_rdata_up, hist_rdata_down;
    logic           dma_point_valid;
    logic [IDX-1:0] dma_point_up, dma_point_down;
    logic [DSZ-1:0] dma_point_val_up, dma_point_val_down;
    logic [HSZ-1:0] dma_point_idx;
    logic [5:0]     status;
    logic           fft_done_o, peak_ready_o;
    logic [IDX-1:0] peak_index_up, peak_index_down;
    logic [DSZ-1:0] peak_value_up, peak_value_down;
    logic [31:0]    we_cnt, point_cnt, scan_point_cnt, overflow_cnt, diag;

    fft_proc #(
        .ASZ(ASZ), .DSZ(DSZ), .FSZ(FSZ), .FRAC(FRAC), .FSSR(FSSR), .FFT_IMPL(4),
        .RSZ(RSZ), .HSZ(HSZ), .QSZ(QSZ), .IQSZ(IQSZ), .READ_DELAY(READ_DELAY)
    ) dut (
        .adc_clk_i(clk), .clk_i(clk), .adc_rstn_in(rstn),
        .data_in(data_in), .enable_in(enable_in), .dvalid_in(1'b1), .trig_in(trig_in),
        .fft_parallel_in(1'b0),
        .fft_threshold_k_in(16'd4), .fft_peak_start_in(FSZ'(16)), .fft_peak_minimum_in(DSZ'(0)),
        .fft_cfar_guard_in(FSZ'(4)), .fft_cfar_train_in(FSZ'(32)),
        .fft_cfar_onesided_in(1'b0), .fft_cfar_so_in(1'b0),
        .fft_ramp_d0_in(23'd0), .fft_ramp_step_in(23'd0),
        .fft_acq_up_in(acq_up_i), .fft_acq_down_in(acq_down_i),
        .fft_acq_up_done_in(acq_up_done), .fft_acq_down_done_in(acq_down_done),
        .sys_addr_in(32'd0),
        .fft_rdata_up_o(rdata_up), .fft_rdata_down_o(rdata_down),
        .fft_index_flush_in(1'b0), .fft_index_valid_in(index_valid), .fft_hist_index_in(hist_index),
        .fft_hist_rdata_up_o(hist_rdata_up), .fft_hist_rdata_down_o(hist_rdata_down),
        .dma_point_valid_o(dma_point_valid), .dma_point_up_o(dma_point_up), .dma_point_down_o(dma_point_down),
        .dma_point_val_up_o(dma_point_val_up), .dma_point_val_down_o(dma_point_val_down),
        .dma_point_idx_o(dma_point_idx),
        .status_o(status), .fft_done_o(fft_done_o), .fft_peak_ready_o(peak_ready_o),
        .fft_peak_index_up_o(peak_index_up), .fft_peak_index_down_o(peak_index_down),
        .fft_peak_value_up_o(peak_value_up), .fft_peak_value_down_o(peak_value_down),
        .fft_we_cnt(we_cnt), .point_cnt_o(point_cnt), .scan_point_cnt_o(scan_point_cnt),
        .fft_conf_data_in(16'd0), .overflow_cnt_o(overflow_cnt), .diag_o(diag)
    );

    // Registers fft_proc leaves un-reset (hardware powers up 0; xsim starts X).
    initial begin
        dut.fft_rstn = '0;   // hardware powers up with the engine reset asserted
        dut.conf_send = 0; dut.fft_conf_reg = '0;
        dut.out_send = 0;  dut.out_send_ = 0;
        dut.point_cnt = 0; dut.scan_point_cnt = 0;
        dut.fft_hist_index = 0; dut.prev_hist_index = 0;
    end

    // ---- scope acquisition-FSM model ----
    typedef enum logic [2:0] {W_IDLE, W_WAIT1, W_UP, W_WAIT2, W_DOWN} wst_t;
    wst_t wst = W_IDLE; integer wcnt = 0, frame = 0, gap = 0;
    function automatic logic [ASZ-1:0] tone(input int k, input int n);
        return ASZ'($rtoi($cos(2.0*PI*k*n/N)*AMP));
    endfunction
    always @(posedge clk) if (!rstn) begin
        wst <= W_IDLE; wcnt <= 0; frame <= 0; trig_in <= 0; index_valid <= 0; enable_in <= 0; gap <= 0;
    end else begin
        trig_in <= 0; index_valid <= 0; enable_in <= 0;
        case (wst)
        W_IDLE: if (fft_done_o && dut.rstn_i && frame < NFRAMES && $time > 1000) begin
                    if (gap >= GAP) begin
                        trig_in <= 1; index_valid <= 1; hist_index <= HSZ'(frame);
                        acq_up_done <= 0; acq_down_done <= 0;
                        wst <= W_WAIT1; wcnt <= 0; gap <= 0;
                    end else gap <= gap + 1;
                end
        W_WAIT1: if (wcnt >= WAIT1) begin wst <= W_UP; wcnt <= 0; end else wcnt <= wcnt + 1;
        W_UP:    if (wcnt >= ACQ_UP) begin wst <= W_WAIT2; wcnt <= 0; acq_up_done <= 1; end
                 else begin enable_in <= 1; data_in <= tone(K1, wcnt); wcnt <= wcnt + 1; end
        W_WAIT2: if (wcnt >= WAIT2) begin wst <= W_DOWN; wcnt <= 0; end else wcnt <= wcnt + 1;
        W_DOWN:  if (wcnt >= ACQ_DOWN) begin wst <= W_IDLE; wcnt <= 0; acq_down_done <= 1; frame <= frame + 1; end
                 else begin enable_in <= 1; data_in <= tone(K2, wcnt); wcnt <= wcnt + 1; end
        endcase
    end

    // ---- xfft-core-level monitors ----
    wire x_tvalid = dut.gen_fft_native.fft_i.fft_ssr_native_bd_i.xfft_0.s_axis_data_tvalid;
    wire x_tready = dut.gen_fft_native.fft_i.fft_ssr_native_bd_i.xfft_0.s_axis_data_tready;
    wire x_tlast  = dut.gen_fft_native.fft_i.fft_ssr_native_bd_i.xfft_0.s_axis_data_tlast;
    wire y_tvalid = dut.gen_fft_native.fft_i.fft_ssr_native_bd_i.xfft_0.m_axis_data_tvalid;
    wire y_tready = dut.gen_fft_native.fft_i.fft_ssr_native_bd_i.mag_0.s_axis_TREADY; // realtime xfft has no m_axis tready pin; the sink is mag_0
    logic in_frame = 0;
    integer in_gaps = 0, out_stalls = 0, xin_frames = 0, xout_beats = 0;
    integer ev_in = 0, ev_out = 0, ev_tm = 0, ev_tu = 0, frames_started = 0;
    integer feed_gaps = 0;           // fft_proc-level: fin_rd && half_open && !valid
    always @(posedge clk) if (rstn) begin
        if (x_tvalid && x_tready) begin
            if (!in_frame) in_frame <= 1;
            if (x_tlast) begin in_frame <= 0; xin_frames <= xin_frames + 1; end
        end else if (in_frame && x_tready && !x_tvalid)
            in_gaps <= in_gaps + 1;
        if (y_tvalid && !y_tready) out_stalls <= out_stalls + 1;
        if (y_tvalid && y_tready) xout_beats <= xout_beats + 1;
        if (dut.ev_in_halt)       ev_in <= ev_in + 1;
        if (dut.ev_out_halt)      ev_out <= ev_out + 1;
        if (dut.ev_tlast_missing) ev_tm <= ev_tm + 1;
        if (dut.ev_tlast_unexp)   ev_tu <= ev_tu + 1;
        if (dut.fft_frame_start)  frames_started <= frames_started + 1;
        if (dut.fin_rd && dut.half_open && !dut.fft_saxi_valid) feed_gaps <= feed_gaps + 1;
    end

    // ---- event trace (first EV_MAX events) ----
    integer ev_n = 0;
    wire y_tlast  = dut.gen_fft_native.fft_i.fft_ssr_native_bd_i.xfft_0.m_axis_data_tlast;
    logic ostall_d = 0, done_d = 0, rstn_d = 0, finrst_d = 0;
    always @(posedge clk) begin
        ostall_d <= (y_tvalid && !y_tready);
        done_d <= fft_done_o; rstn_d <= dut.rstn_i; finrst_d <= dut.fin_rst;
        if (ev_n < 120) begin
            if (dut.rstn_i && !rstn_d)                     begin ev_n++; $display("  [%0t] rstn_i release", $time); end
            if (fft_done_o && !done_d)                     begin ev_n++; $display("  [%0t] fft_done_o rise", $time); end
            if (dut.fin_rst && !finrst_d)                  begin ev_n++; $display("  [%0t] fin_rst pulse", $time); end
            if (trig_in)                                   begin ev_n++; $display("  [%0t] TRIG frame %0d", $time, frame); end
            if (dut.half_open && !dut.fft_saxi_valid && dut.fin_rd) ;
            if (x_tvalid && x_tready && !in_frame)         begin ev_n++; $display("  [%0t] xfft IN  frame start", $time); end
            if (x_tvalid && x_tready && x_tlast)           begin ev_n++; $display("  [%0t] xfft IN  tlast", $time); end
            if (dut.fft_saxi_valid && dut.fft_saxi_rdy && dut.fft_saxi_last) begin ev_n++; $display("  [%0t] feed tlast (we_cnt=%0d up_in=%0d)", $time, we_cnt, dut.up_in); end
            if (y_tvalid && y_tlast)                       begin ev_n++; $display("  [%0t] xfft OUT tlast (tready=%0d)", $time, y_tready); end
            if ((y_tvalid && !y_tready) && !ostall_d)      begin ev_n++; $display("  [%0t] OUT STALL begins (mag tready low) fifo_peak_in: valid=%0d ready=%0d", $time, dut.peak_in_valid, dut.peak_in_ready); end
            if (dut.fft_maxi_valid && dut.fft_maxi_rdy && dut.fft_maxi_last) begin ev_n++; $display("  [%0t] mag OUT tlast", $time); end
            if (dut.peak_in_valid && dut.peak_in_ready && dut.peak_in_last) begin ev_n++; $display("  [%0t] pd IN tlast", $time); end
            if (dut.peak_out_valid)                        begin ev_n++; $display("  [%0t] pd OUT valid data=%h", $time, dut.peak_out_data); end
            if (dut.ev_tlast_unexp)                        begin ev_n++; $display("  [%0t] EV tlast_unexpected", $time); end
            if (dut.ev_tlast_missing)                      begin ev_n++; $display("  [%0t] EV tlast_missing", $time); end
            if (dut.ev_in_halt)                            begin ev_n++; $display("  [%0t] EV in_halt", $time); end
            if (dut.fft_frame_start)                       begin ev_n++; $display("  [%0t] xfft frame_started", $time); end
        end
    end

    // ---- TLAST census at the xfft data input ----
    integer tl_hi = 0, tl_hi_nov = 0, tl_shown = 0;
    logic x_tlast_d = 0;
    always @(posedge clk) if (rstn) begin
        x_tlast_d <= x_tlast;
        if (x_tlast) tl_hi <= tl_hi + 1;
        if (x_tlast && !x_tvalid) tl_hi_nov <= tl_hi_nov + 1;
        if (x_tlast && !x_tlast_d && tl_shown < 12) begin
            tl_shown <= tl_shown + 1;
            $display("  [%0t] x_tlast RISES (tvalid=%0d tready=%0d) in_frame=%0d", $time, x_tvalid, x_tready, in_frame);
        end
        if (!x_tlast && x_tlast_d && tl_shown < 12)
            $display("  [%0t] x_tlast falls (tvalid=%0d)", $time, x_tvalid);
    end
    final $display("TLAST census: cycles high=%0d, high without tvalid=%0d", tl_hi, tl_hi_nov);

    // ---- beats-per-frame census on the output side (uncapped) ----
    integer yb = 0, mb = 0, pb = 0; logic y_busy = 0;
    always @(posedge clk) if (rstn) begin
        if (y_tvalid && y_tready) begin
            if (!y_busy) begin y_busy <= 1; $display("  [%0t] xfft OUT frame begins", $time); end
            yb <= yb + 1;
            if (y_tlast) begin y_busy <= 0; $display("  [%0t] xfft OUT frame ends: %0d beats", $time, yb + 1); yb <= 0; end
        end
        if (dut.fft_maxi_valid && dut.fft_maxi_rdy) begin
            mb <= mb + 1;
            if (dut.fft_maxi_last) begin $display("  [%0t] mag OUT frame ends: %0d beats", $time, mb + 1); mb <= 0; end
        end
        if (dut.peak_in_valid && dut.peak_in_ready) begin
            pb <= pb + 1;
            if (dut.peak_in_last) begin $display("  [%0t] pd IN frame ends: %0d beats", $time, pb + 1); pb <= 0; end
        end
        if (dut.peak_out_valid) $display("  [%0t] pd OUT: k=%0d valid=%0d val=%0d", $time, dut.peak_out_data[DSZ+IDX:DSZ+1] >> FRAC, dut.peak_out_data[DSZ], dut.peak_out_data[DSZ-1:0]);
        if (dut.peak_ready_pretrig) $display("  [%0t] engine pretrig: peak_up=%0d hist_idx_o=%0d", $time, dut.peak_up, dut.fft_hist_index_o);
    end

    // ---- peak capture per frame ----
    integer pk_frames = 0, bad_peaks = 0;
    integer max_count = 0;
    always @(posedge clk) if (rstn) begin
        if (dut.fin_rd_count > max_count) max_count <= dut.fin_rd_count;
        // One DMA point per completed up/down pair, on the engine clock — the
        // product path (the sys-side peak regs lag behind a CDC handshake).
        if (dma_point_valid) begin
            int ku = dma_point_up >> FRAC, kd = dma_point_down >> FRAC;
            pk_frames <= pk_frames + 1;
            $display("  [t=%0t] point %0d (idx %0d): up bin %0d (val %0d)  down bin %0d (val %0d)",
                     $time, pk_frames, dma_point_idx, ku, dma_point_val_up, kd, dma_point_val_down);
            if (ku < K1-1 || ku > K1+1 || kd < K2-1 || kd > K2+1) bad_peaks <= bad_peaks + 1;
        end
    end

    // ---- run ----
    initial begin
        repeat (40) @(posedge clk); rstn = 1;
        wait (pk_frames >= NFRAMES);
        repeat (200) @(posedge clk);
        report();
        $finish;
    end
    initial begin
        #4000000;
        $display("TIMEOUT: frames_started=%0d xin_frames=%0d pk_frames=%0d wst=%0d fft_done=%0d half_open=%0d fin_rd=%0d fin_rd_count=%0d acq_beats=%0d we_cnt=%0d",
                 frames_started, xin_frames, pk_frames, wst, fft_done_o, dut.half_open, dut.fin_rd, dut.fin_rd_count, dut.acq_beats, we_cnt);
        report();
        $finish;
    end
    task report;
        string why = "";
        if (in_gaps)     why = {why, " tvalid-gaps-in-frame"};
        if (out_stalls)  why = {why, " output-stalled"};
        if (ev_in|ev_out|ev_tm|ev_tu) why = {why, " xfft-events"};
        if (status != 0) why = {why, " sticky-status"};
        if (bad_peaks)   why = {why, " wrong-peaks"};
        if (overflow_cnt) why = {why, " fifo-overflow"};
        if (pk_frames < NFRAMES) why = {why, " missing-frames"};
        $display("RESULT ACQ_UP=%0d ACQ_DOWN=%0d WAIT1=%0d WAIT2=%0d GAP=%0d NFRAMES=%0d : %s%s | xin_frames=%0d frames_started=%0d pk_frames=%0d in_gaps=%0d feed_gaps=%0d out_stalls=%0d ev_in=%0d ev_out=%0d ev_tm=%0d ev_tu=%0d status=%b overflow=%0d diag=%h bad_peaks=%0d max_fifo_words=%0d/%0d",
                 ACQ_UP, ACQ_DOWN, WAIT1, WAIT2, GAP, NFRAMES, (why == "") ? "PASS" : "FAIL", why,
                 xin_frames, frames_started, pk_frames, in_gaps, feed_gaps, out_stalls,
                 ev_in, ev_out, ev_tm, ev_tu, status, overflow_cnt, diag, bad_peaks, max_count, (1<<QSZ)/FSSR);
    endtask
endmodule
