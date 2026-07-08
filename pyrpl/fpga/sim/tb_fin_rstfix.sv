// Mechanism probe + fix validation for the per-frame fin FIFO reset
// (HANDOFF_fft_zero_frames.md). Replicates the LIVE failing configuration:
// IMPL=4 N9 (N=512, FSSR=4 -> 128 beats/half), CLK_SEL=0 (single clock),
// free-running chirp trigger every PERIOD samples (417 < N at 300 kHz),
// serial up+down windows wait1/acq1/wait2/acq2 = 40/36/55/105 samples.
//
// Replicated VERBATIM from the RTL:
//   - scope acquisition FSM S_IDLE..S_FFT_DOWN (red_pitaya_scope.sv:1425-1488)
//     including the trigger gate on fft_done_o and the acq_*_done flags
//   - fft_proc feed engine + fin FIFO + post-pad logic (fft_proc.sv)
// A behavioral sink stands in for the xfft (optional -d BACKPRESSURE stalls).
//
// Build variants:
//   (default)  OLD reset: combinational fin_rst = trig_i && fft_done_o,
//              wr_en/rd side NOT gated on wr_rst_busy/rd_rst_busy
//   -d RSTFIX  NEW reset: registered 4-cycle pulse + rst_busy gating + gated
//              data_valid (the fix under test)
//
// Per swept config {period, wait1, acq1, acq2} the TB reports:
//   violW/violR — cycles with wr_en/rd_en active while rst or rst_busy
//                 (the XPM protocol violation the handoff blames)
//   zero        — frame halves fed with ZERO real data beats (the field bug)
//   anom        — data-contiguity skips / wrong frame length
//   drop        — writes rejected by the fix's gate (rst shadow)
//   ovf         — fin_full write rejections
//   postpad     — zero beats substituted inside the data phase (under-run)
`timescale 1ns/1ps

module tb_fin_rstfix;
    localparam int FSSR = 4, ASZ = 14, FSZ = 9;
    localparam int QSZ  = FSZ - $clog2(FSSR);         // 7 -> depth 128 samples
    localparam int FFT_LENGTH          = 128;         // beats per half
    localparam int FFT_LENGTH2         = 256;         // up+down
    localparam int FFT_LENGTH_PLUS_TWO = 130;

    logic clk = 0; always #4 clk = ~clk;
    logic rstn = 0;

    // ---- swept config (quasi-static; changed only under reset) ----
    int period_cfg = 417;
    int wait1_cfg  = 40, acq1_cfg = 36, wait2_cfg = 55, acq2_cfg = 105;
    // feed-side conf mirrors fft_proc: beats = samples >> SSR_BITS (floor)
    int acq_up_beats, acq_down_beats, padding_up, padding_down;
    always_comb begin
        acq_up_beats   = acq1_cfg >> 2;
        acq_down_beats = acq2_cfg >> 2;
        padding_up     = FFT_LENGTH - acq_up_beats;
        padding_down   = FFT_LENGTH - acq_down_beats;
    end

    // ---- free-running chirp trigger ----
    int   trig_cnt = 0;
    logic chirp_trig;
    always @(posedge clk)
        if (!rstn) trig_cnt <= 0;
        else       trig_cnt <= (trig_cnt >= period_cfg-1) ? 0 : trig_cnt + 1;
    assign chirp_trig = rstn && (trig_cnt == 0);

    // =========================================================================
    // Scope acquisition FSM (verbatim structure, red_pitaya_scope.sv)
    // =========================================================================
    typedef enum int {S_IDLE, S_WAIT1, S_FFT_UP, S_WAIT2, S_FFT_DOWN} state_t;
    state_t fft_state;
    int     fft_state_cnt;
    logic   fft_acq_up_done, fft_acq_down_done;
    wire    fft_dvalid = 1'b1;                        // decimation = 1
    logic   fft_done_o;                               // from feed engine (CDC'd)

    always @(posedge clk)
    if (!rstn) begin
        fft_state <= S_IDLE; fft_state_cnt <= 0;
        fft_acq_up_done <= 0; fft_acq_down_done <= 0;
    end else case (fft_state)
        S_IDLE:
            if (chirp_trig && fft_done_o) begin
                fft_state_cnt <= 0;
                fft_state <= S_WAIT1;
                fft_acq_up_done   <= 0;
                fft_acq_down_done <= 0;
            end
        S_WAIT1:
            if (fft_state_cnt >= wait1_cfg) begin
                fft_state_cnt <= 0; fft_state <= S_FFT_UP;
            end else if (fft_dvalid) fft_state_cnt <= fft_state_cnt + 1;
        S_FFT_UP:
            if (fft_state_cnt >= acq1_cfg) begin
                fft_state_cnt <= 0; fft_state <= S_WAIT2;
                fft_acq_up_done <= 1;
            end else if (fft_dvalid) fft_state_cnt <= fft_state_cnt + 1;
        S_WAIT2:
            if (fft_state_cnt >= wait2_cfg) begin
                fft_state_cnt <= 0; fft_state <= S_FFT_DOWN;
            end else if (fft_dvalid) fft_state_cnt <= fft_state_cnt + 1;
        S_FFT_DOWN:
            if (fft_state_cnt >= acq2_cfg) begin
                fft_state_cnt <= 0; fft_state <= S_IDLE;
                fft_acq_down_done <= 1;
            end else if (fft_dvalid) fft_state_cnt <= fft_state_cnt + 1;
    endcase

    wire enable_in = (fft_state == S_FFT_UP) || (fft_state == S_FFT_DOWN);

    // Canonical trigger-ACCEPT pulse (the FSM's S_IDLE->S_WAIT1 condition).
    // -d TRIGACCEPT feeds the engine with THIS instead of the raw chirp trigger,
    // closing the done-CDC lag window: a chirp landing while the feed's
    // clk-domain done is already 1 but the FSM's adc-domain fft_done_o is still
    // 0 is rejected by the FSM (no acquisition) yet STARTS the raw-triggered
    // feed -> a whole frame of post-padded zeros (acq_done still set from the
    // previous frame). The period sweep below walks chirp edges through that
    // 2-3 cycle window.
    wire scope_accept = (fft_state == S_IDLE) && chirp_trig && fft_done_o;
`ifdef TRIGACCEPT
    wire engine_trig_in = scope_accept;
`else
    wire engine_trig_in = chirp_trig;
`endif

    // ---- ADC data: incrementing counter, sampled by the writes ----
    logic [ASZ-1:0] data_in = 0;

    // =========================================================================
    // fft_proc input stage (verbatim: 1-cycle input registration)
    // =========================================================================
    logic [ASZ-1:0] data_i;
    logic           enable_i, dvalid_i, trig_i;
    always @(posedge clk) begin
        data_i   <= data_in;
        enable_i <= enable_in;
        dvalid_i <= fft_dvalid;
        trig_i   <= engine_trig_in;
    end
    // advance the "ADC" counter so consecutive accepted-window samples are
    // consecutive integers (contiguity oracle)
    always @(posedge clk) if (rstn && enable_in && fft_dvalid) data_in <= data_in + 1;

    // CDCs (single clock, CLK_SEL=0: pure 2-FF delays)
    logic t1, fft_trig;
    always @(posedge clk) begin t1 <= trig_i; fft_trig <= t1; end
    logic d1, d2;
    logic fft_done, up_in;
    always @(posedge clk) begin d1 <= fft_done && up_in; d2 <= d1; end
    assign fft_done_o = d2;
    logic [1:0] a1, acq_done_clk;
    always @(posedge clk) begin
        a1 <= {fft_acq_down_done, fft_acq_up_done}; acq_done_clk <= a1;
    end
    wire acq_done = up_in ? acq_done_clk[0] : acq_done_clk[1];

    // =========================================================================
    // fin FIFO + per-frame reset (OLD vs RSTFIX)
    // =========================================================================
    logic [FSSR*ASZ-1:0] fin_dout;
    logic                fin_rd, fin_dvalid;
    logic                padding_done;
    logic [FSZ-1:0]      padding_cnt;
    logic                fin_full;
    wire                 fin_wr_rst_busy, fin_rd_rst_busy;
    wire                 fin_wr_en = enable_i && dvalid_i;

`ifdef RSTFIX
    localparam FIN_RST_CYC = 2;
    logic [FIN_RST_CYC-1:0] fin_rst_sr;
    logic fft_done_o_d;
    always @(posedge clk) begin
        fft_done_o_d <= fft_done_o;
        if (!rstn) fin_rst_sr <= '1;
        else       fin_rst_sr <= {fin_rst_sr[FIN_RST_CYC-2:0], fft_done_o && !fft_done_o_d};
    end
    wire fin_rst   = |fin_rst_sr;
    wire fin_wr_ok = !fin_rst && !fin_wr_rst_busy;
    logic [15:0] rst_drop_cnt;
    always @(posedge clk)
        if (!rstn) rst_drop_cnt <= 0;
        else if (fin_wr_en && !fin_wr_ok) rst_drop_cnt <= rst_drop_cnt + 1;
    wire fin_dvalid_g = fin_dvalid && !fin_rd_rst_busy;
`else
    wire fin_rst   = !rstn || (trig_i && fft_done_o);
    wire fin_wr_ok = 1'b1;
    logic [15:0] rst_drop_cnt = 0;
    wire fin_dvalid_g = fin_dvalid;
`endif

    logic fft_saxi_valid, fft_saxi_rdy, fft_saxi_last;
    logic [FSSR*ASZ-1:0] fft_data_i;
    wire rd_en_w = padding_done & fft_saxi_rdy & fin_rd & fin_dvalid_g;

    xpm_fifo_async #(
        .FIFO_WRITE_DEPTH(1<<QSZ), .WRITE_DATA_WIDTH(ASZ), .READ_DATA_WIDTH(ASZ*FSSR),
        .FIFO_READ_LATENCY(0), .USE_ADV_FEATURES("1001"), .READ_MODE("fwft")
    ) fifo_in (
        .rst(fin_rst),
        .wr_clk(clk), .wr_en(fin_wr_en && fin_wr_ok), .din(data_i),
        .wr_rst_busy(fin_wr_rst_busy),
        .rd_clk(clk), .rd_en(rd_en_w),
        .dout(fin_dout), .data_valid(fin_dvalid),
        .rd_rst_busy(fin_rd_rst_busy),
        .overflow(fin_full)
    );

    // =========================================================================
    // Feed engine (verbatim from fft_proc.sv incl. post-pad + zero-frame diag)
    // =========================================================================
    logic [31:0] fft_we_cnt;
    logic        fft_we_one, fft_we_length_plus_one;
    logic        half_had_data;
    logic [15:0] zero_frame_cnt;
    logic [15:0] overflow_cnt;

    assign fft_saxi_last  = fft_we_one || fft_we_length_plus_one;
    wire   fft_postpad    = padding_done && acq_done && !fin_dvalid_g;
    assign fft_saxi_valid = (!padding_done || fin_dvalid_g || fft_postpad) && fin_rd;
    assign fft_data_i     = (padding_done && fin_dvalid_g) ? fin_dout : '0;

    always @(posedge clk)
    if (!rstn) begin
        fft_we_cnt <= 0; fft_we_one <= 0; fft_we_length_plus_one <= 0;
        fin_rd <= 0; fft_done <= 1; up_in <= 1;
        padding_cnt <= 0; padding_done <= 0;
        overflow_cnt <= 0; half_had_data <= 0; zero_frame_cnt <= 0;
    end else begin
        if (fin_full) overflow_cnt <= overflow_cnt + 1;
        if (fft_trig && fft_done && up_in) begin
            fft_we_cnt <= FFT_LENGTH2;
            fft_we_one <= 0; fft_we_length_plus_one <= 0;
            fin_rd <= 1; padding_cnt <= padding_up[FSZ-1:0];
            padding_done <= 0; fft_done <= 0;
        end else begin
            if (!fft_done && fft_saxi_valid && fft_saxi_rdy) begin
                if (fft_we_one) begin
                    up_in <= 1; fin_rd <= 0;
                end else if (fft_we_length_plus_one) begin
                    up_in <= 0; padding_cnt <= padding_down[FSZ-1:0];
                    padding_done <= 0;
                end else if (!padding_done) begin
                    padding_cnt <= padding_cnt - 1;
                    padding_done <= padding_cnt == 1;
                end
                if (fft_we_one || fft_we_length_plus_one) begin
                    if (!half_had_data && !(padding_done && fin_dvalid_g)
                        && (up_in ? acq_up_beats : acq_down_beats) != 0)
                        zero_frame_cnt <= zero_frame_cnt + 1;
                    half_had_data <= 0;
                end else if (padding_done && fin_dvalid_g)
                    half_had_data <= 1;
                fft_we_cnt <= fft_we_cnt - 1;
                fft_done <= fft_we_one;
                fft_we_one <= fft_we_cnt == 2;
                fft_we_length_plus_one <= fft_we_cnt == FFT_LENGTH_PLUS_TWO;
            end
        end
    end

    // ---- behavioral xfft sink ----
`ifdef BACKPRESSURE
    integer bp = 0;
    logic [31:0] lfsr = 32'h1234_5678;
    always @(posedge clk) lfsr <= {lfsr[30:0], lfsr[31]^lfsr[21]^lfsr[1]^lfsr[0]};
    always @(posedge clk) begin
        if (bp > 0) bp <= bp - 1;
        else if (lfsr[7:0] < 40) bp <= (lfsr[11:8] + 1);
    end
    assign fft_saxi_rdy = (bp == 0);
`else
    assign fft_saxi_rdy = 1'b1;
`endif

    // =========================================================================
    // Monitors
    // =========================================================================
    // XPM protocol violations: enables active while the reset is visible in
    // that enable's own domain. wr side: rst is wr_clk-synchronous, so
    // rst||wr_rst_busy. rd side: only rd_rst_busy — a pop in the 1-2 cycle gap
    // before rd_rst_busy asserts is a legal pre-reset read (the flush then
    // clears whatever that word belonged to).
    integer violW = 0, violR = 0, violR_dbg = 0;
    always @(posedge clk) if (rstn) begin
        if ((fin_wr_en && fin_wr_ok) && (fin_rst || fin_wr_rst_busy)) violW <= violW + 1;
        if (rd_en_w && fin_rd_rst_busy) begin
            violR <= violR + 1;
            if (violR_dbg < 8) begin
                $display("  [t=%0t] violR: rst=%b wrbusy=%b rdbusy=%b dv=%b pdone=%b up=%0b wecnt=%0d",
                         $time, fin_rst, fin_wr_rst_busy, fin_rd_rst_busy,
                         fin_dvalid, padding_done, up_in, fft_we_cnt);
                violR_dbg <= violR_dbg + 1;
            end
        end
    end

    // data contiguity + frame length
    integer beats = 0, frames = 0, anomalies = 0, postpads = 0;
    logic [ASZ-1:0] exp_lane0;
    logic           have_exp = 0;
    logic [ASZ-1:0] frame_base;
    // capture the first sample value of each accepted frame at trigger accept
    always @(posedge clk)
        if (rstn && fft_state == S_IDLE && chirp_trig && fft_done_o) begin
            frame_base <= data_in;
            have_exp   <= 0;    // frame's first data beat re-anchors on frame_base
        end
    always @(posedge clk)
    if (!rstn) begin
        // clear per-frame monitor state across config boundaries: a config
        // reset can abort a frame mid-feed, and a carried partial beat count
        // would flag a bogus FRAME LEN on the next config's first frame
        beats <= 0; have_exp <= 0;
    end else begin
        if (fft_saxi_valid && fft_saxi_rdy) begin
            beats <= beats + 1;
            if (padding_done && fin_dvalid_g) begin
                if (have_exp) begin
                    if (fin_dout[ASZ-1:0] != exp_lane0) begin
                        $display("  [t=%0t] *** DATA SKIP frame %0d: lane0 %0d expected %0d",
                                 $time, frames, fin_dout[ASZ-1:0], exp_lane0);
                        anomalies <= anomalies + 1;
                    end
                end else if (fin_dout[ASZ-1:0] != frame_base) begin
                    $display("  [t=%0t] *** FRAME BASE frame %0d: lane0 %0d expected %0d (stale/lost)",
                             $time, frames, fin_dout[ASZ-1:0], frame_base);
                    anomalies <= anomalies + 1;
                end
                exp_lane0 <= fin_dout[ASZ-1:0] + FSSR;
                have_exp  <= 1;
            end
            if (fft_postpad) postpads <= postpads + 1;
            if (fft_saxi_last) begin
                if (beats + 1 != FFT_LENGTH) begin
                    $display("  [t=%0t] *** FRAME LEN half %0d: %0d beats (expected %0d)",
                             $time, frames, beats+1, FFT_LENGTH);
                    anomalies <= anomalies + 1;
                end
                frames <= frames + 1;   // counts halves
                beats  <= 0;
            end
        end
    end

    // hang detector
    integer idle = 0, last_frames = -1, last_beats = -1;
    logic   hung = 0;
    always @(posedge clk) if (rstn) begin
        if (frames != last_frames || beats != last_beats) begin
            idle <= 0; last_frames <= frames; last_beats <= beats;
        end else if (!fft_done)
            idle <= idle + 1;
        if (idle > 20000) begin
            $display("  [t=%0t] *** HANG: halves=%0d beats=%0d fin_dvalid=%0b",
                     $time, frames, beats, fin_dvalid);
            hung <= 1;
            idle <= 0;
        end
    end

    // =========================================================================
    // Sweep driver
    // =========================================================================
    integer s_frames, s_zero, s_anom, s_violW, s_violR, s_drop, s_ovf, s_pp;
    integer viol_total = 0;       // protocol violations, any config
    integer unexplained = 0;      // zero/anom/hang in configs with NO counted drops
    integer bad_total = 0;        // everything (old-build mechanism verdict)

    task automatic run_config(input int period, input int w1, input int a1,
                              input int a2, input int ntrig);
        int d_zero, d_anom, d_violW, d_violR;
        // full reset between configs -> clean FIFO/counters/FSMs
        rstn = 0;
        period_cfg = period; wait1_cfg = w1; acq1_cfg = a1; acq2_cfg = a2;
        wait2_cfg = 55;
        repeat (30) @(posedge clk);
        s_frames = frames; s_zero = zero_frame_cnt; s_anom = anomalies;
        s_violW = violW;   s_violR = violR;         s_drop = rst_drop_cnt;
        s_ovf = overflow_cnt; s_pp = postpads;
        rstn = 1;
        repeat (30) @(posedge clk);
        repeat (ntrig * period) @(posedge clk);
        // drain the frame in flight; give up if the feed wedged (old build can)
        fork
            wait (fft_done == 1'b1);
            repeat (4 * period) @(posedge clk);
        join_any
        disable fork;
        if (fft_done != 1'b1) hung = 1;
        repeat (50) @(posedge clk);
        d_zero  = zero_frame_cnt - s_zero;  d_anom  = anomalies - s_anom;
        d_violW = violW - s_violW;          d_violR = violR - s_violR;
        $display("CFG period=%0d wait1=%0d acq1=%0d acq2=%0d : halves=%0d zero=%0d anom=%0d violW=%0d violR=%0d drop=%0d ovf=%0d postpad=%0d%s",
                 period, w1, a1, a2,
                 frames - s_frames, d_zero, d_anom, d_violW, d_violR,
                 rst_drop_cnt - s_drop, overflow_cnt - s_ovf, postpads - s_pp,
                 hung ? " HUNG" : "");
        viol_total += d_violW + d_violR;
        bad_total  += d_zero + d_anom + d_violW + d_violR + hung;
        // a config whose writes collided with the reset shadow reports them in
        // rst_drop; data anomalies there are the expected, MEASURED consequence.
        // Anomalies with drop==0 have no innocent explanation.
        if (rst_drop_cnt - s_drop == 0)
            unexplained += d_zero + d_anom + hung;
        hung = 0;
        violR_dbg = 0;
    endtask

    initial begin
        int w1_sweep[12] = '{40, 32, 24, 16, 12, 8, 6, 4, 3, 2, 1, 0};
`ifdef RSTFIX
        $display("=== BUILD: RSTFIX (registered stretched rst + rst_busy gating) ===");
`else
        $display("=== BUILD: OLD (combinational fin_rst, no rst_busy gating) ===");
`endif
        // live failing operating point: N(512) > period(417), sweep wait1
        foreach (w1_sweep[i])
            run_config(417, w1_sweep[i], 36, 105, 60);
        // slow-chirp control (period >> frame + feed): field-clean case
        run_config(700, 40, 36, 105, 40);
        // read-side stress: minimal pre-padding (acq1 = 508 -> 1 pad beat) so
        // data reads begin right after the trigger, inside the reset shadow
        run_config(1500, 2, 508, 105, 25);
        run_config(1500, 0, 508, 105, 25);
        // tight done->trigger gap: period barely above the feed time (~260), so
        // the next trigger lands right after fft_done rises and the reset shadow
        // overlaps the writes again unless wait1 covers the remainder. The fix
        // must report any losses here as rst_drop, never silently.
        run_config(262, 40, 36, 105, 60);
        run_config(262, 0, 36, 105, 60);
        // spurious-start probe: walk the chirp edge through the done-CDC lag
        // window (feed done ~ trig+240..258 at wait1=40). Raw-trigger gating
        // (no TRIGACCEPT) shows whole zero frames here; TRIGACCEPT must not.
        for (int p = 244; p <= 264; p += 2)
            run_config(p, 40, 36, 105, 40);

`ifdef RSTFIX
        // fix must be protocol-clean in EVERY config (gating guarantees it),
        // and every data anomaly must be explained by counted rst_drops
        if (viol_total == 0 && unexplained == 0)
            $display("=== RESULT: PASS (0 violations; every anomaly accounted by rst_drop) ===");
        else
            $display("=== RESULT: FAIL (violations=%0d, unexplained=%0d) ===",
                     viol_total, unexplained);
`else
        if (bad_total == 0)
            $display("=== RESULT: OLD CODE CLEAN in sim — reset overlap NOT reproduced; field zero-frames must come from something sim doesn't model (see handoff review) ===");
        else
            $display("=== RESULT: OLD CODE shows %0d bad events (violations=%0d) — mechanism reproduced ===",
                     bad_total, viol_total);
`endif
        $finish;
    end

    initial begin
        #60000000;  // 60 ms guard
        $display("=== TIMEOUT ===");
        $finish;
    end
endmodule
