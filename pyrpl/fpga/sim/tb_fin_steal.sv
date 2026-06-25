// Focused test of the fft_proc INPUT FSM + fin async FIFO, to check whether a
// data beat can be "stolen" (popped without being consumed) at frame boundaries,
// and whether Fix 1 (gate rd_en with fin_dvalid) changes it.
//
// The FSM + fin FIFO + fft_saxi logic below are copied VERBATIM from fft_proc.sv.
// A behavioral xfft drives fft_saxi_rdy (with backpressure bursts) and counts the
// input beats between tlasts. fin is continuously filled with a sample COUNTER, so
// any popped-but-not-consumed beat shows up as a discontinuity in the consumed
// data stream. Build twice: -d FIX1 (gated rd_en) vs without.
`timescale 1ns/1ps

module tb_fin_steal;
    localparam int FSSR = 4, ASZ = 14, FSZ = 9, QSZ = 6;
    localparam int ACQ_UP = 100, ACQ_DOWN = 90;   // != so up/down differ (like hw)
    localparam int FFT_LENGTH = 128;              // beats/frame (2^7, FSZ-SSR_BITS=7)
    localparam int FFT_LENGTH2 = 256;             // up+down
    localparam int FFT_LENGTH_PLUS_TWO = 130;
    localparam int PADDING_UP   = FFT_LENGTH - ACQ_UP;
    localparam int PADDING_DOWN = FFT_LENGTH - ACQ_DOWN;
    localparam int NTRIG = 12;

    logic clk = 0; always #4 clk = ~clk;
    logic rstn = 0;

    // ---- ADC writer: continuous fill with a counter ----
    logic                dvalid_i = 1;
    logic [ASZ-1:0]      data_i = 0;
    logic                fin_full;        // overflow flag (should stay 0)
    logic                fin_full_flag;   // FIFO full -> gate writes (no overflow)
    // FINITE source: supply EXACTLY (acq_up+acq_down)*FSSR samples per acquisition,
    // then stop. If the FSM reads one beat too many (a stolen beat), fin starves and
    // the frame hangs -> the hang detector fires. This is the real failure condition.
    integer              budget = 0;
    logic                budget_load; logic [31:0] budget_val;
    wire                 enable_i = (budget > 0);
    wire                 fin_wr   = enable_i & dvalid_i & ~fin_full_flag;
    always @(posedge clk) begin
        if (!rstn)             budget <= 0;
        else if (budget_load)  budget <= budget_val;
        else if (fin_wr)       budget <= budget - 1;
    end
    always @(posedge clk) if (rstn && fin_wr) data_i <= data_i + 1;

    // ---- trigger ----
    logic trig_i = 0, fft_trig;
    assign fft_trig = trig_i;                      // same-clock (B2: clk_i==adc_clk)

    // ---- fin async FIFO (verbatim params from fft_proc.sv) ----
    logic [FSSR*ASZ-1:0] fin_dout;
    logic                fin_dvalid, fin_rd;
    logic                padding_done;
    logic [FSZ-1:0]      padding_cnt;
    logic                fft_done, up_in;
    logic                fft_done_o;
    logic                fin_rst;
    assign fin_rst = !rstn || (trig_i && fft_done_o);

    logic                fft_saxi_valid, fft_saxi_rdy, fft_saxi_last;
    logic [FSSR*ASZ-1:0] fft_data_i;

`ifdef FIX1
    wire rd_en_w = padding_done & fft_saxi_rdy & fin_rd & fin_dvalid;
`else
    wire rd_en_w = padding_done & fft_saxi_rdy & fin_rd;
`endif
    xpm_fifo_async #(
        .FIFO_WRITE_DEPTH(1<<QSZ), .WRITE_DATA_WIDTH(ASZ), .READ_DATA_WIDTH(ASZ*FSSR),
        .FIFO_READ_LATENCY(0), .USE_ADV_FEATURES("1001"), .READ_MODE("fwft")
    ) fifo_in (
        .rst(fin_rst), .wr_clk(clk), .wr_en(fin_wr), .din(data_i), .full(fin_full_flag),
        .rd_clk(clk), .rd_en(rd_en_w),
        .dout(fin_dout), .data_valid(fin_dvalid), .overflow(fin_full)
    );
    // diagnostics: real pops (rd_en while data present) and flushes after reset
    integer popcount=0, flushcount=0; logic fin_rst_d=0;
    always @(posedge clk) if (rstn) begin
        if (rd_en_w & fin_dvalid) popcount <= popcount + 1;
        fin_rst_d <= fin_rst;
        if (fin_rst & ~fin_rst_d) flushcount <= flushcount + 1;
    end

    assign fft_saxi_last  = (fft_we_one || fft_we_length_plus_one);
    assign fft_saxi_valid = (!padding_done || fin_dvalid) && fin_rd;
    assign fft_data_i     = padding_done ? fin_dout : '0;

    // ---- input FSM (verbatim from fft_proc.sv:658-702) ----
    logic [31:0] fft_we_cnt;
    logic        fft_we_one, fft_we_length_plus_one;
    logic [15:0] overflow_cnt;

    always @(posedge clk)
    if (rstn == 1'b0) begin
        fft_we_cnt <= 0; fft_we_one <= 0; fft_we_length_plus_one <= 0;
        fin_rd <= 0; fft_done <= 1; up_in <= 1; padding_cnt <= 0; padding_done <= 0;
    end else begin
        if (fin_full) overflow_cnt <= overflow_cnt + 1;
        if (fft_trig && fft_done && up_in) begin
            fft_we_cnt <= FFT_LENGTH2;
            fft_we_one <= 0; fft_we_length_plus_one <= 0;
            fin_rd <= 1; padding_cnt <= PADDING_UP; padding_done <= 0; fft_done <= 0;
        end else begin
            if (!fft_done && fft_saxi_valid && fft_saxi_rdy) begin
                if (fft_we_one) begin
                    up_in <= 1; fin_rd <= 0;
                end else if (fft_we_length_plus_one) begin
                    up_in <= 0; padding_cnt <= PADDING_DOWN; padding_done <= 0;
                end else if (!padding_done) begin
                    padding_cnt <= padding_cnt - 1; padding_done <= padding_cnt == 1;
                end
                fft_we_cnt <= fft_we_cnt - 1;
                fft_done <= fft_we_one;
                fft_we_one <= fft_we_cnt == 2;
                fft_we_length_plus_one <= fft_we_cnt == FFT_LENGTH_PLUS_TWO;
            end
        end
    end

    // ---- fft_done_o: model the CDC latency (2 FF) ----
    logic d1, d2;
    always @(posedge clk) begin d1 <= fft_done && up_in; d2 <= d1; fft_done_o <= d2; end

    // ---- behavioral xfft: rdy with backpressure bursts; count beats/frame ----
    integer bp = 0;
    logic [31:0] lfsr = 32'h1234_5678;
    always @(posedge clk) lfsr <= {lfsr[30:0], lfsr[31]^lfsr[21]^lfsr[1]^lfsr[0]};
    always @(posedge clk) begin
        if (bp > 0) bp <= bp - 1;
        else if (lfsr[7:0] < 40) bp <= (lfsr[11:8] + 1);   // random short stalls
    end
    assign fft_saxi_rdy = (bp == 0);

    // ---- monitors ----
    integer beats=0, dbeats=0, frame=0, anomalies=0;
    logic [ASZ-1:0] exp_lane0; logic have_prev=0;
    always @(posedge clk) if (rstn) begin
        if (fft_saxi_valid && fft_saxi_rdy) begin
            beats  <= beats + 1;
            if (padding_done) begin
                // data beat: lanes should be 4 consecutive counter values
                dbeats <= dbeats + 1;
                if (have_prev && fin_dout[ASZ-1:0] != exp_lane0) begin
                    $display("  [t=%0t] *** DATA SKIP frame %0d dbeat %0d: got %0d expected %0d (STOLEN BEAT)",
                             $time, frame, dbeats, fin_dout[ASZ-1:0], exp_lane0);
                    anomalies <= anomalies + 1;
                end
                exp_lane0 <= fin_dout[ASZ-1:0] + FSSR;  // next data beat lane0
                have_prev <= 1;
            end
            if (fft_saxi_last) begin
                if (beats+1 != FFT_LENGTH) begin
                    $display("  [t=%0t] *** FRAME LEN frame %0d: %0d beats (expected %0d)",
                             $time, frame, beats+1, FFT_LENGTH);
                    anomalies <= anomalies + 1;
                end
                frame <= frame + 1;
                beats <= 0;
                have_prev <= 0;          // contiguity resets across frame boundary / flush
            end
        end
    end

    // ---- write counter (total samples actually written to fin) ----
    integer wrcount=0;
    always @(posedge clk) if (rstn && fin_wr) wrcount <= wrcount + 1;

    // ---- hang detector ----
    integer idle=0, last_beats=-1, last_frame=-1;
    always @(posedge clk) if (rstn) begin
        if (frame!=last_frame || beats!=last_beats) begin idle<=0; last_frame<=frame; last_beats<=beats; end
        else idle <= idle + 1;
        if (idle > 5000) begin
            $display("  [t=%0t] *** HANG: frame=%0d beats=%0d fft_done=%0b fin_dvalid=%0b",
                     $time, frame, beats, fft_done, fin_dvalid);
            $display("      data_beats_consumed=%0d  samples_written=%0d  budget_left=%0d  overflow=%0b",
                     dbeats, wrcount, budget, fin_full);
            $display("      real_pops=%0d (vs consumed %0d -> %0d STOLEN)  flushes=%0d",
                     popcount, dbeats, popcount-dbeats, flushcount);
            $display("      expected data beats per acq = %0d (acq_up+acq_down)", ACQ_UP+ACQ_DOWN);
            $display("=== RESULT: frames=%0d anomalies=%0d (HUNG) ===", frame, anomalies);
            $finish;
        end
    end

    // ---- stimulus: reset, then a trigger per acquisition ----
    initial begin
        budget_load = 0; budget_val = 0;
        rstn = 0; repeat (30) @(posedge clk);
        rstn = 1; repeat (30) @(posedge clk);
        for (int t=0; t<NTRIG; t++) begin
            @(posedge clk); trig_i <= 1; @(posedge clk); trig_i <= 0;
            // after the flush, supply EXACTLY one acquisition's worth of samples
            repeat (3) @(posedge clk);
            budget_load <= 1; budget_val <= (ACQ_UP+ACQ_DOWN)*FSSR; @(posedge clk); budget_load <= 0;
            // wait for this acquisition (up+down) to finish (fft_done back to 1)
            wait (fft_done == 1'b1);
            repeat (40) @(posedge clk);
        end
        $display("=== RESULT: frames=%0d anomalies=%0d %s ===",
                 frame, anomalies, (anomalies==0)?"CLEAN":"*** ANOMALIES ***");
        $finish;
    end

    initial begin #4000000; $display("=== TIMEOUT frames=%0d anomalies=%0d ===", frame, anomalies); $finish; end
endmodule
