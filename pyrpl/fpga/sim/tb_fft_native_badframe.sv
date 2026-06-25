// Stall-injection test for the native-SSR xfft BD (FFT_IMPL=4).
// Feeds NFRAMES frames of a clean tone at bin TONE_K, and injects:
//   - a mid-frame OUTPUT backpressure stall (deassert m_tready) during one frame
//   - a mid-frame INPUT gap (deassert s_tvalid) during another frame
// Reports the peak bin of EVERY output frame. If the native xfft keeps every
// output frame's peak at TONE_K, its framing is robust to backpressure/gaps and
// the stuck/shift bug is in the fft_proc glue. If a post-stall frame's peak
// rotates (e.g. +64 bins) or output stops, the xfft core loses frame alignment.
`timescale 1ns/1ps

module tb_fft_native_badframe;
    localparam int ASZ   = 14;
    localparam int FSSR  = 4;
    localparam int NFFT  = 11;
    localparam int N     = 1 << NFFT;       // 2048
    localparam int BEATS = N / FSSR;        // 512
    localparam int DSZ   = 20;
    localparam int INW   = FSSR*ASZ;        // 56
    localparam int OUTW  = FSSR*DSZ;        // 80
    localparam int NFRAMES = 8;
    localparam int K     = 32;              // tone bin -> peak expected at bin 32
    localparam real AMP  = 4000.0;
    localparam real PI   = 3.14159265358979;

    // injection knobs
    localparam int OUT_STALL_FRAME = 2;     // stall m_tready during this output frame
    localparam int OUT_STALL_BEAT  = 64;    // ...starting at this output beat
    localparam int OUT_STALL_CYC   = 0;   // ...for this many cycles
    localparam int IN_GAP_FRAME    = 4;     // deassert s_tvalid during this input frame
    localparam int IN_GAP_BEAT     = 70;
    localparam int BADF            = 3;   // frame with tlast one beat early    // ...starting at this input beat
    localparam int IN_GAP_CYC      = 0;   // ...for this many cycles

    logic clk = 0, aresetn = 0;
    always #4 clk = ~clk;                    // 125 MHz

    logic [INW-1:0]  s_tdata;
    logic            s_tvalid, s_tready, s_tlast;
    logic [OUTW-1:0] m_tdata;
    logic            m_tvalid, m_tlast;
    logic            m_tready;
    logic            event_frame_started;

    fft_ssr_native_bd_wrapper dut (
        .aclk(clk), .aresetn(aresetn),
        .s_axis_tdata(s_tdata), .s_axis_tvalid(s_tvalid),
        .s_axis_tready(s_tready), .s_axis_tlast(s_tlast),
        .s_axis_tkeep('1), .s_axis_tstrb('1),
        .m_axis_tdata(m_tdata), .m_axis_tvalid(m_tvalid),
        .m_axis_tready(m_tready), .m_axis_tlast(m_tlast),
        .m_axis_tkeep(), .m_axis_tstrb(),
        .event_frame_started(event_frame_started)
    );

    // ---- output capture: peak bin per output frame ----
    integer oframe = 0, ob = 0;
    logic [DSZ-1:0] fmax=0; integer fmaxbin=0;
    integer ev_count = 0; logic ev_d = 0;

    // ---- output backpressure stall control ----
    integer stall_cnt = 0; logic did_stall = 0;
    assign m_tready = (stall_cnt == 0);

    always @(posedge clk) if (aresetn) begin
        // event pulse count
        ev_d <= event_frame_started;
        if (event_frame_started && !ev_d) ev_count <= ev_count + 1;

        // stall trigger / countdown (mid output frame OUT_STALL_FRAME)
        if (stall_cnt > 0)
            stall_cnt <= stall_cnt - 1;
        else if (!did_stall && oframe == OUT_STALL_FRAME && ob == OUT_STALL_BEAT && m_tvalid) begin
            stall_cnt <= OUT_STALL_CYC;
            did_stall <= 1;
            $display("  [t=%0t] >>> injecting OUTPUT stall (m_tready low %0d cyc) mid frame %0d beat %0d",
                     $time, OUT_STALL_CYC, oframe, ob);
        end

        // capture output beats (only when accepted)
        if (m_tvalid && m_tready) begin
            for (int l=0; l<FSSR; l++) begin
                logic [DSZ-1:0] mg; integer bin;
                mg  = m_tdata[l*DSZ +: DSZ];
                bin = ob*FSSR + l;
                if (mg > fmax) begin fmax = mg; fmaxbin = bin; end
            end
            ob <= ob + 1;
            if (m_tlast) begin
                $display("  [t=%0t] OUTPUT frame %0d: peak bin=%0d (expect %0d) mag=%0d  %s",
                         $time, oframe, fmaxbin, K, fmax, (fmaxbin==K||fmaxbin==N-K)?"OK":"*** SHIFTED ***");
                oframe <= oframe + 1;
                fmax = 0; fmaxbin = 0; ob <= 0;
            end
        end
    end

    // ---- stimulus ----
    logic signed [ASZ-1:0] x [0:N-1];
    integer n, b, f;
    real ph;
    function automatic logic [INW-1:0] pack_beat(input int beat);
        logic [INW-1:0] w;
        for (int j=0; j<FSSR; j++) w[j*ASZ +: ASZ] = x[beat*FSSR + j];
        return w;
    endfunction

    initial begin
        for (n=0; n<N; n++) begin ph = 2.0*PI*K*n/N; x[n] = $rtoi($cos(ph)*AMP); end
        s_tvalid=0; s_tlast=0; s_tdata=0;
        repeat (20) @(posedge clk);
        aresetn = 1;
        repeat (20) @(posedge clk);

        for (f=0; f<NFRAMES; f++) begin
            for (b=0; b<BEATS; b++) begin
                // inject input gap mid-frame
                if (f == IN_GAP_FRAME && b == IN_GAP_BEAT) begin
                    s_tvalid <= 1'b0;
                    $display("  [t=%0t] >>> injecting INPUT gap (s_tvalid low %0d cyc) mid frame %0d beat %0d",
                             $time, IN_GAP_CYC, f, b);
                    repeat (IN_GAP_CYC) @(posedge clk);
                end
                s_tdata  <= pack_beat(b);
                s_tvalid <= 1'b1;
                s_tlast  <= (b == ((f==BADF) ? BEATS-2 : BEATS-1));
                if (f==BADF && b==BEATS-2) $display("  [t=%0t] >>> frame %0d: tlast ONE BEAT EARLY (beat %0d)",$time,f,b);
                @(posedge clk);
                while (!s_tready) @(posedge clk);
            end
        end
        s_tvalid <= 1'b0; s_tlast <= 1'b0;

        repeat (6*BEATS + 4000) @(posedge clk);
        $display("=== DONE: output frames captured=%0d, event pulses=%0d ===", oframe, ev_count);
        $finish;
    end

    initial begin
        #6000000;
        $display("TIMEOUT: oframe=%0d ev=%0d (xfft likely DEADLOCKED after stall)", oframe, ev_count);
        $finish;
    end
endmodule
