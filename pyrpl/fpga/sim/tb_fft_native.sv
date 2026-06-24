// Ground-truth ordering test for the native-SSR xfft BD (FFT_IMPL=4).
// Feeds a known clean cosine at bin TONE_K as FSSR real samples/beat and reports
// where the magnitude peak(s) land in (beat, lane) — revealing the core's actual
// input->output SSR ordering. Run two ways via -d INPUT_REVERSE to compare the
// consecutive vs SSR-lane-reversed input packing.
`timescale 1ns/1ps

module tb_fft_native;
    localparam int ASZ   = 14;
    localparam int FSSR  = 4;
    localparam int NFFT  = 11;
    localparam int N     = 1 << NFFT;       // 2048
    localparam int BEATS = N / FSSR;        // 512
    localparam int DSZ   = 20;
    localparam int INW   = FSSR*ASZ;        // 56
    localparam int OUTW  = FSSR*DSZ;        // 80
`ifndef TONE_K
    `define TONE_K 32
`endif
    localparam int K     = `TONE_K;
    localparam real AMP  = 4000.0;          // well below 2^13, no clip
    localparam real PI   = 3.14159265358979;

    logic clk = 0, aresetn = 0;
    always #4 clk = ~clk;                    // 125 MHz

    logic [INW-1:0]  s_tdata;
    logic            s_tvalid, s_tready, s_tlast;
    logic [OUTW-1:0] m_tdata;
    logic            m_tvalid, m_tlast;
    logic            m_tready = 1'b1;
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

    // diagnostics
    integer fed=0, cap=0; logic seen_ready=0, seen_mvalid=0, seen_frame=0;
    always @(posedge clk) if (aresetn) begin
        if (s_tvalid && s_tready) fed <= fed+1;
        if (m_tvalid && m_tready) cap <= cap+1;
        if (s_tready && !seen_ready) begin seen_ready<=1; $display("  [t=%0t] s_tready first high", $time); end
        if (m_tvalid && !seen_mvalid) begin seen_mvalid<=1; $display("  [t=%0t] m_tvalid first high", $time); end
        if (event_frame_started && !seen_frame) begin seen_frame<=1; $display("  [t=%0t] frame_started", $time); end
    end

    // sample buffer
    logic signed [ASZ-1:0] x [0:N-1];
    integer n, b, s;
    real ph;

    function automatic logic [INW-1:0] pack_beat(input int beat);
        logic [INW-1:0] w;
        for (int j=0; j<FSSR; j++) begin
            int samp_idx;
`ifdef INPUT_REVERSE
            samp_idx = beat*FSSR + (FSSR-1-j); // SSR-lane-reversed packing
`else
            samp_idx = beat*FSSR + j;          // consecutive (lane j = LSB-first)
`endif
            w[j*ASZ +: ASZ] = x[samp_idx];
        end
        return w;
    endfunction

    // capture
    logic [DSZ-1:0] mag [0:N-1];             // mag[beat*FSSR+lane]
    integer outbeat = 0;
    integer i;

    initial begin
        // build cosine
        for (n=0; n<N; n++) begin
            ph = 2.0*PI*K*n/N;
            x[n] = $rtoi($cos(ph)*AMP);
        end
        for (i=0;i<N;i++) mag[i]=0;
        s_tvalid=0; s_tlast=0; s_tdata=0;
        repeat (20) @(posedge clk);
        aresetn = 1;
        repeat (20) @(posedge clk);

        // ---- feed one frame of data ----
        fork
            begin : FEED
                for (b=0; b<BEATS; b++) begin
                    s_tdata  <= pack_beat(b);
                    s_tvalid <= 1'b1;
                    s_tlast  <= (b==BEATS-1);
                    @(posedge clk);
                    while (!s_tready) @(posedge clk);
                end
                s_tvalid <= 1'b0; s_tlast <= 1'b0;
            end
            begin : DRAIN
                integer guard;
                guard = 0;
                while (outbeat < BEATS && guard < 200000) begin
                    @(posedge clk);
                    if (m_tvalid && m_tready) begin
                        for (s=0; s<FSSR; s++)
                            mag[outbeat*FSSR + s] = m_tdata[s*DSZ +: DSZ];
                        outbeat = outbeat + 1;
                    end
                    guard = guard + 1;
                end
            end
        join

        // ---- report ----
        $display("=== tb_fft_native: TONE_K=%0d  INPUT_REVERSE=%0d  captured %0d/%0d beats ===",
                 K,
`ifdef INPUT_REVERSE
                 1,
`else
                 0,
`endif
                 outbeat, BEATS);
        // top 8 bins by magnitude (bin = beat*FSSR + lane, natural assumption)
        for (int t=0; t<8; t++) begin
            integer bestbin; logic [DSZ-1:0] bestv;
            bestbin = -1; bestv = 0;
            for (i=0;i<N;i++) if (mag[i] > bestv) begin bestv=mag[i]; bestbin=i; end
            if (bestbin<0) break;
            $display("  peak#%0d  natbin=%0d  (beat=%0d lane=%0d)  mag=%0d",
                     t, bestbin, bestbin/FSSR, bestbin%FSSR, bestv);
            mag[bestbin] = 0; // remove for next
        end
        $finish;
    end

    initial begin
        #300000;
        $display("TIMEOUT: fed=%0d captured=%0d outbeat=%0d", fed, cap, outbeat);
        // dump whatever was captured
        for (int tt=0; tt<8; tt++) begin
            integer bb; logic [DSZ-1:0] vv; bb=-1; vv=0;
            for (int ii=0;ii<N;ii++) if (mag[ii]>vv) begin vv=mag[ii]; bb=ii; end
            if (bb<0) break;
            $display("  peak#%0d natbin=%0d (beat=%0d lane=%0d) mag=%0d", tt, bb, bb/FSSR, bb%FSSR, vv);
            mag[bb]=0;
        end
        $finish;
    end
endmodule
