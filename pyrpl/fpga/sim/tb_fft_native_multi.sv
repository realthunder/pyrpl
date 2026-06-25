// Multi-frame test: does the native-SSR xfft pulse event_frame_started ONCE per
// frame, or only once after reset? Feeds NFRAMES frames back-to-back (continuous
// s_tvalid, s_tlast at each frame boundary) and counts event_frame_started rising
// edges vs output-frame (m_tlast) pulses. If ev_count == NFRAMES the event is
// per-frame and usable for frame counting; if ev_count == 1 it is not.
`timescale 1ns/1ps

module tb_fft_native_multi;
    localparam int ASZ   = 14;
    localparam int FSSR  = 4;
    localparam int NFFT  = 11;
    localparam int N     = 1 << NFFT;       // 2048
    localparam int BEATS = N / FSSR;        // 512
    localparam int DSZ   = 20;
    localparam int INW   = FSSR*ASZ;        // 56
    localparam int OUTW  = FSSR*DSZ;        // 80
    localparam int NFRAMES = 4;
    localparam int K     = 32;
    localparam real AMP  = 4000.0;
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

    // edge-count event_frame_started and m_tlast
    integer ev_count = 0, out_frames = 0, in_beats = 0;
    logic ev_d = 0;
    always @(posedge clk) if (aresetn) begin
        ev_d <= event_frame_started;
        if (event_frame_started && !ev_d) begin
            ev_count <= ev_count + 1;
            $display("  [t=%0t] event_frame_started pulse #%0d", $time, ev_count+1);
        end
        if (s_tvalid && s_tready) in_beats <= in_beats + 1;
        if (m_tvalid && m_tready && m_tlast) begin
            out_frames <= out_frames + 1;
            $display("  [t=%0t] output frame (m_tlast) #%0d", $time, out_frames+1);
        end
    end

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

        // feed NFRAMES frames back-to-back (continuous valid)
        for (f=0; f<NFRAMES; f++) begin
            for (b=0; b<BEATS; b++) begin
                s_tdata  <= pack_beat(b);
                s_tvalid <= 1'b1;
                s_tlast  <= (b==BEATS-1);
                @(posedge clk);
                while (!s_tready) @(posedge clk);
            end
        end
        s_tvalid <= 1'b0; s_tlast <= 1'b0;

        // let the pipeline drain
        repeat (4*BEATS + 2000) @(posedge clk);

        $display("=== RESULT: fed %0d frames | event_frame_started pulses=%0d | output frames=%0d ===",
                 NFRAMES, ev_count, out_frames);
        if (ev_count == NFRAMES)
            $display("=== VERDICT: event pulses PER FRAME (usable for frame_cnt) ===");
        else if (ev_count <= 1)
            $display("=== VERDICT: event pulses ONLY ONCE -> NOT usable per-frame ===");
        else
            $display("=== VERDICT: event pulses %0d times for %0d frames (partial) ===", ev_count, NFRAMES);
        $finish;
    end

    initial begin
        #2000000;
        $display("TIMEOUT: ev_count=%0d out_frames=%0d in_beats=%0d", ev_count, out_frames, in_beats);
        $finish;
    end
endmodule
