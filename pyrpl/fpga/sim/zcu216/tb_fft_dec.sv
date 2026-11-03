`timescale 1ns/1ps
// zcu216_fft_dec against a model of zcu216_fft_dit8's input side (as
// tb_fft_frame): the engine counts ACCEPTED beats into frames of NW, accepts
// while ready (which drops for one clock inside its first frame), and the
// caller registers data_d / valid_d. The source is a known function of the
// sample number, so each capture's means are checked exactly: the window starts
// on the beat that comes with `start`, word b holds the means of samples
// s0 + 8bD .. s0 + 8(b+1)D - 1, D at a time, rounded half up.
// Checked, for D = 2 .. 2^KMAX: framed windows (sync + off), then free-running
// back-to-back frames; every frame NW accepted beats, the capture's words then
// zeros, nothing between frames; windows due while capturing and captures
// complete while playing are skipped, and counted.
module tb_fft_dec;
  localparam int SUB = 5, NW = 1 << SUB, CW = 21, KMAX = 8;
  logic clk = 0;  always #1.6 clk = ~clk;
  logic rstn = 0;

  logic [3:0]    k = 1;
  logic          en = 1, sync = 0, en_play = 1;
  logic [CW-1:0] off = 10;
  logic [23:0]   acq = 0;
  logic [127:0]  din, data_d, s_data;
  logic          valid_d, bnd, skip, s_valid;

  // the source: sample n is a fixed 14-bit pseudo-random value
  function automatic int v(longint n);
    longint h = (n * 64'd2654435761) & 64'hffff_ffff;
    return int'((h >> 16) & 14'h3fff) - 8192;
  endfunction
  longint beat_n = 0;
  always_comb for (int j = 0; j < 8; j++) din[16*j +: 16] = 16'(v(8 * beat_n + j));
  always @(posedge clk) beat_n <= beat_n + 1;

  // the caller's registers, the engine
  logic active = 0, ready = 1, accept;
  int   beat = 0, eng_frames = 0, drop_done = 0;
  assign accept = active && s_valid && ready;
  always @(posedge clk) begin
    s_data  <= data_d;
    s_valid <= valid_d;
    ready   <= !(active && eng_frames == 0 && beat == 5 && !drop_done);
    if (active && eng_frames == 0 && beat == 5) drop_done = 1;
    if (accept) begin
      beat <= (beat + 1) % NW;
      if (beat == NW - 1) eng_frames <= eng_frames + 1;
    end
  end

  zcu216_fft_dec #(.SUB(SUB), .CW(CW), .KMAX(KMAX)) dut (.clk, .rstn, .k, .en, .sync,
    .off, .acq, .din, .en_play, .acc (accept), .data_d, .valid_d, .bnd, .skip);

  // each capture's first sample; the capture each frame plays
  longint cap_s0, play_s0 [$];
  int     play_words [$];
  always @(posedge clk) begin
    if (dut.start) cap_s0 = 8 * beat_n;
    if (dut.pstart) begin
      play_s0.push_back(cap_s0);
      play_words.push_back(int'(dut.words));
    end
  end

  // the expected word b of a capture from s0 at D = 2^k
  function automatic logic [127:0] expect_word(longint s0, int b, int kk);
    logic [127:0] w;
    int d = 1 << kk;
    for (int j = 0; j < 8; j++) begin
      longint sum = 0;
      for (int i = 0; i < d; i++) sum += v(s0 + longint'(8 * b + j) * d + i);
      w[16*j +: 16] = 16'((sum + (d / 2)) >>> kk);
    end
    return w;
  endfunction

  // frames: NW accepted beats from engine beat 0, the words then zeros
  int errs = 0, skips = 0, frames_ok = 0, f_k = -1, f_words = 0, checked = 0;
  longint f_s0;
  always @(posedge clk) if (skip) skips++;
  always @(posedge clk) if (accept) begin
    if (beat == 0) begin
      if (f_k >= 0) begin $display("ERR frame of %0d beats", f_k); errs++; end
      f_k = 0;
      if (play_s0.size() == 0) begin $display("ERR a frame with no capture"); errs++; end
      else begin
        f_s0    = play_s0.pop_front();
        f_words = play_words.pop_front();
      end
    end
    if (f_k < 0) begin $display("ERR beat accepted outside a frame"); errs++; end
    else begin
      // the first engine frame lost a beat to the ready drop: not checked
      if (eng_frames > 0) begin
        logic [127:0] e = (f_k < f_words) ? expect_word(f_s0, f_k, int'(k)) : '0;
        if (s_data !== e) begin
          if (errs < 10) $display("ERR D %0d frame beat %0d of %0d words: %h, expected %h",
                                  1 << k, f_k, f_words, s_data, e);
          errs++;
        end
        checked++;
      end
      f_k++;
      if (f_k == NW) begin frames_ok++; f_k = -1; end
    end
  end

  task automatic pulse_sync();
    @(negedge clk) sync = 1;  @(negedge clk) sync = 0;
  endtask

  initial begin
    repeat (5) @(posedge clk);  rstn = 1;
    repeat (3) @(posedge clk);  active = 1;
    // k = 0 (the full rate, the framer's: no playback) with syncs, as in
    // zcu216_fft_test before the span is chosen; then D = 2, the up window
    // 5 beats after each sync: a frame per sync, nothing skipped
    begin
      int f0, s0;
      @(negedge clk) k = 0;  en_play = 0;  off = 5;  acq = 160;
      for (int i = 0; i < 20; i++) begin pulse_sync(); repeat (98) @(posedge clk); end
      $display("DEC k 0: %0d skipped", skips);
      if (skips != 0) begin $display("ERR k 0 skips"); errs++; end
      @(negedge clk) k = 1;  en_play = 1;
      f0 = frames_ok;  s0 = skips;
      for (int i = 0; i < 20; i++) begin pulse_sync(); repeat (98) @(posedge clk); end
      $display("DEC k 0 -> 1: %0d frames for 20 syncs, %0d skipped", frames_ok - f0, skips - s0);
      if (frames_ok - f0 < 19 || skips != s0) begin $display("ERR k 0 -> 1"); errs++; end
      off = 10;
    end
    // framed, every D: windows of 20 words (then a short one, 3 words), syncs
    // far enough apart for a capture and a frame
    for (int kk = 1; kk <= KMAX; kk++) begin
      int f0 = frames_ok, s0 = skips, per = (NW << kk) + 3 * NW + 40;
      @(negedge clk) k = 4'(kk);
      acq = 24'(20 * 8 << kk);
      repeat (2 * NW) @(posedge clk);                 // the old D's frame, if any
      for (int i = 0; i < 4; i++) begin
        if (i == 3) acq = 24'(3 * 8 << kk) + 24'((8 << kk) - 1);   // not whole: 3 words
        pulse_sync();
        repeat (per) @(posedge clk);
      end
      $display("DEC framed D %3d: %0d frames, %0d skipped", 1 << kk, frames_ok - f0, skips - s0);
      if (frames_ok - f0 < 3 || skips != s0) begin $display("ERR framed D %0d", 1 << kk); errs++; end
    end
    // windows due while capturing: skipped
    begin
      int f0 = frames_ok, s0 = skips;
      @(negedge clk) k = 4'd3;
      acq = 24'(20 * 64);
      repeat (4 * NW) @(posedge clk);
      for (int i = 0; i < 6; i++) begin
        pulse_sync();
        repeat (60) @(posedge clk);                   // a capture is 160 beats
      end
      repeat (8 * NW) @(posedge clk);
      $display("DEC syncs during captures: %0d frames, %0d skipped", frames_ok - f0, skips - s0);
      if (skips == s0) begin $display("ERR no skip"); errs++; end
    end
    // short captures complete while a frame plays: skipped
    begin
      int f0 = frames_ok, s0 = skips;
      acq = 24'(2 * 64);                              // 2 words: 16 beats
      for (int i = 0; i < 6; i++) begin
        pulse_sync();
        repeat (NW / 2 + 14) @(posedge clk);
      end
      repeat (8 * NW) @(posedge clk);
      $display("DEC captures during frames: %0d frames, %0d skipped", frames_ok - f0, skips - s0);
      if (skips == s0) begin $display("ERR no skip"); errs++; end
    end
    // free-running: whole frames back to back
    begin
      int f0 = frames_ok;
      @(negedge clk) en = 0;  k = 4'd2;
      repeat (12 * NW * 4) @(posedge clk);
      $display("DEC free-running D 4: %0d frames", frames_ok - f0);
      if (frames_ok - f0 < 8) begin $display("ERR free-running"); errs++; end
    end
    // playback off: nothing starts, and bnd holds once the frame is out
    begin
      int f0;
      @(negedge clk) en_play = 0;
      repeat (3 * NW) @(posedge clk);
      f0 = frames_ok;
      if (!bnd) begin $display("ERR not at a boundary"); errs++; end
      repeat (8 * NW) @(posedge clk);
      if (frames_ok != f0 || !bnd) begin $display("ERR played while off"); errs++; end
    end
    $display("DEC done: %0d frames, %0d beats checked, %0d errors", frames_ok, checked, errs);
    $finish;
  end
endmodule
