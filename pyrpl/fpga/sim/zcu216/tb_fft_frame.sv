`timescale 1ns/1ps
// zcu216_fft_frame against a model of zcu216_fft_dit8's input side: the engine
// counts ACCEPTED beats into frames of NB, accepts while ready (which drops for
// one clock inside its first frame, as an xfft core applying its config), and
// the data is the ADC beat number (s_data registered as in zcu216_fft_test).
// Checked: free-running = every beat; the switch to framing on a frame
// boundary; each framed frame = NB accepted beats in a row, its first ACQ the
// consecutive ADC beats from sync + OFF (+ the fixed pipeline), the rest zeros;
// nothing between frames; a frame due while the engine is busy is skipped and
// counted; back to free-running on a boundary.
module tb_fft_frame;
  localparam int SUB = 5, NB = 1 << SUB, CW = 21;
  logic clk = 0;  always #1.6 clk = ~clk;
  logic rstn = 0;

  logic          en = 0, sync = 0;
  logic [CW-1:0] off = 10, acq = 20;
  logic          valid_d, zero_d, framed, skip;

  // ADC: beat number, the source register as in zcu216_fft_test
  int unsigned adc = 0;
  logic [31:0] s_data;
  logic        s_valid;
  always @(posedge clk) begin
    adc     <= adc + 1;
    s_data  <= (zero_d || !valid_d) ? 32'h0 : 32'(adc) | 32'h8000_0000;
    s_valid <= valid_d;
  end

  // engine: active after a while, one ready drop inside its first frame
  logic active = 0, ready = 1, accept;
  int   beat = 0, eng_frames = 0, drop_at = -1;
  assign accept = active && s_valid && ready;
  always @(posedge clk) begin
    ready <= !(active && eng_frames == 0 && beat == 7 && drop_at < 0);
    if (active && eng_frames == 0 && beat == 7 && drop_at < 0) drop_at = 1;
    if (accept) begin
      beat <= (beat + 1) % NB;
      if (beat == NB - 1) eng_frames <= eng_frames + 1;
    end
  end

  zcu216_fft_frame #(.SUB(SUB), .CW(CW)) dut (.clk, .rstn, .en, .sync, .off, .acq,
    .acc (accept), .valid_d, .zero_d, .framed, .skip);

  // record every accepted beat: (time, engine beat index, data)
  int errs = 0, skips = 0;
  always @(posedge clk) if (skip) skips++;

  // framed-frame checker: frames start on beat 0, data then zeros, contiguous
  int f_cnt = 0, f_k = -1, f_first_adc = -1, f_gap = 0, frames_ok = 0;
  logic [31:0] f_exp;
  int unsigned sync_adc [$];
  always @(posedge clk) if (framed || f_k >= 0) begin
    if (accept) begin
      if (beat == 0) begin
        if (f_k > 0 && f_k != NB) begin $display("ERR frame of %0d beats", f_k); errs++; end
        f_k = 0;
      end
      if (f_k < 0) begin $display("ERR beat %0d accepted outside a frame", beat); errs++; end
      else begin
        if (f_k < int'(acq)) begin
          if (!s_data[31]) begin $display("ERR frame beat %0d: zero, data expected", f_k); errs++; end
          else if (f_k > 0 && s_data[30:0] != f_exp[30:0] + 1) begin
            $display("ERR frame beat %0d: adc %0d after %0d", f_k, s_data[30:0], f_exp[30:0]); errs++;
          end
          if (f_k == 0) f_first_adc = s_data[30:0];
          f_exp = s_data;
        end else if (s_data != 0) begin
          $display("ERR frame beat %0d: data %h, zero expected", f_k, s_data); errs++;
        end
        f_k++;
        if (f_k == NB) begin frames_ok++; f_k = -1; end
      end
    end
  end

  task automatic pulse_sync();
    @(negedge clk) sync = 1;  @(negedge clk) sync = 0;
  endtask

  initial begin
    repeat (5) @(posedge clk);  rstn = 1;
    repeat (3) @(posedge clk);  active = 1;
    // free-running: 5 frames, every beat
    repeat (5 * NB + 3) @(posedge clk);
    $display("FRAME free run: engine frames %0d, ready dropped once: %0d", eng_frames, drop_at == 1);
    if (eng_frames < 4) begin $display("ERR free run"); errs++; end
    // framing on: switches at the next boundary
    en = 1;
    repeat (2 * NB) @(posedge clk);
    if (!framed) begin $display("ERR not framed"); errs++; end
    if (beat != 0) begin $display("ERR switched off a boundary (beat %0d)", beat); errs++; end
    // syncs every 3 NB clocks: one frame each, first data beat = sync + off + pipeline
    begin
      int lat = -1;
      for (int i = 0; i < 6; i++) begin
        int unsigned t0;
        pulse_sync();
        t0 = adc;
        repeat (3 * NB - 2) @(posedge clk);
        if (f_first_adc >= 0) begin
          int l = int'(f_first_adc) - int'(t0) - int'(off);
          if (lat < 0) lat = l;
          else if (l != lat) begin $display("ERR frame start latency %0d, was %0d", l, lat); errs++; end
        end
      end
      $display("FRAME framed: %0d whole frames, sync -> first data = off + %0d beats", frames_ok, lat);
      if (frames_ok != 6) begin $display("ERR frames %0d of 6", frames_ok); errs++; end
    end
    // syncs closer than a frame: every other one is skipped
    begin
      int f0 = frames_ok, s0 = skips;
      for (int i = 0; i < 6; i++) begin
        pulse_sync();
        repeat (NB / 2) @(posedge clk);
      end
      repeat (3 * NB) @(posedge clk);
      $display("FRAME syncs every NB/2: %0d frames, %0d skipped", frames_ok - f0, skips - s0);
      if (frames_ok - f0 + skips - s0 != 6 || skips == s0) begin $display("ERR skip"); errs++; end
    end
    // acq longer than a frame: clamped (all data, no zeros)
    acq = 3 * NB;
    pulse_sync();
    repeat (3 * NB) @(posedge clk);
    acq = 20;
    // framing off: free-running again, from a boundary
    en = 0;
    repeat (3) @(posedge clk);
    if (framed) begin $display("ERR still framed"); errs++; end
    begin
      int e0 = eng_frames;
      repeat (4 * NB) @(posedge clk);
      $display("FRAME free again: %0d frames in 4 NB clocks", eng_frames - e0);
      if (eng_frames - e0 < 3) begin $display("ERR free again"); errs++; end
    end
    $display("FRAME done, %0d errors", errs);
    $finish;
  end
endmodule
