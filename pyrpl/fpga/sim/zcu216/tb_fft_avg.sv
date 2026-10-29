`timescale 1ns/1ps
// zcu216_fft_avg against a model, bit for bit: random 32-bit magnitudes (some
// near full scale), frames back to back and with gaps, m = 0, 1, 3, 8 and a
// restart in mid-group. Every output frame must be isqrt(sum of mag^2 over its
// group of 2^m frames >> m) per bin, in natural order with its marks, and no
// other frame may come out.
module tb_fft_avg;
  localparam int SUB = 4, NB = 1 << SUB, NBIN = 4 * NB;
  logic clk = 0;  always #1.6 clk = ~clk;
  logic rstn = 0;

  logic [3:0]     m = 0;
  logic           restart = 0;
  logic [31:0]    r_mag [4];
  logic [SUB-1:0] r_t;
  logic           r_valid = 0, r_first = 0, r_last = 0;
  logic [31:0]    n_mag [4];
  logic [SUB-1:0] n_t;
  logic           n_valid, n_first, n_last;

  zcu216_fft_avg #(.SUB(SUB)) dut (.clk, .rstn, .m, .restart, .r_mag, .r_t, .r_valid,
    .r_first, .r_last, .n_mag, .n_t, .n_valid, .n_first, .n_last);

  function automatic logic [31:0] isqrt(logic [63:0] x);
    logic [63:0] r = 0;
    for (int b = 31; b >= 0; b--) begin
      logic [63:0] t = r | (64'(1) << b);
      if (t * t <= x) r = t;
    end
    return 32'(r);
  endfunction

  // model: groups of 2^m frames from the restart; expected output frames queued
  logic [71:0] acc [NBIN];
  int          gi = -1;                            // frame index in the group, -1: none yet
  int          mm = 0;
  logic [31:0] expq [$];                           // expected n_mag words, in order
  int          frames_in = 0, frames_exp = 0;

  task automatic send_frame(int gap);
    logic [31:0] f [NBIN];
    for (int b = 0; b < NBIN; b++)
      case ($urandom % 4)
        0: f[b] = $urandom;                        // anything
        1: f[b] = 32'hFFFF_FFFF - ($urandom % 16); // near full scale
        2: f[b] = $urandom % 1000;                 // small
        default: f[b] = $urandom >> ($urandom % 32);
      endcase
    // model
    if (gi < 0 || gi >= (1 << mm) - 1) gi = 0; else gi++;
    for (int b = 0; b < NBIN; b++) begin
      logic [71:0] p = 72'(64'(f[b]) * 64'(f[b]));
      acc[b] = (gi == 0) ? p : acc[b] + p;
    end
    if (gi == (1 << mm) - 1) begin
      for (int b = 0; b < NBIN; b++) expq.push_back(isqrt(64'(acc[b] >> mm)));
      frames_exp++;
    end
    frames_in++;
    // the frame
    for (int t = 0; t < NB; t++) begin
      @(negedge clk);
      for (int j = 0; j < 4; j++) r_mag[j] = f[4 * t + j];
      r_t = SUB'(t);  r_valid = 1;  r_first = (t == 0);  r_last = (t == NB - 1);
    end
    @(negedge clk) begin r_valid = 0; r_first = 0; r_last = 0; end
    repeat (gap) @(negedge clk);
  endtask

  // checker
  int errs = 0, outs = 0, ot = 0, frames_out = 0;
  always @(posedge clk) if (n_valid) begin
    if (n_t != SUB'(ot)) begin if (errs < 5) $display("ERR t %0d, expected %0d", n_t, ot); errs++; end
    if (n_first != (ot == 0) || n_last != (ot == NB - 1)) begin
      if (errs < 5) $display("ERR marks at t %0d", ot); errs++;
    end
    for (int j = 0; j < 4; j++) begin
      logic [31:0] e;
      if (expq.size() == 0) begin if (errs < 5) $display("ERR output with nothing expected"); errs++; end
      else begin
        e = expq.pop_front();
        if (n_mag[j] !== e) begin
          if (errs < 5) $display("ERR frame %0d bin %0d: %0d, expected %0d", frames_out, 4 * ot + j, n_mag[j], e);
          errs++;
        end
      end
      outs++;
    end
    ot = (ot + 1) % NB;
    if (n_last) frames_out++;
  end

  task automatic run(int mv, int nframes, int gap, string name);
    int f0 = frames_out, e0 = frames_exp;
    m = 4'(mv);  mm = mv;  gi = -1;                 // a new m restarts (as the RTL)
    repeat (4) @(negedge clk);
    for (int i = 0; i < nframes; i++) send_frame(gap);
    repeat (60) @(negedge clk);
    $display("AVG %s: m %0d, %0d frames in, %0d out (%0d expected)", name, mv, nframes,
             frames_out - f0, frames_exp - e0);
    if (frames_out - f0 != frames_exp - e0) errs++;
  endtask

  initial begin
    for (int j = 0; j < 4; j++) r_mag[j] = 0;
    r_t = 0;
    repeat (5) @(negedge clk);  rstn = 1;
    repeat (5) @(negedge clk);
    run(0, 5, 0, "back to back");
    run(1, 8, 3, "gaps");
    run(3, 16, 0, "back to back");
    // restart in mid-group: 3 frames, restart, then two whole groups
    m = 3;  mm = 3;  repeat (4) @(negedge clk);
    for (int i = 0; i < 3; i++) send_frame(2);
    restart = !restart;  gi = -1;
    repeat (4) @(negedge clk);
    for (int i = 0; i < 16; i++) send_frame(1);
    repeat (60) @(negedge clk);
    $display("AVG restart in mid-group: %0d frames out so far, %0d expected", frames_out, frames_exp);
    run(8, 256, 0, "M 256");
    if (expq.size() != 0) begin $display("ERR %0d expected words never came out", expq.size()); errs++; end
    $display("AVG done: %0d words checked, %0d errors", outs, errs);
    $finish;
  end
endmodule
