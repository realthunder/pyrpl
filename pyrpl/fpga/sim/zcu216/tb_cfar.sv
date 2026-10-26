`timescale 1ns/1ps
// zcu216_cfar against the HLS detector: frames from cfar_gen.py streamed as
// zcu216_fft_reorder delivers them (four bins per clock, mostly back to back),
// with each frame's settings applied at its first beat. Writes one result word
// per frame to cfar_rtl.hex for run_cfar.sh to compare with cfar_ref.
module tb_cfar;
  localparam int SUB = `CFAR_SUB, N = 4 << SUB;
  logic clk = 0; always #1.6 clk = ~clk;
  logic rstn = 0;
  logic [31:0] n_mag [4];
  logic [SUB-1:0] n_t = '0;
  logic n_valid = 0, n_first = 0, n_last = 0;
  logic [31:0] cfg [10];
  logic [31:0] m [N];
  logic r_valid, r_overrun;
  logic [63:0] r_word;

  zcu216_cfar #(.SUB(SUB)) dut (
    .clk, .rstn, .n_mag, .n_t, .n_valid, .n_first, .n_last,
    .k_sq (cfg[0][15:0]), .start_index (cfg[1][SUB+2:0]), .end_index (cfg[2][SUB+2:0]),
    .data_min (cfg[3]), .guard (cfg[4][4:0]), .train (cfg[5][6:0]),
    .onesided (cfg[6][0]), .so_mode (cfg[7][0]), .ramp_d0 (cfg[8][22:0]),
    .ramp_step (cfg[9][22:0]),
    .r_valid, .r_word, .r_overrun);

  int fi, fo, frames = 0, outs = 0, overruns = 0;
  initial begin
    fi = $fopen(`CFAR_IN, "r");
    fo = $fopen(`CFAR_OUT, "w");
    repeat (5) @(posedge clk); rstn <= 1; repeat (3) @(posedge clk);
    while (!$feof(fi)) begin
      if ($fscanf(fi, "%h", cfg[0]) != 1) break;
      for (int i = 1; i < 10; i++) void'($fscanf(fi, "%h", cfg[i]));
      for (int b = 0; b < N; b++) void'($fscanf(fi, "%h", m[b]));
      frames++;
      for (int t = 0; t < N / 4; t++) begin
        for (int j = 0; j < 4; j++) n_mag[j] <= m[4*t + j];
        n_t <= SUB'(t);
        n_valid <= 1; n_first <= (t == 0); n_last <= (t == N/4 - 1);
        @(posedge clk);
      end
      n_valid <= 0; n_first <= 0; n_last <= 0;
      if (frames % 3 == 0) repeat ($urandom_range(1, 40)) @(posedge clk);
    end
    repeat (2000) @(posedge clk);
    $display("CFAR frames=%0d outs=%0d overruns=%0d", frames, outs, overruns);
    $fclose(fo);
    $finish;
  end
  always @(posedge clk) begin
    if (r_valid) begin
      $fwrite(fo, "%016h\n", r_word);
      outs++;
    end
    if (r_overrun) overruns++;
  end
endmodule
