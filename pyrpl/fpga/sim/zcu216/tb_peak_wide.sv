`timescale 1ns/1ps
// zcu216_peak_wide against its Python model (hardware_modules/zcu216_wide_peak.py):
// frames from peak_wide_gen.py streamed as zcu216_fft_reorder delivers them,
// mostly back to back, each frame's settings applied at its first beat. Writes one
// result word per frame for run_peak_wide.sh to compare.
module tb_peak_wide;
  localparam int SUB = `PW_SUB, N = 4 << SUB;
  logic clk = 0; always #1.6 clk = ~clk;
  logic rstn = 0;
  logic [31:0] n_mag [4];
  logic [SUB-1:0] n_t = '0;
  logic n_valid = 0, n_first = 0, n_last = 0;
  logic [31:0] cfg [8];
  logic [31:0] m [N];
  logic r_valid;
  logic [63:0] r_word;

  zcu216_peak_wide #(.SUB(SUB)) dut (
    .clk, .rstn, .n_mag, .n_t, .n_valid, .n_first, .n_last,
    .cell_log (cfg[0][3:0]), .peak_log (cfg[1][2:0]), .guard (cfg[2][5:0]),
    .train_log (cfg[3][2:0]), .thr (cfg[4][15:0]), .start_index (cfg[5][SUB+2:0]),
    .end_index (cfg[6][SUB+2:0]), .data_min (cfg[7]),
    .r_valid, .r_word);

  int fi, fo, frames = 0, outs = 0;
  always @(posedge clk)
    if (r_valid) begin
      $fdisplay(fo, "%016h", r_word);
      outs++;
    end

  initial begin
    fi = $fopen(`PW_IN, "r");
    fo = $fopen(`PW_OUT, "w");
    repeat (5) @(posedge clk); rstn <= 1; repeat (3) @(posedge clk);
    while (!$feof(fi)) begin
      if ($fscanf(fi, "%h", cfg[0]) != 1) break;
      for (int i = 1; i < 8; i++) void'($fscanf(fi, "%h", cfg[i]));
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
      else if (frames % 3 == 1) ;                 // back to back
    end
    repeat (200) @(posedge clk);
    $display("PW frames %0d results %0d", frames, outs);
    $fclose(fo);
    $finish;
  end
endmodule
