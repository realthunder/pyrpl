`timescale 1ns/1ps
// zcu216_fft_reorder alone: bins tagged with their own index (plus 1000 x frame), fed
// in bit-reversed k order; every output lane must carry bin 4t + j.
module tb_reorder;
  localparam int SUB = 5, B = 1 << SUB;
  logic clk = 0; always #1.6 clk = ~clk;
  logic rstn = 0;
  logic [31:0] m_mag [4], n_mag [4];
  logic [SUB-1:0] m_k = '0, n_t;
  logic m_valid = 0, m_last = 0, n_valid, n_first, n_last;
  zcu216_fft_reorder #(.SUB(SUB)) dut (.clk, .rstn, .m_mag, .m_k, .m_valid, .m_last,
    .n_mag, .n_t, .n_valid, .n_first, .n_last);
  function automatic logic [SUB-1:0] br(input logic [SUB-1:0] v);
    for (int i = 0; i < SUB; i++) br[i] = v[SUB-1-i];
  endfunction
  int errs = 0, outs = 0;
  initial begin
    repeat (5) @(posedge clk); rstn <= 1; repeat (3) @(posedge clk);
    for (int f = 0; f < 3; f++)
      for (int t = 0; t < B; t++) begin
        m_k <= br(SUB'(t));
        for (int q = 0; q < 4; q++) m_mag[q] <= 1000*f + br(SUB'(t)) + q*B;
        m_valid <= 1; m_last <= (t == B-1);
        @(posedge clk);
      end
    m_valid <= 0; m_last <= 0;
    repeat (3*B) @(posedge clk);
    $display("RO  outs=%0d errs=%0d", outs, errs);
    $finish;
  end
  always @(posedge clk) if (n_valid) begin
    for (int j = 0; j < 4; j++) begin
      outs++;
      if (n_mag[j] % 1000 != 4*n_t + j) begin
        errs++;
        if (errs < 12) $display("t=%0d j=%0d got bin %0d (frame %0d) want %0d", n_t, j, n_mag[j] % 1000, n_mag[j]/1000, 4*n_t+j);
      end
    end
  end
endmodule
