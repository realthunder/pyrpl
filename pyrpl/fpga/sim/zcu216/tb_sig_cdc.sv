`timescale 1ns/1ps
// zcu216_sig_cdc: a counter on sys_clk (250 MHz), checked in the eight lanes per fclk
// (312.5 MHz), from every start phase of the clocks: each value fills exactly 10
// lanes, values follow one by one, none lost, none repeated. After the clocks
// stop and return the reader realigns.
module tb_sig_cdc;
  logic tick = 0, run = 1;
  always #0.2 if (run) tick = ~tick;
  logic fclk = 0, sys_clk = 0;
  int fc = 0, sc = 0;
  always @(posedge tick or negedge tick) begin
    if (fc == 3) fc <= 0; else fc <= fc + 1;
    if (fc == 3) fclk <= ~fclk;
    if (sc == 4) sc <= 0; else sc <= sc + 1;
    if (sc == 4) sys_clk <= ~sys_clk;
  end

  logic [13:0]  din [4];
  logic [127:0] dout [4];
  logic [13:0]  cnt = 0;
  always @(posedge sys_clk) begin
    cnt <= cnt + 1'b1;
    for (int c = 0; c < 4; c++) din[c] <= cnt + 14'(c * 100);
  end
  zcu216_sig_cdc #(.N (4)) dut (.sys_clk, .din, .fclk, .dout);

  int errs = 0, runs = 0, lanes = 0, last = -1, len = 0;
  logic on = 0;
  always @(posedge fclk) if (on) begin
    for (int j = 0; j < 8; j++) begin
      int v = int'(dout[0][16*j +: 14]);
      lanes++;
      if (int'(dout[1][16*j +: 14]) != ((v + 100) & 16383)) begin errs++; $display("ERR ch1"); end
      if (last < 0) last = v;
      if (v == last) len++;
      else begin
        if (len != 10 && runs > 0) begin errs++; if (errs < 8) $display("ERR %0d held %0d lanes", last, len); end
        if (v != ((last + 1) & 16383)) begin errs++; if (errs < 8) $display("ERR %0d after %0d", v, last); end
        runs++;  last = v;  len = 1;
      end
    end
  end

  initial begin
    repeat (40) @(posedge sys_clk);
    on = 1;
    repeat (4000) @(posedge fclk);
    on = 0;
    $display("SIG %0d lanes, %0d samples of 10, %0d errors", lanes, runs, errs);
    for (int k = 0; k < 41; k++) begin
      repeat (30 + k) @(posedge tick);
      run = 0;  #200;  run = 1;
      repeat (40) @(posedge sys_clk);
      last = -1;  len = 0;  runs = 0;
      on = 1;  repeat (300) @(posedge fclk);  on = 0;
    end
    $display("SIG done, %0d errors", errs);
    $finish;
  end
endmodule
