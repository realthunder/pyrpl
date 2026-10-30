`timescale 1ns/1ps
// zcu216_dsp, the signal hub: output_direct masks at the Red Pitaya DSP's
// addresses, off after reset, saturated sums; the sinks' selects read back as
// signal numbers and come out as the fclk muxes' indices.
module tb_dsp;
  logic clk = 0;  always #4 clk = ~clk;
  logic rstn = 0;
  logic signed [13:0] asg [4];
  logic [13:0] out [4];
  logic [3:0]  sc_sel [2], fft_sel [2];
  logic [31:0] addr = 0, wdata = 0, rdata;
  logic        wen = 0, ren = 0, err, ack;
  zcu216_dsp dut (.clk, .rstn, .asg, .out, .sc_sel, .fft_sel, .sys_addr (addr), .sys_wdata (wdata),
    .sys_wen (wen), .sys_ren (ren), .sys_rdata (rdata), .sys_err (err), .sys_ack (ack));
  int errs = 0;
  task automatic wr(int n, int r, int v);
    @(negedge clk) addr = 32'h0060_0000 + n * 32'h10000 + r;  wdata = v;  wen = 1;
    @(negedge clk) wen = 0;
  endtask
  task automatic rd(int n, int r, output logic [31:0] v);
    @(negedge clk) addr = 32'h0060_0000 + n * 32'h10000 + r;  ren = 1;
    @(negedge clk) ren = 0;  v = rdata;
  endtask
  function automatic int sat(int s);  return s > 8191 ? 8191 : s < -8192 ? -8192 : s;  endfunction
  task automatic chk(int k, int e);
    if ($signed(out[k]) !== e) begin errs++; $display("ERR out%0d = %0d, expected %0d", k + 1, $signed(out[k]), e); end
  endtask
  task automatic sel(int n, int r, logic [3:0] got, logic [3:0] e);
    if (got !== e) begin errs++; $display("ERR sink %0d: index %0d, expected %0d", n, got, e); end
  endtask
  initial begin
    logic [31:0] v;
    asg = '{14'sd1000, -14'sd3000, 14'sd8000, 14'sd500};
    repeat (3) @(negedge clk);  rstn = 1;
    repeat (3) @(negedge clk);
    for (int k = 0; k < 4; k++) chk(k, 0);                  // all off after reset
    sel(8, 0, sc_sel[0], 0);  sel(9, 0, sc_sel[1], 1);      // adc0, adc1
    sel(28, 0, fft_sel[0], 0);  sel(29, 0, fft_sel[1], 0);  // tone
    wr(8, 4, 1);  wr(9, 4, 3);  wr(10, 4, 12);  wr(11, 4, 15);
    repeat (2) @(negedge clk);
    chk(0, 1000 - 3000 + 500);  chk(1, -3000 + 500);  chk(2, sat(8500));  chk(3, sat(8500));
    asg[2] = -14'sd8000;  asg[3] = -14'sd1000;  repeat (2) @(negedge clk);
    chk(3, sat(-9000));
    rd(10, 4, v);  if (v !== 12) begin errs++; $display("ERR output_direct readback %0d", v); end
    wr(5, 4, 15);  repeat (2) @(negedge clk);                // not an ASG: ignored
    chk(0, 1000 - 3000 - 1000);
    // every signal number into the scope and the FFT: {number, scope, FFT}
    begin
      int t [][3] = '{'{8,8,5}, '{9,9,6}, '{10,10,7}, '{11,11,8}, '{12,0,1}, '{13,1,2},
                      '{14,12,9}, '{15,13,10}, '{21,6,0}, '{22,2,3}, '{23,3,4},
                      '{24,14,11}, '{25,15,12}, '{26,4,0}, '{27,5,0}, '{30,6,0}, '{0,6,0}};
      foreach (t[i]) begin
        wr(8, 0, t[i][0]);  wr(9, 0, t[i][0]);  wr(28, 0, t[i][0]);  wr(29, 0, t[i][0]);
        repeat (2) @(negedge clk);
        sel(8, 0, sc_sel[0], t[i][1]);  sel(9, 0, sc_sel[1], t[i][1]);
        sel(28, 0, fft_sel[0], t[i][2]);  sel(29, 0, fft_sel[1], t[i][2]);
        rd(9, 0, v);   if (v !== t[i][0]) begin errs++; $display("ERR scope readback %0d", v); end
        rd(29, 0, v);  if (v !== t[i][0]) begin errs++; $display("ERR FFT readback %0d", v); end
      end
    end
    rd(20, 0, v);  if (v !== 0) begin errs++; $display("ERR unused address %0d", v); end
    $display("DSP done, %0d errors", errs);
    $finish;
  end
endmodule
