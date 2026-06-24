// Determine xpm_fifo_async asymmetric (write ASZ, read ASZ*FSSR) lane order:
// is the FIRST-written sample in the LSB lane or the MSB lane of the read word?
`timescale 1ns/1ps
module tb_fifo_order;
  localparam int ASZ=14, FSSR=4, W=ASZ*FSSR;
  logic clk=0; always #4 clk=~clk;
  logic rst=1, wr_en=0, rd_en=0;
  logic [ASZ-1:0] din;
  logic [W-1:0]   dout;
  logic dvalid;

  xpm_fifo_async #(
    .FIFO_WRITE_DEPTH(64), .WRITE_DATA_WIDTH(ASZ), .READ_DATA_WIDTH(W),
    .FIFO_READ_LATENCY(0), .USE_ADV_FEATURES("1001"), .READ_MODE("fwft")
  ) dut (
    .rst(rst), .wr_clk(clk), .wr_en(wr_en), .din(din),
    .rd_clk(clk), .rd_en(rd_en), .dout(dout), .data_valid(dvalid),
    .overflow()
  );

  initial begin
    repeat(10) @(posedge clk);
    rst<=0; repeat(5) @(posedge clk);
    // write samples 1,2,3,4 (distinct, nonzero)
    for (int s=1;s<=FSSR;s++) begin
      din<=s; wr_en<=1; @(posedge clk);
    end
    wr_en<=0;
    // FWFT: data_valid goes high when a full read word is available; dout is valid then.
    repeat(2000) begin @(posedge clk); if (dvalid) break; end
    $display("read word lanes (lane0=LSB .. lane3=MSB):");
    for (int j=0;j<FSSR;j++)
      $display("  lane %0d = %0d", j, dout[j*ASZ +: ASZ]);
    $display("=> first-written sample (1) is in lane %0d", (dout[0 +: ASZ]==1)?0:(dout[(FSSR-1)*ASZ +: ASZ]==1)?FSSR-1:-1);
    $finish;
  end
  initial begin #50000; $display("TIMEOUT"); $finish; end
endmodule
