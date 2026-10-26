`timescale 1ns/1ps
// zcu216_axil_bridge against an AXI4-Lite memory that answers after 0..40 cycles
// (address 0x3fffc answers SLVERR, which the bridge must NOT pass on as sys_err):
// writes, read-back.
module tb_bridge;
  logic clk = 0;  always #5 clk = ~clk;
  logic rstn = 0;
  logic [31:0] sys_addr = 0, sys_wdata = 0, sys_rdata;
  logic sys_wen = 0, sys_ren = 0, sys_err, sys_ack;
  logic [17:0] awaddr, araddr;  logic [31:0] wdata, rdata;  logic [3:0] wstrb;
  logic [1:0] bresp, rresp;
  logic awvalid, awready, wvalid, wready, bvalid, bready, arvalid, arready, rvalid, rready;

  zcu216_axil_bridge #(.AW(18)) dut (.clk, .rstn, .sys_addr, .sys_wdata, .sys_wen, .sys_ren,
    .sys_rdata, .sys_err, .sys_ack,
    .m_awaddr(awaddr), .m_awvalid(awvalid), .m_awready(awready), .m_wdata(wdata), .m_wstrb(wstrb),
    .m_wvalid(wvalid), .m_wready(wready), .m_bresp(bresp), .m_bvalid(bvalid), .m_bready(bready),
    .m_araddr(araddr), .m_arvalid(arvalid), .m_arready(arready), .m_rdata(rdata), .m_rresp(rresp),
    .m_rvalid(rvalid), .m_rready(rready));

  // slave: accepts AW and W independently, answers B after a random delay
  logic [31:0] mem [0:255];
  logic        got_aw = 0, got_w = 0;
  logic [17:0] wa;  logic [31:0] wd;
  int          bdly = -1, rdly = -1;
  logic [17:0] ra;
  always_ff @(posedge clk) begin
    awready <= $urandom_range(0, 1);  wready <= $urandom_range(0, 1);  arready <= $urandom_range(0, 1);
    if (awvalid && awready && !got_aw) begin got_aw <= 1; wa <= awaddr; end
    if (wvalid && wready && !got_w)    begin got_w <= 1;  wd <= wdata;  end
    bvalid <= 0;  rvalid <= 0;
    if (got_aw && got_w && bdly < 0) bdly <= $urandom_range(0, 40);
    if (bdly == 0) begin
      mem[wa[9:2]] <= wd;  bresp <= (wa == 18'h3fffc) ? 2'b10 : 2'b00;
      bvalid <= 1;  got_aw <= 0;  got_w <= 0;  bdly <= -1;
    end else if (bdly > 0) bdly <= bdly - 1;
    if (arvalid && arready && rdly < 0) begin ra <= araddr; rdly <= $urandom_range(0, 40); end
    if (rdly == 0) begin
      rdata <= mem[ra[9:2]];  rresp <= (ra == 18'h3fffc) ? 2'b10 : 2'b00;
      rvalid <= 1;  rdly <= -1;
    end else if (rdly > 0) rdly <= rdly - 1;
  end

  task automatic wr(input logic [31:0] a, d, output logic e);
    @(negedge clk); sys_addr = a; sys_wdata = d; sys_wen = 1;
    @(negedge clk); sys_wen = 0;
    while (!sys_ack) @(negedge clk);
    e = sys_err;
  endtask
  task automatic rd(input logic [31:0] a, output logic [31:0] d, output logic e);
    @(negedge clk); sys_addr = a; sys_ren = 1;
    @(negedge clk); sys_ren = 0;
    while (!sys_ack) @(negedge clk);
    d = sys_rdata;  e = sys_err;
  endtask

  int errs = 0;
  initial begin
    logic e;  logic [31:0] d;
    repeat (5) @(posedge clk);  rstn = 1;
    for (int i = 0; i < 64; i++) begin wr(32'h40400000 + 4*i, 32'hc0de0000 + i*7, e); if (e) errs++; end
    for (int i = 0; i < 64; i++) begin rd(32'h40400000 + 4*i, d, e); if (e || d != 32'hc0de0000 + i*7) errs++; end
    wr(32'h4043fffc, 1, e);  if (e) errs++;
    rd(32'h4043fffc, d, e);  if (e) errs++;
    $display("BRIDGE errs=%0d", errs);
    $finish;
  end
endmodule
