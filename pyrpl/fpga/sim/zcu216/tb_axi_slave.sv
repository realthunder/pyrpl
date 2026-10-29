`timescale 1ns/1ps
// axi_slave (the PS register bus) under back-to-back writes as the ZynqMP's
// core issues them to device memory: several in flight, each W beat sent as
// soon as the W channel is free, so it can arrive while the slave still waits
// for the previous write's ack. A peripheral model acks LAT cycles after
// sys_wen (red_pitaya_asg's tables: 2; hk: 1) and logs every write. Every
// write must reach the bus exactly once, at its own address with its own
// data, and get an OKAY response (a lost one times out: SLVERR, which the PS
// turns into an SError that stops the kernel).
module tb_axi_slave;
  logic clk = 0;  always #5 clk = ~clk;
  logic rstn = 0;

  logic [31:0] awaddr = 0, wdata = 0, araddr = 0;
  logic        awvalid = 0, wvalid = 0, bready = 1, arvalid = 0, rready = 1;
  logic        awready, wready, bvalid, arready, rvalid, rlast;
  logic [1:0]  bresp, rresp;
  logic [15:0] bid, rid;
  logic [31:0] rdata;
  logic [31:0] sys_addr, sys_wdata, sys_rdata = 0;
  logic [3:0]  sys_sel;
  logic        sys_wen, sys_ren, sys_ack, sys_err = 0;

  axi_slave #(.AXI_DW (32), .AXI_AW (32), .AXI_IW (16), .ACK_W (10)) dut (
    .axi_clk_i (clk), .axi_rstn_i (rstn),
    .axi_awid_i (16'h0), .axi_awaddr_i (awaddr), .axi_awlen_i (4'h0), .axi_awsize_i (3'b010),
    .axi_awburst_i (2'b01), .axi_awlock_i (2'b00), .axi_awcache_i (4'h0), .axi_awprot_i (3'h0),
    .axi_awvalid_i (awvalid), .axi_awready_o (awready),
    .axi_wid_i (16'h0), .axi_wdata_i (wdata), .axi_wstrb_i (4'hf), .axi_wlast_i (1'b1),
    .axi_wvalid_i (wvalid), .axi_wready_o (wready),
    .axi_bid_o (bid), .axi_bresp_o (bresp), .axi_bvalid_o (bvalid), .axi_bready_i (bready),
    .axi_arid_i (16'h0), .axi_araddr_i (araddr), .axi_arlen_i (4'h0), .axi_arsize_i (3'b010),
    .axi_arburst_i (2'b01), .axi_arlock_i (2'b00), .axi_arcache_i (4'h0), .axi_arprot_i (3'h0),
    .axi_arvalid_i (arvalid), .axi_arready_o (arready),
    .axi_rid_o (rid), .axi_rdata_o (rdata), .axi_rresp_o (rresp), .axi_rlast_o (rlast),
    .axi_rvalid_o (rvalid), .axi_rready_i (rready),
    .sys_addr_o (sys_addr), .sys_wdata_o (sys_wdata), .sys_sel_o (sys_sel),
    .sys_wen_o (sys_wen), .sys_ren_o (sys_ren), .sys_rdata_i (sys_rdata),
    .sys_err_i (sys_err), .sys_ack_i (sys_ack));

  // peripheral: ack LAT cycles after a write enable (or a read enable)
  int lat = 2;
  logic [7:0] pipe = 0;
  always @(posedge clk) begin
    pipe <= {pipe[6:0], sys_wen || sys_ren};
    sys_ack <= pipe[lat - 1];
  end
  logic [31:0] log_a [$], log_d [$];
  always @(posedge clk) if (sys_wen) begin log_a.push_back(sys_addr); log_d.push_back(sys_wdata); end

  // master: AW and W issued independently, up to 4 writes in flight
  localparam int N = 200;
  int aw_i = 0, w_i = 0, b_n = 0, b_err = 0;
  always @(posedge clk) if (rstn) begin
    if (awvalid && awready) aw_i <= aw_i + 1;
    if (wvalid && wready) w_i <= w_i + 1;
    if (bvalid && bready) begin b_n <= b_n + 1; if (bresp != 0) b_err <= b_err + 1; end
  end
  always @(negedge clk) if (rstn) begin
    awvalid <= aw_i < N && aw_i - b_n < 4 && ($urandom % 4 != 0 || awvalid);
    awaddr  <= 32'h0004_0000 + 4 * aw_i;
    wvalid  <= w_i < N && w_i - b_n < 4 && ($urandom % 4 != 0 || wvalid);
    wdata   <= 32'h1000 + w_i;
  end

  task automatic run(int l);
    int bad = 0;
    lat = l;
    log_a.delete(); log_d.delete();
    rstn = 0; aw_i = 0; w_i = 0; b_n = 0; b_err = 0;
    repeat (5) @(posedge clk);
    rstn = 1;
    fork
      wait (b_n == N);
      begin repeat (N * 1200) @(posedge clk); end
    join_any
    disable fork;
    repeat (20) @(posedge clk);
    if (log_a.size() != N) bad++;
    for (int i = 0; i < N && i < log_a.size(); i++)
      if (log_a[i] != 32'h0004_0000 + 4 * i || log_d[i] != 32'h1000 + i) begin
        if (bad < 4) $display("  write %0d: %h <= %h", i, log_a[i], log_d[i]);
        bad++;
      end
    $display("AXI ack latency %0d: %0d writes, %0d responses (%0d SLVERR), %0d bus writes, %0d bad",
             l, N, b_n, b_err, log_a.size(), bad);
    if (bad || b_err || b_n != N) $display("ERR AXI latency %0d", l);
  endtask

  initial begin
    run(1);
    run(2);
    run(4);
    $display("AXI done");
    $finish;
  end
endmodule
