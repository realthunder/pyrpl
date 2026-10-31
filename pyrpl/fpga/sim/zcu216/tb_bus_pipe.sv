`timescale 1ns/1ps
// zcu216_bus_pipe: two slots behind zcu216_bus_cdc, decoded as in zcu216_top, the
// modules answering as the Red Pitaya's do (a one-clock ack, the data with it, on
// reads and writes) and the second one a clock later. Checked: reads return the
// addressed slot's data, written values land, no ack of one access is taken for the
// next, in back-to-back accesses that alternate slots.
module tb_bus_pipe;
  logic clk_a = 0, clk_b = 0, rstn = 0;
  always #5 clk_a = ~clk_a;
  always #2 clk_b = ~clk_b;

  logic [31:0] a_addr, a_wdata, a_rdata;
  logic        a_wen = 0, a_ren = 0, a_err, a_ack;
  logic [31:0] bk_addr, bk_wdata, bk_rdata;
  logic [3:0]  bk_sel;
  logic        bk_wen, bk_ren, bk_err, bk_ack;

  zcu216_bus_cdc cdc (.clk_a, .rstn_a (rstn), .a_addr, .a_wdata, .a_sel (4'hF), .a_wen, .a_ren,
    .a_rdata, .a_err, .a_ack, .clk_b, .rstn_b (rstn), .b_addr (bk_addr), .b_wdata (bk_wdata),
    .b_sel (bk_sel), .b_wen (bk_wen), .b_ren (bk_ren), .b_rdata (bk_rdata), .b_err (bk_err),
    .b_ack (bk_ack));

  logic [1:0]  cs;
  logic [31:0] addr [2], wdata [2], rdata_o [2], rdata_i [2];
  logic [1:0]  wen, ren, ack_o, ack_i, err_o;
  logic [31:0] mem [2][16];
  assign cs = 2'b01 << bk_addr[20];
  for (genvar i = 0; i < 2; i++) begin : g
    zcu216_bus_pipe p (.clk (clk_b), .rstn_i (rstn), .rstn_o (), .addr_i (bk_addr),
      .wdata_i (bk_wdata), .sel_i (bk_sel), .wen_i (bk_wen && cs[i]), .ren_i (bk_ren && cs[i]),
      .rdata_o (rdata_o[i]), .err_o (err_o[i]), .ack_o (ack_o[i]), .addr_o (addr[i]),
      .wdata_o (wdata[i]), .sel_o (), .wen_o (wen[i]), .ren_o (ren[i]), .rdata_i (rdata_i[i]),
      .err_i (1'b0), .ack_i (ack_i[i]));
    logic w_q = 0, r_q = 0;
    always_ff @(posedge clk_b) begin
      if (wen[i]) mem[i][addr[i][5:2]] <= wdata[i];
      if (i == 0) begin
        ack_i[i]   <= wen[i] | ren[i];
        rdata_i[i] <= mem[i][addr[i][5:2]];
      end else begin
        w_q <= wen[i];  r_q <= ren[i];
        ack_i[i]   <= w_q | r_q;
        rdata_i[i] <= mem[i][addr[i][5:2]];
      end
    end
  end
  assign bk_rdata = rdata_o[bk_addr[20]];
  assign bk_ack   = |(cs & ack_o);
  assign bk_err   = |(cs & err_o);

  task automatic access(input logic we, input logic [31:0] ad, input logic [31:0] d, output logic [31:0] q);
    @(negedge clk_a) a_addr = ad;  a_wdata = d;  a_wen = we;  a_ren = !we;
    @(negedge clk_a) a_wen = 0;  a_ren = 0;
    while (!a_ack) @(posedge clk_a);
    q = a_rdata;
  endtask

  int errs = 0, n = 0;
  initial begin
    logic [31:0] q, want [2][16];
    for (int i = 0; i < 2; i++) for (int j = 0; j < 16; j++) begin mem[i][j] = '0; want[i][j] = '0; end
    repeat (10) @(posedge clk_a);
    rstn = 1;
    repeat (10) @(posedge clk_a);
    for (int k = 0; k < 2000; k++) begin
      int s = $urandom_range(1), r = $urandom_range(15);
      logic [31:0] ad = (32'(s) << 20) | (32'(r) << 2);
      if ($urandom_range(1)) begin
        logic [31:0] d = $urandom;
        access(1'b1, ad, d, q);
        want[s][r] = d;
      end else begin
        access(1'b0, ad, 0, q);
        if (q !== want[s][r]) begin errs++; $display("ERR read %0d %0d: %h, want %h", s, r, q, want[s][r]); end
      end
      n++;
    end
    $display("PIPE %0d accesses", n);
    $display("PIPE done, %0d errors", errs);
    $finish;
  end
endmodule
