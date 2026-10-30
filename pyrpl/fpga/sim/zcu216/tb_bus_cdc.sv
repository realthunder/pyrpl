`timescale 1ns/1ps
// zcu216_bus_cdc: the system bus from pl_clk0 (100 MHz) to the system clock
// (125 MHz) with a timeout. A module model acks LAT cycles after the access
// and reads back a function of the address; writes are logged. Checked:
// every access with both clocks running gets its own data and every write
// reaches the bus exactly once; with the back clock stopped, every access is
// answered within the timeout with 0 (no error); the back clock stopped in
// the middle of an access; once the clock is back, accesses work again (at
// most one more lost); a module that never acks reads 0.
module tb_bus_cdc;
  localparam int TO = 256;
  logic clk_a = 0;  always #5 clk_a = ~clk_a;
  logic clk_b_run = 1, clk_b = 0;
  always #4 if (clk_b_run) clk_b = ~clk_b;
  logic rstn_a = 0, rstn_b = 0;

  logic [31:0] a_addr = 0, a_wdata = 0, a_rdata;
  logic        a_wen = 0, a_ren = 0, a_err, a_ack;
  logic [31:0] b_addr, b_wdata, b_rdata;
  logic [3:0]  b_sel;
  logic        b_wen, b_ren, b_err, b_ack;

  zcu216_bus_cdc #(.TIMEOUT (TO)) dut (.clk_a, .rstn_a, .a_addr, .a_wdata, .a_sel (4'hF),
    .a_wen, .a_ren, .a_rdata, .a_err, .a_ack, .clk_b, .rstn_b, .b_addr, .b_wdata,
    .b_sel, .b_wen, .b_ren, .b_rdata, .b_err, .b_ack);

  function automatic logic [31:0] rd_val(logic [31:0] a);
    return a ^ 32'hA5A5_0000;
  endfunction

  // module model: ack LAT cycles after the access; address 0xDEAD0 never acks
  int lat = 1;
  int nwrites [logic [31:0]];
  logic [31:0] wval [logic [31:0]];
  logic [3:0] pend = 0;
  always @(posedge clk_b) begin
    b_ack <= 1'b0;  b_err <= 1'b0;
    if (b_wen || b_ren) begin
      if (b_wen) begin
        nwrites[b_addr] = nwrites.exists(b_addr) ? nwrites[b_addr] + 1 : 1;
        wval[b_addr] = b_wdata;
      end
      if (b_addr != 32'hDEAD0) pend <= 4'(lat);
    end else if (pend != 0) begin
      pend <= pend - 1'b1;
      if (pend == 1) begin b_ack <= 1'b1;  b_rdata <= rd_val(b_addr); end
    end
  end

  int errs = 0;

  // one access as axi_slave makes it: a one-cycle wen / ren, then wait for ack;
  // returns the clk_a cycles it took
  task automatic access(input logic we, input logic [31:0] addr, input logic [31:0] data,
                        output logic [31:0] rdata, output logic err, output int cycles);
    @(negedge clk_a);
    a_addr = addr;  a_wdata = data;  a_wen = we;  a_ren = !we;
    @(negedge clk_a);
    a_wen = 0;  a_ren = 0;
    cycles = 1;
    while (!a_ack) begin @(negedge clk_a); cycles++; end
    rdata = a_rdata;  err = a_err;
    repeat ($urandom % 3) @(negedge clk_a);
  endtask

  initial begin
    logic [31:0] r;  logic e;  int c, maxc, lost;
    repeat (5) @(negedge clk_a);  rstn_a = 1;  rstn_b = 1;
    repeat (5) @(negedge clk_a);

    // 1. both clocks: 300 accesses, module latency 1..4
    maxc = 0;
    for (int i = 0; i < 300; i++) begin
      logic [31:0] ad = 32'h100 + 4 * ($urandom % 64);
      lat = 1 + $urandom % 4;
      if ($urandom % 2) begin
        access(1, ad, $urandom, r, e, c);
      end else begin
        access(0, ad, 0, r, e, c);
        if (r !== rd_val(ad) || e) begin errs++; if (errs < 5) $display("ERR read %h: %h", ad, r); end
      end
      if (c > maxc) maxc = c;
    end
    begin
      int n = 0, dup = 0;
      foreach (nwrites[k]) begin n += nwrites[k]; end
      $display("CDC running: 300 accesses, longest %0d clk_a cycles, %0d bus writes", maxc, n);
    end

    // writes exactly once: 50 writes to fresh addresses
    for (int i = 0; i < 50; i++) access(1, 32'h10000 + 4 * i, 32'hC0DE_0000 + i, r, e, c);
    for (int i = 0; i < 50; i++)
      if (nwrites[32'h10000 + 4 * i] != 1 || wval[32'h10000 + 4 * i] != 32'hC0DE_0000 + i) begin
        errs++;  if (errs < 5) $display("ERR write %0d: %0d times", i, nwrites[32'h10000 + 4 * i]);
      end

    // 2. module that never acks: back timeout, reads 0
    access(0, 32'hDEAD0, 0, r, e, c);
    $display("CDC no module ack: read %h after %0d cycles", r, c);
    if (r !== 0 || e) errs++;
    access(0, 32'h104, 0, r, e, c);
    if (r !== rd_val(32'h104)) begin errs++; $display("ERR after no-ack: %h", r); end

    // 3. back clock stopped: every access answered within the timeout, with 0
    clk_b_run = 0;
    maxc = 0;
    for (int i = 0; i < 20; i++) begin
      access(i % 2, 32'h200 + 4 * i, 32'hBAD0_0000 + i, r, e, c);
      if (c > maxc) maxc = c;
      if (r !== 0 || e) begin errs++; $display("ERR clock stopped: %h %b", r, e); end
    end
    $display("CDC clock stopped: 20 accesses, longest %0d cycles (timeout %0d)", maxc, TO);
    if (maxc > TO + 4) errs++;

    // 4. clock back: accesses work again, at most one more lost
    clk_b_run = 1;
    lost = 0;
    for (int i = 0; i < 40; i++) begin
      access(0, 32'h300 + 4 * i, 0, r, e, c);
      if (r !== rd_val(32'h300 + 4 * i)) lost++;
      if (r !== rd_val(32'h300 + 4 * i) && r !== 0) begin errs++; $display("ERR wrong data %h", r); end
    end
    $display("CDC clock back: %0d of 40 reads lost", lost);
    if (lost > 1) errs++;

    // 5. clock stopped in the middle of accesses, at random points, then back
    lost = 0;
    for (int k = 0; k < 30; k++) begin
      fork
        begin
          lat = 4;
          access(0, 32'h400 + 4 * k, 0, r, e, c);
          if (r !== rd_val(32'h400 + 4 * k) && r !== 0) begin errs++; $display("ERR wrong data %h", r); end
        end
        begin
          repeat ($urandom % 12) @(posedge clk_a);
          clk_b_run = 0;
          repeat (50 + $urandom % 400) @(posedge clk_a);
          clk_b_run = 1;
        end
      join
      repeat (10) @(negedge clk_a);
      access(0, 32'h500 + 4 * k, 0, r, e, c);
      if (r !== rd_val(32'h500 + 4 * k) && r !== 0) begin errs++; $display("ERR wrong data %h", r); end
      if (r !== rd_val(32'h500 + 4 * k)) begin
        // allowed once right after an outage; the next must work
        lost++;
        access(0, 32'h500 + 4 * k, 0, r, e, c);
        if (r !== rd_val(32'h500 + 4 * k)) begin errs++; $display("ERR stuck after outage %0d: %h", k, r); end
      end
    end
    $display("CDC outages in mid-access: 30, reads lost right after: %0d", lost);

    $display("CDC done, %0d errors", errs);
    $finish;
  end
endmodule
