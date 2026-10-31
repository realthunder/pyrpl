/**
 * Two registers each way, and on the reset, between the system bus and a module
 * on it (zcu216_top). At 250 MHz the wires from zcu216_bus_cdc across the chip to
 * the modules' register banks, and their reset, do not fit in a clock. Accesses
 * wait for the ack, so the bus just takes four clocks more.
 */
module zcu216_bus_pipe (
  input  logic        clk,
  input  logic        rstn_i,
  output logic        rstn_o,
  input  logic [31:0] addr_i,
  input  logic [31:0] wdata_i,
  input  logic [3:0]  sel_i,
  input  logic        wen_i,
  input  logic        ren_i,
  output logic [31:0] rdata_o,
  output logic        err_o,
  output logic        ack_o,
  output logic [31:0] addr_o,
  output logic [31:0] wdata_o,
  output logic [3:0]  sel_o,
  output logic        wen_o,
  output logic        ren_o,
  input  logic [31:0] rdata_i,
  input  logic        err_i,
  input  logic        ack_i
);

// no shift registers: the point is two flops the placer can pull apart
(* shreg_extract = "no" *) logic [31:0] a1, d1, r1;
(* shreg_extract = "no" *) logic [3:0]  s1;
(* shreg_extract = "no" *) logic        w1, n1, x1, e1, k1;

always_ff @(posedge clk) begin
  a1 <= addr_i;   addr_o  <= a1;
  d1 <= wdata_i;  wdata_o <= d1;
  s1 <= sel_i;    sel_o   <= s1;
  w1 <= wen_i;    wen_o   <= w1;
  n1 <= ren_i;    ren_o   <= n1;
  x1 <= rstn_i;   rstn_o  <= x1;
  r1 <= rdata_i;  rdata_o <= r1;
  e1 <= err_i;    err_o   <= e1;
  k1 <= ack_i;    ack_o   <= k1;
end

endmodule
