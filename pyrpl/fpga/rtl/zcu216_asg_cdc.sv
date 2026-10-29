/**
 * The ASG's four 14-bit outputs, one sample per bus clock (sys_clk, 100 MHz),
 * into fclk (312.5 MHz) for the scope: a 16-entry ring in distributed RAM,
 * written every bus cycle, and a Gray-coded write pointer through two flops.
 * The reader takes the entry two behind the pointer it sees: written at least
 * one bus cycle (10 ns) before, and left alone for 13 more. No reset and no
 * flow control, so it needs neither clock running first (fclk only starts once
 * the CLK104 and the RF tiles are up). Each fclk cycle gives the newest sample
 * (repeated about three times per bus cycle).
 */
module zcu216_asg_cdc (
  input  logic        sys_clk,
  input  logic [13:0] din  [4],
  input  logic        fclk,
  output logic [13:0] dout [4]
);

(* ram_style = "distributed" *) logic [55:0] ring [16];
logic [3:0] wp = '0, wg = '0;
always_ff @(posedge sys_clk) begin
  ring[wp] <= {din[3], din[2], din[1], din[0]};
  wp <= wp + 1'b1;
  wg <= (wp + 1'b1) ^ ((wp + 1'b1) >> 1);
end

(* ASYNC_REG = "TRUE" *) logic [3:0] wg_m = '0, wg_s = '0;
logic [3:0]  rp = '0;
logic [55:0] q  = '0;
always_ff @(posedge fclk) begin
  wg_m <= wg;
  wg_s <= wg_m;
  // Gray to binary, then two entries back
  rp   <= {wg_s[3], ^wg_s[3:2], ^wg_s[3:1], ^wg_s[3:0]} - 4'd2;
  q    <= ring[rp];
end

for (genvar c = 0; c < 4; c++) begin : g_out
  assign dout[c] = q[14*c +: 14];
end

endmodule
