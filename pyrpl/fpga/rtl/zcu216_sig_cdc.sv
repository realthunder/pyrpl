/**
 * N 14-bit signals, one sample per system clock (sys_clk, 125 MHz), into fclk
 * (312.5 MHz) for the scope and the FFT (zcu216_dsp): a 16-entry ring in distributed RAM,
 * written every sys_clk, and a Gray-coded write pointer through two flops.
 * The reader takes the entry two behind the pointer it sees: written at least
 * one sys_clk (8 ns) before, and left alone for 13 more. No reset and no
 * flow control, so it needs neither clock running first (fclk only starts once
 * the CLK104 and the RF tiles are up). Each fclk cycle gives the newest sample
 * (repeated two or three times per sys_clk).
 */
module zcu216_sig_cdc #(
  parameter int N = 4
) (
  input  logic        sys_clk,
  input  logic [13:0] din  [N],
  input  logic        fclk,
  output logic [13:0] dout [N]
);

(* ram_style = "distributed" *) logic [14*N-1:0] ring [16];
logic [3:0] wp = '0, wg = '0;
logic [14*N-1:0] w;
always_comb
  for (int c = 0; c < N; c++) w[14*c +: 14] = din[c];
always_ff @(posedge sys_clk) begin
  ring[wp] <= w;
  wp <= wp + 1'b1;
  wg <= (wp + 1'b1) ^ ((wp + 1'b1) >> 1);
end

(* ASYNC_REG = "TRUE" *) logic [3:0] wg_m = '0, wg_s = '0;
logic [3:0]      rp = '0;
logic [14*N-1:0] q  = '0;
always_ff @(posedge fclk) begin
  wg_m <= wg;
  wg_s <= wg_m;
  // Gray to binary, then two entries back
  rp   <= {wg_s[3], ^wg_s[3:2], ^wg_s[3:1], ^wg_s[3:0]} - 4'd2;
  q    <= ring[rp];
end

for (genvar c = 0; c < N; c++) begin : g_out
  assign dout[c] = q[14*c +: 14];
end

endmodule
