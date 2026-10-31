/**
 * N 14-bit signals, one sample per system clock (sys_clk, 250 MHz), into fclk
 * (312.5 MHz) for the scope and the FFT (zcu216_dsp), as eight 16-bit lanes per
 * fclk, 2.5 GS/s like the RF-ADC's: each sample fills exactly 10 consecutive lanes,
 * four samples in 40 lanes, 5 fclk (s is the sample, phases p):
 *
 *   p      0         1         2         3         4
 *   lanes  aaaaaaaa  aabbbbbb  bbbbcccc  ccccccdd  dddddddd
 *
 * Whole fclk cycles of one sample would not do: a sample would be held for one or
 * two of them, an uneven hold that folds a signal at 62.5 MHz offsets (-10 dB for
 * 80 MHz, measured). Now it is the plain hold of a 250 MS/s signal, a spectrum
 * that repeats around 250 MHz with its sin(x)/x, and nothing else. Above 125 MHz a
 * hub signal has only such images.
 *
 * A 16-entry ring in distributed RAM, written every sys_clk, a Gray-coded write
 * pointer through two flops (a set_bus_skew in zcu216_vivado.tcl keeps its bits
 * within one step). The clocks come from one MMCM, so the reader runs freely at
 * the writer's rate, four reads per 5 fclk, DIST entries behind the pointer it
 * sees, and only realigns when that distance leaves GAP_MIN to GAP_MAX. No reset
 * and no flow control, so it needs neither clock running first (fclk only starts
 * once the CLK104 and the RF tiles are up).
 */
module zcu216_sig_cdc #(
  parameter int N = 4
) (
  input  logic         sys_clk,
  input  logic [13:0]  din  [N],
  input  logic         fclk,
  output logic [127:0] dout [N]                    // lane 0 oldest, sign-extended
);

localparam logic [3:0] DIST = 4'd3, GAP_MIN = 4'd1, GAP_MAX = 4'd5;

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
logic [3:0]      wb = '0, rp = '0, gap;
logic [2:0]      p = '0, p_d = '0;                // 0..4
logic [14*N-1:0] cur = '0, prev = '0;
assign gap = wb - rp;

always_ff @(posedge fclk) begin
  wg_m <= wg;
  wg_s <= wg_m;
  wb   <= {wg_s[3], ^wg_s[3:2], ^wg_s[3:1], ^wg_s[3:0]};
  p    <= (p == 3'd4) ? 3'd0 : p + 3'd1;
  p_d  <= p;
  if (p != 3'd4) begin
    rp   <= (gap >= GAP_MIN && gap <= GAP_MAX) ? rp + 1'b1 : wb - DIST;
    cur  <= ring[rp];
    prev <= cur;
  end
end

for (genvar c = 0; c < N; c++) begin : g_out
  logic signed [13:0] a, b;
  assign a = prev[14*c +: 14];
  assign b = cur[14*c +: 14];
  always_ff @(posedge fclk)
    for (int j = 0; j < 8; j++)
      dout[c][16*j +: 16] <= (p_d != 3'd4 && j < 2 * int'(p_d)) ? 16'(a) : 16'(b);
end

endmodule
