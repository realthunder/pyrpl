/**
 * The RF-ADC channels for the system-clock modules (zcu216_dsp): 2.5 GS/s, eight
 * samples per fclk (312.5 MHz), down to 14 bits at 125 MS/s, one sample per
 * sys_clk, as the Red Pitaya's ADC gives them.
 *
 * Each output is the mean of 20 consecutive samples, so no sample is lost and
 * none is used twice: 2.5 fclk per output, so 4-sample groups a (lanes 0-3, the
 * older) and b (lanes 4-7) build two outputs per 5 fclk, phases p:
 *
 *   p      0     1     2         3     4
 *   out A  a b   a b   a
 *   out B                b       a b   a b
 *
 * A 20-sample mean nulls the multiples of 125 MHz and is -3.9 dB at 62.5 MHz.
 * Lanes 0-7 are time order (lane 0 oldest).
 *
 * The sums cross to sys_clk through a 16-entry ring in distributed RAM: the
 * write pointer, Gray coded, moves once per output through two flops. The clocks
 * come from one MMCM, so the reader runs freely at the writer's rate, 3 entries
 * behind the pointer it sees, and only realigns when that distance leaves 2-4
 * (at the start, after the MMCM relocks). A held pointer holds the output. No
 * reset, so it needs neither clock running first.
 *
 * Latency: about 5 fclk + 5 sys_clk after the last sample of a mean. The sums
 * are scaled on sys_clk, where a multiplier has time: mean / 4 (16-bit samples
 * to 14 bits, rounded), 21-bit sum * 2^16 / 5 / 2^20.
 */
module zcu216_adc_dec #(
  parameter int N = 4
) (
  input  logic         fclk,
  input  logic [127:0] adc  [N],                  // 8 lanes of 16 bits, 14 at the top
  input  logic         sys_clk,
  output logic [13:0]  dout [N]                   // two's complement
);

localparam int SW = 21;                           // sum of 20 16-bit samples

////////////////////////////////////////////////////////////////////////////////
// fclk: the sums
////////////////////////////////////////////////////////////////////////////////

// every fclk brings eight new samples, so any stage can take p as it is
logic [2:0]    p = '0;                            // 0..4
logic          emit;                              // sum holds a new output
logic [SW-1:0] sum [N];

always_ff @(posedge fclk) begin
  p    <= (p == 3'd4) ? 3'd0 : p + 3'd1;
  emit <= (p == 3'd2) || (p == 3'd4);
end

for (genvar c = 0; c < N; c++) begin : g_sum
  logic signed [15:0]   x [8];
  logic signed [16:0]   s2 [4];
  logic signed [17:0]   a, b, a_d, b_d;
  logic signed [18:0]   ab;
  logic signed [SW-1:0] acc, add;

  assign add = acc + ((p == 3'd2) ? SW'(a_d) : SW'(ab));

  always_ff @(posedge fclk) begin
    for (int j = 0; j < 8; j++) x[j] <= adc[c][16*j +: 16];
    for (int j = 0; j < 4; j++) s2[j] <= 17'(x[2*j]) + 17'(x[2*j + 1]);
    a   <= 18'(s2[0]) + 18'(s2[1]);
    b   <= 18'(s2[2]) + 18'(s2[3]);
    ab  <= 19'(a) + 19'(b);
    a_d <= a;
    b_d <= b;
    case (p)
      3'd0:    acc <= SW'(ab);
      3'd2:    acc <= SW'(b_d);
      default: acc <= add;
    endcase
    if (p == 3'd2 || p == 3'd4) sum[c] <= add;
  end
end

////////////////////////////////////////////////////////////////////////////////
// the ring
////////////////////////////////////////////////////////////////////////////////

(* ram_style = "distributed" *) logic [N*SW-1:0] ring [16];
logic [3:0]    wp = '0, wg = '0;
logic [N*SW-1:0] w;
always_comb
  for (int c = 0; c < N; c++) w[SW*c +: SW] = sum[c];

always_ff @(posedge fclk)
  if (emit) begin
    ring[wp] <= w;
    wp <= wp + 1'b1;
    wg <= (wp + 1'b1) ^ ((wp + 1'b1) >> 1);
  end

(* ASYNC_REG = "TRUE" *) logic [3:0] wg_m = '0, wg_s = '0;
logic [3:0]      wb = '0, rp = '0, gap;
logic [N*SW-1:0] q = '0;
assign gap = wb - rp;

always_ff @(posedge sys_clk) begin
  wg_m <= wg;
  wg_s <= wg_m;
  wb   <= {wg_s[3], ^wg_s[3:2], ^wg_s[3:1], ^wg_s[3:0]};
  rp   <= (gap >= 4'd2 && gap <= 4'd4) ? rp + 1'b1 : wb - 4'd3;
  q    <= ring[rp];
end

////////////////////////////////////////////////////////////////////////////////
// sys_clk: 16 bits to 14, the mean of 20
////////////////////////////////////////////////////////////////////////////////

for (genvar c = 0; c < N; c++) begin : g_out
  logic signed [SW-1:0] s;
  logic signed [SW+15:0] m;                       // * 13107 = 2^16 / 5
  logic signed [SW+15:0] r;
  assign r = (m + (36'sd1 << 19)) >>> 20;
  always_ff @(posedge sys_clk) begin
    s       <= q[SW*c +: SW];
    m       <= s * 16'sd13107;
    dout[c] <= (r > 8191) ? 14'h1FFF : 14'(r);
  end
end

endmodule
