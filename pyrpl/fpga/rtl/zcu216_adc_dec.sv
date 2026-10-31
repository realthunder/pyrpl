/**
 * The RF-ADC channels for the system-clock modules (zcu216_dsp): 2.5 GS/s, eight
 * samples per fclk (312.5 MHz), down to 14 bits at 250 MS/s, one sample per
 * sys_clk.
 *
 * Each output is the mean of 10 consecutive samples, so no sample is lost and
 * none is used twice: 1.25 fclk per output. Lanes pair up, so a fclk brings four
 * pair sums s0-s3 (s0 the older) and an output takes five, four outputs per 5
 * fclk, phases p:
 *
 *   p      0            1            2         3            4
 *   h(p)                s0           s0 s1     s0 s1 s2     s0 s1 s2 s3
 *   t(p)   s0 s1 s2 s3  s1 s2 s3     s2 s3     s3
 *
 * so each phase but 0 ends one output with its head h(p), and each starts the
 * next with its tail t(p). A 10-sample mean nulls the multiples of 250 MHz and is -3.9 dB at
 * 125 MHz. Lanes 0-7 are time order (lane 0 oldest).
 *
 * The sums cross to sys_clk through zcu216_ring.
 *
 * Latency: about 7 fclk + 8 sys_clk after the last sample of a mean. The sums
 * are scaled on sys_clk, where a multiplier has time: mean / 4 (16-bit samples
 * to 14 bits, rounded), 20-bit sum * 2^16 / 5 / 2^19.
 */
module zcu216_adc_dec #(
  parameter int N = 4
) (
  input  logic         fclk,
  input  logic [127:0] adc  [N],                  // 8 lanes of 16 bits, 14 at the top
  input  logic         sys_clk,
  output logic [13:0]  dout [N]                   // two's complement
);

localparam int SW = 20;                           // sum of 10 16-bit samples

////////////////////////////////////////////////////////////////////////////////
// fclk: the sums
////////////////////////////////////////////////////////////////////////////////

// every fclk brings eight new samples, so any stage can take p as it is
logic [2:0]    p = '0;                            // 0..4
logic          emit_p = 1'b0, emit = 1'b0;        // sum holds a new output
logic [SW-1:0] sum [N];

always_ff @(posedge fclk) begin
  p      <= (p == 3'd4) ? 3'd0 : p + 3'd1;
  emit_p <= p != 3'd0;
  emit   <= emit_p;
end

for (genvar c = 0; c < N; c++) begin : g_sum
  logic signed [15:0]   x [8];
  logic signed [16:0]   s2 [4], s2_d [4];
  logic signed [17:0]   a, b, a_d, b_d;
  logic signed [18:0]   ab, a3, b3, h, t;         // a3 = s0-s2, b3 = s1-s3
  logic signed [16:0]   s0_e, s3_e;
  logic signed [SW-1:0] acc;

  always_ff @(posedge fclk) begin
    for (int j = 0; j < 8; j++) x[j] <= adc[c][16*j +: 16];
    for (int j = 0; j < 4; j++) s2[j] <= 17'(x[2*j]) + 17'(x[2*j + 1]);
    s2_d <= s2;
    a    <= 18'(s2[0]) + 18'(s2[1]);
    b    <= 18'(s2[2]) + 18'(s2[3]);
    ab   <= 19'(a) + 19'(b);
    a3   <= 19'(a) + 19'(s2_d[2]);
    b3   <= 19'(s2_d[1]) + 19'(b);
    a_d  <= a;
    b_d  <= b;
    s0_e <= s2_d[0];
    s3_e <= s2_d[3];
    case (p)
      3'd0:    begin h <= '0;        t <= ab;       end
      3'd1:    begin h <= 19'(s0_e); t <= b3;       end
      3'd2:    begin h <= 19'(a_d);  t <= 19'(b_d); end
      3'd3:    begin h <= a3;        t <= 19'(s3_e); end
      default: begin h <= ab;        t <= '0;       end
    endcase
    acc    <= SW'(t);
    sum[c] <= acc + SW'(h);
  end
end

////////////////////////////////////////////////////////////////////////////////
// the ring
////////////////////////////////////////////////////////////////////////////////

logic [N*SW-1:0] w, q;
always_comb
  for (int c = 0; c < N; c++) w[SW*c +: SW] = sum[c];

zcu216_ring #(.W (N*SW)) i_ring (.fclk, .we (emit), .din (w), .sys_clk, .q);

////////////////////////////////////////////////////////////////////////////////
// sys_clk: 16 bits to 14, the mean of 10
////////////////////////////////////////////////////////////////////////////////

for (genvar c = 0; c < N; c++) begin : g_out
  logic signed [SW-1:0] s;
  logic signed [SW+15:0] m;                       // * 13107 = 2^16 / 5
  logic signed [SW+15:0] r;
  assign r = (m + (36'sd1 << 18)) >>> 19;
  always_ff @(posedge sys_clk) begin
    s       <= q[SW*c +: SW];
    m       <= s * 16'sd13107;
    dout[c] <= (r > 8191) ? 14'h1FFF : 14'(r);
  end
end

endmodule
