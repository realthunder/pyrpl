/**
 * One RF-ADC channel at 250 MS/s, any band of 0-1.25 GHz, properly filtered, for
 * the signal hub (zcu216_dsp): rx0_i and rx0_q are hub signals like in1. It has to
 * sit here, before the hub: the filter must see the 2.5 GS/s samples, since what
 * aliases in a decimation cannot be taken out after it. in1-4 (zcu216_adc_dec)
 * stay the fast route, the mean of 10; this one is clean and about 50 ns later.
 *
 *   ADC lanes -> mixer -> sinc^5, / 5 -> FIR, / 2 -> ring -> real output
 *
 * Mixer: an oscillator at step / 2^32 * 2.5 GHz (32-bit accumulator, 12-bit sine
 * table) takes the band to 0 Hz: I = x cos, Q' = x sin, Q = -Q', so a tone A cos at
 * f + d gives I + jQ = A exp(j 2 pi d t) with gain2 on, which doubles the mixer's
 * output. Off for step = 0, where x cos is x itself: then I is a low-pass of x.
 *
 * sinc^5, / 5: 21 taps at 2.5 GS/s on each of I and Q, two outputs per fclk at
 * most (8 per 5 fclk, lane offsets p below), 500 MS/s.
 *
 * FIR, / 2: 39 taps at 500 MS/s, which also undo the sinc's droop. real_mode
 * chooses the coefficients:
 *   0, wide:   +-100 MHz, stop from +-150 MHz, about 59 dB. I and Q out.
 *   1, narrow: +-37.5 MHz, stop from +-87.5 MHz, about 83 dB. One real output.
 *
 * A hub module takes one real signal, which cannot tell f + d from f - d: for
 * them real_mode moves the band to 62.5 MHz, a quarter of the rate, I, -Q, -I, Q on
 * dout_i, so the tone comes out at 62.5 MHz + d. dout_q is 0 then.
 *
 * sel, step, real_mode, gain2 are settings from sys_clk (quasi-static).
 */
module zcu216_rx #(
  parameter string SIN_FILE = "",
  parameter int    N = 4
) (
  input  logic              fclk,
  input  logic [127:0]      adc [N],              // 8 lanes of 16 bits, 14 at the top
  input  logic [1:0]        sel,
  input  logic [31:0]       step,
  input  logic              real_mode,
  input  logic              gain2,
  input  logic              sys_clk,
  output logic [13:0]       dout_i,               // two's complement
  output logic [13:0]       dout_q
);

////////////////////////////////////////////////////////////////////////////////
// settings into fclk
////////////////////////////////////////////////////////////////////////////////

(* ASYNC_REG = "TRUE" *) logic [35:0] set_m = '0, set_s = '0;
logic [1:0]  sel_f = '0;
logic [31:0] step_f = '0;
logic        real_f = 1'b0, gain2_f = 1'b0;
always_ff @(posedge fclk) begin
  set_m <= {sel, step, real_mode, gain2};
  set_s <= set_m;
  {sel_f, step_f, real_f, gain2_f} <= set_s;
end

////////////////////////////////////////////////////////////////////////////////
// oscillator
////////////////////////////////////////////////////////////////////////////////

(* rom_style = "block" *) logic signed [15:0] rom [0:4095];
initial $readmemh(SIN_FILE, rom);

// phase of lane j = acc + j * step; acc advances 8 * step per fclk. The offsets
// settle in a few clocks (quasi-static); 16 bits are enough for a 12-bit address.
// Step 0 clears the phase: the low-pass is x cos(0), not x at wherever the phase
// stopped.
logic [31:0] acc = '0;
logic [31:0] off [8];
logic        stopped = 1'b1;
always_ff @(posedge fclk) begin
  stopped <= step_f == '0;
  acc    <= stopped ? '0 : acc + {step_f[28:0], 3'b0};
  off[0] <= '0;
  off[1] <= step_f;
  off[2] <= {step_f[30:0], 1'b0};
  off[3] <= off[2] + off[1];
  off[4] <= {step_f[29:0], 2'b0};
  off[5] <= off[4] + off[1];
  off[6] <= {off[3][30:0], 1'b0};
  off[7] <= off[4] + off[3];
end

// a sample and its table value only need the same delay from lane to lane
for (genvar j = 0; j < 8; j++) begin : g_nco
  logic [15:0] ph;
  logic [11:0] a_s, a_c;
  logic signed [15:0] r_s, r_c, sn, cs;
  logic signed [15:0] xm;

  always_ff @(posedge fclk) begin
    ph  <= acc[31:16] + off[j][31:16];
    a_s <= ph[15:4];
    a_c <= ph[15:4] + 12'd1024;
    r_s <= rom[a_s];
    r_c <= rom[a_c];
    sn  <= r_s;
    cs  <= r_c;
    xm  <= adc[sel_f][16*j +: 16];
  end
end

// the mixers, their DSPs' A, B, M and P registers
logic signed [17:0] y [2][8];                     // I, Q'
for (genvar j = 0; j < 8; j++) begin : g_mix
  (* use_dsp = "yes" *) logic signed [31:0] mi, mq;
  logic signed [15:0] xa, ca, sa;
  logic signed [31:0] pi, pq;
  always_ff @(posedge fclk) begin
    xa <= g_nco[j].xm;
    ca <= g_nco[j].cs;
    sa <= g_nco[j].sn;
    mi <= xa * ca;
    mq <= xa * sa;
    pi <= mi;
    pq <= mq;
    y[0][j] <= gain2_f ? 18'(pi >>> 14) : 18'(pi >>> 15);
    y[1][j] <= gain2_f ? 18'(pq >>> 14) : 18'(pq >>> 15);
  end
end

////////////////////////////////////////////////////////////////////////////////
// sinc^5, / 5: the outputs start at lane offsets (p: 0..4)
//   engine a: 0 2 4 1 3 (every fclk)    engine b: 5 7 - 6 -
////////////////////////////////////////////////////////////////////////////////

localparam logic signed [15:0] C1 [11] = '{
  16'd21, 16'd105, 16'd315, 16'd734, 16'd1468, 16'd2538, 16'd3880, 16'd5348, 16'd6711, 16'd7655, 16'd7986};

logic [2:0] p = '0;
always_ff @(posedge fclk) p <= (p == 3'd4) ? 3'd0 : p + 3'd1;

// the phase of the outputs of the sums below: t to s1, 9 clocks
localparam int L1 = 9;
logic [2:0] p_d [L1];
always_ff @(posedge fclk) begin
  p_d[0] <= p;
  for (int i = 1; i < L1; i++) p_d[i] <= p_d[i - 1];
end

logic signed [19:0] s1 [2][2];                    // [I, Q'][a, b], 500 MS/s

for (genvar r = 0; r < 2; r++) begin : g_s1
  logic signed [17:0] w [32];                     // oldest first
  always_ff @(posedge fclk) begin
    for (int k = 0; k < 24; k++) w[k] <= w[k + 8];
    for (int k = 0; k < 8; k++)  w[24 + k] <= y[r][k];
  end

  for (genvar e = 0; e < 2; e++) begin : g_e
    logic signed [17:0] t [21];
    logic signed [18:0] pa [11];
    (* use_dsp = "yes" *) logic signed [34:0] m [11];
    logic signed [34:0] m_q [11];
    logic signed [25:0] l1 [6];
    logic signed [25:0] l2 [3], l2b;
    logic signed [25:0] l3 [2];
    logic signed [25:0] l4;

    always_ff @(posedge fclk) begin
      for (int k = 0; k < 21; k++)
        if (e == 0)
          case (p)
            3'd0:    t[k] <= w[k];
            3'd1:    t[k] <= w[k + 2];
            3'd2:    t[k] <= w[k + 4];
            3'd3:    t[k] <= w[k + 1];
            default: t[k] <= w[k + 3];
          endcase
        else
          case (p)
            3'd0:    t[k] <= w[k + 5];
            3'd1:    t[k] <= w[k + 7];
            default: t[k] <= w[k + 6];                // p = 2, 4: no output
          endcase
      for (int k = 0; k < 10; k++) pa[k] <= 19'(t[k]) + 19'(t[20 - k]);
      pa[10] <= 19'(t[10]);
      for (int k = 0; k < 11; k++) begin
        m[k]   <= pa[k] * 17'(C1[k]);
        m_q[k] <= m[k];
      end
      // the sum of the coefficients is 2^16; 20 bits out: the mixer's * 4
      for (int k = 0; k < 5; k++) l1[k] <= 26'(m_q[2*k] >>> 9) + 26'(m_q[2*k + 1] >>> 9);
      l1[5] <= 26'(m_q[10] >>> 9);
      l2[0] <= l1[0] + l1[1];
      l2[1] <= l1[2] + l1[3];
      l2[2] <= l1[4] + l1[5];
      l3[0] <= l2[0] + l2[1];
      l3[1] <= l2[2];
      l4    <= l3[0] + l3[1];
      s1[r][e] <= 20'(l4 >>> 5);
    end
  end
end

////////////////////////////////////////////////////////////////////////////////
// FIR, / 2. Second-stage samples come 2, 2, 1, 2, 1 per fclk; pairs of them, 4 per
// 5 fclk, shift through d, and each pair ends one output.
////////////////////////////////////////////////////////////////////////////////

localparam logic signed [17:0] C_WIDE [20] = '{
  18'sd67, 18'sd184, 18'sd29, -18'sd428, -18'sd279,
  18'sd785, 18'sd814, -18'sd1233, -18'sd1788, 18'sd1718,
  18'sd3416, -18'sd2127, -18'sd6062, 18'sd2234, 18'sd10544,
  -18'sd1387, -18'sd19426, -18'sd3906, 18'sd45513, 18'sd73736};
localparam logic signed [17:0] C_NARROW [20] = '{
  18'sd49, 18'sd123, 18'sd145, 18'sd5, -18'sd322,
  -18'sd664, -18'sd670, -18'sd39, 18'sd1131, 18'sd2168,
  18'sd2065, 18'sd161, -18'sd3104, -18'sd5927, -18'sd5784,
  -18'sd795, 18'sd8904, 18'sd20688, 18'sd30365, 18'sd34074};

logic signed [17:0] co [20];
always_ff @(posedge fclk)
  for (int k = 0; k < 20; k++) co[k] <= real_f ? C_NARROW[k] : C_WIDE[k];

logic v2 = 1'b0;                                  // a new pair in pair[]
logic signed [19:0] hold [2], pair [2][2];        // [I, Q'][older, newer]

always_ff @(posedge fclk) begin
  v2 <= p_d[L1 - 1] != 3'd2;
  for (int r = 0; r < 2; r++)
    case (p_d[L1 - 1])
      3'd0, 3'd1: begin pair[r][0] <= s1[r][0];  pair[r][1] <= s1[r][1]; end
      3'd2:       hold[r] <= s1[r][0];
      3'd3:       begin pair[r][0] <= hold[r];   pair[r][1] <= s1[r][0];  hold[r] <= s1[r][1]; end
      default:    begin pair[r][0] <= hold[r];   pair[r][1] <= s1[r][0]; end
    endcase
end

// the sum below, from a pair to z, 12 clocks
localparam int L2 = 12;
logic [L2-1:0] v_d = '0;
always_ff @(posedge fclk) v_d <= {v_d[L2-2:0], v2};

logic signed [13:0] z [2];                        // I, Q' at 14 bits

for (genvar r = 0; r < 2; r++) begin : g_s2
  logic signed [19:0] d [40];                     // oldest first
  logic signed [20:0] pa [20];
  (* use_dsp = "yes" *) logic signed [38:0] m [20];
  logic signed [38:0] m_q [20];
  logic signed [29:0] a0 [10], a1 [5], a2 [3], a3 [2], a4;
  logic signed [29:0] rnd;
  logic signed [18:0] v;

  always_ff @(posedge fclk) begin
    if (v2) begin
      for (int k = 0; k < 38; k++) d[k] <= d[k + 2];
      d[38] <= pair[r][0];
      d[39] <= pair[r][1];
    end
    // 39 taps: d[1] .. d[39], the centre d[20]
    for (int k = 0; k < 19; k++) pa[k] <= 21'(d[1 + k]) + 21'(d[39 - k]);
    pa[19] <= 21'(d[20]);
    for (int k = 0; k < 20; k++) begin
      m[k]   <= pa[k] * co[k];
      m_q[k] <= m[k];
    end
    for (int k = 0; k < 10; k++) a0[k] <= 30'(m_q[2*k] >>> 9) + 30'(m_q[2*k + 1] >>> 9);
    for (int k = 0; k < 5; k++)  a1[k] <= a0[2*k] + a0[2*k + 1];
    a2[0] <= a1[0] + a1[1];
    a2[1] <= a1[2] + a1[3];
    a2[2] <= a1[4];
    a3[0] <= a2[0] + a2[1];
    a3[1] <= a2[2];
    a4    <= a3[0] + a3[1];
    // the coefficients' sum is 2^17, the input the mixer's * 4: 8 + 4 bits to go
    rnd   <= a4 + (30'sd1 << 11);
    v     <= 19'(rnd >>> 12);
    z[r]  <= (v > 8191) ? 14'h1FFF : (v < -8192) ? 14'h2000 : 14'(v);
  end
end

////////////////////////////////////////////////////////////////////////////////
// sys_clk
////////////////////////////////////////////////////////////////////////////////

logic [27:0] q;
zcu216_ring #(.W (28)) i_ring (.fclk, .we (v_d[L2-1]), .din ({z[1], z[0]}), .sys_clk, .q);

// I, Q, -I, -Q: at a quarter of the rate, I, -Q, -I, Q; Q = -Q'
logic signed [13:0] qi, qq, qi_d, qq_d, ni, nq;
logic [1:0] n = '0;
assign qi = q[13:0];
assign qq = q[27:14];

always_ff @(posedge sys_clk) begin
  n    <= n + 1'b1;
  qi_d <= qi;
  qq_d <= qq;
  ni   <= (qi == 14'h2000) ? 14'h1FFF : -qi;
  nq   <= (qq == 14'h2000) ? 14'h1FFF : -qq;
  if (real_mode)
    case (n)
      2'd0:    dout_i <= qi_d;
      2'd1:    dout_i <= qq_d;
      2'd2:    dout_i <= ni;
      default: dout_i <= nq;
    endcase
  else
    dout_i <= qi_d;
  dout_q <= real_mode ? 14'h0 : nq;
end

endmodule
