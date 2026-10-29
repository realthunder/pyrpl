/**
 * Trigger-synchronous running average of the ZCU216 scope's decimated samples
 * (zcu216_scope, between the decimator and the trigger/packer), both channels.
 *
 * Every edge of the external input (e_in, the laser's chirp sync: rising or
 * falling, fall) starts a record: the samples after it get indices 0, 1, ...
 * (up to 2^PL; later samples are not averaged and come out as 0), and sample i
 * updates its accumulator, a running (exponential) mean over the records:
 *
 *     acc[i] += ((x << F) - acc[i]) >>> k_r
 *
 * k_r = min(k, floor(log2 r)) for the r-th record since the restart (clr, or
 * en switching on): record 1 loads acc = x, records 2-3 weigh 1/2, 4-7 1/4, ...,
 * then 2^-k for good: a quick warm-up, then a mean over ~2^k records. An entry
 * not written since the restart (an index the earlier records never reached)
 * is loaded the same way, by an epoch tag stored beside it. Between a restart
 * and its first edge nothing is averaged (the output is 0).
 *
 * With en, the output samples are the updated means with two fraction bits
 * (16 bits, 4x the input scale: the input holds 14-bit values), so averaging
 * lowers the floor below one input LSB; without en, the input passes. Either
 * way through the same LAT cycles, with n and the edge flags beside the
 * samples, so the scope's triggers see the same timing on or off.
 *
 * The input is up to four samples per clock (n_in: 4, 2, 1 or 0), consecutive.
 * The accumulators are four lanes (index mod 4) of 2^(PL-2) words of
 * {tag 8, channel 2 mean 32, channel 1 mean 32}, read and written back
 * LAT - 2 cycles later; an index repeats only one record later, so a lane never
 * reads a word before its write-back (records of a few clocks would, and
 * average garbage: the trigger source sets the record length).
 *
 * Resolution: an edge lands on the first sample of the next beat that carries
 * samples (8 input samples; at D >= 16 the next decimated sample), so the
 * records jitter by up to a beat against the chirp. That shrinks a beat note
 * at f by sinc(f * T) in the mean (T = 3.2 ns, or the decimated sample period
 * at D >= 16): about -0.6 dB at 75 MHz, -3.9 dB at 156 MHz.
 */
module zcu216_scope_tavg #(
  parameter int PL  = 15,                         // log2 of the samples per record
  localparam int F  = 16,                         // fraction bits of the means
  localparam int AW = PL - 2                      // lane address bits
) (
  input  logic               clk,
  input  logic               rstn,
  input  logic signed [15:0] d_in [2][4],         // samples 0..n_in-1 valid
  input  logic [2:0]         n_in,
  input  logic [1:0]         e_in,                // [0] rising, [1] falling edge at sample 0
  // settings (quasi-static) and the restart toggle
  input  logic               en,
  input  logic               fall,                // the record edge: 0 rising, 1 falling
  input  logic [3:0]         k,                   // log2 of the records in the mean
  input  logic               clr,                 // toggle: start over
  output logic signed [15:0] d_out [2][4],        // LAT cycles after the input
  output logic [2:0]         n_out,
  output logic [1:0]         e_out,
  // status
  output logic [15:0]        recs,                // records since the restart (saturating)
  output logic [PL:0]        period               // samples in the last whole record
);

localparam int LAT = 8;

////////////////////////////////////////////////////////////////////////////////
// indices: stage 1
////////////////////////////////////////////////////////////////////////////////

logic [PL:0] idx;                                 // index of the next sample (2^PL: none)
logic [7:0]  epoch;
logic        en_d, clr_d;
logic [3:0]  kr;                                  // k_r of the current record

wire        edge_now = (fall ? e_in[1] : e_in[0]) && n_in != 0;
wire        restart  = (clr != clr_d) || (en && !en_d);
wire [PL:0] base     = edge_now ? '0 : idx;
wire [PL+1:0] nxt    = (PL+2)'(base) + (PL+2)'(n_in);

// floor(log2 r) of the next record, r = recs + 1 (a cycle behind recs, which
// moves only on an edge)
logic [3:0] lg;
always_ff @(posedge clk) begin
  lg <= '0;
  for (int b = 1; b < 16; b++) if (17'(recs) + 17'd1 >= (17'd1 << b)) lg <= 4'(b);
end

always_ff @(posedge clk) begin
  en_d  <= en;
  clr_d <= clr;
  if (!rstn) begin
    idx    <= (PL+1)'(1) << PL;
    recs   <= '0;
    period <= (PL+1)'(1) << PL;
    epoch  <= '0;
    kr     <= '0;
  end else if (restart) begin
    idx    <= (PL+1)'(1) << PL;
    recs   <= '0;
    epoch  <= epoch + 1'b1;
  end else begin
    if (n_in != 0) idx <= nxt[PL+1:PL] != 0 ? (PL+1)'(1) << PL : nxt[PL:0];
    if (edge_now) begin
      period <= idx;
      if (recs != 16'hffff) recs <= recs + 1'b1;
      kr     <= (lg < k) ? lg : k;
    end
  end
end

// per lane: the sample landing in it (index mod 4 = lane), its address, valid
logic              l_v  [4];
logic [AW-1:0]     l_a  [4];
logic signed [15:0] l_x [4][2];
logic [3:0]        k1;
logic [7:0]        ep1;
logic signed [15:0] d1 [2][4];
logic [2:0]        n1;
logic [1:0]        e1, r1;                        // r1: lane of sample 0
logic [3:0]        sv1;                           // sample s has an index (< 2^PL)

always_ff @(posedge clk) begin
  d1  <= d_in;
  n1  <= n_in;
  e1  <= e_in;
  r1  <= base[1:0];
  ep1 <= epoch;
  k1  <= edge_now ? ((lg < k) ? lg : k) : kr;
  for (int s = 0; s < 4; s++) begin
    automatic logic [PL:0] j = base + (PL+1)'(s);
    sv1[s] <= en && !restart && 3'(s) < n_in && !j[PL];
  end
  for (int l = 0; l < 4; l++) begin
    automatic logic [1:0]  s  = 2'(l) - base[1:0];
    automatic logic [PL:0] j  = base + (PL+1)'(s);
    l_v[l] <= en && !restart && 3'(s) < n_in && !j[PL];
    l_a[l] <= j[PL-1:2];
    for (int c = 0; c < 2; c++) l_x[l][c] <= d_in[c][s];
  end
end

////////////////////////////////////////////////////////////////////////////////
// accumulators: read at stage 1, data at stage 4, updated over 5-7, written at 7
////////////////////////////////////////////////////////////////////////////////

logic signed [F+16:0] dif5 [4][2];                // (x << F) - acc
logic signed [31:0]   old5 [4][2], new7 [4][2];
logic signed [F+16:0] sh6  [4][2];
logic signed [31:0]   old6 [4][2];
logic signed [15:0]   x5 [4][2], x6 [4][2];
logic                 v [4][8];                   // lane valid, stages 1..7
logic [AW-1:0]        a [4][8];
logic                 i5 [4], i6 [4];             // load (no mean yet)
logic [3:0]           kk [8];
logic [7:0]           ep [8];
logic signed [15:0]   xs [4][2][8];

for (genvar l = 0; l < 4; l++) begin : g_lane
  (* ram_style = "ultra" *) logic [71:0] mem [1 << AW];
  logic [71:0] q2, q3, q4;
  logic        we7;
  logic [71:0] wd7;

`ifndef SYNTHESIS
  initial for (int i = 0; i < (1 << AW); i++) mem[i] = '0;   // URAM powers up 0
`endif
  always_ff @(posedge clk) begin
    if (we7) mem[a[l][7]] <= wd7;
    q2 <= mem[l_a[l]];
    q3 <= q2;
    q4 <= q3;
  end

  // the lane's sample and controls along the pipe
  always_ff @(posedge clk) begin
    v[l][2] <= l_v[l];   a[l][2] <= l_a[l];   xs[l][0][2] <= l_x[l][0];  xs[l][1][2] <= l_x[l][1];
    for (int t = 3; t <= 7; t++) begin
      v[l][t] <= v[l][t-1];  a[l][t] <= a[l][t-1];
      xs[l][0][t] <= xs[l][0][t-1];  xs[l][1][t] <= xs[l][1][t-1];
    end
  end

  // stage 5: the old mean (or a load), the difference
  always_ff @(posedge clk) begin
    automatic logic ld = (q4[71:64] != ep[4]) || kk[4] == 0;
    i5[l] <= ld;
    for (int c = 0; c < 2; c++) begin
      automatic logic signed [31:0] o = q4[32*c +: 32];
      old5[l][c] <= o;
      dif5[l][c] <= ((F+17)'(xs[l][c][4]) <<< F) - (F+17)'(o);
      x5[l][c]   <= xs[l][c][4];
    end
  end
  // stage 6: scaled by 2^-k_r
  always_ff @(posedge clk) begin
    i6[l] <= i5[l];
    for (int c = 0; c < 2; c++) begin
      sh6[l][c]  <= dif5[l][c] >>> kk[5];
      old6[l][c] <= old5[l][c];
      x6[l][c]   <= x5[l][c];
    end
  end
  // stage 7: the new mean, written back
  always_ff @(posedge clk)
    for (int c = 0; c < 2; c++)
      new7[l][c] <= i6[l] ? 32'(x6[l][c]) <<< F : old6[l][c] + 32'(sh6[l][c]);
  assign we7 = v[l][7];
  assign wd7 = {ep[7], new7[l][1], new7[l][0]};
end

always_ff @(posedge clk) begin
  kk[2] <= k1;  ep[2] <= ep1;
  for (int t = 3; t <= 7; t++) begin kk[t] <= kk[t-1];  ep[t] <= ep[t-1]; end
end

////////////////////////////////////////////////////////////////////////////////
// output: stage 8, back in sample order; the input beside it, delayed alike
////////////////////////////////////////////////////////////////////////////////

logic signed [15:0] dp [2][4][2:7];
logic [2:0]         np [2:7];
logic [1:0]         ep_ [2:7], rp [2:7];
logic [3:0]         svp [2:7];
logic               enp [2:7];

always_ff @(posedge clk) begin
  for (int c = 0; c < 2; c++) for (int s = 0; s < 4; s++) begin
    dp[c][s][2] <= d1[c][s];
    for (int t = 3; t <= 7; t++) dp[c][s][t] <= dp[c][s][t-1];
  end
  np[2] <= n1;  ep_[2] <= e1;  rp[2] <= r1;  svp[2] <= sv1;  enp[2] <= en;
  for (int t = 3; t <= 7; t++) begin
    np[t] <= np[t-1];  ep_[t] <= ep_[t-1];  rp[t] <= rp[t-1];  svp[t] <= svp[t-1];
    enp[t] <= enp[t-1];
  end
end

always_ff @(posedge clk) begin
  n_out <= np[7];
  e_out <= ep_[7];
  for (int s = 0; s < 4; s++) begin
    automatic logic [1:0] l = rp[7] + 2'(s);
    for (int c = 0; c < 2; c++) begin
      // the mean with two fraction bits, rounded, saturated
      automatic logic signed [31:0] m = (new7[l][c] + (32'sd1 <<< (F - 3))) >>> (F - 2);
      automatic logic signed [15:0] y = (m > 32'sd32767) ? 16'sd32767 :
                                        (m < -32'sd32768) ? -16'sd32768 : 16'(m);
      d_out[c][s] <= !enp[7] ? dp[c][s][7] : (svp[7][s] ? y : 16'sd0);
    end
  end
end

endmodule
