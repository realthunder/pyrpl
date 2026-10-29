/**
 * Per-bin power average over M = 2^m frames (lidar/docs/Zcu216ChirpFft.md),
 * between zcu216_fft_reorder and everything that takes its frames (spectrum
 * window, CFAR, snapshots, point stream).
 *
 * In and out: four bins per clock in natural order (bin 4t + j on lane j),
 * t = 0 .. 2^SUB - 1, with the frame marks. Per bin, p = mag^2 is summed over
 * M frames into an accumulator; frame M of a group outputs the RMS magnitude
 *
 *     sqrt(sum / M)          (integer square root, 32 bits)
 *
 * so the output has the units and the format of the magnitudes it replaces,
 * and is the square root of the power average (the host's trace_average).
 * The other M - 1 frames of a group output nothing. m = 0 passes every frame
 * (sqrt(mag^2) = mag exactly). The first frame of a group loads the sums
 * instead of adding, so no clearing pass is needed; a restart (toggle, or a
 * new m) makes the next frame the first of a group.
 *
 * Storage: per lane 2^SUB words of 72 bits (64-bit power + 8 bits of growth
 * for M <= 256), one UltraRAM word per bin: at SUB = 13 two URAM288 per lane,
 * eight per instance. A frame reads each bin once and writes it back a few
 * clocks later; the same bin is read again only a frame later.
 *
 * Latency: 1 + 7 clocks to the sum / M, 32 through the square root, 1 out.
 */
module zcu216_fft_avg #(
  parameter int  SUB = 13
) (
  input  logic           clk,
  input  logic           rstn,
  input  logic [3:0]     m,                       // log2 M, clamped to 8 (quasi-static)
  input  logic           restart,                 // toggle
  input  logic [31:0]    r_mag [4],
  input  logic [SUB-1:0] r_t,
  input  logic           r_valid,
  input  logic           r_first,
  input  logic           r_last,
  output logic [31:0]    n_mag [4],
  output logic [SUB-1:0] n_t,
  output logic           n_valid,
  output logic           n_first,
  output logic           n_last
);

localparam int AW = SUB;
localparam int SL = 32;                           // square-root stages
localparam int PL = 7;                            // the sum / M is at stage PL

////////////////////////////////////////////////////////////////////////////////
// frame position in the group: decided at each frame's first beat
////////////////////////////////////////////////////////////////////////////////

logic [3:0] mc, mc_d;                             // m clamped (and a clock ago)
logic [7:0] fr;                                   // index of the last frame in its group
logic       rs_d, pend;                           // a restart not applied yet
logic       g_load, g_out;                        // the current frame: load / output

always_ff @(posedge clk) begin
  mc   <= (m > 4'd8) ? 4'd8 : m;
  mc_d <= mc;
end

wire [7:0] last_fr = 8'((9'(1) << mc) - 1);

always_ff @(posedge clk)
  if (!rstn) begin
    fr     <= '0;
    rs_d   <= 1'b0;
    pend   <= 1'b1;
    g_load <= 1'b1;
    g_out  <= 1'b0;
  end else begin
    rs_d <= restart;
    if (restart != rs_d || mc != mc_d) pend <= 1'b1;
    if (r_valid && r_first) begin
      if (pend || fr >= last_fr) begin            // a group's first frame
        fr     <= '0;
        g_load <= 1'b1;
        g_out  <= last_fr == '0;
        pend   <= 1'b0;
      end else begin
        fr     <= fr + 1'b1;
        g_load <= 1'b0;
        g_out  <= fr + 1'b1 == last_fr;
      end
    end
  end

// the input one clock later, beside its frame's role (stage 0)
logic [31:0]    a_mag [4];
logic [SUB-1:0] a_t;
logic           a_valid, a_first, a_last;
always_ff @(posedge clk) begin
  a_mag   <= r_mag;
  a_t     <= r_t;
  a_valid <= r_valid;
  a_first <= r_first;
  a_last  <= r_last;
end

////////////////////////////////////////////////////////////////////////////////
// per lane: power (stages 1-5), URAM read (1-5), sum (6), / M (7), write (7)
////////////////////////////////////////////////////////////////////////////////

logic [SUB-1:0] t_p [1:PL];
logic           v_p [1:PL], f_p [1:PL], l_p [1:PL], ld_p [1:PL], o_p [1:PL];

always_ff @(posedge clk) begin
  t_p[1] <= a_t;  v_p[1] <= a_valid;  f_p[1] <= a_first;  l_p[1] <= a_last;
  ld_p[1] <= g_load;  o_p[1] <= g_out;
  for (int i = 2; i <= PL; i++) begin
    t_p[i] <= t_p[i-1];  v_p[i] <= v_p[i-1];  f_p[i] <= f_p[i-1];  l_p[i] <= l_p[i-1];
    ld_p[i] <= ld_p[i-1];  o_p[i] <= o_p[i-1];
  end
end

logic [71:0] sum7 [4];                            // stage 7: the new sum (written back)
logic [63:0] q7 [4];                              // stage 7: the sum / M

for (genvar j = 0; j < 4; j++) begin : g_lane
  (* ram_style = "ultra" *) logic [71:0] mem [1 << AW];
`ifndef SYNTHESIS
  initial for (int i = 0; i < (1 << AW); i++) mem[i] = '0;
`endif
  logic [71:0] u1, u2, u3, u4, u5;                // read data, stages 1-5
  logic [31:0] m1;
  logic [63:0] p2, p3, p4, p5;                    // mag^2, stages 2-5 (DSP pipeline)
  logic [71:0] s6;

  always_ff @(posedge clk) begin
    if (v_p[PL]) mem[t_p[PL]] <= sum7[j];
    u1 <= mem[a_t];
    u2 <= u1;  u3 <= u2;  u4 <= u3;  u5 <= u4;
    m1 <= a_mag[j];
    p2 <= 64'(m1) * 64'(m1);
    p3 <= p2;  p4 <= p3;  p5 <= p4;
    s6 <= ld_p[5] ? 72'(p5) : u5 + 72'(p5);
    sum7[j] <= s6;
    q7[j]   <= 64'(s6 >> mc);
  end
end

////////////////////////////////////////////////////////////////////////////////
// integer square root of q7 (64 -> 32 bits), restoring, one bit per stage
////////////////////////////////////////////////////////////////////////////////

logic [63:0]    sx [4][SL+1];                     // radicand bits not yet brought down
logic [33:0]    sr [4][SL+1];                     // remainder (< 2 root + 1)
logic [31:0]    sq [4][SL+1];                     // root so far
logic [SUB-1:0] st [SL+1];
logic           sv [SL+1], sf [SL+1], sl [SL+1];

always_comb begin
  for (int j = 0; j < 4; j++) begin
    sx[j][0] = q7[j];
    sr[j][0] = '0;
    sq[j][0] = '0;
  end
  st[0] = t_p[PL];
  sv[0] = v_p[PL] && o_p[PL];
  sf[0] = f_p[PL];
  sl[0] = l_p[PL];
end

for (genvar k = 0; k < SL; k++) begin : g_root
  always_ff @(posedge clk) begin
    st[k+1] <= st[k];  sv[k+1] <= sv[k];  sf[k+1] <= sf[k];  sl[k+1] <= sl[k];
    for (int j = 0; j < 4; j++) begin
      automatic logic [35:0] rem = {sr[j][k], sx[j][k][63:62]};
      automatic logic [35:0] tr  = {2'b00, sq[j][k], 2'b01};
      sx[j][k+1] <= sx[j][k] << 2;
      if (rem >= tr) begin
        sr[j][k+1] <= 34'(rem - tr);
        sq[j][k+1] <= {sq[j][k][30:0], 1'b1};
      end else begin
        sr[j][k+1] <= 34'(rem);
        sq[j][k+1] <= {sq[j][k][30:0], 1'b0};
      end
    end
  end
end

always_ff @(posedge clk) begin
  for (int j = 0; j < 4; j++) n_mag[j] <= sq[j][SL];
  n_t     <= st[SL];
  n_valid <= sv[SL];
  n_first <= sv[SL] && sf[SL];
  n_last  <= sv[SL] && sl[SL];
end

endmodule
