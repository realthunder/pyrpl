/**
 * |X| for the four bins of each clock, by CORDIC vectoring (NI iterations).
 *
 * mag = round(K * |X| / 2), K = prod sqrt(1 + 2^-2i) = 1.64676 (fixed, so it
 * cancels in any detection or interpolation). |X| rather than |X|^2: from the
 * noise floor to full scale |X| spans ~24 bits, |X|^2 twice that, too wide for
 * the 32-bit reorder RAM. With NI = 14 the residual angle is < 1.3e-4 rad, so
 * the magnitude error is < 1e-8 relative, below the rounding.
 *
 * Latency NI + 2; k, valid and last ride along.
 */

module zcu216_fft_mag #(
  parameter int  XW  = 32,
  parameter int  SUB = 15,
  parameter int  NI  = 14
) (
  input  logic                 clk,
  input  logic signed [XW-1:0] x_re [4],
  input  logic signed [XW-1:0] x_im [4],
  input  logic [SUB-1:0]       x_k,
  input  logic                 x_valid,
  input  logic                 x_last,
  output logic [31:0]          mag [4],
  output logic [SUB-1:0]       m_k,
  output logic                 m_valid,
  output logic                 m_last
);

localparam int G = 3;                        // guard bits
localparam int W = XW + 2 + G;               // |x| <= sqrt2 * 2^(XW-1) * K < 2^(XW+1)

for (genvar q = 0; q < 4; q++) begin : g_lane
  logic signed [W-1:0] x [NI+1], y [NI+1];

  // stage 0: fold into the right half-plane (magnitude unchanged)
  always_ff @(posedge clk) begin
    x[0] <= (x_re[q] < 0) ? -(W'(x_re[q]) <<< G) : (W'(x_re[q]) <<< G);
    y[0] <= (x_re[q] < 0) ? -(W'(x_im[q]) <<< G) : (W'(x_im[q]) <<< G);
  end

  for (genvar i = 0; i < NI; i++) begin : g_it
    always_ff @(posedge clk)
      if (y[i] >= 0) begin
        x[i+1] <= x[i] + (y[i] >>> i);
        y[i+1] <= y[i] - (x[i] >>> i);
      end else begin
        x[i+1] <= x[i] - (y[i] >>> i);
        y[i+1] <= y[i] + (x[i] >>> i);
      end
  end

  // K|X| / 2, rounded
  always_ff @(posedge clk)
    mag[q] <= 32'((x[NI] + (W'(1) <<< G)) >>> (G + 1));
end

zcu216_delay #(.W(SUB + 2), .D(NI + 2)) i_dly (
  .clk, .d({x_k, x_valid, x_last}), .q({m_k, m_valid, m_last}));

endmodule
