/**
 * Test source for the ZCU216 FFT until the RF-ADC is in: a continuous stream of
 * 8 real samples per clock (sample 8b+j on lane j), two tones plus uniform noise,
 * saturated to IN_W bits. Like an ADC it never stalls; zcu216_fft_dit8 does the
 * framing.
 *
 * Tone frequency: inc is the phase step per sample, f = inc / 2^32 * fs. With a
 * 2^NFFT-point FFT, inc = m << (32 - NFFT) puts a tone exactly on bin m. amp is
 * the peak in IN_W-bit LSBs times 2 (0..32767 -> 0..16383 LSB, then saturated).
 * nbits = 0 turns the noise off, otherwise noise is uniform over nbits bits.
 */

module zcu216_tone_gen #(
  parameter int    IN_W     = 14,
  parameter string SIN_FILE = ""
) (
  input  logic              clk,
  input  logic              rstn,
  input  logic              run,
  input  logic [32-1:0]     inc_a,
  input  logic [16-1:0]     amp_a,
  input  logic [32-1:0]     inc_b,
  input  logic [16-1:0]     amp_b,
  input  logic [5-1:0]      nbits,
  output logic [8*16-1:0]   data,
  output logic              valid
);

(* rom_style = "block" *) logic signed [15:0] rom [0:4095];
initial $readmemh(SIN_FILE, rom);

// phase of lane j = acc + j*inc; acc advances 8*inc per beat
logic [31:0] off_a [8], off_b [8];
logic [31:0] acc_a, acc_b;

always_ff @(posedge clk)
  for (int j = 0; j < 8; j++) begin
    off_a[j] <= 32'(j) * inc_a;
    off_b[j] <= 32'(j) * inc_b;
  end

always_ff @(posedge clk)
  if (!rstn) begin
    acc_a <= '0;
    acc_b <= '0;
  end else begin
    acc_a <= acc_a + (inc_a << 3);
    acc_b <= acc_b + (inc_b << 3);
  end

// noise: one xorshift32 per lane
logic [31:0] rnd [8];

logic [31:0] rnd1 [8], rnd2 [8], rnd3 [8];

always_comb
  for (int j = 0; j < 8; j++) begin
    rnd1[j] = rnd[j]  ^ (rnd[j]  << 13);
    rnd2[j] = rnd1[j] ^ (rnd1[j] >> 17);
    rnd3[j] = rnd2[j] ^ (rnd2[j] << 5);
  end

always_ff @(posedge clk)
  for (int j = 0; j < 8; j++)
    if (!rstn) rnd[j] <= 32'h9e3779b9 * (j + 1);
    else       rnd[j] <= rnd3[j];

// pipeline: phase -> ROM (2) -> multiply (2) -> sum, saturate
localparam int LAT = 6;
logic [LAT-1:0] v_sr = '0;

for (genvar j = 0; j < 8; j++) begin : g_lane
  logic [11:0]        pa, pb;
  logic signed [15:0] sa0, sb0, sa, sb;
  logic signed [31:0] ma, mb;
  logic signed [17:0] n0, n;
  logic signed [18:0] s;

  always_ff @(posedge clk) begin
    pa  <= 12'((acc_a + off_a[j]) >> 20);
    pb  <= 12'((acc_b + off_b[j]) >> 20);
    sa0 <= rom[pa];  sa <= sa0;
    sb0 <= rom[pb];  sb <= sb0;
    ma  <= sa * $signed({1'b0, amp_a[14:0]});
    mb  <= sb * $signed({1'b0, amp_b[14:0]});
    n0  <= (nbits == 0) ? '0 : 18'($signed(rnd[j][31:14]) >>> (18 - nbits));
    n   <= n0;
    s   <= 19'(ma >>> 16) + 19'(mb >>> 16) + 19'(n);
    if      (s >  (2**(IN_W-1) - 1)) data[16*j +: 16] <= 16'(2**(IN_W-1) - 1);
    else if (s < -(2**(IN_W-1)))     data[16*j +: 16] <= 16'(-(2**(IN_W-1)));
    else                             data[16*j +: 16] <= 16'(s);
  end
end

always_ff @(posedge clk)
  v_sr <= {v_sr[LAT-2:0], rstn && run};

assign valid = v_sr[LAT-1];

endmodule
