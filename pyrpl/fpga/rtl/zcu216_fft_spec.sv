/**
 * Per-frame spectrum results for one zcu216_fft_dit8 channel:
 *
 *  - |X|^2 for the four bins of each clock (bin = k + q * 2^SUB on lane q);
 *  - the largest |X|^2 of the frame among bins >= min_bin (DC and its leakage are
 *    excluded that way), its bin, and a frame counter, all latched at frame end
 *    and announced by toggling upd;
 *  - a window of WIN consecutive bins from win_start, written into a dual-clock
 *    RAM as min(|X|^2 >> mag_shift, 2^32-1) and read on rd_clk. arm (a toggle)
 *    captures the next whole frame; done (a toggle) follows it when that frame is
 *    in. Bins of one frame all differ in k, and two bins of one clock are 2^SUB
 *    apart, so at most one lane falls in the window per clock.
 *
 * Inputs from the register clock (min_bin, win_start, mag_shift) are quasi-static.
 */

module zcu216_fft_spec #(
  parameter int  SUB  = 15,
  parameter int  XW   = 32,
  parameter int  WIN  = 4096,
  localparam int BW   = SUB + 2,              // bin index, lower half spectrum
  localparam int MW   = 2 * XW - 1,           // |X|^2 width
  localparam int AW   = $clog2(WIN)
) (
  input  logic                 clk,
  input  logic                 rstn,
  input  logic signed [XW-1:0] x_re [4],
  input  logic signed [XW-1:0] x_im [4],
  input  logic [SUB-1:0]       x_k,
  input  logic                 x_valid,
  input  logic                 x_last,
  // settings (quasi-static)
  input  logic [BW-1:0]        min_bin,
  input  logic [BW-1:0]        win_start,
  input  logic [5:0]           mag_shift,
  input  logic                 arm,          // toggle
  // results, stable between upd toggles
  output logic                 upd,          // toggles once per frame
  output logic [31:0]          frames,
  output logic [BW-1:0]        peak_bin,
  output logic [MW-1:0]        peak_mag,
  output logic                 done,         // toggle
  // window RAM read port
  input  logic                 rd_clk,
  input  logic [AW-1:0]        rd_addr,
  output logic [31:0]          rd_data
);

// stage 1-3: |X|^2 per lane (registers around the products for DSP pipelining)
logic signed [XW-1:0]   r_re [4], r_im [4];
logic signed [2*XW-1:0] p_re [4], p_im [4], p2_re [4], p2_im [4];
logic [MW-1:0]          mag [4];
logic [SUB-1:0]         k1, k2, k3, k4;
logic [3:0]             v_sr = '0, l_sr = '0;

always_ff @(posedge clk) begin
  for (int q = 0; q < 4; q++) begin
    r_re[q]  <= x_re[q];          r_im[q]  <= x_im[q];
    p_re[q]  <= r_re[q] * r_re[q]; p_im[q] <= r_im[q] * r_im[q];
    p2_re[q] <= p_re[q];          p2_im[q] <= p_im[q];
    mag[q]   <= MW'(p2_re[q]) + MW'(p2_im[q]);
  end
  k1 <= x_k;  k2 <= k1;  k3 <= k2;  k4 <= k3;
  v_sr <= {v_sr[2:0], x_valid};
  l_sr <= {l_sr[2:0], x_valid && x_last};
end

wire           m_valid = v_sr[3];
wire           m_last  = l_sr[3];
wire [SUB-1:0] m_k     = k4;

// stage 4-5: best of the four lanes (bins below min_bin count as 0)
logic [MW-1:0] b01_mag, b23_mag, b_mag;
logic [BW-1:0] b01_bin, b23_bin, b_bin;
logic [1:0]    b_v = '0, b_l = '0;

function automatic logic [BW-1:0] bin_of(input int q, input logic [SUB-1:0] k);
  return {2'(q), k};
endfunction

always_ff @(posedge clk) begin
  automatic logic [MW-1:0] m [4];
  for (int q = 0; q < 4; q++)
    m[q] = (bin_of(q, m_k) >= min_bin) ? mag[q] : '0;
  {b01_mag, b01_bin} <= (m[1] > m[0]) ? {m[1], bin_of(1, m_k)} : {m[0], bin_of(0, m_k)};
  {b23_mag, b23_bin} <= (m[3] > m[2]) ? {m[3], bin_of(3, m_k)} : {m[2], bin_of(2, m_k)};
  {b_mag, b_bin}     <= (b23_mag > b01_mag) ? {b23_mag, b23_bin} : {b01_mag, b01_bin};
  b_v <= {b_v[0], m_valid};
  b_l <= {b_l[0], m_valid && m_last};
end

// stage 6: running maximum over the frame, latched at frame end
logic [MW-1:0] run_mag;
logic [BW-1:0] run_bin;
logic          first;

always_ff @(posedge clk)
  if (!rstn) begin
    first    <= 1'b1;
    run_mag  <= '0;
    run_bin  <= '0;
    upd      <= 1'b0;
    frames   <= '0;
    peak_bin <= '0;
    peak_mag <= '0;
  end else if (b_v[1]) begin
    automatic logic          take = first || (b_mag > run_mag);
    automatic logic [MW-1:0] nm   = take ? b_mag : run_mag;
    automatic logic [BW-1:0] nb   = take ? b_bin : run_bin;
    run_mag <= nm;
    run_bin <= nb;
    first   <= b_l[1];
    if (b_l[1]) begin
      peak_mag <= nm;
      peak_bin <= nb;
      frames   <= frames + 1'b1;
      upd      <= !upd;
    end
  end

// window capture: arm (toggle) -> wait for a frame start -> capture -> done (toggle)
logic [2:0] arm_s;
logic       armed, capturing, frame_start;
logic       w_en;
logic [AW-1:0] w_addr;
logic [31:0]   w_data;

always_ff @(posedge clk) arm_s <= {arm_s[1:0], arm};

always_ff @(posedge clk)
  if (!rstn) begin
    armed       <= 1'b0;
    capturing   <= 1'b0;
    done        <= 1'b0;
    frame_start <= 1'b1;
  end else begin
    if (arm_s[2] != arm_s[1]) armed <= 1'b1;
    if (m_valid) begin
      frame_start <= m_last;
      if (frame_start && armed) begin
        capturing <= 1'b1;
        armed     <= 1'b0;
      end
      if (m_last && (capturing || (frame_start && armed))) begin
        capturing <= 1'b0;
        done      <= !done;
      end
    end
  end

always_ff @(posedge clk) begin
  automatic logic           hit = 1'b0;
  automatic logic [BW-1:0]  off = '0;
  automatic logic [MW-1:0]  mv  = '0;
  for (int q = 0; q < 4; q++) begin
    automatic logic [BW-1:0] o = bin_of(q, m_k) - win_start;
    if (bin_of(q, m_k) >= win_start && o < BW'(WIN)) begin
      hit = 1'b1;  off = o;  mv = mag[q];
    end
  end
  w_en   <= m_valid && (capturing || (frame_start && armed)) && hit;
  w_addr <= AW'(off);
  w_data <= ((mv >> mag_shift) >> 32) != 0 ? 32'hffff_ffff : 32'(mv >> mag_shift);
end

(* ram_style = "block" *) logic [31:0] ram [0:WIN-1];

always_ff @(posedge clk)
  if (w_en) ram[w_addr] <= w_data;

logic [31:0] rd_q;
always_ff @(posedge rd_clk) begin
  rd_q    <= ram[rd_addr];
  rd_data <= rd_q;
end

endmodule
