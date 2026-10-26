/**
 * Per-frame results for one channel, from the natural-order magnitudes of
 * zcu216_fft_reorder (bin 4t + j on lane j, mag = K|X|/2):
 *
 *  - the largest magnitude of the frame among bins >= min_bin (DC and its
 *    leakage are excluded that way), its bin, and a frame counter, latched at
 *    frame end and announced by toggling upd;
 *  - a window of WIN consecutive bins from win_start (rounded down to a multiple
 *    of 4), written into a dual-clock RAM and read on rd_clk. arm (a toggle)
 *    captures the next whole frame; done (a toggle) follows when it is in.
 *
 * Inputs from the register clock (min_bin, win_start) are quasi-static.
 */

module zcu216_fft_spec #(
  parameter int  SUB  = 15,
  parameter int  WIN  = 4096,
  localparam int BW   = SUB + 2,              // bin index, lower half spectrum
  localparam int AW   = $clog2(WIN)
) (
  input  logic              clk,
  input  logic              rstn,
  input  logic [31:0]       n_mag [4],
  input  logic [SUB-1:0]    n_t,
  input  logic              n_valid,
  input  logic              n_first,
  input  logic              n_last,
  // settings (quasi-static)
  input  logic [BW-1:0]     min_bin,
  input  logic [BW-1:0]     win_start,
  input  logic              arm,              // toggle
  // results, stable between upd toggles
  output logic              upd,              // toggles once per frame
  output logic [31:0]       frames,
  output logic [BW-1:0]     peak_bin,
  output logic [31:0]       peak_mag,
  output logic              done,             // toggle
  // window RAM read port
  input  logic              rd_clk,
  input  logic [AW-1:0]     rd_addr,
  output logic [31:0]       rd_data
);

////////////////////////////////////////////////////////////////////////////////
// peak: best of the four lanes, then a running maximum over the frame
////////////////////////////////////////////////////////////////////////////////

logic [31:0]   b01_mag, b23_mag, b_mag;
logic [BW-1:0] b01_bin, b23_bin, b_bin;
logic [2:0]    b_v = '0, b_f = '0, b_l = '0;

logic [31:0]    m [4];                        // bins below min_bin count as 0
logic [SUB-1:0] m_t;

// the mask gets a cycle of its own: n_mag also feeds the CFAR detector and the
// snapshot, and the mask plus the first compare missed timing behind that load
always_ff @(posedge clk) begin
  for (int j = 0; j < 4; j++)
    m[j] <= ({n_t, 2'(j)} >= min_bin) ? n_mag[j] : '0;
  m_t <= n_t;
end

always_ff @(posedge clk) begin
  {b01_mag, b01_bin} <= (m[1] > m[0]) ? {m[1], {m_t, 2'd1}} : {m[0], {m_t, 2'd0}};
  {b23_mag, b23_bin} <= (m[3] > m[2]) ? {m[3], {m_t, 2'd3}} : {m[2], {m_t, 2'd2}};
  {b_mag, b_bin}     <= (b23_mag > b01_mag) ? {b23_mag, b23_bin} : {b01_mag, b01_bin};
  b_v <= {b_v[1:0], n_valid};
  b_f <= {b_f[1:0], n_valid && n_first};
  b_l <= {b_l[1:0], n_valid && n_last};
end

logic [31:0]   run_mag, nm;
logic [BW-1:0] run_bin, nb;

always_comb begin
  nm = (b_f[2] || b_mag > run_mag) ? b_mag : run_mag;
  nb = (b_f[2] || b_mag > run_mag) ? b_bin : run_bin;
end

always_ff @(posedge clk)
  if (!rstn) begin
    run_mag  <= '0;
    run_bin  <= '0;
    upd      <= 1'b0;
    frames   <= '0;
    peak_bin <= '0;
    peak_mag <= '0;
  end else if (b_v[2]) begin
    run_mag <= nm;
    run_bin <= nb;
    if (b_l[2]) begin
      peak_mag <= nm;
      peak_bin <= nb;
      frames   <= frames + 1'b1;
      upd      <= !upd;
    end
  end

////////////////////////////////////////////////////////////////////////////////
// window capture: arm (toggle) -> next frame start -> capture -> done (toggle)
////////////////////////////////////////////////////////////////////////////////

localparam int BA = AW - 2;                   // per-bank address bits

logic [2:0] arm_s;
logic       armed, capturing;

always_ff @(posedge clk) arm_s <= {arm_s[1:0], arm};

wire start_cap = n_valid && n_first && armed;
wire cap       = n_valid && (capturing || start_cap);

always_ff @(posedge clk)
  if (!rstn) begin
    armed     <= 1'b0;
    capturing <= 1'b0;
    done      <= 1'b0;
  end else begin
    if (arm_s[2] != arm_s[1]) armed <= 1'b1;
    if (start_cap) begin
      armed     <= 1'b0;
      capturing <= 1'b1;
    end
    if (cap && n_last) begin
      capturing <= 1'b0;
      done      <= !done;
    end
  end

// beat t covers bins 4t..4t+3: lane j goes to bank j at t - win_start/4
wire [SUB-1:0] ws4 = win_start[BW-1:2];
wire [SUB-1:0] off = n_t - ws4;

logic          w_en;
logic [BA-1:0] w_addr;
logic [31:0]   w_data [4];

always_ff @(posedge clk) begin
  w_en   <= cap && n_t >= ws4 && off < SUB'(WIN / 4);
  w_addr <= BA'(off);
  w_data <= n_mag;
end

logic [31:0] rq [4];
logic [1:0]  rsel;

for (genvar j = 0; j < 4; j++) begin : g_win
  (* ram_style = "block" *) logic [31:0] ram [0:WIN/4-1];
  always_ff @(posedge clk)
    if (w_en) ram[w_addr] <= w_data[j];
  always_ff @(posedge rd_clk)
    rq[j] <= ram[rd_addr[AW-1:2]];
end

always_ff @(posedge rd_clk) begin
  rsel    <= rd_addr[1:0];
  rd_data <= rq[rsel];
end

endmodule
