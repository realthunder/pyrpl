/**
 * Bit-reversed -> natural order for the zcu216_fft_dit8 bins (after |X|).
 *
 * In: four bins per clock, k + q*2^SUB on lane q, k in the sub-FFTs' bit-reversed
 * order. Out, one frame later: four consecutive bins per clock, 4t + j on lane j,
 * t = 0 .. 2^SUB-1, the layout CFAR and peak interpolation need (neighbours are
 * adjacent lanes and beats).
 *
 * Four banks with a skewed assignment, bank(b) = (b + (b >> SUB)) mod 4 and
 * address b >> 2: the four bins written together (2^SUB apart) and the four read
 * together (consecutive) each land in four different banks, so neither side
 * stalls. Two pages: one frame is read while the next is written.
 *
 * Each bank is 2 pages x 2^SUB entries of 32 bits, packed two per 64-bit word
 * (byte write enables) so it maps onto UltraRAM: 2^SUB words of 64 bits, 8
 * URAM288 per bank at SUB = 15.
 */

module zcu216_fft_reorder #(
  parameter int  SUB = 15,
  localparam int AW  = SUB + 1                // entry address: {page, b >> 2}
) (
  input  logic              clk,
  input  logic              rstn,
  input  logic [31:0]       m_mag [4],
  input  logic [SUB-1:0]    m_k,
  input  logic              m_valid,
  input  logic              m_last,
  output logic [31:0]       n_mag [4],        // bin 4 * n_t + j on lane j
  output logic [SUB-1:0]    n_t,
  output logic              n_valid,
  output logic              n_first,
  output logic              n_last
);

////////////////////////////////////////////////////////////////////////////////
// write side
////////////////////////////////////////////////////////////////////////////////

logic          wpage;                         // page being written
logic          rd_go;                         // a page has been completed
logic [3:0]    we;
logic [AW-1:0] waddr [4];
logic [31:0]   wdata [4];

always_ff @(posedge clk)
  if (!rstn) begin
    wpage <= 1'b0;
    rd_go <= 1'b0;
  end else begin
    rd_go <= m_valid && m_last;
    if (m_valid && m_last) wpage <= !wpage;
  end

// bank B takes lane q = (B - k) mod 4: bin k + q*2^SUB, address (k >> 2) + q*2^(SUB-2).
// (Plain wires for q: an automatic variable used as the m_mag index here was
// evaluated as the loop index by xsim.)
logic [1:0]  wq [4];
logic [31:0] wmux [4];

always_comb
  for (int b = 0; b < 4; b++) begin
    wq[b]   = 2'(b) - m_k[1:0];
    wmux[b] = m_mag[wq[b]];
  end

always_ff @(posedge clk)
  for (int b = 0; b < 4; b++) begin
    we[b]    <= m_valid;
    waddr[b] <= {wpage, wq[b], m_k[SUB-1:2]};
    wdata[b] <= wmux[b];
  end

////////////////////////////////////////////////////////////////////////////////
// read side: one page per completed frame, beat t reads bins 4t .. 4t+3
////////////////////////////////////////////////////////////////////////////////

logic           reading, rpage;
logic [SUB-1:0] t;

always_ff @(posedge clk)
  if (!rstn) begin
    reading <= 1'b0;
    rpage   <= 1'b0;
    t       <= '0;
  end else begin
    if (rd_go) begin
      reading <= 1'b1;
      rpage   <= !wpage;                      // wpage already toggled
      t       <= '0;
    end else if (reading) begin
      t <= t + 1'b1;
      if (&t) reading <= 1'b0;
    end
  end

// bin 4t + j is in bank (j + q_t) mod 4, q_t = t >> (SUB-2), at address t
logic [AW-1:0] raddr;
assign raddr = {rpage, t};

localparam int RL = 3;                        // RAM read latency (address, RAM, output reg)
logic [31:0]    rdata [4];
logic [1:0]     q_d [RL];
logic [SUB-1:0] t_d [RL];
logic [RL-1:0]  v_d = '0, f_d = '0, l_d = '0;

always_ff @(posedge clk) begin
  q_d[0] <= t[SUB-1 -: 2];
  t_d[0] <= t;
  for (int i = 1; i < RL; i++) begin
    q_d[i] <= q_d[i-1];
    t_d[i] <= t_d[i-1];
  end
  v_d <= {v_d[RL-2:0], reading};
  f_d <= {f_d[RL-2:0], reading && t == '0};
  l_d <= {l_d[RL-2:0], reading && &t};
end

for (genvar b = 0; b < 4; b++) begin : g_bank
  zcu216_reorder_ram #(.AW(AW)) i_ram (
    .clk, .we (we[b]), .waddr (waddr[b]), .wdata (wdata[b]),
    .raddr, .rdata (rdata[b]));
end

always_ff @(posedge clk) begin
  for (int j = 0; j < 4; j++)
    n_mag[j] <= rdata[2'(j) + q_d[RL-1]];
  n_t     <= t_d[RL-1];
  n_valid <= v_d[RL-1];
  n_first <= f_d[RL-1];
  n_last  <= l_d[RL-1];
end

endmodule


/**
 * 2^AW x 32-bit RAM, one write and one read port, stored as 2^(AW-1) x 64 bit
 * with byte write enables so it maps onto UltraRAM. Read latency 3 from raddr.
 */
module zcu216_reorder_ram #(
  parameter int AW = 16
) (
  input  logic          clk,
  input  logic          we,
  input  logic [AW-1:0] waddr,
  input  logic [31:0]   wdata,
  input  logic [AW-1:0] raddr,
  output logic [31:0]   rdata
);

(* ram_style = "ultra" *) logic [63:0] mem [0:(1<<(AW-1))-1];

// canonical byte-write template: entry waddr goes into half waddr[0] of word
// waddr >> 1
wire [7:0]  be  = we ? (waddr[0] ? 8'hf0 : 8'h0f) : 8'h00;
wire [63:0] wd  = {wdata, wdata};

always_ff @(posedge clk)
  for (int i = 0; i < 8; i++)
    if (be[i]) mem[waddr[AW-1:1]][8*i +: 8] <= wd[8*i +: 8];

logic [AW-1:0] ra;
logic          half0, half1;
logic [63:0]   q0, q1;

always_ff @(posedge clk) begin
  ra    <= raddr;
  q0    <= mem[ra[AW-1:1]];
  half0 <= ra[0];
  q1    <= q0;
  half1 <= half0;
end

assign rdata = half1 ? q1[63:32] : q1[31:0];

endmodule
