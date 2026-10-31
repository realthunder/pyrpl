/**
 * W bits from fclk to sys_clk at one sample per sys_clk (250 MS/s, zcu216_adc_dec,
 * zcu216_rx), written 4 times per 5 fclk: a 16-entry ring in distributed RAM. The
 * write pointer, Gray coded, moves once per write through two flops (a set_bus_skew
 * in zcu216_vivado.tcl keeps its bits within one fclk). The clocks come from one
 * MMCM, so the reader runs freely at the writer's rate, DIST entries behind the
 * pointer it sees, and only realigns when that distance leaves GAP_MIN to GAP_MAX
 * (at the start, after the MMCM relocks). The pointer it sees moves unevenly (0-2
 * per sys_clk, the 4:5 pattern of the two clocks and the synchroniser), which the
 * window must allow. A held pointer holds the output. No reset, so it needs neither
 * clock running first.
 *
 * Latency: about 7 sys_clk.
 */
module zcu216_ring #(
  parameter int W = 80
) (
  input  logic         fclk,
  input  logic         we,
  input  logic [W-1:0] din,
  input  logic         sys_clk,
  output logic [W-1:0] q = '0
);

localparam logic [3:0] DIST = 4'd3, GAP_MIN = 4'd1, GAP_MAX = 4'd5;

(* ram_style = "distributed" *) logic [W-1:0] ring [16];
logic [3:0] wp = '0, wg = '0;

always_ff @(posedge fclk)
  if (we) begin
    ring[wp] <= din;
    wp <= wp + 1'b1;
    wg <= (wp + 1'b1) ^ ((wp + 1'b1) >> 1);
  end

(* ASYNC_REG = "TRUE" *) logic [3:0] wg_m = '0, wg_s = '0;
logic [3:0] wb = '0, rp = '0, gap;
assign gap = wb - rp;

always_ff @(posedge sys_clk) begin
  wg_m <= wg;
  wg_s <= wg_m;
  wb   <= {wg_s[3], ^wg_s[3:2], ^wg_s[3:1], ^wg_s[3:0]};
  rp   <= (gap >= GAP_MIN && gap <= GAP_MAX) ? rp + 1'b1 : wb - DIST;
  q    <= ring[rp];
end

endmodule
