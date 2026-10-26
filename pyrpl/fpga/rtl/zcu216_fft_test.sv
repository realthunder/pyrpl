/**
 * ZCU216 FFT bring-up block (register slot 1): NCH channels of
 * tone generator -> zcu216_fft_dit8 (N = 2^(SUB+3), real input) -> zcu216_fft_spec,
 * on fclk (8 samples per clock). The tone generator stands in for the RF-ADC.
 *
 * Register map (byte offsets in the slot; RO = read only):
 *   0x00  RO  {16'hf0f6, log2 N, NCH}
 *   0x04      ctrl: [0] run (tone generators and FFTs)
 *   0x08  RO  status: [0] all xfft cores configured, [1] fclk locked,
 *                     [8+c] window of channel c captured (since the last arm)
 *   0x0c      min_bin: peak search ignores bins below it
 *   0x10      win_start: first bin of the captured window
 *   0x14      mag_shift: window words are min(|X|^2 >> mag_shift, 2^32-1)
 *   0x18  W   any write: capture the next whole frame into the windows (all channels)
 *   0x1c  W   any write: clear the sticky xfft event bits
 *   0x20  RO  window length WIN (bins)
 *   0x40 + 0x40*c, channel c:
 *     +0x00 inc_a   +0x04 amp_a   +0x08 inc_b   +0x0c amp_b   +0x10 noise bits (0-18)
 *     +0x20 RO frames      +0x24 RO peak bin      +0x28/+0x2c RO peak |X|^2 lo/hi
 *     +0x30 RO sticky xfft events: [0] data_in halt, [1] tlast unexpected,
 *                                  [2] tlast missing
 *   0x10000 * (c+1) + 4*i: RO window word i of channel c
 *
 * Settings cross into fclk as quasi-static values; results cross back on a
 * per-frame toggle, after which they hold for a whole frame.
 */

module zcu216_fft_test #(
  parameter int    NCH       = 2,
  parameter int    SUB       = 15,
  parameter int    WIN       = 4096,
  parameter string TWC_FILE  = "",
  parameter string TWF_FILE  = "",
  parameter string SIN_FILE  = ""
) (
  // register bus (sys_clk)
  input  logic        sys_clk,
  input  logic        sys_rstn,
  input  logic [31:0] sys_addr,
  input  logic [31:0] sys_wdata,
  input  logic        sys_wen,
  input  logic        sys_ren,
  output logic [31:0] sys_rdata,
  output logic        sys_err,
  output logic        sys_ack,
  // FFT clock
  input  logic        fclk,
  input  logic        frstn,
  input  logic        flocked
);

localparam int NB = SUB + 3;
localparam int BW = SUB + 2;
localparam int XW = 27 + 5;
localparam int MW = 2 * XW - 1;
localparam int AW = $clog2(WIN);

////////////////////////////////////////////////////////////////////////////////
// registers (sys_clk)
////////////////////////////////////////////////////////////////////////////////

logic          run;
logic [BW-1:0] min_bin, win_start;
logic [5:0]    mag_shift;
logic          arm, clr;
logic [31:0]   inc_a [NCH], inc_b [NCH];
logic [15:0]   amp_a [NCH], amp_b [NCH];
logic [4:0]    nbits [NCH];

// results in sys_clk
logic [31:0]   s_frames [NCH];
logic [BW-1:0] s_peak_bin [NCH];
logic [MW-1:0] s_peak_mag [NCH];
logic [2:0]    s_events [NCH];
logic [NCH-1:0] s_win_ready;
logic          s_cfg_done, s_locked;

wire [19:0] a = sys_addr[19:0];

always_ff @(posedge sys_clk)
  if (!sys_rstn) begin
    run       <= 1'b0;
    min_bin   <= BW'(16);
    win_start <= '0;
    mag_shift <= '0;
    arm       <= 1'b0;
    clr       <= 1'b0;
    for (int c = 0; c < NCH; c++) begin
      inc_a[c] <= '0;  amp_a[c] <= '0;
      inc_b[c] <= '0;  amp_b[c] <= '0;
      nbits[c] <= '0;
    end
  end else if (sys_wen) begin
    case (a)
      20'h00004: run       <= sys_wdata[0];
      20'h0000c: min_bin   <= sys_wdata[BW-1:0];
      20'h00010: win_start <= sys_wdata[BW-1:0];
      20'h00014: mag_shift <= sys_wdata[5:0];
      20'h00018: arm       <= !arm;
      20'h0001c: clr       <= !clr;
      default: ;
    endcase
    for (int c = 0; c < NCH; c++)
      case (a)
        20'h00040 + 20'h40*c: inc_a[c] <= sys_wdata;
        20'h00044 + 20'h40*c: amp_a[c] <= sys_wdata[15:0];
        20'h00048 + 20'h40*c: inc_b[c] <= sys_wdata;
        20'h0004c + 20'h40*c: amp_b[c] <= sys_wdata[15:0];
        20'h00050 + 20'h40*c: nbits[c] <= (sys_wdata[4:0] > 5'd18) ? 5'd18 : sys_wdata[4:0];
        default: ;
      endcase
  end

// reads: registers answer after 1 cycle, the window RAMs after 3
logic [AW-1:0] rd_addr;
logic [31:0]   rd_data [NCH];
logic [2:0]    ram_rd;
logic [3:0]    ram_ch;          // held: axi_slave runs one transaction at a time

always_ff @(posedge sys_clk) begin
  sys_err <= 1'b0;
  sys_ack <= 1'b0;
  ram_rd  <= {ram_rd[1:0], 1'b0};
  if (!sys_rstn) begin
    ram_rd <= '0;
  end else if (sys_wen) begin
    sys_ack <= 1'b1;
  end else if (sys_ren) begin
    if (a[19:16] != 0) begin                        // window RAM
      ram_ch  <= a[19:16] - 1'b1;
      rd_addr <= a[AW+1:2];
      ram_rd  <= 3'b001;
    end else begin
      sys_ack   <= 1'b1;
      sys_rdata <= reg_read(a);
    end
  end
  if (ram_rd[2]) begin
    sys_ack   <= 1'b1;
    sys_rdata <= (ram_ch < NCH) ? rd_data[ram_ch] : 32'h0;
  end
end

function automatic logic [31:0] reg_read(input logic [19:0] ad);
  logic [31:0] v = 32'h0;
  case (ad)
    20'h00000: v = {16'hf0f6, 8'(NB), 8'(NCH)};
    20'h00004: v = {31'h0, run};
    20'h00008: v = {{(24-NCH){1'b0}}, s_win_ready, 6'h0, s_locked, s_cfg_done};
    20'h0000c: v = 32'(min_bin);
    20'h00010: v = 32'(win_start);
    20'h00014: v = 32'(mag_shift);
    20'h00020: v = 32'(WIN);
    default: ;
  endcase
  for (int c = 0; c < NCH; c++)
    case (ad)
      20'h00040 + 20'h40*c: v = inc_a[c];
      20'h00044 + 20'h40*c: v = 32'(amp_a[c]);
      20'h00048 + 20'h40*c: v = inc_b[c];
      20'h0004c + 20'h40*c: v = 32'(amp_b[c]);
      20'h00050 + 20'h40*c: v = 32'(nbits[c]);
      20'h00060 + 20'h40*c: v = s_frames[c];
      20'h00064 + 20'h40*c: v = 32'(s_peak_bin[c]);
      20'h00068 + 20'h40*c: v = s_peak_mag[c][31:0];
      20'h0006c + 20'h40*c: v = 32'(s_peak_mag[c][MW-1:32]);
      20'h00070 + 20'h40*c: v = 32'(s_events[c]);
      default: ;
    endcase
  return v;
endfunction

////////////////////////////////////////////////////////////////////////////////
// settings into fclk (quasi-static, two flops)
////////////////////////////////////////////////////////////////////////////////

(* ASYNC_REG = "TRUE" *) logic f_run_m, f_run, f_clr_m, f_clr;
(* ASYNC_REG = "TRUE" *) logic f_arm_m, f_arm;
logic [BW-1:0] f_min_bin_m, f_min_bin, f_win_start_m, f_win_start;
logic [5:0]    f_mag_shift_m, f_mag_shift;

always_ff @(posedge fclk) begin
  {f_run, f_run_m} <= {f_run_m, run};
  {f_clr, f_clr_m} <= {f_clr_m, clr};
  {f_arm, f_arm_m} <= {f_arm_m, arm};
  {f_min_bin,   f_min_bin_m}   <= {f_min_bin_m,   min_bin};
  {f_win_start, f_win_start_m} <= {f_win_start_m, win_start};
  {f_mag_shift, f_mag_shift_m} <= {f_mag_shift_m, mag_shift};
end

////////////////////////////////////////////////////////////////////////////////
// channels
////////////////////////////////////////////////////////////////////////////////

logic [NCH-1:0] f_cfg_done;
logic           f_cfg_all;

assign f_cfg_all = &f_cfg_done;

for (genvar c = 0; c < NCH; c++) begin : g_ch
  logic [31:0] f_inc_a, f_inc_b, f_inc_a_m, f_inc_b_m;
  logic [15:0] f_amp_a, f_amp_b, f_amp_a_m, f_amp_b_m;
  logic [4:0]  f_nbits, f_nbits_m;

  always_ff @(posedge fclk) begin
    {f_inc_a, f_inc_a_m} <= {f_inc_a_m, inc_a[c]};
    {f_inc_b, f_inc_b_m} <= {f_inc_b_m, inc_b[c]};
    {f_amp_a, f_amp_a_m} <= {f_amp_a_m, amp_a[c]};
    {f_amp_b, f_amp_b_m} <= {f_amp_b_m, amp_b[c]};
    {f_nbits, f_nbits_m} <= {f_nbits_m, nbits[c]};
  end

  logic [8*16-1:0]      g_data;
  logic                 g_valid;
  logic signed [XW-1:0] x_re [4], x_im [4];
  logic [SUB-1:0]       x_k;
  logic                 x_valid, x_last;
  logic                 ev_halt, ev_unexp, ev_miss;

  zcu216_tone_gen #(.IN_W(14), .SIN_FILE(SIN_FILE)) i_gen (
    .clk (fclk), .rstn (frstn), .run (f_run),
    .inc_a (f_inc_a), .amp_a (f_amp_a), .inc_b (f_inc_b), .amp_b (f_amp_b),
    .nbits (f_nbits), .data (g_data), .valid (g_valid));

  zcu216_fft_dit8 #(.SUB(SUB), .IN_W(14), .DW(27), .TW(18),
                    .TWC_FILE(TWC_FILE), .TWF_FILE(TWF_FILE)) i_fft (
    .clk (fclk), .rstn (frstn), .cfg_done (f_cfg_done[c]),
    .in_data (g_data), .in_valid (g_valid), .run (f_run && f_cfg_all),
    .in_accept (), .in_first (),
    .x_re, .x_im, .x_k, .x_valid, .x_last,
    .ev_halt, .ev_tlast_unexpected (ev_unexp), .ev_tlast_missing (ev_miss));

  // sticky events, cleared by a clr toggle
  logic [2:0] f_events;
  logic       f_clr_d;
  always_ff @(posedge fclk) begin
    f_clr_d <= f_clr;
    if (!frstn || f_clr != f_clr_d) f_events <= '0;
    else f_events <= f_events | {ev_miss, ev_unexp, ev_halt};
  end

  logic          f_upd, f_done;
  logic [31:0]   f_frames;
  logic [BW-1:0] f_peak_bin;
  logic [MW-1:0] f_peak_mag;

  zcu216_fft_spec #(.SUB(SUB), .XW(XW), .WIN(WIN)) i_spec (
    .clk (fclk), .rstn (frstn),
    .x_re, .x_im, .x_k, .x_valid, .x_last,
    .min_bin (f_min_bin), .win_start (f_win_start), .mag_shift (f_mag_shift),
    .arm (f_arm),
    .upd (f_upd), .frames (f_frames), .peak_bin (f_peak_bin), .peak_mag (f_peak_mag),
    .done (f_done),
    .rd_clk (sys_clk), .rd_addr (rd_addr), .rd_data (rd_data[c]));

  // results into sys_clk: copy when the per-frame toggle moves
  (* ASYNC_REG = "TRUE" *) logic [2:0] upd_s, done_s;
  logic [2:0] ev_m, ev_s;
  always_ff @(posedge sys_clk) begin
    upd_s  <= {upd_s[1:0], f_upd};
    done_s <= {done_s[1:0], f_done};
    {ev_s, ev_m} <= {ev_m, f_events};
    if (upd_s[2] != upd_s[1]) begin
      s_frames[c]   <= f_frames;
      s_peak_bin[c] <= f_peak_bin;
      s_peak_mag[c] <= f_peak_mag;
    end
    s_events[c]    <= ev_s;
    s_win_ready[c] <= (done_s[1] == arm);
  end
end

(* ASYNC_REG = "TRUE" *) logic [1:0] cfg_s, lock_s;
always_ff @(posedge sys_clk) begin
  cfg_s  <= {cfg_s[0], f_cfg_all};
  lock_s <= {lock_s[0], flocked};
end
assign s_cfg_done = cfg_s[1];
assign s_locked   = lock_s[1];

endmodule
