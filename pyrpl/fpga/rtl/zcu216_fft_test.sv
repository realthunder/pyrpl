/**
 * ZCU216 FFT bring-up block (register slot 1): NCH channels of
 *   tone generator -> zcu216_fft_dit8 (N = 2^(SUB+3), real input) ->
 *   zcu216_fft_mag (|X|) -> zcu216_fft_reorder (natural order) -> zcu216_fft_spec
 * on fclk (8 samples per clock). Each channel's source is its tone generator or
 * one of the four RF-ADC channels of tile 224 (src register). A third tone
 * generator drives the four RF-DAC channels of tile 228, for the XM655 loopback.
 * Magnitudes are mag = K|X|/2, K = 1.64676 (CORDIC gain). RF samples are 16 bit
 * with the 14 converter bits at the top.
 *
 * Register map (byte offsets in the slot; RO = read only):
 *   0x00  RO  {16'hf0f6, log2 N, NCH}
 *   0x04      ctrl: [0] run (tone generators and FFTs)
 *   0x08  RO  status: [0] all xfft cores configured, [1] fclk locked,
 *                     [8+c] window of channel c captured (since the last arm)
 *   0x0c      min_bin: peak search ignores bins below it
 *   0x10      win_start: first bin of the captured window (multiple of 4)
 *   0x18  W   any write: capture the next whole frame into the windows (all channels)
 *   0x1c  W   any write: clear the sticky xfft event bits
 *   0x20  RO  window length WIN (bins)
 *   0x24      CLK104 SPI readback select (two pins, the chip id for SDO)
 *   0x200 +   DAC tone generator: +0x00 inc_a  +0x04 amp_a  +0x08 inc_b
 *             +0x0c amp_b  +0x10 noise bits  +0x14 run
 *   0x40 + 0x40*c, channel c:
 *     +0x00 inc_a   +0x04 amp_a   +0x08 inc_b   +0x0c amp_b   +0x10 noise bits (0-18)
 *     +0x14 source: 0 = tone generator, 1-4 = RF-ADC tile 224 channel 0-3
 *     +0x20 RO frames      +0x24 RO peak bin      +0x28 RO peak mag
 *     +0x30 RO sticky xfft events: [0] data_in halt, [1] tlast unexpected,
 *                                  [2] tlast missing
 *   0x10000 * (c+1) + 4*i: RO mag of bin win_start + i, channel c
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
  input  logic        flocked,
  // RF-ADC tile 224 and RF-DAC tile 228 streams (fclk), 8 x 16-bit samples
  input  logic [127:0] adc_data [4],
  input  logic [3:0]   adc_valid,
  output logic [127:0] dac_data,
  output logic         dac_valid,
  // CLK104 SPI readback select
  output logic [1:0]   spi_sdo_sel
);

localparam int NB = SUB + 3;
localparam int BW = SUB + 2;
localparam int XW = 27 + 5;
localparam int AW = $clog2(WIN);

////////////////////////////////////////////////////////////////////////////////
// registers (sys_clk)
////////////////////////////////////////////////////////////////////////////////

logic          run;
logic [BW-1:0] min_bin, win_start;
logic          arm, clr;
logic [31:0]   inc_a [NCH], inc_b [NCH];
logic [15:0]   amp_a [NCH], amp_b [NCH];
logic [4:0]    nbits [NCH];
logic [2:0]    src [NCH];
logic [31:0]   d_inc_a, d_inc_b;
logic [15:0]   d_amp_a, d_amp_b;
logic [4:0]    d_nbits;
logic          d_run;

// results in sys_clk
logic [31:0]   s_frames [NCH];
logic [BW-1:0] s_peak_bin [NCH];
logic [31:0]   s_peak_mag [NCH];
logic [2:0]    s_events [NCH];
logic [NCH-1:0] s_win_ready;
logic          s_cfg_done, s_locked;

wire [19:0] a = sys_addr[19:0];

always_ff @(posedge sys_clk)
  if (!sys_rstn) begin
    run       <= 1'b0;
    min_bin   <= BW'(16);
    win_start <= '0;
    arm       <= 1'b0;
    clr       <= 1'b0;
    spi_sdo_sel <= '0;
    d_inc_a   <= '0;  d_amp_a <= '0;
    d_inc_b   <= '0;  d_amp_b <= '0;
    d_nbits   <= '0;  d_run   <= 1'b0;
    for (int c = 0; c < NCH; c++) begin
      inc_a[c] <= '0;  amp_a[c] <= '0;
      inc_b[c] <= '0;  amp_b[c] <= '0;
      nbits[c] <= '0;  src[c]   <= '0;
    end
  end else if (sys_wen) begin
    case (a)
      20'h00004: run       <= sys_wdata[0];
      20'h0000c: min_bin   <= sys_wdata[BW-1:0];
      20'h00010: win_start <= {sys_wdata[BW-1:2], 2'b00};
      20'h00018: arm       <= !arm;
      20'h0001c: clr       <= !clr;
      20'h00024: spi_sdo_sel <= sys_wdata[1:0];
      20'h00200: d_inc_a   <= sys_wdata;
      20'h00204: d_amp_a   <= sys_wdata[15:0];
      20'h00208: d_inc_b   <= sys_wdata;
      20'h0020c: d_amp_b   <= sys_wdata[15:0];
      20'h00210: d_nbits   <= (sys_wdata[4:0] > 5'd18) ? 5'd18 : sys_wdata[4:0];
      20'h00214: d_run     <= sys_wdata[0];
      default: ;
    endcase
    for (int c = 0; c < NCH; c++)
      case (a)
        20'h00040 + 20'h40*c: inc_a[c] <= sys_wdata;
        20'h00044 + 20'h40*c: amp_a[c] <= sys_wdata[15:0];
        20'h00048 + 20'h40*c: inc_b[c] <= sys_wdata;
        20'h0004c + 20'h40*c: amp_b[c] <= sys_wdata[15:0];
        20'h00050 + 20'h40*c: nbits[c] <= (sys_wdata[4:0] > 5'd18) ? 5'd18 : sys_wdata[4:0];
        20'h00054 + 20'h40*c: src[c]   <= (sys_wdata[2:0] > 3'd4) ? 3'd0 : sys_wdata[2:0];
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
    20'h00020: v = 32'(WIN);
    20'h00024: v = 32'(spi_sdo_sel);
    20'h00200: v = d_inc_a;
    20'h00204: v = 32'(d_amp_a);
    20'h00208: v = d_inc_b;
    20'h0020c: v = 32'(d_amp_b);
    20'h00210: v = 32'(d_nbits);
    20'h00214: v = 32'(d_run);
    default: ;
  endcase
  for (int c = 0; c < NCH; c++)
    case (ad)
      20'h00040 + 20'h40*c: v = inc_a[c];
      20'h00044 + 20'h40*c: v = 32'(amp_a[c]);
      20'h00048 + 20'h40*c: v = inc_b[c];
      20'h0004c + 20'h40*c: v = 32'(amp_b[c]);
      20'h00050 + 20'h40*c: v = 32'(nbits[c]);
      20'h00054 + 20'h40*c: v = 32'(src[c]);
      20'h00060 + 20'h40*c: v = s_frames[c];
      20'h00064 + 20'h40*c: v = 32'(s_peak_bin[c]);
      20'h00068 + 20'h40*c: v = s_peak_mag[c];
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

always_ff @(posedge fclk) begin
  {f_run, f_run_m} <= {f_run_m, run};
  {f_clr, f_clr_m} <= {f_clr_m, clr};
  {f_arm, f_arm_m} <= {f_arm_m, arm};
  {f_min_bin,   f_min_bin_m}   <= {f_min_bin_m,   min_bin};
  {f_win_start, f_win_start_m} <= {f_win_start_m, win_start};
end

////////////////////////////////////////////////////////////////////////////////
// channels
////////////////////////////////////////////////////////////////////////////////

logic [NCH-1:0] f_cfg_done;
logic           f_cfg_all;

assign f_cfg_all = &f_cfg_done;

// RF-DAC tone generator: all four channels of tile 228 play the same samples
logic [31:0] fd_inc_a, fd_inc_b, fd_inc_a_m, fd_inc_b_m;
logic [15:0] fd_amp_a, fd_amp_b, fd_amp_a_m, fd_amp_b_m;
logic [4:0]  fd_nbits, fd_nbits_m;
(* ASYNC_REG = "TRUE" *) logic fd_run_m, fd_run;
logic [8*16-1:0] d_data;
logic            d_valid;

always_ff @(posedge fclk) begin
  {fd_inc_a, fd_inc_a_m} <= {fd_inc_a_m, d_inc_a};
  {fd_inc_b, fd_inc_b_m} <= {fd_inc_b_m, d_inc_b};
  {fd_amp_a, fd_amp_a_m} <= {fd_amp_a_m, d_amp_a};
  {fd_amp_b, fd_amp_b_m} <= {fd_amp_b_m, d_amp_b};
  {fd_nbits, fd_nbits_m} <= {fd_nbits_m, d_nbits};
  {fd_run,   fd_run_m}   <= {fd_run_m,   d_run};
end

zcu216_tone_gen #(.IN_W(14), .SIN_FILE(SIN_FILE)) i_dac_gen (
  .clk (fclk), .rstn (frstn), .run (fd_run),
  .inc_a (fd_inc_a), .amp_a (fd_amp_a), .inc_b (fd_inc_b), .amp_b (fd_amp_b),
  .nbits (fd_nbits), .data (d_data), .valid (d_valid));

// 14-bit samples to the top of the DAC's 16-bit words
always_ff @(posedge fclk) begin
  for (int j = 0; j < 8; j++)
    dac_data[16*j +: 16] <= d_valid ? {d_data[16*j +: 14], 2'b00} : 16'h0;
  dac_valid <= 1'b1;
end

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

  logic [8*16-1:0]      g_data, s_data;
  logic                 g_valid, s_valid;
  logic [2:0]           f_src, f_src_m;

  always_ff @(posedge fclk) {f_src, f_src_m} <= {f_src_m, src[c]};

  // source: the tone generator, or an RF-ADC channel (14 bits at the top of 16)
  always_ff @(posedge fclk)
    if (f_src == 0) begin
      s_data  <= g_data;
      s_valid <= g_valid;
    end else begin
      for (int j = 0; j < 8; j++)
        s_data[16*j +: 16] <= 16'($signed(adc_data[f_src - 1][16*j +: 16]) >>> 2);
      s_valid <= adc_valid[f_src - 1];
    end
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
    .in_data (s_data), .in_valid (s_valid), .run (f_run && f_cfg_all),
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

  logic [31:0]    m_mag [4], n_mag [4];
  logic [SUB-1:0] m_k, n_t;
  logic           m_valid, m_last, n_valid, n_first, n_last;

  zcu216_fft_mag #(.XW(XW), .SUB(SUB)) i_mag (
    .clk (fclk), .x_re, .x_im, .x_k, .x_valid, .x_last,
    .mag (m_mag), .m_k, .m_valid, .m_last);

  zcu216_fft_reorder #(.SUB(SUB)) i_reorder (
    .clk (fclk), .rstn (frstn), .m_mag, .m_k, .m_valid, .m_last,
    .n_mag, .n_t, .n_valid, .n_first, .n_last);

  logic          f_upd, f_done;
  logic [31:0]   f_frames;
  logic [BW-1:0] f_peak_bin;
  logic [31:0]   f_peak_mag;

  zcu216_fft_spec #(.SUB(SUB), .WIN(WIN)) i_spec (
    .clk (fclk), .rstn (frstn),
    .n_mag, .n_t, .n_valid, .n_first, .n_last,
    .min_bin (f_min_bin), .win_start (f_win_start),
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
