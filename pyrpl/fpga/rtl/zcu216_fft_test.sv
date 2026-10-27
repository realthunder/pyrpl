/**
 * ZCU216 FFT bring-up block (register slot 1): NCH channels of
 *   tone generator -> zcu216_fft_dit8 (N = 2^(SUB+3), real input) ->
 *   zcu216_fft_mag (|X|) -> zcu216_fft_reorder (natural order) -> zcu216_fft_spec
 *                                                              -> zcu216_cfar
 * on fclk (8 samples per clock), zcu216_snap, which writes whole frames of
 * |X| or of an FFT's input samples into DDR on request, and zcu216_points,
 * which streams every frame's CFAR results into the DDR ring. Each channel's source is its tone generator or
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
 *   0x28  W   any write: capture the next RAW beats of the raw ADC source and
 *             of the DAC stream (status [16] = done since the last arm)
 *   0x2c      raw capture ADC source: RF-ADC tile 224 channel 0-3
 *   0x300 +   RO clock meters, kHz (1 ms gate on the bus clock): +0x0 + 4*i
 *             meas_clk[i] (clk_adc0, CLK104 PL_CLK, clk_dac0, clk_dac2), +0x10 fclk
 *   0x200 +   DAC tone generator: +0x00 inc_a  +0x04 amp_a  +0x08 inc_b
 *             +0x0c amp_b  +0x10 noise bits  +0x14 run
 *   0x100 +   CFAR detector settings (all channels; hls/peak_detector_cfar.cpp
 *             semantics, taken at each frame start):
 *             +0x00 k^2 (16 bits)  +0x04 start_index  +0x08 end_index (bins,
 *             inclusive)  +0x0c data_min  +0x10 guard cells (<= 16)
 *             +0x14 training cells per side (<= 64)  +0x18 [0] one-sided edge
 *             fallback, [1] SO-CFAR  +0x1c ramp_d0  +0x20 ramp_step (Q3.20)
 *             +0x24 RO {FRAC, TMAX, GMAX} = [19:16], [15:8], [7:0]
 *   0x140 +   snapshot of whole frames into DDR (zcu216_snap):
 *             +0x00 W  arm: capture from the next frame start (ignored while a
 *                      capture is pending; bit 31 set only drops the pending flag)
 *             +0x04    source: [1:0] channel, [4] 0 = |X| (4 x 32-bit bins per
 *                      beat), 1 = the FFT's input (8 x 16-bit samples)
 *             +0x08    frames to capture (0 = 1, clamped to +0x1c)
 *             +0x0c RO [0] pending (armed, not done yet), [1] busy, [2] overflow
 *                      (beats lost: DDR too slow), [3] AXI error, [15:8] captures done
 *             +0x10 RO beats captured
 *             +0x14 RO physical base   +0x18 RO bytes per frame   +0x1c RO max frames
 *   0x180 +   point stream into the DDR ring (zcu216_points; monitor_server
 *             forwards it over UDP, build it with -DDMA_REG_BASE=0x80200180):
 *             +0x00 RO write pointer (64-bit words into the ring)
 *             +0x04 RO data words per packet   +0x08 RO {version, NCH, frames per packet}
 *             = [31:24], [23:16], [15:0]   +0x0c [0] enable   +0x10 RO packets
 *             +0x14 RO words dropped   +0x18 RO [0] AXI error   +0x1c RO ring words
 *   0x40 + 0x40*c, channel c:
 *     +0x00 inc_a   +0x04 amp_a   +0x08 inc_b   +0x0c amp_b   +0x10 noise bits (0-18)
 *     +0x14 source: 0 = tone generator, 1-4 = RF-ADC tile 224 channel 0-3
 *     +0x18 RO CFAR result [31:0] = raw magnitude (reading it latches +0x1c)
 *     +0x1c RO CFAR result [63:32]: [0] detected, [IW:1] k_interp = bin * 256 + offset
 *     +0x20 RO frames      +0x24 RO peak bin      +0x28 RO peak mag
 *     +0x2c RO CFAR results   +0x34 RO CFAR detections   +0x38 RO CFAR overruns
 *     +0x30 RO sticky xfft events: [0] data_in halt, [1] tlast unexpected,
 *                                  [2] tlast missing
 *   0x10000 * (c+1) + 4*i: RO mag of bin win_start + i, channel c
 *   0x100000 +  oscilloscope (zcu216_scope), the Red Pitaya scope's registers
 *             (pyrpl Scope) with the trace in DDR:
 *             +0x00 W [0] arm, [1] reset, [3] keep writing after the trigger;
 *                   R [0] writing (until the trace is in DDR), [2] delay running, [3] keep
 *             +0x04 trigger source: 0 off, 1 now (software), 2/3 channel 1 up/down,
 *                   4/5 channel 2 up/down; back to 0 when the trigger and delay are done
 *             +0x08 threshold, +0x20 hysteresis (14-bit signed, of the 14-bit samples)
 *             +0x10 samples after the trigger   +0x14 decimation (2 .. 65536, powers of 2)
 *             +0x18 RO last sample in DDR   +0x1c RO sample before the trigger (ring indices)
 *             +0x28 [0] average   +0x16c RO [0] pre-trigger samples in
 *             +0x1e0 RO trace length L (samples)   +0x1e4 RO ring base (physical)
 *             +0x1e8 RO ring length (samples)   +0x1ec RO {bits, channels, samples per
 *                   channel per 16-byte word} = [23:16], [15:8], [7:0]
 *             +0x1f0 / +0x1f4 input of channel 1 / 2: 0-3 RF-ADC tile 224 channel 0-3,
 *                   4 + c the input of FFT channel c   +0x1f8 RO minimum decimation
 *             +0x1fc RO sample period (ps) before decimation
 *   0x80000 + 4*i: RO raw ADC capture, beat i/4, 32-bit part i%4 (samples 2(i%4), +1)
 *   0x90000 + 4*i: RO raw DAC capture, same layout
 *
 * Settings cross into fclk as quasi-static values; results cross back on a
 * per-frame toggle, after which they hold for a whole frame.
 */

module zcu216_fft_test #(
  parameter int    NCH       = 2,
  parameter int    SUB       = 15,
  parameter int    WIN       = 4096,
  parameter logic [39:0] SNAP_BASE = 40'h1E10_0000,  // reserved DDR (31 MB)
  parameter int    SNAP_SIZE = 29 << 20,                // up to the scope ring
  parameter int    SC_L      = 1 << 18,                  // scope trace length (samples)
  parameter logic [39:0] SC_BASE = 40'h1FE0_0000,         // scope ring: 2 SC_L samples x 2 x 16 bit
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
  output logic [1:0]   spi_sdo_sel,
  // clocks to measure (debug)
  input  logic [3:0]   meas_clk,
  // snapshot DMA: two AXI4 write masters on fclk (S_AXI_HP0/HP1)
  output logic [48:0]  hp_awaddr  [2],
  output logic [7:0]   hp_awlen   [2],
  output logic [2:0]   hp_awsize  [2],
  output logic [1:0]   hp_awburst [2],
  output logic [3:0]   hp_awcache [2],
  output logic [2:0]   hp_awprot  [2],
  output logic [1:0]   hp_awvalid,
  input  logic [1:0]   hp_awready,
  output logic [127:0] hp_wdata   [2],
  output logic [15:0]  hp_wstrb   [2],
  output logic [1:0]   hp_wlast,
  output logic [1:0]   hp_wvalid,
  input  logic [1:0]   hp_wready,
  input  logic [1:0]   hp_bresp   [2],
  input  logic [1:0]   hp_bvalid,
  output logic [1:0]   hp_bready,
  // point stream: one AXI4 write master on fclk (S_AXI_HP2)
  output logic [48:0]  pt_awaddr,
  output logic [7:0]   pt_awlen,
  output logic [2:0]   pt_awsize,
  output logic [1:0]   pt_awburst,
  output logic [3:0]   pt_awcache,
  output logic [2:0]   pt_awprot,
  output logic         pt_awvalid,
  input  logic         pt_awready,
  output logic [127:0] pt_wdata,
  output logic [15:0]  pt_wstrb,
  output logic         pt_wlast,
  output logic         pt_wvalid,
  input  logic         pt_wready,
  input  logic [1:0]   pt_bresp,
  input  logic         pt_bvalid,
  output logic         pt_bready
);

localparam int RAW = 2048;                    // raw capture depth (beats)

localparam int NB = SUB + 3;
localparam int BW = SUB + 2;
localparam int XW = 27 + 5;
localparam int AW = $clog2(WIN);
localparam int GMAX = 16, TMAX = 64, FRAC = 8;
localparam int GW = $clog2(GMAX + 1), TW = $clog2(TMAX + 1);
localparam int SNAP_FB = 16 << SUB;                       // bytes per frame
localparam int SNAP_MAXF = (SNAP_SIZE / SNAP_FB > 255) ? 255 : SNAP_SIZE / SNAP_FB;
localparam int PT_FPP = 183 / NCH;                         // frames per point packet
localparam int PT_RING = 16384;                            // words, at the start of the reserved DDR
localparam logic [3:0] PT_VERSION = 4'd8;

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
logic          rarm;
logic [1:0]    raw_src;
// CFAR settings
logic [15:0]   cf_ksq;
logic [BW:0]   cf_lo, cf_hi;
logic [31:0]   cf_min;
logic [GW-1:0] cf_g;
logic [TW-1:0] cf_t;
logic [1:0]    cf_flags;
logic [22:0]   cf_d0, cf_step;
// snapshot
logic          sn_arm, sn_pending, sn_done_edge;
logic [4:0]    sn_src;
logic [7:0]    sn_nf;
logic [7:0]    s_sn_cnt;
logic          s_sn_busy, s_sn_ovf, s_sn_err;
logic [31:0]   s_sn_beats;
// scope
localparam int SC_RW = $clog2(2 * SC_L);
logic          sc_arm_t, sc_rst_t, sc_src_t, sc_keep, sc_avg;
logic [3:0]    sc_src;
logic [13:0]   sc_thr, sc_hyst;
logic [31:0]   sc_dly;
logic [16:0]   sc_dec;
logic [2:0]    sc_in [2];
logic [63:0]   s_sc_st;                         // status from fclk (see below)
// point stream
logic          pt_en;
logic [13:0]   s_pt_ptr;
logic [31:0]   s_pt_pkts, s_pt_drops;
logic          s_pt_err;

// results in sys_clk
logic [31:0]   s_frames [NCH];
logic [BW-1:0] s_peak_bin [NCH];
logic [31:0]   s_peak_mag [NCH];
logic [2:0]    s_events [NCH];
logic [63:0]   s_cf_word [NCH];
logic [31:0]   s_cf_hi [NCH];                    // high word, latched by a low-word read
logic [31:0]   s_cf_cnt [NCH], s_cf_det [NCH], s_cf_ovr [NCH];
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
    rarm      <= 1'b0;
    raw_src   <= '0;
    d_inc_a   <= '0;  d_amp_a <= '0;
    d_inc_b   <= '0;  d_amp_b <= '0;
    d_nbits   <= '0;  d_run   <= 1'b0;
    cf_ksq    <= 16'd16;
    cf_lo     <= (BW+1)'(16);
    cf_hi     <= (BW+1)'((1 << BW) - 1);
    cf_min    <= '0;
    cf_g      <= GW'(4);
    cf_t      <= TW'(16);
    cf_flags  <= '0;
    cf_d0     <= '0;
    cf_step   <= '0;
    sn_arm    <= 1'b0;
    sn_pending <= 1'b0;
    sn_src    <= '0;
    sn_nf     <= 8'd1;
    pt_en     <= 1'b0;
    for (int c = 0; c < NCH; c++) begin
      inc_a[c] <= '0;  amp_a[c] <= '0;
      inc_b[c] <= '0;  amp_b[c] <= '0;
      nbits[c] <= '0;  src[c]   <= '0;
    end
  end else begin
    // a capture finished (the arm a write below may make in the same cycle is
    // not pending then: the capture it asks for cannot have finished)
    if (sn_done_edge) sn_pending <= 1'b0;
    if (sys_wen && !sys_addr[20]) begin
    case (a)
      20'h00004: run       <= sys_wdata[0];
      20'h0000c: min_bin   <= sys_wdata[BW-1:0];
      20'h00010: win_start <= {sys_wdata[BW-1:2], 2'b00};
      20'h00018: arm       <= !arm;
      20'h0001c: clr       <= !clr;
      20'h00024: spi_sdo_sel <= sys_wdata[1:0];
      20'h00028: rarm      <= !rarm;
      20'h0002c: raw_src   <= sys_wdata[1:0];
      20'h00200: d_inc_a   <= sys_wdata;
      20'h00204: d_amp_a   <= sys_wdata[15:0];
      20'h00208: d_inc_b   <= sys_wdata;
      20'h0020c: d_amp_b   <= sys_wdata[15:0];
      20'h00210: d_nbits   <= (sys_wdata[4:0] > 5'd18) ? 5'd18 : sys_wdata[4:0];
      20'h00214: d_run     <= sys_wdata[0];
      20'h00100: cf_ksq    <= sys_wdata[15:0];
      20'h00104: cf_lo     <= sys_wdata[BW:0];
      20'h00108: cf_hi     <= sys_wdata[BW:0];
      20'h0010c: cf_min    <= sys_wdata;
      20'h00110: cf_g      <= (sys_wdata > GMAX) ? GW'(GMAX) : sys_wdata[GW-1:0];
      20'h00114: cf_t      <= (sys_wdata > TMAX) ? TW'(TMAX) : sys_wdata[TW-1:0];
      20'h00118: cf_flags  <= sys_wdata[1:0];
      20'h0011c: cf_d0     <= sys_wdata[22:0];
      20'h00120: cf_step   <= sys_wdata[22:0];
      20'h00140: if (sys_wdata[31]) sn_pending <= 1'b0;
                 else if (!sn_pending) begin
                   sn_arm     <= !sn_arm;
                   sn_pending <= 1'b1;
                 end
      20'h00144: sn_src    <= {sys_wdata[4], 2'b00, sys_wdata[1:0]};
      20'h00148: sn_nf     <= sys_wdata[7:0];
      20'h0018c: pt_en     <= sys_wdata[0];
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
  end

// reads: registers answer after 1 cycle, the window RAMs after 3
logic [AW-1:0] rd_addr;
logic [31:0]   rd_data [NCH];
logic [2:0]    ram_rd;
logic [3:0]    ram_ch;          // held: axi_slave runs one transaction at a time
logic [12:0]   raw_addr;
logic [127:0]  raw_q [2];

always_ff @(posedge sys_clk) begin
  sys_err <= 1'b0;
  sys_ack <= 1'b0;
  ram_rd  <= {ram_rd[1:0], 1'b0};
  if (!sys_rstn) begin
    ram_rd <= '0;
  end else if (sys_wen) begin
    sys_ack <= 1'b1;
  end else if (sys_ren && sys_addr[20]) begin     // scope
    sys_ack   <= 1'b1;
    sys_rdata <= sc_read(a);
  end else if (sys_ren) begin
    for (int c = 0; c < NCH; c++)
      if (a == 20'h00058 + 20'h40*c) s_cf_hi[c] <= s_cf_word[c][63:32];
    if (a[19:16] != 0) begin                        // window RAM
      ram_ch  <= a[19:16] - 1'b1;
      rd_addr <= a[AW+1:2];
      raw_addr <= a[14:2];
      ram_rd  <= 3'b001;
    end else begin
      sys_ack   <= 1'b1;
      sys_rdata <= reg_read(a);
    end
  end
  if (ram_rd[2]) begin
    sys_ack   <= 1'b1;
    if (ram_ch == 7 || ram_ch == 8)               // raw captures, 0x80000 / 0x90000
      sys_rdata <= raw_q[ram_ch - 7][32*raw_addr[1:0] +: 32];
    else
      sys_rdata <= (ram_ch < NCH) ? rd_data[ram_ch] : 32'h0;
  end
end

function automatic logic [31:0] reg_read(input logic [19:0] ad);
  logic [31:0] v = 32'h0;
  case (ad)
    20'h00000: v = {16'hf0f6, 8'(NB), 8'(NCH)};
    20'h00004: v = {31'h0, run};
    20'h00008: v = {15'h0, s_raw_ready, {(8-NCH){1'b0}}, s_win_ready, 6'h0, s_locked, s_cfg_done};
    20'h0000c: v = 32'(min_bin);
    20'h00010: v = 32'(win_start);
    20'h00020: v = 32'(WIN);
    20'h00024: v = 32'(spi_sdo_sel);
    20'h0002c: v = 32'(raw_src);
    20'h00300: v = 32'(s_khz[0]);
    20'h00304: v = 32'(s_khz[1]);
    20'h00308: v = 32'(s_khz[2]);
    20'h0030c: v = 32'(s_khz[3]);
    20'h00310: v = 32'(s_khz[4]);
    20'h00200: v = d_inc_a;
    20'h00204: v = 32'(d_amp_a);
    20'h00208: v = d_inc_b;
    20'h0020c: v = 32'(d_amp_b);
    20'h00210: v = 32'(d_nbits);
    20'h00214: v = 32'(d_run);
    20'h00100: v = 32'(cf_ksq);
    20'h00104: v = 32'(cf_lo);
    20'h00108: v = 32'(cf_hi);
    20'h0010c: v = cf_min;
    20'h00110: v = 32'(cf_g);
    20'h00114: v = 32'(cf_t);
    20'h00118: v = 32'(cf_flags);
    20'h0011c: v = 32'(cf_d0);
    20'h00120: v = 32'(cf_step);
    20'h00124: v = {12'h0, 4'(FRAC), 8'(TMAX), 8'(GMAX)};
    20'h00144: v = 32'(sn_src);
    20'h00148: v = 32'(sn_nf);
    20'h0014c: v = {16'h0, s_sn_cnt, 4'h0, s_sn_err, s_sn_ovf, s_sn_busy, sn_pending};
    20'h00150: v = s_sn_beats;
    20'h00154: v = SNAP_BASE[31:0];
    20'h00158: v = 32'(SNAP_FB);
    20'h0015c: v = 32'(SNAP_MAXF);
    20'h00180: v = 32'(s_pt_ptr);
    20'h00184: v = 32'(PT_FPP * NCH);
    20'h00188: v = {8'(PT_VERSION), 8'(NCH), 16'(PT_FPP)};
    20'h0018c: v = 32'(pt_en);
    20'h00190: v = s_pt_pkts;
    20'h00194: v = s_pt_drops;
    20'h00198: v = 32'(s_pt_err);
    20'h0019c: v = 32'(PT_RING);
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
      20'h00058 + 20'h40*c: v = s_cf_word[c][31:0];
      20'h0005c + 20'h40*c: v = s_cf_hi[c];
      20'h0006c + 20'h40*c: v = s_cf_cnt[c];
      20'h00074 + 20'h40*c: v = s_cf_det[c];
      20'h00078 + 20'h40*c: v = s_cf_ovr[c];
      default: ;
    endcase
  return v;
endfunction

// scope registers (slot + 0x100000)
always_ff @(posedge sys_clk)
  if (!sys_rstn) begin
    sc_arm_t <= 1'b0;  sc_rst_t <= 1'b0;  sc_src_t <= 1'b0;
    sc_keep  <= 1'b0;  sc_avg   <= 1'b0;  sc_src   <= '0;
    sc_thr   <= '0;    sc_hyst  <= 14'd20;
    sc_dly   <= 32'(SC_L / 2);
    sc_dec   <= 17'd2;
    sc_in    <= '{3'd0, 3'd1};
  end else if (sys_wen && sys_addr[20])
    case (a)
      20'h00000: begin
        if (sys_wdata[0]) sc_arm_t <= !sc_arm_t;
        if (sys_wdata[1]) sc_rst_t <= !sc_rst_t;
        sc_keep <= sys_wdata[3];
      end
      20'h00004: begin sc_src <= sys_wdata[3:0]; sc_src_t <= !sc_src_t; end
      20'h00008: sc_thr  <= sys_wdata[13:0];
      20'h00010: sc_dly  <= sys_wdata;
      20'h00014: sc_dec  <= (sys_wdata < 2) ? 17'd2 : sys_wdata[16:0];
      20'h00020: sc_hyst <= sys_wdata[13:0];
      20'h00028: sc_avg  <= sys_wdata[0];
      20'h001f0: sc_in[0] <= sys_wdata[2:0];
      20'h001f4: sc_in[1] <= sys_wdata[2:0];
      default: ;
    endcase

// s_sc_st: [0] writing, [1] delay running, [5:2] trigger source, [6] pre-trigger
// in, [7] arm taken, [8] reset taken, [31:32-SC_RW] wp_cur, [63:64-SC_RW] wp_trig
function automatic logic [31:0] sc_read(input logic [19:0] ad);
  logic [31:0] v = 32'h0;
  logic        busy = s_sc_st[0] || (s_sc_st[7] != sc_arm_t);
  case (ad)
    20'h00000: v = {28'h0, sc_keep, s_sc_st[1], 1'b0, busy};
    20'h00004: v = 32'(s_sc_st[5:2]);
    20'h00008: v = 32'(sc_thr);
    20'h00010: v = sc_dly;
    20'h00014: v = 32'(sc_dec);
    20'h00018: v = 32'(s_sc_st[31 -: SC_RW]);
    20'h0001c: v = 32'(s_sc_st[63 -: SC_RW]);
    20'h00020: v = 32'(sc_hyst);
    20'h00028: v = 32'(sc_avg);
    20'h0016c: v = 32'(s_sc_st[6]);
    20'h001e0: v = 32'(SC_L);
    20'h001e4: v = SC_BASE[31:0];
    20'h001e8: v = 32'(2 * SC_L);
    20'h001ec: v = {8'h0, 8'd16, 8'd2, 8'd4};
    20'h001f0: v = 32'(sc_in[0]);
    20'h001f4: v = 32'(sc_in[1]);
    20'h001f8: v = 32'd2;
    20'h001fc: v = 32'd400;
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

// CFAR settings and snapshot control, the same way (sn_arm is a toggle)
logic [15:0]   f_cf_ksq, f_cf_ksq_m;
logic [BW:0]   f_cf_lo, f_cf_lo_m, f_cf_hi, f_cf_hi_m;
logic [31:0]   f_cf_min, f_cf_min_m;
logic [GW-1:0] f_cf_g, f_cf_g_m;
logic [TW-1:0] f_cf_t, f_cf_t_m;
logic [1:0]    f_cf_flags, f_cf_flags_m;
logic [22:0]   f_cf_d0, f_cf_d0_m, f_cf_step, f_cf_step_m;
logic [4:0]    f_sn_src, f_sn_src_m;
logic [7:0]    f_sn_nf, f_sn_nf_m;
(* ASYNC_REG = "TRUE" *) logic f_sn_arm_m, f_sn_arm;
(* ASYNC_REG = "TRUE" *) logic f_pt_en_m, f_pt_en;

always_ff @(posedge fclk) begin
  {f_cf_ksq,   f_cf_ksq_m}   <= {f_cf_ksq_m,   cf_ksq};
  {f_cf_lo,    f_cf_lo_m}    <= {f_cf_lo_m,    cf_lo};
  {f_cf_hi,    f_cf_hi_m}    <= {f_cf_hi_m,    cf_hi};
  {f_cf_min,   f_cf_min_m}   <= {f_cf_min_m,   cf_min};
  {f_cf_g,     f_cf_g_m}     <= {f_cf_g_m,     cf_g};
  {f_cf_t,     f_cf_t_m}     <= {f_cf_t_m,     cf_t};
  {f_cf_flags, f_cf_flags_m} <= {f_cf_flags_m, cf_flags};
  {f_cf_d0,    f_cf_d0_m}    <= {f_cf_d0_m,    cf_d0};
  {f_cf_step,  f_cf_step_m}  <= {f_cf_step_m,  cf_step};
  {f_sn_src,   f_sn_src_m}   <= {f_sn_src_m,   sn_src};
  {f_sn_nf,    f_sn_nf_m}    <= {f_sn_nf_m,    sn_nf};
  {f_sn_arm,   f_sn_arm_m}   <= {f_sn_arm_m,   sn_arm};
  {f_pt_en,    f_pt_en_m}    <= {f_pt_en_m,    pt_en};
end

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

// snapshot sources: |X| of each channel (natural order), then each FFT's input
logic [127:0]     sn_data  [2*NCH];
logic [2*NCH-1:0] sn_valid, sn_first;

// every channel's CFAR results, and channel 0's frame start, for the point stream
logic [NCH-1:0]   pt_valid;
logic [63:0]      pt_word [NCH];
logic             pt_tick;

assign f_cfg_all = &f_cfg_done;

// clock meters (debug): the RF Data Converter's output clocks and fclk, in kHz
logic [23:0] s_khz [5];
logic        s_raw_ready;

for (genvar i = 0; i < 4; i++) begin : g_meter
  zcu216_freq_meter i_meter (.clk_ref (sys_clk), .clk_meas (meas_clk[i]), .khz (s_khz[i]));
end
zcu216_freq_meter i_meter_fclk (.clk_ref (sys_clk), .clk_meas (fclk), .khz (s_khz[4]));

// raw capture (debug): RAW consecutive fclk beats of one RF-ADC channel and of
// the DAC stream, to see the converter data in the time domain
(* ASYNC_REG = "TRUE" *) logic [2:0] f_rarm_s;
logic [1:0]  f_raw_src, f_raw_src_m;
logic        f_rcap, f_rdone;
logic [10:0] f_rcnt;
(* ram_style = "block" *) logic [127:0] raw_adc [0:RAW-1];
(* ram_style = "block" *) logic [127:0] raw_dac [0:RAW-1];

always_ff @(posedge fclk) begin
  f_rarm_s <= {f_rarm_s[1:0], rarm};
  {f_raw_src, f_raw_src_m} <= {f_raw_src_m, raw_src};
  if (!frstn) begin
    f_rcap  <= 1'b0;
    f_rdone <= 1'b0;
    f_rcnt  <= '0;
  end else begin
    if (f_rarm_s[2] != f_rarm_s[1]) begin
      f_rcap <= 1'b1;
      f_rcnt <= '0;
    end else if (f_rcap) begin
      f_rcnt <= f_rcnt + 1'b1;
      if (&f_rcnt) begin
        f_rcap  <= 1'b0;
        f_rdone <= !f_rdone;
      end
    end
  end
  if (f_rcap) begin
    raw_adc[f_rcnt] <= adc_data[f_raw_src];
    raw_dac[f_rcnt] <= dac_data;
  end
end

always_ff @(posedge sys_clk) begin
  raw_q[0] <= raw_adc[raw_addr[12:2]];
  raw_q[1] <= raw_dac[raw_addr[12:2]];
end

(* ASYNC_REG = "TRUE" *) logic [2:0] rdone_s;
always_ff @(posedge sys_clk) begin
  rdone_s     <= {rdone_s[1:0], f_rdone};
  s_raw_ready <= (rdone_s[2] == rarm);
end

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

  logic in_accept, in_first;

  zcu216_fft_dit8 #(.SUB(SUB), .IN_W(14), .DW(27), .TW(18),
                    .TWC_FILE(TWC_FILE), .TWF_FILE(TWF_FILE)) i_fft (
    .clk (fclk), .rstn (frstn), .cfg_done (f_cfg_done[c]),
    .in_data (s_data), .in_valid (s_valid), .run (f_run && f_cfg_all),
    .in_accept, .in_first,
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

  assign sn_data[c]        = {n_mag[3], n_mag[2], n_mag[1], n_mag[0]};
  assign sn_valid[c]       = n_valid;
  assign sn_first[c]       = n_first;
  assign sn_data[NCH + c]  = s_data;
  assign sn_valid[NCH + c] = in_accept;
  assign sn_first[NCH + c] = in_first;

  // CFAR detector on the natural-order spectrum
  logic        cf_valid, cf_ovr;
  logic [63:0] cf_word, f_cf_word;
  logic [31:0] f_cf_cnt, f_cf_det, f_cf_ovr;
  logic        f_cf_upd;

  assign pt_valid[c] = cf_valid;
  assign pt_word[c]  = cf_word;
  if (c == 0) begin : g_tick
    assign pt_tick = n_valid && n_first;
  end

  zcu216_cfar #(.SUB(SUB), .DW(32), .GMAX(GMAX), .TMAX(TMAX), .FRAC(FRAC)) i_cfar (
    .clk (fclk), .rstn (frstn),
    .n_mag, .n_t, .n_valid, .n_first, .n_last,
    .k_sq (f_cf_ksq), .start_index (f_cf_lo), .end_index (f_cf_hi),
    .data_min (f_cf_min), .guard (f_cf_g), .train (f_cf_t),
    .onesided (f_cf_flags[0]), .so_mode (f_cf_flags[1]),
    .ramp_d0 (f_cf_d0), .ramp_step (f_cf_step),
    .r_valid (cf_valid), .r_word (cf_word), .r_overrun (cf_ovr));

  always_ff @(posedge fclk)
    if (!frstn) begin
      f_cf_cnt <= '0;
      f_cf_det <= '0;
      f_cf_ovr <= '0;
      f_cf_upd <= 1'b0;
    end else begin
      if (cf_valid) begin
        f_cf_word <= cf_word;
        f_cf_cnt  <= f_cf_cnt + 1'b1;
        f_cf_det  <= f_cf_det + cf_word[32];
        f_cf_upd  <= !f_cf_upd;
      end
      if (cf_ovr) f_cf_ovr <= f_cf_ovr + 1'b1;
    end

  // results into sys_clk: copy when the per-frame toggle moves
  (* ASYNC_REG = "TRUE" *) logic [2:0] upd_s, done_s, cfu_s;
  logic [2:0] ev_m, ev_s;
  always_ff @(posedge sys_clk) begin
    upd_s  <= {upd_s[1:0], f_upd};
    done_s <= {done_s[1:0], f_done};
    cfu_s  <= {cfu_s[1:0], f_cf_upd};
    {ev_s, ev_m} <= {ev_m, f_events};
    if (upd_s[2] != upd_s[1]) begin
      s_frames[c]   <= f_frames;
      s_peak_bin[c] <= f_peak_bin;
      s_peak_mag[c] <= f_peak_mag;
    end
    if (cfu_s[2] != cfu_s[1]) begin
      s_cf_word[c] <= f_cf_word;
      s_cf_cnt[c]  <= f_cf_cnt;
      s_cf_det[c]  <= f_cf_det;
      s_cf_ovr[c]  <= f_cf_ovr;
    end
    s_events[c]    <= ev_s;
    s_win_ready[c] <= (done_s[1] == arm);
  end
end

// snapshot DMA
logic        f_sn_busy, f_sn_done, f_sn_ovf, f_sn_err;
logic [31:0] f_sn_beats;
localparam int SRW = $clog2(2 * NCH);

// oscilloscope: settings into fclk, two channels from the RF-ADC or the FFT inputs
logic          f_sc_arm, f_sc_rst, f_sc_src_t, f_sc_keep, f_sc_avg;
logic [3:0]    f_sc_src;
logic [13:0]   f_sc_thr, f_sc_hyst;
logic [31:0]   f_sc_dly;
logic [16:0]   f_sc_dec;
logic [2:0]    f_sc_in [2];
logic [127:0]  sc_x [2], sc_xr [2];
logic          sc_req, sc_grant, sc_start, sc_push, sc_flush, sc_idle;
logic [127:0]  sc_data;
logic [31:0]   sc_words;
logic          f_sc_writing, f_sc_dly_do, f_sc_pre, f_sc_arm_ack, f_sc_rst_ack;
logic [3:0]    f_sc_trig;
logic [SC_RW-1:0] f_sc_wp_cur, f_sc_wp_trig;

(* ASYNC_REG = "TRUE" *) logic [1:0] sc_arm_m, sc_rst_m, sc_src_tm;
logic          m_sc_keep, m_sc_avg;
logic [3:0]    m_sc_src;
logic [13:0]   m_sc_thr, m_sc_hyst;
logic [31:0]   m_sc_dly;
logic [16:0]   m_sc_dec;
logic [2:0]    m_sc_in [2];
always_ff @(posedge fclk) begin
  {f_sc_arm, sc_arm_m}     <= {sc_arm_m, sc_arm_t};
  {f_sc_rst, sc_rst_m}     <= {sc_rst_m, sc_rst_t};
  {f_sc_src_t, sc_src_tm}  <= {sc_src_tm, sc_src_t};
  // quasi-static: two flops (the commands above arrive after their values)
  {f_sc_keep, m_sc_keep} <= {m_sc_keep, sc_keep};
  {f_sc_avg,  m_sc_avg}  <= {m_sc_avg,  sc_avg};
  {f_sc_src,  m_sc_src}  <= {m_sc_src,  sc_src};
  {f_sc_thr,  m_sc_thr}  <= {m_sc_thr,  sc_thr};
  {f_sc_hyst, m_sc_hyst} <= {m_sc_hyst, sc_hyst};
  {f_sc_dly,  m_sc_dly}  <= {m_sc_dly,  sc_dly};
  {f_sc_dec,  m_sc_dec}  <= {m_sc_dec,  sc_dec};
  m_sc_in <= sc_in;
  f_sc_in <= m_sc_in;
end

// inputs: 14-bit samples in 16 bits, as the FFTs see them
always_ff @(posedge fclk)
  for (int c = 0; c < 2; c++) begin
    for (int j = 0; j < 8; j++)
      sc_x[c][16*j +: 16] <= (f_sc_in[c] < 4)
          ? 16'($signed(adc_data[f_sc_in[c][1:0]][16*j +: 16]) >>> 2)
          : (f_sc_in[c] - 4 < NCH) ? sn_data[NCH + 32'(f_sc_in[c] - 4)][16*j +: 16] : 16'h0;
    sc_xr[c] <= sc_x[c];
  end

zcu216_scope #(.L(SC_L)) i_scope (
  .clk (fclk), .rstn (frstn), .x1 (sc_xr[0]), .x2 (sc_xr[1]),
  .dec (f_sc_dec), .avg (f_sc_avg), .thr (16'($signed(f_sc_thr))), .hyst (16'(f_sc_hyst)),
  .dly (f_sc_dly), .keep (f_sc_keep), .src_val (f_sc_src), .src_set (f_sc_src_t),
  .arm (f_sc_arm), .rst (f_sc_rst),
  .writing (f_sc_writing), .dly_do (f_sc_dly_do), .trig_src (f_sc_trig),
  .pretrig_ok (f_sc_pre), .wp_cur (f_sc_wp_cur), .wp_trig (f_sc_wp_trig),
  .arm_ack (f_sc_arm_ack), .rst_ack (f_sc_rst_ack),
  .sc_req, .sc_grant, .sc_start, .sc_push, .sc_data, .sc_flush, .sc_idle, .sc_words);

// status into sys_clk: a snapshot every 64 fclk cycles, taken on its toggle
logic [63:0] f_sc_st;
logic [5:0]  f_sc_tick = '0;
logic        f_sc_upd = 1'b0;
always_ff @(posedge fclk) begin
  f_sc_tick <= f_sc_tick + 1'b1;
  if (f_sc_tick == 0) begin
    f_sc_st <= '0;
    f_sc_st[8:0] <= {f_sc_rst_ack, f_sc_arm_ack, f_sc_pre, f_sc_trig, f_sc_dly_do, f_sc_writing};
    f_sc_st[31 -: SC_RW] <= f_sc_wp_cur;
    f_sc_st[63 -: SC_RW] <= f_sc_wp_trig;
    f_sc_upd <= !f_sc_upd;
  end
end
(* ASYNC_REG = "TRUE" *) logic [2:0] sc_upd_s;
always_ff @(posedge sys_clk) begin
  sc_upd_s <= {sc_upd_s[1:0], f_sc_upd};
  if (sc_upd_s[2] != sc_upd_s[1]) s_sc_st <= f_sc_st;
end

zcu216_snap #(.NSRC(2 * NCH), .SUB(SUB), .BASE(SNAP_BASE), .SIZE(SNAP_SIZE),
              .SC_BASE(SC_BASE), .SC_BYTES(8 * SC_L)) i_snap (
  .clk (fclk), .rstn (frstn),
  .s_data (sn_data), .s_valid (sn_valid), .s_first (sn_first),
  .arm (f_sn_arm),
  .src (f_sn_src[4] ? SRW'(NCH + f_sn_src[1:0]) : SRW'(f_sn_src[1:0])),
  .nframes (f_sn_nf),
  .busy (f_sn_busy), .done (f_sn_done), .overflow (f_sn_ovf), .err (f_sn_err),
  .beats (f_sn_beats),
  .sc_req, .sc_grant, .sc_start, .sc_push, .sc_data, .sc_flush, .sc_idle, .sc_words,
  .m_awaddr (hp_awaddr), .m_awlen (hp_awlen), .m_awsize (hp_awsize),
  .m_awburst (hp_awburst), .m_awcache (hp_awcache), .m_awprot (hp_awprot),
  .m_awvalid (hp_awvalid), .m_awready (hp_awready),
  .m_wdata (hp_wdata), .m_wstrb (hp_wstrb), .m_wlast (hp_wlast),
  .m_wvalid (hp_wvalid), .m_wready (hp_wready),
  .m_bresp (hp_bresp), .m_bvalid (hp_bvalid), .m_bready (hp_bready));

// status into sys_clk; beats only matter once done, when it holds still
(* ASYNC_REG = "TRUE" *) logic [2:0] sn_done_s;
(* ASYNC_REG = "TRUE" *) logic [1:0] sn_busy_s, sn_ovf_s, sn_err_s;
logic [31:0] sn_beats_m;
always_ff @(posedge sys_clk) begin
  sn_done_s  <= {sn_done_s[1:0], f_sn_done};
  sn_busy_s  <= {sn_busy_s[0], f_sn_busy};
  sn_ovf_s   <= {sn_ovf_s[0], f_sn_ovf};
  sn_err_s   <= {sn_err_s[0], f_sn_err};
  {s_sn_beats, sn_beats_m} <= {sn_beats_m, f_sn_beats};
  if (!sys_rstn) s_sn_cnt <= '0;
  else if (sn_done_edge) s_sn_cnt <= s_sn_cnt + 1'b1;
end
assign sn_done_edge = sn_done_s[2] != sn_done_s[1];
assign s_sn_busy    = sn_busy_s[1];
assign s_sn_ovf     = sn_ovf_s[1];
assign s_sn_err     = sn_err_s[1];

// point stream
logic [13:0] f_pt_ptr;
logic [31:0] f_pt_pkts, f_pt_drops;
logic        f_pt_err;

zcu216_points #(.NCH(NCH), .FPP(PT_FPP), .BASE(40'h1E00_0000), .RING(PT_RING),
                .VERSION(PT_VERSION)) i_points (
  .clk (fclk), .rstn (frstn), .enable (f_pt_en),
  .tick (pt_tick), .r_valid (pt_valid), .r_word (pt_word),
  .wr_ptr (f_pt_ptr), .packets (f_pt_pkts), .drops (f_pt_drops), .err (f_pt_err),
  .m_awaddr (pt_awaddr), .m_awlen (pt_awlen), .m_awsize (pt_awsize),
  .m_awburst (pt_awburst), .m_awcache (pt_awcache), .m_awprot (pt_awprot),
  .m_awvalid (pt_awvalid), .m_awready (pt_awready),
  .m_wdata (pt_wdata), .m_wstrb (pt_wstrb), .m_wlast (pt_wlast),
  .m_wvalid (pt_wvalid), .m_wready (pt_wready),
  .m_bresp (pt_bresp), .m_bvalid (pt_bvalid), .m_bready (pt_bready));

zcu216_gray_sync #(.W(14)) i_pt_ptr_s (.src_clk (fclk), .src (f_pt_ptr),
                                       .dst_clk (sys_clk), .dst (s_pt_ptr));
zcu216_gray_sync #(.W(32)) i_pt_pkts_s (.src_clk (fclk), .src (f_pt_pkts),
                                        .dst_clk (sys_clk), .dst (s_pt_pkts));
zcu216_gray_sync #(.W(32)) i_pt_drops_s (.src_clk (fclk), .src (f_pt_drops),
                                         .dst_clk (sys_clk), .dst (s_pt_drops));
(* ASYNC_REG = "TRUE" *) logic [1:0] pt_err_s;
always_ff @(posedge sys_clk) pt_err_s <= {pt_err_s[0], f_pt_err};
assign s_pt_err = pt_err_s[1];

(* ASYNC_REG = "TRUE" *) logic [1:0] cfg_s, lock_s;
always_ff @(posedge sys_clk) begin
  cfg_s  <= {cfg_s[0], f_cfg_all};
  lock_s <= {lock_s[0], flocked};
end
assign s_cfg_done = cfg_s[1];
assign s_locked   = lock_s[1];

endmodule


/**
 * Frequency of clk_meas in kHz: a Gray-coded counter in the measured domain,
 * sampled on clk_ref (100 MHz) over a 1 ms gate. 0 when clk_meas is absent.
 */
module zcu216_freq_meter #(
  parameter int GATE = 100000                 // clk_ref cycles per gate (1 ms at 100 MHz)
) (
  input  logic        clk_ref,
  input  logic        clk_meas,
  output logic [23:0] khz
);

logic [23:0] cnt = '0, gray = '0;
always_ff @(posedge clk_meas) begin
  cnt  <= cnt + 1'b1;
  gray <= cnt ^ (cnt >> 1);
end

(* ASYNC_REG = "TRUE" *) logic [23:0] g1, g2;
logic [23:0] bin, last;
logic [16:0] gate;

always_comb begin
  bin[23] = g2[23];
  for (int i = 22; i >= 0; i--) bin[i] = bin[i+1] ^ g2[i];
end

always_ff @(posedge clk_ref) begin
  g1 <= gray;
  g2 <= g1;
  if (gate == 17'(GATE - 1)) begin
    gate <= '0;
    khz  <= bin - last;
    last <= bin;
  end else
    gate <= gate + 1'b1;
end

endmodule
