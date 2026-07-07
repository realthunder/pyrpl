/**
 * $Id: red_pitaya_scope.v 965 2014-01-24 13:39:56Z matej.oblak $
 *
 * @brief Red Pitaya oscilloscope application, used for capturing ADC data
 *        into BRAMs, which can be later read by SW.
 *
 * @Author Matej Oblak
 *
 * (c) Red Pitaya  http://www.redpitaya.com
 *
 * This part of code is written in Verilog hardware description language (HDL).
 * Please visit http://en.wikipedia.org/wiki/Verilog
 * for more details on the language used herein.
 */
/*
###############################################################################
#    pyrpl - DSP servo controller for quantum optics with the RedPitaya
#    Copyright (C) 2014-2016  Leonhard Neuhaus  (neuhaus@spectro.jussieu.fr)
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.
############################################################################### 
*/

/**
 * GENERAL DESCRIPTION:
 *
 * This is simple data aquisition module, primerly used for scilloscope 
 * application. It consists from three main parts.
 *
 *
 *                /--------\      /-----------\            /-----\
 *   ADC CHA ---> | DFILT1 | ---> | AVG & DEC | ---------> | BUF | --->  SW
 *                \--------/      \-----------/     |      \-----/
 *                                                  ˇ         ^
 *                                              /------\      |
 *   ext trigger -----------------------------> | TRIG | -----+
 *                                              \------/      |
 *                                                  ^         ˇ
 *                /--------\      /-----------\     |      /-----\
 *   ADC CHB ---> | DFILT1 | ---> | AVG & DEC | ---------> | BUF | --->  SW
 *                \--------/      \-----------/            \-----/ 
 *
 *
 * Input data is optionaly averaged and decimated via average filter.
 *
 * Trigger section makes triggers from input ADC data or external digital 
 * signal. To make trigger from analog signal schmitt trigger is used, external
 * trigger goes first over debouncer, which is separate for pos. and neg. edge.
 *
 * Data capture buffer is realized with BRAM. Writing into ram is done with 
 * arm/trig logic. With adc_arm_do signal (SW) writing is enabled, this is active
 * until trigger arrives and adc_dly_cnt counts to zero. Value adc_wp_trig
 * serves as pointer which shows when trigger arrived. This is used to show
 * pre-trigger data.
 * 
 */

module red_pitaya_scope #(
  parameter ASZ  = 14,  // ADC input sample data width
  parameter DSZ  = 28,  // FFT_output width
  parameter FSZ  = 13,  // FFT transform length 2^FSZ (max runtime size)
  parameter FRAC = 8,   // sub-bin interpolation fractional bits (k_interp = Q(FSZ).FRAC); 0=off
  parameter RSZ  = 14,  // RAM size 2^RSZ
  parameter HSZ  = 24,  // scan-index (hist_index) width; build knob, default 24
  parameter FSSR     = 1, // FFT super sample rate (parallel channels)
  parameter FFT_IMPL = 3,  // 1=LogiCORE, 2=HLS SSR (DIT), 3=IP SSR (DIF)
  parameter FFT_SINGLE = 0, // 1 = build only fft_a, omit fft_b (e.g. SSR=8 to fit)
  parameter HIST_BLOCK_SIZE = 128
)(

   // ADC
   input                 adc_clk_i       ,  // ADC clock
   input                 adc_rstn_i      ,  // ADC reset - active low
   input      [ASZ-1: 0] adc_a_i         ,  // ADC data CHA
   input      [ASZ-1: 0] adc_b_i         ,  // ADC data CHB
   // trigger sources
   input                 trig_ext_i      ,  // external trigger
   input      [  4-1: 0] trig_asg_i      ,  // ASG trigger
   input                 trig_dsp_i      ,  // DSP module trigger
   output                trig_scope_o    ,  // copy of scope trigger

   input                 fft_clk_i       ,  // FFT clock
   output logic          fft_active_o    ,  // fft captureing
   output       [ 2-1:0] fft_window_o    ,  // fft acq windows {down, up} (adc clk)
   output                scope_sig_o     ,  // scan signaling
   output                x_step_0        ,  // x step
   output logic          y_step_0        ,  // y step

   // Point cloud DMA outputs (AXI-S, one stream per FFT channel, clk_i domain)
   output logic [ 63:0]  dma_a_tdata     ,
   output logic          dma_a_tvalid    ,
   input                 dma_a_tready    ,
   output logic          dma_a_tlast     ,

   output logic [ 63:0]  dma_b_tdata     ,
   output logic          dma_b_tvalid    ,
   input                 dma_b_tready    ,
   output logic          dma_b_tlast     ,

   input                 sync_rst_i      ,  // syncrhonized reset signal (from ASG)

   input      [RSZ-1: 0] x_step_i     ,  // asg1 step index
   input      [RSZ-1: 0] y_step_i     ,  // asg2 step index

   // AXI0 master
   output                axi0_clk_o      ,  // global clock
   output                axi0_rstn_o     ,  // global reset
   output     [ 32-1: 0] axi0_waddr_o    ,  // system write address
   output     [ 64-1: 0] axi0_wdata_o    ,  // system write data
   output     [  8-1: 0] axi0_wsel_o     ,  // system write byte select
   output                axi0_wvalid_o   ,  // system write data valid
   output     [  4-1: 0] axi0_wlen_o     ,  // system write burst length
   output                axi0_wfixed_o   ,  // system write burst type (fixed / incremental)
   input                 axi0_werr_i     ,  // system write error
   input                 axi0_wrdy_i     ,  // system write ready

   // AXI1 master
   output                axi1_clk_o      ,  // global clock
   output                axi1_rstn_o     ,  // global reset
   output     [ 32-1: 0] axi1_waddr_o    ,  // system write address
   output     [ 64-1: 0] axi1_wdata_o    ,  // system write data
   output     [  8-1: 0] axi1_wsel_o     ,  // system write byte select
   output                axi1_wvalid_o   ,  // system write data valid
   output     [  4-1: 0] axi1_wlen_o     ,  // system write burst length
   output                axi1_wfixed_o   ,  // system write burst type (fixed / incremental)
   input                 axi1_werr_i     ,  // system write error
   input                 axi1_wrdy_i     ,  // system write ready

   // System bus
   input      [ 32-1: 0] sys_addr      ,  // bus saddress
   input      [ 32-1: 0] sys_wdata     ,  // bus write data
   input      [  4-1: 0] sys_sel       ,  // bus write byte select
   input                 sys_wen       ,  // bus write enable
   input                 sys_ren       ,  // bus read enable
   output reg [ 32-1: 0] sys_rdata     ,  // bus read data
   output reg            sys_err       ,  // bus error indicator
   output reg            sys_ack          // bus acknowledge signal
);

reg             adc_arm_do   ;
reg             adc_rst_do   ;

// input filter is disabled

//---------------------------------------------------------------------------------
//  Input filtering

wire [ASZ-1: 0] adc_a_filt_in  ;
wire [ASZ-1: 0] adc_a_filt_out ;
wire [ASZ-1: 0] adc_b_filt_in  ;
wire [ASZ-1: 0] adc_b_filt_out ;
/*
reg  [ 18-1: 0] set_a_filt_aa  ;
reg  [ 25-1: 0] set_a_filt_bb  ;
reg  [ 25-1: 0] set_a_filt_kk  ;
reg  [ 25-1: 0] set_a_filt_pp  ;
reg  [ 18-1: 0] set_b_filt_aa  ;
reg  [ 25-1: 0] set_b_filt_bb  ;
reg  [ 25-1: 0] set_b_filt_kk  ;
reg  [ 25-1: 0] set_b_filt_pp  ;
*/


// bypass the filtering for the scope in order to spare the DSP slices for other stuff, 
// since we never look at signals close to nyquist
assign adc_a_filt_in = adc_a_i ;
assign adc_b_filt_in = adc_b_i ;
assign adc_a_filt_out = adc_a_filt_in;
assign adc_b_filt_out = adc_b_filt_in;

/*
red_pitaya_dfilt1 i_dfilt1_cha (
   // ADC
  .adc_clk_i   ( adc_clk_i       ),  // ADC clock
  .adc_rstn_i  ( adc_rstn_i      ),  // ADC reset - active low
  .adc_dat_i   ( adc_a_filt_in   ),  // ADC data
  .adc_dat_o   ( adc_a_filt_out  ),  // ADC data
   // configuration
  .cfg_aa_i    ( set_a_filt_aa   ),  // config AA coefficient
  .cfg_bb_i    ( set_a_filt_bb   ),  // config BB coefficient
  .cfg_kk_i    ( set_a_filt_kk   ),  // config KK coefficient
  .cfg_pp_i    ( set_a_filt_pp   )   // config PP coefficient
);

red_pitaya_dfilt1 i_dfilt1_chb (
   // ADC
  .adc_clk_i   ( adc_clk_i       ),  // ADC clock
  .adc_rstn_i  ( adc_rstn_i      ),  // ADC reset - active low
  .adc_dat_i   ( adc_b_filt_in   ),  // ADC data
  .adc_dat_o   ( adc_b_filt_out  ),  // ADC data
   // configuration
  .cfg_aa_i    ( set_b_filt_aa   ),  // config AA coefficient
  .cfg_bb_i    ( set_b_filt_bb   ),  // config BB coefficient
  .cfg_kk_i    ( set_b_filt_kk   ),  // config KK coefficient
  .cfg_pp_i    ( set_b_filt_pp   )   // config PP coefficient
);
*/
//---------------------------------------------------------------------------------
//  Decimate input data

reg  [ASZ-1: 0] adc_a_dat     ;
reg  [ASZ-1: 0] adc_b_dat     ;
reg  [ 32-1: 0] adc_a_sum     ;
reg  [ 32-1: 0] adc_b_sum     ;
reg  [ 17-1: 0] set_dec       ;
reg  [ 17-1: 0] adc_dec_cnt   ;
reg             set_avg_en    ;
reg             adc_dv        ;

always @(posedge adc_clk_i)
if (adc_rstn_i == 1'b0) begin
   adc_a_sum   <= 32'h0 ;
   adc_b_sum   <= 32'h0 ;
   adc_dec_cnt <= 17'h0 ;
   adc_dv      <=  1'b0 ;
end else begin
   if ((adc_dec_cnt >= set_dec) || adc_arm_do) begin // start again or arm
      adc_dec_cnt <= 17'h1                   ;
      adc_a_sum   <= $signed(adc_a_filt_out) ;
      adc_b_sum   <= $signed(adc_b_filt_out) ;
   end else begin
      adc_dec_cnt <= adc_dec_cnt + 17'h1 ;
      adc_a_sum   <= $signed(adc_a_sum) + $signed(adc_a_filt_out) ;
      adc_b_sum   <= $signed(adc_b_sum) + $signed(adc_b_filt_out) ;
   end

   adc_dv <= (adc_dec_cnt >= set_dec) ;

   case (set_dec & {17{set_avg_en}})
      17'h0     : begin adc_a_dat <= adc_a_filt_out;            adc_b_dat <= adc_b_filt_out;        end
      17'h1     : begin adc_a_dat <= adc_a_sum[15+0 :  0];      adc_b_dat <= adc_b_sum[15+0 :  0];  end
      17'h2     : begin adc_a_dat <= adc_a_sum[15+1 :  1];      adc_b_dat <= adc_b_sum[15+1 :  1];  end
      17'h4     : begin adc_a_dat <= adc_a_sum[15+2 :  2];      adc_b_dat <= adc_b_sum[15+2 :  2];  end
      17'h8     : begin adc_a_dat <= adc_a_sum[15+3 :  3];      adc_b_dat <= adc_b_sum[15+3 :  3];  end
      17'h10    : begin adc_a_dat <= adc_a_sum[15+4 :  4];      adc_b_dat <= adc_b_sum[15+4 :  4];  end
      17'h20    : begin adc_a_dat <= adc_a_sum[15+5 :  5];      adc_b_dat <= adc_b_sum[15+5 :  5];  end
      17'h40    : begin adc_a_dat <= adc_a_sum[15+6 :  6];      adc_b_dat <= adc_b_sum[15+6 :  6];  end
      17'h80    : begin adc_a_dat <= adc_a_sum[15+7 :  7];      adc_b_dat <= adc_b_sum[15+7 :  7];  end
      17'h100   : begin adc_a_dat <= adc_a_sum[15+8 :  8];      adc_b_dat <= adc_b_sum[15+8 :  8];  end
      17'h200   : begin adc_a_dat <= adc_a_sum[15+9 :  9];      adc_b_dat <= adc_b_sum[15+9 :  9];  end
      17'h400   : begin adc_a_dat <= adc_a_sum[15+10: 10];      adc_b_dat <= adc_b_sum[15+10: 10];  end
      17'h800   : begin adc_a_dat <= adc_a_sum[15+11: 11];      adc_b_dat <= adc_b_sum[15+11: 11];  end
      17'h1000  : begin adc_a_dat <= adc_a_sum[15+12: 12];      adc_b_dat <= adc_b_sum[15+12: 12];  end
      17'h2000  : begin adc_a_dat <= adc_a_sum[15+13: 13];      adc_b_dat <= adc_b_sum[15+13: 13];  end
      17'h4000  : begin adc_a_dat <= adc_a_sum[15+14: 14];      adc_b_dat <= adc_b_sum[15+14: 14];  end
      17'h8000  : begin adc_a_dat <= adc_a_sum[15+15: 15];      adc_b_dat <= adc_b_sum[15+15: 15];  end
      17'h10000 : begin adc_a_dat <= adc_a_sum[15+16: 16];      adc_b_dat <= adc_b_sum[15+16: 16];  end
      default   : begin adc_a_dat <= adc_a_sum[15+0 :  0];      adc_b_dat <= adc_b_sum[15+0 :  0];  end
/*
      17'h0     : begin adc_a_dat <= adc_a_filt_out;            adc_b_dat <= adc_b_filt_out;        end
      17'h1     : begin adc_a_dat <= adc_a_sum[15+0 :  0];      adc_b_dat <= adc_b_sum[15+0 :  0];  end
      17'h8     : begin adc_a_dat <= adc_a_sum[15+3 :  3];      adc_b_dat <= adc_b_sum[15+3 :  3];  end
      17'h40    : begin adc_a_dat <= adc_a_sum[15+6 :  6];      adc_b_dat <= adc_b_sum[15+6 :  6];  end
      17'h400   : begin adc_a_dat <= adc_a_sum[15+10: 10];      adc_b_dat <= adc_b_sum[15+10: 10];  end
      17'h2000  : begin adc_a_dat <= adc_a_sum[15+13: 13];      adc_b_dat <= adc_b_sum[15+13: 13];  end
      17'h10000 : begin adc_a_dat <= adc_a_sum[15+16: 16];      adc_b_dat <= adc_b_sum[15+16: 16];  end
      default   : begin adc_a_dat <= adc_a_sum[15+0 :  0];      adc_b_dat <= adc_b_sum[15+0 :  0];  end
*/
   endcase
end

//---------------------------------------------------------------------------------
//  ADC buffer RAM

localparam READ_DELAY = (3-1);
localparam FFT_RDELAY = (7-1);
// fifo_in depth. Reads are blocked during zero-padding; peak occupancy =
// min(acq_samples, padding_duration_in_adc_cycles). The two bounds cross at
// peak = N/(1 + r*FSSR), where r = fft_clk/adc_clk: r=2 (fft_clk_sel=1, fft on
// ser_clk) gives N/(1+2*FSSR); r=1 (fft_clk_sel=0, fft on adc_clk) gives the
// LARGER N/(1+FSSR). fft_clk_sel is a runtime BUFGMUX choice, so the FIFO must
// cover the r=1 worst case: with FSSR < 1+FSSR < 2*FSSR,
// ceil(log2(N/(1+FSSR))) = FSZ - SSR_BITS, hence QSZ = FSZ - SSR_BITS (= N/FSSR
// deep). NOTE: the old FSZ-SSR_BITS-1 sized only for r=2 and OVERFLOWED at
// fft_clk_sel=0 when the acquisition started early (small wait1): the up-ramp
// samples piled up during the (N-acq)/FSSR padding beats faster than the engine
// drained them -> dropped beats -> the FFT engine under-fed and fft_done wedged
// low -> FFT hang. Sizing for r=1 makes any wait1 (incl. 0) safe.
localparam QSZ = FSZ - $clog2(FSSR);

// Scan-index queue depth (2^IQSZ) handed to both fft_proc engines. Holds one entry
// per in-flight FFT frame; production/consumption are rate-matched and flush-reset
// per 2D frame, so only the few frames between trigger and peak readout need to be
// buffered. 32 gives ~8x margin (see fifo_index in fft_proc.sv).
localparam IQSZ = 5;

// Output peak-index width: integer bin + FRAC sub-bin fractional bits (Q(FSZ).FRAC).
// Carries through the inter-channel peak-index regs and history readout (see fft_proc).
localparam IDX = FSZ + FRAC;

// DMA point-cloud packet-layout version. Single source of truth: stamped into
// the header (low nibble, via fft_proc) and reported in the descriptor reg 0x170
// so the host can sanity-check before decoding. Bump on any packet-format change.
// DMA point-cloud packet format selector (build option DMA_PER_CHAN_TAG):
//   DMA_PCT=0 -> v3 combined: one shared header per scan index, NCH-interleaved
//                data words sharing a single position. Lowest overhead (~2.4%).
//   DMA_PCT=1 -> v4 per-channel tag: each data word carries a 4-bit channel tag
//                and each channel re-anchors its own position with its own header
//                (independent/sparse-ready, 16-ch friendly). ~3.9% overhead.
// The host (dma_client.py) auto-detects the version from the packet header.
// Intensity option: each point's index word is followed by a VALUE word
// carrying the raw up/down peak amplitudes (DSZ bits each) so the host can
// derive reflectivity (distance compensation). RUNTIME-switchable via the
// dma_int_en register (0x9C bit 0); when on, the stamped format version bumps
// by +2: v3->v5 (combined), v4->v6 (tagged). The DMA_INTENSITY build option
// now only sets the register's RESET DEFAULT (the logic is always built —
// a few LUTs + the pt_v* regs).
`ifdef DMA_INTENSITY
localparam       DMA_INT_DEF     = 1'b1;
`else
localparam       DMA_INT_DEF     = 1'b0;
`endif
`ifdef DMA_PER_CHAN_TAG
localparam int   DMA_PCT         = 1;
localparam [7:0] DMA_FMT_VERSION = 8'd4;   // BASE version (+2 when intensity on)
`else
localparam int   DMA_PCT         = 0;
localparam [7:0] DMA_FMT_VERSION = 8'd3;   // BASE version (+2 when intensity on)
`endif
// Max DMA channels physically present in this build (fft_b omitted when single).
localparam [3:0] DMA_MAXCH = FFT_SINGLE ? 4'd1 : 4'd2;
// Fixed-width views of the build constants for the packet-format descriptor regs
// (truncate the untyped integer parameters to a defined width).
localparam [ 7:0] DMA_FMT_FSZ  = FSZ;             // integer bin width
localparam [ 7:0] DMA_FMT_FRAC = FRAC;            // sub-bin fractional bits (IDX = FSZ+FRAC)
localparam [ 7:0] DMA_FMT_HSZ  = HSZ;             // hist_index header field width
localparam [15:0] DMA_PKT_BLK  = HIST_BLOCK_SIZE; // data words per packet (pkt = +1 header)

logic [ ASZ-1: 0] adc_a_rd      ;
logic [ ASZ-1: 0] adc_b_rd      ;
reg   [ RSZ-1: 0] adc_wp        ;
reg   [ RSZ-1: 0] adc_a_raddr   ;
reg   [ RSZ-1: 0] adc_b_raddr   ;
reg   [ READ_DELAY: 0] adc_rval ;
wire              adc_rd_dv     ;
reg   [ FFT_RDELAY: 0] fft_rval ;
wire              fft_rd_dv     ;
reg               adc_we        ;
reg               adc_we_keep   ;
reg               adc_trig      ;

reg   [ RSZ-1: 0] adc_wp_trig   ;
reg   [ RSZ-1: 0] adc_wp_cur    ;
reg   [  32-1: 0] set_dly       ;
reg   [  32-1: 0] adc_we_cnt    ;
reg   [  32-1: 0] adc_dly_cnt   ;
reg               adc_dly_do    ;
reg    [ 20-1: 0] set_deb_len   ; // debouncing length (glitch free time after a posedge)
reg    [ 20-1: 0] set_deb_len2  ; // debouncing length (glitch free time after a posedge)

reg               triggered    ;

reg   [ 64 - 1:0] timestamp_trigger;
reg   [ 64 - 1:0] ctr_value        ;
reg   [ASZ - 1:0] pretrig_data_min; // make sure this amount of data has been acquired before trig
reg 			  pretrig_ok;

integer i;

`ifndef USE_XPM_MEMORY

reg   [ ASZ-1: 0] adc_a_buf [0:(1<<RSZ)-1] ;
reg   [ ASZ-1: 0] adc_b_buf [0:(1<<RSZ)-1] ;
always @(posedge adc_clk_i) begin
   if (adc_we && adc_dv) begin
      adc_a_buf[adc_wp] <= adc_a_dat ;
      adc_b_buf[adc_wp] <= adc_b_dat ;
   end
end

always @(posedge adc_clk_i) begin
   adc_a_raddr <= sys_addr[RSZ-1+2:2] ;
   adc_b_raddr <= sys_addr[RSZ-1+2:2] ;
   adc_a_rd    <= adc_a_buf[adc_a_raddr] ;
   adc_b_rd    <= adc_b_buf[adc_b_raddr] ;
end

`else

localparam        WRITE_DELAY = 2-1;
logic [ ASZ-1: 0] adc_a_wdata   [0:WRITE_DELAY];
logic [ ASZ-1: 0] adc_b_wdata   [0:WRITE_DELAY];
logic [ RSZ-1: 0] adc_a_waddr   [0:WRITE_DELAY];
logic [ RSZ-1: 0] adc_b_waddr   [0:WRITE_DELAY];
logic             adc_a_we      [0:WRITE_DELAY];
logic             adc_b_we      [0:WRITE_DELAY];

xpm_memory_sdpram #(
    // .MEMORY_PRIMITIVE       ("block"),
    .MEMORY_SIZE            ((1<<RSZ)*ASZ),
    .ADDR_WIDTH_A           (RSZ),
    .ADDR_WIDTH_B           (RSZ),
    .CLOCKING_MODE          ("common_clock"),
    .READ_LATENCY_B         (1),
    // .WRITE_MODE_B           ("read_first"),
    .READ_DATA_WIDTH_B      (ASZ),
    .WRITE_DATA_WIDTH_A     (ASZ),
    .BYTE_WRITE_WIDTH_A     (ASZ)
) adc_a_buf (
    .clka   (adc_clk_i),
    .addra  (adc_a_waddr[WRITE_DELAY]),
    .dina   (adc_a_wdata[WRITE_DELAY]),
    .wea    (adc_a_we[WRITE_DELAY]),
    .ena    (1'b1),
    .addrb  (adc_a_raddr),
    .doutb  (adc_a_rd),
    .rstb   (1'b0),
    .regceb (1'b1),
    .enb    (1'b1)
);

xpm_memory_sdpram #(
    // .MEMORY_PRIMITIVE       ("block"),
    .MEMORY_SIZE            ((1<<RSZ)*ASZ),
    .ADDR_WIDTH_A           (RSZ),
    .ADDR_WIDTH_B           (RSZ),
    .CLOCKING_MODE          ("common_clock"),
    .READ_LATENCY_B         (1),
    // .WRITE_MODE_B           ("read_first"),
    .READ_DATA_WIDTH_B      (ASZ),
    .WRITE_DATA_WIDTH_A     (ASZ),
    .BYTE_WRITE_WIDTH_A     (ASZ)
) adc_b_buf (
    .clka   (adc_clk_i),
    .addra  (adc_b_waddr[WRITE_DELAY]),
    .dina   (adc_b_wdata[WRITE_DELAY]),
    .wea    (adc_b_we[WRITE_DELAY]),
    .ena    (1'b1),
    .addrb  (adc_b_raddr),
    .doutb  (adc_b_rd),
    .rstb   (1'b0),
    .regceb (1'b1),
    .enb    (1'b1)
);

always @(posedge adc_clk_i) begin
    adc_a_we[0] <= adc_we && adc_dv;
    adc_b_we[0] <= adc_we && adc_dv;
    adc_a_waddr[0] <= adc_wp;
    adc_b_waddr[0] <= adc_wp;
    adc_a_wdata[0] <= adc_a_dat;
    adc_b_wdata[0] <= adc_b_dat;
    for (i=0; i<WRITE_DELAY; i+=1) begin
        adc_a_we[i+1] <= adc_a_we[i];
        adc_b_we[i+1] <= adc_b_we[i];
        adc_a_waddr[i+1] <= adc_a_waddr[i];
        adc_b_waddr[i+1] <= adc_b_waddr[i];
        adc_a_wdata[i+1] <= adc_a_wdata[i];
        adc_b_wdata[i+1] <= adc_b_wdata[i];
    end
end

always @(posedge adc_clk_i) begin
   adc_a_raddr <= sys_addr[RSZ-1+2:2]; // address synchronous to clock
   adc_b_raddr <= sys_addr[RSZ-1+2:2];
end

`endif

always @(posedge adc_clk_i) begin
   if (adc_rstn_i == 1'b0) begin
      adc_wp      <= {RSZ{1'b0}};
      adc_we      <=  1'b0      ;
      adc_wp_trig <= {RSZ{1'b0}};
	  timestamp_trigger <= 64'h0;
	  ctr_value <=         64'h0;
      adc_wp_cur  <= {RSZ{1'b0}};
      adc_we_cnt  <= 32'h0      ;
      adc_dly_cnt <= 32'h0      ;
      adc_dly_do  <=  1'b0      ;
      triggered   <=  1'b0      ;
      pretrig_data_min <=  2**RSZ - set_dly;
      pretrig_ok <= 1'b0; // goes to 1 when enough data has been acquired pretrigger
   end
   else begin
      ctr_value <= ctr_value + 1'b1;
      pretrig_data_min <= 2**RSZ - set_dly; // next line takes care of negative overflow (when set_dly > 2**RSZ)
      // ready for trigger when enough samples are acquired or trigger delay is longer than buffer duration
      pretrig_ok <= (adc_we_cnt > pretrig_data_min) || (|(set_dly[32-1:RSZ]));

      if (adc_arm_do)
         adc_we <= 1'b1 ;
      else if (((adc_dly_do || adc_trig) && (adc_dly_cnt == 32'h0) && ~adc_we_keep) || adc_rst_do) //delayed reached or reset
         adc_we <= 1'b0 ;

      // count how much data was written into the buffer before trigger
      if (adc_rst_do | adc_arm_do)
         adc_we_cnt <= 32'h0;
      if (adc_we & ~adc_dly_do & adc_dv & ~&adc_we_cnt)
         adc_we_cnt <= adc_we_cnt + 1;

      if (adc_rst_do)
         adc_wp <= {RSZ{1'b0}};
      else if (adc_we && adc_dv)
         adc_wp <= adc_wp + 1;

      if (adc_rst_do) begin
         adc_wp_trig <= {RSZ{1'b0}};
		 timestamp_trigger <= ctr_value ;
      end else if (adc_trig && !adc_dly_do && pretrig_ok) begin //last condition added to make sure pretrig data is available
         adc_wp_trig <= adc_wp_cur ; // save write pointer at trigger arrival
		 timestamp_trigger <= ctr_value ;
	  end
      if (adc_rst_do)
         adc_wp_cur <= {RSZ{1'b0}};
      else if (adc_we && adc_dv)
         adc_wp_cur <= adc_wp ; // save current write pointer

      if (adc_trig && pretrig_ok) begin
         adc_dly_do  <= 1'b1 ;
      end else if ((adc_dly_do && (adc_dly_cnt == 32'b0)) || adc_rst_do || adc_arm_do) //delayed reached or reset
         adc_dly_do  <= 1'b0 ;

      if (adc_dly_do && adc_we && adc_dv)
         adc_dly_cnt <= adc_dly_cnt - 1;
      else if (!adc_dly_do)
         adc_dly_cnt <= set_dly ;

      //trigger for fgen recording
      if (adc_trig && adc_we)
         triggered <= 1'b1     ; //communicate the precise moment of the trigger to the main module
      else if ((adc_dly_do && (adc_dly_cnt == 32'b0)) || adc_rst_do || adc_arm_do) //delayed reached or reset
         triggered <= 1'b0     ; 

   end
end

assign trig_scope_o = triggered;

always @(posedge adc_clk_i) begin
    if (adc_rstn_i == 1'b0) begin
      adc_rval <= 0;
      fft_rval <= 0;
    end else begin
      adc_rval <= {adc_rval[READ_DELAY-1:0], (sys_ren || sys_wen)};
      fft_rval <= {fft_rval[FFT_RDELAY-1:0], (sys_ren || sys_wen)};
    end
end
assign adc_rd_dv = adc_rval[READ_DELAY];
assign fft_rd_dv = fft_rval[FFT_RDELAY];


//////////////// FFT /////////////////////

logic               fft_enable;
logic               fft_trig_sync;

typedef enum {
    S_IDLE,
    S_WAIT1,
    S_FFT_UP,
    S_WAIT2,
    S_FFT_DOWN
} fft_state_t;

fft_state_t         fft_state;

logic [ FSZ-1: 0]   fft_wait1_cnt;
logic [ FSZ-1: 0]   fft_wait2_cnt;
logic [ FSZ-1: 0]   fft_acq1_cnt, fft_a_acq1_cnt, fft_b_acq1_cnt;
logic [ FSZ-1: 0]   fft_acq2_cnt, fft_a_acq2_cnt, fft_b_acq2_cnt;
logic [ FSZ-1: 0]   fft_state_cnt;
// "Active" window counts: the FSM thresholds and the fft_proc engines run off
// these; they are latched from the AXI-written (pending) fft_*_cnt only at the
// frame boundary (S_IDLE) so a write never changes a window mid-frame (which
// under-fed the FFT input FIFO and froze the engine). On an acq change the
// engine reconfigures (its conf handshake forces fft_done high during its own
// reset, so &fft_done can't gate us) -> hold S_IDLE for fft_reconf_wait cycles
// until that handshake+reset has settled before feeding the next frame.
logic [ FSZ-1: 0]   fft_wait1_cnt_act, fft_wait2_cnt_act;
logic [ FSZ-1: 0]   fft_acq1_cnt_act,  fft_acq2_cnt_act;
logic [  8-1: 0]    fft_reconf_wait;
localparam [7:0]    RECONF_CYCLES = 8'd64;  // > adc->clk_i handshake + RESET_DELAY + done resync
// Per-half "acquisition window complete" -> fft_proc, so the feed engine can
// post-pad with zeros (instead of deadlocking) if its input FIFO under-runs.
// Set when S_FFT_UP / S_FFT_DOWN ends; cleared when the next frame starts.
logic               fft_acq_up_done, fft_acq_down_done;

localparam IDX_PIPELINE = 3-1;
logic [ HSZ-1: 0]   fft_hist_index[0:IDX_PIPELINE];
logic [ HSZ-1: 0]   fft_hist_step;
logic [ RSZ-1: 0]   x_step;
logic [ RSZ-1: 0]   y_step;

logic [ IDX-1: 0]   fft_hist_rdata_up_a, fft_hist_rdata_up_a_;
logic [ IDX-1: 0]   fft_hist_rdata_down_a, fft_hist_rdata_down_a_;
logic [ IDX-1: 0]   fft_hist_rdata_up_b, fft_hist_rdata_up_b_;
logic [ IDX-1: 0]   fft_hist_rdata_down_b, fft_hist_rdata_down_b_;

logic [ 16-1:  0]   fft_wp_last_a;
logic [ 16-1:  0]   fft_wp_last_b;

logic [ DSZ-1: 0]   fft_rdata_up_a, fft_rdata_up_a_;
logic [ DSZ-1: 0]   fft_rdata_down_a, fft_rdata_down_a_;
logic [ DSZ-1: 0]   fft_rdata_up_b, fft_rdata_up_b_;
logic [ DSZ-1: 0]   fft_rdata_down_b, fft_rdata_down_b_;

logic [ QSZ-1:0]    fft_q_wp_a;
logic [ QSZ-1:0]    fft_q_rp_a;
logic [ 32-1: 0]    fft_overflow_cnt;
logic [ 16-1: 0]    fft_threshold_k, fft_a_threshold_k, fft_b_threshold_k;
logic [ FSZ-1:0]    fft_peak_start, fft_a_peak_start, fft_b_peak_start;
logic [ DSZ-1: 0]   fft_peak_minimum, fft_a_peak_minimum, fft_b_peak_minimum;
// CA-CFAR moving-window params (used only by the PEAK_CFAR peak detector build;
// harmlessly unconnected otherwise). guard/train cells each side of the peak.
logic [ FSZ-1:0]    fft_cfar_guard, fft_a_cfar_guard, fft_b_cfar_guard;
logic [ FSZ-1:0]    fft_cfar_train, fft_a_cfar_train, fft_b_cfar_train;

logic [ 6-1 :  0]   fft_status[0:1];
logic [ 2-1 :  0]   fft_done;
// (* mark_debug = "true" *)
logic [ IDX-1: 0]   fft_peak_index_up_a;
logic [ IDX-1: 0]   fft_peak_index_down_a;
logic [ IDX-1: 0]   fft_peak_index_up_b;
logic [ IDX-1: 0]   fft_peak_index_down_b;
logic [ DSZ-1: 0]   fft_peak_up_a;
logic [ DSZ-1: 0]   fft_peak_down_a;
logic [ DSZ-1: 0]   fft_peak_up_b;
logic [ DSZ-1: 0]   fft_peak_down_b;

// Per-position point output from each fft_proc (on fft_input_clk); the combined
// DMA packet assembler below interleaves both channels under one header.
logic               dma_point_valid_a, dma_point_valid_b;
logic [ IDX-1: 0]   dma_point_up_a, dma_point_down_a;
logic [ IDX-1: 0]   dma_point_up_b, dma_point_down_b;
logic [ DSZ-1: 0]   dma_point_val_up_a, dma_point_val_down_a;
logic [ DSZ-1: 0]   dma_point_val_up_b, dma_point_val_down_b;
logic [ HSZ-1: 0]   dma_point_idx_a, dma_point_idx_b;
// NCH = number of active DMA channels (0=off, 1=ch0, 2=ch0+ch1). Set via the
// control register; bounded to the channels physically present.
logic [ 3:0]        dma_nch;
logic               dma_int_en;   // runtime intensity enable (sys reg 0x9C bit 0)

// Sub-bin FRACTION of each peak index (low FRAC bits of k_interp), zero-extended to
// 16 and packed {down, up} per channel for the 0x174/0x178 registers (down in [31:16],
// up in [15:0]). Guarded for FRAC=0 (interpolation disabled) where the [FRAC-1:0]
// select would be illegal — then the registers read 0.
logic [ 32-1: 0]    fft_peak_frac_a;
logic [ 32-1: 0]    fft_peak_frac_b;
generate
if (FRAC > 0) begin : g_peak_frac
   assign fft_peak_frac_a = {{16-FRAC{1'b0}}, fft_peak_index_down_a[FRAC-1:0],
                             {16-FRAC{1'b0}}, fft_peak_index_up_a[FRAC-1:0]};
   assign fft_peak_frac_b = {{16-FRAC{1'b0}}, fft_peak_index_down_b[FRAC-1:0],
                             {16-FRAC{1'b0}}, fft_peak_index_up_b[FRAC-1:0]};
end else begin : g_peak_frac_off
   assign fft_peak_frac_a = 32'h0;
   assign fft_peak_frac_b = 32'h0;
end
endgenerate

logic [ 32-1: 0]    fft_point_cnt;
logic [ 32-1: 0]    fft_scan_point_cnt;
logic [ 32-1: 0]    fft_we_cnt[1:0];
logic [ 32-1: 0]    fft_skip_cnt;
// logic               fft_index_flush = adc_rstn_i == 1'b0 || sync_rst_i;
logic               fft_index_flush = sync_rst_i;
// One-cycle pulse per completed RASTER: on the scan's wrap back to origin
// (0,0) and, with a ping-pong (zigzag) slow axis, also at the far slow-axis
// turnaround — detected as the scan dwelling on the SAME non-origin cell for
// two consecutive points (both ASG axes double their endpoints there, so the
// far-corner cell legitimately repeats; nowhere else does a running scan
// repeat a cell). Without the second pulse a zigzag frame_cnt spanned the
// full up+down ping-pong, so the host's turnover publish fired only once per
// TWO visual passes and its interval flushes landed mid-sweep, mixing the two
// opposite-direction rasters (the swinging-block artifact, see
// HANDOFF_zigzag_column_shift.md). fft_index_flush is only the ASG sync
// reset. Drives the DMA header frame_cnt (asm_frame_cnt) so the host sees
// frame turnover. Generated in the adc_clk index pipeline below and consumed
// in the fft_input_clk ASM FSM (same/related clock, CLK_SEL=0), like
// fft_index_flush.
logic               fft_frame_start;
logic               fft_rep_d;   // previous sample already repeated its predecessor

logic               fft_peak_ready_a;
logic               fft_peak_ready_b;
logic [ 2-1 : 0]    fft_peak_ready;

logic [ 2-1:  0]    fft_rstn;
logic               fft_rstn_i;

always @(posedge adc_clk_i) begin
    if  ((fft_trig_sync && adc_rst_do) || sync_rst_i || !adc_rstn_i) begin
        fft_rstn <= 0;
    end else
        fft_rstn <= {fft_rstn[0], 1'b1};
end

// assign fft_rstn_i = adc_rstn_i && ~|fft_rst_i;
assign fft_rstn_i = fft_rstn[1];

logic fft_trig;
logic fft_trig_i = fft_trig_sync ? (adc_trig && !adc_dly_do && pretrig_ok) : fft_trig;

// (* mark_debug = "true" *)
logic fft_dvalid = (!fft_trig_sync || adc_we) && adc_dv; 
logic fft_up = fft_state == S_FFT_UP;
logic fft_down = fft_state == S_FFT_DOWN;
// export the acq windows for PID gating (EO-PLL locks only inside them)
assign fft_window_o = {fft_down, fft_up};

localparam IDXSZ = 8;
localparam IHSZ = 10;

// `define DEBUG_FFT_INDEX

`ifdef DEBUG_FFT_INDEX
    logic [ IDXSZ-1 :0]  fft_indices_x[0:(1<<IHSZ)-1];
    logic [ IDXSZ-1 :0]  fft_indices_y[0:(1<<IHSZ)-1];
    logic [ 32-1    :0]  fft_indices[0:(1<<IHSZ)-1];
`endif

logic [ IHSZ-1   :0]  fft_index_raddr1;
logic [ IHSZ-1   :0]  fft_index_raddr2;
logic [ 32-1     :0]  fft_index_rdata;
logic [ 32-1     :0]  fft_index_rdata2;
logic [ IHSZ-1   :0]  fft_indices_pos;

logic [ IDX_PIPELINE+2 :0]    fft_index_valid;

`ifdef DEBUG_FFT_INDEX
logic [8-1 : 0] y_step_cnt;
`endif


always @(posedge adc_clk_i)
if (fft_index_flush) begin
   fft_hist_step <= 0;
   fft_indices_pos <= 1;
   fft_index_valid <= {IDX_PIPELINE+2{1'b0}};
   x_step <= 0;
   y_step <= 0;
   fft_frame_start <= 1'b0;
   fft_rep_d <= 1'b0;
   `ifdef DEBUG_FFT_INDEX
      y_step_0 <= 0;
   `endif
end else begin
    fft_frame_start <= 1'b0;   // default: pulse only on the raster boundaries below
    fft_index_valid = {fft_index_valid[IDX_PIPELINE+1: 0], fft_trig_i && &fft_done};

   `ifdef DEBUG_FFT_INDEX
       if (fft_index_valid[IDX_PIPELINE+2]) begin
           y_step_0 <= 1;
           y_step_cnt <= 0;
       end
   `endif

    if (fft_index_valid[0]) begin
        if (y_step_i == 0 && x_step_i == 0) begin
            fft_indices_pos <= 0;
            fft_hist_step <= 0;
            // New 2D frame: pulse only on a real WRAP into origin (previous cell
            // x_step/y_step non-origin), not while dwelling at origin (advance=0
            // stalled scan re-hits (0,0) each point), which would over-count frames.
            if (x_step != 0 || y_step != 0)
                fft_frame_start <= 1'b1;
        end else begin
            if (fft_hist_step < x_step_i + 1)
                fft_hist_step <= x_step_i + 1;
            // Mid-ping-pong raster boundary (zigzag slow axis): the far slow
            // turnaround dwells on the same cell for two consecutive points
            // (see the fft_frame_start declaration). Qualified to y!=0 (the
            // real far turnaround always is; a y==0 line is where the stride
            // is still growing, so every ascending cell transiently looks
            // like an endpoint) and to a fast-axis ENDPOINT cell (x==0 or
            // x==stride-1; fft_hist_step holds the old stride = x_max+1
            // here), so a scan stalling mid-line (a missed scan one-shot)
            // cannot fake a boundary — only a stall exactly on a line-end
            // cell can, and fft_rep_d bounds any stall/parked scan to ONE
            // extra tick.
            if (y_step_i != 0 &&
                    x_step_i == x_step && y_step_i == y_step && !fft_rep_d &&
                    (x_step_i == 0 || x_step_i + 1 == fft_hist_step))
                fft_frame_start <= 1'b1;
        end

        fft_rep_d <= (x_step_i == x_step) && (y_step_i == y_step);
        x_step <= x_step_i;
        y_step <= y_step_i;
    end

    `ifdef DEBUG_FFT_INDEX
        if (y_step_0) begin
            if (y_step_cnt == 128)
                y_step_0 <= 0;
            else
                y_step_cnt = y_step_cnt + 1;
        end
    `endif

    // Absolute scan cell = row(y_step) * row_stride(fft_hist_step) + col(x_step).
    // WIDTH INVARIANT: fft_hist_step is HSZ-wide (not RSZ) ON PURPOSE — it forces
    // this multiply to evaluate at the HSZ-bit index width, so the product fills
    // the full 2^HSZ cell space. Narrowing it to RSZ would truncate y*stride to
    // RSZ bits *before* this assignment and silently re-wrap the index at 2^RSZ
    // (the old HSZ==RSZ==14 coincidence hid this). x_step/y_step stay RSZ (one
    // axis <= 2^RSZ steps); only the combined cell index needs the HSZ range.
    //
    // TIMING/SIZING: single-cycle is sufficient — this infers ONE registered
    // DSP48E1 MACC (A=fft_hist_step, B=y_step, C=x_step -> P-reg). A DSP's mult
    // delay is fixed by the 25x18 array, so HSZ 14->24 leaves the path delay (and
    // its >0.7 ns slack on the 8 ns / 125 MHz adc_clk) essentially unchanged; no
    // extra pipeline stage is needed. Keep HSZ <= 24 (unsigned A on the 25-bit
    // signed port) to stay in ONE DSP; HSZ >= 25 cascades to two DSPs (still meets
    // timing). fft_hist_index[0]->[1]->[2] are alignment delays, NOT mult pipeline.
    fft_hist_index[0] <= y_step * fft_hist_step + x_step;
    for (int i=0; i<IDX_PIPELINE; i=i+1)
        fft_hist_index[i+1] <= fft_hist_index[i];

    `ifdef DEBUG_FFT_INDEX
        if (fft_index_valid[IDX_PIPELINE+2]) begin
            fft_indices_x[fft_indices_pos] <= x_step;
            fft_indices_y[fft_indices_pos] <= y_step;
            fft_indices[fft_indices_pos] <= fft_hist_index[IDX_PIPELINE];
            fft_indices_pos <= fft_indices_pos + 1;
        end
    `endif
end

`ifdef DEBUG_FFT_INDEX
always @(posedge adc_clk_i) begin
   fft_index_raddr1 <= sys_addr[IHSZ-1+2:2] ;
   fft_index_raddr2  <= fft_index_raddr1;
   fft_index_rdata <= {{16-IDXSZ{1'b0}}, fft_indices_x[fft_index_raddr1], {16-IDXSZ{1'b0}}, fft_indices_y[fft_index_raddr1]};
   fft_index_rdata2 <= fft_indices[fft_index_raddr1];
end
`endif

// Runtime FFT length is only for the LogiCORE IP (FFT_IMPL==1) or the explicit
// FFT_RUNTIME_NFFT opt-in. Otherwise fft_nfft is fixed at FSZ (the sysbus write at
// 0x88 is compiled out below), so fft_conf_data / fft_wp_last fold to constants.
`ifdef FFT_RUNTIME_NFFT
localparam FFT_RT_DEF = 1;
`else
localparam FFT_RT_DEF = 0;
`endif
localparam RUNTIME_NFFT = (FFT_IMPL == 1) || FFT_RT_DEF;

logic [ 5-1: 0] fft_nfft = FSZ;
logic           fft_parallel;
logic           fft_fwd_inv = 1;
logic [16-1: 0] fft_conf_data = {{7-1{1'b0}}, fft_fwd_inv, {3-1{1'b0}}, fft_nfft};

assign fft_wp_last_a = (1<<fft_nfft)-1;
assign fft_wp_last_b = (1<<fft_nfft)-1;

logic           fft_clk_sel_i, fft_clk_sel;

always @(posedge adc_clk_i)
if (adc_rstn_i == 1'b0) begin
    fft_clk_sel_i <= 0;
end else begin
    fft_clk_sel_i <= fft_clk_sel;
end

BUFGMUX clk_sel (
    .O  (fft_input_clk),
    .I0 (adc_clk_i),
    .I1 (fft_clk_i),
    .S  (fft_clk_sel_i)
);

assign fft_rdata_up_a        = fft_rdata_up_a_;
assign fft_rdata_up_b        = fft_rdata_up_b_;
assign fft_rdata_down_b      = fft_rdata_down_b_;
assign fft_rdata_down_a      = fft_parallel ? fft_rdata_up_b_      : fft_rdata_down_a_;
assign fft_hist_rdata_up_a   = fft_hist_rdata_up_a_;
assign fft_hist_rdata_up_b   = fft_hist_rdata_up_b_;
assign fft_hist_rdata_down_b = fft_hist_rdata_down_b_;
assign fft_hist_rdata_down_a = fft_parallel ? fft_hist_rdata_up_b_ : fft_hist_rdata_down_a_;

always @(posedge adc_clk_i) begin
    fft_a_acq1_cnt <= fft_acq1_cnt_act;
    fft_b_acq1_cnt <= fft_acq1_cnt_act;
    fft_a_acq2_cnt <= fft_acq2_cnt_act;
    fft_b_acq2_cnt <= fft_acq2_cnt_act;
    fft_a_peak_start <= fft_peak_start;
    fft_b_peak_start <= fft_peak_start;
    fft_a_peak_minimum <= fft_peak_minimum;
    fft_b_peak_minimum <= fft_peak_minimum;
    fft_a_threshold_k <= fft_threshold_k;
    fft_b_threshold_k <= fft_threshold_k;
    fft_a_cfar_guard <= fft_cfar_guard;
    fft_b_cfar_guard <= fft_cfar_guard;
    fft_a_cfar_train <= fft_cfar_train;
    fft_b_cfar_train <= fft_cfar_train;
end

fft_proc #(.ASZ(ASZ),
           .QSZ(QSZ),
           .IQSZ(IQSZ),
           .DSZ(DSZ),
           .FSZ(FSZ),
           .FRAC(FRAC),
           .RSZ(RSZ),
           .HSZ(HSZ),
           .FSSR(FSSR),
           .FFT_IMPL(FFT_IMPL),
           .READ_DELAY(FFT_RDELAY-2),
           .HIST_BLOCK_SIZE(HIST_BLOCK_SIZE),
           .CHANNEL_ID(4'd0),
           .DMA_FMT_VERSION(DMA_FMT_VERSION[3:0]))
fft_a (
   .adc_clk_i (adc_clk_i),
   .adc_rstn_in (fft_rstn_i),

   .clk_i (fft_input_clk),

   .data_in (adc_a_dat),
   .enable_in (fft_up || (fft_down && !fft_parallel)),
   .dvalid_in (fft_dvalid),
   .trig_in (fft_trig_i),

   .fft_parallel_in (fft_parallel),

   .fft_threshold_k_in (fft_a_threshold_k),
   .fft_peak_start_in (fft_a_peak_start),
   .fft_peak_minimum_in (fft_a_peak_minimum),
   .fft_cfar_guard_in (fft_a_cfar_guard),
   .fft_cfar_train_in (fft_a_cfar_train),

   .fft_acq_up_in (fft_a_acq1_cnt),
   .fft_acq_down_in (fft_a_acq2_cnt),

   .fft_acq_up_done_in (fft_acq_up_done),
   .fft_acq_down_done_in (fft_acq_down_done),

   .sys_addr_in (sys_addr),

   .fft_rdata_up_o (fft_rdata_up_a_),
   .fft_rdata_down_o (fft_rdata_down_a_),

   .fft_index_flush_in (fft_index_flush),
   .fft_index_valid_in (fft_index_valid[IDX_PIPELINE+2]),
   .fft_hist_index_in (fft_hist_index[IDX_PIPELINE]),

   .fft_hist_rdata_up_o (fft_hist_rdata_up_a_),
   .fft_hist_rdata_down_o (fft_hist_rdata_down_a_),

   .dma_point_valid_o (dma_point_valid_a),
   .dma_point_up_o    (dma_point_up_a),
   .dma_point_down_o  (dma_point_down_a),
   .dma_point_val_up_o   (dma_point_val_up_a),
   .dma_point_val_down_o (dma_point_val_down_a),
   .dma_point_idx_o   (dma_point_idx_a),

   .status_o (fft_status[0]),
   .fft_done_o (fft_done[0]),
   .fft_peak_ready_o (fft_peak_ready_a),
   .fft_peak_index_up_o (fft_peak_index_up_a[IDX-1:0]),
   .fft_peak_index_down_o (fft_peak_index_down_a[IDX-1:0]),
   .fft_peak_value_up_o (fft_peak_up_a),
   .fft_peak_value_down_o (fft_peak_down_a),
   .point_cnt_o (fft_point_cnt),
   .scan_point_cnt_o (fft_scan_point_cnt),

   .fft_we_cnt (fft_we_cnt[0]),

   .fft_conf_data_in (fft_conf_data),

   .overflow_cnt_o (fft_overflow_cnt)
);

if (FFT_SINGLE) begin : gen_no_fft_b
   // Single-FFT build (e.g. SSR=8): fft_b is omitted to fit the device. Tie off
   // all of its outputs so the surrounding logic still functions on fft_a alone.
   // done/peak_ready forced high so the &-reductions depend only on channel A.
   assign fft_rdata_up_b_        = '0;
   assign fft_rdata_down_b_      = '0;
   assign fft_hist_rdata_up_b_   = '0;
   assign fft_hist_rdata_down_b_ = '0;
   assign fft_status[1]          = '0;
   assign fft_done[1]            = 1'b1;
   assign fft_peak_ready_b       = 1'b1;
   assign fft_peak_index_up_b    = '0;
   assign fft_peak_index_down_b  = '0;
   assign fft_peak_up_b          = '0;
   assign fft_peak_down_b        = '0;
   assign fft_we_cnt[1]          = '0;
   assign dma_point_valid_b      = 1'b0;
   assign dma_point_up_b         = '0;
   assign dma_point_down_b       = '0;
   assign dma_point_val_up_b     = '0;
   assign dma_point_val_down_b   = '0;
   assign dma_point_idx_b        = '0;
end else begin : gen_fft_b
fft_proc #(.ASZ(ASZ),
           .QSZ(QSZ),
           .IQSZ(IQSZ),
           .DSZ(DSZ),
           .FSZ(FSZ),
           .FRAC(FRAC),
           .RSZ(RSZ),
           .HSZ(HSZ),
           .FSSR(FSSR),
           .FFT_IMPL(FFT_IMPL),
           .READ_DELAY(FFT_RDELAY-2),
           .HIST_BLOCK_SIZE(HIST_BLOCK_SIZE),
           .CHANNEL_ID(4'd1),
           .DMA_FMT_VERSION(DMA_FMT_VERSION[3:0])
) fft_b (
   .adc_clk_i (adc_clk_i),
   .adc_rstn_in (fft_rstn_i),

   .clk_i (fft_input_clk),

   .data_in (fft_parallel ? adc_a_dat : adc_b_dat),
   .enable_in ((!fft_parallel && fft_up) || fft_down),
   .dvalid_in (fft_dvalid),
   .trig_in (fft_trig_i),

   .fft_parallel_in (fft_parallel),

   .fft_threshold_k_in (fft_b_threshold_k),
   .fft_peak_start_in (fft_b_peak_start),
   .fft_peak_minimum_in (fft_b_peak_minimum),
   .fft_cfar_guard_in (fft_b_cfar_guard),
   .fft_cfar_train_in (fft_b_cfar_train),

   .fft_acq_up_in (fft_parallel ? fft_b_acq2_cnt : fft_b_acq1_cnt),
   .fft_acq_down_in (fft_b_acq2_cnt),

   .fft_acq_up_done_in (fft_parallel ? fft_acq_down_done : fft_acq_up_done),
   .fft_acq_down_done_in (fft_acq_down_done),

   .sys_addr_in (sys_addr),

   .fft_rdata_up_o (fft_rdata_up_b_),
   .fft_rdata_down_o (fft_rdata_down_b_),

   .fft_index_flush_in (fft_index_flush),
   .fft_index_valid_in (fft_index_valid[IDX_PIPELINE+2]),
   .fft_hist_index_in (fft_hist_index[IDX_PIPELINE]),

   .fft_hist_rdata_up_o (fft_hist_rdata_up_b_),
   .fft_hist_rdata_down_o (fft_hist_rdata_down_b_),

   .dma_point_valid_o (dma_point_valid_b),
   .dma_point_up_o    (dma_point_up_b),
   .dma_point_down_o  (dma_point_down_b),
   .dma_point_val_up_o   (dma_point_val_up_b),
   .dma_point_val_down_o (dma_point_val_down_b),
   .dma_point_idx_o   (dma_point_idx_b),

   .status_o (fft_status[1]),
   .fft_done_o (fft_done[1]),
   .fft_peak_ready_o (fft_peak_ready_b),
   .fft_peak_index_up_o (fft_peak_index_up_b[IDX-1:0]),
   .fft_peak_index_down_o (fft_peak_index_down_b[IDX-1:0]),
   .fft_peak_value_up_o (fft_peak_up_b),
   .fft_peak_value_down_o (fft_peak_down_b),

   // .point_cnt_o (fft_point_cnt),
   .fft_we_cnt (fft_we_cnt[1]),

   .fft_conf_data_in (fft_conf_data)
);
end

// ===========================================================================
// Combined point-cloud DMA assembler (packet format v2).
// Reads both fft_proc channels at the shared scan position and emits ONE
// segmented packet stream. Format (64-bit words):
//   header [63]=1: [62:59]=NCH (active channels), [58:55]=version,
//                  [54 : 55-HSZ]=hist_index, [54-HSZ : 0]=frame_cnt
//   data   [63]=0: [62]=advance (set on the ch0 word only), [2*IDX-1:IDX]=down,
//                  [IDX-1:0]=up.  One group per scan index = NCH data words
//                  (ch0, ch1, ...); the host steps position on the ch0 advance.
//   value  [63]=0 (intensity builds only, v5/v6): follows its channel's data
//                  word; [2*DSZ-1:DSZ]=down amplitude, [DSZ-1:0]=up amplitude
//                  (raw peak values; advance/dir bits 0, v6 keeps the ch tag).
//                  The host tells data from value words by position (idx/value
//                  alternate after a header; groups never straddle packets).
//   sentinel = all-ones (header bit + NCH nibble 0xF): pads a packet tail too
//              small to hold a whole group, so a group never straddles packets.
// A header is (re)emitted on packet start, scan-position jump (delta not 0/1),
// 2D-frame change, or NCH change. Runs on fft_input_clk; xpm_fifo_axis crosses
// to adc_clk (dma_s2mm). The two fft_proc point strobes are cycle-aligned, so
// dma_point_valid_a gates both channels.
localparam int ASM_PKT  = HIST_BLOCK_SIZE + 1;       // words per packet
localparam int ASM_WCW  = $clog2(ASM_PKT);
localparam int ASM_FCW  = 55 - HSZ;                  // frame-counter bits in header
localparam int ASM_RSVD = 62 - 2*IDX;                // data-word reserved span
// Per-packet (datagram) sequence stamped in the LOW DMA_SEQ_W bits of the header
// frame_cnt field; the host strips them and gap-counts -> OS-independent packet
// drop detection (Windows has no SO_RX_QUEUE_OVFL). frame_cnt keeps the high
// (ASM_FCW-DMA_SEQ_W) bits — turnover detection only needs inequality. Requires
// DMA_SEQ_W < ASM_FCW (true for HSZ <= 38). Advertised in descriptor 0x194[27:20].
localparam int DMA_SEQ_W = 16;
localparam [1:0] S_HDR = 2'd0, S_DATA = 2'd1, S_PAD = 2'd2, S_VAL = 2'd3;
// The value word packs {down,up} amplitudes below the flag/tag bits: 2*DSZ must
// fit in 61 (combined) / 57 (tagged) payload bits or the size-cast would
// truncate. Checked unconditionally — the intensity path is always built.
generate
if (2*DSZ > (DMA_PCT ? 57 : 61)) begin : gen_dma_int_width_check
    $error("DMA intensity value word: 2*DSZ exceeds the payload (DSZ <= %0d required)",
           (DMA_PCT ? 57 : 61)/2);
end
endgenerate

// Runtime intensity enable, adc domain -> FFT clock (quasi-static level).
// The FSM samples it only at PACKET boundaries (asm_int below) so one packet
// never mixes formats — the host detects the version per packet.
logic dma_int_clk;
xpm_cdc_single #(
    .DEST_SYNC_FF (2)
) dma_int_sync (
    .src_clk   (adc_clk_i),
    .src_in    (dma_int_en),
    .dest_clk  (fft_input_clk),
    .dest_out  (dma_int_clk)
);

logic [ASM_WCW-1:0] asm_wc;          // next word index within the packet
logic               asm_need_hdr;
logic [HSZ-1:0]     asm_prev_idx;
logic [3:0]         asm_nch_seg;     // NCH stamped in the active segment header
logic               asm_flush_d, asm_frame_pend;
logic [ASM_FCW-1:0] asm_frame_cnt;       // per-2D-frame counter (header field width)
logic [DMA_SEQ_W-1:0] asm_pkt_seq;       // per-packet (datagram) sequence, header low bits
logic               asm_busy;
logic [1:0]         asm_state;
logic [3:0]         asm_ch;
logic               asm_int;         // intensity format for the CURRENT packet
                                     // (dma_int_clk sampled at packet boundaries)
logic [IDX-1:0]     pt_up0, pt_dn0, pt_up1, pt_dn1;
logic [DSZ-1:0]     pt_vu0, pt_vd0, pt_vu1, pt_vd1;   // raw peak amplitudes (intensity builds)
logic [HSZ-1:0]     pt_idx;
logic               pt_adv;       // step magnitude (1 = ±1 advance, 0 = hold)
logic               pt_dir;       // step direction (1 = -1 backward, 0 = +1 forward)
logic               pt_hdr;       // v4 tagged: this index needs a (per-channel) header
logic [3:0]         pt_nch;
logic [63:0]        asm_tdata;
logic               asm_tvalid, asm_tlast;

wire [HSZ-1:0] asm_delta = dma_point_idx_a - asm_prev_idx;
wire asm_inc = (asm_delta == 1);              // scan stepped +1
wire asm_dec = (asm_delta == {HSZ{1'b1}});    // scan stepped -1 (two's-complement all-ones)
// Frame boundary for the DMA header frame_cnt: the origin-wrap pulse (a real
// per-2D-frame event), NOT fft_index_flush (which is only the ASG sync reset and
// so never ticked during continuous scanning — frame_cnt was stuck).
wire asm_flush_rise = fft_frame_start && !asm_flush_d;
wire asm_last_word  = (asm_wc == ASM_PKT-1);
// A header is needed unless the scan held (delta 0) or stepped by ±1 (which the
// signed advance bits encode). The signed step lets BOTH scan directions ride a
// single segment, instead of a re-anchor header per point when the scan runs
// downward.
wire asm_hdr_need   = asm_need_hdr || asm_frame_pend || asm_flush_rise ||
                      (asm_delta != 0 && !asm_inc && !asm_dec) ||
                      (dma_nch != asm_nch_seg);

always @(posedge fft_input_clk) begin
    asm_tvalid <= 1'b0;
    if (!fft_rstn_i) begin
        asm_wc         <= '0;
        asm_need_hdr   <= 1'b1;
        asm_prev_idx   <= '0;
        asm_nch_seg    <= '0;
        asm_frame_pend <= 1'b0;
        asm_frame_cnt  <= '0;
        asm_pkt_seq    <= '0;
        asm_flush_d    <= 1'b0;
        asm_busy       <= 1'b0;
        asm_tlast      <= 1'b0;
        asm_int        <= dma_int_clk;
    end else begin
        asm_flush_d <= fft_frame_start;
        if (asm_flush_rise) begin
            asm_frame_cnt  <= asm_frame_cnt + 1'b1;
            asm_frame_pend <= 1'b1;
        end

        if (!asm_busy) begin
            if (dma_point_valid_a && dma_nch != 0) begin
                pt_up0 <= dma_point_up_a;  pt_dn0 <= dma_point_down_a;
                pt_up1 <= dma_point_up_b;  pt_dn1 <= dma_point_down_b;
                pt_vu0 <= dma_point_val_up_a;  pt_vd0 <= dma_point_val_down_a;
                pt_vu1 <= dma_point_val_up_b;  pt_vd1 <= dma_point_val_down_b;
                pt_idx <= dma_point_idx_a;
                pt_nch <= dma_nch;
                pt_adv <= asm_inc || asm_dec;     // ±1 step rides the advance bits
                pt_dir <= asm_dec;                // 1 = backward (-1)
                pt_hdr <= asm_hdr_need;           // v4: each channel re-anchors on a jump
                asm_frame_pend <= 1'b0;
                asm_busy <= 1'b1;
                // Reserve the worst-case index span so it never straddles a packet:
                //   v3/v5 combined = 1 shared header + NCH*(1 or 2) data words
                //   v4/v6 tagged   = NCH headers + NCH*(1 or 2) data words
                // (asm_int is stable within the packet, so the reservation and
                // the words actually emitted always agree.)
                if ((asm_wc + (DMA_PCT ? (asm_int ? 3 : 2)*dma_nch
                                       : (1 + (asm_int ? 2 : 1)*dma_nch))) > ASM_PKT)
                    asm_state <= S_PAD;
                else begin
                    asm_state <= asm_hdr_need ? S_HDR : S_DATA;
                    asm_ch    <= '0;
                end
            end
        end else case (asm_state)
            S_PAD: begin
                asm_tdata  <= {64{1'b1}};            // all-ones sentinel
                asm_tvalid <= 1'b1;
                asm_tlast  <= asm_last_word;
                if (asm_last_word) begin
                    asm_wc       <= '0;
                    asm_pkt_seq  <= asm_pkt_seq + 1'b1;   // packet boundary
                    asm_int      <= dma_int_clk;          // format may switch here
                    asm_need_hdr <= 1'b1;
                    asm_state    <= S_HDR;           // new packet -> header
                    asm_ch       <= '0;
                    pt_hdr       <= 1'b1;            // v4: re-anchor every channel
                    pt_adv       <= 1'b0;            // v3: point re-anchored at header
                    pt_dir       <= 1'b0;
                end else
                    asm_wc <= asm_wc + 1'b1;
            end
            S_HDR: begin   // header field: v3 = NCH (one shared header),
                           //               v4 = channel tag (one header per channel)
                // frame_cnt field = { frame_cnt[high ASM_FCW-DMA_SEQ_W bits],
                //                     per-packet seq[DMA_SEQ_W bits] } (host splits it).
                asm_tdata    <= {1'b1, (DMA_PCT ? asm_ch : pt_nch),
                                 4'(DMA_FMT_VERSION[3:0] + (asm_int ? 4'd2 : 4'd0)),
                                 pt_idx,
                                 asm_frame_cnt[ASM_FCW-DMA_SEQ_W-1:0], asm_pkt_seq};
                asm_tvalid   <= 1'b1;
                asm_tlast    <= asm_last_word;
                asm_wc       <= asm_last_word ? '0 : asm_wc + 1'b1;
                if (asm_last_word) begin
                    asm_pkt_seq <= asm_pkt_seq + 1'b1;   // packet boundary
                    asm_int     <= dma_int_clk;
                end
                asm_need_hdr <= asm_last_word;
                asm_nch_seg  <= pt_nch;
                asm_prev_idx <= pt_idx;
                asm_state    <= S_DATA;             // emit data next
                if (!DMA_PCT) begin                 // v3: data starts at channel 0,
                    asm_ch <= '0;                   //     pinned to the header idx
                    pt_adv <= 1'b0;
                    pt_dir <= 1'b0;
                end
            end
            S_DATA: begin
                // v3 combined: advance rides the channel-0 word only (shared pos);
                // v4 tagged:   every word carries its channel tag and (after its own
                //              header, pt_hdr) sits on the anchor with advance 0.
                if (DMA_PCT)
                    asm_tdata <= {1'b0, (pt_hdr ? 1'b0 : pt_adv),
                                  (pt_hdr ? 1'b0 : pt_dir),
                                  asm_ch,                         // 4-bit channel tag
                                  {(ASM_RSVD-5){1'b0}},
                                  (asm_ch == 0 ? pt_dn0 : pt_dn1),
                                  (asm_ch == 0 ? pt_up0 : pt_up1)};
                else
                    asm_tdata <= {1'b0, (asm_ch == 0 ? pt_adv : 1'b0),
                                  (asm_ch == 0 ? pt_dir : 1'b0),
                                  {(ASM_RSVD-1){1'b0}},
                                  (asm_ch == 0 ? pt_dn0 : pt_dn1),
                                  (asm_ch == 0 ? pt_up0 : pt_up1)};
                asm_tvalid <= 1'b1;
                asm_tlast  <= asm_last_word;
                asm_wc     <= asm_last_word ? '0 : asm_wc + 1'b1;
                if (asm_last_word) begin
                    asm_need_hdr <= 1'b1;
                    asm_pkt_seq  <= asm_pkt_seq + 1'b1;          // packet boundary
                    asm_int      <= dma_int_clk;
                end
                asm_prev_idx <= pt_idx;                          // shared position
                if (asm_int)
                    asm_state <= S_VAL;                          // v5/v6: value word next
                else if (asm_ch + 1 >= pt_nch) asm_busy <= 1'b0; // index done
                else begin
                    asm_ch <= asm_ch + 1'b1;
                    if (DMA_PCT) asm_state <= pt_hdr ? S_HDR : S_DATA;  // v4: re-header next ch on jump
                end                                              // v3: stays in S_DATA
            end
            S_VAL: begin   // intensity builds (v5/v6): raw up/down peak amplitudes
                           // for the channel whose index word was just sent.
                           // No advance/dir (position already stepped); v6 keeps
                           // the channel tag so the word routes like its index word.
                if (DMA_PCT)
                    asm_tdata <= {1'b0, 2'b00, asm_ch,
                                  57'({(asm_ch == 0 ? pt_vd0 : pt_vd1),
                                       (asm_ch == 0 ? pt_vu0 : pt_vu1)})};
                else
                    asm_tdata <= {1'b0, 2'b00,
                                  61'({(asm_ch == 0 ? pt_vd0 : pt_vd1),
                                       (asm_ch == 0 ? pt_vu0 : pt_vu1)})};
                asm_tvalid <= 1'b1;
                asm_tlast  <= asm_last_word;
                asm_wc     <= asm_last_word ? '0 : asm_wc + 1'b1;
                if (asm_last_word) begin
                    asm_need_hdr <= 1'b1;
                    asm_pkt_seq  <= asm_pkt_seq + 1'b1;          // packet boundary
                    asm_int      <= dma_int_clk;
                end
                if (asm_ch + 1 >= pt_nch) asm_busy <= 1'b0;      // index done
                else begin
                    asm_ch    <= asm_ch + 1'b1;
                    asm_state <= (DMA_PCT && pt_hdr) ? S_HDR : S_DATA;  // v6: re-header next ch on jump
                end
            end
            default: asm_busy <= 1'b0;
        endcase
    end
end

// Independent-clock FIFO: assemble on fft_input_clk, drain on adc_clk (dma_s2mm).
xpm_fifo_axis #(
    .TDATA_WIDTH      (64),
    .FIFO_DEPTH       (1 << $clog2(HIST_BLOCK_SIZE * 4 + 4)),
    .CLOCKING_MODE    ("independent_clock"),
    .RELATED_CLOCKS   (0),
    .CDC_SYNC_STAGES  (2),
    .USE_ADV_FEATURES (16'h0000)
) i_dma_pkt_fifo (
    .s_aclk          (fft_input_clk),
    .m_aclk          (adc_clk_i),
    .s_aresetn       (fft_rstn_i),
    .s_axis_tdata    (asm_tdata),
    .s_axis_tvalid   (asm_tvalid),
    .s_axis_tready   (),
    .s_axis_tlast    (asm_tlast),
    .m_axis_tdata    (dma_a_tdata),
    .m_axis_tvalid   (dma_a_tvalid),
    .m_axis_tready   (dma_a_tready),
    .m_axis_tlast    (dma_a_tlast)
);
// Single combined stream now; second dma_s2mm input idle.
assign dma_b_tdata  = '0;
assign dma_b_tvalid = 1'b0;
assign dma_b_tlast  = 1'b0;

always @(posedge adc_clk_i)
if (adc_rstn_i == 1'b0) begin
    fft_enable  <= 1'b0 ;
    fft_threshold_k <= 4;
    fft_peak_start <= 0;
    fft_peak_minimum <= 1;
    fft_cfar_guard <= 8;    // CA-CFAR: guard cells each side of the peak (~main-lobe half-width)
    fft_cfar_train <= 32;   // CA-CFAR: training/reference cells each side (on the noise floor)
    fft_wait1_cnt <= 100;
    fft_wait2_cnt <= 200;
    fft_acq1_cnt <= (2**(FSZ-1) - 200) & ~(FSSR-1);
    fft_acq2_cnt <= (2**(FSZ-1) - 200) & ~(FSSR-1);
    fft_trig_sync <= 0;
    fft_clk_sel <= 0;
    dma_nch <= DMA_MAXCH;                 // default: stream all present channels
    dma_int_en <= DMA_INT_DEF;            // runtime intensity (default = build knob)
end else if (sys_wen) begin
    if (sys_addr[19:0]==20'h0)  begin
        fft_parallel <= sys_wdata[4];
        fft_enable <= sys_wdata[5];
        fft_trig_sync <= sys_wdata[6];
        fft_clk_sel <= sys_wdata[11];
    end
    // DMA channel count (0=off,1,2); clamp to channels physically present.
    if (sys_addr[19:0]==20'h98)
        dma_nch <= (sys_wdata[3:0] > DMA_MAXCH) ? DMA_MAXCH : sys_wdata[3:0];
    // Runtime intensity enable: takes effect at the next DMA packet boundary
    // (the ASM FSM resamples per packet); the stamped version follows (+2).
    if (sys_addr[19:0]==20'h9C)
        dma_int_en <= sys_wdata[0];
    if (sys_addr[19:0]==20'h38) fft_peak_start <= sys_wdata[FSZ-1:0];
    if (sys_addr[19:0]==20'h3C) fft_threshold_k <= sys_wdata[16-1:0];
    if (sys_addr[19:0]==20'h40) fft_peak_minimum <= sys_wdata[DSZ-1:0];
    // CA-CFAR moving-window params. 0x44/0x48/0x4C are peak-result READBACK regs;
    // 0x50/0x54 are the genuinely-free slots (see readback comment below).
    if (sys_addr[19:0]==20'h50) fft_cfar_guard <= sys_wdata[FSZ-1:0];
    if (sys_addr[19:0]==20'h54) fft_cfar_train <= sys_wdata[FSZ-1:0];
    if (sys_addr[19:0]==20'h58) fft_wait1_cnt <= sys_wdata[FSZ-1:0];
    if (sys_addr[19:0]==20'h5C) fft_wait2_cnt <= sys_wdata[FSZ-1:0];
    // Force acq counts to whole SSR beats: fin packs FSSR samples/beat and the FFT
    // input FSM reads beats (acq>>SSR_BITS). A non-FSSR-multiple leaves a partial beat
    // stranded in fin -> next frame's lanes shift (fs/FSSR rotation) + flush starvation.
    if (sys_addr[19:0]==20'h60) fft_acq1_cnt <= sys_wdata[FSZ-1:0] & ~(FSSR-1);
    if (sys_addr[19:0]==20'h64) fft_acq2_cnt <= sys_wdata[FSZ-1:0] & ~(FSSR-1);
    // Only writable when runtime FFT length is enabled; otherwise this decode is
    // constant-false and pruned, leaving fft_nfft fixed at its FSZ init value.
    if (RUNTIME_NFFT && sys_addr[19:0]==20'h88) begin
        if (sys_wdata > FSZ)
            fft_nfft <= FSZ;
        else if (sys_wdata < 3)
            fft_nfft <= 3;
        else
            fft_nfft <= sys_wdata[5-1:0];
    end
end

always @(posedge adc_clk_i)
if (fft_rstn_i == 0) begin
    fft_state <= S_IDLE;
    // Active window counts mirror the AXI defaults (see fft_*_cnt reset below);
    // they re-latch from pending on the next S_IDLE cycle anyway.
    fft_wait1_cnt_act <= 100;
    fft_wait2_cnt_act <= 200;
    fft_acq1_cnt_act  <= (2**(FSZ-1) - 200) & ~(FSSR-1);
    fft_acq2_cnt_act  <= (2**(FSZ-1) - 200) & ~(FSSR-1);
    fft_reconf_wait   <= 0;
    fft_acq_up_done   <= 0;
    fft_acq_down_done <= 0;
end else begin
    if (sys_wen && (sys_addr[19:0]==20'h0) && sys_wdata[9]) begin
        fft_peak_ready[0] <= 0;
    end else if (fft_peak_ready_a) begin
        fft_peak_ready[0] <= 1;
    end
    if (sys_wen && (sys_addr[19:0]==20'h0) && sys_wdata[10])
        fft_peak_ready[1] <= 0;
    else if (fft_peak_ready_b) begin
        fft_peak_ready[1] <= 1;
    end

    case (fft_state)
    S_IDLE: begin
        // Frame boundary: latch the pending (AXI-written) window counts into the
        // active set used by the FSM and the fft_proc engines. They stay constant
        // for the whole WAIT1..FFT_DOWN frame (not reassigned in the other states).
        fft_wait1_cnt_act <= fft_wait1_cnt;
        fft_wait2_cnt_act <= fft_wait2_cnt;
        fft_acq1_cnt_act  <= fft_acq1_cnt;
        fft_acq2_cnt_act  <= fft_acq2_cnt;
        // An acq change re-arms the engine's conf handshake + reset; hold here
        // until it settles (fft_done is forced high through that reset, so it
        // cannot gate us). A wait-only change touches the FSM alone -> no hold.
        if (fft_acq1_cnt != fft_acq1_cnt_act || fft_acq2_cnt != fft_acq2_cnt_act)
            fft_reconf_wait <= RECONF_CYCLES;
        else if (fft_reconf_wait != 0)
            fft_reconf_wait <= fft_reconf_wait - 1'b1;

        if (fft_trig_i && &fft_done && fft_reconf_wait == 0) begin
            fft_state_cnt <= 0;
            fft_state <= S_WAIT1;
            // new frame: clear both half-done flags so the engine doesn't post-pad
            // until each half's acquisition window has actually ended below.
            fft_acq_up_done   <= 0;
            fft_acq_down_done <= 0;
        end else
            fft_active_o <= 0;
    end
    S_WAIT1:
        if (fft_state_cnt >= fft_wait1_cnt_act) begin
            fft_state_cnt <= 0;
            fft_state <= S_FFT_UP;
        end else if (fft_dvalid)
            fft_state_cnt <= fft_state_cnt + 1;
    S_FFT_UP:
        if (fft_state_cnt >= fft_acq1_cnt_act) begin
            fft_state_cnt <= 0;
            fft_state <= S_WAIT2;
            fft_acq_up_done <= 1;   // up acquisition complete -> engine may post-pad up half
            // fft_active_o <= 0;
        end else if (fft_dvalid) begin
            fft_active_o <= 1;
            fft_state_cnt <= fft_state_cnt + 1;
        end
    S_WAIT2:
        if (fft_state_cnt >= fft_wait2_cnt_act) begin
            fft_state_cnt <= 0;
            fft_state <= S_FFT_DOWN;
        end else if (fft_dvalid)
            fft_state_cnt <= fft_state_cnt + 1;
    S_FFT_DOWN:
        if (fft_state_cnt >= fft_acq2_cnt_act) begin
            fft_state_cnt <= 0;
            fft_state <= S_IDLE;
            fft_acq_down_done <= 1;   // down acquisition complete -> engine may post-pad down half
        end else if (fft_dvalid) begin
            fft_active_o <= 1;
            fft_state_cnt <= fft_state_cnt + 1;
        end
    default: begin
            fft_state <= S_IDLE;
            fft_state_cnt <= 0;
            fft_active_o <= 0;
        end
    endcase
end

//---------------------------------------------------------------------------------
//
//  AXI CHA connection

reg  [ 32-1: 0] set_a_axi_start    ;
reg  [ 32-1: 0] set_a_axi_stop     ;
reg  [ 32-1: 0] set_a_axi_dly      ;
reg             set_a_axi_en       ;
reg  [ 32-1: 0] set_a_axi_trig     ;
reg  [ 32-1: 0] set_a_axi_cur      ;
reg             axi_a_we           ;
reg  [ 64-1: 0] axi_a_dat          ;
reg  [  2-1: 0] axi_a_dat_sel      ;
reg  [  1-1: 0] axi_a_dat_dv       ;
reg  [ 32-1: 0] axi_a_dly_cnt      ;
reg             axi_a_dly_do       ;
wire            axi_a_clr          ;
wire [ 32-1: 0] axi_a_cur_addr     ;

assign axi_a_clr = adc_rst_do ;


always @(posedge axi0_clk_o) begin
   if (axi0_rstn_o == 1'b0) begin
      axi_a_we      <=  1'b0 ;
      axi_a_dat     <= 64'h0 ;
      axi_a_dat_sel <=  2'h0 ;
      axi_a_dat_dv  <=  1'b0 ;
      axi_a_dly_cnt <= 32'h0 ;
      axi_a_dly_do  <=  1'b0 ;
   end
   /*
   else begin
      if (adc_arm_do && set_a_axi_en)
         axi_a_we <= 1'b1 ;
      else if (((axi_a_dly_do || adc_trig) && (axi_a_dly_cnt == 32'h0)) || adc_rst_do) //delayed reached or reset
         axi_a_we <= 1'b0 ;

      if (adc_trig && axi_a_we)
         axi_a_dly_do  <= 1'b1 ;
      else if ((axi_a_dly_do && (axi_a_dly_cnt == 32'b0)) || axi_a_clr || adc_arm_do) //delayed reached or reset
         axi_a_dly_do  <= 1'b0 ;

      if (axi_a_dly_do && axi_a_we && adc_dv)
         axi_a_dly_cnt <= axi_a_dly_cnt - 1;
      else if (!axi_a_dly_do)
         axi_a_dly_cnt <= set_a_axi_dly ;

      if (axi_a_clr)
         axi_a_dat_sel <= 2'h0 ;
      else if (axi_a_we && adc_dv)
         axi_a_dat_sel <= axi_a_dat_sel + 2'h1 ;

      axi_a_dat_dv <= axi_a_we && (axi_a_dat_sel == 2'b11) && adc_dv ;
   end

   if (axi_a_we && adc_dv) begin
      if (axi_a_dat_sel == 2'b00) axi_a_dat[ 16-1:  0] <= $signed(adc_a_dat);
      if (axi_a_dat_sel == 2'b01) axi_a_dat[ 32-1: 16] <= $signed(adc_a_dat);
      if (axi_a_dat_sel == 2'b10) axi_a_dat[ 48-1: 32] <= $signed(adc_a_dat);
      if (axi_a_dat_sel == 2'b11) axi_a_dat[ 64-1: 48] <= $signed(adc_a_dat);
   end

   if (axi_a_clr)
      set_a_axi_trig <= {RSZ{1'b0}};
   else if (adc_trig && !axi_a_dly_do && axi_a_we)
      set_a_axi_trig <= {axi_a_cur_addr[32-1:3],axi_a_dat_sel,1'b0} ; // save write pointer at trigger arrival

   if (axi_a_clr)
      set_a_axi_cur <= set_a_axi_start ;
   else if (axi0_wvalid_o)
      set_a_axi_cur <= axi_a_cur_addr ;
*/
end
/*
axi_wr_fifo #(
  .DW  (  64    ), // data width (8,16,...,1024)
  .AW  (  32    ), // address width
  .FW  (   8    )  // address width of FIFO pointers
) i_wr0 (
   // global signals
  .axi_clk_i          (  axi0_clk_o        ), // global clock
  .axi_rstn_i         (  axi0_rstn_o       ), // global reset

   // Connection to AXI master
  .axi_waddr_o        (  axi0_waddr_o      ), // write address
  .axi_wdata_o        (  axi0_wdata_o      ), // write data
  .axi_wsel_o         (  axi0_wsel_o       ), // write byte select
  .axi_wvalid_o       (  axi0_wvalid_o     ), // write data valid
  .axi_wlen_o         (  axi0_wlen_o       ), // write burst length
  .axi_wfixed_o       (  axi0_wfixed_o     ), // write burst type (fixed / incremental)
  .axi_werr_i         (  axi0_werr_i       ), // write error
  .axi_wrdy_i         (  axi0_wrdy_i       ), // write ready

   // data and configuration
  .wr_data_i          (  axi_a_dat         ), // write data
  .wr_val_i           (  axi_a_dat_dv      ), // write data valid
  .ctrl_start_addr_i  (  set_a_axi_start   ), // range start address
  .ctrl_stop_addr_i   (  set_a_axi_stop    ), // range stop address
  .ctrl_trig_size_i   (  4'hF              ), // trigger level
  .ctrl_wrap_i        (  1'b1              ), // start from begining when reached stop
  .ctrl_clr_i         (  axi_a_clr         ), // clear / flush
  .stat_overflow_o    (                    ), // overflow indicator
  .stat_cur_addr_o    (  axi_a_cur_addr    ), // current write address
  .stat_write_data_o  (                    )  // write data indicator
);
*/
assign axi0_clk_o  = adc_clk_i ;
assign axi0_rstn_o = adc_rstn_i;

//---------------------------------------------------------------------------------
//
//  AXI CHB connection

reg  [ 32-1: 0] set_b_axi_start    ;
reg  [ 32-1: 0] set_b_axi_stop     ;
reg  [ 32-1: 0] set_b_axi_dly      ;
reg             set_b_axi_en       ;
reg  [ 32-1: 0] set_b_axi_trig     ;
reg  [ 32-1: 0] set_b_axi_cur      ;
reg             axi_b_we           ;
reg  [ 64-1: 0] axi_b_dat          ;
reg  [  2-1: 0] axi_b_dat_sel      ;
reg  [  1-1: 0] axi_b_dat_dv       ;
reg  [ 32-1: 0] axi_b_dly_cnt      ;
reg             axi_b_dly_do       ;
wire            axi_b_clr          ;
wire [ 32-1: 0] axi_b_cur_addr     ;

assign axi_b_clr = adc_rst_do ;


always @(posedge axi1_clk_o) begin
   if (axi1_rstn_o == 1'b0) begin
      axi_b_we      <=  1'b0 ;
      axi_b_dat     <= 64'h0 ;
      axi_b_dat_sel <=  2'h0 ;
      axi_b_dat_dv  <=  1'b0 ;
      axi_b_dly_cnt <= 32'h0 ;
      axi_b_dly_do  <=  1'b0 ;
   end
/*   else begin
      if (adc_arm_do && set_b_axi_en)
         axi_b_we <= 1'b1 ;
      else if (((axi_b_dly_do || adc_trig) && (axi_b_dly_cnt == 32'h0)) || adc_rst_do) //delayed reached or reset
         axi_b_we <= 1'b0 ;

      if (adc_trig && axi_b_we)
         axi_b_dly_do  <= 1'b1 ;
      else if ((axi_b_dly_do && (axi_b_dly_cnt == 32'b0)) || axi_b_clr || adc_arm_do) //delayed reached or reset
         axi_b_dly_do  <= 1'b0 ;

      if (axi_b_dly_do && axi_b_we && adc_dv)
         axi_b_dly_cnt <= axi_b_dly_cnt - 1;
      else if (!axi_b_dly_do)
         axi_b_dly_cnt <= set_b_axi_dly ;

      if (axi_b_clr)
         axi_b_dat_sel <= 2'h0 ;
      else if (axi_b_we && adc_dv)
         axi_b_dat_sel <= axi_b_dat_sel + 2'h1 ;

      axi_b_dat_dv <= axi_b_we && (axi_b_dat_sel == 2'b11) && adc_dv ;
   end

   if (axi_b_we && adc_dv) begin
      if (axi_b_dat_sel == 2'b00) axi_b_dat[ 16-1:  0] <= $signed(adc_b_dat);
      if (axi_b_dat_sel == 2'b01) axi_b_dat[ 32-1: 16] <= $signed(adc_b_dat);
      if (axi_b_dat_sel == 2'b10) axi_b_dat[ 48-1: 32] <= $signed(adc_b_dat);
      if (axi_b_dat_sel == 2'b11) axi_b_dat[ 64-1: 48] <= $signed(adc_b_dat);
   end

   if (axi_b_clr)
      set_b_axi_trig <= {RSZ{1'b0}};
   else if (adc_trig && !axi_b_dly_do && axi_b_we)
      set_b_axi_trig <= {axi_b_cur_addr[32-1:3],axi_b_dat_sel,1'b0} ; // save write pointer at trigger arrival

   if (axi_b_clr)
      set_b_axi_cur <= set_b_axi_start ;
   else if (axi1_wvalid_o)
      set_b_axi_cur <= axi_b_cur_addr ;
    */
end
/*
axi_wr_fifo #(
  .DW  (  64    ), // data width (8,16,...,1024)
  .AW  (  32    ), // address width
  .FW  (   8    )  // address width of FIFO pointers
) i_wr1 (
   // global signals
  .axi_clk_i          (  axi1_clk_o        ), // global clock
  .axi_rstn_i         (  axi1_rstn_o       ), // global reset

   // Connection to AXI master
  .axi_waddr_o        (  axi1_waddr_o      ), // write address
  .axi_wdata_o        (  axi1_wdata_o      ), // write data
  .axi_wsel_o         (  axi1_wsel_o       ), // write byte select
  .axi_wvalid_o       (  axi1_wvalid_o     ), // write data valid
  .axi_wlen_o         (  axi1_wlen_o       ), // write burst length
  .axi_wfixed_o       (  axi1_wfixed_o     ), // write burst type (fixed / incremental)
  .axi_werr_i         (  axi1_werr_i       ), // write error
  .axi_wrdy_i         (  axi1_wrdy_i       ), // write ready

   // data and configuration
  .wr_data_i          (  axi_b_dat         ), // write data
  .wr_val_i           (  axi_b_dat_dv      ), // write data valid
  .ctrl_start_addr_i  (  set_b_axi_start   ), // range start address
  .ctrl_stop_addr_i   (  set_b_axi_stop    ), // range stop address
  .ctrl_trig_size_i   (  4'hF              ), // trigger level
  .ctrl_wrap_i        (  1'b1              ), // start from begining when reached stop
  .ctrl_clr_i         (  axi_b_clr         ), // clear / flush
  .stat_overflow_o    (                    ), // overflow indicator
  .stat_cur_addr_o    (  axi_b_cur_addr    ), // current write address
  .stat_write_data_o  (                    )  // write data indicator
);
*/
assign axi1_clk_o  = adc_clk_i ;
assign axi1_rstn_o = adc_rstn_i;

////////////// END AXI DISABLING ////////////////////


//---------------------------------------------------------------------------------
//  Trigger source selector

reg               adc_trig_ap      ;
reg               adc_trig_an      ;
reg               adc_trig_bp      ;
reg               adc_trig_bn      ;
reg               adc_trig_sw      ;
reg   [   4-1: 0] set_trig_src     ;
wire              ext_trig_p       ;
wire              ext_trig_n       ;
wire              asg_trig_p       ;
wire              asg_trig_n       ;
wire              asg_trig2_p      ;
wire              asg_trig2_n      ;

logic [   4-1: 0] fft_trig_src     ;

always @(posedge adc_clk_i)
if (adc_rstn_i == 1'b0) begin
   adc_arm_do    <= 1'b0 ;
   adc_rst_do    <= 1'b0 ;
   adc_trig_sw   <= 1'b0 ;
   set_trig_src  <= 4'h0 ;
   adc_trig      <= 1'b0 ;

   fft_trig_src  <= 0;
   fft_trig      <= 0;
end else begin
   adc_arm_do  <= sys_wen && (sys_addr[19:0]==20'h0) && sys_wdata[0] ; // SW ARM
   adc_rst_do  <= sys_wen && (sys_addr[19:0]==20'h0) && sys_wdata[1] ;
   adc_trig_sw <= sys_wen && (sys_addr[19:0]==20'h4) && (sys_wdata[3:0]==4'h1); // SW trigger

      if (sys_wen && (sys_addr[19:0]==20'h4)) begin
         set_trig_src <= sys_wdata[3:0] ;
         fft_trig_src <= sys_wdata[3:0] ;
      end else if (((adc_dly_do || adc_trig) && (adc_dly_cnt == 32'h0)) || adc_rst_do) //delayed reached or reset
         set_trig_src <= 4'h0 ;

    case (set_trig_src)
        4'd1 : adc_trig <= adc_trig_sw   ; // manual
        4'd2 : adc_trig <= adc_trig_ap   ; // A ch rising edge
        4'd3 : adc_trig <= adc_trig_an   ; // A ch falling edge
        4'd4 : adc_trig <= adc_trig_bp   ; // B ch rising edge
        4'd5 : adc_trig <= adc_trig_bn   ; // B ch falling edge
        4'd6 : adc_trig <= ext_trig_p    ; // external - rising edge
        4'd7 : adc_trig <= ext_trig_n    ; // external - falling edge
        4'd8 : adc_trig <= asg_trig_p    ; // ASG 1 - rising edge
        4'd9 : adc_trig <= asg_trig_n    ; // ASG 2 - rising edge
        4'd10: adc_trig <= trig_dsp_i    ; // dsp trigger input
        4'd11: adc_trig <= asg_trig2_p   ; // ASG 3 - rising edge
        4'd12: adc_trig <= asg_trig2_n   ; // ASG 4 - rising edge
        default : adc_trig <= 1'b0          ;
    endcase

    case (fft_trig_src)
        4'd1 : fft_trig <= adc_trig_sw   ; // manual
        4'd2 : fft_trig <= adc_trig_ap   ; // A ch rising edge
        4'd3 : fft_trig <= adc_trig_an   ; // A ch falling edge
        4'd4 : fft_trig <= adc_trig_bp   ; // B ch rising edge
        4'd5 : fft_trig <= adc_trig_bn   ; // B ch falling edge
        4'd6 : fft_trig <= ext_trig_p    ; // external - rising edge
        4'd7 : fft_trig <= ext_trig_n    ; // external - falling edge
        4'd8 : fft_trig <= asg_trig_p    ; // ASG 1 - rising edge
        4'd9 : fft_trig <= asg_trig_n    ; // ASG 2 - rising edge
        4'd10: fft_trig <= trig_dsp_i    ; // dsp trigger input
        4'd11: fft_trig <= asg_trig2_p   ; // ASG 3 - rising edge
        4'd12: fft_trig <= asg_trig2_n   ; // ASG 4 - rising edge
        default : fft_trig <= 1'b0          ;
    endcase
end

//---------------------------------------------------------------------------------
//  Trigger created from input signal

reg  [  2-1: 0] adc_scht_ap  ;
reg  [  2-1: 0] adc_scht_an  ;
reg  [  2-1: 0] adc_scht_bp  ;
reg  [  2-1: 0] adc_scht_bn  ;
reg  [ASZ-1: 0] set_a_tresh  ;
reg  [ASZ-1: 0] set_a_treshp ;
reg  [ASZ-1: 0] set_a_treshm ;
//reg  [ASZ-1: 0] set_b_tresh  ;
//reg  [ASZ-1: 0] set_b_treshp ;
//reg  [ASZ-1: 0] set_b_treshm ;
reg  [ASZ-1: 0] set_a_hyst   ;
//reg  [ASZ-1: 0] set_b_hyst   ;

always @(posedge adc_clk_i)
if (adc_rstn_i == 1'b0) begin
   adc_scht_ap  <=  2'h0 ;
   adc_scht_an  <=  2'h0 ;
   adc_scht_bp  <=  2'h0 ;
   adc_scht_bn  <=  2'h0 ;
   adc_trig_ap  <=  1'b0 ;
   adc_trig_an  <=  1'b0 ;
   adc_trig_bp  <=  1'b0 ;
   adc_trig_bn  <=  1'b0 ;
end else begin
   set_a_treshp <= set_a_tresh + set_a_hyst ; // calculate positive
   set_a_treshm <= set_a_tresh - set_a_hyst ; // and negative treshold
   //set_b_treshp <= set_b_tresh + set_b_hyst ;
   //set_b_treshm <= set_b_tresh - set_b_hyst ;

   if (adc_dv) begin
           if ($signed(adc_a_dat) >= $signed(set_a_tresh ))      adc_scht_ap[0] <= 1'b1 ;  // treshold reached
      else if ($signed(adc_a_dat) <  $signed(set_a_treshm))      adc_scht_ap[0] <= 1'b0 ;  // wait until it goes under hysteresis
           if ($signed(adc_a_dat) <= $signed(set_a_tresh ))      adc_scht_an[0] <= 1'b1 ;  // treshold reached
      else if ($signed(adc_a_dat) >  $signed(set_a_treshp))      adc_scht_an[0] <= 1'b0 ;  // wait until it goes over hysteresis

           if ($signed(adc_b_dat) >= $signed(set_a_tresh ))      adc_scht_bp[0] <= 1'b1 ; //set_b_tresh
      else if ($signed(adc_b_dat) <  $signed(set_a_treshm))      adc_scht_bp[0] <= 1'b0 ; //set_b_treshm
           if ($signed(adc_b_dat) <= $signed(set_a_tresh ))      adc_scht_bn[0] <= 1'b1 ; //set_b_tresh
      else if ($signed(adc_b_dat) >  $signed(set_a_treshp))      adc_scht_bn[0] <= 1'b0 ; //set_b_treshp
   end

   adc_scht_ap[1] <= adc_scht_ap[0] ;
   adc_scht_an[1] <= adc_scht_an[0] ;
   adc_scht_bp[1] <= adc_scht_bp[0] ;
   adc_scht_bn[1] <= adc_scht_bn[0] ;

   adc_trig_ap <= adc_scht_ap[0] && !adc_scht_ap[1] ; // make 1 cyc pulse 
   adc_trig_an <= adc_scht_an[0] && !adc_scht_an[1] ;
   adc_trig_bp <= adc_scht_bp[0] && !adc_scht_bp[1] ;
   adc_trig_bn <= adc_scht_bn[0] && !adc_scht_bn[1] ;
end

//---------------------------------------------------------------------------------
//  External trigger

reg  [  3-1: 0] ext_trig_in    ;
reg  [  2-1: 0] ext_trig_dp    ;
reg  [  2-1: 0] ext_trig_dn    ;
reg  [ 20-1: 0] ext_trig_debp  ;
reg  [ 20-1: 0] ext_trig_debn  ;
reg  [  3-1: 0] asg_trig_in_ch1;
reg  [  3-1: 0] asg_trig_in_ch2;
reg  [  3-1: 0] asg_trig_in_ch3;
reg  [  3-1: 0] asg_trig_in_ch4;
reg  [  2-1: 0] asg_trig_dp    ;
reg  [  2-1: 0] asg_trig_dn    ;
reg  [  2-1: 0] asg_trig2_dp   ;
reg  [  2-1: 0] asg_trig2_dn   ;
reg  [ 20-1: 0] asg_trig_debp  ;
reg  [ 20-1: 0] asg_trig_debn  ;
reg  [ 20-1: 0] asg_trig2_debp ;
reg  [ 20-1: 0] asg_trig2_debn ;

always @(posedge adc_clk_i)
if (adc_rstn_i == 1'b0) begin
   ext_trig_in   <=  3'h0 ;
   ext_trig_dp   <=  2'h0 ;
   ext_trig_dn   <=  2'h0 ;
   ext_trig_debp <= 20'h0 ;
   ext_trig_debn <= 20'h0 ;
   asg_trig_in_ch1 <=  3'h0 ;
   asg_trig_in_ch2 <=  3'h0 ;
   asg_trig_in_ch3 <=  3'h0 ;
   asg_trig_in_ch4 <=  3'h0 ;
   asg_trig_dp   <=  2'h0 ;
   asg_trig_dn   <=  2'h0 ;
   asg_trig2_dp  <=  2'h0 ;
   asg_trig2_dn  <=  2'h0 ;
   asg_trig_debp <= 20'h0 ;
   asg_trig_debn <= 20'h0 ;
   asg_trig2_debp<= 20'h0 ;
   asg_trig2_debn<= 20'h0 ;
end else begin
   //----------- External trigger
   // synchronize FFs
   ext_trig_in <= {ext_trig_in[1:0],trig_ext_i} ;

   // look for input changes
   if ((ext_trig_debp == 20'h0) && (ext_trig_in[1] && !ext_trig_in[2]))
      ext_trig_debp <= set_deb_len ; // ~0.5ms
   else if (ext_trig_debp != 20'h0)
      ext_trig_debp <= ext_trig_debp - 20'd1 ;

   if ((ext_trig_debn == 20'h0) && (!ext_trig_in[1] && ext_trig_in[2]))
      ext_trig_debn <= set_deb_len ; // ~0.5ms
   else if (ext_trig_debn != 20'h0)
      ext_trig_debn <= ext_trig_debn - 20'd1 ;

   // update output values
   ext_trig_dp[1] <= ext_trig_dp[0] ;
   if (ext_trig_debp == 20'h0)
      ext_trig_dp[0] <= ext_trig_in[1] ;

   ext_trig_dn[1] <= ext_trig_dn[0] ;
   if (ext_trig_debn == 20'h0)
      ext_trig_dn[0] <= ext_trig_in[1] ;

   //----------- ASG trigger - instead of pos/neg. edge we use ch1 pos edge / ch2 pos edge
   // synchronize FFs
   asg_trig_in_ch1 <= {asg_trig_in_ch1[1:0],trig_asg_i[0]} ;
   asg_trig_in_ch2 <= {asg_trig_in_ch2[1:0],trig_asg_i[1]} ;
   asg_trig_in_ch3 <= {asg_trig_in_ch3[1:0],trig_asg_i[2]} ;
   asg_trig_in_ch4 <= {asg_trig_in_ch4[1:0],trig_asg_i[3]} ;

   // look for input changes -ch1
   if ((asg_trig_debp == 20'h0) && (asg_trig_in_ch1[1] && !asg_trig_in_ch1[2]))
      asg_trig_debp <= set_deb_len2 ; // ~0.5ms
   else if (asg_trig_debp != 20'h0)
      asg_trig_debp <= asg_trig_debp - 20'd1 ;

   // look for input changes - ch2
   if ((asg_trig_debn == 20'h0) && (asg_trig_in_ch2[1] && !asg_trig_in_ch2[2]))
      asg_trig_debn <= set_deb_len2 ; // ~0.5ms
   else if (asg_trig_debn != 20'h0)
      asg_trig_debn <= asg_trig_debn - 20'd1 ;

   // look for input changes - ch3
   if ((asg_trig2_debp == 20'h0) && (asg_trig_in_ch3[1] && !asg_trig_in_ch3[2]))
      asg_trig2_debp <= set_deb_len2 ; // ~0.5ms
   else if (asg_trig2_debp != 20'h0)
      asg_trig2_debp <= asg_trig2_debp - 20'd1 ;

   // look for input changes - ch4
   if ((asg_trig2_debn == 20'h0) && (asg_trig_in_ch4[1] && !asg_trig_in_ch4[2]))
      asg_trig2_debn <= set_deb_len2 ; // ~0.5ms
   else if (asg_trig2_debn != 20'h0)
      asg_trig2_debn <= asg_trig2_debn - 20'd1 ;

   // update output values
   asg_trig_dp[1] <= asg_trig_dp[0] ;
   if (asg_trig_debp == 20'h0)
      asg_trig_dp[0] <= asg_trig_in_ch1[1] ;

   asg_trig_dn[1] <= asg_trig_dn[0] ;
   if (asg_trig_debn == 20'h0)
      asg_trig_dn[0] <= asg_trig_in_ch2[1] ;

   asg_trig2_dp[1] <= asg_trig2_dp[0] ;
   if (asg_trig2_debp == 20'h0)
      asg_trig2_dp[0] <= asg_trig_in_ch3[1] ;

   asg_trig2_dn[1] <= asg_trig2_dn[0] ;
   if (asg_trig2_debn == 20'h0)
      asg_trig2_dn[0] <= asg_trig_in_ch4[1] ;
end

assign ext_trig_p = (ext_trig_dp == 2'b01) ;
assign ext_trig_n = (ext_trig_dn == 2'b10) ;
assign asg_trig_p = (asg_trig_dp == 2'b01) ;
assign asg_trig_n = (asg_trig_dn == 2'b01) ;
assign asg_trig2_p= (asg_trig2_dp == 2'b01) ;
assign asg_trig2_n= (asg_trig2_dn == 2'b01) ;

logic [32-1:0] scope_sig_dly;

//---------------------------------------------------------------------------------
//  System bus connection

always @(posedge adc_clk_i)
if (adc_rstn_i == 1'b0) begin
   adc_we_keep   <=   0      ;
   set_a_tresh   <=   0      ;
   //set_b_tresh   <=  ASZ'd0000   ;
   set_dly       <=  2**(RSZ-1);
   set_dec       <=  17'h2000; // corresponds to 1s duration, formerly at minimum: 17'd1
   set_a_hyst    <=  20     ;
   //set_b_hyst    <=  ASZ'd20     ;
   set_avg_en    <=   1'b0      ;
/*   set_a_filt_aa <=  18'h0      ;
   set_a_filt_bb <=  25'h0      ;
   set_a_filt_kk <=  25'hFFFFFF ;
   set_a_filt_pp <=  25'h0      ;
   set_b_filt_aa <=  18'h0      ;
   set_b_filt_bb <=  25'h0      ;
   set_b_filt_kk <=  25'hFFFFFF ;
   set_b_filt_pp <=  25'h0      ;*/
   set_deb_len   <=  20'd62500  ;
   set_deb_len2  <=  20'd0      ;
   set_a_axi_en  <=   1'b0      ;
   set_b_axi_en  <=   1'b0      ;
   scope_sig_dly <= 6875      ;

end else begin
   if (sys_wen) begin
      if (sys_addr[19:0]==20'h00)   adc_we_keep   <= sys_wdata[     3] ;

      if (sys_addr[19:0]==20'h08)   set_a_tresh   <= sys_wdata[ASZ-1:0] ;
      //if (sys_addr[19:0]==20'h0C)   set_b_tresh   <= sys_wdata[ASZ-1:0] ;
      if (sys_addr[19:0]==20'h10)   set_dly       <= sys_wdata[32-1:0] ;
      if (sys_addr[19:0]==20'h14)   set_dec       <= sys_wdata[17-1:0] ;
      if (sys_addr[19:0]==20'h20)   set_a_hyst    <= sys_wdata[ASZ-1:0] ;
      //if (sys_addr[19:0]==20'h24)   set_b_hyst    <= sys_wdata[ASZ-1:0] ;
      if (sys_addr[19:0]==20'h28)   set_avg_en    <= sys_wdata[     0] ;

      /*
      if (sys_addr[19:0]==20'h30)   set_a_filt_aa <= sys_wdata[18-1:0] ;
      if (sys_addr[19:0]==20'h34)   set_a_filt_bb <= sys_wdata[25-1:0] ;
      if (sys_addr[19:0]==20'h38)   set_a_filt_kk <= sys_wdata[25-1:0] ;
      if (sys_addr[19:0]==20'h3C)   set_a_filt_pp <= sys_wdata[25-1:0] ;
      if (sys_addr[19:0]==20'h40)   set_b_filt_aa <= sys_wdata[18-1:0] ;
      if (sys_addr[19:0]==20'h44)   set_b_filt_bb <= sys_wdata[25-1:0] ;
      if (sys_addr[19:0]==20'h48)   set_b_filt_kk <= sys_wdata[25-1:0] ;
      if (sys_addr[19:0]==20'h4C)   set_b_filt_pp <= sys_wdata[25-1:0] ;
      */
      /*
      if (sys_addr[19:0]==20'h50)   set_a_axi_start <= sys_wdata[32-1:0] ;
      if (sys_addr[19:0]==20'h54)   set_a_axi_stop  <= sys_wdata[32-1:0] ;
      if (sys_addr[19:0]==20'h58)   set_a_axi_dly   <= sys_wdata[32-1:0] ;
      if (sys_addr[19:0]==20'h5C)   set_a_axi_en    <= sys_wdata[     0] ;

      if (sys_addr[19:0]==20'h70)   set_b_axi_start <= sys_wdata[32-1:0] ;
      if (sys_addr[19:0]==20'h74)   set_b_axi_stop  <= sys_wdata[32-1:0] ;
      if (sys_addr[19:0]==20'h78)   set_b_axi_dly   <= sys_wdata[32-1:0] ;
      if (sys_addr[19:0]==20'h7C)   set_b_axi_en    <= sys_wdata[     0] ;
      */
      if (sys_addr[19:0]==20'h90)   set_deb_len <= sys_wdata[20-1:0] ;
      if (sys_addr[19:0]==20'h94)   set_deb_len2<= sys_wdata[20-1:0] ;
      if (sys_addr[19:0]==20'h18C)  scope_sig_dly <= sys_wdata[32-1:0];
   end
end

wire sys_en;
assign sys_en = sys_wen | sys_ren;

logic scope_sig;
logic [32-1:0] scope_sig_pre_cnt;
logic [8-1:0] scope_sig_post_cnt;
// Width of the scope_sig_o scan-trigger pulse, in adc cycles. Was 125 (1 us) only
// so a cheap oscilloscope could see it; that 1 us is ~30% of the 300 kHz chirp
// period and, since scope_sig blocks re-arming while high, it made the scan-
// trigger FSM overrun the chirp period and skip chirps. A few cycles is plenty to
// edge-trigger the fast-axis ASG (same adc_clk domain).
localparam [7:0] SCAN_SIG_POST = 8'd4;
assign scope_sig_o = scope_sig && scope_sig_pre_cnt == 0;

assign scope_done_o = (!fft_enable && !adc_we) || (fft_enable && fft_state==S_IDLE);
assign x_step_0 = fft_hist_index[IDX_PIPELINE][0];

always @(posedge adc_clk_i)
if (adc_rstn_i == 1'b0) begin
   sys_err <= 1'b0 ;
   sys_ack <= 1'b0 ;
   scope_sig <= 1'b0;

end else begin
   sys_err <= 1'b0 ;

   if (!scope_sig) begin
     scope_sig <= fft_trig_i && &fft_done;
     scope_sig_pre_cnt <= scope_sig_dly;
     scope_sig_post_cnt <= SCAN_SIG_POST; // short scan-trigger pulse (was 125 = 1us debug)
   end else if (scope_sig_pre_cnt != 0)
       scope_sig_pre_cnt <= scope_sig_pre_cnt - 1;
   else if (scope_sig_post_cnt != 0)
       scope_sig_post_cnt <= scope_sig_post_cnt - 1;
   else
       scope_sig <= 0;

   casez (sys_addr[19:0])
     20'h00000 : begin sys_ack <= sys_en;          sys_rdata <= {  {8-6{1'b0}}
                                                                 , fft_status[1]
                                                                 , {8-6{1'b0}}
                                                                 , fft_status[0]

                                                                 , {16-12{1'b0}}
                                                                 , fft_clk_sel
                                                                 , fft_peak_ready[2-1:0]
                                                                 , fft_done[2-1:0]
                                                                 , fft_trig_sync
                                                                 , fft_enable
                                                                 , fft_parallel
                                                                 , adc_we_keep               // do not disarm on 
                                                                 , adc_dly_do                // trigger status
                                                                 , 1'b0                      // reset
                                                                 // , adc_we | (fft_enable & (~&fft_done | (fft_trig_sync & ~&fft_peak_ready))) }
                                                                 , adc_we | (fft_enable & (fft_trig_sync & (~&fft_done | ~&fft_peak_ready))) }
                                                                 ; end // arm

     20'h00004 : begin sys_ack <= sys_en;          sys_rdata <= {{32- 4{1'b0}}, set_trig_src}       ; end 

     20'h00008 : begin sys_ack <= sys_en;          sys_rdata <= {{32-ASZ{1'b0}}, set_a_tresh}       ; end
     //20'h0000C : begin sys_ack <= sys_en;          sys_rdata <= {{32-ASZ{1'b0}}, set_b_tresh}        ; end
     20'h00010 : begin sys_ack <= sys_en;          sys_rdata <= {               set_dly}            ; end
     20'h00014 : begin sys_ack <= sys_en;          sys_rdata <= {{32-17{1'b0}}, set_dec}            ; end

     20'h00018 : begin sys_ack <= sys_en;          sys_rdata <= {{32-RSZ{1'b0}}, adc_wp_cur}        ; end
     20'h0001C : begin sys_ack <= sys_en;          sys_rdata <= {{32-RSZ{1'b0}}, adc_wp_trig}       ; end

     20'h00020 : begin sys_ack <= sys_en;          sys_rdata <= {{32-ASZ{1'b0}}, set_a_hyst}        ; end

     20'h00024 : begin sys_ack <= sys_en;          sys_rdata <= 2**HSZ                              ; end

     20'h00028 : begin sys_ack <= sys_en;          sys_rdata <= {{32- 1{1'b0}}, set_avg_en}         ; end

     20'h0002C : begin sys_ack <= sys_en;          sys_rdata <=                 adc_we_cnt          ; end

     20'h00030 : begin sys_ack <= sys_en;          sys_rdata <= fft_point_cnt                       ; end
     20'h00034 : begin sys_ack <= sys_en;          sys_rdata <= DSZ                                 ; end
     20'h00038 : begin sys_ack <= sys_en;          sys_rdata <= fft_peak_start                      ; end
     20'h0003C : begin sys_ack <= sys_en;          sys_rdata <= fft_threshold_k                     ; end
     20'h00040 : begin sys_ack <= sys_en;          sys_rdata <= fft_peak_minimum                    ; end
     // Legacy 16-bit packed peak-bin indices (backward compatible) — INTEGER bin only.
     // Each 16-bit field is the integer part of k_interp, i.e. k_interp[IDX-1:FRAC]
     // (FSZ bits, zero-extended to 16). 0x44 = {second, up_a}, where second =
     // parallel?up_b:down_a (mirrors the peak-VALUE reg 0x4C); 0x7C = {down_b, up_b}.
     // 0x50/0x54 are free again; the full Q(FSZ).FRAC fixed-point indices live at 0x170-0x17C.
     20'h00044 : begin sys_ack <= sys_en;          sys_rdata <= {{16-FSZ{1'b0}}, (fft_parallel ? fft_peak_index_up_b[IDX-1:FRAC] : fft_peak_index_down_a[IDX-1:FRAC]), {16-FSZ{1'b0}}, fft_peak_index_up_a[IDX-1:FRAC]}; end
     20'h00048 : begin sys_ack <= sys_en;          sys_rdata <= fft_peak_up_a                       ; end
     20'h0004C : begin sys_ack <= sys_en;          sys_rdata <= fft_parallel?fft_peak_up_b:fft_peak_down_a ; end
     // CA-CFAR moving-window params (write+read; only meaningful for PEAK_CFAR builds).
     20'h00050 : begin sys_ack <= sys_en;          sys_rdata <= {{32-FSZ{1'b0}}, fft_cfar_guard}    ; end
     20'h00054 : begin sys_ack <= sys_en;          sys_rdata <= {{32-FSZ{1'b0}}, fft_cfar_train}    ; end
     20'h00058 : begin sys_ack <= sys_en;          sys_rdata <= fft_wait1_cnt                       ; end
     20'h0005C : begin sys_ack <= sys_en;          sys_rdata <= fft_wait2_cnt                       ; end
     20'h00060 : begin sys_ack <= sys_en;          sys_rdata <= fft_acq1_cnt                        ; end
     20'h00064 : begin sys_ack <= sys_en;          sys_rdata <= fft_acq2_cnt                        ; end
     20'h00068 : begin sys_ack <= sys_en;          sys_rdata <= {fft_wp_last_a, fft_wp_last_b}      ; end
     20'h0006C : begin sys_ack <= sys_en;          sys_rdata <= fft_state                           ; end
     // 20'h00070 : begin sys_ack <= sys_en;          sys_rdata <= fft_we_cnt[0]                       ; end
     20'h00074 : begin sys_ack <= sys_en;          sys_rdata <= fft_scan_point_cnt                  ; end
     20'h0007C : begin sys_ack <= sys_en;          sys_rdata <= {{16-FSZ{1'b0}}, fft_peak_index_down_b[IDX-1:FRAC], {16-FSZ{1'b0}}, fft_peak_index_up_b[IDX-1:FRAC]}; end
     20'h00080 : begin sys_ack <= sys_en;          sys_rdata <= fft_peak_up_b                       ; end
     20'h00084 : begin sys_ack <= sys_en;          sys_rdata <= fft_peak_down_b                     ; end
     20'h00088 : begin sys_ack <= sys_en;          sys_rdata <= fft_nfft                            ; end
     20'h0008C : begin sys_ack <= sys_en;          sys_rdata <= 1<<fft_nfft                         ; end

     /*
     20'h00030 : begin sys_ack <= sys_en;          sys_rdata <= {{32-18{1'b0}}, set_a_filt_aa}      ; end
     20'h00034 : begin sys_ack <= sys_en;          sys_rdata <= {{32-25{1'b0}}, set_a_filt_bb}      ; end
     20'h00038 : begin sys_ack <= sys_en;          sys_rdata <= {{32-25{1'b0}}, set_a_filt_kk}      ; end
     20'h0003C : begin sys_ack <= sys_en;          sys_rdata <= {{32-25{1'b0}}, set_a_filt_pp}      ; end
     20'h00040 : begin sys_ack <= sys_en;          sys_rdata <= {{32-18{1'b0}}, set_b_filt_aa}      ; end
     20'h00044 : begin sys_ack <= sys_en;          sys_rdata <= {{32-25{1'b0}}, set_b_filt_bb}      ; end
     20'h00048 : begin sys_ack <= sys_en;          sys_rdata <= {{32-25{1'b0}}, set_b_filt_kk}      ; end
     20'h0004C : begin sys_ack <= sys_en;          sys_rdata <= {{32-25{1'b0}}, set_b_filt_pp}      ; end
     */
     /*
     20'h00050 : begin sys_ack <= sys_en;          sys_rdata <=                 set_a_axi_start     ; end
     20'h00054 : begin sys_ack <= sys_en;          sys_rdata <=                 set_a_axi_stop      ; end
     20'h00058 : begin sys_ack <= sys_en;          sys_rdata <=                 set_a_axi_dly       ; end
     20'h0005C : begin sys_ack <= sys_en;          sys_rdata <= {{32- 1{1'b0}}, set_a_axi_en}       ; end
     20'h00060 : begin sys_ack <= sys_en;          sys_rdata <=                 set_a_axi_trig      ; end
     20'h00064 : begin sys_ack <= sys_en;          sys_rdata <=                 set_a_axi_cur       ; end

     20'h00070 : begin sys_ack <= sys_en;          sys_rdata <=                 set_b_axi_start     ; end
     20'h00074 : begin sys_ack <= sys_en;          sys_rdata <=                 set_b_axi_stop      ; end
     20'h00078 : begin sys_ack <= sys_en;          sys_rdata <=                 set_b_axi_dly       ; end
     20'h0007C : begin sys_ack <= sys_en;          sys_rdata <= {{32- 1{1'b0}}, set_b_axi_en}       ; end
     20'h00080 : begin sys_ack <= sys_en;          sys_rdata <=                 set_b_axi_trig      ; end
     20'h00084 : begin sys_ack <= sys_en;          sys_rdata <=                 set_b_axi_cur       ; end
     */

     20'h00090 : begin sys_ack <= sys_en;          sys_rdata <= {{32-20{1'b0}}, set_deb_len}        ; end
     20'h00094 : begin sys_ack <= sys_en;          sys_rdata <= {{32-20{1'b0}}, set_deb_len2}        ; end
    
     20'h00154 : begin sys_ack <= sys_en;          sys_rdata <= {{32-ASZ{1'b0}}, adc_a_i }          ; end
     20'h00158 : begin sys_ack <= sys_en;          sys_rdata <= {{32-ASZ{1'b0}}, adc_b_i }          ; end
	 
	 20'h0015c : begin sys_ack <= sys_en;          sys_rdata <= ctr_value[32-1:0]     		        ; end
	 20'h00160 : begin sys_ack <= sys_en;          sys_rdata <= ctr_value[64-1:32]			        ; end
	 
	 20'h00164 : begin sys_ack <= sys_en;          sys_rdata <= timestamp_trigger[32-1:0]           ; end
	 20'h00168 : begin sys_ack <= sys_en;          sys_rdata <= timestamp_trigger[64-1:32]	        ; end
     
     20'h0016c : begin sys_ack <= sys_en;          sys_rdata <= {{32-1{1'b0}}, pretrig_ok}          ; end

     // DMA point-cloud packet-format descriptor (read-only, self-describing).
     // Lets the host configure its UDP unpacker from hardware instead of hardcoding.
     //   0x170 [7:0]=FSZ [15:8]=FRAC [23:16]=HSZ [31:24]=FMT_VERSION
     //   0x194 [15:0]=HIST_BLOCK_SIZE [19:16]=channel-field width
     // Data-word peak field width IDX = FSZ+FRAC; host: bin = field / 2^FRAC.
     // (FRAC was previously exposed standalone here; it is now the [15:8] sub-field.)
     20'h00098 : begin sys_ack <= sys_en;          sys_rdata <= {28'h0, dma_nch}                          ; end
     20'h0009C : begin sys_ack <= sys_en;          sys_rdata <= {31'h0, dma_int_en}                       ; end
     // Version byte is LIVE: base +2 while the runtime intensity enable is on,
     // matching what the packets stamp (after the next packet boundary).
     20'h00170 : begin sys_ack <= sys_en;          sys_rdata <= {8'(DMA_FMT_VERSION + (dma_int_en ? 8'd2 : 8'd0)),
                                                                 DMA_FMT_HSZ, DMA_FMT_FRAC, DMA_FMT_FSZ}  ; end
     // Peak-bin sub-bin FRACTION (the low FRAC bits of k_interp), per channel.
     // 0x174/0x178 pack {down, up} for channel A/B — each the FRAC-bit fraction
     // zero-extended to 16, down in [31:16], up in [15:0]. Combine with the integer
     // bin from 0x44/0x7C: bin = integer + fraction/2^FRAC. All read 0 when FRAC=0.
     20'h00174 : begin sys_ack <= sys_en;          sys_rdata <= fft_peak_frac_a                         ; end
     20'h00178 : begin sys_ack <= sys_en;          sys_rdata <= fft_peak_frac_b                         ; end

     20'h00188 : begin sys_ack <= sys_en;          sys_rdata <= {{16-RSZ{1'b0}}, y_step, {16-RSZ{1'b0}}, x_step}; end
     20'h0018C : begin sys_ack <= sys_en;          sys_rdata <= scope_sig_dly                     ; end
     20'h00190 : begin sys_ack <= sys_en;          sys_rdata <= fft_overflow_cnt                    ; end
     // DMA packet geometry (read-only): data words per packet + channel field width
     //   + per-packet seq width. 0x194 [15:0]=HIST_BLOCK_SIZE [19:16]=channel width
     //   [27:20]=DMA_SEQ_W (header low bits used as a per-packet sequence; 0 = none).
     // [28] = runtime-intensity capable (the 0x9C enable register exists);
     // older bitstreams read 0 there, so the host can feature-detect.
     20'h00194 : begin sys_ack <= sys_en;          sys_rdata <= {3'h0, 1'b1, 8'(DMA_SEQ_W), 4'd4, DMA_PKT_BLK}    ; end
     // 20'h00198 : begin sys_ack <= sys_en;          sys_rdata <= fft_we_cnt[1]                       ; end

     20'h1???? : begin sys_ack <= adc_rd_dv;       sys_rdata <= {16'h0, 2'h0,adc_a_rd}              ; end
     20'h2???? : begin sys_ack <= adc_rd_dv;       sys_rdata <= {16'h0, 2'h0,adc_b_rd}              ; end

     // 20'h3???? : begin sys_ack <= fft_rd_dv;       sys_rdata <= sys_addr[2] ? (fft_parallel ? fft_rdata_up_b : fft_rdata_down_a) : fft_rdata_up_a; end
     20'h3???? : begin sys_ack <= fft_rd_dv;       sys_rdata <= sys_addr[2] ? fft_rdata_down_a : fft_rdata_up_a; end
     20'h4???? : begin sys_ack <= fft_rd_dv;       sys_rdata <= sys_addr[2] ? fft_rdata_down_b : fft_rdata_up_b; end

     // Peak-index history readback. Each entry is k_interp (Q(FSZ).FRAC, IDX bits),
     // too wide to pack up+down in one 32-bit word, so up/down get separate address
     // ranges (all read the same hist position from sys_addr): 0x5=up_a, 0x7=down_a,
     // 0x6=up_b, 0x8=down_b. Host recovers fractional bin = value / 2^FRAC.
     20'h5???? : begin sys_ack <= fft_rd_dv;       sys_rdata <= {{32-IDX{1'b0}}, fft_hist_rdata_up_a}   ; end
     20'h7???? : begin sys_ack <= fft_rd_dv;       sys_rdata <= {{32-IDX{1'b0}}, fft_hist_rdata_down_a} ; end
     20'h6???? : begin sys_ack <= fft_rd_dv;       sys_rdata <= {{32-IDX{1'b0}}, fft_hist_rdata_up_b}   ; end
     20'h8???? : begin sys_ack <= fft_rd_dv;       sys_rdata <= {{32-IDX{1'b0}}, fft_hist_rdata_down_b} ; end

       default : begin sys_ack <= sys_en;          sys_rdata <=  32'h0                              ; end
   endcase
end

endmodule
