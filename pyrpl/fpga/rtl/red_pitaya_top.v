/**
 * $Id: red_pitaya_top.v 1271 2014-02-25 12:32:34Z matej.oblak $
 *
 * @brief Red Pitaya TOP module. It connects external pins and PS part with 
 *        other application modules. 
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
 * Top module connects PS part with rest of Red Pitaya applications.  
 *
 *
 *                   /-------\      
 *   PS DDR <------> |  PS   |      AXI <-> custom bus
 *   PS MIO <------> |   /   | <------------+
 *   PS CLK -------> |  ARM  |              |
 *                   \-------/              |
 *                                          |
 *                            /-------\     |
 *                         -> | SCOPE | <---+
 *                         |  \-------/     |
 *                         |                |
 *            /--------\   |   /-----\      |
 *   ADC ---> |        | --+-> |     |      |
 *            | ANALOG |       | DSP | <----+
 *   DAC <--- |        | <---- |     |      |
 *            \--------/   ^   \-----/      |
 *                         |                |
 *                         |  /-------\     |
 *                         -- |  ASG  | <---+ 
 *                            \-------/     |
 *                                          |
 *             /--------\                   |
 *    RX ----> |        |                   |
 *   SATA      | DAISY  | <-----------------+
 *    TX <---- |        | 
 *             \--------/ 
 *               |    |
 *               |    |
 *               (FREE)
 *
 *
 * Inside analog module, ADC data is translated from unsigned neg-slope into
 * two's complement. Similar is done on DAC data.
 *
 * Scope module stores data from ADC into RAM, arbitrary signal generator (ASG)
 * sends data from RAM to DAC. MIMO PID uses ADC ADC as input and DAC as its output.
 *
 * Daisy chain connects with other boards with fast serial link. Data which is
 * send and received is at the moment undefined. This is left for the user.
 * 
 */

module red_pitaya_top #(
    CLK_DIFF = 1,
    CLK_DIV = 1,
    CLK_MULT = 8,
    CLK_ADC_DIV = 8,
    ADC_SZ = 14,
    FFT_NFFT = 13,
    FFT_SSR  = 1,
    FFT_WIDTH = 28,
    FFT_FRAC = 8,
    FFT_IMPL = 3,
    FFT_SINGLE = 0,
    HIST_BLOCK_SIZE = 183,
    HSZ = 24
)(
   // PS connections
   inout  [54-1: 0] FIXED_IO_mio       ,
   inout            FIXED_IO_ps_clk    ,
   inout            FIXED_IO_ps_porb   ,
   inout            FIXED_IO_ps_srstb  ,
   inout            FIXED_IO_ddr_vrn   ,
   inout            FIXED_IO_ddr_vrp   ,
   // DDR
   inout  [15-1: 0] DDR_addr           ,
   inout  [ 3-1: 0] DDR_ba             ,
   inout            DDR_cas_n          ,
   inout            DDR_ck_n           ,
   inout            DDR_ck_p           ,
   inout            DDR_cke            ,
   inout            DDR_cs_n           ,
   inout  [ 4-1: 0] DDR_dm             ,
   inout  [32-1: 0] DDR_dq             ,
   inout  [ 4-1: 0] DDR_dqs_n          ,
   inout  [ 4-1: 0] DDR_dqs_p          ,
   inout            DDR_odt            ,
   inout            DDR_ras_n          ,
   inout            DDR_reset_n        ,
   inout            DDR_we_n           ,

   // Red Pitaya periphery
  
   // ADC
   input  [16-1: 2] adc_dat_a_i        ,  // ADC CH1
   input  [16-1: 2] adc_dat_b_i        ,  // ADC CH2
   input            adc_clk_p_i        ,  // ADC data clock
   input            adc_clk_n_i        ,  // ADC data clock
   output [ 2-1: 0] adc_clk_o          ,  // optional ADC clock source
   output           adc_cdcs_o         ,  // ADC clock duty cycle stabilizer
   // DAC
   output [14-1: 0] dac_dat_o          ,  // DAC combined data
   output           dac_wrt_o          ,  // DAC write
   output           dac_sel_o          ,  // DAC channel select
   output           dac_clk_o          ,  // DAC clock
   output           dac_rst_o          ,  // DAC reset
   // PWM DAC
   output [ 4-1: 0] dac_pwm_o          ,  // serial PWM DAC
   // XADC
   input  [ 5-1: 0] vinp_i             ,  // voltages p
   input  [ 5-1: 0] vinn_i             ,  // voltages n
   // Expansion connector
   inout  [ 8-1: 0] exp_p_io           ,
   inout  [ 8-1: 0] exp_n_io           ,
   // SATA connector
   output [ 2-1: 0] daisy_p_o          ,  // line 1 is clock capable
   output [ 2-1: 0] daisy_n_o          ,
   input  [ 2-1: 0] daisy_p_i          ,  // line 1 is clock capable
   input  [ 2-1: 0] daisy_n_i          ,
   // LED
   output [ 8-1: 0] led_o       
);

//---------------------------------------------------------------------------------
//
//  Connections to PS

wire  [  4-1: 0] fclk               ; //[0]-125MHz, [1]-250MHz, [2]-50MHz, [3]-200MHz
wire  [  4-1: 0] frstn              ;

wire             ps_sys_clk         ;
wire             ps_sys_rstn        ;
wire  [ 32-1: 0] ps_sys_addr        ;
wire  [ 32-1: 0] ps_sys_wdata       ;
wire  [  4-1: 0] ps_sys_sel         ;
wire             ps_sys_wen         ;
wire             ps_sys_ren         ;
wire  [ 32-1: 0] ps_sys_rdata       ;
wire             ps_sys_err         ;
wire             ps_sys_ack         ;

// AXI masters
wire             axi1_clk    , axi0_clk    ;
wire             axi1_rstn   , axi0_rstn   ;
wire  [ 32-1: 0] axi1_waddr  , axi0_waddr  ;
wire  [ 64-1: 0] axi1_wdata  , axi0_wdata  ;
wire  [  8-1: 0] axi1_wsel   , axi0_wsel   ;
wire             axi1_wvalid , axi0_wvalid ;
wire  [  4-1: 0] axi1_wlen   , axi0_wlen   ;
wire             axi1_wfixed , axi0_wfixed ;
wire             axi1_werr   , axi0_werr   ;
wire             axi1_wrdy   , axi0_wrdy   ;

// HP2 wires for dma_s2mm → i_ps
wire [ 31:0] hp2_awaddr;
wire [  3:0] hp2_awlen;
wire [  2:0] hp2_awsize;
wire [  1:0] hp2_awburst;
wire [  1:0] hp2_awlock;
wire [  3:0] hp2_awcache;
wire [  2:0] hp2_awprot;
wire [  3:0] hp2_awqos;
wire [  5:0] hp2_awid;
wire         hp2_awvalid, hp2_awready;
wire [ 63:0] hp2_wdata;
wire [  7:0] hp2_wstrb;
wire         hp2_wlast;
wire [  5:0] hp2_wid;
wire         hp2_wvalid, hp2_wready;
wire         hp2_bvalid, hp2_bready;
wire [  1:0] hp2_bresp;
wire [  5:0] hp2_bid;

red_pitaya_ps i_ps (
  .FIXED_IO_mio       (  FIXED_IO_mio                ),
  .FIXED_IO_ps_clk    (  FIXED_IO_ps_clk             ),
  .FIXED_IO_ps_porb   (  FIXED_IO_ps_porb            ),
  .FIXED_IO_ps_srstb  (  FIXED_IO_ps_srstb           ),
  .FIXED_IO_ddr_vrn   (  FIXED_IO_ddr_vrn            ),
  .FIXED_IO_ddr_vrp   (  FIXED_IO_ddr_vrp            ),
  // DDR
  .DDR_addr      (DDR_addr    ),
  .DDR_ba        (DDR_ba      ),
  .DDR_cas_n     (DDR_cas_n   ),
  .DDR_ck_n      (DDR_ck_n    ),
  .DDR_ck_p      (DDR_ck_p    ),
  .DDR_cke       (DDR_cke     ),
  .DDR_cs_n      (DDR_cs_n    ),
  .DDR_dm        (DDR_dm      ),
  .DDR_dq        (DDR_dq      ),
  .DDR_dqs_n     (DDR_dqs_n   ),
  .DDR_dqs_p     (DDR_dqs_p   ),
  .DDR_odt       (DDR_odt     ),
  .DDR_ras_n     (DDR_ras_n   ),
  .DDR_reset_n   (DDR_reset_n ),
  .DDR_we_n      (DDR_we_n    ),

  .fclk_clk_o    (fclk        ),
  .fclk_rstn_o   (frstn       ),
   // system read/write channel
  .sys_clk_o     (ps_sys_clk  ),  // system clock
  .sys_rstn_o    (ps_sys_rstn ),  // system reset - active low
  .sys_addr_o    (ps_sys_addr ),  // system read/write address
  .sys_wdata_o   (ps_sys_wdata),  // system write data
  .sys_sel_o     (ps_sys_sel  ),  // system write byte select
  .sys_wen_o     (ps_sys_wen  ),  // system write enable
  .sys_ren_o     (ps_sys_ren  ),  // system read enable
  .sys_rdata_i   (ps_sys_rdata),  // system read data
  .sys_err_i     (ps_sys_err  ),  // system error indicator
  .sys_ack_i     (ps_sys_ack  ),  // system acknowledge signal

  // AXI masters
  .axi1_clk_i    (axi1_clk    ),  .axi0_clk_i    (axi0_clk    ),  // global clock
  .axi1_rstn_i   (axi1_rstn   ),  .axi0_rstn_i   (axi0_rstn   ),  // global reset
  .axi1_waddr_i  (axi1_waddr  ),  .axi0_waddr_i  (axi0_waddr  ),  // system write address
  .axi1_wdata_i  (axi1_wdata  ),  .axi0_wdata_i  (axi0_wdata  ),  // system write data
  .axi1_wsel_i   (axi1_wsel   ),  .axi0_wsel_i   (axi0_wsel   ),  // system write byte select
  .axi1_wvalid_i (axi1_wvalid ),  .axi0_wvalid_i (axi0_wvalid ),  // system write data valid
  .axi1_wlen_i   (axi1_wlen   ),  .axi0_wlen_i   (axi0_wlen   ),  // system write burst length
  .axi1_wfixed_i (axi1_wfixed ),  .axi0_wfixed_i (axi0_wfixed ),  // system write burst type (fixed / incremental)
  .axi1_werr_o   (axi1_werr   ),  .axi0_werr_o   (axi0_werr   ),  // system write error
  .axi1_wrdy_o   (axi1_wrdy   ),  .axi0_wrdy_o   (axi0_wrdy   ),  // system write ready
  // HP2 — point cloud DMA (125 MHz, matches i_dma_s2mm clk_i)
  .hp2_aclk_i    (adc_clk     ),
  .hp2_awaddr_i  (hp2_awaddr  ),  .hp2_awready_o (hp2_awready ),
  .hp2_awlen_i   (hp2_awlen   ),
  .hp2_awsize_i  (hp2_awsize  ),
  .hp2_awburst_i (hp2_awburst ),
  .hp2_awlock_i  (hp2_awlock  ),
  .hp2_awcache_i (hp2_awcache ),
  .hp2_awprot_i  (hp2_awprot  ),
  .hp2_awqos_i   (hp2_awqos   ),
  .hp2_awid_i    (hp2_awid    ),
  .hp2_awvalid_i (hp2_awvalid ),
  .hp2_wdata_i   (hp2_wdata   ),  .hp2_wready_o  (hp2_wready  ),
  .hp2_wstrb_i   (hp2_wstrb   ),
  .hp2_wlast_i   (hp2_wlast   ),
  .hp2_wid_i     (hp2_wid     ),
  .hp2_wvalid_i  (hp2_wvalid  ),
  .hp2_bvalid_o  (hp2_bvalid  ),  .hp2_bready_i  (hp2_bready  ),
  .hp2_bresp_o   (hp2_bresp   ),
  .hp2_bid_o     (hp2_bid     )
);

////////////////////////////////////////////////////////////////////////////////
// system bus decoder & multiplexer (it breaks memory addresses into 8 regions)
////////////////////////////////////////////////////////////////////////////////

wire              sys_clk   = ps_sys_clk  ;
wire              sys_rstn  = ps_sys_rstn ;
wire  [  32-1: 0] sys_addr  = ps_sys_addr ;
wire  [  32-1: 0] sys_wdata = ps_sys_wdata;
wire  [   4-1: 0] sys_sel   = ps_sys_sel  ;
wire  [8   -1: 0] sys_wen   ;
wire  [8   -1: 0] sys_ren   ;
wire  [8*32-1: 0] sys_rdata ;
wire  [8* 1-1: 0] sys_err   ;
wire  [8* 1-1: 0] sys_ack   ;
wire  [8   -1: 0] sys_cs    ;

assign sys_cs = 8'h01 << sys_addr[23:21];

assign sys_wen = sys_cs & {8{ps_sys_wen}};
assign sys_ren = sys_cs & {8{ps_sys_ren}};

assign ps_sys_rdata = sys_rdata[sys_addr[23:21]*32+:32];

assign ps_sys_err   = |(sys_cs & sys_err);
assign ps_sys_ack   = |(sys_cs & sys_ack);

// unused system bus slave ports

// DMA descriptor (slot 5, base 0x40A00000):
//   0x0 = ring-buffer write pointer (word index; PS/monitor_server polls this)
//   0x4 = HIST_BLOCK_SIZE — data words per packet (full packet = +1 header word),
//         so monitor_server discovers the packet size instead of hardcoding it.
// i_dma_s2mm now runs on adc_clk, which is the same net as the system-bus clock
// (sys_clk = axi0_clk_o = adc_clk). dma_wr_ptr is therefore already in the
// sys_clk domain — the former fft_clk->sys_clk CDC is no longer needed.
localparam [15:0] DMA_PKT_BLK = HIST_BLOCK_SIZE;
assign sys_rdata[5*32+:32] = sys_addr[2]
                           ? {16'h0, DMA_PKT_BLK}
                           : {{(32-$clog2(16384)){1'b0}}, dma_wr_ptr};
assign sys_err  [5       ] =  1'b0;
assign sys_ack  [5       ] =  1'b1;

assign sys_rdata[6*32+:32] = 32'h0; 
assign sys_err  [6       ] =  1'b0;
assign sys_ack  [6       ] =  1'b1;

assign sys_rdata[7*32+:32] = 32'h0; 
assign sys_err  [7       ] =  1'b0;
assign sys_ack  [7       ] =  1'b1;

////////////////////////////////////////////////////////////////////////////////
// local signals
////////////////////////////////////////////////////////////////////////////////

// PLL signals
wire                  adc_clk_in;
wire                  pll_adc_clk;
wire                  pll_dac_clk_1x;
wire                  pll_dac_clk_2x;
wire                  pll_dac_clk_2p;
wire                  pll_ser_clk;
wire                  pll_pwm_clk;
wire                  pll_locked;

// fast serial signals
wire                  ser_clk ;

// PWM clock and reset
wire                  pwm_clk ;
reg                   pwm_rstn;

// ADC signals
wire                  adc_clk;
reg                   adc_rstn_i;
wire                  adc_rstn;
reg          [14-1:0] adc_dat_a, adc_dat_b;
wire  signed [14-1:0] adc_a    , adc_b    ;

// DAC signals
wire                  dac_clk_1x;
wire                  dac_clk_2x;
wire                  dac_clk_2p;
reg                   dac_rst;
reg          [14-1:0] dac_dat_a, dac_dat_b;
wire         [14-1:0] dac_a    , dac_b    ;

// ASG
wire  signed [14-1:0] asg_a    , asg_b    , asg_c    , asg_d    ;

// configuration
wire                  digital_loop;

////////////////////////////////////////////////////////////////////////////////
// PLL (clock and reaset)
////////////////////////////////////////////////////////////////////////////////

if (CLK_DIFF) begin
    // diferential clock input
    IBUFDS i_clk (.I (adc_clk_p_i), .IB (adc_clk_n_i), .O (adc_clk_in));  // differential clock input
end else begin
    assign adc_clk_in = adc_clk_p_i;
end

red_pitaya_pll #(.DIV(CLK_DIV), .MULT(CLK_MULT), .DIV_ADC(CLK_ADC_DIV)) pll (
  // inputs
  .clk         (adc_clk_in),  // clock
  .rstn        (frstn[0]  ),  // reset - active low
  // output clocks
  .clk_adc     (pll_adc_clk   ),  // ADC clock
  .clk_dac_1x  (pll_dac_clk_1x),  // DAC clock 125MHz
  .clk_dac_2x  (pll_dac_clk_2x),  // DAC clock 250MHz
  .clk_dac_2p  (pll_dac_clk_2p),  // DAC clock 250MHz -45DGR
  .clk_ser     (pll_ser_clk   ),  // fast serial clock (FFT_CLK_200 -> 200 MHz)
  .clk_pwm     (pll_pwm_clk   ),  // PWM clock
  // status outputs
  .pll_locked  (pll_locked)
);

BUFG bufg_adc_clk    (.O (adc_clk   ), .I (pll_adc_clk   ));
BUFG bufg_dac_clk_1x (.O (dac_clk_1x), .I (pll_dac_clk_1x));
BUFG bufg_dac_clk_2x (.O (dac_clk_2x), .I (pll_dac_clk_2x));
BUFG bufg_dac_clk_2p (.O (dac_clk_2p), .I (pll_dac_clk_2p));
BUFG bufg_ser_clk    (.O (ser_clk   ), .I (pll_ser_clk   ));
BUFG bufg_pwm_clk    (.O (pwm_clk   ), .I (pll_pwm_clk   ));

wire fft_clk;
assign fft_clk = ser_clk;

// ADC reset (active low)
always @(posedge adc_clk)
adc_rstn_i <=  frstn[0] &  pll_locked;

BUFG bufg_rstn    (.O (adc_rstn   ), .I (adc_rstn_i   ));

// DAC reset (active high)
always @(posedge dac_clk_1x)
dac_rst  <= ~frstn[0] | ~pll_locked;

// PWM reset (active low)
always @(posedge pwm_clk)
pwm_rstn <=  frstn[0] &  pll_locked;

////////////////////////////////////////////////////////////////////////////////
// ADC IO
////////////////////////////////////////////////////////////////////////////////

// generating ADC clock is disabled
assign adc_clk_o = 2'b10;
//ODDR i_adc_clk_p ( .Q(adc_clk_o[0]), .D1(1'b1), .D2(1'b0), .C(fclk[0]), .CE(1'b1), .R(1'b0), .S(1'b0));
//ODDR i_adc_clk_n ( .Q(adc_clk_o[1]), .D1(1'b0), .D2(1'b1), .C(fclk[0]), .CE(1'b1), .R(1'b0), .S(1'b0));

// ADC clock duty cycle stabilizer is enabled
assign adc_cdcs_o = 1'b1 ;

// IO block registers should be used here
// lowest 2 bits reserved for 16bit ADC
always @(posedge adc_clk)
begin
  adc_dat_a <= adc_dat_a_i[16-1:2];
  adc_dat_b <= adc_dat_b_i[16-1:2];
end
    
// transform into 2's complement (negative slope)
`ifdef DSP_FB_PIPELINE
// Pipeline the digital-loopback feedback (dac->adc) to break the per-cycle adc_clk
// recurrence: sum{1,2} -> red_pitaya_saturate -> dac_{a,b} -> adc_{a,b} -> i_dsp
// input mux -> module input filter (iq/trigger/scope lpf) -> delta_reg, the binding
// pll_adc_clk path at ~178 MHz once input_select is multicycled. Only the loopback
// arm is registered; the real ADC input (digital_loop=0) keeps its original latency.
// Cost: +1 adc_clk cycle of loopback latency, taken only in digital_loop test/lock
// mode. Build-gated (DSP_FB_PIPELINE), default OFF.
reg [14-1:0] dac_a_lb, dac_b_lb;
always @(posedge adc_clk) begin
  dac_a_lb <= dac_a;
  dac_b_lb <= dac_b;
end
assign adc_a = digital_loop ? dac_a_lb : {adc_dat_a[14-1], ~adc_dat_a[14-2:0]};
assign adc_b = digital_loop ? dac_b_lb : {adc_dat_b[14-1], ~adc_dat_b[14-2:0]};
`else
assign adc_a = digital_loop ? dac_a : {adc_dat_a[14-1], ~adc_dat_a[14-2:0]};
assign adc_b = digital_loop ? dac_b : {adc_dat_b[14-1], ~adc_dat_b[14-2:0]};
`endif

////////////////////////////////////////////////////////////////////////////////
// DAC IO
////////////////////////////////////////////////////////////////////////////////

// output registers + signed to unsigned (also to negative slope)
always @(posedge dac_clk_1x)
begin
  dac_dat_a <= {dac_a[14-1], ~dac_a[14-2:0]};
  dac_dat_b <= {dac_b[14-1], ~dac_b[14-2:0]};
end

// DDR outputs
ODDR oddr_dac_clk          (.Q(dac_clk_o), .D1(1'b0     ), .D2(1'b1     ), .C(dac_clk_2p), .CE(1'b1), .R(1'b0   ), .S(1'b0));
ODDR oddr_dac_wrt          (.Q(dac_wrt_o), .D1(1'b0     ), .D2(1'b1     ), .C(dac_clk_2x), .CE(1'b1), .R(1'b0   ), .S(1'b0));
ODDR oddr_dac_sel          (.Q(dac_sel_o), .D1(1'b1     ), .D2(1'b0     ), .C(dac_clk_1x), .CE(1'b1), .R(dac_rst), .S(1'b0));
ODDR oddr_dac_rst          (.Q(dac_rst_o), .D1(dac_rst  ), .D2(dac_rst  ), .C(dac_clk_1x), .CE(1'b1), .R(1'b0   ), .S(1'b0));
ODDR oddr_dac_dat [14-1:0] (.Q(dac_dat_o), .D1(dac_dat_b), .D2(dac_dat_a), .C(dac_clk_1x), .CE(1'b1), .R(dac_rst), .S(1'b0));

//---------------------------------------------------------------------------------
//  House Keeping

wire  [  8-1: 0] exp_p_in , exp_n_in ;
wire  [  8-1: 0] exp_p_out, exp_n_out;
wire  [  8-1: 0] exp_p_dir, exp_n_dir;

wire scope_fft_o;
wire [2-1:0] fft_window;   // scope fft acq windows {down, up} -> dsp pid gating
wire scope_sig_o;
wire [ 4-1: 0] asg_play_active;   // per-ASG-channel playing (dac_do); [2] = asg3 chirp
wire           enc_trig_tick;     // gated encoder tick (scope enc block -> ASG enc_tick)
wire x_step_0;
wire y_step_0;
// DO NOT DELETE THIS MUX TO FREE LOGIC. Removing it (2026-09-11) cost n11
// ~0.38 ns of setup and made every placement directive fail: these four taps
// anchor scope logic to the exp_p IOBs at the die edge, and without them the
// cluster collapses inward. At ~90% LUT this design is congestion-bound, not
// area-bound — the mux-less netlist was 38 LUT SMALLER and 0.2 ns WORSE.
// Restoring it took det10 x AggressiveExplore from -0.220 to +0.160.
// scope_debug_en now resets to 0 (red_pitaya_hk.v), so the taps are wired but
// not driving pins unless the host opts in.
wire [3:0] scope_sigs = {y_step_0, x_step_0, scope_sig_o, scope_fft_o};

wire    [14-1: 0] scan_x;
wire    [14-1: 0] scan_y;
wire    [14-1: 0] scan_x_step;
wire    [14-1: 0] scan_y_step;

red_pitaya_hk i_hk (
  // system signals
  .clk_i           (  adc_clk                    ),  // clock
  .rstn_i          (  adc_rstn                   ),  // reset - active low
  // LED
  .led_o           (  led_o                      ),  // LED output
  // global configuration
  .digital_loop    (  digital_loop               ),
  // Expansion connector
  .exp_p_dat_i     (  exp_p_in                   ),  // input data
  .exp_p_dat_o     (  exp_p_out                  ),  // output data
  .exp_p_dir_o     (  exp_p_dir                  ),  // 1-output enable
  .exp_n_dat_i     (  exp_n_in                   ),
  .exp_n_dat_o     (  exp_n_out                  ),
  .exp_n_dir_o     (  exp_n_dir                  ),

  .scope_sigs_i    (  scope_sigs                 ),

  .scan_x_i        (  scan_x                     ),
  .scan_x_step_i   (  scan_x_step                ),
  .scan_y_i        (  scan_y                     ),
  .scan_y_step_i   (  scan_y_step                ),

   // System bus
  .sys_addr        (  sys_addr                   ),  // address
  .sys_wdata       (  sys_wdata                  ),  // write data
  .sys_sel         (  sys_sel                    ),  // write byte select
  .sys_wen         (  sys_wen[0]                 ),  // write enable
  .sys_ren         (  sys_ren[0]                 ),  // read enable
  .sys_rdata       (  sys_rdata[ 0*32+31: 0*32]  ),  // read data
  .sys_err         (  sys_err[0]                 ),  // error indicator
  .sys_ack         (  sys_ack[0]                 )   // acknowledge signal
);

IOBUF i_iobufp [8-1:0] (.O(exp_p_in), .IO(exp_p_io), .I(exp_p_out), .T(~exp_p_dir) );
IOBUF i_iobufn [8-1:0] (.O(exp_n_in), .IO(exp_n_io), .I(exp_n_out), .T(~exp_n_dir) );

//---------------------------------------------------------------------------------
//  Oscilloscope application

wire    [  4-1:0] trig_asg_out;
wire trig_scope_out;
wire    [14-1: 0] to_scope_a;
wire    [14-1: 0] to_scope_b;
wire dsp_trigger;

// ---- Scope-input feedback pipeline (build-gated, default OFF) --------------
// The i_dsp module sum (scope{1,2}_o -> to_scope_{a,b}) routes into the scope's
// ADC-input capture (i_scope/adc_{a,b}_dat). At high SSR / large NFFT that route is
// congestion-bound and becomes the worst pll_adc_clk path (sum2_reg -> adc_b_dat_reg,
// ~73% route). Register it here to split the route. Symmetric on BOTH channels so
// CH1/CH2 stay sample-aligned (scope + FFT see a uniform +1 adc_clk latency). This is
// a SEPARATE knob from DSP_FB_PIPELINE (which registers the dac->adc loopback arm) —
// this one is the module-sum->scope path. Default OFF (no latency change).
`ifdef SCOPE_FB_PIPELINE
reg  [14-1: 0] to_scope_a_p, to_scope_b_p;
always @(posedge adc_clk) begin
  to_scope_a_p <= to_scope_a;
  to_scope_b_p <= to_scope_b;
end
wire [14-1: 0] scope_a_in = to_scope_a_p;
wire [14-1: 0] scope_b_in = to_scope_b_p;
`else
wire [14-1: 0] scope_a_in = to_scope_a;
wire [14-1: 0] scope_b_in = to_scope_b;
`endif

wire [ 63:0] scope_dma_a_tdata,  scope_dma_b_tdata;
wire         scope_dma_a_tvalid, scope_dma_b_tvalid;
wire         scope_dma_a_tready, scope_dma_b_tready;
wire         scope_dma_a_tlast,  scope_dma_b_tlast;

wire [$clog2(16384)-1:0] dma_wr_ptr;

dma_s2mm #(
    .BUF_BASE  (32'h1e000000),
    .BUF_WORDS (16384)
) i_dma_s2mm (
    .clk_i        (adc_clk             ),  // 125 MHz: DMA reads fifo_dma_out on adc clk (CDC in FIFO)
    .rstn_i       (adc_rstn            ),
    .a_tdata_i    (scope_dma_a_tdata   ),
    .a_tvalid_i   (scope_dma_a_tvalid  ),
    .a_tready_o   (scope_dma_a_tready  ),
    .a_tlast_i    (scope_dma_a_tlast   ),
    .b_tdata_i    (scope_dma_b_tdata   ),
    .b_tvalid_i   (scope_dma_b_tvalid  ),
    .b_tready_o   (scope_dma_b_tready  ),
    .b_tlast_i    (scope_dma_b_tlast   ),
    .wr_ptr_o     (dma_wr_ptr          ),
    .axi_awaddr_o (hp2_awaddr          ),
    .axi_awlen_o  (hp2_awlen           ),
    .axi_awsize_o (hp2_awsize          ),
    .axi_awburst_o(hp2_awburst         ),
    .axi_awlock_o (hp2_awlock          ),
    .axi_awcache_o(hp2_awcache         ),
    .axi_awprot_o (hp2_awprot          ),
    .axi_awqos_o  (hp2_awqos           ),
    .axi_awid_o   (hp2_awid            ),
    .axi_awvalid_o(hp2_awvalid         ),
    .axi_awready_i(hp2_awready         ),
    .axi_wdata_o  (hp2_wdata           ),
    .axi_wstrb_o  (hp2_wstrb           ),
    .axi_wlast_o  (hp2_wlast           ),
    .axi_wid_o    (hp2_wid             ),
    .axi_wvalid_o (hp2_wvalid          ),
    .axi_wready_i (hp2_wready          ),
    .axi_bvalid_i (hp2_bvalid          ),
    .axi_bresp_i  (hp2_bresp           ),
    .axi_bid_i    (hp2_bid             ),
    .axi_bready_o (hp2_bready          ),
    .axi_arvalid_o(                    ),
    .axi_rready_o (                    )
);

red_pitaya_scope #(.ASZ(ADC_SZ), .FSZ(FFT_NFFT), .FSSR(FFT_SSR), .DSZ(FFT_WIDTH), .FRAC(FFT_FRAC), .FFT_IMPL(FFT_IMPL),
                   .FFT_SINGLE(FFT_SINGLE), .HIST_BLOCK_SIZE(HIST_BLOCK_SIZE), .HSZ(HSZ)) i_scope (
  // ADC
  .adc_a_i         (  scope_a_in[14-1:14-ADC_SZ] ),  // CH 1 (optionally pipelined, see SCOPE_FB_PIPELINE)
  .adc_b_i         (  scope_b_in[14-1:14-ADC_SZ] ),  // CH 2 (optionally pipelined, see SCOPE_FB_PIPELINE)
  .adc_clk_i       (  adc_clk                    ),  // clock
  .adc_rstn_i      (  adc_rstn                   ),  // reset - active low
  .trig_ext_i      (  exp_p_in[0]                ),  // external trigger ONLY (DIO0_P)
  .trig_extn_i     (  exp_n_in[2]                ),  // encoder index I (DIO2_N)
  .trig_quad_i     (  exp_p_in[1]                ),  // encoder channel B (DIO1_P, Scanner360 v3)
  .trig_quadn_i    (  exp_n_in[3]                ),  // encoder channel A (DIO3_N)
                                                     // DIO0_P to free it for the external trigger
  .asg_busy_i      (  asg_play_active[2]         ),  // asg3 (chirp) playing — tick gate
  .trig_enc_o      (  enc_trig_tick              ),  // gated encoder tick -> ASG enc_tick source
  .trig_asg_i      (  trig_asg_out               ),  // ASG trigger
  .trig_dsp_i      (  dsp_trigger                ),
  .trig_scope_o    (  trig_scope_out             ),  // scope trigger to feed other instruments

  .fft_clk_i       (  fft_clk                    ),
  .fft_active_o    (  scope_fft_o                ),  // scope acquisition done signal
  .fft_window_o    (  fft_window                 ),  // acq windows {down, up} for PID gating
  .scope_sig_o     (  scope_sig_o                ),
  .x_step_0        (  x_step_0                   ),
  .y_step_0        (  y_step_0                   ),
  .sync_rst_i      (  asg_sync_rst_o             ),

  .dma_a_tdata     (  scope_dma_a_tdata          ),
  .dma_a_tvalid    (  scope_dma_a_tvalid         ),
  .dma_a_tready    (  scope_dma_a_tready         ),
  .dma_a_tlast     (  scope_dma_a_tlast          ),

  .dma_b_tdata     (  scope_dma_b_tdata          ),
  .dma_b_tvalid    (  scope_dma_b_tvalid         ),
  .dma_b_tready    (  scope_dma_b_tready         ),
  .dma_b_tlast     (  scope_dma_b_tlast          ),


  .x_step_i        (  scan_x_step                ),
  .y_step_i        (  scan_y_step                ),

  // AXI0 master                 // AXI1 master
  .axi0_clk_o    (axi0_clk   ),  .axi1_clk_o    (axi1_clk   ),
  .axi0_rstn_o   (axi0_rstn  ),  .axi1_rstn_o   (axi1_rstn  ),
  .axi0_waddr_o  (axi0_waddr ),  .axi1_waddr_o  (axi1_waddr ),
  .axi0_wdata_o  (axi0_wdata ),  .axi1_wdata_o  (axi1_wdata ),
  .axi0_wsel_o   (axi0_wsel  ),  .axi1_wsel_o   (axi1_wsel  ),
  .axi0_wvalid_o (axi0_wvalid),  .axi1_wvalid_o (axi1_wvalid),
  .axi0_wlen_o   (axi0_wlen  ),  .axi1_wlen_o   (axi1_wlen  ),
  .axi0_wfixed_o (axi0_wfixed),  .axi1_wfixed_o (axi1_wfixed),
  .axi0_werr_i   (axi0_werr  ),  .axi1_werr_i   (axi1_werr  ),
  .axi0_wrdy_i   (axi0_wrdy  ),  .axi1_wrdy_i   (axi1_wrdy  ),


  // System bus
  .sys_addr        (  sys_addr                   ),  // address
  .sys_wdata       (  sys_wdata                  ),  // write data
  .sys_sel         (  sys_sel                    ),  // write byte select
  .sys_wen         (  sys_wen[1]                 ),  // write enable
  .sys_ren         (  sys_ren[1]                 ),  // read enable
  .sys_rdata       (  sys_rdata[ 1*32+31: 1*32]  ),  // read data
  .sys_err         (  sys_err[1]                 ),  // error indicator
  .sys_ack         (  sys_ack[1]                 )   // acknowledge signal
);

//---------------------------------------------------------------------------------
//  DAC arbitrary signal generator
wire    [14-1: 0] asg1phase_o;
wire    [14-1: 0] asg1_step;
wire    [14-1: 0] asg2_step;
wire    [14-1: 0] asg3_step;
wire    [14-1: 0] asg4_step;

red_pitaya_asg i_asg (
   // DAC
  .dac_a_o         (  asg_a                      ),  // CH 1
  .dac_b_o         (  asg_b                      ),  // CH 2
  .dac_c_o         (  asg_c                      ),  // CH 3
  .dac_d_o         (  asg_d                      ),  // CH 4
  .dac_clk_i       (  adc_clk                    ),  // clock
  .dac_rstn_i      (  adc_rstn                   ),  // reset - active low
  .trig_a_i        (  exp_p_in[0]                ),
  .trig_b_i        (  exp_p_in[0]                ),
  .trig_c_i        (  exp_p_in[0]                ),
  .trig_d_i        (  exp_p_in[0]                ),
  .trig_enc_i      (  enc_trig_tick              ),  // gated encoder tick (trigger_source=enc_tick)
  .trig_out_o      (  trig_asg_out               ),
  .play_active_o   (  asg_play_active            ),
  .trig_scope_i    (  trig_scope_out             ),
  .scope_trig_i    (  scope_sig_o                ),
  .sync_rst_o      (  asg_sync_rst_o             ),
  .asg1phase_o     (  asg1phase_o                ),

  .step_a_o        (  asg1_step                  ),
  .step_b_o        (  asg2_step                  ),
  .step_c_o        (  asg3_step                  ),
  .step_d_o        (  asg4_step                  ),
  
  // System bus
  .sys_addr        (  sys_addr                   ),  // address
  .sys_wdata       (  sys_wdata                  ),  // write data
  .sys_sel         (  sys_sel                    ),  // write byte select
  .sys_wen         (  sys_wen[2]                 ),  // write enable
  .sys_ren         (  sys_ren[2]                 ),  // read enable
  .sys_rdata       (  sys_rdata[ 2*32+31: 2*32]  ),  // read data
  .sys_err         (  sys_err[2]                 ),  // error indicator
  .sys_ack         (  sys_ack[2]                 )   // acknowledge signal
);

//---------------------------------------------------------------------------------
//  DSP module

red_pitaya_dsp i_dsp (
   // signals
  .clk_i           (  adc_clk                    ),  // clock
  .rstn_i          (  adc_rstn                   ),  // reset - active low
  .fft_window_i    (  fft_window                 ),  // scope fft acq windows for PID gating
  .dat_a_i         (  adc_a                      ),  // in 1
  .dat_b_i         (  adc_b                      ),  // in 2
  .dat_a_o         (  dac_a                      ),  // out 1
  .dat_b_o         (  dac_b                      ),  // out 2
  
  .asg1_i          (  asg_a                  ),
  .asg2_i          (  asg_b                  ),
  .asg3_i          (  asg_c                  ),
  .asg4_i          (  asg_d                  ),
  .scope1_o        (  to_scope_a             ),
  .scope2_o        (  to_scope_b             ),
  .asg1phase_i     (  asg1phase_o            ),
  .asg1_step_i     (  asg1_step             ),
  .asg2_step_i     (  asg2_step             ),
  .asg3_step_i     (  asg3_step             ),
  .asg4_step_i     (  asg4_step             ),

  .scan_x_o        (  scan_x                 ),
  .scan_y_o        (  scan_y                 ),
  .scan_x_step_o   (  scan_x_step            ),
  .scan_y_step_o   (  scan_y_step            ),

  .xadc1_i         (  xadc_signals[0]        ),
  .xadc2_i         (  xadc_signals[1]        ),
  .xadc3_i         (  xadc_signals[2]        ),
  .xadc4_i         (  xadc_signals[3]        ),

  .pwm0            (  pwm_signals[0]         ),
  .pwm1            (  pwm_signals[1]         ),
  .pwm2            (  pwm_signals[2]         ),
  .pwm3            (  pwm_signals[3]         ),

  .trig_o          (  dsp_trigger            ),

  // System bus
  .sys_addr        (  sys_addr                   ),  // address
  .sys_wdata       (  sys_wdata                  ),  // write data
  .sys_sel         (  sys_sel                    ),  // write byte select
  .sys_wen         (  sys_wen[3]                 ),  // write enable
  .sys_ren         (  sys_ren[3]                 ),  // read enable
  .sys_rdata       (  sys_rdata[ 3*32+31: 3*32]  ),  // read data
  .sys_err         (  sys_err[3]                 ),  // error indicator
  .sys_ack         (  sys_ack[3]                 )   // acknowledge signal
);

// the ams module has been obsoleted by PWM control via DSP module (outputs)
// and by the fact that RedPitaya has migrated aux. inputs to be PS controlled
// we keep the module to go back to FPGA controlled aux. inputs if needed

//---------------------------------------------------------------------------------
//  Analog mixed signals
//  XADC and slow PWM DAC control

wire  [ 24-1: 0] pwm_cfg_a;
wire  [ 24-1: 0] pwm_cfg_b;
wire  [ 24-1: 0] pwm_cfg_c;
wire  [ 24-1: 0] pwm_cfg_d;

red_pitaya_ams i_ams (
   // power test
  .clk_i           (  adc_clk                    ),  // clock
  .rstn_i          (  adc_rstn                   ),  // reset - active low
  // ADC analog inputs
  .vinp_i          (  vinp_i                     ),  // voltages p
  .vinn_i          (  vinn_i                     ),  // voltages n
  // PWM configuration
  .dac_a_o         (  pwm_cfg_a                  ),
  .dac_b_o         (  pwm_cfg_b                  ),
  .dac_c_o         (  pwm_cfg_c                  ),
  .dac_d_o         (  pwm_cfg_d                  ),
  .adc_a_r         (  xadc_signals[0]            ),
  .adc_b_r         (  xadc_signals[1]            ),
  .adc_c_r         (  xadc_signals[2]            ),
  .adc_d_r         (  xadc_signals[3]            ),
  .pwm0_i 		   (  pwm_signals[0]             ),
  .pwm1_i 		   (  pwm_signals[1]             ),
  .pwm2_i 		   (  pwm_signals[2]             ),
  .pwm3_i 		   (  pwm_signals[3]             ),
   // System bus
  .sys_addr        (  sys_addr                   ),  // address
  .sys_wdata       (  sys_wdata                  ),  // write data
  .sys_sel         (  sys_sel                    ),  // write byte select
  .sys_wen         (  sys_wen[4]                 ),  // write enable
  .sys_ren         (  sys_ren[4]                 ),  // read enable
  .sys_rdata       (  sys_rdata[ 4*32+31: 4*32]  ),  // read data
  .sys_err         (  sys_err[4]                 ),  // error indicator
  .sys_ack         (  sys_ack[4]                 )   // acknowledge signal
);


wire  [ 12-1: 0] xadc_signals[4-1:0];
wire  [ 14-1: 0] pwm_signals[4-1:0];

red_pitaya_pwm pwm [4-1:0] (
  // system signals
  .clk   (pwm_clk ),
  .rstn  (pwm_rstn),
  // configuration
  .cfg   ({pwm_cfg_d, pwm_cfg_c, pwm_cfg_b, pwm_cfg_a}),
  //.signal_i ({pwm_signals[3],pwm_signals[2],pwm_signals[1],pwm_signals[0]}),
  // PWM outputs
  .pwm_o (dac_pwm_o),
  .pwm_s ()
);

//---------------------------------------------------------------------------------
//  Daisy chain
//  simple communication module

assign daisy_p_o = 1'bz;
assign daisy_n_o = 1'bz;

endmodule
