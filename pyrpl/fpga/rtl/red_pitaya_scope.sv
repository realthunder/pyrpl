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
  parameter DSZ = 28,  // FFT_output width
  parameter FSZ = 13,  // FFT transform length 2^FSZ
  parameter RSZ = 14,  // RAM size 2^RSZ
  parameter HSZ = 14  // fft history buffer size 2^HSZ
)(

   // ADC
   input                 adc_clk_i       ,  // ADC clock
   input                 adc_rstn_i      ,  // ADC reset - active low
   input      [ 14-1: 0] adc_a_i         ,  // ADC data CHA
   input      [ 14-1: 0] adc_b_i         ,  // ADC data CHB
   // trigger sources
   input                 trig_ext_i      ,  // external trigger
   input      [  4-1: 0] trig_asg_i      ,  // ASG trigger
   input                 trig_dsp_i      ,  // DSP module trigger
   output                trig_scope_o    ,  // copy of scope trigger
   output                scope_done_o    ,  // scope done processing
   input                 sync_rst_i      ,  // syncrhonized reset signal (from ASG)

   input      [RSZ-1: 0] asg2_step_i     ,  // asg2 (1-based index, so it is the second asg channel) current step index
   input      [RSZ-1: 0] asg3_step_i     ,  // asg3 current step index

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

wire [ 14-1: 0] adc_a_filt_in  ;
wire [ 14-1: 0] adc_a_filt_out ;
wire [ 14-1: 0] adc_b_filt_in  ;
wire [ 14-1: 0] adc_b_filt_out ;
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

reg  [ 14-1: 0] adc_a_dat     ;
reg  [ 14-1: 0] adc_b_dat     ;
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

reg   [  14-1: 0] adc_a_buf [0:(1<<RSZ)-1] ;
reg   [  14-1: 0] adc_b_buf [0:(1<<RSZ)-1] ;
reg   [  14-1: 0] adc_a_rd      ;
reg   [  14-1: 0] adc_b_rd      ;
reg   [ RSZ-1: 0] adc_wp        ;
reg   [ RSZ-1: 0] adc_raddr     ;
reg   [ RSZ-1: 0] adc_a_raddr   ;
reg   [ RSZ-1: 0] adc_b_raddr   ;
reg   [   4-1: 0] adc_rval      ;
wire              adc_rd_dv     ;
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
reg   [ 14 - 1:0] pretrig_data_min; // make sure this amount of data has been acquired before trig
reg 			  pretrig_ok;

// Write
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
   if (adc_we && adc_dv) begin
      adc_a_buf[adc_wp] <= adc_a_dat ;
      adc_b_buf[adc_wp] <= adc_b_dat ;
   end
end

// Read
always @(posedge adc_clk_i) begin
   if (adc_rstn_i == 1'b0)
      adc_rval <= 4'h0 ;
   else
      adc_rval <= {adc_rval[2:0], (sys_ren || sys_wen)};
end
assign adc_rd_dv = adc_rval[3];

always @(posedge adc_clk_i) begin
   adc_raddr   <= sys_addr[RSZ-1+2:2] ; // address synchronous to clock
   adc_a_raddr <= adc_raddr     ; // double register 
   adc_b_raddr <= adc_raddr     ; // otherwise memory corruption at reading
   adc_a_rd    <= adc_a_buf[adc_a_raddr] ;
   adc_b_rd    <= adc_b_buf[adc_b_raddr] ;
end



//////////////// FFT /////////////////////

logic               fft_enable;
logic               fft_trig_sync;

typedef enum logic [2:0] {
    S_IDLE  = 0,
    S_DELAY = 1,
    S_WAIT1 = 2,
    S_FFT_A = 3,
    S_WAIT2 = 4,
    S_FFT_B = 5
} fft_state_t;

fft_state_t         fft_state;

logic [ RSZ-1: 0]   fft_wait1_cnt;
logic [ RSZ-1: 0]   fft_wait2_cnt;
logic [ RSZ-1: 0]   fft_acq1_cnt;
logic [ RSZ-1: 0]   fft_acq2_cnt;
logic [ RSZ-1: 0]   fft_state_cnt;

localparam IDX_PIPELINE = 4-1;
logic [ HSZ-1: 0]   fft_hist_index[0:IDX_PIPELINE];
logic [ HSZ-1: 0]   fft_hist_step;

logic [ HSZ-1: 0]   fft_hist_raddr;
logic [ 16-1:  0]   fft_hist_rdata_a;
logic [ 16-1:  0]   fft_hist_rdata_b;
logic [ FSZ-1: 0]   fft_raddr;
logic [ FSZ-1: 0]   fft_wp_last[0:1];
logic [ DSZ-1: 0]   fft_rdata_a;
logic [ DSZ-1: 0]   fft_rdata_b;

assign fft_raddr = sys_addr[FSZ-1+3:3] ;
assign fft_hist_raddr = sys_addr[HSZ-1+2:2]  ;

logic [ 16-1: 0]    fft_threshold_k;
logic [ FSZ-1:0]    fft_peak_start;
logic [ DSZ-1: 0]   fft_peak_minimum;

logic [ 6-1 :  0]   fft_status[0:1];
logic [ 2-1 :  0]   fft_done;
logic [ FSZ: 0]     fft_count_a;
logic [ FSZ: 0]     fft_count_b;
logic [FSZ+DSZ-1:0] fft_sum_a;
logic [FSZ+DSZ-1:0] fft_sum_b;
logic [ 8-1 : 0]    fft_peak_state_a;
logic [ 8-1 : 0]    fft_peak_state_b;
logic [ 16-1: 0]    fft_peak_index_a;
logic [ 16-1: 0]    fft_peak_index_b;
logic [ DSZ-1: 0]   fft_peak_a;
logic [ DSZ-1: 0]   fft_peak_b;
logic [ 32-1: 0]    fft_frame_cnt;
logic [ 32-1: 0]    fft_we_cnt[1:0];
logic [ 32-1: 0]    fft_skip_cnt;
// logic               fft_index_flush = adc_rstn_i == 1'b0 || sync_rst_i;
logic               fft_index_flush = sync_rst_i;

logic [ 2-1 :  0]   fft_peak_ready_a;
logic [ 2-1 :  0]   fft_peak_ready_b;
logic [ 2-1 :  0]   fft_peak_ready;

logic [ 2-1:  0]    fft_rst_i;
logic               fft_rstn_i;
always @(posedge adc_clk_i) begin
    fft_rst_i <= {fft_rst_i[0], (fft_trig_sync && adc_rst_do) || sync_rst_i};
end

// assign fft_rstn_i = adc_rstn_i && ~|fft_rst_i;
assign fft_rstn_i = ~|fft_rst_i;

logic fft_trig_i = fft_trig_sync ? (adc_trig && !adc_dly_do && pretrig_ok) : fft_trig;

// (* mark_debug = "true" *)
logic fft_dvalid = (!fft_trig_sync || adc_we) && adc_dv; 
logic fft_a_enable = fft_state == S_FFT_A;
logic fft_b_enable = fft_state == S_FFT_B;

integer i;
localparam IDXSZ = 8;
// logic [ IDXSZ-1 :0]  fft_indices_x[0:(1<<HSZ)-1];
// logic [ IDXSZ-1 :0]  fft_indices_y[0:(1<<HSZ)-1];
logic [ IDXSZ-1 :0]  fft_index_wx;
logic [ IDXSZ-1 :0]  fft_index_wy;
logic [ HSZ-1   :0]  fft_index_raddr1;
logic [ HSZ-1   :0]  fft_index_raddr2;
logic [ 32-1    :0]  fft_index_rdata;
logic [ HSZ-1   :0]  fft_indices_pos;

localparam ISZ = 6-1;
logic [ ISZ  :0]    fft_index_valid;


always @(posedge adc_clk_i)
if (fft_index_flush) begin
   fft_hist_step <= 0;
   fft_hist_index[0] <= 0;
   fft_indices_pos <= 1;
   fft_index_valid <= 0;
end else begin
    if (asg3_step_i == 0 && asg2_step_i == 0) begin
        fft_indices_pos <= 0;
        fft_hist_step <= 0;
        for (int i=0; i<=IDX_PIPELINE; i=i+1)
            fft_hist_index[0] <= 0;
    end else begin
        fft_hist_index[0] <= asg3_step_i * fft_hist_step + asg2_step_i;
        for (int i=0; i<IDX_PIPELINE; i=i+1)
            fft_hist_index[i+1] = fft_hist_index[i];
        if (fft_hist_step < asg2_step_i + 1)
            fft_hist_step = asg2_step_i + 1; 
    end

    fft_index_valid = {fft_index_valid[ISZ-1: 0], fft_trig_i && fft_done[0]};
    fft_index_wx <= asg2_step_i;
    fft_index_wy <= asg3_step_i;
    // if (fft_index_valid[ISZ]) begin
    //     fft_indices_x[fft_indices_pos] <= fft_index_wx;
    //     fft_indices_y[fft_indices_pos] <= fft_index_wy;
    //     fft_indices_pos <= fft_indices_pos + 1;
    // end
end

// always @(posedge adc_clk_i) begin
//    fft_index_raddr1 <= sys_addr[HSZ-1+2:2] ;
//    fft_index_raddr2  <= fft_index_raddr1;
//    fft_index_rdata <= {{16-IDXSZ{1'b0}}, fft_indices_x[fft_index_raddr1], {16-IDXSZ{1'b0}}, fft_indices_y[fft_index_raddr1]};
// end


fft_proc #(.DSZ(DSZ), .FSZ(FSZ), .RSZ(RSZ), .HSZ(HSZ)) fft_a (
   .clk_i (adc_clk_i),
   .rstn_i (fft_rstn_i),
   .data_i (adc_a_dat),
   .enable_i (fft_a_enable),
   .dvalid_i (fft_dvalid),
   .trig_i (fft_trig_i),
   .set_dly (set_dly),

   .fft_threshold_k (fft_threshold_k),
   .fft_peak_start (fft_peak_start),
   .fft_peak_minimum (fft_peak_minimum),

   .fft_raddr_i (fft_raddr),
   .fft_rdata_o (fft_rdata_a),
   .fft_wp_last (fft_wp_last[0]),

   .fft_index_flush_i (fft_index_flush),
   .fft_hist_index_i (fft_hist_index[IDX_PIPELINE]),

   .fft_hist_raddr_i (fft_hist_raddr),
   .fft_hist_rdata_o (fft_hist_rdata_a[FSZ-1:0]),

   .status_o (fft_status[0]),
   .fft_done (fft_done[0]),
   .fft_peak_ready (fft_peak_ready_a),
   .fft_count (fft_count_a),
   .fft_sum (fft_sum_a),
   .fft_peak_state (fft_peak_state_a),
   .fft_peak_index (fft_peak_index_a[FSZ-1:0]),
   .fft_peak_value (fft_peak_a),
   .fft_frame_cnt (fft_frame_cnt[15:0]),
   .fft_we_cnt (fft_we_cnt[0]),
   .fft_skip_cnt (fft_skip_cnt[15:0])
);

fft_proc #(.DSZ(DSZ), .FSZ(FSZ), .RSZ(RSZ), .HSZ(HSZ)) fft_b (
   .clk_i (adc_clk_i),
   .rstn_i (fft_rstn_i),
   .data_i (adc_a_dat),
   .enable_i (fft_b_enable),
   .dvalid_i (fft_dvalid),
   .trig_i (fft_trig_i),
   .set_dly (set_dly),

   .fft_threshold_k (fft_threshold_k),
   .fft_peak_start (fft_peak_start),
   .fft_peak_minimum (fft_peak_minimum),

   .fft_raddr_i (fft_raddr),
   .fft_rdata_o (fft_rdata_b),
   .fft_wp_last (fft_wp_last[1]),

   .fft_index_flush_i (fft_index_flush),
   .fft_hist_index_i (fft_hist_index[IDX_PIPELINE]),

   .fft_hist_raddr_i (fft_hist_raddr),
   .fft_hist_rdata_o (fft_hist_rdata_b[FSZ-1:0]),

   .status_o (fft_status[1]),
   .fft_done (fft_done[1]),
   .fft_peak_ready (fft_peak_ready_b),
   .fft_count (fft_count_b),
   .fft_sum (fft_sum_b),
   .fft_peak_state (fft_peak_state_b),
   .fft_peak_index (fft_peak_index_b[FSZ-1:0]),
   .fft_peak_value (fft_peak_b),
   .fft_frame_cnt (fft_frame_cnt[31:16]),
   .fft_we_cnt (fft_we_cnt[1]),
   .fft_skip_cnt (fft_skip_cnt[31:16])
);

always @(posedge adc_clk_i)
if (adc_rstn_i == 1'b0) begin
    fft_enable  <= 1'b0 ;
    fft_threshold_k <= 4;
    fft_peak_start <= 0;
    fft_peak_minimum <= 1;
    fft_wait1_cnt <= 100;
    fft_wait2_cnt <= 200;
    fft_acq1_cnt <= 2**(FSZ-1) - 200;
    fft_acq2_cnt <= 2**(FSZ-1) - 200;
    fft_trig_sync <= 1;
end else if (sys_wen) begin
    if (sys_addr[19:0]==20'h0)  fft_enable <= sys_wdata[5];
    if (sys_addr[19:0]==20'h0)  fft_trig_sync <= sys_wdata[6];
    if (sys_addr[19:0]==20'h38) fft_peak_start <= sys_wdata[FSZ-1:0];
    if (sys_addr[19:0]==20'h3C) fft_threshold_k <= sys_wdata[16-1:0];
    if (sys_addr[19:0]==20'h40) fft_peak_minimum <= sys_wdata[DSZ-1:0];
    if (sys_addr[19:0]==20'h58) fft_wait1_cnt <= sys_wdata[FSZ-1:0];
    if (sys_addr[19:0]==20'h5C) fft_wait2_cnt <= sys_wdata[FSZ-1:0];
    if (sys_addr[19:0]==20'h60) fft_acq1_cnt <= sys_wdata[FSZ-1:0];
    if (sys_addr[19:0]==20'h64) fft_acq2_cnt <= sys_wdata[FSZ-1:0];
end

always @(posedge adc_clk_i)
if (fft_rstn_i == 0) begin
    fft_state <= S_IDLE;
    fft_peak_ready <= 0;
end else begin

    if (fft_peak_ready_a == 2'b01)
        fft_peak_ready[0] <= 1;
    if (fft_peak_ready_b == 2'b01)
        fft_peak_ready[1] <= 1;

    case (fft_state)
    S_IDLE: 
        if (fft_trig_i && &fft_done) begin
            fft_state_cnt <= 0;
            if (set_dly <= 2**FSZ)
                fft_state <= S_WAIT1;
            else
                fft_state <= S_DELAY;
        end
    S_DELAY:
        if (fft_we_cnt[0] <= 2**FSZ)
            fft_state <= S_WAIT1;
    S_WAIT1:
        if (fft_state_cnt >= fft_wait1_cnt) begin
            fft_state_cnt <= 0;
            fft_state <= S_FFT_A;
        end else if (fft_dvalid)
            fft_state_cnt <= fft_state_cnt + 1;
    S_FFT_A:
        if (fft_state_cnt >= fft_acq1_cnt) begin
            fft_state_cnt <= 0;
            fft_state <= S_WAIT2;
        end else if (fft_dvalid)
            fft_state_cnt <= fft_state_cnt + 1;
    S_WAIT2:
        if (fft_state_cnt >= fft_wait2_cnt) begin
            fft_state_cnt <= 0;
            fft_state <= S_FFT_B;
        end else if (fft_dvalid)
            fft_state_cnt <= fft_state_cnt + 1;
    S_FFT_B:
        if (fft_state_cnt >= fft_acq2_cnt) begin
            fft_state_cnt <= 0;
            fft_state <= S_IDLE;
        end else if (fft_dvalid)
            fft_state_cnt <= fft_state_cnt + 1;
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

logic             fft_trig         ;
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
reg  [ 14-1: 0] set_a_tresh  ;
reg  [ 14-1: 0] set_a_treshp ;
reg  [ 14-1: 0] set_a_treshm ;
//reg  [ 14-1: 0] set_b_tresh  ;
//reg  [ 14-1: 0] set_b_treshp ;
//reg  [ 14-1: 0] set_b_treshm ;
reg  [ 14-1: 0] set_a_hyst   ;
//reg  [ 14-1: 0] set_b_hyst   ;

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

//---------------------------------------------------------------------------------
//  System bus connection

always @(posedge adc_clk_i)
if (adc_rstn_i == 1'b0) begin
   adc_we_keep   <=   1'b0      ;
   set_a_tresh   <=  14'd0000   ;
   //set_b_tresh   <=  14'd0000   ;
   set_dly       <=  2**(RSZ-1);
   set_dec       <=  17'h2000; // corresponds to 1s duration, formerly at minimum: 17'd1
   set_a_hyst    <=  14'd20     ;
   //set_b_hyst    <=  14'd20     ;
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

end else begin
   if (sys_wen) begin
      if (sys_addr[19:0]==20'h00)   adc_we_keep   <= sys_wdata[     3] ;

      if (sys_addr[19:0]==20'h08)   set_a_tresh   <= sys_wdata[14-1:0] ;
      //if (sys_addr[19:0]==20'h0C)   set_b_tresh   <= sys_wdata[14-1:0] ;
      if (sys_addr[19:0]==20'h10)   set_dly       <= sys_wdata[32-1:0] ;
      if (sys_addr[19:0]==20'h14)   set_dec       <= sys_wdata[17-1:0] ;
      if (sys_addr[19:0]==20'h20)   set_a_hyst    <= sys_wdata[14-1:0] ;
      //if (sys_addr[19:0]==20'h24)   set_b_hyst    <= sys_wdata[14-1:0] ;
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
   end
end

wire sys_en;
assign sys_en = sys_wen | sys_ren;

assign _scope_done = (!fft_enable && !adc_we) || (fft_enable && &fft_done && (!fft_trig_sync || &fft_peak_ready));
logic [1: 0] scope_done;
assign scope_done_o = scope_done == 2'b01;

always @(posedge adc_clk_i)
if (adc_rstn_i == 1'b0) begin
   sys_err <= 1'b0 ;
   sys_ack <= 1'b0 ;
   scope_done <= 0;

end else begin
   sys_err <= 1'b0 ;

   scope_done <= {scope_done[0], _scope_done};

   casez (sys_addr[19:0])
     20'h00000 : begin sys_ack <= sys_en;          sys_rdata <= {  {8-6{1'b0}}
                                                                 , fft_status[1]
                                                                 , {8-6{1'b0}}
                                                                 , fft_status[0]

                                                                 , {16-9{1'b0}}
                                                                 , fft_done[2-1:0]
                                                                 , fft_trig_sync
                                                                 , fft_enable
                                                                 , 1'b0
                                                                 , adc_we_keep               // do not disarm on 
                                                                 , adc_dly_do                // trigger status
                                                                 , 1'b0                      // reset
                                                                 , adc_we | (fft_enable & (~&fft_done | (fft_trig_sync & ~&fft_peak_ready))) }
                                                                 ; end // arm

     20'h00004 : begin sys_ack <= sys_en;          sys_rdata <= {{32- 4{1'b0}}, set_trig_src}       ; end 

     20'h00008 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_a_tresh}        ; end
     //20'h0000C : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_b_tresh}        ; end
     20'h00010 : begin sys_ack <= sys_en;          sys_rdata <= {               set_dly}            ; end
     20'h00014 : begin sys_ack <= sys_en;          sys_rdata <= {{32-17{1'b0}}, set_dec}            ; end

     20'h00018 : begin sys_ack <= sys_en;          sys_rdata <= {{32-RSZ{1'b0}}, adc_wp_cur}        ; end
     20'h0001C : begin sys_ack <= sys_en;          sys_rdata <= {{32-RSZ{1'b0}}, adc_wp_trig}       ; end

     20'h00020 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_a_hyst}         ; end
     //20'h00024 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_b_hyst}         ; end

     20'h00028 : begin sys_ack <= sys_en;          sys_rdata <= {{32- 1{1'b0}}, set_avg_en}         ; end

     20'h0002C : begin sys_ack <= sys_en;          sys_rdata <=                 adc_we_cnt          ; end

     20'h00030 : begin sys_ack <= sys_en;          sys_rdata <= fft_frame_cnt                      ; end
     20'h00038 : begin sys_ack <= sys_en;          sys_rdata <= fft_peak_start                      ; end
     20'h0003C : begin sys_ack <= sys_en;          sys_rdata <= fft_threshold_k                     ; end
     20'h00040 : begin sys_ack <= sys_en;          sys_rdata <= fft_peak_minimum                    ; end
     20'h00044 : begin sys_ack <= sys_en;          sys_rdata <= {fft_peak_index_b, fft_peak_index_a}; end
     20'h00048 : begin sys_ack <= sys_en;          sys_rdata <= fft_peak_a                          ; end
     20'h0004C : begin sys_ack <= sys_en;          sys_rdata <= fft_peak_b                          ; end
     20'h00050 : begin sys_ack <= sys_en;          sys_rdata <= fft_sum_a                           ; end
     20'h00054 : begin sys_ack <= sys_en;          sys_rdata <= fft_count_a                         ; end
     20'h00058 : begin sys_ack <= sys_en;          sys_rdata <= fft_wait1_cnt                       ; end
     20'h0005C : begin sys_ack <= sys_en;          sys_rdata <= fft_wait2_cnt                       ; end
     20'h00060 : begin sys_ack <= sys_en;          sys_rdata <= fft_acq1_cnt                        ; end
     20'h00064 : begin sys_ack <= sys_en;          sys_rdata <= fft_acq2_cnt                        ; end
     20'h00068 : begin sys_ack <= sys_en;          sys_rdata <= {{16-FSZ{1'b0}}, fft_wp_last[1], {16-FSZ{1'b0}}, fft_wp_last[0]}; end
     20'h0006C : begin sys_ack <= sys_en;          sys_rdata <= fft_state                           ; end
     20'h00070 : begin sys_ack <= sys_en;          sys_rdata <= fft_we_cnt[0]                       ; end
     20'h00074 : begin sys_ack <= sys_en;          sys_rdata <= fft_skip_cnt                        ; end
     20'h00078 : begin sys_ack <= sys_en;          sys_rdata <= fft_peak_state_a                    ; end

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
    
     20'h00154 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, adc_a_i }           ; end
     20'h00158 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, adc_b_i }           ; end
	 
	 20'h0015c : begin sys_ack <= sys_en;          sys_rdata <= ctr_value[32-1:0]     		        ; end
	 20'h00160 : begin sys_ack <= sys_en;          sys_rdata <= ctr_value[64-1:32]			        ; end
	 
	 20'h00164 : begin sys_ack <= sys_en;          sys_rdata <= timestamp_trigger[32-1:0]           ; end
	 20'h00168 : begin sys_ack <= sys_en;          sys_rdata <= timestamp_trigger[64-1:32]	        ; end
     
     20'h0016c : begin sys_ack <= sys_en;          sys_rdata <= {{32-1{1'b0}}, pretrig_ok}          ; end

     20'h1???? : begin sys_ack <= adc_rd_dv;       sys_rdata <= {16'h0, 2'h0,adc_a_rd}              ; end
     20'h2???? : begin sys_ack <= adc_rd_dv;       sys_rdata <= {16'h0, 2'h0,adc_b_rd}              ; end

     20'h3???? : begin sys_ack <= adc_rd_dv;       sys_rdata <= sys_addr[2] ? fft_rdata_b : fft_rdata_a; end

     20'h4???? : begin sys_ack <= adc_rd_dv;       sys_rdata <= {fft_hist_rdata_b, fft_hist_rdata_a}; end

     20'h5???? : begin sys_ack <= adc_rd_dv;       sys_rdata <= fft_index_rdata                     ; end

       default : begin sys_ack <= sys_en;          sys_rdata <=  32'h0                              ; end
   endcase
end

endmodule
