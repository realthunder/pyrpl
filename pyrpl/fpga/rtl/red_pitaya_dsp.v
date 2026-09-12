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
/***********************************************************
DSP Module

This module hosts the different submodules used for digital signal processing.

1)
The first half of this file manages the connection between different submodules by
implementing a bus between them: 

connecting the output_signal of submodule i to the input_signal of submodule j is 
done by setting the register 
input_select[j] <= i;
 
Similarly, a second, possibly different output is allowed for each module: output_direct.
This output is added to the analog output 1 and/or 2 depending on the value
of the register output_select: setting the first bit enables output1, the 2nd bit enables output 2.
Example:
output_select[i] = OUT2;

By default, all routing is done as in the original redpitaya. 

2) 
The second half of this file defines the different submodules. For custom submodules, 
a good point to start is red_pitaya_pid_block.v. 

Submodule i is assigned the address space
0x40300000 + i*0x10000 + (0x0000 to 0xFFFF), that is 2**16 bytes.

Addresses 0x403z00zz where z is an arbitrary hex character are reserved to manage 
the input/output routing of the submodule and are not forwarded, and therefore 
should not be used.  
*************************************************************/


module red_pitaya_dsp #(
    parameter RSZ = 14,  // RAM size 2^RSZ
	parameter MODULES = 8
)
(
   // signals
   input                 clk_i           ,  //!< processing clock
   input                 rstn_i          ,  //!< processing reset - active low
   input      [ 14-1: 0] dat_a_i         ,  //!< input data CHA
   input      [ 14-1: 0] dat_b_i         ,  //!< input data CHB
   output     [ 14-1: 0] dat_a_o         ,  //!< output data CHA
   output     [ 14-1: 0] dat_b_o         ,  //!< output data CHB

   output     [ 14-1: 0] scope1_o,
   output     [ 14-1: 0] scope2_o,

   input      [ 14-1: 0] asg1_i,
   input      [ 14-1: 0] asg2_i,
   input      [ 14-1: 0] asg3_i,
   input      [ 14-1: 0] asg4_i,
   input      [ 14-1: 0] asg1phase_i,

   input      [RSZ-1: 0] asg1_step_i,
   input      [RSZ-1: 0] asg2_step_i,
   input      [RSZ-1: 0] asg3_step_i,
   input      [RSZ-1: 0] asg4_step_i,

   input      [ 12-1: 0] xadc1_i,
   input      [ 12-1: 0] xadc2_i,
   input      [ 12-1: 0] xadc3_i,
   input      [ 12-1: 0] xadc4_i,

   // scope fft acq windows {down, up} for PID gating
   input      [  2-1: 0] fft_window_i ,

   // scanner outputs
   output     [ 14-1: 0] scan_x_o     ,  // scanner x value
   output     [ 14-1: 0] scan_y_o     ,  // scanner y value
   output     [RSZ-1: 0] scan_x_step_o,  // scanner x step
   output     [RSZ-1: 0] scan_y_step_o,  // scanner y step

   // pwm outputs
   output     [ 14-1: 0] pwm0,
   output     [ 14-1: 0] pwm1,
   output     [ 14-1: 0] pwm2,
   output     [ 14-1: 0] pwm3,

   // trigger outputs for the scope
   output                trig_o,   // output from trigger dsp module

   // system bus
   input      [ 32-1: 0] sys_addr        ,  //!< bus address
   input      [ 32-1: 0] sys_wdata       ,  //!< bus write data
   input      [  4-1: 0] sys_sel         ,  //!< bus write byte select
   input                 sys_wen         ,  //!< bus write enable
   input                 sys_ren         ,  //!< bus read enable
   output reg [ 32-1: 0] sys_rdata   ,  //!< bus read data
   output reg            sys_err         ,  //!< bus error indicator
   output reg            sys_ack            //!< bus acknowledge signal
);

localparam EXTRAMODULES = 4; //need two extra control registers for scope/asg
localparam EXTRAINPUTS = 9; //four extra input signals for dac(2)/adc(2) + iq2_2 + xadc(4, slow adc)
localparam EXTRAOUTPUTS = 2; //four extra output signals for pwm channels
localparam LOG_MODULES = 5;// ceil(log2(EXTRAINPUTS+EXTRMODULES+MODULES))

//Module numbers
localparam PID0  = 'd0; //formerly PID11
localparam PID1  = 'd1; //formerly PID12: input2->output1
localparam PID2  = 'd2; //formerly PID21: input1->output2
localparam PID3  = 'd3; //formerly PID22
localparam TRIG  = 'd3; //formerly PID3
localparam IIR   = 'd4; //IIR filter to connect in series to PID module
localparam IQ0   = 'd5; //for PDH signal generation
localparam IQ1   = 'd6; //for NA functionality
localparam IQ2   = 'd7; //for PFD error signal
//localparam CUSTOM1 = 'd8; //available slots
localparam NONE = 2**LOG_MODULES-1; //code for no module; only used to switch off PWM outputs

//EXTRAMODULES numbers
localparam ASG1   = MODULES; //scope and asg can have the same number
localparam ASG2   = MODULES+1; //because one only has outputs, the other only inputs
localparam SCOPE1 = MODULES;
localparam SCOPE2 = MODULES+1;
localparam ASG3  = MODULES+2;
localparam ASG4  = MODULES+3;
localparam PWM0  = MODULES+2;
localparam PWM1  = MODULES+3;
//EXTRAINPUTS numbers
localparam ADC1  = MODULES+4;
localparam ADC2  = MODULES+5;
localparam DAC1  = MODULES+6;
localparam DAC2  = MODULES+7;
localparam IQ2_2 = MODULES+8;
localparam XADC1 = MODULES+9;
localparam XADC2 = MODULES+10;
localparam XADC3 = MODULES+11;
localparam XADC4 = MODULES+12;
//EXTRAOUTPUTS numbers
localparam PWM2  = MODULES+4;
localparam PWM3  = MODULES+5;

//output states
localparam BOTH = 2'b11;
localparam OUT1 = 2'b01;
localparam OUT2 = 2'b10;
localparam OFF  = 2'b00;


// the selected input signal of each module: modules and extramodules have inputs
// extraoutputs are treated like extramodules that do not provide their own output_signal
wire [14-1:0] input_signal [MODULES+EXTRAMODULES+EXTRAOUTPUTS-1:0];
// the selected input signal NUMBER of each module
reg [LOG_MODULES-1:0] input_select [MODULES+EXTRAMODULES+EXTRAOUTPUTS-1:0];

// the output of each module for internal routing, including 'virtual outputs' for the EXTRAINPUTS
wire [14-1:0] output_signal [MODULES+EXTRAMODULES+EXTRAINPUTS-1+1:0];

// the output of each module that is added to the chosen DAC
wire [14-1:0] output_direct [MODULES+EXTRAMODULES-1:0];
// the channel that the module's output_direct is added to (bit0: DAC1, bit 1: DAC2) 
reg [2-1:0] output_select [MODULES+EXTRAMODULES-1:0]; 

// syncronization register to trigger simultaneous action of different dsp modules
reg [MODULES-1:0] sync;

// bus read data of individual modules (only needed for 'real' modules)
wire [ 32-1: 0] module_rdata [MODULES-1:0];  
wire            module_ack   [MODULES-1:0];

wire [RSZ-1: 0] asg_i[0:3];
assign asg_i[0] = asg1_i;
assign asg_i[1] = asg2_i;
assign asg_i[2] = asg3_i;
assign asg_i[3] = asg4_i;

wire [RSZ-1: 0] asg_step_i[0:3];
assign asg_step_i[0] = asg1_step_i;
assign asg_step_i[1] = asg2_step_i;
assign asg_step_i[2] = asg3_step_i;
assign asg_step_i[3] = asg4_step_i;

reg [2-1:0] scan_select [1:0];
assign scan_x_o = asg_i[scan_select[0]];
assign scan_y_o = asg_i[scan_select[1]];
// Scan step indices to the scope are REGISTERED (one clk_i cycle late relative
// to the asg's step_o): the mux output fanned straight into the scope's
// fft_hist_step adder / DSP CE (asg step_o -> this mux -> i_scope, -0.090 ns on
// the n11 die). Alignment is unaffected: the scope samples x/y_step_i one cycle
// after a trigger accept (fft_index_valid[0]), while the scope-triggered asg
// one-shot advances step_o no earlier than ~5 cycles after that accept
// (scope_sig register -> asg 2-FF scope_trig -> ch trig_in -> dac_trig), so
// both the delayed and the undelayed view sample the pre-kick position.
reg [RSZ-1: 0] scan_x_step_r, scan_y_step_r;
always @(posedge clk_i) begin
    scan_x_step_r <= asg_step_i[scan_select[0]];
    scan_y_step_r <= asg_step_i[scan_select[1]];
end
assign scan_x_step_o = scan_x_step_r;
assign scan_y_step_o = scan_y_step_r;

//connect scope
assign scope1_o = input_signal[SCOPE1];
assign scope2_o = input_signal[SCOPE2];

//connect asg output
assign output_signal[ASG1] = asg1_i;
assign output_signal[ASG2] = asg2_i;
assign output_signal[ASG3] = asg3_i;
assign output_signal[ASG4] = asg4_i;

assign output_direct[ASG1] = asg1_i;
assign output_direct[ASG2] = asg2_i;
assign output_direct[ASG3] = asg3_i;
assign output_direct[ASG4] = asg4_i;

assign output_signal[XADC1] = xadc1_i;
assign output_signal[XADC2] = xadc2_i;
assign output_signal[XADC3] = xadc3_i;
assign output_signal[XADC4] = xadc4_i;

//connect dac/adc to internal signals
assign output_signal[ADC1] = dat_a_i;
assign output_signal[ADC2] = dat_b_i;
assign output_signal[DAC1] = dat_a_o;
assign output_signal[DAC2] = dat_b_o;

assign pwm0 = (input_select[PWM0] == NONE) ? 14'h0 : output_signal[input_select[PWM0]];
assign pwm1 = (input_select[PWM1] == NONE) ? 14'h0 : output_signal[input_select[PWM1]];
assign pwm2 = (input_select[PWM2] == NONE) ? 14'h0 : output_signal[input_select[PWM2]];
assign pwm3 = (input_select[PWM3] == NONE) ? 14'h0 : output_signal[input_select[PWM3]];

reg  signed [   14+LOG_MODULES-1: 0] sum1; 
reg  signed [   14+LOG_MODULES-1: 0] sum2; 

wire dac_a_saturated; //high when dac_a is saturated
wire dac_b_saturated; //high when dac_b is saturated

integer i;
genvar j;

//select inputs
generate for (j = 0; j < MODULES+EXTRAMODULES; j = j+1)
   assign input_signal[j] = (input_select[j]==NONE) ? 14'b0 : output_signal[input_select[j]];
endgenerate

//sum together the direct outputs
//
//use a tree-like structure where at most 2 numbers are added per cycle (4 should be possible as well but lets go slow)
//CHANNELS is the number of numbers to add, CHANNELS-1 the number of tree nodes (represented in presum)
//presum... 0-7: sum of pairs of input signals (for 16 channels)
//presum... 8 = 0+1, 9=2+3, 10=4+5 etc... => end result in presum[CHANNELS-1-1] 
//right now we have an extra delay to go into sum, just to make sure that the slack is maximally positive

localparam CHANNELS = 2**LOG_MODULES; //not the same as MODULES+EXTRAMODULES
reg  signed [   14+LOG_MODULES-1: 0] presum1 [CHANNELS-1-1:0]; 
reg  signed [   14+LOG_MODULES-1: 0] presum2 [CHANNELS-1-1:0]; 

always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
     for (i=0;i<CHANNELS-1;i=i+1) begin
        presum1[i] <= {14+LOG_MODULES{1'b0}};
        presum2[i] <= {14+LOG_MODULES{1'b0}};
     end
     sum1 <= {14+LOG_MODULES{1'b0}};
     sum2 <= {14+LOG_MODULES{1'b0}};
   end
   else begin
     //first sum pairs if they are set to be summed
     for (i=0;i<(MODULES+EXTRAMODULES)/2;i=i+1) begin
        presum1[i] <= ({14+LOG_MODULES{(|(output_select[2*i]&OUT1))}} & {{LOG_MODULES{output_direct[2*i][14-1]}},output_direct[2*i]}) + ({14+LOG_MODULES{(|(output_select[2*i+1]&OUT1))}} & {{LOG_MODULES{output_direct[2*i+1][14-1]}},output_direct[2*i+1]});
        presum2[i] <= ({14+LOG_MODULES{(|(output_select[2*i]&OUT2))}} & {{LOG_MODULES{output_direct[2*i][14-1]}},output_direct[2*i]}) + ({14+LOG_MODULES{(|(output_select[2*i+1]&OUT2))}} & {{LOG_MODULES{output_direct[2*i+1][14-1]}},output_direct[2*i+1]});
     end
     //then sum the sums of pairs to go up the tree
     for (i=0;i<CHANNELS/2-1;i=i+1) begin
        presum1[CHANNELS/2+i] <= presum1[2*i]+presum1[2*i+1];
        presum2[CHANNELS/2+i] <= presum2[2*i]+presum2[2*i+1];
     end
     //finally add some (probably unnecessary) delay
     sum1 <= presum1[CHANNELS-1-1];
     sum2 <= presum2[CHANNELS-1-1];
   end
end

//saturation of outputs
red_pitaya_saturate #(
    .BITS_IN (14+LOG_MODULES), 
    .SHIFT(0), 
    .BITS_OUT(14)
    ) dac_saturate [1:0] (
   .input_i({sum2,sum1}),
   .output_o({dat_b_o,dat_a_o}),
   .overflow ({dat_b_saturated,dac_a_saturated})
   );   

//  System bus connection
always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      //default settings for backwards compatibility with original code
      input_select [PID0] <= ADC1;
      output_select[PID0] <= OFF;
      
      input_select [PID1] <= ADC1;
      output_select[PID1] <= OFF;

      input_select [PID2] <= ADC1;
      output_select[PID2] <= OFF;

      input_select [PID3] <= ADC1;
      output_select[PID3] <= OFF;

      input_select [IIR] <= ADC1;
      output_select[IIR] <= OFF;

      input_select [IQ0] <= ADC1;
      output_select[IQ0] <= OFF;
      
      input_select [IQ1] <= ADC1;
      output_select[IQ1] <= OFF;

      input_select [IQ2] <= ADC1;
      output_select[IQ2] <= OFF;

      input_select [SCOPE1] <= ADC1;
      input_select [SCOPE2] <= ADC2;
      output_select[ASG1] <= OFF;
      output_select[ASG2] <= OFF;
      output_select[ASG3] <= OFF;
      output_select[ASG4] <= OFF;
      
      input_select [PWM0] <= NONE;
      input_select [PWM1] <= NONE;
      input_select [PWM2] <= NONE;
      input_select [PWM3] <= NONE;

      scan_select[0] <= 1;
      scan_select[1] <= 2;
      
      sync <= {MODULES{1'b1}} ;  // all modules on by default
   end
   else begin
      if (sys_wen) begin
         if (sys_addr[16-1:0]==16'h00)     input_select[sys_addr[16+LOG_MODULES-1:16]] <= sys_wdata[ LOG_MODULES-1:0];
         if (sys_addr[16-1:0]==16'h04)    output_select[sys_addr[16+LOG_MODULES-1:16]] <= sys_wdata[ 2-1:0];
         if (sys_addr[16-1:0]==16'h0C)                                            sync <= sys_wdata[MODULES-1:0];
         if (sys_addr[16-1:0]==16'h14) begin
             scan_select[0] <= sys_wdata[1:0];
             scan_select[1] <= sys_wdata[3:2];
         end
      end
   end
end

wire sys_en;
assign sys_en = sys_wen | sys_ren;
always @(posedge clk_i)
if (rstn_i == 1'b0) begin
   sys_err <= 1'b0 ;
   sys_ack <= 1'b0 ;
end else begin
   sys_err <= 1'b0 ;
   casez (sys_addr[16-1:0])
      20'h00 : begin sys_ack <= sys_en;          sys_rdata <= {{32- LOG_MODULES{1'b0}},input_select[sys_addr[16+LOG_MODULES-1:16]]}; end 
	  20'h04 : begin sys_ack <= sys_en;          sys_rdata <= {{32- 2{1'b0}},output_select[sys_addr[16+LOG_MODULES-1:16]]}; end
	  20'h08 : begin sys_ack <= sys_en;          sys_rdata <= {{32- 2{1'b0}},dat_b_saturated,dac_a_saturated}; end
	  20'h0C : begin sys_ack <= sys_en;          sys_rdata <= {{32-MODULES{1'b0}},sync} ; end
      20'h10 : begin sys_ack <= sys_en;          sys_rdata <= {{32- 14{1'b0}},output_signal[sys_addr[16+LOG_MODULES-1:16]]} ; end
      20'h14 : begin sys_ack <= sys_en;          sys_rdata <= {{32 - 4{1'b0}}, scan_select[1], scan_select[0]}; end

     default : begin sys_ack <= module_ack[sys_addr[16+LOG_MODULES-1:16]];    sys_rdata <=  module_rdata[sys_addr[16+LOG_MODULES-1:16]]  ; end
   endcase
end


/**********************************************
 MODULE DEFINITIONS
 *********************************************/

//PID

wire [14-1:0] diff_input_signal [3-1:0];
wire [14-1:0] diff_output_signal [3-1:0];
//assign diff_input_signal[0] = input_signal[1]; // difference input of PID0 is PID1
//assign diff_input_signal[1] = input_signal[0]; // difference input of PID1 is PID0
assign diff_input_signal[0] = diff_output_signal[1]; // difference input of PID0 is PID1
assign diff_input_signal[1] = diff_output_signal[0]; // difference input of PID1 is PID0
assign diff_input_signal[2] = {14{1'b0}};      // difference input of PID2 is zero

// PID0 is always present unless DSP_LEAN strips it (lean images: pid0 only
// served the abandoned EO-PLL leg, and its module-sum feedback endpoint
// pid_out_reg is one of the recurring pll_adc_clk walls). PID1/PID2 are
// restored by defining ENABLE_PID_EXTRA (off by default to relieve DSP/LUT
// congestion). Mirrors the ENABLE_IIR guard.
`ifdef ENABLE_PID_EXTRA
localparam PID_LAST = PID2;
`else
localparam PID_LAST = PID0;
`endif
`ifdef DSP_LEAN
localparam PID_FIRST = PID1;
`else
localparam PID_FIRST = PID0;
`endif
// PID_FILTERSTAGES trims the pid input-filter chain (4 cascaded first-order
// stages by default, ~530 LUT; 2 halves that). pyrpl reads the stage count
// back from the module (reg 0x220), so the host adapts automatically.
`ifndef PID_FILTERSTAGES
`define PID_FILTERSTAGES 4
`endif
generate for (j = PID_FIRST; j <= PID_LAST; j = j+1) begin
   red_pitaya_pid_block #(.FILTERSTAGES(`PID_FILTERSTAGES)) i_pid (
     // data
     .clk_i        (  clk_i          ),  // clock
     .rstn_i       (  rstn_i         ),  // reset - active low
     .sync_i       (  sync[j]        ),  // syncronization of different dsp modules
     .window_i     (  fft_window_i   ),  // fft acq windows for optional gating
     .dat_i        (  input_signal [j] ),  // input data
     .dat_o        (  output_direct[j]),  // output data
	 .diff_dat_i   (  diff_input_signal[j] ),  // input data for differential mode
	 .diff_dat_o   (  diff_output_signal[j] ),  // output data for differential mode

	 //communincation with PS
	 .addr ( sys_addr[16-1:0] ),
	 .wen  ( sys_wen & (sys_addr[21-1:16]==j) ),
	 .ren  ( sys_ren & (sys_addr[21-1:16]==j) ),
	 .ack  ( module_ack[j] ),
	 .rdata (module_rdata[j]),
     .wdata (sys_wdata)
   );
   assign output_signal[j] = output_direct[j];
end
endgenerate
`ifndef ENABLE_PID_EXTRA
// PID1, PID2 disabled — tie off their outputs and bus (see ENABLE_PID_EXTRA).
assign output_direct[PID1]      = 14'h0;
assign output_signal[PID1]      = 14'h0;
assign diff_output_signal[PID1] = 14'h0;
assign module_ack[PID1]         = 1'b0;
assign module_rdata[PID1]       = 32'h0;
assign output_direct[PID2]      = 14'h0;
assign output_signal[PID2]      = 14'h0;
assign diff_output_signal[PID2] = 14'h0;
assign module_ack[PID2]         = 1'b0;
assign module_rdata[PID2]       = 32'h0;
`endif
`ifdef DSP_LEAN
// PID0 stripped — tie off output, diff link and bus (reads land on the
// axi_slave timeout ack, same as the other disabled modules).
assign output_direct[PID0]      = 14'h0;
assign output_signal[PID0]      = 14'h0;
assign diff_output_signal[PID0] = 14'h0;
assign module_ack[PID0]         = 1'b0;
assign module_rdata[PID0]       = 32'h0;
`endif

wire trig_signal;
//TRIG
generate for (j = TRIG; j < TRIG+1; j = j+1) begin
   red_pitaya_trigger_block i_trigger (
     // data
     .clk_i        (  clk_i          ),  // clock
     .rstn_i       (  rstn_i         ),  // reset - active low
     .dat_i        (  input_signal [j] ),  // input data
     .dat_o        (  output_direct[j]),  // output data
     .signal_o     (  output_signal[j]),  // output signal
     .phase1_i     (  asg1phase_i ),  // phase input
     .trig_o       (  trig_signal ),

	 //communincation with PS
	 .addr ( sys_addr[16-1:0] ),
	 .wen  ( sys_wen & (sys_addr[21-1:16]==j) ),
	 .ren  ( sys_ren & (sys_addr[21-1:16]==j) ),
	 .ack  ( module_ack[j] ),
	 .rdata (module_rdata[j]),
     .wdata (sys_wdata)
   );
end
endgenerate
assign trig_o = trig_signal;

//IIR module
`ifdef ENABLE_IIR
generate for (j = IIR; j < IIR+1; j = j+1) begin
    red_pitaya_iir_block iir (
	     // data
	     .clk_i        (  clk_i          ),  // clock
	     .rstn_i       (  rstn_i         ),  // reset - active low
	     .dat_i        (  input_signal [j] ),  // input data
	     .dat_o        (  output_direct[j]),  // output data

		 //communincation with PS
		 .addr ( sys_addr[16-1:0] ),
		 .wen  ( sys_wen & (sys_addr[21-1:16]==j) ),
		 .ren  ( sys_ren & (sys_addr[21-1:16]==j) ),
		 .ack  ( module_ack[j] ),
		 .rdata (module_rdata[j]),
	     .wdata (sys_wdata)
      );
	  assign output_signal[j] = output_direct[j];
end endgenerate
`else
assign output_direct[IIR] = 14'h0;
assign output_signal[IIR] = 14'h0;
assign module_ack  [IIR]  = 1'b0;
assign module_rdata[IIR]  = 32'h0;
`endif


//IQ modules iq0, iq1
// IQ0 is always present unless DSP_LEAN strips it (lean images: iq0 only serves
// the offline tune_fft_window iq-probe — use a full image for that — and its
// inputfilter delta_reg is the other recurring pll_adc_clk feedback endpoint).
// IQ1 is restored by defining ENABLE_IQ1 (off by default).
`ifdef ENABLE_IQ1
localparam IQ_LAST = IQ1;
`else
localparam IQ_LAST = IQ0;
`endif
`ifdef DSP_LEAN
localparam IQ_FIRST = IQ1;
`else
localparam IQ_FIRST = IQ0;
`endif
generate for (j = IQ_FIRST; j <= IQ_LAST; j = j+1) begin
    red_pitaya_iq_block
      iq
      (
	     // data
	     .clk_i        (  clk_i          ),  // clock
	     .rstn_i       (  rstn_i         ),  // reset - active low
         .sync_i       (  sync[j]        ),  // syncronization of different dsp modules
	     .dat_i        (  input_signal [j] ),  // input data
	     .dat_o        (  output_direct[j]),  // output data
		 .signal_o     (  output_signal[j]),  // output signal

         // not using 2nd quadrature for most iq's: multipliers will be
         // synthesized away by Vivado
         //.signal2_o  (  output_signal[j*2]),  // output signal

		 //communincation with PS
		 .addr ( sys_addr[16-1:0] ),
		 .wen  ( sys_wen & (sys_addr[21-1:16]==j) ),
		 .ren  ( sys_ren & (sys_addr[21-1:16]==j) ),
		 .ack  ( module_ack[j] ),
		 .rdata (module_rdata[j]),
	     .wdata (sys_wdata)
      );
end endgenerate
`ifndef ENABLE_IQ1
// IQ1 disabled — tie off outputs and bus.
assign output_direct[IQ1] = 14'h0;
assign output_signal[IQ1] = 14'h0;
assign module_ack[IQ1]    = 1'b0;
assign module_rdata[IQ1]  = 32'h0;
`endif
`ifdef DSP_LEAN
// IQ0 stripped — tie off outputs and bus.
assign output_direct[IQ0] = 14'h0;
assign output_signal[IQ0] = 14'h0;
assign module_ack[IQ0]    = 1'b0;
assign module_rdata[IQ0]  = 32'h0;
`endif

// IQ with two outputs iq2 — restored by defining ENABLE_IQ2 (off by default).
`ifdef ENABLE_IQ2
red_pitaya_iq_block   #( .QUADRATUREFILTERSTAGES(4) )
    iq_2_outputs
    (
        // data
        .clk_i        (  clk_i          ),  // clock
        .rstn_i       (  rstn_i         ),  // reset - active low
        .sync_i       (  sync[IQ2]        ),  // syncronization of different dsp modules
        .dat_i        (  input_signal [IQ2] ),  // input data
        .dat_o        (  output_direct[IQ2]),  // output data
        .signal_o     (  output_signal[IQ2]),  // output signal
        .signal2_o    (  output_signal[IQ2_2]),  // output signal 2

        //communincation with PS
        .addr ( sys_addr[16-1:0] ),
        .wen  ( sys_wen & (sys_addr[21-1:16]==IQ2) ),
        .ren  ( sys_ren & (sys_addr[21-1:16]==IQ2) ),
        .ack  ( module_ack[IQ2] ),
        .rdata (module_rdata[IQ2]),
        .wdata (sys_wdata)
    );
`else
// iq_2_outputs disabled — tie off both outputs and bus.
assign output_direct[IQ2]   = 14'h0;
assign output_signal[IQ2]   = 14'h0;
assign output_signal[IQ2_2] = 14'h0;
assign module_ack[IQ2]      = 1'b0;
assign module_rdata[IQ2]    = 32'h0;
`endif

endmodule
