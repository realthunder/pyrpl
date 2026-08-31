/**
 * $Id: red_pitaya_asg.v 961 2014-01-21 11:40:39Z matej.oblak $
 *
 * @brief Red Pitaya arbitrary signal generator (ASG).
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
 * Arbitrary signal generator takes data stored in buffer and sends them to DAC.
 *
 *
 *                /-----\         /--------\
 *   SW --------> | BUF | ------> | kx + o | ---> DAC CHA
 *                \-----/         \--------/
 *                   ^
 *                   |
 *                /-----\
 *   SW --------> |     |
 *                | FSM | ------> trigger notification
 *   trigger ---> |     |
 *                \-----/
 *                   |
 *                   ˇ
 *                /-----\         /--------\
 *   SW --------> | BUF | ------> | kx + o | ---> DAC CHB
 *                \-----/         \--------/ 
 *
 *
 * Buffers are filed with SW. It also sets finite state machine which take control
 * over read pointer. All registers regarding reading from buffer has additional 
 * 16 bits used as decimal points. In this way we can make better ratio betwen 
 * clock cycle and frequency of output signal. 
 *
 * Finite state machine can be set for one time sequence or continously wrapping.
 * Starting trigger can come from outside, notification trigger used to synchronize
 * with other applications (scope) is also available. Both channels are independant.
 *
 * Output data is scaled with linear transmormation.
 * 
 */

module red_pitaya_asg  #(
  parameter RSZ = 14  // RAM size 2^RSZ
)(
  // DAC
  output     [ 14-1: 0] dac_a_o   ,  // DAC data CHA
  output     [ 14-1: 0] dac_b_o   ,  // DAC data CHB
  output     [ 14-1: 0] dac_c_o   ,  // DAC data CHC
  output     [ 14-1: 0] dac_d_o   ,  // DAC data CHD
  input                 dac_clk_i ,  // DAC clock
  input                 dac_rstn_i,  // DAC reset - active low
  input                 trig_a_i  ,  // starting trigger CHA
  input                 trig_b_i  ,  // starting trigger CHB
  input                 trig_c_i  ,  // starting trigger CHC
  input                 trig_d_i  ,  // starting trigger CHD
  input                 trig_enc_i,  // encoder tick trigger (clean 1-cycle pulse from red_pitaya_enc)
  output     [  4-1: 0] trig_out_o,  // notification trigger
  output     [  4-1: 0] play_active_o, // per-channel playing (dac_do) — asg3 busy gates encoder ticks
 
  input                 trig_scope_i    ,  // trigger from the scope
  input                 scope_trig_i    ,  // scope done signal

  output                sync_rst_o,  // syncrhonized reset signal

  output     [ 14-1: 0] asg1phase_o,

  output     [RSZ-1: 0] step_a_o,
  output     [RSZ-1: 0] step_b_o,
  output     [RSZ-1: 0] step_c_o,
  output     [RSZ-1: 0] step_d_o,

  // System bus
  input      [ 32-1: 0] sys_addr  ,  // bus address
  input      [ 32-1: 0] sys_wdata ,  // bus write data
  input      [  4-1: 0] sys_sel   ,  // bus write byte select
  input                 sys_wen   ,  // bus write enable
  input                 sys_ren   ,  // bus read enable
  output reg [ 32-1: 0] sys_rdata ,  // bus read data
  output reg            sys_err   ,  // bus error indicator
  output reg            sys_ack      // bus acknowledge signal
);

//---------------------------------------------------------------------------------
//
// generating signal from DAC table 

reg   [RSZ+15: 0] set_a_size   , set_b_size   , set_c_size   , set_d_size   ;
reg   [RSZ+15: 0] set_a_step   , set_b_step   , set_c_step   , set_d_step   ;
reg   [RSZ+15: 0] set_a_ofs    , set_b_ofs    , set_c_ofs    , set_d_ofs    ;
reg               set_a_rst    , set_b_rst    , set_c_rst    , set_d_rst    ;
reg               set_a_steping, set_b_steping, set_c_steping, set_d_steping;
reg               set_a_wrap   , set_b_wrap   , set_c_wrap   , set_d_wrap   ;
reg   [  14-1: 0] set_a_amp    , set_b_amp    , set_c_amp    , set_d_amp    ;
reg   [  14-1: 0] set_a_dc     , set_b_dc     , set_c_dc     , set_d_dc     ;
reg               set_a_zero   , set_b_zero   , set_c_zero   , set_d_zero   ;
reg   [  32-1: 0] set_a_ncyc   , set_b_ncyc   , set_c_ncyc   , set_d_ncyc   ;
reg   [  16-1: 0] set_a_rnum   , set_b_rnum   , set_c_rnum   , set_d_rnum   ;
reg   [  32-1: 0] set_a_rdly   , set_b_rdly   , set_c_rdly   , set_d_rdly   ;
reg               set_a_rgate  , set_b_rgate  , set_c_rgate  , set_d_rgate  ;
reg               buf_a_we     , buf_b_we     , buf_c_we     , buf_d_we     ;
reg   [ RSZ-1: 0] buf_a_addr   , buf_b_addr   , buf_c_addr   , buf_d_addr   ;
wire  [  14-1: 0] buf_a_rdata  , buf_b_rdata  , buf_c_rdata  , buf_d_rdata  ;
wire  [ RSZ-1: 0] buf_a_rpnt   , buf_b_rpnt   , buf_c_rpnt   , buf_d_rpnt   ;
wire  [ RSZ-1: 0]_step_a_o     ,_step_b_o     ,_step_c_o     ,_step_d_o     ;
reg   [  32-1: 0] buf_a_rpnt_rd, buf_b_rpnt_rd, buf_c_rpnt_rd, buf_d_rpnt_rd;
reg               trig_a_sw    , trig_b_sw    , trig_c_sw    , trig_d_sw    ;
reg   [   3-1: 0] trig_a_src   , trig_b_src   , trig_c_src   , trig_d_src   ;
wire              trig_a_done  , trig_b_done  , trig_c_done  , trig_d_done  ;
wire              play_a_done  , play_b_done  , play_c_done  , play_d_done  ;
wire              play_a_act   , play_b_act   , play_c_act   , play_d_act   ;
reg               slave_a_trig , slave_b_trig , slave_c_trig , slave_d_trig ;
reg               scope_a_trig , scope_b_trig , scope_c_trig , scope_d_trig ;
wire              trig_a_slave , trig_b_slave , trig_c_slave , trig_d_slave ;
reg               reverse_a_on , reverse_b_on , reverse_c_on , reverse_d_on ;
reg               sync_a_on    , sync_b_on    , sync_c_on    , sync_d_on    ;
reg               rand_a_on    , rand_b_on    , rand_c_on    , rand_d_on    ;
wire  [ RSZ-1: 0] rand_pnt;

assign step_a_o = set_a_steping ? _step_a_o : {RSZ{1'b0}};
assign step_b_o = set_b_steping ? _step_b_o : {RSZ{1'b0}};
assign step_c_o = set_c_steping ? _step_c_o : {RSZ{1'b0}};
assign step_d_o = set_d_steping ? _step_d_o : {RSZ{1'b0}};

wire              _set_a_rst   , _set_b_rst   , _set_c_rst   , _set_d_rst    ;
assign sync_rst_o = (sync_a_on & set_a_rst) | (sync_b_on & set_b_rst) | (sync_c_on & set_c_rst) | (sync_d_on & set_d_rst) ;
assign _set_a_rst = set_a_rst | (sync_a_on & sync_rst_o); 
assign _set_b_rst = set_b_rst | (sync_b_on & sync_rst_o); 
assign _set_c_rst = set_c_rst | (sync_c_on & sync_rst_o); 
assign _set_d_rst = set_d_rst | (sync_d_on & sync_rst_o); 

wire              _set_a_zero   , _set_b_zero   , _set_c_zero   , _set_d_zero    ;
wire              set_zero ;
assign set_zero = (sync_a_on & set_a_zero) | (sync_b_on & set_b_zero) | (sync_c_on & set_c_zero) | (sync_d_on & set_d_zero) ;
// assign set_zero = set_a_zero | set_b_zero | set_c_zero | set_d_zero ;
assign _set_a_zero = set_a_zero | (sync_a_on & set_zero); 
assign _set_b_zero = set_b_zero | (sync_b_on & set_zero); 
assign _set_c_zero = set_c_zero | (sync_c_on & set_zero); 
assign _set_d_zero = set_d_zero | (sync_d_on & set_zero); 

wire              _trig_a_sw   , _trig_b_sw   , _trig_c_sw   , _trig_d_sw    ;
wire              trig_sw ;
assign trig_sw = (sync_a_on & trig_a_sw) | (sync_b_on & trig_b_sw) | (sync_c_on & trig_c_sw) | (sync_d_on & trig_d_sw) ;
// assign trig_sw = trig_a_sw | trig_b_sw | trig_c_sw | trig_d_sw ;
assign _trig_a_sw = trig_a_sw | (sync_a_on & trig_sw); 
assign _trig_b_sw = trig_b_sw | (sync_b_on & trig_sw); 
assign _trig_c_sw = trig_c_sw | (sync_c_on & trig_sw); 
assign _trig_d_sw = trig_d_sw | (sync_d_on & trig_sw); 

//advanced triggers for both channels
reg [64-1:0] at_counts_a;
reg          at_reset_a;
reg          at_invert_a;
reg          at_autorearm_a;
wire         at_trig_a;
// DISABLE_ASG_ADVTRIG strips the four advanced-trigger blocks (~600 LUT on the
// full n11 die). The feature is pyrpl alpha functionality unused by the FMCW
// product; its registers reset to at_reset=1, in which state the block is
// combinationally TRANSPARENT (trig_o = trig_i) — so the plain-wire replacement
// is bit-identical to the untouched default. The at_* registers stay (readback
// intact); only the counters/FSMs go.
`ifdef DISABLE_ASG_ADVTRIG
assign at_trig_a = trig_a_i;
`else
red_pitaya_adv_trigger adv_trig_a (
    .dac_clk_i (dac_clk_i) ,
    .reset_i   (at_reset_a),
    .trig_i    (trig_a_i)  ,
    .trig_o    (at_trig_a) ,
    .invert_i  (at_invert_a),
    .rearm_i   (at_autorearm_a),
    .hysteresis_i (at_counts_a)//stay on for hysteresis_i cycles
    );
`endif

reg [64-1:0] at_counts_b;
reg          at_reset_b;
reg          at_invert_b;
reg          at_autorearm_b;
wire         at_trig_b;
`ifdef DISABLE_ASG_ADVTRIG
assign at_trig_b = trig_b_i;
`else
red_pitaya_adv_trigger adv_trig_b (
    .dac_clk_i (dac_clk_i) ,
    .reset_i   (at_reset_b),
    .trig_i    (trig_b_i)  ,
    .trig_o    (at_trig_b)   ,
    .invert_i  (at_invert_b),
    .rearm_i   (at_autorearm_b),
    .hysteresis_i (at_counts_b)//stay on for hysteresis_i cycles
    );
`endif

reg [64-1:0] at_counts_c;
reg          at_reset_c;
reg          at_invert_c;
reg          at_autorearm_c;
wire         at_trig_c;
`ifdef DISABLE_ASG_ADVTRIG
assign at_trig_c = trig_c_i;
`else
red_pitaya_adv_trigger adv_trig_c (
    .dac_clk_i (dac_clk_i) ,
    .reset_i   (at_reset_c),
    .trig_i    (trig_c_i)  ,
    .trig_o    (at_trig_c)   ,
    .invert_i  (at_invert_c),
    .rearm_i   (at_autorearm_c),
    .hysteresis_i (at_counts_c)//stay on for hysteresis_i cycles
    );
`endif

reg [64-1:0] at_counts_d;
reg          at_reset_d;
reg          at_invert_d;
reg          at_autorearm_d;
wire         at_trig_d;
`ifdef DISABLE_ASG_ADVTRIG
assign at_trig_d = trig_d_i;
`else
red_pitaya_adv_trigger adv_trig_d (
    .dac_clk_i (dac_clk_i) ,
    .reset_i   (at_reset_d),
    .trig_i    (trig_d_i)  ,
    .trig_o    (at_trig_d)   ,
    .invert_i  (at_invert_d),
    .rearm_i   (at_autorearm_d),
    .hysteresis_i (at_counts_d)//stay on for hysteresis_i cycles
    );
`endif


red_pitaya_asg_ch  #(.RSZ (RSZ)) ch [4-1:0] (
  // DAC
  .dac_o           ({dac_d_o          , dac_c_o          , dac_b_o          , dac_a_o          }),  // dac data output
  .dac_clk_i       ({dac_clk_i        , dac_clk_i        , dac_clk_i        , dac_clk_i        }),  // dac clock
  .dac_rstn_i      ({dac_rstn_i       , dac_rstn_i       , dac_rstn_i       , dac_rstn_i       }),  // dac reset - active low
  // trigger
  .trig_sw_i       ({_trig_d_sw       ,_trig_c_sw        ,_trig_b_sw        ,_trig_a_sw        }),  // software trigger
  .trig_ext_i      ({at_trig_d        , at_trig_c        , at_trig_a        , at_trig_b        }),  // advanced trigger as ext trigger - backwards-compatible with original version
  .trig_enc_i      ({trig_enc_i       , trig_enc_i       , trig_enc_i       , trig_enc_i       }),  // encoder tick (same pulse; used only by channels with trig_src=6)
  .trig_src_i      ({trig_d_src       , trig_c_src       , trig_b_src       , trig_a_src       }),  // trigger source selector
  .trig_slave_i    ({trig_d_slave     , trig_c_slave     , trig_b_slave     , trig_a_slave     }),  // slave trigger
  .trig_done_o     ({trig_d_done      , trig_c_done      , trig_b_done      , trig_a_done      }),  // trigger event
  .play_done_o     ({play_d_done      , play_c_done      , play_b_done      , play_a_done      }),  // data play done event
  .play_active_o   ({play_d_act       , play_c_act       , play_b_act       , play_a_act       }),  // playing (dac_do)
  // buffer ctrl
  .buf_we_i        ({buf_d_we         , buf_c_we         , buf_b_we         , buf_a_we         }),  // buffer buffer write
  .buf_addr_i      ({buf_d_addr       , buf_c_addr       , buf_b_addr       , buf_a_addr       }),  // buffer address
  .buf_wdata_i     ({sys_wdata[14-1:0], sys_wdata[14-1:0], sys_wdata[14-1:0], sys_wdata[14-1:0]}),  // buffer write data
  .buf_rdata_o     ({buf_d_rdata      , buf_c_rdata      , buf_b_rdata      , buf_a_rdata      }),  // buffer read data
  .buf_rpnt_o      ({buf_d_rpnt       , buf_c_rpnt       , buf_b_rpnt       , buf_a_rpnt       }),  // buffer current read pointer
  .step_o          ({_step_d_o        ,_step_c_o         ,_step_b_o         ,_step_a_o         }),  // buffer current step
  // configuration
  .set_size_i      ({set_d_size       , set_c_size       , set_b_size       , set_a_size       }),  // set table data size
  .set_step_i      ({set_d_step       , set_c_step       , set_b_step       , set_a_step       }),  // set pointer step
  .set_ofs_i       ({set_d_ofs        , set_c_ofs        , set_b_ofs        , set_a_ofs        }),  // set reset offset
  .set_rst_i       ({_set_d_rst       ,_set_c_rst        ,_set_b_rst        ,_set_a_rst        }),  // set FMS to reset
  .set_wrap_i      ({set_d_wrap       , set_c_wrap       , set_b_wrap       , set_a_wrap       }),  // set wrap pointer
  .set_amp_i       ({set_d_amp        , set_c_amp        , set_b_amp        , set_a_amp        }),  // set amplitude scale
  .set_dc_i        ({set_d_dc         , set_c_dc         , set_b_dc         , set_a_dc         }),  // set output offset
  .set_zero_i      ({_set_d_zero      ,_set_c_zero       ,_set_b_zero       ,_set_a_zero       }),  // set output to zero
  .set_ncyc_i      ({set_d_ncyc       , set_c_ncyc       , set_b_ncyc       , set_a_ncyc       }),  // set number of cycle
  .set_rnum_i      ({set_d_rnum       , set_c_rnum       , set_b_rnum       , set_a_rnum       }),  // set number of repetitions
  .set_rdly_i      ({set_d_rdly       , set_c_rdly       , set_b_rdly       , set_a_rdly       }),  // set delay between repetitions
  .set_rgate_i     ({set_d_rgate      , set_c_rgate      , set_b_rgate      , set_a_rgate      }),  // set external gated repetition
  .reverse_on_i    ({reverse_d_on     , reverse_c_on     , reverse_b_on     , reverse_a_on     }),
  .rand_on_i       ({rand_d_on        , rand_c_on        , rand_b_on        , rand_a_on        }),
  .rand_pnt_i      ({rand_pnt         , rand_pnt         , rand_pnt         , rand_pnt         })
);



reg  [RSZ-1: 0] trigbuf_rp_a       ;
reg  [RSZ-1: 0] trigbuf_rp_b       ;
reg  [RSZ-1: 0] trigbuf_rp_c       ;
reg  [RSZ-1: 0] trigbuf_rp_d       ;

always @(posedge dac_clk_i) begin
   if (dac_rstn_i == 1'b0) begin
      trigbuf_rp_a <= {RSZ{1'b1}} ;
      trigbuf_rp_b <= {RSZ{1'b1}} ;
      trigbuf_rp_c <= {RSZ{1'b1}} ;
      trigbuf_rp_d <= {RSZ{1'b1}} ;
      end
   else if (trig_scope_i) begin
      trigbuf_rp_a <= buf_a_rpnt;
      trigbuf_rp_b <= buf_b_rpnt;
      trigbuf_rp_c <= buf_c_rpnt;
      trigbuf_rp_d <= buf_d_rpnt;
      end
end



always @(posedge dac_clk_i)
begin
   buf_a_we   <= sys_wen && (sys_addr[19:RSZ+2] == 'h1);
   buf_b_we   <= sys_wen && (sys_addr[19:RSZ+2] == 'h2);
   buf_c_we   <= sys_wen && (sys_addr[19:RSZ+2] == 'h3);
   buf_d_we   <= sys_wen && (sys_addr[19:RSZ+2] == 'h4);
   buf_a_addr <= sys_addr[RSZ+1:2] ;  // address timing violation
   buf_b_addr <= sys_addr[RSZ+1:2] ;  // can change only synchronous to write clock
   buf_c_addr <= sys_addr[RSZ+1:2] ; 
   buf_d_addr <= sys_addr[RSZ+1:2] ; 
end

assign trig_out_o = {trig_d_done, trig_c_done, trig_b_done, trig_a_done};
assign play_active_o = {play_d_act, play_c_act, play_b_act, play_a_act};

reg [1: 0] scope_trig;
wire _scope_trig = scope_trig == 2'b01;

// assign trig_a_slave = (!slave_a_trig && !scope_a_trig) || (slave_a_trig && trig_d_done) || (scope_a_trig && scope_trig);
assign trig_a_slave = (!slave_a_trig && !scope_a_trig) ||  (scope_a_trig && _scope_trig);
assign trig_b_slave = (!slave_b_trig && !scope_b_trig) || (slave_b_trig && play_a_done) || (scope_b_trig && _scope_trig);
assign trig_c_slave = (!slave_c_trig && !scope_c_trig) || (slave_c_trig && play_b_done) || (scope_c_trig && _scope_trig);
assign trig_d_slave = (!slave_d_trig && !scope_d_trig) || (slave_d_trig && play_c_done) || (scope_d_trig && _scope_trig);

//---------------------------------------------------------------------------------
//
//  System bus connection

reg  [3-1: 0] ren_dly ;
reg           ack_dly ;

always @(posedge dac_clk_i)
if (dac_rstn_i == 1'b0) begin
   trig_a_sw   <=  1'b0    ;
   trig_a_src  <=  3'h0    ;
   slave_a_trig <= 1'b0    ;
   scope_a_trig <= 1'b0    ;
   set_a_amp   <= 14'h2000 ;
   set_a_dc    <= 14'h0    ;
   set_a_zero  <=  1'b0    ;
   set_a_rst   <=  1'b0    ;
   set_a_steping <= 1'b1   ;
   set_a_wrap  <=  1'b0    ;
   set_a_size  <= {RSZ+16{1'b1}} ;
   set_a_ofs   <= {RSZ+16{1'b0}} ;
   set_a_step  <={{RSZ+15{1'b0}},1'b0} ;
   set_a_ncyc  <= 32'h0    ;
   set_a_rnum  <= 16'h0    ;
   set_a_rdly  <= 32'h0    ;
   set_a_rgate <=  1'b0    ;
   trig_b_sw   <=  1'b0    ;
   trig_b_src  <=  3'h0    ;
   slave_b_trig <= 1'b0    ;
   scope_b_trig <= 1'b0    ;
   set_b_amp   <= 14'h2000 ;
   set_b_dc    <= 14'h0    ;
   set_b_zero  <=  1'b0    ;
   set_b_rst   <=  1'b0    ;
   set_b_steping <= 1'b1   ;
   set_b_wrap  <=  1'b0    ;
   set_b_size  <= {RSZ+16{1'b1}} ;
   set_b_ofs   <= {RSZ+16{1'b0}} ;
   set_b_step  <={{RSZ+15{1'b0}},1'b0} ;
   set_b_ncyc  <= 32'h0    ;
   set_b_rnum  <= 16'h0    ;
   set_b_rdly  <= 32'h0    ;
   set_b_rgate <=  1'b0    ;
   trig_c_sw   <=  1'b0    ;
   trig_c_src  <=  3'h0    ;
   slave_c_trig <= 1'b0    ;
   scope_c_trig <= 1'b0    ;
   set_c_amp   <= 14'h2000 ;
   set_c_dc    <= 14'h0    ;
   set_c_zero  <=  1'b0    ;
   set_c_rst   <=  1'b0    ;
   set_c_steping <= 1'b1   ;
   set_c_wrap  <=  1'b0    ;
   set_c_size  <= {RSZ+16{1'b1}} ;
   set_c_ofs   <= {RSZ+16{1'b0}} ;
   set_c_step  <={{RSZ+15{1'b0}},1'b0} ;
   set_c_ncyc  <= 32'h0    ;
   set_c_rnum  <= 16'h0    ;
   set_c_rdly  <= 32'h0    ;
   set_c_rgate <=  1'b0    ;
   trig_d_sw   <=  1'b0    ;
   trig_d_src  <=  3'h0    ;
   slave_d_trig <= 1'b0    ;
   scope_d_trig <= 1'b0    ;
   set_d_amp   <= 14'h2000 ;
   set_d_dc    <= 14'h0    ;
   set_d_zero  <=  1'b0    ;
   set_d_rst   <=  1'b0    ;
   set_d_steping <= 1'b1   ;
   set_d_wrap  <=  1'b0    ;
   set_d_size  <= {RSZ+16{1'b1}} ;
   set_d_ofs   <= {RSZ+16{1'b0}} ;
   set_d_step  <={{RSZ+15{1'b0}},1'b0} ;
   set_d_ncyc  <= 32'h0    ;
   set_d_rnum  <= 16'h0    ;
   set_d_rdly  <= 32'h0    ;
   set_d_rgate <=  1'b0    ;
   ren_dly     <=  3'h0    ;
   ack_dly     <=  1'b0    ;
   
   at_counts_a <= {64{1'b0}};
   at_reset_a <= 1'b1; 
   at_invert_a <= 1'b0;
   at_autorearm_a <= 1'b0;
   at_counts_b <= {64{1'b0}};
   at_reset_b <= 1'b1; 
   at_invert_b <= 1'b0;
   at_autorearm_b <= 1'b0;
   at_counts_c <= {64{1'b0}};
   at_reset_c <= 1'b1; 
   at_invert_c <= 1'b0;
   at_autorearm_c <= 1'b0;
   at_counts_d <= {64{1'b0}};
   at_reset_d <= 1'b1; 
   at_invert_d <= 1'b0;
   at_autorearm_d <= 1'b0;

   reverse_a_on <= 1'b0;
   reverse_b_on <= 1'b0;
   reverse_c_on <= 1'b0;
   reverse_d_on <= 1'b0;

   sync_a_on <= 1'b0;
   sync_b_on <= 1'b0;
   sync_c_on <= 1'b0;
   sync_d_on <= 1'b0;

   rand_a_on <= 1'b0;
   rand_b_on <= 1'b0;
   rand_c_on <= 1'b0;
   rand_d_on <= 1'b0;

   scope_trig <= 2'b0;

end else begin

   scope_trig <= {scope_trig[0], scope_trig_i};

   trig_a_sw  <= sys_wen && (sys_addr[19:0]==20'h0) && sys_wdata[0]  ;
   if (sys_wen && (sys_addr[19:0]==20'h0))
      trig_a_src <= sys_wdata[2:0] ;

   trig_b_sw  <= sys_wen && (sys_addr[19:0]==20'h0) && sys_wdata[16]  ;
   if (sys_wen && (sys_addr[19:0]==20'h0))
      trig_b_src <= sys_wdata[18:16] ;

   trig_c_sw  <= sys_wen && (sys_addr[19:0]==20'h50) && sys_wdata[0]  ;
   if (sys_wen && (sys_addr[19:0]==20'h50))
      trig_c_src <= sys_wdata[2:0] ;

   trig_d_sw  <= sys_wen && (sys_addr[19:0]==20'h50) && sys_wdata[16]  ;
   if (sys_wen && (sys_addr[19:0]==20'h50))
      trig_d_src <= sys_wdata[18:16] ;

   if (sys_wen) begin
      if (sys_addr[19:0]==20'h0)   {sync_a_on, reverse_a_on, slave_a_trig, rand_a_on, at_autorearm_a, at_invert_a, at_reset_a, set_a_rgate, set_a_zero, set_a_rst, set_a_steping, set_a_wrap, scope_a_trig} <= sys_wdata[15: 3] ;
      if (sys_addr[19:0]==20'h0)   {sync_b_on, reverse_b_on, slave_b_trig, rand_b_on, at_autorearm_b, at_invert_b, at_reset_b, set_b_rgate, set_b_zero, set_b_rst, set_b_steping, set_b_wrap, scope_b_trig} <= sys_wdata[31:19] ;

      if (sys_addr[19:0]==20'h4)   set_a_amp  <= sys_wdata[  0+13: 0] ;
      if (sys_addr[19:0]==20'h4)   set_a_dc   <= sys_wdata[ 16+13:16] ;
      if (sys_addr[19:0]==20'h8)   set_a_size <= sys_wdata[RSZ+15: 0] ;
      if (sys_addr[19:0]==20'hC)   set_a_ofs  <= sys_wdata[RSZ+15: 0] ;
      if (sys_addr[19:0]==20'h10)  set_a_step <= sys_wdata[RSZ+15: 0] ;
      if (sys_addr[19:0]==20'h18)  set_a_ncyc <= sys_wdata[  32-1: 0] ;
      if (sys_addr[19:0]==20'h1C)  set_a_rnum <= sys_wdata[  16-1: 0] ;
      if (sys_addr[19:0]==20'h20)  set_a_rdly <= sys_wdata[  32-1: 0] ;

      if (sys_addr[19:0]==20'h24)  set_b_amp  <= sys_wdata[  0+13: 0] ;
      if (sys_addr[19:0]==20'h24)  set_b_dc   <= sys_wdata[ 16+13:16] ;
      if (sys_addr[19:0]==20'h28)  set_b_size <= sys_wdata[RSZ+15: 0] ;
      if (sys_addr[19:0]==20'h2C)  set_b_ofs  <= sys_wdata[RSZ+15: 0] ;
      if (sys_addr[19:0]==20'h30)  set_b_step <= sys_wdata[RSZ+15: 0] ;
      if (sys_addr[19:0]==20'h38)  set_b_ncyc <= sys_wdata[  32-1: 0] ;
      if (sys_addr[19:0]==20'h3C)  set_b_rnum <= sys_wdata[  16-1: 0] ;
      if (sys_addr[19:0]==20'h40)  set_b_rdly <= sys_wdata[  32-1: 0] ;

      if (sys_addr[19:0]==20'h118)  at_counts_a[32-1:0]  <= sys_wdata[32-1: 0] ;
      if (sys_addr[19:0]==20'h11C)  at_counts_a[64-1:32] <= sys_wdata[32-1: 0] ;
      if (sys_addr[19:0]==20'h138)  at_counts_b[32-1:0]  <= sys_wdata[32-1: 0] ;
      if (sys_addr[19:0]==20'h13C)  at_counts_b[64-1:32] <= sys_wdata[32-1: 0] ;

      if (sys_addr[19:0]==20'h50)   {sync_c_on, reverse_c_on, slave_c_trig, rand_c_on, at_autorearm_c, at_invert_c, at_reset_c, set_c_rgate, set_c_zero, set_c_rst, set_c_steping, set_c_wrap, scope_c_trig} <= sys_wdata[15: 3] ;
      if (sys_addr[19:0]==20'h50)   {sync_d_on, reverse_d_on, slave_d_trig, rand_d_on, at_autorearm_d, at_invert_d, at_reset_d, set_d_rgate, set_d_zero, set_d_rst, set_d_steping, set_d_wrap, scope_d_trig} <= sys_wdata[31:19] ;

      if (sys_addr[19:0]==20'h54)   set_c_amp  <= sys_wdata[  0+13: 0] ;
      if (sys_addr[19:0]==20'h54)   set_c_dc   <= sys_wdata[ 16+13:16] ;
      if (sys_addr[19:0]==20'h58)   set_c_size <= sys_wdata[RSZ+15: 0] ;
      if (sys_addr[19:0]==20'h5C)   set_c_ofs  <= sys_wdata[RSZ+15: 0] ;
      if (sys_addr[19:0]==20'h60)  set_c_step <= sys_wdata[RSZ+15: 0] ;
      if (sys_addr[19:0]==20'h68)  set_c_ncyc <= sys_wdata[  32-1: 0] ;
      if (sys_addr[19:0]==20'h6C)  set_c_rnum <= sys_wdata[  16-1: 0] ;
      if (sys_addr[19:0]==20'h70)  set_c_rdly <= sys_wdata[  32-1: 0] ;

      if (sys_addr[19:0]==20'h74)  set_d_amp  <= sys_wdata[  0+13: 0] ;
      if (sys_addr[19:0]==20'h74)  set_d_dc   <= sys_wdata[ 16+13:16] ;
      if (sys_addr[19:0]==20'h78)  set_d_size <= sys_wdata[RSZ+15: 0] ;
      if (sys_addr[19:0]==20'h7C)  set_d_ofs  <= sys_wdata[RSZ+15: 0] ;
      if (sys_addr[19:0]==20'h80)  set_d_step <= sys_wdata[RSZ+15: 0] ;
      if (sys_addr[19:0]==20'h88)  set_d_ncyc <= sys_wdata[  32-1: 0] ;
      if (sys_addr[19:0]==20'h8C)  set_d_rnum <= sys_wdata[  16-1: 0] ;
      if (sys_addr[19:0]==20'h90)  set_d_rdly <= sys_wdata[  32-1: 0] ;

      if (sys_addr[19:0]==20'h158)  at_counts_c[32-1:0]  <= sys_wdata[32-1: 0] ;
      if (sys_addr[19:0]==20'h15C)  at_counts_c[64-1:32] <= sys_wdata[32-1: 0] ;
      if (sys_addr[19:0]==20'h178)  at_counts_d[32-1:0]  <= sys_wdata[32-1: 0] ;
      if (sys_addr[19:0]==20'h17C)  at_counts_d[64-1:32] <= sys_wdata[32-1: 0] ;

   end

   // disabled to improve fpga timing and space
   //if (sys_ren) begin
   //   buf_a_rpnt_rd <= {{32-RSZ-2{1'b0}},buf_a_rpnt,2'h0};
   //   buf_b_rpnt_rd <= {{32-RSZ-2{1'b0}},buf_b_rpnt,2'h0};
   //end

   ren_dly <= {ren_dly[3-2:0], sys_ren};
   ack_dly <=  ren_dly[3-1] || sys_wen ;
end

wire [32-1: 0] r0_rd = {sync_b_on, reverse_b_on, slave_b_trig, rand_b_on,at_autorearm_b,at_invert_b,at_reset_b,set_b_rgate, set_b_zero,set_b_rst,set_b_steping,set_b_wrap, scope_b_trig, trig_b_src,
                        sync_a_on, reverse_a_on, slave_a_trig, rand_a_on,at_autorearm_a,at_invert_a,at_reset_a,set_a_rgate, set_a_zero,set_a_rst,set_a_steping,set_a_wrap, scope_a_trig, trig_a_src };

wire [32-1: 0] r1_rd = {sync_d_on, reverse_d_on, slave_d_trig, rand_d_on,at_autorearm_d,at_invert_d,at_reset_d,set_d_rgate, set_d_zero,set_d_rst,set_d_steping,set_d_wrap, scope_d_trig, trig_d_src,
                        sync_c_on, reverse_c_on, slave_c_trig, rand_c_on,at_autorearm_c,at_invert_c,at_reset_c,set_c_rgate, set_c_zero,set_c_rst,set_c_steping,set_c_wrap, scope_c_trig, trig_c_src };

wire sys_en;
assign sys_en = sys_wen | sys_ren;

always @(posedge dac_clk_i)
if (dac_rstn_i == 1'b0) begin
   sys_err <= 1'b0 ;
   sys_ack <= 1'b0 ;
end else begin
   sys_err <= 1'b0 ;

   casez (sys_addr[19:0])
     20'h00000 : begin sys_ack <= sys_en;          sys_rdata <= r0_rd                              ; end

     20'h00004 : begin sys_ack <= sys_en;          sys_rdata <= {2'h0, set_a_dc, 2'h0, set_a_amp}  ; end
     20'h00008 : begin sys_ack <= sys_en;          sys_rdata <= {{32-RSZ-16{1'b0}},set_a_size}     ; end
     20'h0000C : begin sys_ack <= sys_en;          sys_rdata <= {{32-RSZ-16{1'b0}},set_a_ofs}      ; end
     20'h00010 : begin sys_ack <= sys_en;          sys_rdata <= {{32-RSZ-16{1'b0}},set_a_step}     ; end
     20'h00014 : begin sys_ack <= sys_en;          sys_rdata <= buf_a_rpnt_rd                      ; end
     20'h00018 : begin sys_ack <= sys_en;          sys_rdata <= set_a_ncyc                         ; end
     20'h0001C : begin sys_ack <= sys_en;          sys_rdata <= {{32-16{1'b0}},set_a_rnum}         ; end
     20'h00020 : begin sys_ack <= sys_en;          sys_rdata <= set_a_rdly                         ; end

     20'h00024 : begin sys_ack <= sys_en;          sys_rdata <= {2'h0, set_b_dc, 2'h0, set_b_amp}  ; end
     20'h00028 : begin sys_ack <= sys_en;          sys_rdata <= {{32-RSZ-16{1'b0}},set_b_size}     ; end
     20'h0002C : begin sys_ack <= sys_en;          sys_rdata <= {{32-RSZ-16{1'b0}},set_b_ofs}      ; end
     20'h00030 : begin sys_ack <= sys_en;          sys_rdata <= {{32-RSZ-16{1'b0}},set_b_step}     ; end
     20'h00034 : begin sys_ack <= sys_en;          sys_rdata <= buf_b_rpnt_rd                      ; end
     20'h00038 : begin sys_ack <= sys_en;          sys_rdata <= set_b_ncyc                         ; end
     20'h0003C : begin sys_ack <= sys_en;          sys_rdata <= {{32-16{1'b0}},set_b_rnum}         ; end
     20'h00040 : begin sys_ack <= sys_en;          sys_rdata <= set_b_rdly                         ; end

     20'h00050 : begin sys_ack <= sys_en;          sys_rdata <= r1_rd                              ; end

     20'h00054 : begin sys_ack <= sys_en;          sys_rdata <= {2'h0, set_c_dc, 2'h0, set_c_amp}  ; end
     20'h00058 : begin sys_ack <= sys_en;          sys_rdata <= {{32-RSZ-16{1'b0}},set_c_size}     ; end
     20'h0005C : begin sys_ack <= sys_en;          sys_rdata <= {{32-RSZ-16{1'b0}},set_c_ofs}      ; end
     20'h00060 : begin sys_ack <= sys_en;          sys_rdata <= {{32-RSZ-16{1'b0}},set_c_step}     ; end
     20'h00064 : begin sys_ack <= sys_en;          sys_rdata <= buf_c_rpnt_rd                      ; end
     20'h00068 : begin sys_ack <= sys_en;          sys_rdata <= set_c_ncyc                         ; end
     20'h0006C : begin sys_ack <= sys_en;          sys_rdata <= {{32-16{1'b0}},set_c_rnum}         ; end
     20'h00070 : begin sys_ack <= sys_en;          sys_rdata <= set_c_rdly                         ; end

     20'h00074 : begin sys_ack <= sys_en;          sys_rdata <= {2'h0, set_d_dc, 2'h0, set_d_amp}  ; end
     20'h00078 : begin sys_ack <= sys_en;          sys_rdata <= {{32-RSZ-16{1'b0}},set_d_size}     ; end
     20'h0007C : begin sys_ack <= sys_en;          sys_rdata <= {{32-RSZ-16{1'b0}},set_d_ofs}      ; end
     20'h00080 : begin sys_ack <= sys_en;          sys_rdata <= {{32-RSZ-16{1'b0}},set_d_step}     ; end
     20'h00084 : begin sys_ack <= sys_en;          sys_rdata <= buf_d_rpnt_rd                      ; end
     20'h00088 : begin sys_ack <= sys_en;          sys_rdata <= set_d_ncyc                         ; end
     20'h0008C : begin sys_ack <= sys_en;          sys_rdata <= {{32-16{1'b0}},set_d_rnum}         ; end
     20'h00090 : begin sys_ack <= sys_en;          sys_rdata <= set_d_rdly                         ; end

     20'h00114 : begin sys_ack <= sys_en;          sys_rdata <= {{32-RSZ-2{1'b0}},trigbuf_rp_a}    ; end
     20'h00118 : begin sys_ack <= sys_en;          sys_rdata <= at_counts_a[32-1:0]                ; end
     20'h0011C : begin sys_ack <= sys_en;          sys_rdata <= at_counts_a[64-1:32]               ; end
     20'h00120 : begin sys_ack <= sys_en;          sys_rdata <= step_a_o                           ; end

     20'h00134 : begin sys_ack <= sys_en;          sys_rdata <= {{32-RSZ-2{1'b0}},trigbuf_rp_b}    ; end
     20'h00138 : begin sys_ack <= sys_en;          sys_rdata <= at_counts_b[32-1:0]                ; end
     20'h0013C : begin sys_ack <= sys_en;          sys_rdata <= at_counts_b[64-1:32]               ; end
     20'h00140 : begin sys_ack <= sys_en;          sys_rdata <= step_b_o                           ; end

     20'h00164 : begin sys_ack <= sys_en;          sys_rdata <= {{32-RSZ-2{1'b0}},trigbuf_rp_c}    ; end
     20'h00168 : begin sys_ack <= sys_en;          sys_rdata <= at_counts_c[32-1:0]                ; end
     20'h0016C : begin sys_ack <= sys_en;          sys_rdata <= at_counts_c[64-1:32]               ; end
     20'h00170 : begin sys_ack <= sys_en;          sys_rdata <= step_c_o                           ; end

     20'h00184 : begin sys_ack <= sys_en;          sys_rdata <= {{32-RSZ-2{1'b0}},trigbuf_rp_d}    ; end
     20'h00188 : begin sys_ack <= sys_en;          sys_rdata <= at_counts_d[32-1:0]                ; end
     20'h0018C : begin sys_ack <= sys_en;          sys_rdata <= at_counts_d[64-1:32]               ; end
     20'h00190 : begin sys_ack <= sys_en;          sys_rdata <= step_d_o                           ; end

	 20'h1zzzz : begin sys_ack <= ack_dly;         sys_rdata <= {{32-14{1'b0}},buf_a_rdata}        ; end
     20'h2zzzz : begin sys_ack <= ack_dly;         sys_rdata <= {{32-14{1'b0}},buf_b_rdata}        ; end
     20'h3zzzz : begin sys_ack <= ack_dly;         sys_rdata <= {{32-14{1'b0}},buf_c_rdata}        ; end
     20'h4zzzz : begin sys_ack <= ack_dly;         sys_rdata <= {{32-14{1'b0}},buf_d_rdata}        ; end

       default : begin sys_ack <= sys_en;          sys_rdata <=  32'h0                             ; end
   endcase
end

// forward the current phase of asg1;
assign asg1phase_o = buf_a_rpnt;



//red_pitaya_prng_lehmer
red_pitaya_prng_xor  #(.OUTBITS (RSZ)) prng (
  .clk_i       (dac_clk_i  ),  // dac clock
  .reset_i     (dac_rstn_i ),  // dac reset - active low
  .signal_o    (rand_pnt)
);


endmodule
