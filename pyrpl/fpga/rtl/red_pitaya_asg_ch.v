/**
 * $Id: red_pitaya_asg_ch.v 1271 2014-02-25 12:32:34Z matej.oblak $
 *
 * @brief Red Pitaya ASG submodule. Holds table and FSM for one channel.
 *
 * @Author Matej Oblak
 *
 * (c) Red Pitaya  http://www.redpitaya.com
 *
 * This part of code is written in Verilog hardware description language (HDL).
 * Please visit http://en.wikipedia.org/wiki/Verilog
 * for more details on the language used herein.
 */

/**
 * GENERAL DESCRIPTION:
 *
 * Arbitrary signal generator takes data stored in buffer and sends them to DAC.
 *
 *
 *                /-----\         /--------\
 *   SW --------> | BUF | ------> | kx + o | ---> DAC DAT
 *          |     \-----/         \--------/
 *          |        ^
 *          |        |
 *          |     /-----\
 *          ----> |     |
 *                | FSM | ------> trigger notification
 *   trigger ---> |     |
 *                \-----/
 *
 *
 * Submodule for ASG which hold buffer data and control registers for one channel.
 * 
 */

module red_pitaya_asg_ch #(
   parameter RSZ = 14,
   parameter CYCLE_BITS = 32
)(
   // DAC
   output reg [ 14-1: 0] dac_o           ,  //!< dac data output
   input                 dac_clk_i       ,  //!< dac clock
   input                 dac_rstn_i      ,  //!< dac reset - active low
   // trigger
   input                 trig_sw_i       ,  //!< software trigger
   input                 trig_ext_i      ,  //!< external trigger
   input      [  3-1: 0] trig_src_i      ,  //!< trigger source selector
   input                 trig_slave_i    ,  //!< slave trigger
   output                trig_done_o     ,  //!< trigger event
   output                play_done_o     ,  //!< data play done event
   
   // buffer ctrl
   input                 buf_we_i        ,  //!< buffer write enable
   input      [ 14-1: 0] buf_addr_i      ,  //!< buffer address
   input      [ 14-1: 0] buf_wdata_i     ,  //!< buffer write data
   output reg [ 14-1: 0] buf_rdata_o     ,  //!< buffer read data
   output reg [RSZ-1: 0] buf_rpnt_o      ,  //!< buffer current read pointer
   output reg [RSZ-1: 0] step_o          ,  //!< buffer current step

   // configuration
   input     [RSZ+16-1: 0] set_size_i      ,  //!< set table data size
   input     [RSZ+16-1: 0] set_step_i      ,  //!< set pointer step
   input     [RSZ+16-1: 0] set_ofs_i       ,  //!< set reset offset
   input                 set_rst_i       ,  //!< set FSM to reset
   input                 set_once_i      ,  //!< set only once  -- not used
   input                 set_wrap_i      ,  //!< set wrap enable
   input     [  14-1: 0] set_amp_i       ,  //!< set amplitude scale
   input     [  14-1: 0] set_dc_i        ,  //!< set output offset
   input                 set_zero_i      ,  //!< set output to zero
   input     [  CYCLE_BITS-1: 0] set_ncyc_i      ,  //!< set number of cycle
   input     [  16-1: 0] set_rnum_i      ,  //!< set number of repetitions
   input     [  32-1: 0] set_rdly_i      ,  //!< set delay between repetitions
   input                 set_rgate_i     ,  //!< set external gated repetition

   input                 reverse_on_i  ,  //!< enable reverse play on each repetition

   input                 rand_on_i     , // random number generator on
   input     [RSZ-1:0]   rand_pnt_i      // random pointer for output data

);

//---------------------------------------------------------------------------------
//
//  DAC buffer RAM

wire  [  14-1: 0] dac_rdat  ;
reg   [ RSZ-1: 0] dac_rp    ;
reg   [RSZ+16-1: 0] dac_pnt   ; // read pointer
reg   [RSZ+16-1: 0] dac_pntp  ; // previous read pointer
reg   [RSZ+17-1: 0] dac_npnt  ; // next read pointer

wire  [RSZ+16-1: 0] dac_npnt2 ; // next read pointer without sign bit
assign dac_npnt2 = dac_npnt[RSZ+16-1: 0];

wire  [RSZ+17-1: 0] dac_npnt_sub ;
wire              dac_npnt_sub_neg;

reg   [  28-1: 0] dac_mult  ;
reg   [  15-1: 0] dac_sum   ;

reg               buf_we;
reg   [  14-1: 0] dac_wd;
reg   [  14-1: 0] buf_addr;

xpm_memory_sdpram #(
    .MEMORY_PRIMITIVE       ("block"),
    .MEMORY_SIZE            ((1<<RSZ)*14),
    .ADDR_WIDTH_A           (RSZ),
    .ADDR_WIDTH_B           (RSZ),
    .CLOCKING_MODE          ("common_clock"),
    .WRITE_MODE_B           ("read_first"),
    .READ_LATENCY_B         (2),
    .READ_DATA_WIDTH_B      (14),
    .WRITE_DATA_WIDTH_A     (14),
    .BYTE_WRITE_WIDTH_A     (14)
) dac_buf (
    .addra  (buf_addr),
    .addrb  (dac_rp),
    .clka   (dac_clk_i),
    .dina   (dac_wd),
    .doutb  (dac_rdat),
    .ena    (buf_we),
    .wea    (1'b1),
    .rstb   (1'b0),
    .regceb (1'b1),
    .enb    (1'b1)
);

// read
always @(posedge dac_clk_i)
begin
   buf_rpnt_o <= dac_pnt[16+RSZ-1:16];
   dac_rp     <= (rand_on_i == 1'b1) ? rand_pnt_i : dac_pnt[RSZ+15:16];
end

// write
always @(posedge dac_clk_i) begin
    dac_wd <= buf_wdata_i[14-1:0];
    buf_addr <= buf_addr_i;
    buf_we <= buf_we_i;
end

// scale and offset
always @(posedge dac_clk_i)
begin
   dac_mult <= $signed(dac_rdat) * $signed({1'b0,set_amp_i}) ;
   dac_sum  <= $signed(dac_mult[28-1:13]) + $signed(set_dc_i) ;

   // saturation
   if (set_zero_i)  dac_o <= 14'h0;
   else             dac_o <= ^dac_sum[15-1:15-2] ? {dac_sum[15-1], {13{~dac_sum[15-1]}}} : dac_sum[13:0];
end

//---------------------------------------------------------------------------------
//
//  read pointer & state machine

reg              trig_in      ;
wire             ext_trig_p   ;
wire             ext_trig_n   ;

reg  [  32-1: 0] cyc_cnt      ;
reg  [  16-1: 0] rep_cnt      ;
reg  [  32-1: 0] dly_cnt      ;
reg  [   8-1: 0] dly_tick     ;

reg              dac_do       ;
reg              dac_rep      ;
wire             dac_trig     ;
reg              dac_trigr    ;

reg              reverse_run  ;

// state machine
always @(posedge dac_clk_i) begin
   if (dac_rstn_i == 1'b0) begin
      cyc_cnt   <= {CYCLE_BITS{1'b0}} ;
      rep_cnt   <= 16'h0 ;
      dly_cnt   <= 32'h0 ;
      dly_tick  <=  8'h0 ;
      dac_do    <=  1'b0 ;
      dac_rep   <=  1'b0 ;
      trig_in   <=  1'b0 ;
      dac_pntp  <= {RSZ+16{1'b0}} ;
      dac_trigr <=  1'b0 ;
   end
   else begin
      // make 1us tick
      if (dac_do || (dly_tick == 8'd124))
         dly_tick <= 8'h0 ;
      else
         dly_tick <= dly_tick + 8'h1 ;

      // delay between repetitions 
      if (set_rst_i || dac_do)
         dly_cnt <= set_rdly_i ;
      else if (|dly_cnt && (dly_tick == 8'd124))
         dly_cnt <= dly_cnt - 32'h1 ;

      // repetitions counter
      if (trig_in && !dac_do)
         rep_cnt <= set_rnum_i ;
      else if (!set_rgate_i && (|rep_cnt && dac_rep && (dac_trig && !dac_do)))
         rep_cnt <= rep_cnt - 16'h1 ;
      else if (set_rgate_i && ((!trig_ext_i && trig_src_i==3'd2) || (trig_ext_i && trig_src_i==3'd3)))
         rep_cnt <= 16'h0 ;

      // count number of table read cycles
      dac_pntp  <= dac_pnt;
      dac_trigr <= dac_trig; // ignore trigger when count
      if (dac_trig)
         cyc_cnt <= set_ncyc_i ;
      else if (!dac_trigr && |cyc_cnt && ({1'b0,dac_pntp} > {1'b0,dac_pnt}))
         cyc_cnt <= cyc_cnt - 32'h1 ;

      // trigger arrived
      case (trig_src_i)
          3'd1 : trig_in <= trig_sw_i   ; // sw
          3'd2 : trig_in <= ext_trig_p  ; // external positive edge
          3'd3 : trig_in <= ext_trig_n  ; // external negative edge
          3'd4 : trig_in <= trig_ext_i  ; // unprocessed ext trigger
          3'd5 : trig_in <= 1'b1  ;       // always high

       default : trig_in <= 1'b0        ;
      endcase

      // in cycle mode
      if (dac_trig && !set_rst_i)
         dac_do <= 1'b1 ;
      else if (set_rst_i || ((cyc_cnt==32'h1) && ~dac_npnt_sub_neg) )
         dac_do <= 1'b0 ;

      // in repetition mode
      if (dac_trig && !set_rst_i)
         dac_rep <= 1'b1 ;
      else if (set_rst_i || (rep_cnt==16'h0))
         dac_rep <= 1'b0 ;
   end
end

assign dac_trig = (!dac_rep && trig_in) || (dac_rep && |rep_cnt && (dly_cnt == 32'h0)) ;

assign dac_npnt_sub = dac_npnt - {1'b0,set_size_i} - 1;
assign dac_npnt_sub_neg = dac_npnt_sub[RSZ+16];

wire trig_done     ;
assign trig_done = ((~dac_npnt_sub_neg) | (reverse_run && reverse_on_i && step_o==0)) && trig_slave_i;
assign play_done_o = trig_done;
reg trig_done_prev ;
assign trig_done_o = (!dac_rep && trig_in) | (trig_done && !trig_done_prev);

// read pointer logic
always @(posedge dac_clk_i)
if (dac_rstn_i == 1'b0) begin
   dac_pnt  <= {RSZ+16{1'b0}};
   reverse_run  <=  0 ;
   step_o <= 1'b0 ;
   trig_done_prev <= 0;
end else begin
   trig_done_prev <= trig_done;
   if (set_rst_i || (dac_trig && !dac_do)) begin// manual reset or start
      if (!set_rst_i && reverse_run && reverse_on_i) begin
         dac_pnt <= set_size_i - 1;
         dac_npnt <= {1'b0, set_size_i - 1 - set_step_i};
      end else begin
         dac_pnt <= set_ofs_i;
         dac_npnt <= set_ofs_i + set_step_i;
         step_o <= 0;
      end
      if (set_rst_i) begin
         reverse_run <= 0;
      end
   end else if (dac_do && trig_slave_i) begin
      if (reverse_run && reverse_on_i) begin
         if (step_o == 0) begin
            reverse_run <= 0;
            dac_pnt <= set_ofs_i;
            dac_npnt <= set_ofs_i + set_step_i;
         end else begin
            dac_pnt <= dac_npnt2;
            if (dac_npnt2 > set_ofs_i + set_step_i)
                dac_npnt <= {1'b0, dac_npnt2 - set_step_i};
            else
                dac_npnt <= {1'b0, set_ofs_i};
            step_o <= step_o - 1;
         end
      end else if (~dac_npnt_sub_neg) begin
         if (reverse_on_i) begin
            reverse_run <= 1;
            dac_npnt <= dac_pnt - set_step_i;
         end else begin
            reverse_run <= 0;
            dac_pnt <= set_wrap_i ? dac_npnt_sub : set_ofs_i; // wrap or go to start
            dac_npnt <= (set_wrap_i ? dac_npnt_sub : set_ofs_i) + set_step_i;
            step_o <= 0;
         end
      end else begin
         dac_pnt <= dac_npnt2; // normal increase
         dac_npnt <= dac_npnt + set_step_i;
         step_o <= step_o + 1;
      end
   end
end

//---------------------------------------------------------------------------------
//
//  External trigger

reg  [  3-1: 0] ext_trig_in    ;
reg  [  2-1: 0] ext_trig_dp    ;
reg  [  2-1: 0] ext_trig_dn    ;
reg  [ 20-1: 0] ext_trig_debp  ;
reg  [ 20-1: 0] ext_trig_debn  ;

always @(posedge dac_clk_i) begin
   if (dac_rstn_i == 1'b0) begin
      ext_trig_in   <=  3'h0 ;
      ext_trig_dp   <=  2'h0 ;
      ext_trig_dn   <=  2'h0 ;
      ext_trig_debp <= 20'h0 ;
      ext_trig_debn <= 20'h0 ;
   end
   else begin
      //----------- External trigger
      // synchronize FFs
      ext_trig_in <= {ext_trig_in[1:0],trig_ext_i} ;

      // look for input changes
      if ((ext_trig_debp == 20'h0) && (ext_trig_in[1] && !ext_trig_in[2]))
         ext_trig_debp <= 20'd62500 ; // ~0.5ms
      else if (ext_trig_debp != 20'h0)
         ext_trig_debp <= ext_trig_debp - 20'd1 ;

      if ((ext_trig_debn == 20'h0) && (!ext_trig_in[1] && ext_trig_in[2]))
         ext_trig_debn <= 20'd62500 ; // ~0.5ms
      else if (ext_trig_debn != 20'h0)
         ext_trig_debn <= ext_trig_debn - 20'd1 ;

      // update output values
      ext_trig_dp[1] <= ext_trig_dp[0] ;
      if (ext_trig_debp == 20'h0)
         ext_trig_dp[0] <= ext_trig_in[1] ;

      ext_trig_dn[1] <= ext_trig_dn[0] ;
      if (ext_trig_debn == 20'h0)
         ext_trig_dn[0] <= ext_trig_in[1] ;
   end
end

assign ext_trig_p = (ext_trig_dp == 2'b01) ;
assign ext_trig_n = (ext_trig_dn == 2'b10) ;

endmodule
