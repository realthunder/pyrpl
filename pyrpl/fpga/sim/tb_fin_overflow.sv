// tb_fin_overflow — characterize fin FIFO overflow vs wait1 + xfft padding-phase
// backpressure (the "bubble"), find the peak occupancy / optimal depth, verify the
// "no overflow after padding_done" property, and test the pad-compensation fix.
//
// Faithful to fft_proc.sv at FFT_CLK_SEL=0 (clk_i == adc_clk, single clock):
//   * engine feed FSM copied verbatim (padding then data, up + down halves)
//   * writes are NOT gated on full (HW drops on overflow) -> can actually overflow
//   * a write-side FSM mirrors the scope windows: trig -> WAIT1 -> UP(acq) ->
//     WAIT2 -> DOWN(acq), 1 sample/cycle, started by the same trigger as the engine
//   * xfft backpressure modelled as a stall of saxi_rdy for BUBBLE cycles at frame
//     start (i.e. during the engine's up-padding) — the worst case for fill.
//
// Regression for the input-FIFO under-run hang + post-pad fix (fft_proc.sv).
// Default (shipped fix) -> RESULT ... : OK ; build -d NOPOSTPAD -> ... : HUNG.
// Knobs (-d): QSZ WAIT1 WAIT2 BUBBLE ACQ_UP_B ACQ_DOWN_B NOPOSTPAD.
`timescale 1ns/1ps
`ifndef QSZ
  `define QSZ 7
`endif
`ifndef WAIT1
  `define WAIT1 0
`endif
`ifndef WAIT2
  `define WAIT2 84
`endif
`ifndef BUBBLE
  `define BUBBLE 60
`endif
`ifndef ACQ_UP_B
  `define ACQ_UP_B 31
`endif
`ifndef ACQ_DOWN_B
  `define ACQ_DOWN_B 31
`endif

module tb_fin_overflow;
  localparam int FSSR = 4, ASZ = 14;
  localparam int QSZ = `QSZ;
  localparam int DEPTH = (1<<QSZ);                 // FIFO depth in samples
  localparam int FFT_LEN  = 128;                   // beats per half frame (2^(FSZ-SSRbits))
  localparam int FFT_LEN2 = 256;                   // up+down beats
  localparam int FFT_LEN_PLUS_TWO = 130;
  localparam int ACQ_UP_B   = `ACQ_UP_B;           // up   data beats
  localparam int ACQ_DOWN_B = `ACQ_DOWN_B;         // down data beats
  localparam int PAD_UP   = FFT_LEN - ACQ_UP_B;    // up   padding beats (97)
  localparam int PAD_DOWN = FFT_LEN - ACQ_DOWN_B;
  localparam int ACQ_UP_S   = ACQ_UP_B*FSSR;       // up   samples written
  localparam int ACQ_DOWN_S = ACQ_DOWN_B*FSSR;
  localparam int WAIT1 = `WAIT1, WAIT2 = `WAIT2;
  localparam int BUBBLE = `BUBBLE;
  localparam int PERIOD = 415;                     // chirp period (cycles)
  localparam int NTRIG  = 30;

  logic clk = 0; always #4 clk = ~clk;
  logic rstn = 0;

  // ---------- engine FSM signals (verbatim names from fft_proc.sv) ----------
  logic [31:0] fft_we_cnt;
  logic        fft_we_one, fft_we_length_plus_one;
  logic        fin_rd, fft_done, up_in;
  logic [15:0] padding_cnt;            // wide enough for pad-comp bumps
  logic        padding_done;
  logic        fft_done_o;
  logic        fin_rst;
  logic        fft_trig;

  // ---------- fin FIFO ----------
  logic [FSSR*ASZ-1:0] fin_dout;
  logic                fin_dvalid, fin_full, fin_full_flag;
  logic                wr_en;
  logic [ASZ-1:0]      din;
  wire                 rd_en = padding_done & saxi_rdy & fin_rd;

  assign fin_rst = !rstn || (fft_trig && fft_done_o);

  xpm_fifo_async #(
    .FIFO_WRITE_DEPTH(DEPTH), .WRITE_DATA_WIDTH(ASZ), .READ_DATA_WIDTH(ASZ*FSSR),
    .FIFO_READ_LATENCY(0), .USE_ADV_FEATURES("1001"), .READ_MODE("fwft")
  ) fifo_in (
    .rst(fin_rst), .wr_clk(clk), .wr_en(wr_en), .din(din), .full(fin_full_flag),
    .rd_clk(clk), .rd_en(rd_en), .dout(fin_dout), .data_valid(fin_dvalid),
    .overflow(fin_full)
  );

  // ---------- acquisition-complete per half (from the write/scope FSM) ----------
  // Latches when this half's acquisition window has ended (no more real samples
  // will be written). Resets at each frame trigger. up_in selects the half.
  logic wrote_up_done, wrote_down_done;
  wire  acq_done = up_in ? wrote_up_done : wrote_down_done;

  // saxi handshake. POST-PAD: once this half's acquisition is done AND the FIFO is
  // drained of real beats, feed ZERO beats instead of stalling -> the frame always
  // completes (fft_we_cnt reaches the boundary), data+postpad self-balance to acq.
`ifndef NOPOSTPAD
  wire postpad = padding_done && acq_done && !fin_dvalid;   // shipped fix (default) -> expect OK
`else
  wire postpad = 1'b0;                                       // -d NOPOSTPAD reproduces the bug -> expect HUNG
`endif
  logic        saxi_rdy, saxi_valid, saxi_last;
  assign saxi_valid = (!padding_done || fin_dvalid || postpad) && fin_rd;
  assign saxi_last  = fft_we_one || fft_we_length_plus_one;

  // ---------- xfft backpressure model: stall BUBBLE cycles after each trigger ----------
  integer bub = 0;
  always @(posedge clk) if (!rstn) bub <= 0; else if (fft_trig) bub <= BUBBLE; else if (bub>0) bub <= bub-1;
  assign saxi_rdy = (bub == 0);

  // ---------- engine FSM (verbatim from fft_proc.sv:687-730; postpad needs NO
  // FSM change — it works purely via saxi_valid feeding zeros) ----------
  always @(posedge clk)
  if (!rstn) begin
    fft_we_cnt<=0; fft_we_one<=0; fft_we_length_plus_one<=0;
    fin_rd<=0; fft_done<=1; up_in<=1; padding_cnt<=0; padding_done<=0;
  end else begin
    if (fft_trig && fft_done && up_in) begin
      fft_we_cnt <= FFT_LEN2; fft_we_one<=0; fft_we_length_plus_one<=0;
      fin_rd<=1; padding_cnt<=PAD_UP; padding_done<=0; fft_done<=0;
    end else begin
      if (!fft_done && saxi_valid && saxi_rdy) begin
        if (fft_we_one) begin
          up_in<=1; fin_rd<=0;
        end else if (fft_we_length_plus_one) begin
          up_in<=0; padding_cnt<=PAD_DOWN; padding_done<=0;
        end else if (!padding_done) begin
          padding_cnt<=padding_cnt-1; padding_done<=(padding_cnt==1);
        end
        fft_we_cnt<=fft_we_cnt-1;
        fft_done<=fft_we_one;
        fft_we_one<=(fft_we_cnt==2);
        fft_we_length_plus_one<=(fft_we_cnt==FFT_LEN_PLUS_TWO);
      end
    end
  end

  // fft_done_o: model adc<-clk CDC (2FF) — at SEL=0 just a couple FF of latency
  logic d1,d2; always @(posedge clk) begin d1<=fft_done&&up_in; d2<=d1; fft_done_o<=d2; end

  // ---------- write-side FSM: mirror scope windows ----------
  typedef enum logic [2:0] {W_IDLE,W_WAIT1,W_UP,W_WAIT2,W_DOWN} wst_t;
  wst_t wst; integer wcnt;
  logic trig_tick; integer pcnt;
  always @(posedge clk) if (!rstn) begin pcnt<=0; trig_tick<=0; end
    else begin trig_tick<=(pcnt==0); pcnt<=(pcnt>=PERIOD-1)?0:pcnt+1; end

  assign fft_trig = trig_tick && (wst==W_IDLE) && fft_done;  // chirp trig accepted only when ready
  always @(posedge clk) if (!rstn) begin wst<=W_IDLE; wcnt<=0; wr_en<=0; din<=0; end
  else begin
    wr_en <= 0;
    case (wst)
      W_IDLE:  if (fft_trig) begin wst<=W_WAIT1; wcnt<=0; end
      W_WAIT1: if (wcnt>=WAIT1-1) begin wst<=W_UP; wcnt<=0; end else wcnt<=wcnt+1;
      W_UP:    begin wr_en<=1; din<=din+1; if (wcnt>=ACQ_UP_S-1) begin wst<=W_WAIT2; wcnt<=0; end else wcnt<=wcnt+1; end
      W_WAIT2: if (wcnt>=WAIT2-1) begin wst<=W_DOWN; wcnt<=0; end else wcnt<=wcnt+1;
      W_DOWN:  begin wr_en<=1; din<=din+1; if (wcnt>=ACQ_DOWN_S-1) begin wst<=W_IDLE; wcnt<=0; end else wcnt<=wcnt+1; end
    endcase
  end

  // acquisition-complete latches (per half), reset at trigger
  always @(posedge clk) if (!rstn) begin wrote_up_done<=0; wrote_down_done<=0; end
  else begin
    if (fft_trig) begin wrote_up_done<=0; wrote_down_done<=0; end
    if (wst==W_UP   && wcnt>=ACQ_UP_S-1)   wrote_up_done<=1;
    if (wst==W_DOWN && wcnt>=ACQ_DOWN_S-1) wrote_down_done<=1;
  end

  // ---------- instrumentation ----------
  integer fill=0, maxfill=0, drops=0, frames=0;
  integer ovf_after_pad=0;          // PROOF check: overflow pulses while padding_done==1
  logic   acc; assign acc = wr_en && !fin_full_flag;   // accepted write
  always @(posedge clk) if (rstn) begin
    if (fin_rst) fill<=0;
    else fill <= fill + (acc?1:0) - ((rd_en&&fin_dvalid)?FSSR:0);
    if (fill>maxfill) maxfill<=fill;
    if (fin_full) begin drops<=drops+1; if (padding_done) ovf_after_pad<=ovf_after_pad+1; end
    if (saxi_valid && saxi_rdy && saxi_last) frames<=frames+1;
  end

  // ---------- hang detector ----------
  integer idle=0; integer lf=-1, lc=-1;
  always @(posedge clk) if (rstn) begin
    if (frames!=lf || fft_we_cnt!=lc) begin idle<=0; lf<=frames; lc<=fft_we_cnt; end
    else idle<=idle+1;
    if (idle>4000) begin
      $display("RESULT QSZ=%0d WAIT1=%0d WAIT2=%0d BUBBLE=%0d POSTPAD=%s : HUNG frames=%0d maxfill=%0d/%0d drops=%0d ovf_after_pad=%0d",
        QSZ,WAIT1,WAIT2,BUBBLE,`ifdef NOPOSTPAD "0" `else "1" `endif, frames, maxfill, DEPTH, drops, ovf_after_pad);
      $finish;
    end
  end

  initial begin
    rstn=0; repeat(40) @(posedge clk); rstn=1;
    wait(frames>=NTRIG);
    $display("RESULT QSZ=%0d WAIT1=%0d WAIT2=%0d BUBBLE=%0d POSTPAD=%s : OK frames=%0d maxfill=%0d/%0d drops=%0d ovf_after_pad=%0d",
      QSZ,WAIT1,WAIT2,BUBBLE,`ifdef NOPOSTPAD "0" `else "1" `endif, frames, maxfill, DEPTH, drops, ovf_after_pad);
    $finish;
  end
  initial begin #6000000; $display("RESULT TIMEOUT frames=%0d maxfill=%0d drops=%0d ovf_after_pad=%0d", frames, maxfill, drops, ovf_after_pad); $finish; end
endmodule
