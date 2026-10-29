/**
 * ZCU216 oscilloscope, Red Pitaya scope semantics (red_pitaya_scope.sv, pyrpl
 * Scope) with the buffer in DDR: two channels of 8 samples per clock (14-bit
 * values in 16 bits, 2.5 GS/s at fclk = 312.5 MHz), decimated by D = 2^k
 * (k = 1..16; with average, the mean of each group, else its first sample),
 * written as a ring of R = 2L samples per channel at BASE through two DDR ports
 * of its own (S_AXI_HP3 and S_AXI_HP2, 4 KB blocks alternating, as the frame
 * snapshots do on HP0/HP1), so it runs alongside the snapshots and the point
 * stream (HPC0). Each 128-bit word holds four consecutive samples of each
 * channel: [63:0] channel 1, [127:64] channel 2, sample 0 in the low 16 bits.
 * D = 2 is one word per clock (5 GB/s), half of each port; ovf says a port fell
 * behind and words were lost.
 *
 * As on the Red Pitaya: arm starts writing; once L - dly samples are in (or
 * dly >= L), a trigger (software, channel 1/2 crossing thr upwards or
 * downwards with hysteresis, or an edge of the external input ext: sources
 * 6 rising / 7 falling, as on the Red Pitaya) is taken; dly more samples are written, then
 * writing stops (keep: continues). wp_trig is the sample before the trigger,
 * so the trace is the L samples ending at wp_trig + dly. trig_src returns to 0
 * when the trigger and the delay are done, and writing stays up until the last
 * word is in DDR, so "writing and dly_do both low" means the trace is readable.
 * The ring is 2L: the words written past wp_trig + dly (at most 3 samples)
 * never reach into the trace. wp_cur is the last sample in DDR: in a free run
 * (no trigger, rolling mode) the PC reads the L samples ending there; slow
 * rates flush partial blocks so it keeps moving.
 */
module zcu216_scope #(
  parameter int          L    = 1 << 18,       // samples per channel per trace
  parameter logic [39:0] BASE = 40'h1FE0_0000, // the ring: 8L bytes
  parameter int          FD   = 10,            // writer FIFO depth, log2 words per port
  localparam int         RW   = $clog2(2 * L)  // ring index bits (samples)
) (
  input  logic          clk,
  input  logic          rstn,
  input  logic [127:0]  x1,                    // 8 samples, sample 0 in [15:0]
  input  logic [127:0]  x2,
  // settings (quasi-static) and commands (toggles)
  input  logic [16:0]   dec,
  input  logic          avg,
  input  logic signed [15:0] thr,
  input  logic [15:0]   hyst,
  input  logic [31:0]   dly,
  input  logic          keep,
  input  logic [3:0]    src_val,               // taken on a src_set toggle
  input  logic          src_set,
  input  logic          arm,
  input  logic          rst,
  input  logic          ext,                   // external trigger (asynchronous)
  // status
  output logic          writing,
  output logic          dly_do,
  output logic [3:0]    trig_src,
  output logic          pretrig_ok,
  output logic [RW-1:0] wp_cur,
  output logic [RW-1:0] wp_trig,
  output logic          arm_ack,               // follows arm once the arm is taken
  output logic          rst_ack,
  output logic          ovf,                   // words lost since the arm (DDR too slow)
  output logic          err,                   // a write response was not OKAY
  // two AXI4 write masters, 128 bits
  output logic [48:0]   m_awaddr  [2],
  output logic [7:0]    m_awlen   [2],
  output logic [2:0]    m_awsize  [2],
  output logic [1:0]    m_awburst [2],
  output logic [3:0]    m_awcache [2],
  output logic [2:0]    m_awprot  [2],
  output logic [1:0]    m_awvalid,
  input  logic [1:0]    m_awready,
  output logic [127:0]  m_wdata   [2],
  output logic [15:0]   m_wstrb   [2],
  output logic [1:0]    m_wlast,
  output logic [1:0]    m_wvalid,
  input  logic [1:0]    m_wready,
  input  logic [1:0]    m_bresp   [2],
  input  logic [1:0]    m_bvalid,
  output logic [1:0]    m_bready
);

localparam int LB = 8, BLK = 1 << LB;           // 4 KB blocks, alternating ports

logic         sc_start, sc_push, sc_flush, sc_idle;
logic [127:0] sc_data;
logic [31:0]  sc_words;                        // words in DDR since the start, no gap
logic [31:0]  wcnt;                            // words pushed since the start
logic [31:0]  acked [2], gnext [2];
logic [1:0]   w_idle, w_ovf, w_err;

always_ff @(posedge clk)
  if (sc_start) wcnt <= '0;
  else if (sc_push) wcnt <= wcnt + 1'b1;

for (genvar p = 0; p < 2; p++) begin : g_wr
  zcu216_snap_wr #(.FD(FD), .LB(LB), .STEP(2)) i_wr (
    .clk, .rstn, .start (sc_start), .a0 (BASE + 40'(p * BLK * 16)),
    .a_lim (BASE + 40'(8 * L)), .flush (sc_flush),
    .push (sc_push && wcnt[LB] == 1'(p)), .din (sc_data),
    .ovf (w_ovf[p]), .err (w_err[p]), .idle (w_idle[p]), .acked (acked[p]),
    .awaddr (m_awaddr[p]), .awlen (m_awlen[p]), .awsize (m_awsize[p]),
    .awburst (m_awburst[p]), .awcache (m_awcache[p]), .awprot (m_awprot[p]),
    .awvalid (m_awvalid[p]), .awready (m_awready[p]),
    .wdata (m_wdata[p]), .wstrb (m_wstrb[p]), .wlast (m_wlast[p]),
    .wvalid (m_wvalid[p]), .wready (m_wready[p]),
    .bresp (m_bresp[p]), .bvalid (m_bvalid[p]), .bready (m_bready[p]));
  // the first word of this port not yet acknowledged, as a ring word index
  // (port p holds blocks p, p + 2, ...)
  assign gnext[p] = ((acked[p] >> LB) << (LB + 1)) + 32'(p * BLK) + (acked[p] & 32'(BLK - 1));
end

always_ff @(posedge clk) sc_words <= (gnext[0] < gnext[1]) ? gnext[0] : gnext[1];
assign sc_idle = &w_idle;
assign ovf     = |w_ovf;
assign err     = |w_err;

// decimation: k = log2 D, 1..16
logic [4:0] k;
always_ff @(posedge clk) begin
  k <= 5'd1;
  for (int i = 2; i <= 16; i++) if (dec[i]) k <= 5'(i);
end

logic signed [15:0] thm, thp;                  // thr - hyst, thr + hyst
always_ff @(posedge clk) begin
  thm <= thr - $signed(hyst);
  thp <= thr + $signed(hyst);
end

// commands
logic [2:0] arm_s, rst_s, set_s;
always_ff @(posedge clk) begin
  arm_s <= {arm_s[1:0], arm};
  rst_s <= {rst_s[1:0], rst};
  set_s <= {set_s[1:0], src_set};
end
wire c_arm = arm_s[2] != arm_s[1];
wire c_rst = rst_s[2] != rst_s[1];
wire c_set = set_s[2] != set_s[1];

////////////////////////////////////////////////////////////////////////////////
// decimation: sums of 2, 4 and 8 samples per beat, longer groups over beats
////////////////////////////////////////////////////////////////////////////////

logic signed [15:0] x  [2][8], xs2 [2][8], xs3 [2][8];
logic signed [16:0] p1 [2][4], p1s3 [2][4];
logic signed [17:0] p2 [2][2], p2s3 [2][2];
logic signed [18:0] p3 [2];
logic               restart;                   // decimation groups start over (arm)
logic [3:0]         rs;

always_ff @(posedge clk) begin
  for (int j = 0; j < 8; j++) begin
    x[0][j] <= x1[16*j +: 16];
    x[1][j] <= x2[16*j +: 16];
  end
  for (int c = 0; c < 2; c++) begin
    for (int i = 0; i < 4; i++) p1[c][i] <= x[c][2*i] + x[c][2*i+1];
    xs2[c] <= x[c];
    for (int i = 0; i < 2; i++) p2[c][i] <= p1[c][2*i] + p1[c][2*i+1];
    p1s3[c] <= p1[c];
    xs3[c]  <= xs2[c];
    p3[c]   <= p2[c][0] + p2[c][1];
    p2s3[c] <= p2[c];
  end
  rs <= {rs[2:0], restart};
end

// stage 4: up to four output samples per channel, o[c][0..n-1]
logic signed [15:0] o [2][4];
logic [2:0]         n;
logic signed [34:0] acc [2], asum [2];
logic signed [15:0] first [2];
logic [12:0]        gcnt;                      // beats into a group (k >= 4)

always_comb
  for (int c = 0; c < 2; c++)
    asum[c] = (gcnt == 0) ? 35'(p3[c]) : acc[c] + 35'(p3[c]);

always_ff @(posedge clk) begin
  n <= '0;
  case (k)
    5'd1: begin
      n <= 3'd4;
      for (int c = 0; c < 2; c++)
        for (int i = 0; i < 4; i++)
          o[c][i] <= avg ? 16'(p1s3[c][i] >>> 1) : xs3[c][2*i];
    end
    5'd2: begin
      n <= 3'd2;
      for (int c = 0; c < 2; c++)
        for (int i = 0; i < 2; i++)
          o[c][i] <= avg ? 16'(p2s3[c][i] >>> 2) : xs3[c][4*i];
    end
    5'd3: begin
      n <= 3'd1;
      for (int c = 0; c < 2; c++) o[c][0] <= avg ? 16'(p3[c] >>> 3) : xs3[c][0];
    end
    default: begin
      for (int c = 0; c < 2; c++) begin
        acc[c] <= asum[c];
        if (gcnt == 0) first[c] <= xs3[c][0];
        o[c][0] <= avg ? 16'(asum[c] >>> k) : ((gcnt == 0) ? xs3[c][0] : first[c]);
      end
      if (gcnt == 13'((1 << (k - 3)) - 1)) begin
        n    <= 3'd1;
        gcnt <= '0;
      end else
        gcnt <= gcnt + 1'b1;
    end
  endcase
  if (rs[3]) gcnt <= '0;
end

////////////////////////////////////////////////////////////////////////////////
// trigger: Schmitt triggers on both channels over the samples of a beat
////////////////////////////////////////////////////////////////////////////////

logic signed [15:0] o5 [2][4];
logic [2:0]         n5;
logic [3:0]         ge [2], lt [2], le [2], gt [2];

always_ff @(posedge clk) begin
  o5 <= o;
  n5 <= n;
  for (int c = 0; c < 2; c++)
    for (int i = 0; i < 4; i++) begin
      ge[c][i] <= o[c][i] >= thr;
      lt[c][i] <= o[c][i] <  thm;
      le[c][i] <= o[c][i] <= thr;
      gt[c][i] <= o[c][i] >  thp;
    end
end

// the state before / after each sample and the edges (the state switching on)
logic [1:0] sp, sn;                            // per channel: above / below
logic [4:0] ca [2], cb [2];                    // [i] = state before sample i
logic [3:0] ep [2], en [2];                    // edge at sample i
logic [1:0] sp_n, sn_n;

always_comb
  for (int c = 0; c < 2; c++) begin
    ca[c][0] = sp[c];
    cb[c][0] = sn[c];
    for (int i = 0; i < 4; i++) begin
      if (3'(i) < n5) begin
        ca[c][i+1] = ge[c][i] || (ca[c][i] && !lt[c][i]);
        cb[c][i+1] = le[c][i] || (cb[c][i] && !gt[c][i]);
      end else begin
        ca[c][i+1] = ca[c][i];
        cb[c][i+1] = cb[c][i];
      end
      ep[c][i] = ca[c][i+1] && !ca[c][i];
      en[c][i] = cb[c][i+1] && !cb[c][i];
    end
    sp_n[c] = ca[c][4];
    sn_n[c] = cb[c][4];
  end

always_ff @(posedge clk)
  if (!rstn) begin
    sp <= '0;
    sn <= '0;
  end else begin
    sp <= sp_n;
    sn <= sn_n;
  end

// external input: synchronised; an edge waits for the next beat that carries
// samples and lands on its first one (beat resolution, 8 input samples)
(* ASYNC_REG = "TRUE" *) logic [1:0] ext_m;
logic       ext_d, xr_p, xf_p;                 // rising / falling edge pending
always_ff @(posedge clk) begin
  ext_m <= {ext_m[0], ext};
  ext_d <= ext_m[1];
  if (!rstn) begin
    xr_p <= 1'b0;
    xf_p <= 1'b0;
  end else if (n5 != 0) begin
    xr_p <= ext_m[1] && !ext_d;
    xf_p <= !ext_m[1] && ext_d;
  end else begin
    xr_p <= xr_p || (ext_m[1] && !ext_d);
    xf_p <= xf_p || (!ext_m[1] && ext_d);
  end
end

// edges of the selected source this beat
logic [3:0] ev;
always_comb
  case (trig_src)
    4'd1:    ev = (n5 != 0) ? 4'b0001 : 4'b0000;  // software / immediately
    4'd2:    ev = ep[0];
    4'd3:    ev = en[0];
    4'd4:    ev = ep[1];
    4'd5:    ev = en[1];
    4'd6:    ev = (n5 != 0 && xr_p) ? 4'b0001 : 4'b0000;
    4'd7:    ev = (n5 != 0 && xf_p) ? 4'b0001 : 4'b0000;
    default: ev = '0;
  endcase

logic [1:0] epos;                              // the first edge
always_comb begin
  epos = 2'd0;
  for (int i = 3; i >= 0; i--) if (ev[i]) epos = 2'(i);
end

////////////////////////////////////////////////////////////////////////////////
// acquisition
////////////////////////////////////////////////////////////////////////////////

typedef enum logic [1:0] {S_IDLE, S_START, S_RUN, S_FLUSH} state_t;
state_t st;

logic [31:0] scnt;                             // samples since the start
logic [31:0] stop_at;                          // the last sample to keep
logic        triggered, arm_pend;
logic [63:0] pk [2];                           // the word being filled
logic [1:0]  fill;                             // samples in it (multiple of n)
logic [13:0] tick = '0;                        // partial-block flushes at slow rates

wire [31:0] s_next = scnt + 32'(n5);
wire        word   = (st == S_RUN) && n5 != 0 && 3'(fill) + n5 == 3'd4;

// both checks a cycle behind scnt (registered, off the state logic's path): a
// trigger may wait one beat longer, and up to a word more is written after the
// last sample, which the ring (2L) has room for
logic past_stop;                               // scnt is beyond the last sample to keep
logic trig_d;                                  // triggered a cycle ago: past_stop uses this stop_at
always_ff @(posedge clk) begin
  pretrig_ok <= (dly >= 32'(L)) || (scnt >= 32'(L) - dly);
  past_stop  <= scnt > stop_at;
  trig_d     <= triggered;
end
assign writing    = (st != S_IDLE) || arm_pend;

always_ff @(posedge clk) begin
  sc_start <= 1'b0;
  sc_push  <= 1'b0;
  restart  <= 1'b0;
  tick     <= tick + 1'b1;
  if (!rstn) begin
    st        <= S_IDLE;
    triggered <= 1'b0;
    dly_do    <= 1'b0;
    arm_pend  <= 1'b0;
    trig_src  <= '0;
    wp_trig   <= '0;
    arm_ack   <= 1'b0;
    rst_ack   <= 1'b0;
    sc_flush  <= 1'b0;
  end else begin
    if (c_set) trig_src <= src_val;
    // reset: stop (the writers drain first), back to 0
    if (c_rst) begin
      rst_ack   <= !rst_ack;
      trig_src  <= '0;
      dly_do    <= 1'b0;
      triggered <= 1'b0;
      wp_trig   <= '0;
      if (st == S_RUN || st == S_START) st <= (st == S_RUN) ? S_FLUSH : S_IDLE;
    end
    // arm: start over, after the writers have drained if they are busy
    if (c_arm) begin
      arm_ack  <= !arm_ack;
      arm_pend <= 1'b1;
      dly_do   <= 1'b0;
      if (st == S_RUN) st <= S_FLUSH;
    end
    case (st)
      S_IDLE: if (arm_pend && !c_rst) st <= S_START;
      S_START: begin
          st        <= S_RUN;
          arm_pend  <= 1'b0;
          sc_start  <= 1'b1;
          restart   <= 1'b1;
          triggered <= 1'b0;
          scnt      <= '0;
          fill      <= '0;
          sc_flush  <= 1'b0;
        end
      S_RUN: if (!c_arm && !c_rst) begin
        // trigger
        if (!triggered && |ev && pretrig_ok) begin
          triggered <= 1'b1;
          dly_do    <= 1'b1;
          wp_trig   <= RW'(scnt + 32'(epos) - 1);
          stop_at   <= scnt + 32'(epos) - 1 + dly;
        end
        // samples into the word
        for (int i = 0; i < 4; i++)
          if (3'(i) < n5) begin
            pk[0][16*(fill+i) +: 16] <= o5[0][i];
            pk[1][16*(fill+i) +: 16] <= o5[1][i];
          end
        fill <= 2'(3'(fill) + n5);
        scnt <= s_next;
        if (word) begin
          sc_push <= 1'b1;
          // done once the word holding the last sample is out
          if (triggered && trig_d && past_stop) begin
            dly_do   <= 1'b0;
            trig_src <= '0;
            if (!keep) st <= S_FLUSH;
            else triggered <= 1'b0;
          end
        end
        sc_flush <= (k >= 5'd5) && tick == '1;  // keep a slow free run visible
      end
      S_FLUSH: begin
        sc_flush <= 1'b1;
        // a cycle with flush up first: the last push is in a FIFO by then
        if (sc_flush && sc_idle) begin
          sc_flush <= 1'b0;
          st       <= arm_pend ? S_START : S_IDLE;
        end
      end
      default: st <= S_IDLE;
    endcase
  end
end

// the word: the samples of this beat land in the same cycle as the push
always_comb begin
  sc_data = {pk[1], pk[0]};
end

// the last sample in DDR
always_ff @(posedge clk)
  if (!rstn) wp_cur <= '1;
  else wp_cur <= RW'({sc_words, 2'b00} - 1);

endmodule
