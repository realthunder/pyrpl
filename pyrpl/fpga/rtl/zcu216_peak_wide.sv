/**
 * Peak detector for wide peaks on any smooth baseline, one channel, on the same
 * natural-order magnitudes as zcu216_cfar (bin 4t + j on lane j). CFAR judges one
 * bin against its close neighbours, which suits a sharp peak on a flat floor. A
 * beat smeared over hundreds of bins fills CFAR's whole window, and a detector
 * response tilts the floor by more than the peak's height; this one asks instead
 * whether a stretch of the spectrum stands above the straight line through its
 * two sides, in dB, so any slope or slow curve cancels:
 *
 *   sub-block   the mean of 2^(b-2) bins (four per cell)
 *   logarithm   log2, Q5.8: everything after it is 13 bits
 *   cell        the median of five sub-blocks, one per 2^b bins: a spur of a few
 *               bins moves one sub-block, so it does not move the median
 *   score       P - (L + R) / 2, the mean of 2^peak_log cells against the means
 *               of the 2^train_log cells on each side, guard cells away:
 *
 *               | L: 2^t |  G  | P: 2^p |  G  | R: 2^t |    newest cell at the right
 *
 *   result      the frame's highest score with P's centre in start_index ..
 *               end_index, the position refined by a parabola through its two
 *               neighbours; detected when the score exceeds thr (log2 of the
 *               magnitude, Q8.8) and the level of P exceeds data_min.
 *
 * Choose P about the peak's width, G about its skirts, and L and R wider than
 * anything else on the baseline; a curve that turns within the window, or a
 * single-bin peak (lost in the median), is not for this detector.
 * hardware_modules/zcu216_wide_peak.py is the same thing in Python, bit for bit.
 *
 * Cost: one RAMB18 of cell history and no DSP. A frame needs nothing after its
 * last bin but the parabola's division. Settings are taken at each frame's first
 * beat. Same result word as zcu216_cfar: [31:0] the magnitude (the level of P),
 * [32] detected, [32+IW:33] k_interp = bin * 2^FRAC + offset. cell_log >= 5, so a
 * cell takes at least 8 clocks, the time window needs per cell.
 */
module zcu216_peak_wide #(
  parameter int  SUB  = 13,                   // 2^SUB beats of 4 bins per frame
  parameter int  DW   = 32,                   // magnitude bits
  parameter int  FRAC = 8,                    // k_interp fraction bits
  localparam int BW   = SUB + 2,              // bin index
  localparam int SW   = BW + 1,               // start/end index
  localparam int IW   = BW + FRAC
) (
  input  logic           clk,
  input  logic           rstn,
  input  logic [DW-1:0]  n_mag [4],
  input  logic [SUB-1:0] n_t,
  input  logic           n_valid,
  input  logic           n_first,
  input  logic           n_last,
  // settings (quasi-static)
  input  logic [3:0]     cell_log,            // b: 2^b bins per cell, 5 .. 9
  input  logic [2:0]     peak_log,            // cells in P: 2^peak_log, up to 32
  input  logic [5:0]     guard,               // cells each side of P, up to 63
  input  logic [2:0]     train_log,           // cells in L and R: 2^train_log, up to 64
  input  logic [15:0]    thr,                 // Q8.8 log2 of the magnitude
  input  logic [SW-1:0]  start_index,
  input  logic [SW-1:0]  end_index,
  input  logic [DW-1:0]  data_min,
  // one result per frame
  output logic           r_valid,
  output logic [63:0]    r_word
);

localparam int LW = 13;                       // Q5.8 log2
localparam int HW = 9;                        // cell history, 512 cells
localparam int CW = (SUB > 12) ? SUB - 2 : 10; // cell index, and room for the window spans
localparam int YW = LW + 8 + 2;               // score, signed

typedef struct packed {
  logic [3:0]    b;
  logic [2:0]    p;
  logic [5:0]    g;
  logic [2:0]    t;
  logic [15:0]   thr;
  logic [SW-1:0] lo, hi;
  logic [DW-1:0] data_min;
} cfg_t;

cfg_t cfg_in, fa;
always_comb begin
  cfg_in = '{b: cell_log, p: peak_log, g: guard, t: train_log, thr: thr,
             lo: start_index, hi: end_index, data_min: data_min};
  if (cell_log < 4'd5) cfg_in.b = 4'd5;
  if (cell_log > 4'd9) cfg_in.b = 4'd9;
  if (peak_log > 3'd5) cfg_in.p = 3'd5;
  if (train_log > 3'd6) cfg_in.t = 3'd6;
end

function automatic logic [7:0] log_rom(input logic [6:0] m);
  // round(256 * log2(1 + m / 128)), 255 at most
  case (m)
    7'd0: return 8'd0; 7'd1: return 8'd3; 7'd2: return 8'd6; 7'd3: return 8'd9; 7'd4: return 8'd11;
    7'd5: return 8'd14; 7'd6: return 8'd17; 7'd7: return 8'd20; 7'd8: return 8'd22; 7'd9: return 8'd25;
    7'd10: return 8'd28; 7'd11: return 8'd30; 7'd12: return 8'd33; 7'd13: return 8'd36; 7'd14: return 8'd38;
    7'd15: return 8'd41; 7'd16: return 8'd44; 7'd17: return 8'd46; 7'd18: return 8'd49; 7'd19: return 8'd51;
    7'd20: return 8'd54; 7'd21: return 8'd56; 7'd22: return 8'd59; 7'd23: return 8'd61; 7'd24: return 8'd63;
    7'd25: return 8'd66; 7'd26: return 8'd68; 7'd27: return 8'd71; 7'd28: return 8'd73; 7'd29: return 8'd75;
    7'd30: return 8'd78; 7'd31: return 8'd80; 7'd32: return 8'd82; 7'd33: return 8'd85; 7'd34: return 8'd87;
    7'd35: return 8'd89; 7'd36: return 8'd92; 7'd37: return 8'd94; 7'd38: return 8'd96; 7'd39: return 8'd98;
    7'd40: return 8'd100; 7'd41: return 8'd103; 7'd42: return 8'd105; 7'd43: return 8'd107; 7'd44: return 8'd109;
    7'd45: return 8'd111; 7'd46: return 8'd113; 7'd47: return 8'd116; 7'd48: return 8'd118; 7'd49: return 8'd120;
    7'd50: return 8'd122; 7'd51: return 8'd124; 7'd52: return 8'd126; 7'd53: return 8'd128; 7'd54: return 8'd130;
    7'd55: return 8'd132; 7'd56: return 8'd134; 7'd57: return 8'd136; 7'd58: return 8'd138; 7'd59: return 8'd140;
    7'd60: return 8'd142; 7'd61: return 8'd144; 7'd62: return 8'd146; 7'd63: return 8'd148; 7'd64: return 8'd150;
    7'd65: return 8'd152; 7'd66: return 8'd154; 7'd67: return 8'd155; 7'd68: return 8'd157; 7'd69: return 8'd159;
    7'd70: return 8'd161; 7'd71: return 8'd163; 7'd72: return 8'd165; 7'd73: return 8'd167; 7'd74: return 8'd169;
    7'd75: return 8'd170; 7'd76: return 8'd172; 7'd77: return 8'd174; 7'd78: return 8'd176; 7'd79: return 8'd178;
    7'd80: return 8'd179; 7'd81: return 8'd181; 7'd82: return 8'd183; 7'd83: return 8'd185; 7'd84: return 8'd186;
    7'd85: return 8'd188; 7'd86: return 8'd190; 7'd87: return 8'd192; 7'd88: return 8'd193; 7'd89: return 8'd195;
    7'd90: return 8'd197; 7'd91: return 8'd198; 7'd92: return 8'd200; 7'd93: return 8'd202; 7'd94: return 8'd203;
    7'd95: return 8'd205; 7'd96: return 8'd207; 7'd97: return 8'd208; 7'd98: return 8'd210; 7'd99: return 8'd212;
    7'd100: return 8'd213; 7'd101: return 8'd215; 7'd102: return 8'd216; 7'd103: return 8'd218; 7'd104: return 8'd220;
    7'd105: return 8'd221; 7'd106: return 8'd223; 7'd107: return 8'd224; 7'd108: return 8'd226; 7'd109: return 8'd228;
    7'd110: return 8'd229; 7'd111: return 8'd231; 7'd112: return 8'd232; 7'd113: return 8'd234; 7'd114: return 8'd235;
    7'd115: return 8'd237; 7'd116: return 8'd238; 7'd117: return 8'd240; 7'd118: return 8'd241; 7'd119: return 8'd243;
    7'd120: return 8'd244; 7'd121: return 8'd246; 7'd122: return 8'd247; 7'd123: return 8'd249; 7'd124: return 8'd250;
    7'd125: return 8'd252; 7'd126: return 8'd253; default: return 8'd255;
  endcase
endfunction

////////////////////////////////////////////////////////////////////////////////
// sub-blocks: the sum of a beat, then of 2^(b-4) beats
////////////////////////////////////////////////////////////////////////////////

logic [DW:0]     s01, s23;
logic [DW+1:0]   s4;
logic            v1, f1, l1, v2, f2, l2;
logic [DW+6:0]   acc, acc_n;
logic [4:0]      nb, nb_x;                    // beats left in this block, less one
logic [DW-1:0]   sub;
logic            sub_v, sub_f, sub_l;

always_ff @(posedge clk) begin
  if (n_valid && n_first) fa <= cfg_in;
  s01 <= (DW+1)'(n_mag[0]) + (DW+1)'(n_mag[1]);
  s23 <= (DW+1)'(n_mag[2]) + (DW+1)'(n_mag[3]);
  v1  <= rstn && n_valid;
  f1  <= n_first;
  l1  <= n_last;
  s4  <= (DW+2)'(s01) + (DW+2)'(s23);
  v2  <= v1;
  f2  <= f1;
  l2  <= l1;
end

logic [4:0] nb_full;
assign nb_full = 5'((5'd1 << (fa.b - 4'd4)) - 5'd1);

logic blk_first = 1'b0;
logic [DW+6:0] acc_s;
logic [4:0]    nb_s;
logic          e_v, e_f, e_l;
logic [3:0]    e_b;

always_comb begin
  nb_x  = f2 ? nb_full : nb;
  acc_n = (DW+7)'(s4) + (f2 ? '0 : acc);
end

always_ff @(posedge clk) begin
  e_v <= 1'b0;
  if (v2) begin
    if (nb_x == '0) begin
      nb  <= nb_full;
      acc <= '0;
      e_v <= 1'b1;
      e_f <= blk_first || f2;
      e_l <= l2;
      blk_first <= 1'b0;
    end else begin
      nb  <= nb_x - 5'd1;
      acc <= acc_n;
      if (f2) blk_first <= 1'b1;
    end
    acc_s <= acc_n;
  end
  e_b <= fa.b;
  // the mean of the block's 2^(b-2) bins
  sub_v <= e_v;
  sub_f <= e_f;
  sub_l <= e_l;
  sub   <= DW'(acc_s >> (e_b - 4'd2));
end

////////////////////////////////////////////////////////////////////////////////
// log2, Q5.8: normalise in five steps, then the mantissa
////////////////////////////////////////////////////////////////////////////////

logic [DW-1:0] ny [6];
logic [4:0]    ne [6];
logic          nv [6], nf [6], nl [6], nz [6];
logic [LW-1:0] lg;
logic          lg_v, lg_f, lg_l;

always_comb begin
  ny[0] = sub;
  ne[0] = '0;
  nv[0] = sub_v;
  nf[0] = sub_f;
  nl[0] = sub_l;
  nz[0] = sub == '0;
end

for (genvar i = 0; i < 5; i++) begin : g_norm
  localparam int S = 16 >> i;
  logic [DW-1:0] y_q;
  logic [4:0]    e_q;
  logic          v_q, f_q, l_q, z_q;
  always_ff @(posedge clk) begin
    if (ny[i][DW-1 -: S] != '0) begin
      y_q <= ny[i];
      e_q <= ne[i] | 5'(S);
    end else begin
      y_q <= ny[i] << S;
      e_q <= ne[i];
    end
    v_q <= nv[i];
    f_q <= nf[i];
    l_q <= nl[i];
    z_q <= nz[i];
  end
  assign ny[i+1] = y_q;
  assign ne[i+1] = e_q;
  assign nv[i+1] = v_q;
  assign nf[i+1] = f_q;
  assign nl[i+1] = l_q;
  assign nz[i+1] = z_q;
end

always_ff @(posedge clk) begin
  lg   <= nz[5] ? '0 : {ne[5], log_rom(ny[5][DW-2 -: 7])};
  lg_v <= nv[5];
  lg_f <= nf[5];
  lg_l <= nl[5];
end

////////////////////////////////////////////////////////////////////////////////
// cells: the median of five sub-blocks, one cell per four
////////////////////////////////////////////////////////////////////////////////

logic [LW-1:0] w5 [5];                        // [0] the newest
logic [1:0]    j4;                            // sub-block of this frame, mod 4
logic          first_due;                     // the frame's first cell is still to come
logic          m_in, m_first, m_end;

always_ff @(posedge clk) begin
  m_in  <= 1'b0;
  m_end <= lg_v && lg_l;
  if (lg_v) begin
    w5[0] <= lg;
    for (int i = 1; i < 5; i++) w5[i] <= w5[i-1];
    j4 <= lg_f ? 2'd0 : j4 + 2'd1;
    if (lg_f) first_due <= 1'b1;
    else if (j4 == 2'd3) begin                // five sub-blocks in: 0-4, 4-8, ..
      m_in      <= 1'b1;
      m_first   <= first_due;
      first_due <= 1'b0;
    end
  end
end

// median of five, one compare and swap a stage (w5 settled the clock after m_in)
localparam int MS = 7;
localparam int CA [MS] = '{0, 3, 0, 1, 1, 2, 1};
localparam int CB [MS] = '{1, 4, 3, 4, 2, 3, 2};
logic [LW-1:0] ms [MS+1][5];
logic [MS:0]   mv, me, mf;

assign ms[0] = w5;
assign mv[0] = m_in;
assign me[0] = m_end;
assign mf[0] = m_first;

for (genvar i = 0; i < MS; i++) begin : g_med
  logic [LW-1:0] q [5];
  logic v_q, e_q, f_q;
  always_ff @(posedge clk) begin
    q <= ms[i];
    if (ms[i][CA[i]] > ms[i][CB[i]]) begin
      q[CA[i]] <= ms[i][CB[i]];
      q[CB[i]] <= ms[i][CA[i]];
    end
    v_q <= mv[i];
    e_q <= me[i];
    f_q <= mf[i];
  end
  assign ms[i+1] = q;
  assign mv[i+1] = v_q;
  assign me[i+1] = e_q;
  assign mf[i+1] = f_q;
end

wire [LW-1:0] x_cell = ms[MS][2];
wire          x_v    = mv[MS];
wire          x_e    = me[MS];
wire          x_f    = mf[MS];

////////////////////////////////////////////////////////////////////////////////
// the three windows: running sums over a history of cells
////////////////////////////////////////////////////////////////////////////////

(* ram_style = "block" *) logic [LW-1:0] hist [1 << HW];
logic [LW-1:0] rd_d, rq;
logic [HW-1:0] ra;

cfg_t            c;                           // this frame's settings
logic [9:0]      d_ro, d_pi, d_po, d_li, d_lo;// how far back each window edge is
logic [CW-1:0]   ei, en;                      // the newest cell, the next
logic [LW-1:0]   xq;
logic            f_ro, f_pi, f_po, f_li, f_lo;
logic [LW+5:0]   ps;
logic [LW+6:0]   ls, rs;
logic [12:0]     sr;                          // steps after a cell

logic [10:0] t_cells, p_cells;
always_comb begin
  t_cells = 11'd1 << c.t;
  p_cells = 11'd1 << c.p;
end

wire [CW-1:0] ncell = x_f ? '0 : en;
wire          new_frame = x_v && x_f;

always_ff @(posedge clk) begin
  if (new_frame) c <= fa;
  // (these follow c by a clock or two: nothing reads them before sr[0])
  d_ro <= 10'(t_cells);
  d_pi <= 10'(t_cells) + 10'(c.g);
  d_po <= 10'(t_cells) + 10'(c.g) + 10'(p_cells);
  d_li <= 10'(t_cells) + 10'(c.g) + 10'(p_cells) + 10'(c.g);
  d_lo <= 10'(t_cells) + 10'(c.g) + 10'(p_cells) + 10'(c.g) + 10'(t_cells);

  sr <= {sr[11:0], x_v};
  if (!rstn) sr <= '0;
  if (x_v) begin
    hist[HW'(ncell)] <= x_cell;
    xq <= x_cell;
    ei <= ncell;
    en <= ncell + 1'b1;
  end
  rd_d <= hist[ra];
  rq   <= rd_d;

  // the edges of the windows are the cells d_* behind the newest; they exist
  // once that many cells have gone by
  if (sr[0]) begin
    f_ro <= ei >= CW'(d_ro);
    f_pi <= ei >= CW'(d_pi);
    f_po <= ei >= CW'(d_po);
    f_li <= ei >= CW'(d_li);
    f_lo <= ei >= CW'(d_lo);
  end
end

always_ff @(posedge clk) begin
  if (x_v && x_f) begin
    ps <= '0;
    ls <= '0;
  end
  rs <= (x_v && x_f) ? '0 : rs;
  if (sr[0]) rs <= rs + (LW+7)'(xq);
  if (sr[0]) ra <= HW'(ei) - HW'(d_ro);
  if (sr[1]) ra <= HW'(ei) - HW'(d_pi);
  if (sr[2]) ra <= HW'(ei) - HW'(d_po);
  if (sr[3]) ra <= HW'(ei) - HW'(d_li);
  if (sr[4]) ra <= HW'(ei) - HW'(d_lo);
  // a read issued at sr[k] is in rq at sr[k+3], so every sum is done by sr[8],
  // the clock before the next cell can touch rs
  if (sr[3] && f_ro) rs <= rs - (LW+7)'(rq);
  if (sr[4] && f_pi) ps <= ps + (LW+6)'(rq);
  if (sr[5] && f_po) ps <= ps - (LW+6)'(rq);
  if (sr[6] && f_li) ls <= ls + (LW+7)'(rq);
  if (sr[7] && f_lo) ls <= ls - (LW+7)'(rq);
end

////////////////////////////////////////////////////////////////////////////////
// the score, the frame's best
////////////////////////////////////////////////////////////////////////////////

logic [LW+5:0]        pbar, lbar, rbar;       // means, * 2^6
logic [CW-1:0]        es;
logic signed [YW-1:0] y, best, y0, y2, y_prev;
logic [CW-1:0]        as;                     // P's first cell
logic [SW+1:0]        cb, bc;                 // P's centre, in bins
logic                 have, in_lo, in_hi, got, want, y0ok, y2ok, prev_ok;
logic [LW-1:0]        pm;                     // P's level, Q5.8

always_ff @(posedge clk) begin
  if (sr[7]) es <= ei;                        // the next cell may land on sr[8]
  if (sr[8]) begin
    pbar <= (LW+6)'(ps) << (3'd6 - c.p);
    lbar <= (LW+6)'(ls) << (3'd6 - c.t);
    rbar <= (LW+6)'(rs) << (3'd6 - c.t);
  end
  if (sr[9]) begin
    y    <= YW'({1'b0, pbar, 1'b0}) - YW'(lbar) - YW'(rbar);
    as   <= es + 1'b1 - CW'(d_po);
    have <= es + 1'b1 >= CW'(d_lo);
  end
  if (sr[10])
    cb <= ((SW+2)'(as) << c.b) + ((SW+2)'(p_cells) << (c.b - 4'd1))
          + ((SW+2)'(1) << (c.b - 4'd3));
  if (sr[11]) begin
    in_lo <= cb >= (SW+2)'(c.lo);
    in_hi <= cb <= (SW+2)'(c.hi);
  end
  if (x_v && x_f) begin
    got     <= 1'b0;
    want    <= 1'b0;
    y0ok    <= 1'b0;
    y2ok    <= 1'b0;
    prev_ok <= 1'b0;
  end else if (sr[12] && have) begin
    if (in_lo && in_hi && (!got || y > best)) begin
      best <= y;
      bc   <= cb;
      pm   <= pbar[LW+5:6];
      y0   <= y_prev;
      y0ok <= prev_ok;
      y2ok <= 1'b0;
      got  <= 1'b1;
      want <= 1'b1;
    end else if (want) begin
      y2   <= y;
      y2ok <= 1'b1;
      want <= 1'b0;
    end
    y_prev  <= y;
    prev_ok <= 1'b1;
  end
end

////////////////////////////////////////////////////////////////////////////////
// frame end: the parabola, the level, the result
////////////////////////////////////////////////////////////////////////////////

typedef enum logic [2:0] {J_IDLE, J_DEN, J_START, J_DIV, J_MAG, J_OUT} jst_t;

jst_t                 js;
logic                 ended;
logic signed [YW-1:0] jy0, jy1, jy2;
logic                 jgot, jok;
logic [SW+1:0]        jbc;
logic [LW-1:0]        jpm;
logic [3:0]           jb;
logic [15:0]          jthr;
logic [DW-1:0]        jmin;
logic [YW+3:0]        den;
logic [YW+3:0]        rem;
logic                 neg, den_pos;
logic [3:0]           cnt;
logic [7:0]           q;
logic [DW-1:0]        lvl;
logic signed [IW+1:0] ki;
wire  [YW+4:0]        rem2 = {rem, 1'b0};
wire signed [YW+3:0]  den_s = (YW+4)'(2) * ((YW+4)'(2) * (YW+4)'(jy1) - (YW+4)'(jy0) - (YW+4)'(jy2));
wire signed [YW+1:0]  dy    = (YW+2)'(jy2) - (YW+2)'(jy0);

always_ff @(posedge clk) begin
  r_valid <= 1'b0;
  if (x_e) ended <= 1'b1;
  if (!rstn) begin
    js    <= J_IDLE;
    ended <= 1'b0;
  end else case (js)
    J_IDLE:
      if (ended && sr == '0) begin
        ended <= 1'b0;
        jy0   <= y0;
        jy1   <= best;
        jy2   <= y2;
        jgot  <= got;
        jok   <= y0ok && y2ok;
        jbc   <= bc;
        jpm   <= pm;
        jb    <= c.b;
        jthr  <= c.thr;
        jmin  <= c.data_min;
        js    <= J_DEN;
      end
    J_DEN: begin
      den     <= den_s;
      den_pos <= den_s > 0;
      jok     <= jok && jy1 >= jy0 && jy1 >= jy2;
      rem     <= (YW+4)'(dy < 0 ? -dy : dy);
      neg     <= dy < 0;
      q       <= '0;
      cnt     <= '0;
      js      <= J_START;
    end
    J_START:
      js <= (jok && den_pos) ? J_DIV : J_MAG;
    // floor(|y2 - y0| * 2^8 / den)
    J_DIV: begin
      if (rem2 >= (YW+5)'(den)) begin
        rem <= (YW+4)'(rem2 - (YW+5)'(den));
        q   <= {q[6:0], 1'b1};
      end else begin
        rem <= (YW+4)'(rem2);
        q   <= {q[6:0], 1'b0};
      end
      cnt <= cnt + 1'b1;
      if (cnt == 4'd7) js <= J_MAG;
    end
    J_MAG: begin
      lvl <= DW'((40'd256 + 40'(jpm[7:0])) << jpm[LW-1:8] >> 8);
      ki  <= $signed((IW+2)'(jbc) << FRAC)
             + (neg ? -$signed((IW+2)'(q) << jb) : $signed((IW+2)'(q) << jb));
      js  <= J_OUT;
    end
    J_OUT: begin
      r_word  <= !jgot ? '0 :
                 64'({IW'(ki), jy1 > $signed({1'b0, jthr, 7'b0}) && lvl > jmin, lvl});
      r_valid <= 1'b1;
      js      <= J_IDLE;
    end
    default: js <= J_IDLE;
  endcase
end

endmodule
