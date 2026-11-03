/**
 * The FFT span below the RF-ADC rate, for one zcu216_fft_dit8 engine: its input
 * decimated by D = 2^k (k = 1..KMAX), each output the mean of D consecutive
 * samples (the Red Pitaya's averaging scope decimation, which its FFT took), so
 * the 2^(SUB+3)-point transform covers D times the time at 1/D the rate.
 *
 * The engine's cores run in realtime mode: inside a frame they take a beat every
 * clock, so a decimated stream (8 samples every D beats) cannot go in as it
 * comes. Each window is captured into a buffer of one frame (2^SUB words of 8
 * samples) and played into the engine at a beat per clock, then zeros up to the
 * frame's 2^SUB beats, as zcu216_fft_frame presents a window at the full rate.
 *
 * Windows. Framed (en): a sync arms a countdown of OFF beats (as
 * zcu216_fft_frame), then ACQ ADC samples are captured (ACQ / D decimated
 * samples, whole words, at most a frame). Free-running: whole frames, back to
 * back. A window due while the previous capture is still going is skipped, and
 * so is a capture complete while the engine still plays the previous one (skip).
 * Playback reads a word per clock and the next capture writes one every D
 * clocks, starting after the playback does, so one buffer suffices.
 *
 * Like zcu216_fft_frame, the beat to present NEXT clock is decided here
 * (valid_d, data_d: the caller registers them) and the beats the engine ACCEPTED
 * are counted (acc), so a frame is always 2^SUB accepted beats; a beat the engine
 * drops (its cores' first-frame config) is made up by the next one. bnd: no
 * frame going or starting, so the engine is at a frame boundary.
 *
 * Latency: the decimator 7 clocks, the buffer read 4.
 */
module zcu216_fft_dec #(
  parameter int SUB  = 13,                        // 2^SUB beats per frame
  parameter int CW   = 21,                        // countdown bits (beats)
  parameter int KMAX = 8                          // D up to 2^KMAX
) (
  input  logic          clk,
  input  logic          rstn,
  input  logic [3:0]    k,                        // log2 D, 1..KMAX (quasi-static)
  input  logic          en,                       // framed (else back-to-back frames)
  input  logic          sync,                     // one clock per chirp sync
  input  logic [CW-1:0] off,                      // sync -> window start (beats)
  input  logic [23:0]   acq,                      // window (ADC samples)
  input  logic [127:0]  din,                      // 8 lanes of 16-bit signed, lane 0 oldest
  input  logic          en_play,                  // frames may start
  input  logic          acc,                      // the beat presented now was accepted
  output logic [127:0]  data_d,                   // the beat to present next clock
  output logic          valid_d,
  output logic          bnd,                      // the engine at a frame boundary
  output logic          skip                      // a window or a frame skipped
);

localparam int NW = 1 << SUB;                     // words (beats) per frame
localparam int AW = 16 + KMAX + 1;                // a sum of up to 2^KMAX samples, rounded
localparam int L  = 4;                            // buffer read latency (URAM cascade)

////////////////////////////////////////////////////////////////////////////////
// windows (input side)
////////////////////////////////////////////////////////////////////////////////

logic [CW-1:0] cd;
logic          pend, cap, start, done;
logic [3:0]    k_q;
logic          koff;                              // k = 0, or it changed: no capture

// a capture under another k would never fill its words (the packing assumes
// its samples per beat): it is dropped, and none starts at k = 0
assign koff = k == '0 || k != k_q;
always_ff @(posedge clk) k_q <= k;
logic [SUB:0]  words, words_n;
logic [23:0]   acq_w;

assign acq_w   = acq >> (5'(k) + 5'd3);           // whole words of decimated samples
assign words_n = (acq_w >= 24'(NW)) ? (SUB+1)'(NW) : (SUB+1)'(acq_w);

wire go = pend && cd == '0;
assign start = rstn && !cap && !koff && (en ? (go && words_n != '0) : 1'b1);

always_ff @(posedge clk)
  if (!rstn) begin
    pend  <= 1'b0;
    cd    <= '0;
    cap   <= 1'b0;
    words <= '0;
  end else begin
    if (sync && en) begin
      pend <= 1'b1;
      cd   <= off;
    end else if (pend) begin
      if (cd == '0) pend <= 1'b0;
      else          cd   <= cd - 1'b1;
    end
    if (start) begin
      cap   <= 1'b1;
      words <= en ? words_n : (SUB+1)'(NW);
    end else if (done || koff)
      cap <= 1'b0;
  end

////////////////////////////////////////////////////////////////////////////////
// decimation: sums of 2, 4 and 8 samples per beat, longer groups over beats
// (as zcu216_scope); a window's start restarts the groups
////////////////////////////////////////////////////////////////////////////////

logic signed [15:0]   x  [8];
logic signed [16:0]   p1 [4], p1d [4], p1dd [4];
logic signed [17:0]   p2 [2], p2d [2];
logic signed [18:0]   p3;
logic                 st1 = 0, st2 = 0, st3 = 0, st4 = 0, st5 = 0, st6 = 0;

always_ff @(posedge clk) begin
  for (int j = 0; j < 8; j++) x[j] <= din[16*j +: 16];
  for (int i = 0; i < 4; i++) p1[i] <= x[2*i] + x[2*i+1];
  for (int i = 0; i < 2; i++) p2[i] <= p1[2*i] + p1[2*i+1];
  p3   <= p2[0] + p2[1];
  p1d  <= p1;
  p1dd <= p1d;
  p2d  <= p2;
  {st6, st5, st4, st3, st2, st1} <= {st5, st4, st3, st2, st1, start};
end

// stage 5: up to four sums of D samples, n of them
logic signed [AW-1:0] sm [4], gacc, gsum;
logic [2:0]           n5 = '0, n6 = '0;
logic [KMAX-4:0]      gcnt = '0;                  // beats into a group (k >= 4)
logic [KMAX-4:0]      glast;

assign glast = (KMAX-3)'((1 << (k - 3)) - 1);
always_comb gsum = (st4 || gcnt == '0) ? AW'(p3) : gacc + AW'(p3);

always_ff @(posedge clk) begin
  n5 <= '0;
  case (k)
    4'd1: begin
      n5 <= 3'd4;
      for (int i = 0; i < 4; i++) sm[i] <= AW'(p1dd[i]);
    end
    4'd2: begin
      n5 <= 3'd2;
      for (int i = 0; i < 2; i++) sm[i] <= AW'(p2d[i]);
    end
    4'd3: begin
      n5 <= 3'd1;
      sm[0] <= AW'(p3);
    end
    default: begin
      gacc  <= gsum;
      sm[0] <= gsum;
      if ((st4 ? '0 : gcnt) == glast) begin
        n5   <= 3'd1;
        gcnt <= '0;
      end else
        gcnt <= (st4 ? '0 : gcnt) + 1'b1;
    end
  endcase
end

// stage 6: the means, rounded half up
logic signed [15:0]   o [4];
logic signed [AW:0]   rnd;

assign rnd = (AW+1)'(1) <<< (k - 1);

always_ff @(posedge clk) begin
  n6 <= n5;
  for (int i = 0; i < 4; i++) o[i] <= 16'(((AW+1)'(sm[i]) + rnd) >>> k);
end

// stage 7: eight samples per word
logic signed [15:0]   wb [8], wn [8];
logic [2:0]           pos = '0, pp;
logic                 full, fp, wv, wf;
logic [127:0]         ww;

always_comb begin
  pp = st6 ? 3'd0 : pos;
  wn = wb;
  for (int i = 0; i < 4; i++)
    if (3'(i) < n6) wn[3'(pp + 3'(i))] = o[i];
  full = n6 != '0 && 4'(pp) + 4'(n6) == 4'd8;
end

always_ff @(posedge clk) begin
  wb <= wn;
  wv <= full;
  if (full) begin
    for (int j = 0; j < 8; j++) ww[16*j +: 16] <= wn[j];
    wf <= st6 || fp;
  end
  if (st6 || n6 != '0) pos <= full ? 3'd0 : 3'(pp + n6);
  if (!rstn || full) fp <= 1'b0;
  else if (st6)      fp <= 1'b1;
end

////////////////////////////////////////////////////////////////////////////////
// the buffer: a capture's words, from its first
////////////////////////////////////////////////////////////////////////////////

(* ram_style = "ultra" *) logic [127:0] mem [NW];
logic               we, capo;
logic [SUB-1:0]     wa;
logic [127:0]       wd;
logic [SUB:0]       wcnt;

always_ff @(posedge clk)
  if (!rstn) begin
    we   <= 1'b0;
    capo <= 1'b0;
    wcnt <= '0;
    done <= 1'b0;
  end else begin
    we   <= 1'b0;
    done <= 1'b0;
    if (koff)
      capo <= 1'b0;
    else if (wv && (wf || capo)) begin
      we   <= 1'b1;
      wa   <= wf ? '0 : wcnt[SUB-1:0];
      wd   <= ww;
      wcnt <= wf ? (SUB+1)'(1) : wcnt + 1'b1;
      if ((wf ? (SUB+1)'(1) : wcnt + 1'b1) == words) begin
        capo <= 1'b0;
        done <= 1'b1;
      end else
        capo <= 1'b1;
    end
  end

always_ff @(posedge clk)
  if (we) mem[wa] <= wd;

////////////////////////////////////////////////////////////////////////////////
// playback: a word per clock, then zeros, 2^SUB accepted beats in all
////////////////////////////////////////////////////////////////////////////////

logic               playing;
logic [SUB+1:0]     ri;                           // words issued (past a frame on drops)
logic [SUB:0]       kc, pw;
logic [L-1:0]       pv, pz;
logic [127:0]       q [L];
logic [SUB:0]       k_acc;
logic               pstart;

assign k_acc   = kc + (SUB+1)'(acc);
assign pstart  = done && en_play && !playing;
assign valid_d = playing && pv[L-1] && k_acc != (SUB+1)'(NW);
assign data_d  = pz[L-1] ? 128'h0 : q[L-1];
assign bnd     = !playing && !pstart;

always_ff @(posedge clk) begin
  q[0] <= mem[ri[SUB-1:0]];
  for (int i = 1; i < L; i++) q[i] <= q[i-1];
end

always_ff @(posedge clk)
  if (!rstn) begin
    playing <= 1'b0;
    ri      <= '0;
    kc      <= '0;
    pw      <= '0;
    pv      <= '0;
    pz      <= '0;
    skip    <= 1'b0;
  end else begin
    pv <= {pv[L-2:0], playing};
    pz <= {pz[L-2:0], ri >= (SUB+2)'(pw)};
    if (pstart) begin
      playing <= 1'b1;
      ri      <= '0;
      kc      <= '0;
      pw      <= words;
    end else if (playing) begin
      ri <= ri + 1'b1;
      kc <= k_acc;
      if (k_acc == (SUB+1)'(NW)) begin
        playing <= 1'b0;
        pv      <= '0;
      end
    end
    skip <= (go && cap && en && !koff) || (done && playing);
  end

endmodule
