/**
 * Frame snapshots into DDR, read back by the PC like registers.
 *
 * arm (a toggle) captures the next nframes whole frames of source src, from its
 * next frame start, into the reserved memory at BASE: frame f, beat t at
 * BASE + (f * 2^SUB + t) * 16, little-endian, i.e. the same 128-bit beats the
 * source delivers (four 32-bit bins, or eight 16-bit samples). Nothing is
 * written until the PC arms it, and nothing more after the last frame; done
 * toggles once everything is in DDR (the last write response is back), and the
 * PC fetches the frames through monitor_server's memory window, then arms
 * again. The PC sets the pace.
 *
 * The sources stream one beat per clock without stopping (5 GB/s at 312.5 MHz),
 * more than one HP port sustains into DDR, so consecutive blocks of BLK beats
 * (4 KB; half a frame for frames under 512 beats) alternate between two ports
 * (S_AXI_HP0 / HP1, separate DDR controller ports), each behind its own FIFO.
 * overflow says a FIFO filled and beats were lost; err that a write response
 * was not OKAY. An arm while busy is ignored.
 */

module zcu216_snap #(
  parameter int          NSRC = 4,
  parameter int          SUB  = 15,             // 2^SUB beats per frame
  parameter logic [39:0] BASE = 40'h1E10_0000,
  parameter int          SIZE = 31 << 20,       // bytes available from BASE
  parameter int          FD   = 10,             // FIFO depth, log2 beats per port
  localparam int         SRW  = $clog2(NSRC)
) (
  input  logic           clk,
  input  logic           rstn,
  input  logic [127:0]   s_data [NSRC],
  input  logic [NSRC-1:0] s_valid,
  input  logic [NSRC-1:0] s_first,
  // control (quasi-static, arm is a toggle)
  input  logic           arm,
  input  logic [SRW-1:0] src,
  input  logic [7:0]     nframes,               // 0 counts as 1; clamped to what fits
  output logic           busy,
  output logic           done,                  // toggle
  output logic           overflow,
  output logic           err,
  output logic [31:0]    beats,                 // beats captured
  // AXI4 write masters, 128 bits (read channels unused)
  output logic [48:0]    m_awaddr  [2],
  output logic [7:0]     m_awlen   [2],
  output logic [2:0]     m_awsize  [2],
  output logic [1:0]     m_awburst [2],
  output logic [3:0]     m_awcache [2],
  output logic [2:0]     m_awprot  [2],
  output logic [1:0]     m_awvalid,
  input  logic [1:0]     m_awready,
  output logic [127:0]   m_wdata   [2],
  output logic [15:0]    m_wstrb   [2],
  output logic [1:0]     m_wlast,
  output logic [1:0]     m_wvalid,
  input  logic [1:0]     m_wready,
  input  logic [1:0]     m_bresp   [2],
  input  logic [1:0]     m_bvalid,
  output logic [1:0]     m_bready
);

localparam int MAXF = (SIZE >> (SUB + 4)) > 255 ? 255 : (SIZE >> (SUB + 4));
localparam int LB   = (SUB >= 9) ? 8 : SUB - 1;  // log2 BLK
localparam int BLK  = 1 << LB;                   // beats per block (one burst)

// every source registered where it comes from, then the selected one: the
// sources sit across the die and one hop from each to the mux missed timing
logic [127:0]    y_data [NSRC];
logic [NSRC-1:0] y_valid, y_first;
logic [127:0]    x_data;
logic            x_valid, x_first;
logic [SRW-1:0]  csrc;

always_ff @(posedge clk) begin
  y_data  <= s_data;
  y_valid <= s_valid;
  y_first <= s_first & s_valid;
  x_data  <= y_data[csrc];
  x_valid <= y_valid[csrc];
  x_first <= y_first[csrc];
end

logic [2:0]  arm_s;
logic        armed, cap, start;
logic [31:0] total;
logic [1:0]  wr_idle, wr_ovf, wr_err;
logic [1:0]  push;

always_ff @(posedge clk) arm_s <= {arm_s[1:0], arm};

wire go  = x_valid && (cap || (armed && x_first));
wire fin = go && beats == total - 1;

always_ff @(posedge clk)
  if (!rstn) begin
    busy  <= 1'b0;
    armed <= 1'b0;
    cap   <= 1'b0;
    done  <= 1'b0;
    start <= 1'b0;
  end else begin
    start <= 1'b0;
    if (arm_s[2] != arm_s[1] && !busy) begin
      busy  <= 1'b1;
      armed <= 1'b1;
      start <= 1'b1;
      csrc  <= src;
      total <= 32'((nframes == 0) ? 1 : (nframes > MAXF) ? MAXF : nframes) << SUB;
      beats <= '0;
    end
    if (go) begin
      armed <= 1'b0;
      cap   <= !fin;
      beats <= beats + 1'b1;
    end
    // finished: all beats taken, both FIFOs drained, all responses back
    if (busy && !armed && !cap && !start && &wr_idle) begin
      busy <= 1'b0;
      done <= !done;
    end
  end

assign overflow = |wr_ovf;
assign err      = |wr_err;

// blocks alternate between the ports
always_comb begin
  push[0] = go && !beats[LB];
  push[1] = go &&  beats[LB];
end

for (genvar p = 0; p < 2; p++) begin : g_wr
  zcu216_snap_wr #(.FD(FD), .LB(LB), .STEP(2)) i_wr (
    .clk, .rstn, .start, .a0 (BASE + 40'(p * BLK * 16)), .a_lim ('1),
    .flush (busy && !armed && !cap),
    .push (push[p]), .din (x_data),
    .ovf (wr_ovf[p]), .err (wr_err[p]), .idle (wr_idle[p]), .acked (),
    .awaddr (m_awaddr[p]), .awlen (m_awlen[p]), .awsize (m_awsize[p]),
    .awburst (m_awburst[p]), .awcache (m_awcache[p]), .awprot (m_awprot[p]),
    .awvalid (m_awvalid[p]), .awready (m_awready[p]),
    .wdata (m_wdata[p]), .wstrb (m_wstrb[p]), .wlast (m_wlast[p]),
    .wvalid (m_wvalid[p]), .wready (m_wready[p]),
    .bresp (m_bresp[p]), .bvalid (m_bvalid[p]), .bready (m_bready[p]));
end

endmodule


/**
 * One snapshot port: a FIFO of 2^FD beats and an AXI4 writer that sends it out
 * in bursts of 2^LB beats to every STEP-th block (2: two ports alternating
 * blocks): a0, a0 + STEP blocks, ..., back to a0 at a_lim. flush also sends what there is of a block; the rest of that
 * block follows in later bursts. acked counts the beats whose write response is
 * back (since start).
 */
module zcu216_snap_wr #(
  parameter int FD = 10,
  parameter int LB = 8,                       // log2 beats per burst (<= 8: 4 KB)
  parameter int STEP = 2                      // blocks from one block to the next
) (
  input  logic         clk,
  input  logic         rstn,
  input  logic         start,                 // new capture: address back to a0
  input  logic [39:0]  a0,
  input  logic [39:0]  a_lim,                 // the next block from here on: back to a0
  input  logic         flush,                 // send a partial block too
  input  logic         push,
  input  logic [127:0] din,
  output logic         ovf,                   // a beat met a full FIFO (since start)
  output logic         err,                   // a response was not OKAY (since start)
  output logic         idle,                  // FIFO empty, nothing in flight
  output logic [31:0]  acked,                 // beats written (responses back)
  output logic [48:0]  awaddr,
  output logic [7:0]   awlen,
  output logic [2:0]   awsize,
  output logic [1:0]   awburst,
  output logic [3:0]   awcache,
  output logic [2:0]   awprot,
  output logic         awvalid,
  input  logic         awready,
  output logic [127:0] wdata,
  output logic [15:0]  wstrb,
  output logic         wlast,
  output logic         wvalid,
  input  logic         wready,
  input  logic [1:0]   bresp,
  input  logic         bvalid,
  output logic         bready
);

assign awsize  = 3'b100;                      // 16 bytes
assign awburst = 2'b01;                       // INCR
assign awcache = 4'b0011;
assign awprot  = 3'b000;
assign wstrb   = '1;
assign bready  = 1'b1;

// FIFO, first word fall-through: wdata is its head when hv
(* ram_style = "block" *) logic [127:0] mem [2**FD];
logic [FD:0] wp, rp;
logic [FD:0] used;                            // in mem, not counting the head
logic        hv, pop;
logic        inw;                             // sending a burst's beats

assign used   = wp - rp;
assign wvalid = hv && inw;                    // data only inside a burst
assign pop    = wvalid && wready;

always_ff @(posedge clk) begin
  if (push && used != (FD+1)'(2**FD)) mem[wp[FD-1:0]] <= din;
  if ((!hv || pop) && used != 0) wdata <= mem[rp[FD-1:0]];
end

always_ff @(posedge clk)
  if (!rstn || start) begin
    wp  <= '0;
    rp  <= '0;
    hv  <= 1'b0;
    ovf <= 1'b0;
  end else begin
    if (push) begin
      if (used != (FD+1)'(2**FD)) wp <= wp + 1'b1;
      else ovf <= 1'b1;
    end
    if ((!hv || pop) && used != 0) begin
      rp <= rp + 1'b1;
      hv <= 1'b1;
    end else if (pop)
      hv <= 1'b0;
  end

// bursts: the rest of the current block once the FIFO holds it (a whole block
// normally), or what there is when flushing; the next block's address once a
// block is complete
logic [FD:0] avail, room;
logic [7:0]  cnt;
logic [4:0]  outst;                           // bursts awaiting their response
logic [39:0] addr, next;
logic [LB:0] boff;                            // beats of the current block already sent
logic [8:0]  blen;                            // the pending burst's length
logic [8:0]  lq [16];                         // lengths of the bursts in flight
logic [3:0]  lq_wr, lq_rd;

assign avail = used + (FD+1)'(hv);
assign room  = (FD+1)'(2**LB) - (FD+1)'(boff);
assign wlast = (cnt == awlen);
assign next  = addr + 40'(STEP * 2**LB * 16); // (with STEP 2, the other port's in between)

always_ff @(posedge clk)
  if (!rstn || start) begin
    awvalid <= 1'b0;
    inw     <= 1'b0;
    cnt     <= '0;
    outst   <= '0;
    addr    <= a0;
    boff    <= '0;
    err     <= 1'b0;
    acked   <= '0;
    lq_wr   <= '0;
    lq_rd   <= '0;
  end else begin
    if (!awvalid && !inw && (avail >= room || (flush && avail != 0)) && outst < 5'd16) begin
      awvalid <= 1'b1;
      awaddr  <= 49'(addr + 40'({boff, 4'b0}));
      awlen   <= (avail >= room) ? 8'(room - 1'b1) : 8'(avail - 1'b1);
      blen    <= (avail >= room) ? 9'(room) : 9'(avail);
    end
    if (awvalid && awready) begin
      awvalid <= 1'b0;
      inw     <= 1'b1;
      lq[lq_wr] <= blen;
      lq_wr   <= lq_wr + 1'b1;
      if (9'(boff) + blen == 9'(2**LB)) begin
        boff <= '0;
        addr <= (next >= a_lim) ? a0 : next;
      end else
        boff <= boff + (LB+1)'(blen);
    end
    if (pop) begin
      cnt <= wlast ? 8'd0 : cnt + 1'b1;
      if (wlast) inw <= 1'b0;
    end
    outst <= outst + (awvalid && awready) - bvalid;
    if (bvalid) begin
      acked <= acked + 32'(lq[lq_rd]);
      lq_rd <= lq_rd + 1'b1;
      if (bresp != 2'b00) err <= 1'b1;
    end
  end

assign idle = !awvalid && !inw && avail == 0 && outst == 0;

endmodule
