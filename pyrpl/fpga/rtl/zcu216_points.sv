/**
 * ZCU216 point stream: every frame's CFAR result of every channel, packed into
 * fixed-size packets in the reserved DDR ring that monitor_server forwards over
 * UDP (its dma_poll_thread: ring of RING 64-bit words at BASE, write pointer and
 * data words per packet in two registers).
 *
 * At each frame start of channel 0 (tick) the latest result of every channel
 * since the previous tick goes out, so a packet's frames are consecutive ticks:
 *   header [63] = 1, [62:59] NCH, [58:55] version, [54:0] tick count of its
 *          first frame
 *   data   [63] = 0, [62] a result arrived for this channel in this frame,
 *          [57:0] the CFAR word ([31:0] raw magnitude, [32] detected,
 *          [57:33] k_interp = bin * 2^FRAC + offset); NCH words per frame,
 *          channel 0 first
 * FPP frames per packet: 1 + FPP * NCH words (183 words, 1464 bytes, for NCH = 2:
 * one UDP datagram in a 1500-byte MTU).
 * Results come ~2T + 400 cycles after their frame's end, so a tick carries the
 * frame before the one that just ended.
 *
 * Packets start on enable and always finish. Words go out as single 64-bit AXI
 * writes on a 128-bit port (S_AXI_HPC0), one at a time; wr_ptr advances on each
 * write response, so it never leads the data in DDR. A word that finds the
 * FIFO full is dropped and counted (monitor_server re-aligns on the headers).
 */
module zcu216_points #(
  parameter int          NCH     = 2,
  parameter int          FPP     = 183 / NCH,          // frames per packet
  parameter logic [39:0] BASE    = 40'h1E00_0000,
  parameter int          RING    = 16384,              // words (monitor_server DMA_BUF_WORDS)
  parameter logic [3:0]  VERSION = 4'd8,
  localparam int         PW      = $clog2(RING)
) (
  input  logic          clk,
  input  logic          rstn,
  input  logic          enable,
  input  logic          tick,                         // frame start, channel 0
  input  logic [NCH-1:0] r_valid,
  input  logic [63:0]   r_word [NCH],
  output logic [PW-1:0] wr_ptr,                       // next word to be written
  output logic [31:0]   packets,
  output logic [31:0]   drops,
  output logic          err,                          // sticky: a write got SLVERR/DECERR
  // AXI4 write master, 128 bits
  output logic [48:0]   m_awaddr,
  output logic [7:0]    m_awlen,
  output logic [2:0]    m_awsize,
  output logic [1:0]    m_awburst,
  output logic [3:0]    m_awcache,
  output logic [2:0]    m_awprot,
  output logic          m_awvalid,
  input  logic          m_awready,
  output logic [127:0]  m_wdata,
  output logic [15:0]   m_wstrb,
  output logic          m_wlast,
  output logic          m_wvalid,
  input  logic          m_wready,
  input  logic [1:0]    m_bresp,
  input  logic          m_bvalid,
  output logic          m_bready
);

localparam int FW = $clog2(FPP + 1);
localparam int QD = 64;                               // FIFO depth (words)
localparam int QW = $clog2(QD);

// latest result per channel since the last tick
logic [57:0]    res [NCH];
logic [NCH-1:0] has;
logic [54:0]    ticks;

// the words of one tick, sent one per cycle: header (first frame of a packet)
// then one word per channel
logic [63:0]    grp [NCH + 1];
logic [$clog2(NCH + 2)-1:0] gi, gn;
logic [FW-1:0]  fi;                                   // frames into the packet
logic           in_pkt;

logic [63:0]    q [QD];
logic [QW:0]    qwr, qrd;
wire            q_full  = (qwr - qrd) == (QW+1)'(QD);
wire            q_empty = qwr == qrd;
wire            push    = gi != gn;

always_ff @(posedge clk) begin
  if (!rstn) begin
    has     <= '0;
    for (int c = 0; c < NCH; c++) res[c] <= '0;
    ticks   <= '0;
    gi      <= '0;
    gn      <= '0;
    fi      <= '0;
    in_pkt  <= 1'b0;
    qwr     <= '0;
    packets <= '0;
    drops   <= '0;
  end else begin
    for (int c = 0; c < NCH; c++)
      if (r_valid[c]) begin
        res[c] <= r_word[c][57:0];
        has[c] <= 1'b1;
      end
    if (tick) begin
      ticks <= ticks + 1'b1;
      for (int c = 0; c < NCH; c++)
        if (!r_valid[c]) has[c] <= 1'b0;
      if (in_pkt || enable) begin
        grp[0] <= {1'b1, 4'(NCH), VERSION, ticks};
        for (int c = 0; c < NCH; c++)
          grp[c + 1] <= {1'b0, has[c], 4'b0, res[c]};
        gi     <= (fi != 0) ? 1 : 0;                // header only on a packet's first frame
        gn     <= NCH + 1;
        in_pkt <= 1'b1;
        if (fi == FW'(FPP - 1)) begin
          fi      <= '0;
          in_pkt  <= 1'b0;
          packets <= packets + 1'b1;
        end else
          fi <= fi + 1'b1;
      end
    end else if (push) begin
      gi <= gi + 1'b1;
      if (q_full) drops <= drops + 1'b1;
      else begin
        q[qwr[QW-1:0]] <= grp[gi];
        qwr <= qwr + 1'b1;
      end
    end
  end
end

// writer: one 64-bit word per AXI transaction, the pointer after its response
logic          aw_pend, w_pend, b_wait;
logic [PW-1:0] wa;

assign m_awlen   = 8'd0;
assign m_awsize  = 3'd3;                              // 8 bytes
assign m_awburst = 2'b01;
assign m_awcache = 4'b0011;
assign m_awprot  = 3'b000;
assign m_wlast   = 1'b1;
assign m_bready  = 1'b1;
assign m_awvalid = aw_pend;
assign m_wvalid  = w_pend;

always_ff @(posedge clk) begin
  if (!rstn) begin
    aw_pend <= 1'b0;
    w_pend  <= 1'b0;
    b_wait  <= 1'b0;
    qrd     <= '0;
    wa      <= '0;
    wr_ptr  <= '0;
    err     <= 1'b0;
  end else begin
    if (aw_pend && m_awready) aw_pend <= 1'b0;
    if (w_pend && m_wready)   w_pend  <= 1'b0;
    if (!b_wait && !q_empty) begin
      m_awaddr <= 49'(BASE) + 49'({wa, 3'b000});
      m_wdata  <= {2{q[qrd[QW-1:0]]}};
      m_wstrb  <= wa[0] ? 16'hff00 : 16'h00ff;
      aw_pend  <= 1'b1;
      w_pend   <= 1'b1;
      b_wait   <= 1'b1;
      qrd      <= qrd + 1'b1;
    end
    if (b_wait && m_bvalid) begin
      b_wait <= 1'b0;
      wa     <= wa + 1'b1;
      wr_ptr <= wa + 1'b1;
      if (m_bresp[1]) err <= 1'b1;
    end
  end
end

endmodule


/**
 * A counter that steps by one at a time, into another clock domain: Gray code
 * registered at the source, two flops, back to binary.
 */
module zcu216_gray_sync #(
  parameter int W = 32
) (
  input  logic         src_clk,
  input  logic [W-1:0] src,
  input  logic         dst_clk,
  output logic [W-1:0] dst
);

logic [W-1:0] g = '0;
(* ASYNC_REG = "TRUE" *) logic [W-1:0] g1, g2;

always_ff @(posedge src_clk) g <= src ^ (src >> 1);

always_ff @(posedge dst_clk) begin
  g1 <= g;
  g2 <= g1;
  for (int i = W - 1; i >= 0; i--)
    dst[i] <= ^(g2 >> i);
end

endmodule
