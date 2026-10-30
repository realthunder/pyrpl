/**
 * The Red Pitaya system bus across two clocks, with a timeout.
 *
 * Front (clk_a): axi_slave's side, on a clock that always runs (the PS's
 * pl_clk0). Back (clk_b): the modules, on the system clock (125 MHz from the RF
 * clock), which only runs once the CLK104 and the RF tiles are up and stops
 * whenever a tile restarts or the CLK104 is reprogrammed. A register access
 * while it is down must not hang the PS, so the front answers every access
 * itself: with the module's data, or after TIMEOUT clk_a cycles with 0 (and no
 * error: axi_slave would turn an error into SLVERR, which the PS takes as an
 * SError and stops the kernel).
 *
 * Four-phase handshake: the front holds the request (address, data, a 4-bit
 * sequence number) and raises req; the back synchronises req, issues the
 * access as a one-cycle sys_wen / sys_ren, waits for the module's ack (or its
 * own timeout), holds the result with the sequence number and raises ack until
 * req falls. The front takes the result only with its own sequence number, so
 * the late answer to a request it gave up on is never taken for the next one;
 * and it raises req only once the back's ack has fallen. A timed-out write may
 * still reach the module when its clock returns; a timed-out read reads 0.
 *
 * Latency: about 2 + 2 synchroniser cycles on each side, plus the module's own.
 */
module zcu216_bus_cdc #(
  parameter int TIMEOUT   = 256,                  // clk_a cycles (axi_slave's own: 512)
  parameter int B_TIMEOUT = 255                   // clk_b cycles for a module ack
) (
  input  logic        clk_a,
  input  logic        rstn_a,
  input  logic [31:0] a_addr,
  input  logic [31:0] a_wdata,
  input  logic [3:0]  a_sel,
  input  logic        a_wen,
  input  logic        a_ren,
  output logic [31:0] a_rdata,
  output logic        a_err,
  output logic        a_ack,

  input  logic        clk_b,
  input  logic        rstn_b,
  output logic [31:0] b_addr,
  output logic [31:0] b_wdata,
  output logic [3:0]  b_sel,
  output logic        b_wen,
  output logic        b_ren,
  input  logic [31:0] b_rdata,
  input  logic        b_err,
  input  logic        b_ack
);

////////////////////////////////////////////////////////////////////////////////
// front (clk_a)
////////////////////////////////////////////////////////////////////////////////

logic [31:0] p_addr, p_wdata;                     // the access axi_slave gave us
logic [3:0]  p_sel;
logic        p_we;
logic [31:0] f_addr, f_wdata;                     // the request, held for the back
logic [3:0]  f_sel, f_seq;
logic        f_we, f_load, f_sent, f_req, f_busy;
logic [2:0]  f_gap;                               // clk_a cycles since req fell
logic [$clog2(TIMEOUT+1)-1:0] f_cnt;

// the back's answer, held while its ack is high
logic [31:0] k_rdata;
logic [3:0]  k_seq;
logic        k_err, k_ack;

(* ASYNC_REG = "TRUE" *) logic [1:0] ack_s;

// The request registers change only while req is low, the back's ack is low
// and req has been low for 4 cycles (the back may still see it high for two of
// its own): the back never samples them while they change. req rises the
// cycle after they are loaded.
always_ff @(posedge clk_a)
  if (!rstn_a) begin
    ack_s  <= '0;
    f_req  <= 1'b0;
    f_load <= 1'b0;
    f_busy <= 1'b0;
    f_sent <= 1'b0;
    f_gap  <= '1;
    f_seq  <= '0;
    a_ack  <= 1'b0;
    a_err  <= 1'b0;
  end else begin
    ack_s  <= {ack_s[0], k_ack};
    a_ack  <= 1'b0;
    f_load <= 1'b0;
    if (!f_req && f_gap != '1) f_gap <= f_gap + 1'b1;
    if (!f_busy) begin
      if (a_wen || a_ren) begin
        p_addr  <= a_addr;
        p_wdata <= a_wdata;
        p_sel   <= a_sel;
        p_we    <= a_wen;
        f_sent  <= 1'b0;
        f_busy  <= 1'b1;
        f_cnt   <= '0;
      end
    end else begin
      f_cnt <= f_cnt + 1'b1;
      if (!f_req && !f_load && !ack_s[1] && f_gap[2] && !f_sent) begin
        f_addr  <= p_addr;                        // load ...
        f_wdata <= p_wdata;
        f_sel   <= p_sel;
        f_we    <= p_we;
        f_seq   <= f_seq + 1'b1;
        f_load  <= 1'b1;
        f_sent  <= 1'b1;
      end
      if (f_load)
        f_req <= 1'b1;                            // ... then ask
      if (f_req && ack_s[1] && k_seq == f_seq) begin
        a_rdata <= k_rdata;                       // stable: held while ack is high
        a_err   <= k_err;
        a_ack   <= 1'b1;
        f_req   <= 1'b0;
        f_gap   <= '0;
        f_busy  <= 1'b0;
      end else if (f_cnt == TIMEOUT - 1) begin
        a_rdata <= 32'h0;                         // the back is not answering
        a_err   <= 1'b0;
        a_ack   <= 1'b1;
        f_req   <= 1'b0;
        f_gap   <= '0;
        f_busy  <= 1'b0;
      end
    end
  end

////////////////////////////////////////////////////////////////////////////////
// back (clk_b)
////////////////////////////////////////////////////////////////////////////////

(* ASYNC_REG = "TRUE" *) logic [1:0] req_s;
logic       b_busy;
logic [7:0] b_cnt;

always_ff @(posedge clk_b)
  if (!rstn_b) begin
    req_s  <= '0;
    b_busy <= 1'b0;
    b_wen  <= 1'b0;
    b_ren  <= 1'b0;
    k_ack  <= 1'b0;
  end else begin
    req_s <= {req_s[0], f_req};
    b_wen <= 1'b0;
    b_ren <= 1'b0;
    if (k_ack) begin
      if (!req_s[1]) k_ack <= 1'b0;               // the front has taken it
    end else if (!b_busy) begin
      if (req_s[1]) begin                         // the request has been stable 2+ cycles
        b_addr  <= f_addr;
        b_wdata <= f_wdata;
        b_sel   <= f_sel;
        k_seq   <= f_seq;
        b_wen   <= f_we;
        b_ren   <= !f_we;
        b_busy  <= 1'b1;
        b_cnt   <= '0;
      end
    end else begin
      b_cnt <= b_cnt + 1'b1;
      if (b_ack || b_cnt == 8'(B_TIMEOUT)) begin
        k_rdata <= b_ack ? b_rdata : 32'h0;
        k_err   <= b_ack && b_err;
        k_ack   <= 1'b1;
        b_busy  <= 1'b0;
      end
    end
  end

endmodule
