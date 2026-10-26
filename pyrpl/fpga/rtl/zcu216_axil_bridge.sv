/**
 * Red Pitaya system bus -> AXI4-Lite master, for the RF Data Converter's
 * register interface in a register slot. One transaction at a time (axi_slave
 * runs one), acked when the AXI response comes back. The AXI side runs on the
 * bus clock. An AXI error response is NOT passed on as sys_err: the PS turns a
 * bus error into a fault that kills the process making the access, and that is
 * usually monitor_server. Like every Red Pitaya peripheral, it just acks.
 *
 * RFDC register accesses can take far longer than other registers, so the
 * axi_slave in front of this must time out later than its default 32 cycles
 * (ACK_W).
 */

module zcu216_axil_bridge #(
  parameter int AW = 18
) (
  input  logic          clk,
  input  logic          rstn,
  // system bus
  input  logic [31:0]   sys_addr,
  input  logic [31:0]   sys_wdata,
  input  logic          sys_wen,
  input  logic          sys_ren,
  output logic [31:0]   sys_rdata,
  output logic          sys_err,
  output logic          sys_ack,
  // AXI4-Lite master
  output logic [AW-1:0] m_awaddr,
  output logic          m_awvalid,
  input  logic          m_awready,
  output logic [31:0]   m_wdata,
  output logic [3:0]    m_wstrb,
  output logic          m_wvalid,
  input  logic          m_wready,
  input  logic [1:0]    m_bresp,
  input  logic          m_bvalid,
  output logic          m_bready,
  output logic [AW-1:0] m_araddr,
  output logic          m_arvalid,
  input  logic          m_arready,
  input  logic [31:0]   m_rdata,
  input  logic [1:0]    m_rresp,
  input  logic          m_rvalid,
  output logic          m_rready
);

assign m_wstrb  = 4'hf;
assign m_bready = 1'b1;
assign m_rready = 1'b1;

always_ff @(posedge clk)
  if (!rstn) begin
    m_awvalid <= 1'b0;
    m_wvalid  <= 1'b0;
    m_arvalid <= 1'b0;
    sys_ack   <= 1'b0;
    sys_err   <= 1'b0;
  end else begin
    sys_ack <= 1'b0;
    sys_err <= 1'b0;

    if (sys_wen) begin
      m_awaddr  <= sys_addr[AW-1:0];
      m_wdata   <= sys_wdata;
      m_awvalid <= 1'b1;
      m_wvalid  <= 1'b1;
    end else if (sys_ren) begin
      m_araddr  <= sys_addr[AW-1:0];
      m_arvalid <= 1'b1;
    end

    if (m_awvalid && m_awready) m_awvalid <= 1'b0;
    if (m_wvalid  && m_wready)  m_wvalid  <= 1'b0;
    if (m_arvalid && m_arready) m_arvalid <= 1'b0;

    if (m_bvalid)
      sys_ack <= 1'b1;
    if (m_rvalid) begin
      sys_ack   <= 1'b1;
      sys_rdata <= m_rdata;
    end
  end

endmodule
