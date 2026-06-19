// dma_s2mm.sv
// Two-channel AXI-S to AXI3 write master for HP2 (point cloud ring buffer).
// Accepts ch_a and ch_b AXI-S streams, arbitrates at packet boundary,
// writes 64-bit words to DDR via single-beat AXI3 transactions.
// BUF_WORDS must be a power of 2.

module dma_s2mm #(
    parameter bit [31:0] BUF_BASE  = 32'h1e000000,
    parameter int        BUF_WORDS = 16384          // 64-bit words, power-of-2
)(
    input  logic        clk_i,
    input  logic        rstn_i,

    // AXI-S channel A
    input  logic [63:0] a_tdata_i,
    input  logic        a_tvalid_i,
    output logic        a_tready_o,
    input  logic        a_tlast_i,

    // AXI-S channel B
    input  logic [63:0] b_tdata_i,
    input  logic        b_tvalid_i,
    output logic        b_tready_o,
    input  logic        b_tlast_i,

    // Ring-buffer write pointer (word index) for PS polling
    output logic [$clog2(BUF_WORDS)-1:0] wr_ptr_o,

    // AXI3 write master → HP2
    output logic [31:0] axi_awaddr_o,
    output logic [ 3:0] axi_awlen_o,
    output logic [ 2:0] axi_awsize_o,
    output logic [ 1:0] axi_awburst_o,
    output logic [ 1:0] axi_awlock_o,
    output logic [ 3:0] axi_awcache_o,
    output logic [ 2:0] axi_awprot_o,
    output logic [ 3:0] axi_awqos_o,
    output logic [ 5:0] axi_awid_o,
    output logic        axi_awvalid_o,
    input  logic        axi_awready_i,

    output logic [63:0] axi_wdata_o,
    output logic [ 7:0] axi_wstrb_o,
    output logic        axi_wlast_o,
    output logic [ 5:0] axi_wid_o,
    output logic        axi_wvalid_o,
    input  logic        axi_wready_i,

    input  logic        axi_bvalid_i,
    input  logic [ 1:0] axi_bresp_i,
    input  logic [ 5:0] axi_bid_i,
    output logic        axi_bready_o,

    // Read channel tied off — connected to HP2 which is write-only from PL
    output logic        axi_arvalid_o,
    output logic        axi_rready_o
);

localparam PTR_W = $clog2(BUF_WORDS);

typedef enum logic [1:0] {S_IDLE, S_AW, S_W, S_B} state_t;
state_t state;

logic              sel;           // 0 = ch_a, 1 = ch_b
logic              pkt_done;      // tlast was seen on the current packet's last word
logic              word_valid;    // one-word buffer between stream and W channel
logic [PTR_W-1:0]  wr_ptr;

// Mux to selected stream
wire [63:0] s_tdata  = sel ? b_tdata_i  : a_tdata_i;
wire        s_tvalid = sel ? b_tvalid_i : a_tvalid_i;
wire        s_tlast  = sel ? b_tlast_i  : a_tlast_i;

// tready is registered to cut the combinational path into the FIFO read counter.
// Accepted-this-cycle wires suppress tready for one cycle after a successful pop
// to prevent the FIFO advancing before we've consumed the captured word.
wire a_accepted = a_tready_o && a_tvalid_i;
wire b_accepted = b_tready_o && b_tvalid_i;

// AXI3 constant fields
assign axi_awlen_o   = 4'h0;        // single beat
assign axi_awsize_o  = 3'b011;      // 8 bytes
assign axi_awburst_o = 2'b01;       // INCR
assign axi_awlock_o  = 2'b00;
assign axi_awcache_o = 4'b0011;     // bufferable
assign axi_awprot_o  = 3'b000;
assign axi_awqos_o   = 4'h0;
assign axi_awid_o    = 6'h0;
assign axi_wstrb_o   = 8'hff;
assign axi_wlast_o   = 1'b1;        // always last (one beat per transaction)
assign axi_wid_o     = 6'h0;
assign axi_arvalid_o = 1'b0;
assign axi_rready_o  = 1'b0;
assign wr_ptr_o      = wr_ptr;

function automatic logic [31:0] ptr_addr(input logic [PTR_W-1:0] p);
    return BUF_BASE + ({{(32-PTR_W){1'b0}}, p} << 3);
endfunction

always_ff @(posedge clk_i) begin
    if (!rstn_i) begin
        a_tready_o    <= 1'b0;
        b_tready_o    <= 1'b0;
    end else begin
        a_tready_o <= (state == S_W) && !sel && !word_valid && !a_accepted;
        b_tready_o <= (state == S_W) &&  sel && !word_valid && !b_accepted;
    end
end

always_ff @(posedge clk_i) begin
    if (!rstn_i) begin
        state         <= S_IDLE;
        sel           <= 1'b0;
        pkt_done      <= 1'b0;
        word_valid    <= 1'b0;
        wr_ptr        <= '0;
        axi_awaddr_o  <= BUF_BASE;
        axi_awvalid_o <= 1'b0;
        axi_wdata_o   <= '0;
        axi_wvalid_o  <= 1'b0;
        axi_bready_o  <= 1'b0;
    end else begin
        case (state)

        S_IDLE: begin
            if (a_tvalid_i || b_tvalid_i) begin
                // round-robin at packet boundary; fall back to whoever is valid
                sel           <= (a_tvalid_i && b_tvalid_i) ? !sel : b_tvalid_i;
                axi_awaddr_o  <= ptr_addr(wr_ptr);
                axi_awvalid_o <= 1'b1;
                state         <= S_AW;
            end
        end

        S_AW: begin
            if (axi_awready_i) begin
                axi_awvalid_o <= 1'b0;
                state         <= S_W;
            end
        end

        S_W: begin
            // Latch one word from stream (tready registered; guard on word_valid prevents double-latch)
            if (s_tvalid && (sel ? b_tready_o : a_tready_o) && !word_valid) begin
                axi_wdata_o <= s_tdata;
                axi_wvalid_o <= 1'b1;
                word_valid  <= 1'b1;
                pkt_done    <= s_tlast;
            end
            // HP2 accepts the word
            if (axi_wvalid_o && axi_wready_i) begin
                axi_wvalid_o <= 1'b0;
                word_valid   <= 1'b0;
                wr_ptr       <= wr_ptr + 1;     // wraps at BUF_WORDS (power-of-2)
                axi_bready_o <= 1'b1;
                state        <= S_B;
            end
        end

        S_B: begin
            if (axi_bvalid_i) begin
                axi_bready_o <= 1'b0;
                if (pkt_done) begin
                    state <= S_IDLE;
                end else begin
                    axi_awaddr_o  <= ptr_addr(wr_ptr);
                    axi_awvalid_o <= 1'b1;
                    state         <= S_AW;
                end
            end
        end

        endcase
    end
end

endmodule
