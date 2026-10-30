/**
 * The ASG's output routing (slot 3), at the Red Pitaya DSP's register
 * addresses so pyrpl's output_direct works unchanged.
 *
 * On the Red Pitaya every DSP module n has its registers at slot 3 + n * 0x10000
 * and its output_direct at +0x4: 0 off, 1 out1, 2 out2, 3 both, and the DAC
 * outputs are the saturated sums of what is routed to them. Here only the
 * ASG's four channels exist (DSP numbers 8-11, asg0-3) and there are four
 * outputs, the FMC DAC card's channels: output_direct is a mask, bit k = out
 * k+1 (so 1 / 2 / 3 mean what they do on the Red Pitaya, 4 and 8 are out3 and
 * out4). Every mask is 0 after reset: the DACs sit at 0 V until an ASG is
 * routed to them, as on the Red Pitaya.
 *
 * Every other address in the slot reads 0 and ignores writes. Latency: one
 * clock from the ASG to out.
 */
module zcu216_dsp_out (
  input  logic        clk,
  input  logic        rstn,
  input  logic [13:0] asg [4],                    // asg0-3, two's complement
  output logic [13:0] out [4],                    // out1-4, saturated sums
  // system bus
  input  logic [31:0] sys_addr,
  input  logic [31:0] sys_wdata,
  input  logic        sys_wen,
  input  logic        sys_ren,
  output logic [31:0] sys_rdata,
  output logic        sys_err,
  output logic        sys_ack
);

localparam int ASG_DSP = 8;                       // DSP number of asg0

logic [3:0] mask [4];                             // per ASG channel

// the ASG channel an address belongs to, if its register is output_direct
logic [4:0] num;
logic       hit;
assign num = sys_addr[20:16];
assign hit = sys_addr[15:0] == 16'h0004 && num >= 5'(ASG_DSP) && num < 5'(ASG_DSP + 4);

always_ff @(posedge clk)
  if (!rstn) begin
    for (int i = 0; i < 4; i++) mask[i] <= '0;
  end else if (sys_wen && hit)
    mask[num - 5'(ASG_DSP)] <= sys_wdata[3:0];

always_ff @(posedge clk) begin
  sys_ack   <= sys_wen || sys_ren;
  sys_rdata <= hit ? {28'h0, mask[num - 5'(ASG_DSP)]} : 32'h0;
end
assign sys_err = 1'b0;

always_ff @(posedge clk)
  for (int k = 0; k < 4; k++) begin
    automatic logic signed [15:0] s = '0;
    for (int i = 0; i < 4; i++)
      if (mask[i][k]) s += 16'(signed'(asg[i]));
    out[k] <= (s > 16'sd8191) ? 14'h1FFF : (s < -16'sd8192) ? 14'h2000 : s[13:0];
  end

endmodule
