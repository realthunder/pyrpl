/**
 * The ZCU216's signal hub (slot 3), the Red Pitaya's DSP block as far as this
 * board has modules: every signal has a DSP number, every sink has a select
 * register at its own DSP number + 0x0, and the outputs are the saturated sums
 * of what output_direct routes to them, at the Red Pitaya's addresses (slot 3 +
 * 0x10000 * number), so pyrpl's registers work as they are.
 *
 *   number  signal             number  sink (register +0x0)
 *   8-11    asg0-3             8, 9    scope channel 1, 2
 *   12, 13  in1, in2           28, 29  FFT channel 0, 1
 *           (RF-ADC 0, 1)
 *   14, 15  out1, out2
 *   21      off
 *   22, 23  RF-ADC 2, 3
 *   24, 25  out3, out4
 *   26, 27  FFT input 0, 1
 *   30      tone generator (FFT sinks only)
 *
 * output_direct (+0x4 of asg0-3) is a mask, bit k = out k+1: 1 / 2 / 3 mean
 * what they do on the Red Pitaya, 4 and 8 are out3 and out4. Every mask is 0
 * after reset, so the DACs sit at 0 V until an ASG is routed to them.
 *
 * Signals cross to their sink's clock through zcu216_sig_cdc: slow to fast, a
 * sample is held. The muxes themselves sit with their data on fclk
 * (zcu216_fft_test); this block keeps the settings, and hands them over as
 * indices: 0-3 RF-ADC, 4-5 FFT inputs, 6-7 off, 8-11 asg, 12-15 out for the
 * scope; 0 tone, 1-4 RF-ADC, 5-12 asg0-3 and out1-4 for the FFT.
 *
 * Every other address in the slot reads 0 and ignores writes. Latency: one
 * clock from the ASG to out.
 */
module zcu216_dsp (
  input  logic        clk,
  input  logic        rstn,
  input  logic [13:0] asg [4],                    // asg0-3, two's complement
  output logic [13:0] out [4],                    // out1-4, saturated sums
  output logic [3:0]  sc_sel [2],                 // scope channel inputs
  output logic [3:0]  fft_sel [2],                // FFT channel inputs
  // system bus
  input  logic [31:0] sys_addr,
  input  logic [31:0] sys_wdata,
  input  logic        sys_wen,
  input  logic        sys_ren,
  output logic [31:0] sys_rdata,
  output logic        sys_err,
  output logic        sys_ack
);

localparam logic [4:0] N_ASG0 = 5'd8, N_SC0 = 5'd8, N_FFT0 = 5'd28;
localparam logic [4:0] N_IN1 = 5'd12, N_IN2 = 5'd13, N_TONE = 5'd30;

function automatic logic [3:0] sc_index(logic [4:0] n);
  case (n)
    5'd8, 5'd9, 5'd10, 5'd11: return 4'(n);
    5'd12:   return 4'd0;
    5'd13:   return 4'd1;
    5'd14:   return 4'd12;
    5'd15:   return 4'd13;
    5'd22:   return 4'd2;
    5'd23:   return 4'd3;
    5'd24:   return 4'd14;
    5'd25:   return 4'd15;
    5'd26:   return 4'd4;
    5'd27:   return 4'd5;
    default: return 4'd6;
  endcase
endfunction

function automatic logic [3:0] fft_index(logic [4:0] n);
  case (n)
    5'd12:   return 4'd1;
    5'd13:   return 4'd2;
    5'd22:   return 4'd3;
    5'd23:   return 4'd4;
    5'd8, 5'd9, 5'd10, 5'd11: return 4'(n - 5'd3);    // 5-8
    5'd14:   return 4'd9;
    5'd15:   return 4'd10;
    5'd24:   return 4'd11;
    5'd25:   return 4'd12;
    default: return 4'd0;
  endcase
endfunction

logic [3:0] mask [4];                             // per ASG channel
logic [4:0] sc_n [2], fft_n [2];                  // the sinks' signal numbers

logic [4:0] num;
logic       r0, r4, h_od, h_sc, h_fft;
assign num   = sys_addr[20:16];
assign r0    = sys_addr[15:0] == 16'h0000;
assign r4    = sys_addr[15:0] == 16'h0004;
assign h_od  = r4 && num >= N_ASG0 && num < N_ASG0 + 5'd4;
assign h_sc  = r0 && num >= N_SC0  && num < N_SC0  + 5'd2;
assign h_fft = r0 && num >= N_FFT0 && num < N_FFT0 + 5'd2;

always_ff @(posedge clk)
  if (!rstn) begin
    for (int i = 0; i < 4; i++) mask[i] <= '0;
    sc_n[0]  <= N_IN1;  sc_n[1]  <= N_IN2;
    fft_n[0] <= N_TONE; fft_n[1] <= N_TONE;
  end else if (sys_wen) begin
    if (h_od)  mask[2'(num - N_ASG0)] <= sys_wdata[3:0];
    if (h_sc)  sc_n[1'(num - N_SC0)]   <= sys_wdata[4:0];
    if (h_fft) fft_n[1'(num - N_FFT0)] <= sys_wdata[4:0];
  end

always_ff @(posedge clk) begin
  sc_sel[0]  <= sc_index(sc_n[0]);
  sc_sel[1]  <= sc_index(sc_n[1]);
  fft_sel[0] <= fft_index(fft_n[0]);
  fft_sel[1] <= fft_index(fft_n[1]);
  sys_ack    <= sys_wen || sys_ren;
  sys_rdata  <= h_od  ? {28'h0, mask[2'(num - N_ASG0)]} :
                h_sc  ? {27'h0, sc_n[1'(num - N_SC0)]} :
                h_fft ? {27'h0, fft_n[1'(num - N_FFT0)]} : 32'h0;
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
