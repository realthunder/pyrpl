/**
 * The ZCU216's signal hub (slot 3), the Red Pitaya's DSP block: every signal has
 * a DSP number, every sink has a select register at its own number + 0x0, the
 * outputs are the saturated sums of what output_direct routes to them, and the
 * modules are the Red Pitaya's (red_pitaya_pid_block, _iq_block, _iir_block), at
 * its numbers and addresses (slot 3 + 0x10000 * number), so pyrpl's registers
 * work as they are. PIDS, IQS and IIR say which are built: numbers 0-2, 5-7, 4.
 *
 *   number  signal             number  sink (register +0x0)
 *   0-2     pid0-2             0-2     pid0-2
 *   4       iir                4       iir
 *   5-7     iq0-2              5-7     iq0-2
 *   8-11    asg0-3             8, 9    scope channel 1, 2
 *   12, 13  in1, in2           28, 29  FFT channel 0, 1
 *           (RF-ADC 0, 1)
 *   14, 15  out1, out2
 *   21      off
 *   22, 23  in3, in4 (RF-ADC 2, 3)
 *   24, 25  out3, out4
 *   26, 27  FFT input 0, 1
 *   30      tone generator (FFT sinks only)
 *
 * Registers of any number, as red_pitaya_dsp: +0x0 input select, +0x4
 * output_direct, +0x8 outputs saturated (bit k = out k+1), +0xC sync (bit =
 * module number, all 1 after reset), +0x10 the number's current signal. Other
 * addresses go to the module; numbers with no module read 0.
 *
 * output_direct is a mask, bit k = out k+1: 1 / 2 / 3 mean what they do on the
 * Red Pitaya, 4 and 8 are out3 and out4. Every mask is 0 after reset, so the DACs
 * sit at 0 V until something is routed to them.
 *
 * Everything here runs on the system clock, 125 MS/s. in1-4 are the RF-ADC
 * channels decimated to that rate (zcu216_adc_dec); only the scope and the FFT
 * see them at 2.5 GS/s, by the fclk path. So a scope or FFT sink has two routes:
 * an RF-ADC channel or FFT input is taken on fclk, selected there by index
 * (sc_sel: 0-3 RF-ADC, 4-5 FFT inputs, 6 + channel the hub signal; fft_sel: 0
 * tone, 1-4 RF-ADC, 5 the hub signal); any other number is selected here into
 * sig_o, which crosses to fclk through zcu216_sig_cdc (slow to fast, a sample is
 * held). A new module therefore adds nothing on fclk.
 *
 * Latency: none from a signal to a module's input, as on the Red Pitaya; two
 * clocks from a module's or ASG's output to out.
 */
module zcu216_dsp #(
  parameter int PIDS = 1,                         // 0..3
  parameter int IQS  = 1,                         // 0..2
  parameter bit IIR  = 0
) (
  input  logic        clk,
  input  logic        rstn,
  input  logic [13:0] in  [4],                    // RF-ADC 0-3, 125 MS/s
  input  logic [13:0] asg [4],                    // asg0-3, two's complement
  output logic [13:0] out [4],                    // out1-4, saturated sums
  output logic [2:0]  sc_sel [2],                 // scope channel inputs
  output logic [2:0]  fft_sel [2],                // FFT channel inputs
  output logic [13:0] sig_o [4],                  // scope 1, 2 and FFT 0, 1's signals
  // system bus
  input  logic [31:0] sys_addr,
  input  logic [31:0] sys_wdata,
  input  logic        sys_wen,
  input  logic        sys_ren,
  output logic [31:0] sys_rdata,
  output logic        sys_err,
  output logic        sys_ack
);

localparam int N_PID = 0, N_IIR = 4, N_IQ = 5, N_ASG = 8, N_SC = 8;
localparam int N_IN1 = 12, N_OUT1 = 14, N_IN3 = 22, N_OUT3 = 24, N_FFT = 28, N_TONE = 30;

// which numbers exist
localparam logic [31:0] M_PID = ((32'd1 << PIDS) - 32'd1) << N_PID;
localparam logic [31:0] M_IQ  = ((32'd1 << IQS)  - 32'd1) << N_IQ;
localparam logic [31:0] M_IIR = 32'(IIR) << N_IIR;
localparam logic [31:0] M_MOD = M_PID | M_IQ | M_IIR;     // modules
localparam logic [31:0] M_ASG = 32'hF << N_ASG;
localparam logic [31:0] M_IN  = M_MOD | (32'h3 << N_SC) | (32'h3 << N_FFT);   // sinks
localparam logic [31:0] M_OD  = M_MOD | M_ASG;            // sources of output_direct

function automatic logic [2:0] sc_index(logic [4:0] n, bit c);
  case (n)
    5'd12:   return 3'd0;
    5'd13:   return 3'd1;
    5'd22:   return 3'd2;
    5'd23:   return 3'd3;
    5'd26:   return 3'd4;
    5'd27:   return 3'd5;
    default: return 3'd6 + 3'(c);
  endcase
endfunction

function automatic logic [2:0] fft_index(logic [4:0] n);
  case (n)
    5'd30:   return 3'd0;
    5'd12:   return 3'd1;
    5'd13:   return 3'd2;
    5'd22:   return 3'd3;
    5'd23:   return 3'd4;
    default: return 3'd5;
  endcase
endfunction

logic [4:0]  input_select [32];
logic [3:0]  output_select [32];
logic [7:0]  sync;                                // bit = module number
logic [3:0]  out_sat;

logic [13:0] mod_sig [32], mod_od [32];           // the modules' signal, output_direct
logic [31:0] mod_rdata [32];
logic [31:0] mod_ack;
logic [13:0] signal [32];                         // every number's signal
logic [13:0] input_signal [32];                   // every sink's selected signal

always_comb begin
  signal = mod_sig;
  for (int i = 0; i < 4; i++) signal[N_ASG + i] = asg[i];
  for (int i = 0; i < 2; i++) begin
    signal[N_IN1  + i] = in[i];
    signal[N_IN3  + i] = in[2 + i];
    signal[N_OUT1 + i] = out[i];
    signal[N_OUT3 + i] = out[2 + i];
  end
end

for (genvar n = 0; n < 32; n++) begin : g_in
  assign input_signal[n] = signal[input_select[n]];
end

////////////////////////////////////////////////////////////////////////////////
// bus
////////////////////////////////////////////////////////////////////////////////

logic [4:0] num;
logic       en;
assign num = sys_addr[20:16];
assign en  = sys_wen || sys_ren;

always_ff @(posedge clk)
  if (!rstn) begin
    for (int i = 0; i < 32; i++) begin
      input_select[i]  <= 5'(N_IN1);              // as the Red Pitaya: in1, outputs off
      output_select[i] <= '0;
    end
    input_select[N_SC + 1]  <= 5'(N_IN1 + 1);
    input_select[N_FFT]     <= 5'(N_TONE);
    input_select[N_FFT + 1] <= 5'(N_TONE);
    sync <= '1;
  end else if (sys_wen)
    case (sys_addr[15:0])
      16'h00: if (M_IN[num]) input_select[num]  <= sys_wdata[4:0];
      16'h04: if (M_OD[num]) output_select[num] <= sys_wdata[3:0];
      16'h0C: sync <= sys_wdata[7:0];
      default: ;
    endcase

always_ff @(posedge clk) begin
  sys_ack   <= en;
  sys_rdata <= 32'h0;
  case (sys_addr[15:0])
    16'h00: sys_rdata <= M_IN[num] ? 32'(input_select[num])  : 32'h0;
    16'h04: sys_rdata <= M_OD[num] ? 32'(output_select[num]) : 32'h0;
    16'h08: sys_rdata <= 32'(out_sat);
    16'h0C: sys_rdata <= 32'(sync);
    16'h10: sys_rdata <= 32'(signal[num]);
    16'h14: ;                                     // the Red Pitaya's scanner: not here
    default:
      if (M_MOD[num]) begin
        sys_ack   <= mod_ack[num];
        sys_rdata <= mod_rdata[num];
      end
  endcase
end
assign sys_err = 1'b0;

////////////////////////////////////////////////////////////////////////////////
// outputs
////////////////////////////////////////////////////////////////////////////////

// two stages (the Red Pitaya's adder tree has six): the modules' sum and the
// ASGs' sum, then the two together with saturation
logic signed [18:0] s_mod [4], s_asg [4], s_all [4];

always_comb
  for (int k = 0; k < 4; k++) s_all[k] = s_mod[k] + s_asg[k];

always_ff @(posedge clk)
  for (int k = 0; k < 4; k++) begin
    automatic logic signed [18:0] sm = '0, sa = '0;
    for (int n = 0; n < 8; n++)
      if (M_MOD[n] && output_select[n][k]) sm += 19'(signed'(mod_od[n]));
    for (int i = 0; i < 4; i++)
      if (output_select[N_ASG + i][k]) sa += 19'(signed'(asg[i]));
    s_mod[k]   <= sm;
    s_asg[k]   <= sa;
    out[k]     <= (s_all[k] > 19'sd8191) ? 14'h1FFF : (s_all[k] < -19'sd8192) ? 14'h2000
                                                                             : s_all[k][13:0];
    out_sat[k] <= (s_all[k] > 19'sd8191) || (s_all[k] < -19'sd8192);
  end

////////////////////////////////////////////////////////////////////////////////
// scope and FFT sinks
////////////////////////////////////////////////////////////////////////////////

always_ff @(posedge clk) begin
  sc_sel[0]  <= sc_index(input_select[N_SC], 1'b0);
  sc_sel[1]  <= sc_index(input_select[N_SC + 1], 1'b1);
  fft_sel[0] <= fft_index(input_select[N_FFT]);
  fft_sel[1] <= fft_index(input_select[N_FFT + 1]);
  sig_o[0]   <= input_signal[N_SC];
  sig_o[1]   <= input_signal[N_SC + 1];
  sig_o[2]   <= input_signal[N_FFT];
  sig_o[3]   <= input_signal[N_FFT + 1];
end

////////////////////////////////////////////////////////////////////////////////
// modules
////////////////////////////////////////////////////////////////////////////////

// the differential mode's partners: pid0 and pid1
logic [13:0] diff_o [3];

for (genvar n = 0; n < 32; n++) begin : g_mod
  if (M_PID[n]) begin : g_pid
    red_pitaya_pid_block i_pid (
      .clk_i      (clk                 ),
      .rstn_i     (rstn                ),
      .sync_i     (sync[n]             ),
      .window_i   (2'b11               ),  // no FFT window gating here
      .dat_i      (input_signal[n]     ),
      .dat_o      (mod_od[n]           ),
      .diff_dat_i ((n < 2 && PIDS > 1) ? diff_o[1 - (n % 2)] : 14'h0),
      .diff_dat_o (diff_o[n]           ),
      .addr       (sys_addr[15:0]      ),
      .wen        (sys_wen && num == n ),
      .ren        (sys_ren && num == n ),
      .ack        (mod_ack[n]          ),
      .rdata      (mod_rdata[n]        ),
      .wdata      (sys_wdata           )
    );
    assign mod_sig[n] = mod_od[n];
  end else if (M_IQ[n]) begin : g_iq
    red_pitaya_iq_block i_iq (
      .clk_i     (clk                 ),
      .rstn_i    (rstn                ),
      .sync_i    (sync[n]             ),
      .dat_i     (input_signal[n]     ),
      .dat_o     (mod_od[n]           ),
      .signal_o  (mod_sig[n]          ),
      .signal2_o (                    ),
      .addr      (sys_addr[15:0]      ),
      .wen       (sys_wen && num == n ),
      .ren       (sys_ren && num == n ),
      .ack       (mod_ack[n]          ),
      .rdata     (mod_rdata[n]        ),
      .wdata     (sys_wdata           )
    );
  end else if (M_IIR[n]) begin : g_iir
    red_pitaya_iir_block i_iir (
      .clk_i  (clk                 ),
      .rstn_i (rstn                ),
      .dat_i  (input_signal[n]     ),
      .dat_o  (mod_od[n]           ),
      .addr   (sys_addr[15:0]      ),
      .wen    (sys_wen && num == n ),
      .ren    (sys_ren && num == n ),
      .ack    (mod_ack[n]          ),
      .rdata  (mod_rdata[n]        ),
      .wdata  (sys_wdata           )
    );
    assign mod_sig[n] = mod_od[n];
  end else begin : g_none
    assign mod_sig[n]   = 14'h0;
    assign mod_od[n]    = 14'h0;
    assign mod_ack[n]   = 1'b0;
    assign mod_rdata[n] = 32'h0;
  end
end

endmodule
