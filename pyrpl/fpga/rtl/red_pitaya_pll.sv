/**
 * @brief Red Pitaya PLL module.
 *
 * @Author Matej Oblak, Iztok Jeras
 *
 * (c) Red Pitaya  http://www.redpitaya.com
 *
 * This part of code is written in Verilog hardware description language (HDL).
 * Please visit http://en.wikipedia.org/wiki/Verilog
 * for more details on the language used herein.
 */

module red_pitaya_pll #(
  parameter DIV = 1,
  parameter MULT = 8,
  parameter DIV_ADC = 8
)(
  // inputs
  input  logic clk       ,  // clock
  input  logic rstn      ,  // reset - active low
  // output clocks
  output logic clk_adc   ,  // ADC clock
  output logic clk_dac_1x,  // DAC clock
  output logic clk_dac_2x,  // DAC clock
  output logic clk_dac_2p,  // DAC clock
  output logic clk_ser   ,  // fast serial clock
  output logic clk_pwm   ,  // PWM clock
  // status outputs
  output logic pll_locked
);

logic clk_fb;

// CLKOUT4 (clk_ser) divide.  clk_ser is currently consumed only by the FFT clock
// mux (see red_pitaya_top.v: `fft_clk = ser_clk`), so we reuse it as the FFT
// clock.  VCO = CLKIN*MULT/DIV = 125*8/1 = 1000 MHz.
//   default          : /4 -> 250 MHz
//   `define FFT_CLK_200: /5 -> 200 MHz   (relaxes the FFT timing budget to 5 ns)
//   `define FFT_CLK_178: retune VCO 1000->1250 MHz (CLKFBOUT_MULT 8->10) and scale
//                        every output divider x1.25 so adc/dac/pwm stay byte-identical,
//                        then CLKOUT4 = 1250/7 = 178.57 MHz (5.6 ns).  178.57 MHz is
//                        unreachable from the 1000 MHz VCO with an integer divider, so
//                        this is the only way to land an FFT clock strictly in 170-200.
//   FFT_CLK_178 takes precedence over FFT_CLK_200; do not set both.
`ifdef FFT_CLK_178
localparam MULT_EFF    = 10;             // VCO = 125*10/1 = 1250 MHz
localparam DIVADC_EFF  = DIV_ADC * 10/8; // adc:  8 -> 10  (125 MHz)
localparam DAC1X_DIV   = 10;             // dac_1x: 1250/10 = 125 MHz
localparam DAC2X_DIV   = 5;              // dac_2x/2p, pwm: 1250/5 = 250 MHz
localparam CLKOUT4_DIV = 7;              // fft: 1250/7 = 178.57 MHz
`elsif FFT_CLK_200
localparam MULT_EFF    = MULT;           // VCO = 1000 MHz (unchanged)
localparam DIVADC_EFF  = DIV_ADC;
localparam DAC1X_DIV   = 8;
localparam DAC2X_DIV   = 4;
localparam CLKOUT4_DIV = 5;              // 1000/5 = 200 MHz FFT clock
`else
localparam MULT_EFF    = MULT;           // VCO = 1000 MHz (unchanged)
localparam DIVADC_EFF  = DIV_ADC;
localparam DAC1X_DIV   = 8;
localparam DAC2X_DIV   = 4;
localparam CLKOUT4_DIV = 4;              // 1000/4 = 250 MHz (original ser clock)
`endif

PLLE2_ADV #(
   .BANDWIDTH            ("OPTIMIZED"),
   .COMPENSATION         ("ZHOLD"    ),
   .DIVCLK_DIVIDE        ( DIV       ),
   .CLKFBOUT_MULT        ( MULT_EFF  ),
   .CLKFBOUT_PHASE       ( 0.000     ),
   .CLKOUT0_DIVIDE       ( DIVADC_EFF ),
   .CLKOUT0_PHASE        ( 0.000     ),
   .CLKOUT0_DUTY_CYCLE   ( 0.5       ),
   .CLKOUT1_DIVIDE       ( DAC1X_DIV ),
   .CLKOUT1_PHASE        ( 0.000     ),
   .CLKOUT1_DUTY_CYCLE   ( 0.5       ),
   .CLKOUT2_DIVIDE       ( DAC2X_DIV ),
   .CLKOUT2_PHASE        ( 0.000     ),
   .CLKOUT2_DUTY_CYCLE   ( 0.5       ),
   .CLKOUT3_DIVIDE       ( DAC2X_DIV ),
   .CLKOUT3_PHASE        (-45.000    ),
   .CLKOUT3_DUTY_CYCLE   ( 0.5       ),
   .CLKOUT4_DIVIDE       ( CLKOUT4_DIV ),  // 4->250, 5->200 (FFT_CLK_200), 7->178.57 (FFT_CLK_178)
   .CLKOUT4_PHASE        ( 0.000     ),
   .CLKOUT4_DUTY_CYCLE   ( 0.5       ),
   .CLKOUT5_DIVIDE       ( DAC2X_DIV ),
   .CLKOUT5_PHASE        ( 0.000     ),
   .CLKOUT5_DUTY_CYCLE   ( 0.5       ),
   .CLKIN1_PERIOD        ( 8.000     ),
   .REF_JITTER1          ( 0.010     )
) pll (
   // Output clocks
   .CLKFBOUT     (clk_fb    ),
   .CLKOUT0      (clk_adc   ),
   .CLKOUT1      (clk_dac_1x),
   .CLKOUT2      (clk_dac_2x),
   .CLKOUT3      (clk_dac_2p),
   .CLKOUT4      (clk_ser   ),
   .CLKOUT5      (clk_pwm   ),
   // Input clock control
   .CLKFBIN      (clk_fb    ),
   .CLKIN1       (clk       ),
   .CLKIN2       (1'b0      ),
   // Tied to always select the primary input clock
   .CLKINSEL     (1'b1 ),
   // Ports for dynamic reconfiguration
   .DADDR        (7'h0 ),
   .DCLK         (1'b0 ),
   .DEN          (1'b0 ),
   .DI           (16'h0),
   .DO           (     ),
   .DRDY         (     ),
   .DWE          (1'b0 ),
   // Other control and status signals
   .LOCKED       (pll_locked),
   .PWRDWN       (1'b0      ),
   .RST          (!rstn     )
);

endmodule: red_pitaya_pll
