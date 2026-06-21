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
`ifdef FFT_CLK_200
localparam CLKOUT4_DIV = 5;   // 1000/5 = 200 MHz FFT clock
`else
localparam CLKOUT4_DIV = 4;   // 1000/4 = 250 MHz (original ser clock)
`endif

PLLE2_ADV #(
   .BANDWIDTH            ("OPTIMIZED"),
   .COMPENSATION         ("ZHOLD"    ),
   .DIVCLK_DIVIDE        ( DIV       ),
   .CLKFBOUT_MULT        ( MULT      ),
   .CLKFBOUT_PHASE       ( 0.000     ),
   .CLKOUT0_DIVIDE       ( DIV_ADC   ),
   .CLKOUT0_PHASE        ( 0.000     ),
   .CLKOUT0_DUTY_CYCLE   ( 0.5       ),
   .CLKOUT1_DIVIDE       ( 8         ),
   .CLKOUT1_PHASE        ( 0.000     ),
   .CLKOUT1_DUTY_CYCLE   ( 0.5       ),
   .CLKOUT2_DIVIDE       ( 4         ),
   .CLKOUT2_PHASE        ( 0.000     ),
   .CLKOUT2_DUTY_CYCLE   ( 0.5       ),
   .CLKOUT3_DIVIDE       ( 4         ),
   .CLKOUT3_PHASE        (-45.000    ),
   .CLKOUT3_DUTY_CYCLE   ( 0.5       ),
   .CLKOUT4_DIVIDE       ( CLKOUT4_DIV ),  // 4->250MHz, 5->200MHz (FFT_CLK_200)
   .CLKOUT4_PHASE        ( 0.000     ),
   .CLKOUT4_DUTY_CYCLE   ( 0.5       ),
   .CLKOUT5_DIVIDE       ( 4         ),
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
