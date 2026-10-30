/**
 * The Puzhi PZ-FL9767D FMC DAC card (two AD9767, four 14-bit channels up to
 * 125 MS/s) on the ZCU216's FMC+ connector J28, driven from the output
 * routing's out1-4 (zcu216_dsp) at sys_clk, 125 MS/s.
 *
 * Card (schematic 3312272550PZ-FL9767D-SCH): every AD9767 port has its own 14
 * data pins, CLK and WRT on the FMC LA pins, through SN74AVC16T245 shifters
 * (A side = VADJ, B side = 3.3 V, fixed A -> B); MODE is pulled high (dual-port
 * mode) and SLEEP low. Per port, IOUTA / IOUTB into 24.9 ohm, an AD8066
 * difference stage (gain 2, inverting) and an inverting stage (gain 5.1) to the
 * SMB through R57 / R59 (DAC1), R61 / R64 (DAC2); R58 / R60 / R62 / R63 (not
 * fitted) take the first stage to the SMB instead. Measured (DC, ASG +-1):
 * +-5.0 V as fitted, +-1.0 V and inverted with the second stage bypassed
 * (asg.invert corrects a table).
 *
 *   ch 0  DAC1 port 1  SMB J2        ch 2  DAC2 port 1  SMB J4
 *   ch 1  DAC1 port 2  SMB J3        ch 3  DAC2 port 2  SMB J5
 *
 * The AD9767 takes straight binary (0 = most negative); din is the ASG's two's
 * complement, so the sign bit is inverted. Data leave from IOB flops on the
 * rising clock edge; CLK and WRT are the clock inverted (ODDRE1, D1 = 0, D2 =
 * 1), so their rising edges, which latch the input and update the DAC, fall
 * in the middle of the data eye, as on the Red Pitaya.
 *
 * VADJ: the card has no IPMI EEPROM, so the ZCU216 system controller leaves
 * VADJ_FMC (banks 66 / 67, the shifters' A side) off; autostart turns it on at
 * 1.8 V (zcu216/vadj) before the bitstream. Without it the shifters hold their
 * outputs off.
 */
module zcu216_fmc_dac (
  input  logic        clk,                        // sample clock
  input  logic [13:0] din [4],                    // two's complement
  output logic [55:0] fmc_dac_db,                 // ch * 14 + bit
  output logic [3:0]  fmc_dac_clk,
  output logic [3:0]  fmc_dac_wrt
);

// mid-scale (0 V) from configuration on, so the first WRT edges latch 0 V and
// not the most negative code
(* IOB = "TRUE" *) logic [55:0] db_q = {4{14'h2000}};

always_ff @(posedge clk)
  for (int c = 0; c < 4; c++)
    db_q[c*14 +: 14] <= {~din[c][13], din[c][12:0]};

assign fmc_dac_db = db_q;

for (genvar c = 0; c < 4; c++) begin : g_ch
  ODDRE1 i_clk (.Q (fmc_dac_clk[c]), .C (clk), .D1 (1'b0), .D2 (1'b1), .SR (1'b0));
  ODDRE1 i_wrt (.Q (fmc_dac_wrt[c]), .C (clk), .D1 (1'b0), .D2 (1'b1), .SR (1'b0));
end

endmodule
