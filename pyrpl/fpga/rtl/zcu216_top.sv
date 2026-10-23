/**
 * ZCU216 (XCZU49DR RFSoC) top level.
 *
 * Phase 1 of the Red Pitaya port: the processing system and a heartbeat on the
 * eight green RGB LEDs, to prove the board build flow end to end. The LEDs show
 * a binary count of the 100 MHz PL clock: LED 0 toggles every 0.67 s, LED 7
 * every 86 s. All dark means the PS never started pl_clk0 or holds the reset.
 */

module zcu216_top (
  output logic [7:0] led_g
);

logic clk, rstn;

system_zcu216_wrapper i_ps (
  .pl_clk0    (clk ),
  .pl_resetn0 (rstn)
);

logic [33:0] cnt;

always_ff @(posedge clk)
  if (!rstn) cnt <= '0;
  else       cnt <= cnt + 1'b1;

assign led_g = cnt[33:26];

endmodule
