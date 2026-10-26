/**
 * ZCU216 (XCZU49DR RFSoC) top level.
 *
 * Phase 2 of the Red Pitaya port: the processor reaches the Red Pitaya
 * register bus. M_AXI_HPM0_LPD (0x8000_0000, 16 MB, 32 bits) feeds the Red
 * Pitaya axi_slave, and the bus is split into the same eight 2 MB slots as
 * red_pitaya_top.v, so every register keeps its Red Pitaya offset:
 *
 *   slot 0  0x8000_0000  hk (ID, DNA, LEDs; 0x40 = board 1, 0x44 = DNA[95:64])
 *   slot 1  0x8020_0000  FFT bring-up block (zcu216_fft_test), when FFT_EN
 *   slot 2-7             not built yet: reads return 0, writes are ignored
 *
 * The bus runs on pl_clk0 (100 MHz). The FFT runs on fclk, 312.5 MHz from an
 * MMCM on pl_clk0 for now (the RF-ADC tile clock replaces it later): 8 samples
 * per clock is 2.5 GS/s. LEDs 0-6 are hk's LED register (0x30); LED 7 is a
 * heartbeat that toggles every 0.67 s, so a running PL clock is visible before
 * any register access.
 */

module zcu216_top #(
  parameter bit    FFT_EN   = 1,
  parameter int    FFT_NFFT = 18,     // log2 of the real FFT length
  parameter int    FFT_NCH  = 2,
  parameter int    FFT_WIN  = 4096,
  parameter string TWC_FILE = "out/zcu216_twc.mem",
  parameter string TWF_FILE = "out/zcu216_twf.mem",
  parameter string SIN_FILE = "out/zcu216_sin.mem"
) (
  output logic [7:0] led_g
);

////////////////////////////////////////////////////////////////////////////////
// processing system
////////////////////////////////////////////////////////////////////////////////

logic          clk, ps_rstn;

logic [40-1:0] gp0_araddr , gp0_awaddr ;
logic [ 2-1:0] gp0_arburst, gp0_awburst;
logic [ 4-1:0] gp0_arcache, gp0_awcache;
logic [16-1:0] gp0_arid   , gp0_awid   ;
logic [ 8-1:0] gp0_arlen  , gp0_awlen  ;
logic          gp0_arlock , gp0_awlock ;
logic [ 3-1:0] gp0_arprot , gp0_awprot ;
logic [ 3-1:0] gp0_arsize , gp0_awsize ;
logic          gp0_arvalid, gp0_awvalid;
logic          gp0_arready, gp0_awready;
logic [32-1:0] gp0_wdata  ;
logic [ 4-1:0] gp0_wstrb  ;
logic          gp0_wlast  , gp0_wvalid , gp0_wready;
logic [16-1:0] gp0_bid    ;
logic [ 2-1:0] gp0_bresp  ;
logic          gp0_bvalid , gp0_bready ;
logic [16-1:0] gp0_rid    ;
logic [32-1:0] gp0_rdata  ;
logic [ 2-1:0] gp0_rresp  ;
logic          gp0_rlast  , gp0_rvalid , gp0_rready;

system_zcu216_wrapper i_ps (
  .pl_clk0            (clk        ),
  .pl_resetn0         (ps_rstn    ),
  .M_AXI_GP0_araddr   (gp0_araddr ),  .M_AXI_GP0_awaddr   (gp0_awaddr ),
  .M_AXI_GP0_arburst  (gp0_arburst),  .M_AXI_GP0_awburst  (gp0_awburst),
  .M_AXI_GP0_arcache  (gp0_arcache),  .M_AXI_GP0_awcache  (gp0_awcache),
  .M_AXI_GP0_arid     (gp0_arid   ),  .M_AXI_GP0_awid     (gp0_awid   ),
  .M_AXI_GP0_arlen    (gp0_arlen  ),  .M_AXI_GP0_awlen    (gp0_awlen  ),
  .M_AXI_GP0_arlock   (gp0_arlock ),  .M_AXI_GP0_awlock   (gp0_awlock ),
  .M_AXI_GP0_arprot   (gp0_arprot ),  .M_AXI_GP0_awprot   (gp0_awprot ),
  .M_AXI_GP0_arqos    (           ),  .M_AXI_GP0_awqos    (           ),
  .M_AXI_GP0_arsize   (gp0_arsize ),  .M_AXI_GP0_awsize   (gp0_awsize ),
  .M_AXI_GP0_aruser   (           ),  .M_AXI_GP0_awuser   (           ),
  .M_AXI_GP0_arvalid  (gp0_arvalid),  .M_AXI_GP0_awvalid  (gp0_awvalid),
  .M_AXI_GP0_arready  (gp0_arready),  .M_AXI_GP0_awready  (gp0_awready),
  .M_AXI_GP0_wdata    (gp0_wdata  ),
  .M_AXI_GP0_wstrb    (gp0_wstrb  ),
  .M_AXI_GP0_wlast    (gp0_wlast  ),
  .M_AXI_GP0_wvalid   (gp0_wvalid ),
  .M_AXI_GP0_wready   (gp0_wready ),
  .M_AXI_GP0_bid      (gp0_bid    ),
  .M_AXI_GP0_bresp    (gp0_bresp  ),
  .M_AXI_GP0_bvalid   (gp0_bvalid ),
  .M_AXI_GP0_bready   (gp0_bready ),
  .M_AXI_GP0_rid      (gp0_rid    ),
  .M_AXI_GP0_rdata    (gp0_rdata  ),
  .M_AXI_GP0_rresp    (gp0_rresp  ),
  .M_AXI_GP0_rlast    (gp0_rlast  ),
  .M_AXI_GP0_rvalid   (gp0_rvalid ),
  .M_AXI_GP0_rready   (gp0_rready )
);

////////////////////////////////////////////////////////////////////////////////
// AXI4 -> Red Pitaya system bus
////////////////////////////////////////////////////////////////////////////////

// axi_slave speaks AXI3 and takes single 32-bit beats only. The 8-bit AXI4
// length is folded into its 4-bit one with the high bits OR-ed in, so an AXI4
// burst longer than 16 beats is still refused rather than truncated. AXI4 has
// no write ID; axi_slave answers with the address ID, so WID is not needed.

logic          sys_clk, sys_rstn;
logic [32-1:0] ps_sys_addr, ps_sys_wdata, ps_sys_rdata;
logic [ 4-1:0] ps_sys_sel;
logic          ps_sys_wen, ps_sys_ren, ps_sys_err, ps_sys_ack;

assign sys_clk = clk;

always_ff @(posedge clk)
  sys_rstn <= ps_rstn;

axi_slave #(
  .AXI_DW (32),
  .AXI_AW (32),
  .AXI_IW (16)
) i_axi_slave (
  .axi_clk_i      (sys_clk                                     ),
  .axi_rstn_i     (sys_rstn                                    ),
  .axi_awid_i     (gp0_awid                                    ),
  .axi_awaddr_i   (gp0_awaddr[31:0]                            ),
  .axi_awlen_i    ({gp0_awlen[3:1], gp0_awlen[0] | |gp0_awlen[7:4]}),
  .axi_awsize_i   (gp0_awsize                                  ),
  .axi_awburst_i  (gp0_awburst                                 ),
  .axi_awlock_i   ({1'b0, gp0_awlock}                          ),
  .axi_awcache_i  (gp0_awcache                                 ),
  .axi_awprot_i   (gp0_awprot                                  ),
  .axi_awvalid_i  (gp0_awvalid                                 ),
  .axi_awready_o  (gp0_awready                                 ),
  .axi_wid_i      (16'h0                                       ),
  .axi_wdata_i    (gp0_wdata                                   ),
  .axi_wstrb_i    (gp0_wstrb                                   ),
  .axi_wlast_i    (gp0_wlast                                   ),
  .axi_wvalid_i   (gp0_wvalid                                  ),
  .axi_wready_o   (gp0_wready                                  ),
  .axi_bid_o      (gp0_bid                                     ),
  .axi_bresp_o    (gp0_bresp                                   ),
  .axi_bvalid_o   (gp0_bvalid                                  ),
  .axi_bready_i   (gp0_bready                                  ),
  .axi_arid_i     (gp0_arid                                    ),
  .axi_araddr_i   (gp0_araddr[31:0]                            ),
  .axi_arlen_i    ({gp0_arlen[3:1], gp0_arlen[0] | |gp0_arlen[7:4]}),
  .axi_arsize_i   (gp0_arsize                                  ),
  .axi_arburst_i  (gp0_arburst                                 ),
  .axi_arlock_i   ({1'b0, gp0_arlock}                          ),
  .axi_arcache_i  (gp0_arcache                                 ),
  .axi_arprot_i   (gp0_arprot                                  ),
  .axi_arvalid_i  (gp0_arvalid                                 ),
  .axi_arready_o  (gp0_arready                                 ),
  .axi_rid_o      (gp0_rid                                     ),
  .axi_rdata_o    (gp0_rdata                                   ),
  .axi_rresp_o    (gp0_rresp                                   ),
  .axi_rlast_o    (gp0_rlast                                   ),
  .axi_rvalid_o   (gp0_rvalid                                  ),
  .axi_rready_i   (gp0_rready                                  ),
  .sys_addr_o     (ps_sys_addr                                 ),
  .sys_wdata_o    (ps_sys_wdata                                ),
  .sys_sel_o      (ps_sys_sel                                  ),
  .sys_wen_o      (ps_sys_wen                                  ),
  .sys_ren_o      (ps_sys_ren                                  ),
  .sys_rdata_i    (ps_sys_rdata                                ),
  .sys_err_i      (ps_sys_err                                  ),
  .sys_ack_i      (ps_sys_ack                                  )
);

////////////////////////////////////////////////////////////////////////////////
// system bus decoder & multiplexer (8 regions of 2 MB, as red_pitaya_top.v)
////////////////////////////////////////////////////////////////////////////////

logic [8   -1:0] sys_cs, sys_wen, sys_ren, sys_err, sys_ack;
logic [8*32-1:0] sys_rdata;

assign sys_cs  = 8'h01 << ps_sys_addr[23:21];
assign sys_wen = sys_cs & {8{ps_sys_wen}};
assign sys_ren = sys_cs & {8{ps_sys_ren}};

assign ps_sys_rdata = sys_rdata[ps_sys_addr[23:21]*32+:32];
assign ps_sys_err   = |(sys_cs & sys_err);
assign ps_sys_ack   = |(sys_cs & sys_ack);

generate
for (genvar i = 2; i < 8; i++) begin : g_empty_slot
  assign sys_rdata[i*32+:32] = 32'h0;
  assign sys_err  [i       ] = 1'b0;
  assign sys_ack  [i       ] = 1'b1;
end
endgenerate

////////////////////////////////////////////////////////////////////////////////
// housekeeping (slot 0)
////////////////////////////////////////////////////////////////////////////////

logic [8-1:0] hk_led;

red_pitaya_hk #(
  .DNA_W (96),
  .BOARD (4'h1)
) i_hk (
  .clk_i         (sys_clk           ),
  .rstn_i        (sys_rstn          ),
  .led_o         (hk_led            ),
  .digital_loop  (                  ),
  .exp_p_dat_i   (8'h0              ),
  .exp_p_dat_o   (                  ),
  .exp_p_dir_o   (                  ),
  .exp_n_dat_i   (8'h0              ),
  .exp_n_dat_o   (                  ),
  .exp_n_dir_o   (                  ),
  .scope_sigs_i  (4'h0              ),
  .scan_x_i      ('0                ),
  .scan_x_step_i ('0                ),
  .scan_y_i      ('0                ),
  .scan_y_step_i ('0                ),
  .sys_addr      (ps_sys_addr       ),
  .sys_wdata     (ps_sys_wdata      ),
  .sys_sel       (ps_sys_sel        ),
  .sys_wen       (sys_wen[0]        ),
  .sys_ren       (sys_ren[0]        ),
  .sys_rdata     (sys_rdata[0*32+:32]),
  .sys_err       (sys_err[0]        ),
  .sys_ack       (sys_ack[0]        )
);

////////////////////////////////////////////////////////////////////////////////
// FFT bring-up (slot 1) on fclk = 312.5 MHz: pl_clk0 x 12.5 / 4 (VCO 1250 MHz)
////////////////////////////////////////////////////////////////////////////////

if (FFT_EN) begin : g_fft
  logic fb, fb_buf, fclk_mmcm, fclk, locked;

  MMCME4_BASE #(
    .CLKIN1_PERIOD    (10.0 ),
    .DIVCLK_DIVIDE    (1    ),
    .CLKFBOUT_MULT_F  (12.5 ),
    .CLKOUT0_DIVIDE_F (4.0  )
  ) i_mmcm (
    .CLKIN1    (clk       ),
    .CLKFBIN   (fb_buf    ),
    .CLKFBOUT  (fb        ),
    .CLKFBOUTB (          ),
    .CLKOUT0   (fclk_mmcm ),
    .CLKOUT0B  (          ),
    .CLKOUT1   (          ),  .CLKOUT1B  (          ),
    .CLKOUT2   (          ),  .CLKOUT2B  (          ),
    .CLKOUT3   (          ),  .CLKOUT3B  (          ),
    .CLKOUT4   (          ),  .CLKOUT5   (          ),
    .CLKOUT6   (          ),
    .LOCKED    (locked    ),
    .PWRDWN    (1'b0      ),
    .RST       (!ps_rstn  )
  );

  BUFG i_fb_buf  (.I (fb       ), .O (fb_buf));
  BUFG i_clk_buf (.I (fclk_mmcm), .O (fclk  ));

  // fclk reset: held while the MMCM is unlocked or the bus is in reset
  (* ASYNC_REG = "TRUE" *) logic [2:0] frst_s;
  logic frstn;
  always_ff @(posedge fclk or negedge locked)
    if (!locked) frst_s <= '0;
    else         frst_s <= {frst_s[1:0], sys_rstn};
  always_ff @(posedge fclk) frstn <= frst_s[2];

  zcu216_fft_test #(
    .NCH      (FFT_NCH       ),
    .SUB      (FFT_NFFT - 3  ),
    .WIN      (FFT_WIN       ),
    .TWC_FILE (TWC_FILE      ),
    .TWF_FILE (TWF_FILE      ),
    .SIN_FILE (SIN_FILE      )
  ) i_fft (
    .sys_clk   (sys_clk            ),
    .sys_rstn  (sys_rstn           ),
    .sys_addr  (ps_sys_addr        ),
    .sys_wdata (ps_sys_wdata       ),
    .sys_wen   (sys_wen[1]         ),
    .sys_ren   (sys_ren[1]         ),
    .sys_rdata (sys_rdata[1*32+:32]),
    .sys_err   (sys_err[1]         ),
    .sys_ack   (sys_ack[1]         ),
    .fclk      (fclk               ),
    .frstn     (frstn              ),
    .flocked   (locked             )
  );
end else begin : g_no_fft
  assign sys_rdata[1*32+:32] = 32'h0;
  assign sys_err  [1]        = 1'b0;
  assign sys_ack  [1]        = 1'b1;
end

////////////////////////////////////////////////////////////////////////////////
// LEDs
////////////////////////////////////////////////////////////////////////////////

logic [26:0] beat;

always_ff @(posedge clk)
  if (!sys_rstn) beat <= '0;
  else           beat <= beat + 1'b1;

assign led_g = {beat[26], hk_led[6:0]};

endmodule
