/**
 * ZCU216 (XCZU49DR RFSoC) top level.
 *
 * The processor reaches the Red Pitaya register bus: M_AXI_HPM0_LPD (0x8000_0000,
 * 16 MB, 32 bits) feeds the Red Pitaya axi_slave, and the bus is split into the
 * same eight 2 MB slots as red_pitaya_top.v, so every register keeps its Red
 * Pitaya offset (Red Pitaya address = ZCU216 address - 0x4000_0000):
 *
 *   slot 0  0x8000_0000  hk (ID, DNA, LEDs; 0x40 = board 1, 0x44 = DNA[95:64])
 *   slot 1  0x8020_0000  FFT / RF bring-up block (zcu216_fft_test)        [RF]
 *                        (CFAR detectors, frame snapshots into DDR through
 *                        S_AXI_HP0/HP1, the oscilloscope through S_AXI_HP2 +
 *                        S_AXI_HP3, the point stream through S_AXI_HPC0 (a
 *                        few hundred KB/s), all on fclk)
 *   slot 2  0x8040_0000  RF Data Converter registers (AXI4-Lite, 256 KB)  [RF]
 *   slot 3  0x8060_0000  signal hub (zcu216_dsp), at the Red Pitaya DSP's
 *                        addresses: the signals' numbers, the sinks' selects,
 *                        asgN.output_direct; out1-4 = the FMC DAC's channels
 *   slot 4  0x8080_0000  ASG (red_pitaya_asg, the Red Pitaya's slot-2 registers):
 *                        four channels on sys_clk (125 MS/s); the scope sees
 *                        them as inputs 8-11 [RF]
 *   slot 5-7             not built yet: reads return 0, writes are ignored
 *
 * [RF] (define ZCU216_RF, set by zcu216_vivado.tcl unless FFT_IMPL=0): the RF
 * Data Converter (ip/rfdc_zcu216.tcl: ADC tile 224 and DAC tile 228 at 2.5 GS/s)
 * and the FFT. Their clock, fclk = 312.5 MHz = 2.5 GS/s / 8, comes from the
 * RF-ADC tile's own output clock clk_adc0 (through an MMCM), as in AMD's RF
 * DC evaluation design, so it is frequency-locked to the converters' FIFOs.
 * (Taking it from the CLK104 PL_CLK instead left the tile FIFOs flagging
 * overflow and underflow, with word slips in the data.) Nothing runs there
 * until zcu216/clk104 has programmed the CLK104 and the tiles are up; the MMCM
 * retries its lock until then.
 *
 * [FMC_DAC] (define ZCU216_FMC_DAC, set by zcu216_vivado.tcl unless FMC_DAC=0):
 * the PZ-FL9767D DAC card on FMC+ J28, fed by the output routing on sys_clk.
 *
 * Clocks: the modules on the register bus run on sys_clk, 125 MHz from the RF
 * clock as the Red Pitaya's run on its ADC clock (pl_clk0, 100 MHz, without
 * RF); the AXI front end, slot 2 and the LED heartbeat on pl_clk0, the bus
 * crossing through zcu216_bus_cdc. LEDs 0-6 are hk's LED register (0x30);
 * LED 7 is a heartbeat that toggles every 0.67 s, so a running PL clock is
 * visible before any register access.
 *
 * hk's expansion GPIOs are on the PMOD headers: exp_p[7:0] = PMOD0_0..7 (J46),
 * exp_n[7:0] = PMOD1_0..7 (J47), direction and data per pin as on the Red
 * Pitaya (hk 0x10-0x24). The pins are 1.8 V (bank 88) behind TXS0108E
 * auto-direction shifters to the 3.3 V headers: weak drive, no LEDs or heavy
 * loads, no strong pull-ups. Jumper J27 must stay open (it ties PMOD1_0/1 to
 * the MSP430's reset and test pins).
 */

module zcu216_top #(
  parameter int    FFT_NFFT = 16,     // log2 of the real FFT length
  parameter int    FFT_NCH  = 2,
  parameter int    FFT_WIN  = 4096,
  parameter string TWC_FILE = "out/zcu216_twc.mem",
  parameter string TWF_FILE = "out/zcu216_twf.mem",
  parameter string SIN_FILE = "out/zcu216_sin.mem"
) (
`ifdef ZCU216_RF
  // CLK104
  input  logic       pl_clk_p,  pl_clk_n,      // PL_CLK, 125 MHz LVDS
  output logic [1:0] spi_sdo_sel,              // which chip drives the SPI readback
  input  logic       adc2_clk_p, adc2_clk_n,   // ADC_REFCLK -> tile 226
  input  logic       dac2_clk_p, dac2_clk_n,   // DAC_REFCLK -> tile 230
  input  logic       sysref_in_p, sysref_in_n,
  // RF-ADC / RF-DAC pins (dedicated)
  input  logic       vin00_p, vin00_n, vin01_p, vin01_n, vin02_p, vin02_n,
  input  logic       vin03_p, vin03_n, vin10_p, vin10_n, vin20_p, vin20_n,
  output logic       vout00_p, vout00_n, vout01_p, vout01_n, vout02_p, vout02_n,
  output logic       vout03_p, vout03_n, vout10_p, vout10_n, vout20_p, vout20_n,
`endif
`ifdef ZCU216_FMC_DAC
  // the PZ-FL9767D DAC card on FMC+ J28 (zcu216_fmc_dac)
  output logic [55:0] fmc_dac_db,
  output logic [3:0]  fmc_dac_clk,
  output logic [3:0]  fmc_dac_wrt,
`endif
  output logic [7:0] led_g,
  inout  wire  [7:0] pmod0,                    // hk exp_p (J46)
  inout  wire  [7:0] pmod1                     // hk exp_n (J47)
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

`ifdef ZCU216_RF
// snapshot DMA (zcu216_snap) into DDR: two AXI4 write ports, fclk
logic         fclk;
logic [48:0]  hp_awaddr  [2];
logic [7:0]   hp_awlen   [2];
logic [2:0]   hp_awsize  [2];
logic [1:0]   hp_awburst [2];
logic [3:0]   hp_awcache [2];
logic [2:0]   hp_awprot  [2];
logic [1:0]   hp_awvalid, hp_awready, hp_wlast, hp_wvalid, hp_wready;
logic [1:0]   hp_bvalid, hp_bready;
logic [127:0] hp_wdata   [2];
logic [15:0]  hp_wstrb   [2];
logic [1:0]   hp_bresp   [2];
// point stream (zcu216_points) into the DDR ring: one AXI4 write port (HPC0), fclk
logic [48:0]  pt_awaddr;
logic [7:0]   pt_awlen;
logic [2:0]   pt_awsize;
logic [1:0]   pt_awburst;
logic [3:0]   pt_awcache;
logic [2:0]   pt_awprot;
logic         pt_awvalid, pt_awready, pt_wlast, pt_wvalid, pt_wready, pt_bvalid, pt_bready;
logic [127:0] pt_wdata;
logic [15:0]  pt_wstrb;
logic [1:0]   pt_bresp;
// oscilloscope ring (zcu216_scope): two AXI4 write ports (HP3, HP2), fclk
logic [48:0]  sc_awaddr  [2];
logic [7:0]   sc_awlen   [2];
logic [2:0]   sc_awsize  [2];
logic [1:0]   sc_awburst [2];
logic [3:0]   sc_awcache [2];
logic [2:0]   sc_awprot  [2];
logic [1:0]   sc_awvalid, sc_awready, sc_wlast, sc_wvalid, sc_wready, sc_bvalid, sc_bready;
logic [127:0] sc_wdata   [2];
logic [15:0]  sc_wstrb   [2];
logic [1:0]   sc_bresp   [2];
`endif

system_zcu216_wrapper i_ps (
  .pl_clk0            (clk        ),
  .pl_resetn0         (ps_rstn    ),
`ifdef ZCU216_RF
  .hp_clk             (fclk       ),
  .S_AXI_HP0_awaddr   (hp_awaddr[0] ),  .S_AXI_HP1_awaddr   (hp_awaddr[1] ),
  .S_AXI_HP0_awlen    (hp_awlen[0]  ),  .S_AXI_HP1_awlen    (hp_awlen[1]  ),
  .S_AXI_HP0_awsize   (hp_awsize[0] ),  .S_AXI_HP1_awsize   (hp_awsize[1] ),
  .S_AXI_HP0_awburst  (hp_awburst[0]),  .S_AXI_HP1_awburst  (hp_awburst[1]),
  .S_AXI_HP0_awcache  (hp_awcache[0]),  .S_AXI_HP1_awcache  (hp_awcache[1]),
  .S_AXI_HP0_awprot   (hp_awprot[0] ),  .S_AXI_HP1_awprot   (hp_awprot[1] ),
  .S_AXI_HP0_awid     (6'd0         ),  .S_AXI_HP1_awid     (6'd0         ),
  .S_AXI_HP0_awlock   (1'b0         ),  .S_AXI_HP1_awlock   (1'b0         ),
  .S_AXI_HP0_awqos    (4'd0         ),  .S_AXI_HP1_awqos    (4'd0         ),
  .S_AXI_HP0_awuser   (1'b0         ),  .S_AXI_HP1_awuser   (1'b0         ),
  .S_AXI_HP0_awvalid  (hp_awvalid[0]),  .S_AXI_HP1_awvalid  (hp_awvalid[1]),
  .S_AXI_HP0_awready  (hp_awready[0]),  .S_AXI_HP1_awready  (hp_awready[1]),
  .S_AXI_HP0_wdata    (hp_wdata[0]  ),  .S_AXI_HP1_wdata    (hp_wdata[1]  ),
  .S_AXI_HP0_wstrb    (hp_wstrb[0]  ),  .S_AXI_HP1_wstrb    (hp_wstrb[1]  ),
  .S_AXI_HP0_wlast    (hp_wlast[0]  ),  .S_AXI_HP1_wlast    (hp_wlast[1]  ),
  .S_AXI_HP0_wvalid   (hp_wvalid[0] ),  .S_AXI_HP1_wvalid   (hp_wvalid[1] ),
  .S_AXI_HP0_wready   (hp_wready[0] ),  .S_AXI_HP1_wready   (hp_wready[1] ),
  .S_AXI_HP0_bid      (             ),  .S_AXI_HP1_bid      (             ),
  .S_AXI_HP0_bresp    (hp_bresp[0]  ),  .S_AXI_HP1_bresp    (hp_bresp[1]  ),
  .S_AXI_HP0_bvalid   (hp_bvalid[0] ),  .S_AXI_HP1_bvalid   (hp_bvalid[1] ),
  .S_AXI_HP0_bready   (hp_bready[0] ),  .S_AXI_HP1_bready   (hp_bready[1] ),
  // write only: the read channels stay idle
  .S_AXI_HP0_araddr   ('0           ),  .S_AXI_HP1_araddr   ('0           ),
  .S_AXI_HP0_arburst  (2'b01        ),  .S_AXI_HP1_arburst  (2'b01        ),
  .S_AXI_HP0_arcache  (4'd0         ),  .S_AXI_HP1_arcache  (4'd0         ),
  .S_AXI_HP0_arid     (6'd0         ),  .S_AXI_HP1_arid     (6'd0         ),
  .S_AXI_HP0_arlen    (8'd0         ),  .S_AXI_HP1_arlen    (8'd0         ),
  .S_AXI_HP0_arlock   (1'b0         ),  .S_AXI_HP1_arlock   (1'b0         ),
  .S_AXI_HP0_arprot   (3'd0         ),  .S_AXI_HP1_arprot   (3'd0         ),
  .S_AXI_HP0_arqos    (4'd0         ),  .S_AXI_HP1_arqos    (4'd0         ),
  .S_AXI_HP0_arsize   (3'b100       ),  .S_AXI_HP1_arsize   (3'b100       ),
  .S_AXI_HP0_aruser   (1'b0         ),  .S_AXI_HP1_aruser   (1'b0         ),
  .S_AXI_HP0_arvalid  (1'b0         ),  .S_AXI_HP1_arvalid  (1'b0         ),
  .S_AXI_HP0_arready  (             ),  .S_AXI_HP1_arready  (             ),
  .S_AXI_HP0_rdata    (             ),  .S_AXI_HP1_rdata    (             ),
  .S_AXI_HP0_rid      (             ),  .S_AXI_HP1_rid      (             ),
  .S_AXI_HP0_rlast    (             ),  .S_AXI_HP1_rlast    (             ),
  .S_AXI_HP0_rresp    (             ),  .S_AXI_HP1_rresp    (             ),
  .S_AXI_HP0_rvalid   (             ),  .S_AXI_HP1_rvalid   (             ),
  .S_AXI_HP0_rready   (1'b1         ),  .S_AXI_HP1_rready   (1'b1         ),
  .S_AXI_HPC0_awaddr   (pt_awaddr    ),  .S_AXI_HPC0_awlen    (pt_awlen     ),
  .S_AXI_HPC0_awsize   (pt_awsize    ),  .S_AXI_HPC0_awburst  (pt_awburst   ),
  .S_AXI_HPC0_awcache  (pt_awcache   ),  .S_AXI_HPC0_awprot   (pt_awprot    ),
  .S_AXI_HPC0_awid     (6'd0         ),  .S_AXI_HPC0_awlock   (1'b0         ),
  .S_AXI_HPC0_awqos    (4'd0         ),  .S_AXI_HPC0_awuser   (1'b0         ),
  .S_AXI_HPC0_awvalid  (pt_awvalid   ),  .S_AXI_HPC0_awready  (pt_awready   ),
  .S_AXI_HPC0_wdata    (pt_wdata     ),  .S_AXI_HPC0_wstrb    (pt_wstrb     ),
  .S_AXI_HPC0_wlast    (pt_wlast     ),  .S_AXI_HPC0_wvalid   (pt_wvalid    ),
  .S_AXI_HPC0_wready   (pt_wready    ),  .S_AXI_HPC0_bid      (             ),
  .S_AXI_HPC0_bresp    (pt_bresp     ),  .S_AXI_HPC0_bvalid   (pt_bvalid    ),
  .S_AXI_HPC0_bready   (pt_bready    ),
  .S_AXI_HPC0_araddr   ('0           ),  .S_AXI_HPC0_arburst  (2'b01        ),
  .S_AXI_HPC0_arcache  (4'd0         ),  .S_AXI_HPC0_arid     (6'd0         ),
  .S_AXI_HPC0_arlen    (8'd0         ),  .S_AXI_HPC0_arlock   (1'b0         ),
  .S_AXI_HPC0_arprot   (3'd0         ),  .S_AXI_HPC0_arqos    (4'd0         ),
  .S_AXI_HPC0_arsize   (3'b100       ),  .S_AXI_HPC0_aruser   (1'b0         ),
  .S_AXI_HPC0_arvalid  (1'b0         ),  .S_AXI_HPC0_arready  (             ),
  .S_AXI_HPC0_rdata    (             ),  .S_AXI_HPC0_rid      (             ),
  .S_AXI_HPC0_rlast    (             ),  .S_AXI_HPC0_rresp    (             ),
  .S_AXI_HPC0_rvalid   (             ),  .S_AXI_HPC0_rready   (1'b1         ),
  .S_AXI_HP3_awaddr   (sc_awaddr[0] ),  .S_AXI_HP3_awlen    (sc_awlen[0]  ),
  .S_AXI_HP3_awsize   (sc_awsize[0] ),  .S_AXI_HP3_awburst  (sc_awburst[0]),
  .S_AXI_HP3_awcache  (sc_awcache[0]),  .S_AXI_HP3_awprot   (sc_awprot[0] ),
  .S_AXI_HP3_awid     (6'd0         ),  .S_AXI_HP3_awlock   (1'b0         ),
  .S_AXI_HP3_awqos    (4'd0         ),  .S_AXI_HP3_awuser   (1'b0         ),
  .S_AXI_HP3_awvalid  (sc_awvalid[0]),  .S_AXI_HP3_awready  (sc_awready[0]),
  .S_AXI_HP3_wdata    (sc_wdata[0]  ),  .S_AXI_HP3_wstrb    (sc_wstrb[0]  ),
  .S_AXI_HP3_wlast    (sc_wlast[0]  ),  .S_AXI_HP3_wvalid   (sc_wvalid[0] ),
  .S_AXI_HP3_wready   (sc_wready[0] ),  .S_AXI_HP3_bid      (             ),
  .S_AXI_HP3_bresp    (sc_bresp[0]  ),  .S_AXI_HP3_bvalid   (sc_bvalid[0] ),
  .S_AXI_HP3_bready   (sc_bready[0] ),
  .S_AXI_HP3_araddr   ('0           ),  .S_AXI_HP3_arburst  (2'b01        ),
  .S_AXI_HP3_arcache  (4'd0         ),  .S_AXI_HP3_arid     (6'd0         ),
  .S_AXI_HP3_arlen    (8'd0         ),  .S_AXI_HP3_arlock   (1'b0         ),
  .S_AXI_HP3_arprot   (3'd0         ),  .S_AXI_HP3_arqos    (4'd0         ),
  .S_AXI_HP3_arsize   (3'b100       ),  .S_AXI_HP3_aruser   (1'b0         ),
  .S_AXI_HP3_arvalid  (1'b0         ),  .S_AXI_HP3_arready  (             ),
  .S_AXI_HP3_rdata    (             ),  .S_AXI_HP3_rid      (             ),
  .S_AXI_HP3_rlast    (             ),  .S_AXI_HP3_rresp    (             ),
  .S_AXI_HP3_rvalid   (             ),  .S_AXI_HP3_rready   (1'b1         ),
  .S_AXI_HP2_awaddr   (sc_awaddr[1] ),  .S_AXI_HP2_awlen    (sc_awlen[1]  ),
  .S_AXI_HP2_awsize   (sc_awsize[1] ),  .S_AXI_HP2_awburst  (sc_awburst[1]),
  .S_AXI_HP2_awcache  (sc_awcache[1]),  .S_AXI_HP2_awprot   (sc_awprot[1] ),
  .S_AXI_HP2_awid     (6'd0         ),  .S_AXI_HP2_awlock   (1'b0         ),
  .S_AXI_HP2_awqos    (4'd0         ),  .S_AXI_HP2_awuser   (1'b0         ),
  .S_AXI_HP2_awvalid  (sc_awvalid[1]),  .S_AXI_HP2_awready  (sc_awready[1]),
  .S_AXI_HP2_wdata    (sc_wdata[1]  ),  .S_AXI_HP2_wstrb    (sc_wstrb[1]  ),
  .S_AXI_HP2_wlast    (sc_wlast[1]  ),  .S_AXI_HP2_wvalid   (sc_wvalid[1] ),
  .S_AXI_HP2_wready   (sc_wready[1] ),  .S_AXI_HP2_bid      (             ),
  .S_AXI_HP2_bresp    (sc_bresp[1]  ),  .S_AXI_HP2_bvalid   (sc_bvalid[1] ),
  .S_AXI_HP2_bready   (sc_bready[1] ),
  .S_AXI_HP2_araddr   ('0           ),  .S_AXI_HP2_arburst  (2'b01        ),
  .S_AXI_HP2_arcache  (4'd0         ),  .S_AXI_HP2_arid     (6'd0         ),
  .S_AXI_HP2_arlen    (8'd0         ),  .S_AXI_HP2_arlock   (1'b0         ),
  .S_AXI_HP2_arprot   (3'd0         ),  .S_AXI_HP2_arqos    (4'd0         ),
  .S_AXI_HP2_arsize   (3'b100       ),  .S_AXI_HP2_aruser   (1'b0         ),
  .S_AXI_HP2_arvalid  (1'b0         ),  .S_AXI_HP2_arready  (             ),
  .S_AXI_HP2_rdata    (             ),  .S_AXI_HP2_rid      (             ),
  .S_AXI_HP2_rlast    (             ),  .S_AXI_HP2_rresp    (             ),
  .S_AXI_HP2_rvalid   (             ),  .S_AXI_HP2_rready   (1'b1         ),
`endif
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

logic [32-1:0] ps_sys_addr, ps_sys_wdata, ps_sys_rdata;
logic [ 4-1:0] ps_sys_sel;
logic          ps_sys_wen, ps_sys_ren, ps_sys_err, ps_sys_ack;

// front reset (clk = pl_clk0): axi_slave, the bridges' front ends
logic a_rstn;
always_ff @(posedge clk)
  a_rstn <= ps_rstn;

////////////////////////////////////////////////////////////////////////////////
// clocks: sys_clk = the system clock, 125 MHz from the RF clock
////////////////////////////////////////////////////////////////////////////////

// As on the Red Pitaya, where everything on the register bus runs on the ADC's
// 125 MHz: sys_clk = 1250 MHz / 10 from the MMCM that makes fclk (1250 / 4)
// out of the RF-ADC tile clock, so the ASG, the FMC DAC and the RF converters
// share one reference. hk, the ASG, the output routing, the FMC DAC and the
// FFT block's registers run on it. It only runs once the CLK104 is programmed
// and the tiles are up, so the PS reaches it through zcu216_bus_cdc (reads
// return 0 while it is down); the AXI front end and the RF Data Converter's
// registers (slot 2) stay on pl_clk0. Without RF, sys_clk = pl_clk0 (100 MHz).
logic sys_clk, sys_rstn;

`ifdef ZCU216_RF
logic pl_clk_ibuf, pl_clk_bufg, clk_adc0, adc_clk_bufg, fb, fb_buf, fclk_mmcm, sys_mmcm, locked;

IBUFDS i_pl_clk_ibuf (.I (pl_clk_p), .IB (pl_clk_n), .O (pl_clk_ibuf));
BUFG   i_pl_clk_bufg (.I (pl_clk_ibuf), .O (pl_clk_bufg));

// the tile clock only appears once the CLK104 is programmed and the tile is up:
// while unlocked, reset the MMCM every 1.3 ms (pl_clk0) so it locks soon after
(* ASYNC_REG = "TRUE" *) logic [1:0] locked_s;
logic [16:0] lock_cnt;
logic        mmcm_rst;

always_ff @(posedge clk) begin
  locked_s <= {locked_s[0], locked};
  if (locked_s[1]) lock_cnt <= '0;
  else             lock_cnt <= lock_cnt + 1'b1;
  mmcm_rst <= !locked_s[1] && lock_cnt[16:4] == '0;   // 16 cycles every 2^17
end

// clk_adc0 (312.5 MHz) x 4 = VCO 1250 MHz; / 4 = fclk, / 10 = sys_clk
MMCME4_BASE #(
  .CLKIN1_PERIOD    (3.2  ),
  .DIVCLK_DIVIDE    (1    ),
  .CLKFBOUT_MULT_F  (4.0  ),
  .CLKOUT0_DIVIDE_F (4.0  ),
  .CLKOUT1_DIVIDE   (10   )
) i_mmcm (
  .CLKIN1    (adc_clk_bufg),
  .CLKFBIN   (fb_buf    ),
  .CLKFBOUT  (fb        ),
  .CLKFBOUTB (          ),
  .CLKOUT0   (fclk_mmcm ),
  .CLKOUT0B  (          ),
  .CLKOUT1   (sys_mmcm  ),  .CLKOUT1B  (          ),
  .CLKOUT2   (          ),  .CLKOUT2B  (          ),
  .CLKOUT3   (          ),  .CLKOUT3B  (          ),
  .CLKOUT4   (          ),  .CLKOUT5   (          ),
  .CLKOUT6   (          ),
  .LOCKED    (locked    ),
  .PWRDWN    (1'b0      ),
  .RST       (mmcm_rst || !ps_rstn)
);

BUFG i_fb_buf  (.I (fb       ), .O (fb_buf ));
BUFG i_clk_buf (.I (fclk_mmcm), .O (fclk   ));
BUFG i_sys_buf (.I (sys_mmcm ), .O (sys_clk));

// fclk and sys_clk resets: held while the MMCM is unlocked or the PS is in reset
(* ASYNC_REG = "TRUE" *) logic [2:0] frst_s, srst_s;
logic frstn;
always_ff @(posedge fclk or negedge locked)
  if (!locked) frst_s <= '0;
  else         frst_s <= {frst_s[1:0], a_rstn};
always_ff @(posedge fclk) frstn <= frst_s[2];
always_ff @(posedge sys_clk or negedge locked)
  if (!locked) srst_s <= '0;
  else         srst_s <= {srst_s[1:0], a_rstn};
always_ff @(posedge sys_clk) sys_rstn <= srst_s[2];
`else
assign sys_clk = clk;
always_ff @(posedge clk)
  sys_rstn <= ps_rstn;
`endif

axi_slave #(
  .AXI_DW (32),
  .AXI_AW (32),
  .AXI_IW (16),
  .ACK_W  (10)                // RF Data Converter accesses take longer than 32 cycles
) i_axi_slave (
  .axi_clk_i      (clk                                         ),
  .axi_rstn_i     (a_rstn                                      ),
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

// Front (clk): slot 2, the RF Data Converter's registers, is answered here, so
// the tiles can be restarted while sys_clk is down; every other slot goes
// through zcu216_bus_cdc to the back (sys_clk).
logic          f_rf;                              // the access is to slot 2
logic [32-1:0] rf_sys_rdata, cdc_rdata;
logic          rf_sys_err, rf_sys_ack, cdc_err, cdc_ack;

assign f_rf         = ps_sys_addr[23:21] == 3'd2;
assign ps_sys_rdata = f_rf ? rf_sys_rdata : cdc_rdata;
assign ps_sys_err   = f_rf ? rf_sys_err   : cdc_err;
assign ps_sys_ack   = f_rf ? rf_sys_ack   : cdc_ack;

logic [32-1:0] bk_addr, bk_wdata;                 // the back bus (sys_clk)
logic [ 4-1:0] bk_sel;
logic          bk_wen, bk_ren, bk_err, bk_ack;
logic [32-1:0] bk_rdata;

zcu216_bus_cdc i_bus_cdc (
  .clk_a   (clk                    ), .rstn_a  (a_rstn                 ),
  .a_addr  (ps_sys_addr            ), .a_wdata (ps_sys_wdata           ),
  .a_sel   (ps_sys_sel             ),
  .a_wen   (ps_sys_wen && !f_rf    ), .a_ren   (ps_sys_ren && !f_rf    ),
  .a_rdata (cdc_rdata              ), .a_err   (cdc_err                ),
  .a_ack   (cdc_ack                ),
  .clk_b   (sys_clk                ), .rstn_b  (sys_rstn               ),
  .b_addr  (bk_addr                ), .b_wdata (bk_wdata               ),
  .b_sel   (bk_sel                 ),
  .b_wen   (bk_wen                 ), .b_ren   (bk_ren                 ),
  .b_rdata (bk_rdata               ), .b_err   (bk_err                 ),
  .b_ack   (bk_ack                 )
);

// back (sys_clk): the slot decoder, as red_pitaya_top.v
logic [8   -1:0] sys_cs, sys_wen, sys_ren, sys_err, sys_ack;
logic [8*32-1:0] sys_rdata;

assign sys_cs  = 8'h01 << bk_addr[23:21];
assign sys_wen = sys_cs & {8{bk_wen}};
assign sys_ren = sys_cs & {8{bk_ren}};

assign bk_rdata = sys_rdata[bk_addr[23:21]*32+:32];
assign bk_err   = |(sys_cs & sys_err);
assign bk_ack   = |(sys_cs & sys_ack);

generate
for (genvar i = 2; i < 8; i++) begin : g_empty_slot
  if (i != 3 && i != 4) begin : g_slot
    assign sys_rdata[i*32+:32] = 32'h0;
    assign sys_err  [i       ] = 1'b0;
    assign sys_ack  [i       ] = 1'b1;
  end
end
endgenerate

////////////////////////////////////////////////////////////////////////////////
// housekeeping (slot 0)
////////////////////////////////////////////////////////////////////////////////

logic [8-1:0] hk_led;

// PMOD pins: IOBUFs, inputs through two flops (asynchronous to the bus)
logic [7:0] exp_p_in, exp_n_in, exp_p_out, exp_n_out, exp_p_dir, exp_n_dir;
(* ASYNC_REG = "TRUE" *) logic [7:0] exp_p_m, exp_p_s, exp_n_m, exp_n_s;

for (genvar i = 0; i < 8; i++) begin : g_pmod
  IOBUF i_iobuf_p (.O(exp_p_in[i]), .IO(pmod0[i]), .I(exp_p_out[i]), .T(~exp_p_dir[i]));
  IOBUF i_iobuf_n (.O(exp_n_in[i]), .IO(pmod1[i]), .I(exp_n_out[i]), .T(~exp_n_dir[i]));
end

always_ff @(posedge sys_clk) begin
  exp_p_m <= exp_p_in;  exp_p_s <= exp_p_m;
  exp_n_m <= exp_n_in;  exp_n_s <= exp_n_m;
end

// hk capabilities (red_pitaya_hk.v 0x48..0x5C): RF-ADC/DAC 2.5 GS/s, 14 bits,
// four channels each (ADC tile 224, DAC tile 228); the bus modules on sys_clk.
// Modules: asg0-3 always; the scope (DDR ring) and the RF FFT block with RF;
// bit 28 = the ASG drives the FMC DAC card, 29 = the signal hub.
`ifdef ZCU216_FMC_DAC
localparam logic [31:0] HK_ASG = (32'hF << 3) | (32'h1 << 28) | (32'h1 << 29);
`else
localparam logic [31:0] HK_ASG = (32'hF << 3) | (32'h1 << 29);
`endif
`ifdef ZCU216_RF
localparam logic [31:0] HK_MODULES = HK_ASG | (32'h1 << 1) | (32'h1 << 26) | (32'h1 << 27);
`else
localparam logic [31:0] HK_MODULES = HK_ASG;
`endif

`ifdef ZCU216_RF
localparam logic [31:0] SYS_HZ = 32'd125000000;
`else
localparam logic [31:0] SYS_HZ = 32'd100000000;
`endif

red_pitaya_hk #(
  .DNA_W   (96),
  .BOARD   (4'h1),
  .ADC_HZ  (32'd2500000000),
  .DAC_HZ  (32'd2500000000),
  .SYS_HZ  (SYS_HZ        ),
  .CONV    (32'h040E_040E),
  .MODULES (HK_MODULES)
) i_hk (
  .clk_i         (sys_clk           ),
  .rstn_i        (sys_rstn          ),
  .led_o         (hk_led            ),
  .digital_loop  (                  ),
  .exp_p_dat_i   (exp_p_s           ),
  .exp_p_dat_o   (exp_p_out         ),
  .exp_p_dir_o   (exp_p_dir         ),
  .exp_n_dat_i   (exp_n_s           ),
  .exp_n_dat_o   (exp_n_out         ),
  .exp_n_dir_o   (exp_n_dir         ),
  .scope_sigs_i  (4'h0              ),
  .scan_x_i      ('0                ),
  .scan_x_step_i ('0                ),
  .scan_y_i      ('0                ),
  .scan_y_step_i ('0                ),
  .sys_addr      (bk_addr           ),
  .sys_wdata     (bk_wdata          ),
  .sys_sel       (bk_sel            ),
  .sys_wen       (sys_wen[0]        ),
  .sys_ren       (sys_ren[0]        ),
  .sys_rdata     (sys_rdata[0*32+:32]),
  .sys_err       (sys_err[0]        ),
  .sys_ack       (sys_ack[0]        )
);

////////////////////////////////////////////////////////////////////////////////
// arbitrary signal generator (slot 4)
////////////////////////////////////////////////////////////////////////////////

// red_pitaya_asg as on the Red Pitaya: four channels, 16k-sample tables, the
// same registers (Red Pitaya slot 2, here slot 4: slot 2 is the RF Data
// Converter). It runs on sys_clk, 125 MS/s as on the Red Pitaya (100 without
// RF; hk 0x54 tells pyrpl). Its outputs reach the scope as inputs 8-11 and,
// through the output routing (slot 3), the FMC DAC card. The external trigger
// is PMOD0_0, as DIO0_P is on the Red Pitaya.
logic [13:0] asg_out [4];
logic [3:0]  asg_trig;                            // chirp starts, one sys_clk each
logic [3:0]  asg_tog = '0;                        // ... as toggles, for fclk
logic        sc_trig_p;                           // the scope's trigger, one sys_clk

always_ff @(posedge sys_clk) asg_tog <= asg_tog ^ asg_trig;

red_pitaya_asg i_asg (
  .dac_a_o       (asg_out[0]          ),
  .dac_b_o       (asg_out[1]          ),
  .dac_c_o       (asg_out[2]          ),
  .dac_d_o       (asg_out[3]          ),
  .dac_clk_i     (sys_clk             ),
  .dac_rstn_i    (sys_rstn            ),
  .trig_a_i      (exp_p_s[0]          ),
  .trig_b_i      (exp_p_s[0]          ),
  .trig_c_i      (exp_p_s[0]          ),
  .trig_d_i      (exp_p_s[0]          ),
  .trig_enc_i    (1'b0                ),
  .trig_out_o    (asg_trig            ),
  .play_active_o (                    ),
  .trig_scope_i  (sc_trig_p           ),
  .scope_trig_i  (sc_trig_p           ),
  .sync_rst_o    (                    ),
  .asg1phase_o   (                    ),
  .step_a_o      (                    ),
  .step_b_o      (                    ),
  .step_c_o      (                    ),
  .step_d_o      (                    ),
  .sys_addr      (bk_addr             ),
  .sys_wdata     (bk_wdata            ),
  .sys_sel       (bk_sel              ),
  .sys_wen       (sys_wen[4]          ),
  .sys_ren       (sys_ren[4]          ),
  .sys_rdata     (sys_rdata[4*32+:32] ),
  .sys_err       (sys_err[4]          ),
  .sys_ack       (sys_ack[4]          )
);

////////////////////////////////////////////////////////////////////////////////
// output routing (slot 3) and the FMC DAC card
////////////////////////////////////////////////////////////////////////////////

// the signal hub: the Red Pitaya DSP's addresses; each ASG channel adds into
// any of out1-4 (saturated, all off after reset), and the scope's and the
// FFT's inputs take any signal
logic [13:0] dac_out [4];
logic [3:0]  sc_sel [2], fft_sel [2];

zcu216_dsp i_dsp (
  .clk       (sys_clk            ),
  .rstn      (sys_rstn           ),
  .asg       (asg_out            ),
  .out       (dac_out            ),
  .sc_sel,
  .fft_sel,
  .sys_addr  (bk_addr            ),
  .sys_wdata (bk_wdata           ),
  .sys_wen   (sys_wen[3]         ),
  .sys_ren   (sys_ren[3]         ),
  .sys_rdata (sys_rdata[3*32+:32]),
  .sys_err   (sys_err[3]         ),
  .sys_ack   (sys_ack[3]         )
);

`ifdef ZCU216_FMC_DAC
// the FMC DAC card: out1-4 -> DAC channels 0-3 (SMB J2-J5), one sample per
// sys_clk (125 MS/s, the AD9767's maximum)
zcu216_fmc_dac i_fmc_dac (.clk (sys_clk), .din (dac_out), .fmc_dac_db, .fmc_dac_clk,
  .fmc_dac_wrt);
`endif

////////////////////////////////////////////////////////////////////////////////
// RF Data Converter (slot 2) and FFT / RF bring-up (slot 1)
////////////////////////////////////////////////////////////////////////////////

`ifdef ZCU216_RF
// the sys_clk signals into fclk for their sinks: asg0-3, out1-4
logic [13:0] sig_s [8], sig_f [8];
always_comb
  for (int i = 0; i < 4; i++) begin
    sig_s[i]     = asg_out[i];
    sig_s[4 + i] = dac_out[i];
  end
zcu216_sig_cdc #(.N (8)) i_sig_cdc (.sys_clk (sys_clk), .din (sig_s), .fclk (fclk), .dout (sig_f));

// the scope's trigger into sys_clk, for the ASG
logic sc_trig_tog;
(* ASYNC_REG = "TRUE" *) logic [2:0] sc_trig_s;
always_ff @(posedge sys_clk) begin
  sc_trig_s <= {sc_trig_s[1:0], sc_trig_tog};
  sc_trig_p <= sc_trig_s[2] != sc_trig_s[1];
end

// RF Data Converter: AXI4-Lite from slot 2, streams on fclk
logic [17:0]  rf_awaddr, rf_araddr;
logic [31:0]  rf_wdata, rf_rdata;
logic [3:0]   rf_wstrb;
logic [1:0]   rf_bresp, rf_rresp;
logic         rf_awvalid, rf_awready, rf_wvalid, rf_wready, rf_bvalid, rf_bready;
logic         rf_arvalid, rf_arready, rf_rvalid, rf_rready;

zcu216_axil_bridge #(.AW (18)) i_rf_bridge (
  .clk       (clk                     ),   // the front: works while sys_clk is down
  .rstn      (a_rstn                  ),
  .sys_addr  (ps_sys_addr             ),
  .sys_wdata (ps_sys_wdata            ),
  .sys_wen   (ps_sys_wen && f_rf      ),
  .sys_ren   (ps_sys_ren && f_rf      ),
  .sys_rdata (rf_sys_rdata            ),
  .sys_err   (rf_sys_err              ),
  .sys_ack   (rf_sys_ack              ),
  .m_awaddr  (rf_awaddr ), .m_awvalid (rf_awvalid), .m_awready (rf_awready),
  .m_wdata   (rf_wdata  ), .m_wstrb   (rf_wstrb  ), .m_wvalid  (rf_wvalid ),
  .m_wready  (rf_wready ), .m_bresp   (rf_bresp  ), .m_bvalid  (rf_bvalid ),
  .m_bready  (rf_bready ), .m_araddr  (rf_araddr ), .m_arvalid (rf_arvalid),
  .m_arready (rf_arready), .m_rdata   (rf_rdata  ), .m_rresp   (rf_rresp  ),
  .m_rvalid  (rf_rvalid ), .m_rready  (rf_rready )
);

logic [127:0] adc_data [4];
logic [3:0]   adc_valid;
// the converters' output clocks: clk_adc0 makes fclk; all are measured (debug)
logic         clk_adc2, clk_dac0, clk_dac2;
logic [3:0]   meas_clk;
BUFG i_adc_clk_bufg (.I (clk_adc0), .O (adc_clk_bufg));
BUFG i_meas2 (.I (clk_dac0), .O (meas_clk[2]));
BUFG i_meas3 (.I (clk_dac2), .O (meas_clk[3]));
assign meas_clk[0] = adc_clk_bufg;
assign meas_clk[1] = pl_clk_bufg;
logic [127:0] dac_data;
logic         dac_valid;

rfdc i_rfdc (
  .adc2_clk_p (adc2_clk_p), .adc2_clk_n (adc2_clk_n),
  .dac2_clk_p (dac2_clk_p), .dac2_clk_n (dac2_clk_n),
  .clk_adc0 (clk_adc0), .clk_adc1 (), .clk_adc2 (clk_adc2),
  .clk_dac0 (clk_dac0), .clk_dac1 (), .clk_dac2 (clk_dac2),
  .s_axi_aclk    (clk       ), .s_axi_aresetn (a_rstn    ),
  .s_axi_awaddr  (rf_awaddr ), .s_axi_awvalid (rf_awvalid), .s_axi_awready (rf_awready),
  .s_axi_wdata   (rf_wdata  ), .s_axi_wstrb   (rf_wstrb  ), .s_axi_wvalid  (rf_wvalid ),
  .s_axi_wready  (rf_wready ), .s_axi_bresp   (rf_bresp  ), .s_axi_bvalid  (rf_bvalid ),
  .s_axi_bready  (rf_bready ), .s_axi_araddr  (rf_araddr ), .s_axi_arvalid (rf_arvalid),
  .s_axi_arready (rf_arready), .s_axi_rdata   (rf_rdata  ), .s_axi_rresp   (rf_rresp  ),
  .s_axi_rvalid  (rf_rvalid ), .s_axi_rready  (rf_rready ),
  .irq (),
  .sysref_in_p (sysref_in_p), .sysref_in_n (sysref_in_n),
  .vin00_p (vin00_p), .vin00_n (vin00_n), .vin01_p (vin01_p), .vin01_n (vin01_n),
  .vin02_p (vin02_p), .vin02_n (vin02_n), .vin03_p (vin03_p), .vin03_n (vin03_n),
  .vin10_p (vin10_p), .vin10_n (vin10_n), .vin20_p (vin20_p), .vin20_n (vin20_n),
  .vout00_p (vout00_p), .vout00_n (vout00_n), .vout01_p (vout01_p), .vout01_n (vout01_n),
  .vout02_p (vout02_p), .vout02_n (vout02_n), .vout03_p (vout03_p), .vout03_n (vout03_n),
  .vout10_p (vout10_p), .vout10_n (vout10_n), .vout20_p (vout20_p), .vout20_n (vout20_n),
  .m0_axis_aclk (fclk), .m0_axis_aresetn (frstn),
  .m00_axis_tdata (adc_data[0]), .m00_axis_tvalid (adc_valid[0]), .m00_axis_tready (1'b1),
  .m01_axis_tdata (adc_data[1]), .m01_axis_tvalid (adc_valid[1]), .m01_axis_tready (1'b1),
  .m02_axis_tdata (adc_data[2]), .m02_axis_tvalid (adc_valid[2]), .m02_axis_tready (1'b1),
  .m03_axis_tdata (adc_data[3]), .m03_axis_tvalid (adc_valid[3]), .m03_axis_tready (1'b1),
  .m1_axis_aclk (fclk), .m1_axis_aresetn (frstn),
  .m10_axis_tdata (), .m10_axis_tvalid (), .m10_axis_tready (1'b1),
  .m2_axis_aclk (fclk), .m2_axis_aresetn (frstn),
  .m20_axis_tdata (), .m20_axis_tvalid (), .m20_axis_tready (1'b1),
  .s0_axis_aclk (fclk), .s0_axis_aresetn (frstn),
  .s00_axis_tdata (dac_data), .s00_axis_tvalid (dac_valid), .s00_axis_tready (),
  .s01_axis_tdata (dac_data), .s01_axis_tvalid (dac_valid), .s01_axis_tready (),
  .s02_axis_tdata (dac_data), .s02_axis_tvalid (dac_valid), .s02_axis_tready (),
  .s03_axis_tdata (dac_data), .s03_axis_tvalid (dac_valid), .s03_axis_tready (),
  .s1_axis_aclk (fclk), .s1_axis_aresetn (frstn),
  .s10_axis_tdata ('0), .s10_axis_tvalid (1'b1), .s10_axis_tready (),
  .s2_axis_aclk (fclk), .s2_axis_aresetn (frstn),
  .s20_axis_tdata ('0), .s20_axis_tvalid (1'b1), .s20_axis_tready ()
);

zcu216_fft_test #(
  .NCH      (FFT_NCH       ),
  .SUB      (FFT_NFFT - 3  ),
  .WIN      (FFT_WIN       ),
  .TWC_FILE (TWC_FILE      ),
  .TWF_FILE (TWF_FILE      ),
  .SIN_FILE (SIN_FILE      ),
  .SYS_HZ   (SYS_HZ        )
) i_fft (
  .sys_clk     (sys_clk            ),
  .sys_rstn    (sys_rstn           ),
  .sys_addr    (bk_addr            ),
  .sys_wdata   (bk_wdata           ),
  .sys_wen     (sys_wen[1]         ),
  .sys_ren     (sys_ren[1]         ),
  .sys_rdata   (sys_rdata[1*32+:32]),
  .sys_err     (sys_err[1]         ),
  .sys_ack     (sys_ack[1]         ),
  .fclk        (fclk               ),
  .frstn       (frstn              ),
  .flocked     (locked             ),
  .adc_data    (adc_data           ),
  .adc_valid   (adc_valid          ),
  .dac_data    (dac_data           ),
  .dac_valid   (dac_valid          ),
  .spi_sdo_sel (spi_sdo_sel        ),
  .meas_clk    (meas_clk           ),
  .ext_trig    (exp_p_in[0]        ),  // PMOD0_0 (J46 pin 1), the scope trigger 6/7
  .sig_data    (sig_f              ),  // asg0-3, out1-4
  .asg_tog,
  .sc_trig_tog,
  .sc_sel,
  .fft_sel,
  .hp_awaddr, .hp_awlen, .hp_awsize, .hp_awburst, .hp_awcache, .hp_awprot,
  .hp_awvalid, .hp_awready, .hp_wdata, .hp_wstrb, .hp_wlast, .hp_wvalid,
  .hp_wready, .hp_bresp, .hp_bvalid, .hp_bready,
  .pt_awaddr, .pt_awlen, .pt_awsize, .pt_awburst, .pt_awcache, .pt_awprot,
  .pt_awvalid, .pt_awready, .pt_wdata, .pt_wstrb, .pt_wlast, .pt_wvalid,
  .pt_wready, .pt_bresp, .pt_bvalid, .pt_bready,
  .sc_awaddr, .sc_awlen, .sc_awsize, .sc_awburst, .sc_awcache, .sc_awprot,
  .sc_awvalid, .sc_awready, .sc_wdata, .sc_wstrb, .sc_wlast, .sc_wvalid,
  .sc_wready, .sc_bresp, .sc_bvalid, .sc_bready
);
`else
assign sc_trig_p = 1'b0;
assign sys_rdata[1*32+:32] = 32'h0;
assign sys_err  [1       ] = 1'b0;
assign sys_ack  [1       ] = 1'b1;
assign rf_sys_rdata = 32'h0;
assign rf_sys_err   = 1'b0;
assign rf_sys_ack   = 1'b1;
`endif

////////////////////////////////////////////////////////////////////////////////
// LEDs
////////////////////////////////////////////////////////////////////////////////

logic [26:0] beat;

always_ff @(posedge clk)
  if (!a_rstn) beat <= '0;
  else           beat <= beat + 1'b1;

assign led_g = {beat[26], hk_led[6:0]};

endmodule
