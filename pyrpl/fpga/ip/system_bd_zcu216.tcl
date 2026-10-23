################################################################################
# ZCU216 processing-system block design (system_zcu216).
#
# The Zynq UltraScale+ PS with the ZCU216 board preset (DDR4, MIO, UART, GEM,
# SD, USB as AMD ships them), plus:
#   - pl_clk0 (100 MHz) and pl_resetn0 to the top level;
#   - M_AXI_HPM0_LPD, 32 bits wide, brought out as the AXI4 port M_AXI_GP0 and
#     mapped at 0x8000_0000 (16 MB). zcu216_top.sv puts the Red Pitaya register
#     bus behind it (axi_slave.v), so the Red Pitaya offsets are unchanged and
#     only the base moves from 0x4000_0000.
#
# Needs board_part xilinx.com:zcu216:part0:2.0 on the project (zcu216_vivado.tcl
# sets it), because the preset comes from the board files.
################################################################################

set design_name system_zcu216
create_bd_design $design_name

set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e ps]
apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e \
    -config {apply_board_preset "1"} $ps

# Only the LPD master port; the preset's two FPD ports stay off.
set_property -dict [list \
    CONFIG.PSU__USE__M_AXI_GP0 {0} \
    CONFIG.PSU__USE__M_AXI_GP1 {0} \
    CONFIG.PSU__USE__M_AXI_GP2 {1} \
    CONFIG.PSU__MAXIGP2__DATA_WIDTH {32} \
    CONFIG.PSU__FPGA_PL0_ENABLE {1} \
    CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {100} \
] $ps

create_bd_port -dir O -type clk pl_clk0
create_bd_port -dir O -type rst pl_resetn0
connect_bd_net [get_bd_pins ps/pl_clk0]    [get_bd_ports pl_clk0]
connect_bd_net [get_bd_pins ps/pl_resetn0] [get_bd_ports pl_resetn0]

# The register port runs on pl_clk0.
set gp0 [create_bd_intf_port -mode Master -vlnv xilinx.com:interface:aximm_rtl:1.0 M_AXI_GP0]
set_property -dict [list \
    CONFIG.PROTOCOL {AXI4} \
    CONFIG.DATA_WIDTH {32} \
    CONFIG.ADDR_WIDTH {40} \
    CONFIG.FREQ_HZ [get_property CONFIG.FREQ_HZ [get_bd_pins ps/pl_clk0]] \
] $gp0
connect_bd_intf_net [get_bd_intf_pins ps/M_AXI_HPM0_LPD] $gp0
connect_bd_net [get_bd_pins ps/pl_clk0] [get_bd_pins ps/maxihpm0_lpd_aclk]
set_property CONFIG.ASSOCIATED_BUSIF {M_AXI_GP0} [get_bd_ports pl_clk0]

assign_bd_address -offset 0x80000000 -range 16M [get_bd_addr_segs M_AXI_GP0/Reg]

validate_bd_design
save_bd_design
