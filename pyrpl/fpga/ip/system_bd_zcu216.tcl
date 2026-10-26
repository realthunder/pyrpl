################################################################################
# ZCU216 processing-system block design (system_zcu216).
#
# The Zynq UltraScale+ PS with the ZCU216 board preset (DDR4, MIO, UART, GEM,
# SD, USB as AMD ships them), plus:
#   - pl_clk0 (100 MHz) and pl_resetn0 to the top level;
#   - M_AXI_HPM0_LPD, 32 bits wide, brought out as the AXI4 port M_AXI_GP0 and
#     mapped at 0x8000_0000 (16 MB). zcu216_top.sv puts the Red Pitaya register
#     bus behind it (axi_slave.v), so the Red Pitaya offsets are unchanged and
#     only the base moves from 0x4000_0000;
#   - in RF builds, S_AXI_HP0/HP1 for the snapshot DMA (below).
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

# RF builds ($zcu216_hp, set by zcu216_vivado.tcl): the snapshot DMA's two write
# ports into DDR, S_AXI_HP0_FPD and S_AXI_HP1_FPD (separate DDR controller
# ports), 128 bits, brought out as S_AXI_HP0 / S_AXI_HP1 on hp_clk (fclk,
# 312.5 MHz, from the top level). The PS sees them at their DDR addresses.
if {[info exists zcu216_hp] && $zcu216_hp} {
    set_property -dict [list \
        CONFIG.PSU__USE__S_AXI_GP2 {1} \
        CONFIG.PSU__USE__S_AXI_GP3 {1} \
        CONFIG.PSU__SAXIGP2__DATA_WIDTH {128} \
        CONFIG.PSU__SAXIGP3__DATA_WIDTH {128} \
    ] $ps
    create_bd_port -dir I -type clk -freq_hz 312500000 hp_clk
    connect_bd_net [get_bd_ports hp_clk] \
        [get_bd_pins ps/saxihp0_fpd_aclk] [get_bd_pins ps/saxihp1_fpd_aclk]
    foreach {port pin} {S_AXI_HP0 S_AXI_HP0_FPD S_AXI_HP1 S_AXI_HP1_FPD} {
        make_bd_intf_pins_external [get_bd_intf_pins ps/$pin]
        set_property NAME $port [get_bd_intf_ports ${pin}_0]
        set_property CONFIG.FREQ_HZ 312500000 [get_bd_intf_ports $port]
    }
    set_property CONFIG.ASSOCIATED_BUSIF {S_AXI_HP0:S_AXI_HP1} [get_bd_ports hp_clk]
    assign_bd_address
}

validate_bd_design
save_bd_design
