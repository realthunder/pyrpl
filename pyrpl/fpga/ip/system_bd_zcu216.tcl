################################################################################
# ZCU216 processing-system block design (system_zcu216).
#
# The Zynq UltraScale+ PS with the ZCU216 board preset (DDR4, MIO, UART, GEM,
# SD, USB as AMD ships them), plus the PL clock and reset brought out to the
# top level. No AXI master ports yet: the register bridge to the PL arrives in
# phase 2 of the port.
#
# Needs board_part xilinx.com:zcu216:part0:2.0 on the project (zcu216_vivado.tcl
# sets it), because the preset comes from the board files.
################################################################################

set design_name system_zcu216
create_bd_design $design_name

set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e ps]
apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e \
    -config {apply_board_preset "1"} $ps

# The preset enables M_AXI_HPM0/1_FPD; with nothing on them their clock pins
# would be left unconnected, so switch them off until phase 2 needs them.
set_property -dict [list \
    CONFIG.PSU__USE__M_AXI_GP0 {0} \
    CONFIG.PSU__USE__M_AXI_GP1 {0} \
    CONFIG.PSU__USE__M_AXI_GP2 {0} \
    CONFIG.PSU__FPGA_PL0_ENABLE {1} \
    CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {100} \
] $ps

create_bd_port -dir O -type clk pl_clk0
create_bd_port -dir O -type rst pl_resetn0
connect_bd_net [get_bd_pins ps/pl_clk0]    [get_bd_ports pl_clk0]
connect_bd_net [get_bd_pins ps/pl_resetn0] [get_bd_ports pl_resetn0]

validate_bd_design
save_bd_design
