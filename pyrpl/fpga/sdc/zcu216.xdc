################################################################################
# ZCU216 pin constraints (pins and standards from the AMD zcu216 2.0 board files)
#
# pl_clk0 comes from the PS block, which constrains it itself.
################################################################################

# Green RGB LEDs. 0-3 sit in a 1.8 V bank, 4-7 in a 1.2 V bank.
set_property PACKAGE_PIN C13  [get_ports {led_g[0]}]
set_property PACKAGE_PIN D14  [get_ports {led_g[1]}]
set_property PACKAGE_PIN D12  [get_ports {led_g[2]}]
set_property PACKAGE_PIN D13  [get_ports {led_g[3]}]
set_property PACKAGE_PIN AW18 [get_ports {led_g[4]}]
set_property PACKAGE_PIN AV18 [get_ports {led_g[5]}]
set_property PACKAGE_PIN BA19 [get_ports {led_g[6]}]
set_property PACKAGE_PIN AP21 [get_ports {led_g[7]}]
set_property IOSTANDARD LVCMOS18 [get_ports {led_g[0] led_g[1] led_g[2] led_g[3]}]
set_property IOSTANDARD LVCMOS12 [get_ports {led_g[4] led_g[5] led_g[6] led_g[7]}]

# The LEDs are slow status outputs; no timing to analyse.
set_false_path -to [get_ports {led_g[*]}]
