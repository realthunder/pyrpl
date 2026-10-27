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

# PMOD headers: hk's expansion GPIOs, exp_p = PMOD0 (J46), exp_n = PMOD1 (J47).
# Bank 88 (1.8 V), level-shifted to 3.3 V by TXS0108E U45/U46. Pins from AMD's
# board XDC (rdf0546, zcu216_master.xdc).
set_property PACKAGE_PIN G15 [get_ports {pmod0[0]}]
set_property PACKAGE_PIN G16 [get_ports {pmod0[1]}]
set_property PACKAGE_PIN H14 [get_ports {pmod0[2]}]
set_property PACKAGE_PIN H15 [get_ports {pmod0[3]}]
set_property PACKAGE_PIN G13 [get_ports {pmod0[4]}]
set_property PACKAGE_PIN H13 [get_ports {pmod0[5]}]
set_property PACKAGE_PIN J13 [get_ports {pmod0[6]}]
set_property PACKAGE_PIN J14 [get_ports {pmod0[7]}]
set_property PACKAGE_PIN L17 [get_ports {pmod1[0]}]
set_property PACKAGE_PIN M17 [get_ports {pmod1[1]}]
set_property PACKAGE_PIN M14 [get_ports {pmod1[2]}]
set_property PACKAGE_PIN N14 [get_ports {pmod1[3]}]
set_property PACKAGE_PIN M15 [get_ports {pmod1[4]}]
set_property PACKAGE_PIN N15 [get_ports {pmod1[5]}]
set_property PACKAGE_PIN M16 [get_ports {pmod1[6]}]
set_property PACKAGE_PIN N16 [get_ports {pmod1[7]}]
set_property IOSTANDARD LVCMOS18 [get_ports {pmod0[*] pmod1[*]}]

# Register-driven GPIOs, inputs synchronised in the fabric: no timing to analyse.
set_false_path -to   [get_ports {pmod0[*] pmod1[*]}]
set_false_path -from [get_ports {pmod0[*] pmod1[*]}]
