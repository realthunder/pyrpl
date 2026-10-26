################################################################################
# ZCU216 RF build (ZCU216_RF): CLK104 pins on the PL side. The RF Data Converter
# pins (ADC/DAC clocks, SYSREF, vin/vout) are dedicated and need no placement.
# Pins and standards as in AMD's RF DC evaluation design for this board.
################################################################################

# PL_CLK from the CLK104 LMK04828: 125 MHz (config 250M_PL_125M_SYSREF_10M).
# Only measured now (fclk comes from the RF-ADC tile clock).
set_property PACKAGE_PIN E10      [get_ports pl_clk_p]
set_property IOSTANDARD  LVDS_25  [get_ports {pl_clk_p pl_clk_n}]
create_clock -name pl_clk -period 8.000 [get_ports pl_clk_p]
# PL_CLK goes through a BUFG to its clock meter, which need not sit next to the pin
set_property CLOCK_DEDICATED_ROUTE ANY_CMT_COLUMN [get_nets pl_clk_bufg]

# CLK104 SPI readback mux: which chip's SDO reaches the I2C-SPI bridge
set_property PACKAGE_PIN G10      [get_ports {spi_sdo_sel[0]}]
set_property PACKAGE_PIN H11      [get_ports {spi_sdo_sel[1]}]
set_property IOSTANDARD  LVCMOS18 [get_ports {spi_sdo_sel[*]}]
set_false_path -to [get_ports {spi_sdo_sel[*]}]

# clk_adc0 (RF-ADC tile 224 output clock) -> BUFG -> fclk MMCM
set_property CLOCK_DEDICATED_ROUTE ANY_CMT_COLUMN [get_nets adc_clk_bufg]
