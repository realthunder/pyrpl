################################################################################
# The Puzhi PZ-FL9767D FMC DAC card on the FMC+ connector J28 (rtl/zcu216_fmc_dac.sv)
#
# Net -> LA pin from the card's schematic (3312272550PZ-FL9767D-SCH, sheet 1),
# LA pin -> package pin from AMD's board XDC (rdf0546, zcu216_master.xdc). LA00-16
# are bank 67, LA17-33 bank 66, both on VADJ_FMC: 1.8 V, which the system
# controller only applies once set by hand (the card has no IPMI EEPROM).
################################################################################

# ch0: DAC1 port 1 (SMB J2)
set_property PACKAGE_PIN C31   [get_ports {fmc_dac_db[0]}] ;# DB0  LA07_N bank 67
set_property PACKAGE_PIN C30   [get_ports {fmc_dac_db[1]}] ;# DB1  LA07_P bank 67
set_property PACKAGE_PIN C29   [get_ports {fmc_dac_db[2]}] ;# DB2  LA08_N bank 67
set_property PACKAGE_PIN D29   [get_ports {fmc_dac_db[3]}] ;# DB3  LA08_P bank 67
set_property PACKAGE_PIN E29   [get_ports {fmc_dac_db[4]}] ;# DB4  LA05_N bank 67
set_property PACKAGE_PIN D27   [get_ports {fmc_dac_db[5]}] ;# DB5  LA06_N bank 67
set_property PACKAGE_PIN A32   [get_ports {fmc_dac_db[6]}] ;# DB6  LA04_N bank 67
set_property PACKAGE_PIN B32   [get_ports {fmc_dac_db[7]}] ;# DB7  LA04_P bank 67
set_property PACKAGE_PIN B31   [get_ports {fmc_dac_db[8]}] ;# DB8  LA03_N bank 67
set_property PACKAGE_PIN B30   [get_ports {fmc_dac_db[9]}] ;# DB9  LA03_P bank 67
set_property PACKAGE_PIN F29   [get_ports {fmc_dac_db[10]}] ;# DB10 LA05_P bank 67
set_property PACKAGE_PIN E27   [get_ports {fmc_dac_db[11]}] ;# DB11 LA06_P bank 67
set_property PACKAGE_PIN A29   [get_ports {fmc_dac_db[12]}] ;# DB12 LA02_P bank 67
set_property PACKAGE_PIN A30   [get_ports {fmc_dac_db[13]}] ;# DB13 LA02_N bank 67
set_property PACKAGE_PIN F30   [get_ports {fmc_dac_clk[0]}] ;# CLK LA00_P bank 67
set_property PACKAGE_PIN E30   [get_ports {fmc_dac_wrt[0]}] ;# WRT LA00_N bank 67
# ch1: DAC1 port 2 (SMB J3)
set_property PACKAGE_PIN A27   [get_ports {fmc_dac_db[14]}] ;# DB0  LA14_N bank 67
set_property PACKAGE_PIN B27   [get_ports {fmc_dac_db[15]}] ;# DB1  LA14_P bank 67
set_property PACKAGE_PIN J28   [get_ports {fmc_dac_db[16]}] ;# DB2  LA13_N bank 67
set_property PACKAGE_PIN J27   [get_ports {fmc_dac_db[17]}] ;# DB3  LA13_P bank 67
set_property PACKAGE_PIN J26   [get_ports {fmc_dac_db[18]}] ;# DB4  LA16_N bank 67
set_property PACKAGE_PIN K26   [get_ports {fmc_dac_db[19]}] ;# DB5  LA16_P bank 67
set_property PACKAGE_PIN L29   [get_ports {fmc_dac_db[20]}] ;# DB6  LA10_N bank 67
set_property PACKAGE_PIN F28   [get_ports {fmc_dac_db[21]}] ;# DB7  LA09_N bank 67
set_property PACKAGE_PIN L28   [get_ports {fmc_dac_db[22]}] ;# DB8  LA10_P bank 67
set_property PACKAGE_PIN L25   [get_ports {fmc_dac_db[23]}] ;# DB9  LA11_P bank 67
set_property PACKAGE_PIN K25   [get_ports {fmc_dac_db[24]}] ;# DB10 LA11_N bank 67
set_property PACKAGE_PIN F27   [get_ports {fmc_dac_db[25]}] ;# DB11 LA09_P bank 67
set_property PACKAGE_PIN K29   [get_ports {fmc_dac_db[26]}] ;# DB12 LA12_P bank 67
set_property PACKAGE_PIN J29   [get_ports {fmc_dac_db[27]}] ;# DB13 LA12_N bank 67
set_property PACKAGE_PIN G27   [get_ports {fmc_dac_clk[1]}] ;# CLK LA01_P bank 67
set_property PACKAGE_PIN G28   [get_ports {fmc_dac_wrt[1]}] ;# WRT LA01_N bank 67
# ch2: DAC2 port 1 (SMB J4)
set_property PACKAGE_PIN AN24  [get_ports {fmc_dac_db[28]}] ;# DB0  LA25_P bank 66
set_property PACKAGE_PIN AN25  [get_ports {fmc_dac_db[29]}] ;# DB1  LA25_N bank 66
set_property PACKAGE_PIN AL23  [get_ports {fmc_dac_db[30]}] ;# DB2  LA21_P bank 66
set_property PACKAGE_PIN AM23  [get_ports {fmc_dac_db[31]}] ;# DB3  LA21_N bank 66
set_property PACKAGE_PIN AT22  [get_ports {fmc_dac_db[32]}] ;# DB4  LA22_P bank 66
set_property PACKAGE_PIN AU22  [get_ports {fmc_dac_db[33]}] ;# DB5  LA22_N bank 66
set_property PACKAGE_PIN AP23  [get_ports {fmc_dac_db[34]}] ;# DB6  LA23_N bank 66
set_property PACKAGE_PIN AN23  [get_ports {fmc_dac_db[35]}] ;# DB7  LA23_P bank 66
set_property PACKAGE_PIN AU23  [get_ports {fmc_dac_db[36]}] ;# DB8  LA20_P bank 66
set_property PACKAGE_PIN AV24  [get_ports {fmc_dac_db[37]}] ;# DB9  LA19_P bank 66
set_property PACKAGE_PIN AW24  [get_ports {fmc_dac_db[38]}] ;# DB10 LA19_N bank 66
set_property PACKAGE_PIN AV23  [get_ports {fmc_dac_db[39]}] ;# DB11 LA20_N bank 66
set_property PACKAGE_PIN B28   [get_ports {fmc_dac_db[40]}] ;# DB12 LA15_P bank 67
set_property PACKAGE_PIN A28   [get_ports {fmc_dac_db[41]}] ;# DB13 LA15_N bank 67
set_property PACKAGE_PIN AP24  [get_ports {fmc_dac_clk[2]}] ;# CLK LA18_P bank 66
set_property PACKAGE_PIN AR24  [get_ports {fmc_dac_wrt[2]}] ;# WRT LA18_N bank 66
# ch3: DAC2 port 2 (SMB J5)
set_property PACKAGE_PIN AH24  [get_ports {fmc_dac_db[42]}] ;# DB0  LA32_P bank 66
set_property PACKAGE_PIN AJ24  [get_ports {fmc_dac_db[43]}] ;# DB1  LA32_N bank 66
set_property PACKAGE_PIN AJ23  [get_ports {fmc_dac_db[44]}] ;# DB2  LA33_P bank 66
set_property PACKAGE_PIN AK24  [get_ports {fmc_dac_db[45]}] ;# DB3  LA33_N bank 66
set_property PACKAGE_PIN AJ26  [get_ports {fmc_dac_db[46]}] ;# DB4  LA30_P bank 66
set_property PACKAGE_PIN AK26  [get_ports {fmc_dac_db[47]}] ;# DB5  LA30_N bank 66
set_property PACKAGE_PIN AL25  [get_ports {fmc_dac_db[48]}] ;# DB6  LA31_N bank 66
set_property PACKAGE_PIN AK25  [get_ports {fmc_dac_db[49]}] ;# DB7  LA31_P bank 66
set_property PACKAGE_PIN AM25  [get_ports {fmc_dac_db[50]}] ;# DB8  LA29_N bank 66
set_property PACKAGE_PIN AJ22  [get_ports {fmc_dac_db[51]}] ;# DB9  LA28_P bank 66
set_property PACKAGE_PIN AK22  [get_ports {fmc_dac_db[52]}] ;# DB10 LA28_N bank 66
set_property PACKAGE_PIN AL24  [get_ports {fmc_dac_db[53]}] ;# DB11 LA29_P bank 66
set_property PACKAGE_PIN AM26  [get_ports {fmc_dac_db[54]}] ;# DB12 LA24_P bank 66
set_property PACKAGE_PIN AN26  [get_ports {fmc_dac_db[55]}] ;# DB13 LA24_N bank 66
set_property PACKAGE_PIN AP22  [get_ports {fmc_dac_clk[3]}] ;# CLK LA17_P bank 66
set_property PACKAGE_PIN AR22  [get_ports {fmc_dac_wrt[3]}] ;# WRT LA17_N bank 66

set_property IOSTANDARD LVCMOS18 [get_ports {fmc_dac_db[*] fmc_dac_clk[*] fmc_dac_wrt[*]}]
set_property DRIVE      8        [get_ports {fmc_dac_db[*] fmc_dac_clk[*] fmc_dac_wrt[*]}]
set_property SLEW       MEDIUM   [get_ports {fmc_dac_db[*] fmc_dac_clk[*] fmc_dac_wrt[*]}]

# Source-synchronous outputs: each port's WRT (the bus clock inverted by its
# ODDRE1) latches its 14 data bits. AD9767 setup 2.0 ns / hold 1.5 ns at WRT,
# plus 1 ns for the shifters' channel-to-channel skew. The data leave from IOB
# flops half a period before the WRT edge, so either side has ~2 ns to spare.
for {set c 0} {$c < 4} {incr c} {
    create_generated_clock -name fmc_dac_wrt$c -divide_by 1 -invert \
        -source [get_pins "i_fmc_dac/g_ch\[$c\].i_wrt/C"] [get_ports "fmc_dac_wrt\[$c\]"]
    set db [get_ports [lmap b {0 1 2 3 4 5 6 7 8 9 10 11 12 13} {format "fmc_dac_db\[%d\]" [expr {$c * 14 + $b}]}]]
    set_output_delay -clock fmc_dac_wrt$c -max  3.0 $db
    set_output_delay -clock fmc_dac_wrt$c -min -2.5 $db
}
# CLK (the DAC latch) runs on the same edge as WRT; nothing else is timed to it.
set_false_path -to [get_ports {fmc_dac_clk[*]}]
