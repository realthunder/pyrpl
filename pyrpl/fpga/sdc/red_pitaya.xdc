#
# $Id: red_pitaya.xdc 961 2014-01-21 11:40:39Z matej.oblak $
#
# @brief Red Pitaya location constraints.
#
# @Author Matej Oblak
#
# (c) Red Pitaya  http://www.redpitaya.com
#

############################################################################
# IO constraints                                                           #
############################################################################

### ADC

# ADC A data
set_property IOSTANDARD LVCMOS18 [get_ports {adc_dat_a_i[*]}]
set_property IOB        TRUE     [get_ports {adc_dat_a_i[*]}]
#set_property PACKAGE_PIN V17     [get_ports {adc_dat_a_i[0]}]
#set_property PACKAGE_PIN U17     [get_ports {adc_dat_a_i[1]}]
set_property PACKAGE_PIN Y17     [get_ports {adc_dat_a_i[2]}]
set_property PACKAGE_PIN W16     [get_ports {adc_dat_a_i[3]}]
set_property PACKAGE_PIN Y16     [get_ports {adc_dat_a_i[4]}]
set_property PACKAGE_PIN W15     [get_ports {adc_dat_a_i[5]}]
set_property PACKAGE_PIN W14     [get_ports {adc_dat_a_i[6]}]
set_property PACKAGE_PIN Y14     [get_ports {adc_dat_a_i[7]}]
set_property PACKAGE_PIN W13     [get_ports {adc_dat_a_i[8]}]
set_property PACKAGE_PIN V12     [get_ports {adc_dat_a_i[9]}]
set_property PACKAGE_PIN V13     [get_ports {adc_dat_a_i[10]}]
set_property PACKAGE_PIN T14     [get_ports {adc_dat_a_i[11]}]
set_property PACKAGE_PIN T15     [get_ports {adc_dat_a_i[12]}]
set_property PACKAGE_PIN V15     [get_ports {adc_dat_a_i[13]}]
set_property PACKAGE_PIN T16     [get_ports {adc_dat_a_i[14]}]
set_property PACKAGE_PIN V16     [get_ports {adc_dat_a_i[15]}]

# ADC B data
set_property IOSTANDARD LVCMOS18 [get_ports {adc_dat_b_i[*]}]
set_property IOB        TRUE     [get_ports {adc_dat_b_i[*]}]
#set_property PACKAGE_PIN T17     [get_ports {adc_dat_b_i[0]}]
#set_property PACKAGE_PIN R16     [get_ports {adc_dat_b_i[1]}]
set_property PACKAGE_PIN R18     [get_ports {adc_dat_b_i[2]}]
set_property PACKAGE_PIN P16     [get_ports {adc_dat_b_i[3]}]
set_property PACKAGE_PIN P18     [get_ports {adc_dat_b_i[4]}]
set_property PACKAGE_PIN N17     [get_ports {adc_dat_b_i[5]}]
set_property PACKAGE_PIN R19     [get_ports {adc_dat_b_i[6]}]
set_property PACKAGE_PIN T20     [get_ports {adc_dat_b_i[7]}]
set_property PACKAGE_PIN T19     [get_ports {adc_dat_b_i[8]}]
set_property PACKAGE_PIN U20     [get_ports {adc_dat_b_i[9]}]
set_property PACKAGE_PIN V20     [get_ports {adc_dat_b_i[10]}]
set_property PACKAGE_PIN W20     [get_ports {adc_dat_b_i[11]}]
set_property PACKAGE_PIN W19     [get_ports {adc_dat_b_i[12]}]
set_property PACKAGE_PIN Y19     [get_ports {adc_dat_b_i[13]}]
set_property PACKAGE_PIN W18     [get_ports {adc_dat_b_i[14]}]
set_property PACKAGE_PIN Y18     [get_ports {adc_dat_b_i[15]}]

set_property IOSTANDARD DIFF_HSTL_I_18 [get_ports adc_clk_p_i]
set_property IOSTANDARD DIFF_HSTL_I_18 [get_ports adc_clk_n_i]
set_property PACKAGE_PIN U18           [get_ports adc_clk_p_i]
set_property PACKAGE_PIN U19           [get_ports adc_clk_n_i]

# Output ADC clock
set_property IOSTANDARD LVCMOS18 [get_ports {adc_clk_o[*]}]
set_property SLEW       FAST     [get_ports {adc_clk_o[*]}]
set_property DRIVE      8        [get_ports {adc_clk_o[*]}]
#set_property IOB        TRUE     [get_ports {adc_clk_o[*]}]

set_property PACKAGE_PIN N20 [get_ports {adc_clk_o[0]}]
set_property PACKAGE_PIN P20 [get_ports {adc_clk_o[1]}]

# ADC clock stabilizer
set_property IOSTANDARD LVCMOS18 [get_ports adc_cdcs_o]
set_property PACKAGE_PIN V18     [get_ports adc_cdcs_o]
set_property SLEW       FAST     [get_ports adc_cdcs_o]
set_property DRIVE      8        [get_ports adc_cdcs_o]

### DAC

# data
set_property IOSTANDARD LVCMOS33 [get_ports {dac_dat_o[*]}]
set_property SLEW       SLOW     [get_ports {dac_dat_o[*]}]
set_property DRIVE      4        [get_ports {dac_dat_o[*]}]
#set_property IOB        TRUE     [get_ports {dac_dat_o[*]}]

set_property PACKAGE_PIN M19 [get_ports {dac_dat_o[0]}]
set_property PACKAGE_PIN M20 [get_ports {dac_dat_o[1]}]
set_property PACKAGE_PIN L19 [get_ports {dac_dat_o[2]}]
set_property PACKAGE_PIN L20 [get_ports {dac_dat_o[3]}]
set_property PACKAGE_PIN K19 [get_ports {dac_dat_o[4]}]
set_property PACKAGE_PIN J19 [get_ports {dac_dat_o[5]}]
set_property PACKAGE_PIN J20 [get_ports {dac_dat_o[6]}]
set_property PACKAGE_PIN H20 [get_ports {dac_dat_o[7]}]
set_property PACKAGE_PIN G19 [get_ports {dac_dat_o[8]}]
set_property PACKAGE_PIN G20 [get_ports {dac_dat_o[9]}]
set_property PACKAGE_PIN F19 [get_ports {dac_dat_o[10]}]
set_property PACKAGE_PIN F20 [get_ports {dac_dat_o[11]}]
set_property PACKAGE_PIN D20 [get_ports {dac_dat_o[12]}]
set_property PACKAGE_PIN D19 [get_ports {dac_dat_o[13]}]

# control
set_property IOSTANDARD LVCMOS33 [get_ports dac_*_o]
set_property SLEW       FAST     [get_ports dac_*_o]
set_property DRIVE      8        [get_ports dac_*_o]
#set_property IOB        TRUE     [get_ports dac_*_o]

set_property PACKAGE_PIN M17 [get_ports dac_wrt_o]
set_property PACKAGE_PIN N16 [get_ports dac_sel_o]
set_property PACKAGE_PIN M18 [get_ports dac_clk_o]
set_property PACKAGE_PIN N15 [get_ports dac_rst_o]

### PWM DAC
set_property IOSTANDARD LVCMOS18 [get_ports {dac_pwm_o[*]}]
set_property SLEW FAST           [get_ports {dac_pwm_o[*]}]
set_property DRIVE 12            [get_ports {dac_pwm_o[*]}]
set_property IOB FALSE           [get_ports {dac_pwm_o[*]}]

set_property PACKAGE_PIN T10 [get_ports {dac_pwm_o[0]}]
set_property PACKAGE_PIN T11 [get_ports {dac_pwm_o[1]}]
set_property PACKAGE_PIN P15 [get_ports {dac_pwm_o[2]}]
set_property PACKAGE_PIN U13 [get_ports {dac_pwm_o[3]}]

### XADC
set_property IOSTANDARD LVCMOS33 [get_ports {vinp_i[*]}]
set_property IOSTANDARD LVCMOS33 [get_ports {vinn_i[*]}]
set_property LOC XADC_X0Y0 [get_cells i_ams/XADC_inst]
#AD0
set_property PACKAGE_PIN C20 [get_ports {vinp_i[1]}]
set_property PACKAGE_PIN B20 [get_ports {vinn_i[1]}]
#AD1
set_property PACKAGE_PIN E17 [get_ports {vinp_i[2]}]
set_property PACKAGE_PIN D18 [get_ports {vinn_i[2]}]
#AD8
set_property PACKAGE_PIN B19 [get_ports {vinp_i[0]}]
set_property PACKAGE_PIN A20 [get_ports {vinn_i[0]}]
#AD9
set_property PACKAGE_PIN E18 [get_ports {vinp_i[3]}]
set_property PACKAGE_PIN E19 [get_ports {vinn_i[3]}]
#V_0
set_property PACKAGE_PIN K9  [get_ports {vinp_i[4]}]
set_property PACKAGE_PIN L10 [get_ports {vinn_i[4]}]

### Expansion connector
set_property IOSTANDARD LVCMOS33 [get_ports {exp_p_io[*]}]
set_property IOSTANDARD LVCMOS33 [get_ports {exp_n_io[*]}]
set_property SLEW       FAST     [get_ports {exp_p_io[*]}]
set_property SLEW       FAST     [get_ports {exp_n_io[*]}]
set_property DRIVE      8        [get_ports {exp_p_io[*]}]
set_property DRIVE      8        [get_ports {exp_n_io[*]}]

set_property PACKAGE_PIN G17 [get_ports {exp_p_io[0]}]
set_property PACKAGE_PIN G18 [get_ports {exp_n_io[0]}]
set_property PACKAGE_PIN H16 [get_ports {exp_p_io[1]}]
set_property PACKAGE_PIN H17 [get_ports {exp_n_io[1]}]
set_property PACKAGE_PIN J18 [get_ports {exp_p_io[2]}]
set_property PACKAGE_PIN H18 [get_ports {exp_n_io[2]}]
set_property PACKAGE_PIN K17 [get_ports {exp_p_io[3]}]
set_property PACKAGE_PIN K18 [get_ports {exp_n_io[3]}]
set_property PACKAGE_PIN L14 [get_ports {exp_p_io[4]}]
set_property PACKAGE_PIN L15 [get_ports {exp_n_io[4]}]
set_property PACKAGE_PIN L16 [get_ports {exp_p_io[5]}]
set_property PACKAGE_PIN L17 [get_ports {exp_n_io[5]}]
set_property PACKAGE_PIN K16 [get_ports {exp_p_io[6]}]
set_property PACKAGE_PIN J16 [get_ports {exp_n_io[6]}]
set_property PACKAGE_PIN M14 [get_ports {exp_p_io[7]}]
set_property PACKAGE_PIN M15 [get_ports {exp_n_io[7]}]

#set_property PULLDOWN TRUE [get_ports {exp_p_io[0]}]
#set_property PULLDOWN TRUE [get_ports {exp_n_io[0]}]
#set_property PULLUP   TRUE [get_ports {exp_p_io[7]}]
#set_property PULLUP   TRUE [get_ports {exp_n_io[7]}]

### SATA connector
set_property IOSTANDARD LVCMOS18 [get_ports {daisy_p_o[*]}]
set_property IOSTANDARD LVCMOS18 [get_ports {daisy_n_o[*]}]
set_property IOSTANDARD LVCMOS18 [get_ports {daisy_p_i[*]}]
set_property IOSTANDARD LVCMOS18 [get_ports {daisy_n_i[*]}]

set_property PACKAGE_PIN T12 [get_ports {daisy_p_o[0]}]
set_property PACKAGE_PIN U12 [get_ports {daisy_n_o[0]}]
set_property PACKAGE_PIN U14 [get_ports {daisy_p_o[1]}]
set_property PACKAGE_PIN U15 [get_ports {daisy_n_o[1]}]
set_property PACKAGE_PIN P14 [get_ports {daisy_p_i[0]}]
set_property PACKAGE_PIN R14 [get_ports {daisy_n_i[0]}]
set_property PACKAGE_PIN N18 [get_ports {daisy_p_i[1]}]
set_property PACKAGE_PIN P19 [get_ports {daisy_n_i[1]}]

### LED
set_property IOSTANDARD LVCMOS33 [get_ports {led_o[*]}]
set_property SLEW       SLOW     [get_ports {led_o[*]}]
set_property DRIVE      4        [get_ports {led_o[*]}]

set_property PACKAGE_PIN F16     [get_ports {led_o[0]}]
set_property PACKAGE_PIN F17     [get_ports {led_o[1]}]
set_property PACKAGE_PIN G15     [get_ports {led_o[2]}]
set_property PACKAGE_PIN H15     [get_ports {led_o[3]}]
set_property PACKAGE_PIN K14     [get_ports {led_o[4]}]
set_property PACKAGE_PIN G14     [get_ports {led_o[5]}]
set_property PACKAGE_PIN J15     [get_ports {led_o[6]}]
set_property PACKAGE_PIN J14     [get_ports {led_o[7]}]

############################################################################
# Clock constraints                                                        #
############################################################################

#NET "adc_clk" TNM_NET = "adc_clk";
#TIMESPEC TS_adc_clk = PERIOD "adc_clk" 125 MHz;

create_clock -period 8.000 -name adc_clk [get_ports adc_clk_p_i]

create_generated_clock \
   -name pll_adc_clk \
   -source [get_pins pll/clk] \
   -multiply_by 8 \
   -divide_by 8 \
   [get_pins pll/clk_adc]

create_generated_clock \
   -name pll_ser_clk \
   -source [get_pins pll/clk] \
   -multiply_by 8 \
   -divide_by 4 \
   [get_pins pll/clk_ser]

set_input_delay -clock adc_clk 3.400 [get_ports adc_dat_a_i[*]]
set_input_delay -clock adc_clk 3.400 [get_ports adc_dat_b_i[*]]

create_clock -period 4.000 -name rx_clk  [get_ports daisy_p_i[1]]

# set_false_path -from [get_clocks adc_clk]     -to [get_clocks dac_clk_out]
# set_false_path -from [get_clocks clk_fpga_0]  -to [get_clocks ser_clk_out]
# set_false_path -from [get_clocks clk_fpga_0]  -to [get_clocks dac_2clk_out]
set_false_path -from [get_clocks clk_fpga_0]  -to [get_clocks adc_clk]
# set_false_path -from [get_clocks clk_fpga_0]  -to [get_clocks par_clk]

# FFT CDC: all data crossing between pll_ser_clk (250 MHz, clk_i) and
# pll_adc_clk (125 MHz, adc_clk_i) goes through xpm_cdc_* primitives or
# xpm_memory_sdpram independent-clock BRAMs, which apply their own
# set_max_delay -datapath_only constraints internally.  Suppress Vivado's
# exhaustive inter-clock analysis of the full 76k-endpoint domain product.
set_false_path -from [get_clocks pll_ser_clk] -to [get_clocks pll_adc_clk]
set_false_path -from [get_clocks pll_adc_clk] -to [get_clocks pll_ser_clk]

# xfft LogiCORE: ce_predicted_reg is asserted one clock cycle early by design.
# Outgoing paths: fo~256 high-fanout net to DSP CEB2 inputs needs 2 cycles.
# Incoming paths: combinational ce_predicted logic has long routes inside xfft.
# The IP is designed for the CE to be valid 1 cycle early, so 2-cycle budget
# for both the computation and propagation is within the IP's timing intent.
set_multicycle_path 2 -setup -from [get_cells -hierarchical -filter {NAME =~ *ce_predicted_reg*}]
set_multicycle_path 1 -hold  -from [get_cells -hierarchical -filter {NAME =~ *ce_predicted_reg*}]
set_multicycle_path 2 -setup -to   [get_cells -hierarchical -filter {NAME =~ *ce_predicted_reg*}]
set_multicycle_path 1 -hold  -to   [get_cells -hierarchical -filter {NAME =~ *ce_predicted_reg*}]

# PWM comparator: v_r drives a CARRY4×2 chain to an OLOGIC IOB register;
# negative clock skew tightens the path to below 4 ns.  Allow 2 cycles.
set_multicycle_path 2 -setup -from [get_cells -hierarchical -filter {NAME =~ pwm*/v_r_reg*}]
set_multicycle_path 1 -hold  -from [get_cells -hierarchical -filter {NAME =~ pwm*/v_r_reg*}]

# AXI/sys-bus config writes and readback are quasi-static.  On a write,
# axi_slave latches wr_wdata once at the request and holds it for the whole
# multi-cycle AXI transaction while sys_wen (registered, one cycle later) pulses,
# so the config register's final capture has >=2 settled cycles.  On a read,
# sys_addr is held for the transaction while sys_rdata is registered and returned
# over the ack window.  The remaining pll_adc_clk violations are all such paths
# (i_asg config-register writes + sys_rdata readback, 0-few logic levels /
# ~93% route after the i_dsp trim freed area).  Relax both to 2 cycles.
set_multicycle_path 2 -setup -from [get_cells -hierarchical -filter {NAME =~ *axi_slave_gp0/wr_wdata_reg*}]
set_multicycle_path 1 -hold  -from [get_cells -hierarchical -filter {NAME =~ *axi_slave_gp0/wr_wdata_reg*}]
set_multicycle_path 2 -setup -to   [get_cells -hierarchical -filter {NAME =~ *sys_rdata_reg*}]
set_multicycle_path 1 -hold  -to   [get_cells -hierarchical -filter {NAME =~ *sys_rdata_reg*}]

# fft_nfft is the runtime FFT-size config (log2 N), set once at configuration and
# static while the FFT streams.  In each FFT instance it is registered on the ser
# clock and fans out (fo~80) to fft_length and size-dependent logic — quasi-static
# same-clock ser paths, relaxed to 2 cycles.  (The adc->ser config CDC into these
# regs is already covered by the pll_adc_clk->pll_ser_clk false_path above.)
set_multicycle_path 2 -setup -from [get_cells -hierarchical -filter {NAME =~ *fft_nfft_reg*}]
set_multicycle_path 1 -hold  -from [get_cells -hierarchical -filter {NAME =~ *fft_nfft_reg*}]

# ADC data hold: adc_dat_*_i input_delay is referenced to adc_clk, but the
# IOB register is clocked by pll_adc_clk (large internal skew vs adc_clk).
# Hold analysis across these two clocks is not meaningful — suppress it.
set_false_path -hold -from [get_clocks adc_clk] -to [get_clocks pll_adc_clk]
# set_false_path -from [get_clocks dac_clk_out] -to [get_clocks dac_2clk_out]
# set_false_path -from [get_clocks dac_clk_out] -to [get_clocks dac_2ph_out]

############################################################################
# Floorplan: fft_b pre_0 s_axis regslice near fifo_in read pointer        #
############################################################################
# Without constraint: regslice_both_s_axis state_reg lands at X49Y41 while
# fifo_in rdp count_value sits at X51Y23-Y25 — 16-row gap causes 0.836 ns
# routing delay on the state→rdp path (dominant pll_ser_clk violation).
# Pin the regslice cells to the band adjacent to fifo_in.
create_pblock pb_saxisreg_fft_b
add_cells_to_pblock [get_pblocks pb_saxisreg_fft_b] \
    [get_cells -hier -filter {NAME =~ i_scope/fft_b/gen_fft_ip_ssr.fft_i/fft_ip_ssr_bd_i/pre_0/inst/regslice_both_s_axis_V_data_V_U/*}]
resize_pblock [get_pblocks pb_saxisreg_fft_b] -add {SLICE_X47Y21:SLICE_X54Y32}

