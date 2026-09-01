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

# FFT CDC: all data crossing between pll_ser_clk (the FFT clock when
# FFT_CLK_SEL=1: 250 MHz default, or 200 MHz with FFT_CLK_200) and
# pll_adc_clk (125 MHz, adc_clk_i) goes through xpm_cdc_* primitives or
# xpm_memory_sdpram / xpm_fifo independent-clock BRAMs, which apply their own
# set_max_delay -datapath_only constraints internally.  Suppress Vivado's
# exhaustive inter-clock analysis of the full 76k-endpoint domain product.
# (Frequency-independent: the crossing is asynchronous either way.)
set_false_path -from [get_clocks pll_ser_clk] -to [get_clocks pll_adc_clk]
set_false_path -from [get_clocks pll_adc_clk] -to [get_clocks pll_ser_clk]

# (xfft ce_predicted multicycle moved to the per-FFT-implementation section below)

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

# input_select is the per-module input-source mux select in red_pitaya_dsp: written
# ONLY by a sys-bus register write (addr 0x00) and otherwise static while the design
# runs.  It is registered on pll_adc_clk and used solely as the *select* of the
# output_signal mux that feeds each module (PID/IIR/IQ/scope/trigger) — never as
# per-cycle data — so the real ADC datapaths (output_signal*_reg -> mux -> module)
# launch from different startpoints and keep their single-cycle requirement.  The
# pll_adc_clk setup paths sourced here (trigger input mux -> lpf -> schmitt) are
# quasi-static; relax to 2 cycles.  A reconfig leaves input_signal momentarily mixed
# for a couple of cycles, which is harmless (the trigger is re-armed after setup, and
# the lpf integrates transients).  The clk_fpga_0->adc_clk config CDC is already
# false-pathed above.
set_multicycle_path 2 -setup -from [get_cells -hierarchical -filter {NAME =~ *i_dsp/input_select_reg* && IS_SEQUENTIAL}]
set_multicycle_path 1 -hold  -from [get_cells -hierarchical -filter {NAME =~ *i_dsp/input_select_reg* && IS_SEQUENTIAL}]

# Quasi-static sys-bus config regs: static while running but fan into pll_adc_clk
# logic. Relax setup to 2 (-from), hold 1; same as input_select/fft_nfft above.
# Signal names are explicit; wildcards are used ONLY for the looped instance
# (ASG channel a/b/c/d, dsp genblk index) and the register bit index. Strobe/reset
# config (rst, zero, steping, trig_sw, *_trig, *_on, dsp sync) is excluded, and so
# are the HW-updated set_*_axi_cur / set_*_axi_trig DMA pointers.

# ASG (i_asg, channel * = a/b/c/d). NCO config (set_*_size/step/ofs -> dac_npnt =
# binding path) + set_*_amp, set_*_dc, set_*_ncyc, set_*_rnum, set_*_rdly, at_counts_*.
set asg_cfg [get_cells {
    i_asg/set_*_size_reg[*]  i_asg/set_*_step_reg[*]  i_asg/set_*_ofs_reg[*]
    i_asg/set_*_amp_reg[*]   i_asg/set_*_dc_reg[*]    i_asg/set_*_ncyc_reg[*]
    i_asg/set_*_rnum_reg[*]  i_asg/set_*_rdly_reg[*]  i_asg/at_counts_*_reg[*]
}]
set_multicycle_path 2 -setup -from $asg_cfg
set_multicycle_path 1 -hold  -from $asg_cfg

# set_*_steping is excluded from the strobe carve-out above ONLY for its cross-
# module fan-in to the scope (step_o gating -> i_scope hist/zigzag index): it is
# a mode bit written once at scan configuration, static while scanning, and an
# 8 ns-late enable of the position stepping is sub-point. It became the binding
# pll_adc_clk path (-0.037, i_scope fft_hist_index DSP CE) on the 100%-full
# ramp-enabled n11 die. Kept tight INSIDE the asg (waveform-start gating).
set asg_steping [get_cells {i_asg/set_*_steping_reg}]
set_multicycle_path 2 -setup -from $asg_steping -to [get_cells i_scope/*]
set_multicycle_path 1 -hold  -from $asg_steping -to [get_cells i_scope/*]

# PID (i_dsp/<genblk>*.i_pid). set_sp, set_kp, set_ki, set_kd, set_filter,
# out_min, out_max  (-> pid_out).
set pid_cfg [get_cells {
    i_dsp/*.i_pid/set_sp_reg[*]   i_dsp/*.i_pid/set_kp_reg[*]
    i_dsp/*.i_pid/set_ki_reg[*]   i_dsp/*.i_pid/set_kd_reg[*]
    i_dsp/*.i_pid/set_filter_reg[*]
    i_dsp/*.i_pid/out_min_reg[*]  i_dsp/*.i_pid/out_max_reg[*]
}]
set_multicycle_path 2 -setup -from $pid_cfg
set_multicycle_path 1 -hold  -from $pid_cfg

# IQ (i_dsp/<genblk>*.iq). g1, g2, g3, g4, input_filter, quadrature_filter,
# start_phase, shift_phase (static demod freq/phase words).
set iq_cfg [get_cells {
    i_dsp/*.iq/g1_reg[*]  i_dsp/*.iq/g2_reg[*]  i_dsp/*.iq/g3_reg[*]  i_dsp/*.iq/g4_reg[*]
    i_dsp/*.iq/input_filter_reg[*]      i_dsp/*.iq/quadrature_filter_reg[*]
    i_dsp/*.iq/start_phase_reg[*]       i_dsp/*.iq/shift_phase_reg[*]
}]
set_multicycle_path 2 -setup -from $iq_cfg
set_multicycle_path 1 -hold  -from $iq_cfg

# Trigger (i_dsp/<genblk>*.i_trigger). set_a_thresh, set_a_hyst, set_filter,
# phase_offset, trigger_source (detection input already pipelined in RTL).
set trig_cfg [get_cells {
    i_dsp/*.i_trigger/set_a_thresh_reg[*]  i_dsp/*.i_trigger/set_a_hyst_reg[*]
    i_dsp/*.i_trigger/set_filter_reg[*]    i_dsp/*.i_trigger/phase_offset_reg[*]
    i_dsp/*.i_trigger/trigger_source_reg[*]
}]
set_multicycle_path 2 -setup -from $trig_cfg
set_multicycle_path 1 -hold  -from $trig_cfg

# DSP routing muxes (i_dsp). output_select, scan_select (select lines only).
set dsp_cfg [get_cells {
    i_dsp/output_select_reg[*]  i_dsp/scan_select_reg[*]
}]
set_multicycle_path 2 -setup -from $dsp_cfg
set_multicycle_path 1 -hold  -from $dsp_cfg

# Scope (i_scope). set_a_tresh, set_a_hyst, set_dec, set_dly, fft_peak_start,
# fft_threshold_k, fft_peak_minimum, fft_wait1_cnt, fft_wait2_cnt, fft_acq1_cnt,
# fft_acq2_cnt, scope_sig_dly, plus the Scanner360 config: enc_ctrl (0x1A8:
# enable/gates/divider/glitch/kick), az_modulus (0x1AC, ticks per turn) and
# dma_az_en (0x9C[1], azimuth scan-cell mode). All are written once by the host
# before a scan; enc_ctrl/az_modulus fan into i_enc's compare/reload logic and
# dma_az_en into the hist-index DSP (RSTA/C mux) and the ASM (packet-sampled
# through xpm_cdc), so a 2-cycle settle is invisible.
# (Input-filter set_*_filt_* and AXI-DMA set_*_axi_* are commented out in scope.sv
#  — registers undriven/optimized away, so they are NOT constrained here.)
set scope_cfg [get_cells {
    i_scope/set_a_tresh_reg[*]  i_scope/set_a_hyst_reg[*]
    i_scope/set_dec_reg[*]      i_scope/set_dly_reg[*]
    i_scope/fft_peak_start_reg[*]    i_scope/fft_threshold_k_reg[*]
    i_scope/fft_peak_minimum_reg[*]
    i_scope/fft_wait1_cnt_reg[*]  i_scope/fft_wait2_cnt_reg[*]
    i_scope/fft_acq1_cnt_reg[*]   i_scope/fft_acq2_cnt_reg[*]
    i_scope/scope_sig_dly_reg[*]
    i_scope/enc_ctrl_reg[*]     i_scope/az_modulus_reg[*]
    i_scope/dma_az_en_reg
}]
set_multicycle_path 2 -setup -from $scope_cfg
set_multicycle_path 1 -hold  -from $scope_cfg

# DAC output-register SYNCHRONOUS RESET (dac_rst -> oddr_dac_dat/oddr_dac_sel .R).
# dac_rst = ~frstn[0] | ~pll_locked (red_pitaya_top.v): a constant 0 during all
# normal operation, asserting only at power-on/fabric-reset or PLL loss-of-lock.
# The only edge whose capture cycle could matter is reset RELEASE at startup, when
# the DAC/galvo output is meaningless and unused — a sub-cycle skew across the 14
# bits there is an invisible power-on transient. So this reset never needs
# single-cycle timing; relax setup to 2 (hold 1), same idiom as the config regs
# above. Scoped to the .R pins of the data/sel ODDRs ONLY — NOT oddr_dac_rst, whose
# .D1/.D2 carry dac_rst as DATA to the external DAC and must stay fully timed.
set dac_rst_src  [get_cells -hierarchical -filter {NAME =~ *dac_rst_reg}]
set dac_oddr_rst [get_cells -hierarchical -filter {NAME =~ *oddr_dac_dat* || NAME =~ *oddr_dac_sel*}]
set_multicycle_path 2 -setup -from $dac_rst_src -to $dac_oddr_rst
set_multicycle_path 1 -hold  -from $dac_rst_src -to $dac_oddr_rst

# ADC data hold: adc_dat_*_i input_delay is referenced to adc_clk, but the
# IOB register is clocked by pll_adc_clk (large internal skew vs adc_clk).
# Hold analysis across these two clocks is not meaningful — suppress it.
set_false_path -hold -from [get_clocks adc_clk] -to [get_clocks pll_adc_clk]
# set_false_path -from [get_clocks dac_clk_out] -to [get_clocks dac_2clk_out]
# set_false_path -from [get_clocks dac_clk_out] -to [get_clocks dac_2ph_out]

############################################################################
# Per-FFT-implementation timing / floorplan constraints                    #
############################################################################
# These depend on which FFT core is instantiated and are kept in separate
# constraint files (sdc/fft_*.xdc) read conditionally from red_pitaya_vivado.tcl
# based on $fft_impl.  They cannot live here behind a tcl `if`: read_xdc uses a
# restricted interpreter that rejects `if`/`set`/`expr` ("Command 'if' is not
# supported in the xdc constraint file").  The generic system/CDC exceptions
# above (clk_fpga->adc, ser<->adc CDC, pwm, axi/sys_rdata, fft_nfft, adc hold)
# hold for every impl and stay global here.
#   fft_xfft_ce.xdc      - ce_predicted multicycle  (FFT_IMPL 1 / 3 / 4)
#   fft_impl3_pblock.xdc - fft_b pre_0 regslice pblock (FFT_IMPL 3)
#   fft_impl4_pblock.xdc - adc sum / sum1 replica pblock (FFT_IMPL 4)

