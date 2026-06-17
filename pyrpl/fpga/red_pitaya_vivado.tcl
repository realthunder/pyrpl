################################################################################
# Vivado tcl script for building RedPitaya FPGA in non project mode
#
# Usage:
# vivado -mode tcl -source red_pitaya_vivado.tcl
################################################################################

################################################################################
# define paths
################################################################################

set_param general.maxThreads 8

set path_rtl rtl
set path_ip  ip
set path_sdc sdc

set path_out out
set path_sdk sdk

file mkdir $path_out
file mkdir $path_sdk

################################################################################
# setup an in memory project
################################################################################

set part           [expr {[info exists env(FPGA_PART)]      ? $env(FPGA_PART)      : "xc7z020clg400-1"}]
set clk_diff       [expr {[info exists env(CLK_DIFF)]       ? $env(CLK_DIFF)       : 1}]
set clk_mult       [expr {[info exists env(CLK_MULT)]       ? $env(CLK_MULT)       : 8}]
set clk_adc_div    [expr {[info exists env(CLK_ADC_DIV)]    ? $env(CLK_ADC_DIV)    : 8}]
set adc_sz         [expr {[info exists env(ADC_SZ)]         ? $env(ADC_SZ)         : 14}]
set fft_width      [expr {[info exists env(FFT_WIDTH)]      ? $env(FFT_WIDTH)      : 28}]
set fft_nfft       [expr {[info exists env(FFT_NFFT)]       ? $env(FFT_NFFT)       : 12}]
set fft_ssr        [expr {[info exists env(FFT_SSR)]        ? $env(FFT_SSR)        : 2}]
set fft_clk_period [expr {[info exists env(FFT_CLK_PERIOD)] ? $env(FFT_CLK_PERIOD) : 4.0}]
# FFT_IMPL: 1=plain LogiCORE, 2=HLS SSR (Vitis library), 3=IP SSR (LogiCORE sub-FFTs, natural order)
set fft_impl       [expr {[info exists env(FFT_IMPL)]       ? $env(FFT_IMPL)       : 3}]

if {[llength $argv] > 1 && [lindex $argv 0] == "alinx"} {
    set clk_diff 0
    set adc_sz 12
    set clk_mult 20
    set clk_adc_div 4
    set argv [lrange $argv 1 end]
}

if {[llength $argv] > 1 && [lindex $argv 0] == "hls"} {
    source hls/fft_ssr.tcl
    exit
}

create_project -in_memory -part $part

# experimental attempts to avoid a warning
#get_projects
#get_designs
#list_property  [current_project]
#set_property FAMILY 7SERIES [current_project]
#set_property SIM_DEVICE 7SERIES [current_project]

################################################################################
# create PS BD (processing system block design)
################################################################################

# file was created from GUI using "write_bd_tcl -force ip/system_bd.tcl"
# create PS BD
source                            $path_ip/system_bd.tcl

# generate SDK files
generate_target all [get_files    system.bd]
write_hwdef              -file    $path_sdk/red_pitaya.hwdef


if {$fft_impl == 1} {
    source                        $path_ip/fft_bd.tcl
    generate_target all [get_files fft.bd]
} elseif {$fft_impl == 2} {
    source                        $path_ip/fft_ssr_bd.tcl
    generate_target all [get_files fft_ssr_bd.bd]
} else {
    source                        $path_ip/fft_ip_ssr_bd.tcl
    generate_target all [get_files fft_ip_ssr_bd.bd]
}

source                            $path_ip/peak_detector_bd.tcl
generate_target all [get_files    peak_detector_bd.bd]

################################################################################
# read files:
# 1. RTL design sources
# 2. IP database files
# 3. constraints
################################################################################

# template
#read_verilog                      $path_rtl/...

read_verilog                      .srcs/sources_1/bd/system/hdl/system_wrapper.v

if {$fft_impl == 1} {
    read_verilog                  .srcs/sources_1/bd/fft/hdl/fft_wrapper.v
} elseif {$fft_impl == 2} {
    read_verilog                  .srcs/sources_1/bd/fft_ssr_bd/hdl/fft_ssr_bd_wrapper.v
} else {
    read_verilog                  .srcs/sources_1/bd/fft_ip_ssr_bd/hdl/fft_ip_ssr_bd_wrapper.v
}

read_verilog                      .srcs/sources_1/bd/peak_detector_bd/hdl/peak_detector_bd_wrapper.v

read_verilog                      $path_rtl/axi_master.v
read_verilog                      $path_rtl/axi_slave.v
read_verilog                      $path_rtl/axi_wr_fifo.v

read_verilog                      $path_rtl/fft_proc.sv

read_verilog                      $path_rtl/red_pitaya_ams.v
read_verilog                      $path_rtl/red_pitaya_asg_ch.v
read_verilog                      $path_rtl/red_pitaya_asg.v
read_verilog                      $path_rtl/red_pitaya_dfilt1.v
read_verilog                      $path_rtl/red_pitaya_hk.v
read_verilog                      $path_rtl/red_pitaya_pid_block.v
read_verilog                      $path_rtl/red_pitaya_dsp.v
read_verilog                      $path_rtl/red_pitaya_pll.sv
read_verilog                      $path_rtl/red_pitaya_ps.v
read_verilog                      $path_rtl/red_pitaya_pwm.sv
read_verilog                      $path_rtl/red_pitaya_scope.sv
read_verilog                      $path_rtl/red_pitaya_top.v

#custom modules
read_verilog                      $path_rtl/red_pitaya_adv_trigger.v
read_verilog                      $path_rtl/red_pitaya_saturate.v
read_verilog                      $path_rtl/red_pitaya_product_sat.v
read_verilog                      $path_rtl/red_pitaya_iir_block.v
read_verilog                      $path_rtl/red_pitaya_iq_modulator_block.v
read_verilog                      $path_rtl/red_pitaya_lpf_block.v
read_verilog                      $path_rtl/red_pitaya_filter_block.v
#read_verilog                     $path_rtl/red_pitaya_iq_lpf_block.v
read_verilog                      $path_rtl/red_pitaya_iq_demodulator_block.v
read_verilog                      $path_rtl/red_pitaya_pfd_block.v
#read_verilog                     $path_rtl/red_pitaya_iq_hpf_block.v
read_verilog                      $path_rtl/red_pitaya_iq_fgen_block.v
read_verilog                      $path_rtl/red_pitaya_iq_block.v
read_verilog                      $path_rtl/red_pitaya_trigger_block.v
read_verilog                      $path_rtl/red_pitaya_prng.v

read_verilog                      [glob $path_rtl/../elements/*.v]

#constraints
read_xdc                          $path_sdc/red_pitaya.xdc

################################################################################
# run synthesis
# report utilization and timing estimates
# write checkpoint design
################################################################################

#synth_design -top red_pitaya_top
synth_design -top red_pitaya_top -flatten_hierarchy none -bufg 16 -keep_equivalent_registers \
    -generic ADC_SZ=$adc_sz \
    -generic CLK_DIFF=$clk_diff \
    -generic CLK_MULT=$clk_mult \
    -generic CLK_ADC_DIV=$clk_adc_div \
    -generic FFT_NFFT=$fft_nfft \
    -generic FFT_SSR=$fft_ssr \
    -generic FFT_WIDTH=$fft_width \
    -generic FFT_IMPL=$fft_impl \

# set debug_nets {asg_trig_n asg_trig2_p fft_dvalid fft_a_enable fft_b_enable}
# set debug_nets {}
set debug_nets [get_nets -hierarchical -filter {MARK_DEBUG == 1}]

if {[llength $debug_nets] > 0} {
    puts "INFO: debug probe nets $debug_nets"
    create_debug_core u_ila_0 ila
    set_property C_DATA_DEPTH 1024 [get_debug_cores u_ila_0]

    set_property port_width 1 [get_debug_ports u_ila_0/clk]
    connect_debug_port u_ila_0/clk [get_nets [list adc_clk]]

    set probe_idx 0
    foreach net $debug_nets {
        set nets [get_nets -hier $net]
        # set_property mark_debug true $nets
        set net_width [llength $nets]
        set probe_port_name "probe$probe_idx"
        if {$probe_idx > 0} {
            create_debug_port u_ila_0 probe
        }
        set_property port_width $net_width [get_debug_ports u_ila_0/$probe_port_name]
        connect_debug_port u_ila_0/$probe_port_name $nets
        puts "INFO: Connected $net (Width: $net_width) to $probe_port_name."
        incr probe_idx
    }
}

write_checkpoint         -force   $path_out/post_synth
report_timing_summary    -file    $path_out/post_synth_timing_summary.rpt
report_power             -file    $path_out/post_synth_power.rpt

################################################################################
# run placement and logic optimization
# report utilization and timing estimates
# write checkpoint design
################################################################################

# set multicyle_path_to  [concat [get_cells i_dsp*/*iir*/p_*reg*] \
#                                [get_cells i_dsp*/*iir*/overflow_reg*] \
#                        ]
# set_multicycle_path -setup 2 -to $multicyle_path_to
# set_multicycle_path -hold 1 -to $multicyle_path_to

# set multicyle_path [concat [get_cells i_scope*/fft_nfft*] \
#                            [get_cells i_scope*/fft*/fft_nfft*] \
#                            [get_cells i_scope*/fft*/*_arg_*] \
#                    ]
# set_multicycle_path -hold 1 -from $multicyle_path
# set_multicycle_path -setup 2 -from $multicyle_path
# set_multicycle_path -hold 1 -to $multicyle_path
# set_multicycle_path -setup 2 -to $multicyle_path
set_false_path -from [get_cells -hier -filter {NAME =~ *fft_threshold_k_arg*}] \
				-to   [get_cells -hier -filter {NAME =~ *pd_i*}]
set_false_path -from [get_cells -hier -filter {NAME =~ *fft_peak_start_arg*}] \
				-to   [get_cells -hier -filter {NAME =~ *pd_i*}]
set_false_path -from [get_cells -hier -filter {NAME =~ *fft_peak_minimum_arg*}] \
				-to   [get_cells -hier -filter {NAME =~ *pd_i*}]

# opt_design
opt_design -directive NoBramPowerOpt
# power_opt_design
place_design

# phys_opt_design
# phys_opt_design -directive AggressivePhysOptimization  (Vivado 2021+)
phys_opt_design -directive AggressiveExplore

# Force replication of the xfft NonRealTime CE register (ce_predicted_reg, fo≈1000+).
# Its Q output drives all pipeline CEs across the full IP footprint (94% routing delay).
# Target the Q pin explicitly so we replicate the right register, not its D-input logic.
# -force_replication_on_nets overrides DONT_TOUCH on the xfft IP cells.
set ce_pins [get_pins -hier -quiet -filter {NAME =~ *fft_ip_ssr_bd_i*ce_predicted_reg/Q}]
if {[llength $ce_pins] > 0} {
    set ce_nets [get_nets -of_objects $ce_pins]
    phys_opt_design -force_replication_on_nets $ce_nets
}

# Force replication of peak_detector pipeline-enable registers (ap_enable_reg_pp0_iter*).
# Same pattern: high-routing-delay enable signals feeding DSP C-inputs via LUT6.
# Logic delay alone (2.7 ns) fits in 4 ns; replication cuts the 2.9 ns routing overhead.
set pd_en_pins [get_pins -hier -quiet -filter {NAME =~ *peak_detector_0*ap_enable_reg_pp0_iter*/Q}]
if {[llength $pd_en_pins] > 0} {
    set pd_en_nets [get_nets -of_objects $pd_en_pins]
    phys_opt_design -force_replication_on_nets $pd_en_nets
}

# Force replication of i_dsp/sum1_reg (pll_adc_clk).
# sum1_reg feeds a 12-LUT read-data mux tree to sys_rdata_reg (6.111 ns routing).
# Replication plants a copy near the mux sinks to cut the long route.
set sum1_pins [get_pins -hier -quiet -filter {NAME =~ i_dsp/sum1_reg*/Q}]
if {[llength $sum1_pins] > 0} {
    set sum1_nets [get_nets -of_objects $sum1_pins]
    phys_opt_design -force_replication_on_nets $sum1_nets
}

write_checkpoint         -force   $path_out/post_place
report_timing_summary    -file    $path_out/post_place_timing_summary.rpt
#write_hwdef              -file    $path_sdk/red_pitaya.hwdef

# Write the debug probes information to a file
write_debug_probes -force $path_out/debug_probes.ltx


################################################################################
# run router
# report actual utilization and timing,
# write checkpoint design
# run drc, write verilog and xdc out
################################################################################

route_design
# Post-route phys_opt disabled: both AggressiveExplore and Default directives
# create new −2.6 ns setup paths + 8521 hold violations from a clean −0.512 ns
# baseline. Address remaining violations with targeted RTL/constraint changes.
write_checkpoint         -force   $path_out/post_route
report_timing_summary    -file    $path_out/post_route_timing_summary.rpt
report_timing            -file    $path_out/post_route_timing.rpt -sort_by group -max_paths 1000 -path_type summary
report_clock_utilization -file    $path_out/clock_util.rpt
# report_utilization       -file    $path_out/post_route_util.rpt
report_utilization       -file    $path_out/post_route_util.rpt -hierarchical -hierarchical_depth 3
report_power             -file    $path_out/post_route_power.rpt
report_drc               -file    $path_out/post_imp_drc.rpt
#write_verilog            -force   $path_out/bft_impl_netlist.v
write_xdc -no_fixed_only -force   $path_out/bft_impl.xdc

report_timing -slack_lesser_than 0 -max_paths 20000 -file $path_out/tns_failing_paths.txt

################################################################################
# generate a bitstream
################################################################################

set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
write_bitstream -force $path_out/red_pitaya.bit

################################################################################
# generate the .bin file for flashing via 'cat red_pitaya.bin > /dev/xdevcfg'
################################################################################

# This is not working
## write_bitstream -force -bin_file  red_pitaya

# This may works, but need bif file
## exec bootgen -image $path_out/red_pitaya.bif -arch zynq -process_bitstream bin -o red_pitaya.bin -w

set_property BITSTREAM.GENERAL.COMPRESS FALSE [current_design]
write_bitstream -force $path_out/red_pitaya_uncompressed.bit
write_cfgmem -force -format BIN -size 4 -interface SMAPx32 -disablebitswap -loadbit "up 0x0 $path_out/red_pitaya_uncompressed.bit" red_pitaya.bin

################################################################################
# generate system definition
################################################################################

write_sysdef             -hwdef   $path_sdk/red_pitaya.hwdef \
                         -bitfile $path_out/red_pitaya.bit \
                         -file    $path_sdk/red_pitaya.sysdef


