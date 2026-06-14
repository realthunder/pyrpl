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

# set part xc7z010clg400-1
set part xc7z020clg400-1
# set part xc7z100ffg900-1

set clk_diff 1
set clk_mult 8
set clk_adc_div 8
set adc_sz 14
set fft_width 28
set fft_nfft 12
set fft_ssr 4
set fft_clk_period 4.0  ;# ns (250 MHz)

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


# source                            $path_ip/fft_bd.tcl
# generate_target all [get_files    fft.bd]

source                            $path_ip/fft_ssr_bd.tcl
generate_target all [get_files    fft_ssr.bd]

source                            $path_ip/peak_detector_bd.tcl
generate_target all [get_files    peak_detector.bd]

################################################################################
# read files:
# 1. RTL design sources
# 2. IP database files
# 3. constraints
################################################################################

# template
#read_verilog                      $path_rtl/...

read_verilog                      .srcs/sources_1/bd/system/hdl/system_wrapper.v
# read_verilog                      .srcs/sources_1/bd/fft/hdl/fft_wrapper.v
read_verilog                      .srcs/sources_1/bd/fft_ssr/hdl/fft_ssr_wrapper.v
read_verilog                      .srcs/sources_1/bd/peak_detector/hdl/peak_detector_wrapper.v

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
phys_opt_design -directive AggressiveExplore
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


