################################################################################
# Vivado tcl script for building RedPitaya FPGA in non project mode
#
# Usage:
# vivado -mode tcl -source red_pitaya_vivado.tcl
################################################################################

################################################################################
# define paths
################################################################################

# Reproducibility / placement sweep: DETERMINISTIC selects a place_design
# *directive* (Vivado has no place_design -seed; directives are its deterministic
# placement-variation lever). 0 (default) = fast 8-thread place/route with the
# Default directive, which is NOT run-to-run reproducible — Vivado's parallel
# placer reshuffles the basin per run (the ~0.2 ns pll_adc_clk swing). Any value
# >=1 forces single-threaded P&R (bit-identical/reproducible) and maps the value to
# a directive from det_place_dirs below (1->first, wrapping). Sweep DETERMINISTIC=1..N
# to search placements for one that closes; the winner is reproducible and shippable.
# Set via DETERMINISTIC=<n> ./make.sh; the directive is echoed and recorded in BUILD_INFO.
set det_place_dirs {Explore ExtraNetDelay_high AltSpreadLogic_high WLDrivenBlockPlacement ExtraPostPlacementOpt EarlyBlockPlacement AltSpreadLogic_medium Default}
set det_seed 0
if {[info exists env(DETERMINISTIC)] && [string is integer -strict $env(DETERMINISTIC)] \
        && $env(DETERMINISTIC) >= 1} {
    set det_seed $env(DETERMINISTIC)
}
set deterministic [expr {$det_seed >= 1}]
set_param general.maxThreads [expr {$deterministic ? 1 : 8}]
if {$deterministic} {
    set det_dir [lindex $det_place_dirs [expr {($det_seed-1) % [llength $det_place_dirs]}]]
    puts "INFO: DETERMINISTIC build — maxThreads 1, place_design -directive $det_dir (DETERMINISTIC=$det_seed)"
} else {
    puts "INFO: non-deterministic build (maxThreads 8); set DETERMINISTIC=<n> for reproducible directive-swept P&R"
}

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
set fft_nfft       [expr {[info exists env(FFT_NFFT)]       ? $env(FFT_NFFT)       : 12}]
set fft_ssr        [expr {[info exists env(FFT_SSR)]        ? $env(FFT_SSR)        : 4}]
set fft_clk_period [expr {[info exists env(FFT_CLK_PERIOD)] ? $env(FFT_CLK_PERIOD) : 4.0}]
# FFT_IMPL: 1=plain LogiCORE, 2=HLS SSR (Vitis library), 3=IP SSR (LogiCORE sub-FFTs),
#           4=native-SSR xfft, 5=direct hls::fft (default; SSR=4 @ 125 MHz)
set fft_impl       [expr {[info exists env(FFT_IMPL)]       ? $env(FFT_IMPL)       : 5}]
# FFT_SCALED: 0=unscaled 28-bit (~140 dB, needs large device), 1=scaled 16-bit (~72 dB),
#             2=saturating 20-bit (default, fits xc7z020, ~90 dB small-signal detection)
set fft_scaled     [expr {[info exists env(FFT_SCALED)]     ? $env(FFT_SCALED)     : 2}]
set fft_use_approx    [expr {[info exists env(FFT_USE_APPROX)]    ? $env(FFT_USE_APPROX)    : 1}]
# FFT_RUNTIME_NFFT (IMPL=5 only): run-time configurable FFT transform length.
# Off by default — adds ~2700 LUTs and regresses adc/dac fabric timing on the
# 88%-full xc7z020. When on, drives the nfft port on the BD + a Verilog define so
# fft_proc.sv connects it. Global var name (lowercase) is read by fft_hls_direct_bd.tcl.
set fft_runtime_nfft  [expr {[info exists env(FFT_RUNTIME_NFFT)]  ? $env(FFT_RUNTIME_NFFT)  : 0}]
set hist_block_size   [expr {[info exists env(HIST_BLOCK_SIZE)]   ? $env(HIST_BLOCK_SIZE)   : 183}]
# FFT_CLK_200: retune the PLL's CLKOUT4 (clk_ser, which currently feeds only the
# FFT clock mux) from VCO/4 = 250 MHz to VCO/5 = 200 MHz.  Drives a Verilog define
# read by red_pitaya_pll.sv and redefines the pll_ser_clk generated clock below so
# the FFT is timed at 200 MHz (5 ns).  Use with FFT_CLK_SEL=1 to route it into the
# FFT.  Default 0 = unchanged 250 MHz build.
set fft_clk_200    [expr {[info exists env(FFT_CLK_200)]    ? $env(FFT_CLK_200)    : 0}]
# fft_width = DSZ (magnitude output bits). Override with FFT_WIDTH env var if needed.
if {[info exists env(FFT_WIDTH)]} {
    set fft_width $env(FFT_WIDTH)
} else {
    set ssr_bits_ [expr {int(log($fft_ssr) / log(2) + 0.5)}]
    set sub_nfft_ [expr {$fft_nfft - $ssr_bits_}]
    # 0→28 full unscaled, 1→16 scaled, 2→20 saturating unscaled
    set fft_width [expr {$fft_scaled == 1 ? 16 : ($fft_scaled == 2 ? 20 : (($sub_nfft_ + 14 + 3) / 4 * 4))}]
}

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
# write_hwdef removed in Vivado 2020.2+; replaced by write_hw_platform below after bitstream


if {$fft_impl == 1} {
    source                        $path_ip/fft_bd.tcl
    generate_target all [get_files fft.bd]
} elseif {$fft_impl == 2} {
    source                        $path_ip/fft_ssr_bd.tcl
    generate_target all [get_files fft_ssr_bd.bd]
} elseif {$fft_impl == 4} {
    source                        $path_ip/fft_ssr_native_bd.tcl
    generate_target all [get_files fft_ssr_native_bd.bd]
} elseif {$fft_impl == 5} {
    source                        $path_ip/fft_hls_direct_bd.tcl
    generate_target all [get_files fft_hls_direct_bd.bd]
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

# Vivado 2025.2+ generates BD wrappers into .gen/ instead of .srcs/.
# Search both so the script works across versions.
proc read_bd_wrapper {name} {
    foreach base {.gen .srcs} {
        set f "$base/sources_1/bd/$name/hdl/${name}_wrapper.v"
        if {[file exists $f]} { read_verilog $f; return }
    }
    error "Cannot find BD wrapper for '$name' in .gen/ or .srcs/"
}

read_bd_wrapper system

if {$fft_impl == 1} {
    read_bd_wrapper fft
} elseif {$fft_impl == 2} {
    read_bd_wrapper fft_ssr_bd
} elseif {$fft_impl == 4} {
    read_bd_wrapper fft_ssr_native_bd
} elseif {$fft_impl == 5} {
    read_bd_wrapper fft_hls_direct_bd
} else {
    read_bd_wrapper fft_ip_ssr_bd
}

read_bd_wrapper peak_detector_bd

read_verilog                      $path_rtl/axi_master.v
read_verilog                      $path_rtl/axi_slave.v
read_verilog                      $path_rtl/axi_wr_fifo.v

read_verilog                      $path_rtl/fft_proc.sv
read_verilog                      $path_rtl/dma_s2mm.sv

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
# Verilog define gating the IMPL=5 runtime-nfft port connection in fft_proc.sv;
# must match the IP/BD build (same fft_runtime_nfft env var).
set verilog_defines [expr {$fft_runtime_nfft ? "-verilog_define FFT_RUNTIME_NFFT" : ""}]
if {$fft_clk_200} { lappend verilog_defines -verilog_define FFT_CLK_200 }
synth_design -top red_pitaya_top -flatten_hierarchy none -bufg 16 -keep_equivalent_registers \
    {*}$verilog_defines \
    -generic ADC_SZ=$adc_sz \
    -generic CLK_DIFF=$clk_diff \
    -generic CLK_MULT=$clk_mult \
    -generic CLK_ADC_DIV=$clk_adc_div \
    -generic FFT_NFFT=$fft_nfft \
    -generic FFT_SSR=$fft_ssr \
    -generic FFT_WIDTH=$fft_width \
    -generic FFT_IMPL=$fft_impl \
    -generic HIST_BLOCK_SIZE=$hist_block_size

# Per-FFT-implementation constraints.  read_xdc's restricted interpreter rejects
# tcl `if`/`set`/`expr`, so the impl conditioning lives here (real tcl) and each
# file holds only plain constraint commands.  Read AFTER synth_design so the
# pblock get_cells membership resolves against the synthesized netlist (reading
# before synthesis would match nothing and create silent empty pblocks).
if {$fft_impl == 1 || $fft_impl == 3 || $fft_impl == 4 || $fft_impl == 5} {
    read_xdc                      $path_sdc/fft_xfft_ce.xdc
}
if {$fft_impl == 3} {
    read_xdc                      $path_sdc/fft_impl3_pblock.xdc
}
if {$fft_impl == 4} {
    read_xdc                      $path_sdc/fft_impl4_pblock.xdc
}

# FFT_CLK_200: red_pitaya.xdc declares pll_ser_clk as VCO/4 = 250 MHz, but with
# FFT_CLK_200 the PLL's CLKOUT4 is rebuilt as VCO/5 = 200 MHz (red_pitaya_pll.sv).
# Redefine the generated clock so the FFT is analysed at the real 200 MHz / 5 ns
# (re-issuing create_generated_clock with the same -name replaces the prior one).
# Done in tcl (not the static xdc) because read_xdc rejects the `if` guard.
if {$fft_clk_200} {
    create_generated_clock -name pll_ser_clk -source [get_pins pll/clk] \
        -multiply_by 8 -divide_by 5 [get_pins pll/clk_ser]
    puts "INFO: FFT_CLK_200 — pll_ser_clk redefined to 200 MHz (VCO/5, 5 ns)"
}

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
# Optional: pin the FFT clock mux for timing analysis. The FFT runs off a runtime
# BUFGMUX (i_scope/clk_sel) selecting adc_clk (125 MHz, I0) vs fft_clk (250 MHz, I1)
# via fft_clk_sel. With FFT_CLK_SEL=0 the timing engine analyses the FFT at 125 MHz
# only (e.g. for SSR=4, which is meant to run at 125 MHz). Unset → analyse both
# (default 250 MHz path dominates), so the default build is unaffected.
set fft_clk_sel [expr {[info exists env(FFT_CLK_SEL)] ? $env(FFT_CLK_SEL) : 0}]
if {$fft_clk_sel == 0 || $fft_clk_sel == 1} {
    set sel_q [get_pins -hier -quiet -filter {NAME =~ *fft_clk_sel_i_reg/Q}]
    if {[llength $sel_q] > 0} {
        set_case_analysis $fft_clk_sel $sel_q
        set sel1_freq [expr {$fft_clk_200 ? {200 MHz fft_clk} : {250 MHz fft_clk}}]
        puts "INFO: FFT clock pinned via set_case_analysis fft_clk_sel=$fft_clk_sel \
              ([expr {$fft_clk_sel == 0 ? {125 MHz adc_clk} : $sel1_freq}])"
    } else {
        puts "WARNING: FFT_CLK_SEL set but fft_clk_sel_i_reg/Q pin not found"
    }
}

set_false_path -from [get_cells -hier -filter {NAME =~ *fft_threshold_k_arg*}] \
				-to   [get_cells -hier -filter {NAME =~ *pd_i*}]
set_false_path -from [get_cells -hier -filter {NAME =~ *fft_peak_start_arg*}] \
				-to   [get_cells -hier -filter {NAME =~ *pd_i*}]
set_false_path -from [get_cells -hier -filter {NAME =~ *fft_peak_minimum_arg*}] \
				-to   [get_cells -hier -filter {NAME =~ *pd_i*}]

# opt_design
opt_design -directive NoBramPowerOpt
# power_opt_design
# NOTE: place_design -directive Explore was tried and regressed both clocks
# (adc -0.483->-0.593, ser -0.674->-0.771). The default placer finds a better
# basin for this design; leave it on default.

if {$deterministic} { place_design -directive $det_dir } else { place_design }

# phys_opt_design
# phys_opt_design -directive AggressivePhysOptimization  (Vivado 2021+)
phys_opt_design -directive AggressiveExplore

# adc: i_dsp/sum1_reg (the DSP output summer) is a high-fanout source feeding the
# pyrpl output-bus loopback (sum1 -> dac_saturate -> dat_a -> iq/trigger/... inputs).
# Its route to the IQ input-filter is the lone adc violation (~-0.022); pinning one
# consumer just displaces the others, so instead replicate the high-fanout sum1 nets
# and let phys_opt place a local copy per consumer cluster. This is an adc-clock-
# domain path (i_dsp is always on adc_clk), independent of the FFT clock mux, so run
# it for any FFT_CLK_SEL — at FFT_CLK_SEL=1 this same loopback is the worst adc path.
if {$fft_clk_sel == 0 || $fft_clk_sel == 1} {
    set sum1_pins [get_pins -hier -quiet -filter {NAME =~ i_dsp/sum1_reg*/Q}]
    if {[llength $sum1_pins] > 0} {
        set sum1_nets [get_nets -of_objects $sum1_pins]
        phys_opt_design -force_replication_on_nets $sum1_nets
        puts "INFO: forced replication on [llength $sum1_nets] sum1 nets (adc)"
    }
}

# [2020.1 levers — disabled on 2025.2; FFT is no longer critical]
# Force replication of the xfft NonRealTime CE register (ce_predicted_reg, fo≈1000+).
# Its Q output drives all pipeline CEs across the full IP footprint (94% routing delay).
# Target the Q pin explicitly so we replicate the right register, not its D-input logic.
# -force_replication_on_nets overrides DONT_TOUCH on the xfft IP cells.
# set ce_pins [get_pins -hier -quiet -filter {NAME =~ *fft_ip_ssr_bd_i*ce_predicted_reg/Q}]
# if {[llength $ce_pins] > 0} {
#     set ce_nets [get_nets -of_objects $ce_pins]
#     phys_opt_design -force_replication_on_nets $ce_nets
# }
#
# # Force replication of icmp_ln617_reg_522_pp0_iter7_reg (fo=307 CE loads in post_0).
# set icmp_pins [get_pins -hier -quiet -filter {NAME =~ *post_0*icmp_ln617_reg_522_pp0_iter7_reg_reg*/Q}]
# if {[llength $icmp_pins] > 0} {
#     set icmp_nets [get_nets -of_objects $icmp_pins]
#     phys_opt_design -force_replication_on_nets $icmp_nets
# }
#
# # Force replication of k_reg_152_reg[0] (fo=168 sync-reset loads in post_0).
# set k_pins [get_pins -hier -quiet -filter {NAME =~ *post_0*k_reg_152_reg[0]/Q}]
# if {[llength $k_pins] > 0} {
#     set k_nets [get_nets -of_objects $k_pins]
#     phys_opt_design -force_replication_on_nets $k_nets
# }


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
# Write the .bin into out/ so it is archived alongside the .bit when out/ is
# rotated to out.d/, then copy it to the repo root for flashing
# (cat red_pitaya.bin > /dev/xdevcfg). The .bin is derived from the
# uncompressed .bit, so any archived build's .bin can be regenerated from it.
write_cfgmem -force -format BIN -size 4 -interface SMAPx32 -disablebitswap -loadbit "up 0x0 $path_out/red_pitaya_uncompressed.bit" $path_out/red_pitaya.bin
file copy -force $path_out/red_pitaya.bin red_pitaya.bin

################################################################################
# generate hardware platform (XSA) — replaces write_hwdef/write_sysdef (removed 2020.2+)
################################################################################

write_hw_platform -fixed -force -include_bit -file $path_sdk/red_pitaya.xsa


