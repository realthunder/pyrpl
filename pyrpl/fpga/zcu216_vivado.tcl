################################################################################
# Vivado tcl script for building the ZCU216 (XCZU49DR RFSoC) image in non
# project mode. The Red Pitaya build stays in red_pitaya_vivado.tcl.
#
# Register bus: the PS reaches the Red Pitaya register map at 0x8000_0000
# (Red Pitaya: 0x4000_0000), same offsets; see rtl/zcu216_top.sv.
#
# FFT (slot 1), set by make.sh: FFT_IMPL=6 (0 = shell only), FFT_NFFT = log2 of
# the real FFT length, FFT_CH = channels. The eight xfft sub-cores per channel
# are generated here (xfft_sub, 2^(FFT_NFFT-3) points), and the twiddle and sine
# tables are written to out/ (zcu216/gen_tables.tcl).
#
# Usage (selects this script and the part):
#   BOARD=zcu216 ./make.sh
#
# Outputs in out/: zcu216.bit, zcu216.bit.bin (for fpgautil -b on the board's
# Linux), checkpoints and reports; sdk/zcu216.xsa for PetaLinux.
################################################################################

# DETERMINISTIC as in red_pitaya_vivado.tcl: 0 = 8-thread P&R with the Default
# place directive (not reproducible), >=1 = single-threaded with a directive
# picked from the list, giving a bit-identical bitstream.
set det_place_dirs {Explore ExtraNetDelay_high AltSpreadLogic_high WLDrivenBlockPlacement ExtraPostPlacementOpt EarlyBlockPlacement AltSpreadLogic_medium Default ExtraTimingOpt ExtraNetDelay_low AltSpreadLogic_low}
set det_seed 0
if {[info exists env(DETERMINISTIC)] && [string is integer -strict $env(DETERMINISTIC)] \
        && $env(DETERMINISTIC) >= 1} {
    set det_seed $env(DETERMINISTIC)
}
set deterministic [expr {$det_seed >= 1}]
# MAX_THREADS: thread count of a non-deterministic build (default 8).
set max_threads [expr {[info exists env(MAX_THREADS)] ? $env(MAX_THREADS) : 8}]
set_param general.maxThreads [expr {$deterministic ? 1 : $max_threads}]
if {$deterministic} {
    set det_dir [lindex $det_place_dirs [expr {($det_seed-1) % [llength $det_place_dirs]}]]
    puts "INFO: DETERMINISTIC build — maxThreads 1, place_design -directive $det_dir (DETERMINISTIC=$det_seed)"
} else {
    puts "INFO: non-deterministic build (maxThreads $max_threads); set DETERMINISTIC=<n> for reproducible directive-swept P&R"
}

set path_rtl rtl
set path_ip  ip
set path_sdc sdc
set path_out out
set path_sdk sdk

file mkdir $path_out
file mkdir $path_sdk

################################################################################
# in-memory project on the ZCU216 board part (the PS preset needs it)
################################################################################

set part       [expr {[info exists env(FPGA_PART)] ? $env(FPGA_PART) : "xczu49dr-ffvf1760-2-e"}]
set board_part xilinx.com:zcu216:part0:2.0

create_project -in_memory -part $part
set_property board_part $board_part [current_project]

set fft_impl [expr {[info exists env(FFT_IMPL)] ? $env(FFT_IMPL) : 6}]
set fft_nfft [expr {[info exists env(FFT_NFFT)] ? $env(FFT_NFFT) : 18}]
set fft_ch   [expr {[info exists env(FFT_CH)]   ? $env(FFT_CH)   : 2}]
set fft_en   [expr {$fft_impl == 6}]
if {$fft_impl != 0 && $fft_impl != 6} { error "zcu216: FFT_IMPL must be 6 or 0, got $fft_impl" }
if {$fft_en && ($fft_nfft < 6 || $fft_nfft > 19)} {
    error "zcu216: FFT_NFFT=$fft_nfft out of range (sub-FFT 2^(NFFT-3) must be 8..64k)"
}
puts "INFO: FFT_IMPL=$fft_impl FFT_NFFT=$fft_nfft FFT_CH=$fft_ch"

source                            $path_ip/system_bd_zcu216.tcl
generate_target all [get_files    system_zcu216.bd]

# Vivado 2025.2+ generates BD wrappers into .gen/ instead of .srcs/.
proc read_bd_wrapper {name} {
    foreach base {.gen .srcs} {
        set f "$base/sources_1/bd/$name/hdl/${name}_wrapper.v"
        if {[file exists $f]} { read_verilog $f; return }
    }
    error "Cannot find BD wrapper for '$name' in .gen/ or .srcs/"
}

read_bd_wrapper system_zcu216

# Shared with the Red Pitaya build: the AXI-to-system-bus bridge and hk.
read_verilog                      $path_rtl/axi_slave.v
read_verilog                      $path_rtl/red_pitaya_hk.v
read_verilog -sv                  $path_rtl/zcu216_top.sv

if {$fft_en} {
    # sub-FFT: 1 sample/clock, unscaled, bit-reversed output with xk_index (with
    # super_sample_rates > 1 xfft forces natural order and drops xk_index)
    create_ip -name xfft -vendor xilinx.com -library ip -version 9.1 \
              -module_name xfft_sub -dir $path_out/ip -force
    set_property -dict [list \
        CONFIG.aresetn                                {true} \
        CONFIG.transform_length                       [expr {1 << ($fft_nfft - 3)}] \
        CONFIG.data_format                            {fixed_point} \
        CONFIG.implementation_options                 {pipelined_streaming_io} \
        CONFIG.input_width                            {16} \
        CONFIG.phase_factor_width                     {18} \
        CONFIG.rounding_modes                         {convergent_rounding} \
        CONFIG.run_time_configurable_transform_length {false} \
        CONFIG.scaling_options                        {unscaled} \
        CONFIG.target_clock_frequency                 {312.5} \
        CONFIG.throttle_scheme                        {realtime} \
    ] [get_ips xfft_sub]
    set_property CONFIG.output_ordering {bit_reversed_order} [get_ips xfft_sub]
    set_property CONFIG.xk_index        {true}               [get_ips xfft_sub]
    foreach p {transform_length output_ordering xk_index} {
        puts "INFO: xfft_sub $p = [get_property CONFIG.$p [get_ips xfft_sub]]"
    }
    set_property generate_synth_checkpoint false [get_files xfft_sub.xci]
    generate_target {synthesis} [get_ips xfft_sub]

    source zcu216/gen_tables.tcl
    zcu216_gen_tables $path_out $fft_nfft

    read_verilog -sv              $path_rtl/zcu216_fft_dit8.sv
    read_verilog -sv              $path_rtl/zcu216_tone_gen.sv
    read_verilog -sv              $path_rtl/zcu216_fft_spec.sv
    read_verilog -sv              $path_rtl/zcu216_fft_test.sv
}

read_xdc                          $path_sdc/zcu216.xdc

################################################################################
# synthesis
################################################################################

synth_design -top zcu216_top -flatten_hierarchy none \
    -generic FFT_EN=$fft_en -generic FFT_NFFT=$fft_nfft -generic FFT_NCH=$fft_ch

# pl_clk0 (register bus) and fclk (FFT): every crossing is a quasi-static
# setting, a per-frame toggle handshake or the dual-clock window RAM
if {$fft_en} {
    set_clock_groups -asynchronous \
        -group [get_clocks clk_pl_0] \
        -group [get_clocks -of_objects [get_pins -hier -filter {NAME =~ *i_mmcm/CLKOUT0}]]
}

write_checkpoint         -force   $path_out/post_synth
report_timing_summary    -file    $path_out/post_synth_timing_summary.rpt
report_utilization       -file    $path_out/post_synth_util.rpt

################################################################################
# place, physical optimisation, route
################################################################################

set opt_dir [expr {[info exists env(OPT_DIRECTIVE)] ? $env(OPT_DIRECTIVE) : "Default"}]
puts "INFO: opt_design -directive $opt_dir"
opt_design -directive $opt_dir

if {$deterministic} { place_design -directive $det_dir } else { place_design }

set phys_opt_dir [expr {[info exists env(PHYS_OPT)] ? $env(PHYS_OPT) : "AggressiveExplore"}]
if {[lsearch -exact {none off skip ""} $phys_opt_dir] >= 0} {
    puts "INFO: PHYS_OPT=$phys_opt_dir — skipping phys_opt_design"
} else {
    puts "INFO: phys_opt_design -directive $phys_opt_dir"
    phys_opt_design -directive $phys_opt_dir
}

write_checkpoint         -force   $path_out/post_place
report_timing_summary    -file    $path_out/post_place_timing_summary.rpt

route_design

write_checkpoint         -force   $path_out/post_route
report_timing_summary    -file    $path_out/post_route_timing_summary.rpt
report_clock_utilization -file    $path_out/clock_util.rpt
report_utilization       -file    $path_out/post_route_util.rpt -hierarchical -hierarchical_depth 3
report_power             -file    $path_out/post_route_power.rpt
report_drc               -file    $path_out/post_imp_drc.rpt

if {![catch {open $path_out/BUILD_INFO.txt a} bi]} {
    puts $bi ""
    puts $bi "# ---- Resolved build options (defaults applied) ----"
    foreach {k v} [list board zcu216  fpga_part $part  board_part $board_part \
                        fft_impl $fft_impl  fft_nfft $fft_nfft  fft_ch $fft_ch \
                        opt_dir $opt_dir  phys_opt_dir $phys_opt_dir] {
        puts $bi [format "%-17s= %s" $k $v]
    }
    close $bi
}

################################################################################
# bitstream, the .bit.bin that fpgautil loads, and the XSA for PetaLinux
################################################################################

set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
write_bitstream -force $path_out/zcu216.bit

# bootgen turns the .bit into out/zcu216.bit.bin (header stripped, byte order
# the ZynqMP FPGA manager expects).
set bif [open $path_out/zcu216.bif w]
puts $bif "all:\n{\n    [file normalize $path_out/zcu216.bit]\n}"
close $bif
exec bootgen -arch zynqmp -process_bitstream bin -image $path_out/zcu216.bif -w on
if {![file exists $path_out/zcu216.bit.bin]} {
    error "bootgen did not write $path_out/zcu216.bit.bin"
}

write_hw_platform -fixed -force -include_bit -file $path_sdk/zcu216.xsa
