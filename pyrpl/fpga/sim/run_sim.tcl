# Behavioral sim of the native-SSR xfft BD to determine its true input->output
# SSR ordering. Usage:
#   vivado -mode batch -source sim/run_sim.tcl
# Verilog defines (tone bin, input packing) via env SIM_DEFS, e.g.:
#   SIM_DEFS="TONE_K=32 INPUT_REVERSE" vivado -mode batch -source sim/run_sim.tcl
set part xc7z020clg400-1
set origin [file normalize [file dirname [info script]]]
set root   [file normalize "$origin/.."]
cd $root

# globals consumed by ip/fft_ssr_native_bd.tcl
set fft_ssr    4
set fft_nfft   11
set fft_scaled 2

set defs "TONE_K=32"
if {[info exists ::env(SIM_DEFS)] && $::env(SIM_DEFS) ne ""} { set defs $::env(SIM_DEFS) }
puts "==> SIM defines: $defs"

create_project -force tb_fft_native $origin/proj -part $part

set_property ip_repo_paths [list \
    $root/.hls/fft_native_pre/solution1/impl/ip \
    $root/.hls/fft_native_mag/solution1/impl/ip ] [current_project]
update_ip_catalog

# Build the block design (adds fft_ssr_native_bd to the project)
source $root/ip/fft_ssr_native_bd.tcl
set bd [get_files fft_ssr_native_bd.bd]
generate_target {synthesis simulation} $bd
make_wrapper -files $bd -top -import

# Testbench
add_files -fileset sim_1 -norecurse $origin/tb_fft_native.sv
set_property top tb_fft_native [get_filesets sim_1]
set_property verilog_define $defs [get_filesets sim_1]
set_property -name {xsim.simulate.runtime} -value {500us} -objects [get_filesets sim_1]

launch_simulation
puts "==> simulation finished"
