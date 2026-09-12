# Full IMPL=4 chain sim (tb_fft_chain.sv): real fft_proc.sv + fft_ssr_native_bd +
# peak_detector_bd, scope-FSM model. Proves the realtime-throttle feed contract.
#   vivado -mode batch -source sim/run_sim_chain.tcl
# Env: FFT_THROTTLE (realtime|nonrealtime), CHAIN_DEFS ("ACQ_UP=2048 WAIT1=0 ..."),
#      CHAIN_TAG (project dir suffix for parallel runs).
# Needs the ssr4n11 HLS IPs in .hls (fft_native_pre/mag DSZ=24, peak_detector cfar+ramp).
set part xc7z020clg400-1
set origin [file normalize [file dirname [info script]]]
set root   [file normalize "$origin/.."]
cd $root

set fft_ssr    4
set fft_nfft   11
set fft_scaled 2
set fft_width  24
set peak_algo  cfar
set peak_ramp  1
set fft_throttle realtime
if {[info exists ::env(FFT_THROTTLE)] && $::env(FFT_THROTTLE) ne ""} { set fft_throttle $::env(FFT_THROTTLE) }
set defs "PEAK_CFAR PEAK_RAMP"
if {[info exists ::env(CHAIN_DEFS)] && $::env(CHAIN_DEFS) ne ""} { set defs "$defs $::env(CHAIN_DEFS)" }
set tag ""
if {[info exists ::env(CHAIN_TAG)]} { set tag $::env(CHAIN_TAG) }
puts "==> throttle=$fft_throttle defines: $defs"

create_project -force tb_fft_chain$tag $origin/proj_chain$tag -part $part
set_property XPM_LIBRARIES {XPM_FIFO XPM_CDC XPM_MEMORY} [current_project]
set_property ip_repo_paths [list \
    $root/.hls/fft_native_pre/solution1/impl/ip \
    $root/.hls/fft_native_mag/solution1/impl/ip \
    $root/.hls/peak_detector/solution1/impl/ip ] [current_project]
update_ip_catalog

source $root/ip/fft_ssr_native_bd.tcl
set bd [get_files fft_ssr_native_bd.bd]
generate_target {synthesis simulation} $bd
make_wrapper -files $bd -top -import

source $root/ip/peak_detector_bd.tcl
set bd2 [get_files peak_detector_bd.bd]
generate_target {synthesis simulation} $bd2
make_wrapper -files $bd2 -top -import

add_files -norecurse $root/rtl/fft_proc.sv
add_files -fileset sim_1 -norecurse $origin/tb_fft_chain.sv
set_property top tb_fft_chain [get_filesets sim_1]
set_property verilog_define $defs [get_filesets sources_1]
set_property verilog_define $defs [get_filesets sim_1]
set_property -name {xsim.simulate.runtime} -value {5ms} -objects [get_filesets sim_1]
launch_simulation
puts "==> sim finished"
