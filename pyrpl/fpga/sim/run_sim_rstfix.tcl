# Per-frame fin FIFO reset: mechanism probe (old) vs fix validation (RSTFIX).
# Usage:
#   FIN_TAG=_old                     vivado -mode batch -source sim/run_sim_rstfix.tcl
#   FIN_TAG=_fix  FIN_DEFS=RSTFIX=1  vivado -mode batch -source sim/run_sim_rstfix.tcl
# Optional extra define: BACKPRESSURE=1 (random fft_saxi_rdy stalls).
set part xc7z020clg400-1
set origin [file normalize [file dirname [info script]]]
set root   [file normalize "$origin/.."]
cd $root
set tag ""
if {[info exists ::env(FIN_TAG)]} { set tag $::env(FIN_TAG) }
create_project -force tb_fin_rstfix$tag $origin/proj_rstfix$tag -part $part
set_property XPM_LIBRARIES {XPM_FIFO XPM_CDC} [current_project]
add_files -fileset sim_1 -norecurse $origin/tb_fin_rstfix.sv
set_property top tb_fin_rstfix [get_filesets sim_1]
if {[info exists ::env(FIN_DEFS)] && $::env(FIN_DEFS) ne ""} {
    set_property verilog_define $::env(FIN_DEFS) [get_filesets sim_1]
}
set_property -name {xsim.simulate.runtime} -value {60ms} -objects [get_filesets sim_1]
launch_simulation
puts "==> sim finished"
