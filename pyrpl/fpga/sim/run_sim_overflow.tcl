# Regression: fft_proc input-FIFO under-run hang + post-pad fix (tb_fin_overflow.sv).
#   default  -> RESULT ... POSTPAD=1 ... : OK   (shipped fix survives heavy overflow)
#   OVF_DEFS=NOPOSTPAD -> RESULT ... POSTPAD=0 ... : HUNG   (reproduces the bug)
# Other defines (space-separated in OVF_DEFS): QSZ=<n> WAIT1=<n> WAIT2=<n> BUBBLE=<n>.
set part xc7z020clg400-1
set origin [file normalize [file dirname [info script]]]
set root   [file normalize "$origin/.."]
cd $root
set tag ""
if {[info exists ::env(OVF_TAG)]} { set tag $::env(OVF_TAG) }
create_project -force tb_fin_overflow$tag $origin/proj_overflow$tag -part $part
set_property XPM_LIBRARIES {XPM_FIFO XPM_CDC} [current_project]
add_files -fileset sim_1 -norecurse $origin/tb_fin_overflow.sv
set_property top tb_fin_overflow [get_filesets sim_1]
if {[info exists ::env(OVF_DEFS)] && $::env(OVF_DEFS) ne ""} {
    set_property verilog_define $::env(OVF_DEFS) [get_filesets sim_1]
}
set_property -name {xsim.simulate.runtime} -value {6ms} -objects [get_filesets sim_1]
launch_simulation
puts "==> sim finished"
