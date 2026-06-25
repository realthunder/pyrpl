set part xc7z020clg400-1
set origin [file normalize [file dirname [info script]]]
set root   [file normalize "$origin/.."]
cd $root
set tag ""
if {[info exists ::env(FIN_TAG)]} { set tag $::env(FIN_TAG) }
create_project -force tb_fin_steal$tag $origin/proj_fin$tag -part $part
set_property XPM_LIBRARIES {XPM_FIFO XPM_CDC} [current_project]
add_files -fileset sim_1 -norecurse $origin/tb_fin_steal.sv
set_property top tb_fin_steal [get_filesets sim_1]
if {[info exists ::env(FIN_DEFS)] && $::env(FIN_DEFS) ne ""} {
    set_property verilog_define $::env(FIN_DEFS) [get_filesets sim_1]
}
set_property -name {xsim.simulate.runtime} -value {4ms} -objects [get_filesets sim_1]
launch_simulation
puts "==> sim finished"
