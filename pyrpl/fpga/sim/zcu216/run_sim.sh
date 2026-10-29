#!/bin/bash
# xsim checks of the ZCU216 FFT at N = 256 (eight 32-point xfft cores), the same
# RTL as the 256k build:
#   tb_dit8  continuous source -> zcu216_fft_dit8 -> _mag -> _reorder; every bin
#            (complex, and the natural-order magnitudes) compared with numpy
#            (cmp.py); TB_DW=17 forces the 2-bit cut that the 256k build makes
#   tb_test  zcu216_fft_test through the register bus: tones, peaks, window
#   tb_reorder  zcu216_fft_reorder alone, with tagged bins
#   tb_bridge   zcu216_axil_bridge against an AXI4-Lite memory with random delays
# Usage: sim/zcu216/run_sim.sh [tb_dit8|tb_test|tb_reorder|tb_bridge|all] [TB_DW]   (from fpga/)
set -e
here=$(cd "$(dirname "$0")" && pwd)
fpga=$(cd "$here/../.." && pwd)
work="$here/work"
which="${1:-all}"
dw="${2:-27}"
mkdir -p "$work"
echo "localparam string SIMDIR = \"$work\";" > "$work/simdir.svh"
source /tools/Xilinx/2025.2/Vivado/settings64.sh
cat > "$work/sim.tcl" <<TCL
source $fpga/zcu216/gen_tables.tcl
zcu216_gen_tables $work/mem 8
if {[file exists $work/p/p.xpr]} {
  open_project $work/p/p.xpr
} else {
  create_project -force p $work/p -part xczu49dr-ffvf1760-2-e
  create_ip -name xfft -vendor xilinx.com -library ip -version 9.1 -module_name xfft_sub
  set_property -dict [list CONFIG.aresetn {true} CONFIG.transform_length {32} \
    CONFIG.data_format {fixed_point} CONFIG.implementation_options {pipelined_streaming_io} \
    CONFIG.input_width {16} CONFIG.phase_factor_width {18} CONFIG.rounding_modes {convergent_rounding} \
    CONFIG.run_time_configurable_transform_length {false} CONFIG.scaling_options {unscaled} \
    CONFIG.target_clock_frequency {312.5} CONFIG.throttle_scheme {realtime}] [get_ips xfft_sub]
  set_property CONFIG.output_ordering {bit_reversed_order} [get_ips xfft_sub]
  set_property CONFIG.xk_index {true} [get_ips xfft_sub]
  generate_target {simulation} [get_ips xfft_sub]
  add_files -fileset sim_1 -norecurse [list $here/tb_dit8.sv $here/tb_test.sv $here/tb_reorder.sv $here/tb_bridge.sv]
  set_property include_dirs [list $work] [get_filesets sim_1]
  set_property -name {xsim.simulate.runtime} -value {all} -objects [get_filesets sim_1]
}
# (also on an existing project: files added since it was made)
foreach f {zcu216_fft_dit8.sv zcu216_fft_mag.sv zcu216_fft_reorder.sv zcu216_tone_gen.sv zcu216_fft_spec.sv zcu216_fft_test.sv zcu216_axil_bridge.sv zcu216_cfar.sv zcu216_snap.sv zcu216_points.sv zcu216_scope.sv zcu216_scope_tavg.sv} {
  if {![llength [get_files -quiet $fpga/rtl/\$f]]} { add_files -norecurse $fpga/rtl/\$f }
}
set_property -name {xsim.compile.xvlog.more_options} -value {-d TB_DW=$dw} -objects [get_filesets sim_1]
foreach top {$([[ $which == all ]] && echo tb_dit8 tb_test tb_reorder tb_bridge || echo $which)} {
  set_property top \$top [get_filesets sim_1]
  update_compile_order -fileset sim_1
  launch_simulation -simset sim_1 -mode behavioral
  close_sim
}
TCL
cd "$work"
vivado -mode batch -nojournal -nolog -source sim.tcl > sim.log 2>&1 || { tail -30 sim.log; exit 1; }
grep -E "^(ID|STATUS|CH[0-9]|WIN|LOOP|RAW|TB_DONE|EVENT|RO |BRIDGE|CFAR|SNAP|POINTS|SCOPE|ERR)" sim.log || true
if [[ $which == all || $which == tb_dit8 ]]; then
  # the core outputs are cut by max(0, IN_W + SUB - DW) = max(0, 19 - DW) bits
  python3 -I "$here/cmp.py" "$work" 256 $(( dw < 19 ? 1 << (19 - dw) : 1 ))
fi
