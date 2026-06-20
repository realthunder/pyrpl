# LogiCORE xfft ce_predicted early-CE budget.  Read for FFT_IMPL 1 / 3 / 4
# (LogiCORE xfft cores) from red_pitaya_vivado.tcl.  The HLS-based impls 2/5 have
# no ce_predicted_reg, which is why this is conditioned rather than global.
#
# ce_predicted_reg is asserted one clock cycle early by design.  Outgoing paths:
# fo~256 high-fanout net to DSP CEB2 inputs needs 2 cycles.  Incoming paths:
# combinational ce_predicted logic has long routes inside xfft.  The IP is
# designed for the CE to be valid 1 cycle early, so a 2-cycle budget for both the
# computation and propagation is within the IP's timing intent.
set_multicycle_path 2 -setup -from [get_cells -hierarchical -filter {NAME =~ *ce_predicted_reg*}]
set_multicycle_path 1 -hold  -from [get_cells -hierarchical -filter {NAME =~ *ce_predicted_reg*}]
set_multicycle_path 2 -setup -to   [get_cells -hierarchical -filter {NAME =~ *ce_predicted_reg*}]
set_multicycle_path 1 -hold  -to   [get_cells -hierarchical -filter {NAME =~ *ce_predicted_reg*}]
