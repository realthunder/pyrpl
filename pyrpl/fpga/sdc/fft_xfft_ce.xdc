# LogiCORE xfft ce_predicted early-CE budget.  Read for FFT_IMPL 1 / 3 / 4 / 5
# from red_pitaya_vivado.tcl -- every impl whose netlist instantiates the xfft
# LogiCORE core.  IMPL=5 (direct hls::fft) DOES contain ce_predicted_reg: hls::fft
# wraps the xfft LogiCORE subcore (gen_ce_non_real_time.ce_predicted_reg).  Only
# IMPL=2 (Vitis xf::dsp SSR FFT) has no ce_predicted_reg, so this is conditioned
# rather than global.  NOTE: omitting it for IMPL=5 forces the high-fanout CE net
# to close single-cycle -- the tool over-replicates it (8 -> 87 copies) and the
# placement basin shift regressed pll_adc_clk from -0.002 to -0.193.
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
