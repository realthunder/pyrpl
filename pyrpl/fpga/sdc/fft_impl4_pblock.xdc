# FFT_IMPL 4 (native SSR xfft): ADC sum / sum1-replica closure constraints.
# Read from red_pitaya_vivado.tcl only when fft_impl==4.
#
# (No active constraints — placement pinning was tried and abandoned.)
#
# Failing path (pll_adc_clk WNS -0.293): a 13-level combinational sweep
#   i_dsp/sum1_reg[*] -> dac_saturate -> dac_dat feedback -> scope source mux
#   (scope1_o) -> i_scope/adc_a_sum CARRY4 chain -> adc_a_sum_reg[29].
#
# A pblock pinning the source replica + destination accumulator was tried and
# REGRESSED timing to WNS -1.230 (656 failing EPs) because:
#   1. i_dsp/sum1_reg[*]_replica_1 is created by phys_opt_design, so it does not
#      exist when this file is read (just after synth_design) — get_cells matched
#      nothing and only the destination accumulator got pinned.
#   2. Confining the 30-bit adc_a_sum CARRY4 chain into a tight band without its
#      source over-constrained placement.
# The robust fix is RTL: pipeline the scope source mux feeding adc_a_sum (breaks
# the 13-level combinational path), conditioned on FFT_IMPL==4.  Tracked separately.
