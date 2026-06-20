# FFT_IMPL 3 (IP-SSR) only: fft_b pre_0 s_axis regslice floorplan.
# Read from red_pitaya_vivado.tcl only when fft_impl==3, so the pre_0 hierarchy
# (fft_ip_ssr_bd/pre_0) is guaranteed present and no cell-existence guard is
# needed (read_xdc cannot evaluate a tcl `if` anyway).
#
# Without constraint: regslice_both_s_axis state_reg lands at X49Y41 while
# fifo_in rdp count_value sits at X51Y23-Y25 — a 16-row gap causes 0.836 ns
# routing delay on the state->rdp path (dominant pll_ser_clk violation).  Pin the
# regslice cells to the band adjacent to fifo_in.
create_pblock pb_saxisreg_fft_b
add_cells_to_pblock [get_pblocks pb_saxisreg_fft_b] [get_cells -hier -quiet -filter \
    {NAME =~ i_scope/fft_b/gen_fft_ip_ssr.fft_i/fft_ip_ssr_bd_i/pre_0/inst/regslice_both_s_axis_V_data_V_U/*}]
resize_pblock [get_pblocks pb_saxisreg_fft_b] -add {SLICE_X47Y21:SLICE_X54Y32}
