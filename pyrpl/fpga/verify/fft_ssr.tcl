# csim of fft_ssr (FFT_IMPL=2, Vitis xf::dsp SSR FFT) — full spectrum.
# Run via verify/run.sh 2 [SSR].
proc getp {name def} {
    set e [string toupper $name]
    if {[info exists ::env($e)] && $::env($e) ne ""} { return $::env($e) }
    return $def
}
set part     [getp part     xc7z020clg400-1]
set fft_ssr  [getp fft_ssr  2]
set fft_nfft [getp fft_nfft 12]

# xf::dsp SSR FFT headers from the Vitis_Libraries checkout (separate repo, not
# shipped with Vivado/Vitis). Cloned to ../Vitis_Libraries; override with
# VITIS_LIBRARIES if it lives elsewhere.
set vitis_root [expr {[info exists ::env(VITIS_LIBRARIES)] ? $::env(VITIS_LIBRARIES) : "../Vitis_Libraries"}]
set vitis_inc  "$vitis_root/dsp/L1/include/hw/vitis_fft/fixed"
set defs "-DFFT_SSR=$fft_ssr -DFFT_NFFT=$fft_nfft"

file mkdir .hls; cd .hls
open_project -reset fft_ssr_verify
add_files     ../hls/fft_ssr.cpp        -cflags "-I$vitis_inc $defs"
add_files -tb ../verify/fft_ssr_tb.cpp  -cflags "-I../verify $defs"
set_top fft_ssr
open_solution -reset solution1
set_part $part
create_clock -period 8.0
csim_design
exit
