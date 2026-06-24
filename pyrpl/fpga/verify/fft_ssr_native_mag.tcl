# Magnitude csim of fft_native_mag (FFT_IMPL=4, native-SSR xfft back-end).
# Checks slot extraction / alpha-max-beta-min magnitude / DSZ saturation against a
# C reference. The xfft ordering is validated on the DIF path. Run via verify/run.sh 4.
proc getp {name def} {
    set e [string toupper $name]
    if {[info exists ::env($e)] && $::env($e) ne ""} { return $::env($e) }
    return $def
}
set part     [getp part     xc7z020clg400-1]
set fft_ssr  [getp fft_ssr  4]
set fft_nfft [getp fft_nfft 11]
set fft_scaled    [getp fft_scaled 2]
set dsz           [getp dsz [expr {$fft_scaled == 1 ? 16 : ($fft_scaled == 2 ? 20 : 28)}]]
set fft_use_approx [getp fft_use_approx 1]
set approx_flag   [expr {$fft_use_approx ? "-DUSE_APPROXIMATION" : ""}]
set scaled_flag   [expr {$fft_scaled == 1 ? "-DFFT_SCALED=1" : ""}]

set defs "-DFFT_SSR=$fft_ssr -DFFT_NFFT=$fft_nfft -DASZ=14 -DINTERNAL_W=16 \
          -DDSZ=$dsz $approx_flag $scaled_flag"
file mkdir .hls; cd .hls
open_project -reset fft_native_mag_verify
add_files     ../hls/fft_ssr_native.cpp            -cflags "-I../hls $defs"
add_files -tb ../verify/fft_ssr_native_mag_tb.cpp  -cflags "-I../verify $defs"
set_top fft_native_mag
open_solution -reset solution1
set_part $part
create_clock -period 5.6
csim_design
exit
