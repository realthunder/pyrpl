# Noise-floor / spectral dynamic-range csim for fft_hls_direct (FFT_IMPL=5).
# Injects ONE coherent tone and measures peak-vs-floor with a WIDE DSZ (48) so the
# output never saturates and masks the measurement — isolating the datapath precision
# (INTERNAL_W / scaling mode) from DSZ-output truncation. Reports SFDR and peak/RMS.
#
# Run (from the fpga/ dir), mirroring verify/run.sh's environment:
#   export XILINX_VITIS=/opt/Xilinx/2025.2/Vitis XILINX_VIVADO=/opt/Xilinx/2025.2/Vivado
#   source $XILINX_VITIS/settings64.sh
#   export LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/lib32:$LIBRARY_PATH
#   export CPATH=/usr/include/x86_64-linux-gnu:$CPATH
#   FFT_UNSCALED=1 FFT_INTERNAL_W=16 FFT_USE_APPROX=1 \
#     $XILINX_VITIS/bin/loader -exec vitis_hls -f verify/fft_hls_noise.tcl
# Sweep FFT_UNSCALED / FFT_INTERNAL_W / FFT_USE_APPROX to compare modes.
proc getp {name def} {
    set e [string toupper $name]
    if {[info exists ::env($e)] && $::env($e) ne ""} { return $::env($e) }
    return $def
}
set vfy_dir [file normalize [file dirname [info script]]]
set hls_dir [file normalize [file join $vfy_dir .. hls]]
set part            xc7z020clg400-1
set fft_ssr         [getp fft_ssr 2]
set fft_nfft        [getp fft_nfft 13]
set fft_internal_w  [getp fft_internal_w 16]
set fft_unscaled    [getp fft_unscaled 0]
set fft_use_approx  [getp fft_use_approx 1]
set fft_cordic_iter [getp fft_cordic_iter 10]
set dsz 48
set fft_size [expr {1 << $fft_nfft}]

# twiddle ROM must match FFT_SIZE/FFT_SSR (gen if missing/mismatched)
set lut $hls_dir/fft_ip_ssr_twiddle.hpp
set need [expr {$fft_ssr >= 2}]
if {$need && [file exists $lut]} {
    set fh [open $lut r]; set h [read $fh 256]; close $fh
    if {[string match "*FFT_SIZE=$fft_size *FFT_SSR=$fft_ssr *" $h]} { set need 0 }
}
if {$need} {
    set o [pwd]; cd $hls_dir
    if {[catch {exec g++ -O2 -o gen_twiddle_lut_exe gen_twiddle_lut.cpp}] || \
        [catch {exec ./gen_twiddle_lut_exe $fft_size $fft_ssr 18} m]} {
        exec python3 gen_twiddle_lut.py $fft_size $fft_ssr 18
    }
    cd $o
}

set defs "-DFFT_SSR=$fft_ssr -DFFT_NFFT=$fft_nfft -DASZ=14 -DDSZ=$dsz -DINTERNAL_W=$fft_internal_w -DCORDIC_ITER=$fft_cordic_iter"
if {$fft_use_approx} { append defs " -DUSE_APPROXIMATION" }
if {$fft_unscaled}   { append defs " -DFFT_UNSCALED" }
puts "NOISE-CFG: internal_w=$fft_internal_w unscaled=$fft_unscaled approx=$fft_use_approx nfft=$fft_nfft ssr=$fft_ssr"

file mkdir .hls; cd .hls
open_project -reset fft_hls_noise
add_files     $hls_dir/fft_hls_direct.cpp     -cflags "-I$hls_dir $defs"
add_files -tb $vfy_dir/fft_hls_noise_tb.cpp   -cflags "-I$vfy_dir $defs"
set_top fft_hls_direct
open_solution -reset solution1
set_part $part
create_clock -period 8.0
csim_design
exit
